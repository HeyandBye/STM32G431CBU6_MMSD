/**
******************************************************************************
* @file     ctl_foc_openloop.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 开环控制实现 —— 虚拟旋转磁场 + SVPWM
*
* @note     开环 FOC 不依赖编码器角度做闭环控制。
*           虚拟电角度每步累加 Δθ = 2π × f × Ts（Ts=50µs），
*           经 InvPark → SVPWM 产生三相 PWM 占空比。
*           编码器角度仅用于串口监控, 不参与控制。
*
*           频率上限: Nyquist f_max < 10kHz, 实用 < 1kHz（100 步/电周期）
*           GM3506: 25Hz ≈ 500 步/电周期, 精度足够。
*
*           虚拟角度回绕: 防止 float 在大数时精度损失
*           （float 有效 ~7 位, 角度 > 10^6 后增量丢失精度）。
******************************************************************************
*/

#include "ctl_foc_openloop.h"
#include "ctl_foc_core.h"         /* g_fol */
#include "ctl_math.h"              /* CTL_InvPark, CTL_SVPWM */
#include "drv_adc_sampling.h"      /* g_bus_vol */
#include "drv_spi_as5048a.h"       /* drv_as5048a_read_angle */
#include "drv_tim_pwm.h"           /* drv_tim_pwm_set_duty_f */
#include <math.h>

/** @brief GM3506 极对数 */
#define OL_POLE_PAIRS   11U

/** @brief 2π */
#define OL_2PI          6.283185307f

/** @brief PWM 周期 = 1/20000 = 50µs */
#define OL_TS           0.00005f

/** @brief 角度回绕阈值 = 2π × 极对数 */
#define OL_ANGLE_WRAP   (OL_2PI * (float)OL_POLE_PAIRS)

/*==========================================================================*/
/* ADC 回调包装                                                               */
/*==========================================================================*/

void FOC_OpenLoop_Run_Callback(void)
{
    FOC_OpenLoop_Run(&g_fol);
}

/*==========================================================================*/
/* FOC_OpenLoop_TestRamp —— 测试斜坡                                          */
/*==========================================================================*/

/**
 * @brief   开环测试斜坡（主循环调用, 根据运行时间自动调整频率和幅值）
 * @param   fol      控制器指针
 * @param   tick_ms  系统运行时间 (ms)
 * @note    0~1s: 5Hz/10% → 1~6s: 5→50Hz + 10%→35% → 6s后: 50Hz/35% 稳定。
 */
void FOC_OpenLoop_TestRamp(FOC_OpenLoop_t *fol, uint32_t tick_ms)
{
    if (fol == NULL)
    {
        return;
    }

    if (tick_ms < 1000U)
    {
        /* 0~1s: 保持初始值 5Hz 10% */
    }
    else if (tick_ms < 6000U)
    {
        float t = (float)(tick_ms - 1000U) / 5000.0f;
        FOC_OpenLoop_SetFreq(fol, 5.0f + 45.0f * t);
        FOC_OpenLoop_SetAmplitude(fol, 0.10f + 0.25f * t);
    }
    else
    {
        FOC_OpenLoop_SetFreq(fol, 50.0f);
        FOC_OpenLoop_SetAmplitude(fol, 0.35f);
    }
}

/*==========================================================================*/
/* FOC_OpenLoop_Init                                                        */
/*==========================================================================*/

/**
 * @brief   开环 FOC 初始化
 * @param   fol        控制器指针
 * @param   freq_hz    初始电频率 (Hz), 推荐 2~5Hz
 * @param   amplitude  初始电压幅值比 [0, 1], 推荐 0.10~0.20
 * @note    虚拟角度清零, 设置初始频率和幅值。
 */
void FOC_OpenLoop_Init(FOC_OpenLoop_t *fol, float freq_hz, float amplitude)
{
    if (fol == NULL)
    {
        return;
    }

    fol->virtual_angle = 0.0f;
    fol->elec_freq_hz  = freq_hz;
    fol->amplitude     = amplitude;
    fol->duty_a        = 0.5f;
    fol->duty_b        = 0.5f;
    fol->duty_c        = 0.5f;
    fol->enc_raw       = 0U;
    fol->step_count    = 0U;
}

/*==========================================================================*/
/* FOC_OpenLoop_SetFreq                                                     */
/*==========================================================================*/

/**
 * @brief   设置电频率 (Hz)
 * @param   fol      控制器指针
 * @param   freq_hz  电频率 (Hz)
 */
void FOC_OpenLoop_SetFreq(FOC_OpenLoop_t *fol, float freq_hz)
{
    if (fol == NULL)
    {
        return;
    }
    fol->elec_freq_hz = freq_hz;
}

/*==========================================================================*/
/* FOC_OpenLoop_SetAmplitude                                                */
/*==========================================================================*/

/**
 * @brief   设置电压幅值比
 * @param   fol  控制器指针
 * @param   amp  电压幅值比 [0, 1]
 */
void FOC_OpenLoop_SetAmplitude(FOC_OpenLoop_t *fol, float amp)
{
    if (fol == NULL)
    {
        return;
    }
    if (amp < 0.0f)
    {
        amp = 0.0f;
    }
    if (amp > 1.0f)
    {
        amp = 1.0f;
    }
    fol->amplitude = amp;
}

/*==========================================================================*/
/* FOC_OpenLoop_Step                                                        */
/*==========================================================================*/

/**
 * @brief   开环 FOC 单步更新（20kHz ISR 内调用）
 * @param   fol      控制器指针
 * @param   vbus     当前母线电压 (V)
 * @param   enc_raw  编码器原始值 (0~16383), 只读取不参与控制
 *
 * @note    算法（4 步）:
 *          Step 1 - 虚拟角度累加: virtual_angle += 2π × freq_hz × 50µs,
 *            角度回绕到 [0, 2π×P) 防 float 精度损失
 *          Step 2 - 电压给定: Vd=0（MTPA 等效）, Vq = amplitude × Vbus
 *          Step 3 - 逆 Park: Vα=-Vq×sin(θ), Vβ=Vq×cos(θ)（Vd=0 简化）
 *          Step 4 - SVPWM: CTL_SVPWM(Vα,Vβ,Vbus) → duty_a/b/c
 */
void FOC_OpenLoop_Step(FOC_OpenLoop_t *fol, float vbus, uint16_t enc_raw)
{
    float delta_angle;
    float vd, vq;
    float v_alpha, v_beta;

    if (fol == NULL)
    {
        return;
    }

    /* Step 1: 虚拟角度累加 + 回绕 */
    delta_angle = OL_2PI * fol->elec_freq_hz * OL_TS;
    fol->virtual_angle = fol->virtual_angle + delta_angle;

    if (fol->virtual_angle >= OL_ANGLE_WRAP)
    {
        fol->virtual_angle = fol->virtual_angle - OL_ANGLE_WRAP;
    }

    /* Step 2: 电压给定 — Vd=0, Vq = amplitude × Vbus */
    vd = 0.0f;
    vq = fol->amplitude * vbus;
    if (vbus < 0.1f)
    {
        vbus = 12.0f;
    }

    /* Step 3: 逆 Park — Vd,Vq → Vα,Vβ */
    CTL_InvPark(vd, vq, fol->virtual_angle, &v_alpha, &v_beta);

    /* Step 4: SVPWM — Vα,Vβ → duty [0,1] */
    CTL_SVPWM(v_alpha, v_beta, vbus, &fol->duty_a, &fol->duty_b, &fol->duty_c);

    /* 记录编码器（仅监控用） */
    fol->enc_raw    = enc_raw;
    fol->step_count = fol->step_count + 1U;
}

/*==========================================================================*/
/* FOC_OpenLoop_Run —— 完整运行周期，供 HAL 回调调用                           */
/*==========================================================================*/

/**
 * @brief   开环 FOC 完整运行周期
 * @param   fol  控制器指针
 * @note    封装: 读编码器 → 读母线电压 → Step → 写 PWM。
 *          ⚠️ 中断上下文, 耗时 ≈ 12.5µs（25% PWM 周期）。
 */
void FOC_OpenLoop_Run(FOC_OpenLoop_t *fol)
{
    uint16_t raw_angle;
    float    vbus;

    if (fol == NULL)
    {
        return;
    }

    /* 读传感器 */
    raw_angle = drv_as5048a_read_angle();
    vbus      = g_bus_vol;

    /* 开环 Step: 虚拟角度累加 + InvPark + SVPWM */
    FOC_OpenLoop_Step(fol, vbus, raw_angle);

    /* 更新 PWM 占空比 */
    drv_tim_pwm_set_duty_f(fol->duty_a, fol->duty_b, fol->duty_c);
}

/*==========================================================================*/
/* FOC_OpenLoop_Start / Stop —— 对标 FOC_Current_Start / Stop                 */
/*==========================================================================*/

/**
 * @brief   启动开环控制（对标 FOC_Current_Start）
 * @param   fol  开环控制器指针
 * @note    清零虚拟角度和步数计数, 使电机从电角度 0 开始运行。
 *          不影响当前频率/幅值设置。
 */
void FOC_OpenLoop_Start(FOC_OpenLoop_t *fol)
{
    if (fol == NULL)
    {
        return;
    }
    fol->virtual_angle = 0.0f;
    fol->step_count    = 0U;
}

/**
 * @brief   停止开环控制（对标 FOC_Current_Stop）
 * @param   fol  开环控制器指针
 * @note    三相占空比全部归 50%（上下管对称导通, 绕组平均电压为零）,
 *          电机自由滑行。PWM 保持输出, 可随时 Start 恢复。
 */
void FOC_OpenLoop_Stop(FOC_OpenLoop_t *fol)
{
    if (fol == NULL)
    {
        return;
    }
    fol->duty_a = 0.5f;
    fol->duty_b = 0.5f;
    fol->duty_c = 0.5f;
    drv_tim_pwm_set_duty_f(0.5f, 0.5f, 0.5f);
}
