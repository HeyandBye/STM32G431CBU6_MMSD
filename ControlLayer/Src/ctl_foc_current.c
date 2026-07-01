/**
******************************************************************************
* @file     ctl_foc_current.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 电流闭环实现 —— Id=0 策略 + 编码器校准 + PI 控制
*
* @note     归一化: 整个 FOC 链路使用物理单位（A / V / rad），
*           仅 SVPWM 输出 duty ∈ [0,1]。详见 ctl_math.c / ctl_pid.c。
*
*           目标电机: iPower GM3506（11 对极, 5.6Ω, 1A@12V）
*           编码器: AS5048A 14-bit 磁编码器
*
*           调用链:
*           ADC1 转换完成 (20kHz) → HAL_ADC_ConvCpltCallback
*           → FOC_Current_Run_Callback → FOC_Current_Run
*           → 读编码器 → FOC_Current_Step → 写 PWM → 故障检测
******************************************************************************
*/

#include "ctl_foc_current.h"
#include "ctl_math.h"
#include "ctl_pid.h"
#include "drv_adc_sampling.h"
#include "drv_spi_as5048a.h"
#include "drv_tim_pwm.h"
#include "arm_math.h"
#include "stm32g4xx_hal.h"

/** @brief 微分滤波默认系数 */
#define DEFAULT_DERIV_ALPHA  0.1f

/*==========================================================================*/
/* ADC 回调包装                                                               */
/*==========================================================================*/

void FOC_Current_Run_Callback(void)
{
    FOC_Current_Run(&g_foc);
}

/*==========================================================================*/
/* FOC_Current_TestRamp —— 测试斜坡                                          */
/*==========================================================================*/

/**
 * @brief   电流闭环测试斜坡（主循环调用, 根据运行时间自动调整 Id/Iq 给定）
 * @param   foc      控制器指针
 * @param   tick_ms  系统运行时间 (ms)
 * @note    0~1s: Id=0.5A 锁转子 → 1~1.5s: Id→0, Iq 0→1.0A → 1.5s后: Id=0, Iq=1.0 持续。
 */
void FOC_Current_TestRamp(FOC_t *foc, uint32_t tick_ms)
{
    if (foc == NULL)
    {
        return;
    }

    if (tick_ms < 1000U)
    {
        /* 0~1s: Id=0.5A 锁 */
    }
    else if (tick_ms < 1500U)
    {
        float t = (float)(tick_ms - 1000U) / 500.0f;
        FOC_Current_SetRef(foc, 0.5f * (1.0f - t), 1.0f * t);
    }
    else
    {
        FOC_Current_SetRef(foc, 0.0f, 1.0f);
    }
}

/*==========================================================================*/
/* FOC_Current_Init —— 对标 FOC_OpenLoop_Init                                 */
/*==========================================================================*/

/**
 * @brief   电流闭环初始化（对标 FOC_OpenLoop_Init, 一步完成核心配置）
 * @param   foc  FOC 控制器指针
 * @param   cfg  初始化配置（极对数、额定电流/限幅、母线电压、PI 增益）
 * @note    内部依次调用:
 *          ① FOC_Init: 清零 FOC_t（含 PID 内部状态）
 *          ② FOC_SetMotorParams: 写入电机参数, 置 READY
 *          ③ FOC_Current_ConfigPID: 配置 PI, 积分限幅覆盖为 ±Vbus
 *          编码器偏置初始为 0, 需后续 CalibrateEncoder 校准。
 */
void FOC_Current_Init(FOC_t *foc, const FOC_Current_Config_t *cfg)
{
    if (foc == NULL || cfg == NULL)
    {
        return;
    }

    /* ① 清零 */
    FOC_Init(foc);

    /* ② 电机参数（enc_offset 初始为 0, 由 CalibrateEncoder 校准） */
    FOC_SetMotorParams(foc,
                       cfg->pole_pairs,
                       0,
                       cfg->rated_current,
                       cfg->current_limit,
                       cfg->vbus_nominal);

    /* ③ PI 参数 */
    FOC_Current_ConfigPID(foc,
                          cfg->kp_id, cfg->ki_id,
                          cfg->kp_iq, cfg->ki_iq);
}

/**
 * @brief   配置 Id/Iq PI 参数（Kd=0, Kr=1.0）
 * @param   foc     FOC 控制器指针
 * @param   kp_id   d 轴比例增益（V/A, 推荐 0.5~2.0）
 * @param   ki_id   d 轴积分增益（推荐 0.01~0.05）
 * @param   kp_iq   q 轴比例增益（V/A, 推荐 0.5~2.0）
 * @param   ki_iq   q 轴积分增益（推荐 0.01~0.05）
 *
 * @note    Ki = Kp × Ts / Ti, Ts=50µs (20kHz)。GM3506 R=5.6Ω: kp=2.0, ki=0.05。
 */
void FOC_Current_ConfigPID(FOC_t *foc,
                           float kp_id, float ki_id,
                           float kp_iq, float ki_iq)
{
    float vbus_limit;

    if (foc == NULL)
    {
        return;
    }

    vbus_limit = foc->motor.vbus_nominal;
    if (vbus_limit < 0.1f)
    {
        vbus_limit = 12.0f;
    }

    CTL_PID_Init(&foc->pid_id, kp_id, ki_id, 0.0f, 1.0f,
                 DEFAULT_DERIV_ALPHA, -vbus_limit, vbus_limit);
    CTL_PID_Init(&foc->pid_iq, kp_iq, ki_iq, 0.0f, 1.0f,
                 DEFAULT_DERIV_ALPHA, -vbus_limit, vbus_limit);

    foc->pid_id.integral_max =  vbus_limit;
    foc->pid_id.integral_min = -vbus_limit;
    foc->pid_iq.integral_max =  vbus_limit;
    foc->pid_iq.integral_min = -vbus_limit;
}

/**
 * @brief   手动设置编码器偏置（跳过自动校准）
 * @param   foc     FOC 控制器指针
 * @param   offset  编码器零点偏置 (0~16383)
 */
void FOC_Current_SetEncoderOffset(FOC_t *foc, uint16_t offset)
{
    if (foc == NULL)
    {
        return;
    }
    foc->motor.enc_offset = offset;
}

/**
 * @brief   编码器电角度零点自动校准（阻塞 ~500ms）
 * @param   foc             FOC 控制器指针
 * @param   read_angle_fn   编码器读取函数
 * @param   set_duty_fn     PWM 占空比设置函数
 *
 * @note    注入固定 θ=0 电压矢量（duty_offset=0.40, Vα=4.8V, I≈0.86A）。
 *          ⚠️ 阻塞调用, PWM 需已使能。
 */
void FOC_Current_CalibrateEncoder(FOC_t      *foc,
                                  uint16_t (*read_angle_fn)(void),
                                  void     (*set_duty_fn)(float, float, float))
{
    uint32_t tick_start;
    uint16_t raw;

    if (foc == NULL)
    {
        return;
    }
    if (read_angle_fn == NULL || set_duty_fn == NULL)
    {
        return;
    }

    /* θ=0 矢量: duty_offset=0.40 → da=0.90, db=0.30, dc=0.30 */
    set_duty_fn(0.90f, 0.30f, 0.30f);

    tick_start = HAL_GetTick();
    while ((HAL_GetTick() - tick_start) < 500U)
    {
        HAL_Delay(1);
    }

    raw = read_angle_fn();
    foc->motor.enc_offset = raw;
}

/**
 * @brief   启动电流闭环（READY → RUNNING）
 * @param   foc  FOC 控制器指针
 * @note    重置 PID, 同步当前电流给定到 setpoint。
 */
void FOC_Current_Start(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }
    if (foc->state != FOC_STATE_READY)
    {
        return;
    }

    CTL_PID_Reset(&foc->pid_id);
    CTL_PID_Reset(&foc->pid_iq);

    CTL_PID_SetSetpoint(&foc->pid_id, foc->id_ref);
    CTL_PID_SetSetpoint(&foc->pid_iq, foc->iq_ref);

    foc->state      = FOC_STATE_RUNNING;
    foc->loop_count = 0U;
    foc->fault_code = FOC_FAULT_NONE;
    foc->speed_rpm  = 0.0f;
    foc->theta_prev = 0.0f;
}

/**
 * @brief   停止电流闭环（RUNNING → READY）
 * @param   foc  FOC 控制器指针
 * @note    占空比归 50%（绕组电压为零）, 电机自由滑行。
 */
void FOC_Current_Stop(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    foc->state  = FOC_STATE_READY;
    foc->duty_a = 0.5f;
    foc->duty_b = 0.5f;
    foc->duty_c = 0.5f;
    drv_tim_pwm_set_duty_f(0.5f, 0.5f, 0.5f);

    CTL_PID_Reset(&foc->pid_id);
    CTL_PID_Reset(&foc->pid_iq);
}

/**
 * @brief   设置电流给定（Id/Iq 目标值）
 * @param   foc     FOC 控制器指针
 * @param   id_ref  d 轴电流给定 (A), Id=0 策略固定为 0
 * @param   iq_ref  q 轴电流给定 (A), 正值=正转矩
 * @note    输入钳位到 [-current_limit, +current_limit]。
 *          PID setpoint 同步写入, 下一周期自动生效。ISR 中安全调用。
 */
void FOC_Current_SetRef(FOC_t *foc, float id_ref, float iq_ref)
{
    float limit;

    if (foc == NULL)
    {
        return;
    }

    limit = foc->motor.current_limit;

    if (id_ref >  limit)
    {
        id_ref =  limit;
    }
    if (id_ref < -limit)
    {
        id_ref = -limit;
    }
    if (iq_ref >  limit)
    {
        iq_ref =  limit;
    }
    if (iq_ref < -limit)
    {
        iq_ref = -limit;
    }

    foc->id_ref = id_ref;
    foc->iq_ref = iq_ref;

    CTL_PID_SetSetpoint(&foc->pid_id, id_ref);
    CTL_PID_SetSetpoint(&foc->pid_iq, iq_ref);
}

/**
 * @brief   FOC 电流环单步算法（20kHz ISR 内调用）
 * @param   foc        FOC 控制器指针
 * @param   raw_angle  编码器原始值 (0~16383)
 * @param   ia         A 相电流 (A)
 * @param   ib         B 相电流 (A)
 * @param   vbus       母线电压 (V)
 *
 * @note    7 步: raw→θ → Clarke → Park → PI → InvPark → SVPWM → 诊断。
 *          耗时 ≈ 4.8µs @ 170MHz。
 */
void FOC_Current_Step(FOC_t   *foc,
                      uint16_t raw_angle,
                      float    ia,
                      float    ib,
                      float    vbus)
{
    if (foc == NULL)
    {
        return;
    }

    /* 1. 电角度 + sin/cos（arm_sin_cos_f32 一次查表同时得两者） */
    foc->raw_angle  = raw_angle;
    foc->theta_elec = CTL_RawToElectrical(raw_angle,
                                          foc->motor.pole_pairs,
                                          foc->motor.enc_offset);
    arm_sin_cos_f32(foc->theta_elec * 57.295779513f,
                    &foc->sin_theta, &foc->cos_theta);

    /* 2. Clarke */
    CTL_Clarke(ia, ib, &foc->i_alpha, &foc->i_beta);

    /* 3. Park */
    CTL_Park(foc->i_alpha, foc->i_beta, foc->theta_elec,
             &foc->id, &foc->iq);

    /* 4. PI (Kd=0) */
    foc->vd = CTL_PID_Update(&foc->pid_id, foc->id);
    foc->vq = CTL_PID_Update(&foc->pid_iq, foc->iq);

    /* 5. InvPark */
    CTL_InvPark(foc->vd, foc->vq, foc->theta_elec,
                &foc->v_alpha, &foc->v_beta);

    /* 6. SVPWM */
    CTL_SVPWM(foc->v_alpha, foc->v_beta, vbus,
              &foc->duty_a, &foc->duty_b, &foc->duty_c);

    /* 7. 诊断 */
    foc->ia    = ia;
    foc->ib    = ib;
    foc->vbus  = vbus;
    foc->loop_count = foc->loop_count + 1U;
}

/**
 * @brief   FOC 电流闭环完整周期（ADC 回调中调用）
 * @param   foc  FOC 控制器指针
 * @note    封装: 状态检查 → 读传感器 → Step → 写 PWM → 故障检测（分级消抖）。
 *          总耗时 ≈ 16µs（32% PWM 周期）。
 *
 *          分级故障消抖:
 *          - 硬故障（过流/过压/欠压）: 连续 5 次 → 立即停机（250µs）
 *          - 软故障（编码器通信）:   连续 100 次 → 停机（5ms）
 *          杜邦线接触不良导致偶发 SPI 失败时, 驱动层已做 3 次重试;
 *          仍未成功则用上一有效角度继续运行, 不触发误停机。
 */
#define FAULT_DEBOUNCE_HARD  5U    /**< 硬故障消抖次数（5 × 50µs = 250µs） */
#define FAULT_DEBOUNCE_ENC   100U  /**< 编码器故障消抖次数（100 × 50µs = 5ms） */

void FOC_Current_Run(FOC_t *foc)
{
    uint16_t  raw_angle;
    float     ia, ib, vbus;
    uint32_t  fault;
    uint32_t  threshold;

    if (foc == NULL)
    {
        return;
    }
    if (foc->state != FOC_STATE_RUNNING)
    {
        return;
    }

    /* 读传感器 */
    raw_angle = drv_as5048a_read_angle();
    ia        = g_curr_ia;
    ib        = g_curr_ib;
    vbus      = g_bus_vol;

    /* 编码器读数兜底: 失败时用上一有效角度, 避免传 0 导致磁场角度错误 */
    if (raw_angle == 0U && drv_as5048a_get_error() != 0U)
    {
        raw_angle = foc->raw_angle;
    }

    /* FOC 算法 */
    FOC_Current_Step(foc, raw_angle, ia, ib, vbus);

    /* 写 PWM */
    drv_tim_pwm_set_duty_f(foc->duty_a, foc->duty_b, foc->duty_c);

    /* 故障检测（分级消抖: 硬故障 5 次, 编码器 100 次） */
    fault = FOC_FAULT_NONE;
    if (foc->loop_count > 2000U)
    {
        fault = FOC_CheckFault(foc, 20.0f, 2.5f,
                               drv_as5048a_get_error() == 0);
    }

    if (fault != FOC_FAULT_NONE)
    {
        /* 累计连续故障计数（按故障类型区分） */
        if (fault == foc->fault_code)
        {
            foc->fault_consec = foc->fault_consec + 1U;
        }
        else
        {
            foc->fault_code  = fault;
            foc->fault_consec = 1U;
        }

        /* 按故障类型选阈值: 编码器=100 次(5ms), 硬故障=5 次(250µs) */
        if (fault == FOC_FAULT_ENCODER)
        {
            threshold = FAULT_DEBOUNCE_ENC;
        }
        else
        {
            threshold = FAULT_DEBOUNCE_HARD;
        }

        if (foc->fault_consec >= threshold)
        {
            FOC_EmergencyStop(foc);
            drv_tim_pwm_moe_off();
        }
    }
    else
    {
        /* 无故障 → 清零连续计数器 */
        foc->fault_consec = 0U;
    }
}
