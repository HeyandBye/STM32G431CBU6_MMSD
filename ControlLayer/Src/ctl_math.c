/**
******************************************************************************
* @file     ctl_math.c
* @author   lidongyang
* @version  0.0.3
* @date     23-June-2026
* @brief    FOC 坐标变换 & SVPWM 实现
*
* @note     归一化策略:
*           所有函数使用物理单位（A / V / rad），不归一化到 [-1,1]。
*           理由:
*           1. FOC 物理量固定（电流、电压），不像通用 PID 需支持多种量纲
*           2. 物理单位调试直读含义（"Id=0.3A" vs "Id=0.15归一化"）
*           3. 唯一归一化处: SVPWM 输出 duty ∈ [0,1]（PWM 硬件接口硬性要求）
*           4. Cortex-M4 FPU 单周期 float, 无额外性能开销
*
*           性能（Cortex-M4 @ 170MHz, CMSIS-DSP 加速）:
*           - arm_sin_cos_f32 × 2: ~0.3µs（查表+插值, 比 sinf/cosf 快 ~5×）
*           - 坐标变换 × 3:        ~1.0µs
*           - SVPWM:               ~1.5µs
*           - 合计 ~2.8µs（50µs PWM 周期 < 6%）
******************************************************************************
*/

#include "ctl_math.h"
#include "arm_math.h"

/*==========================================================================*/
/* 常量                                                                      */
/*==========================================================================*/

/** @brief 1/√3，Clarke 变换 Iβ 计算用 */
#define ONE_OVER_SQRT3  0.577350269f

/** @brief √3/2，SVPWM 反 Clarke 变换用 */
#define SQRT3_OVER_2    0.866025404f

/** @brief 弧度/编码器 LSB = 2π/16384，角度转换系数 */
#define ENC_RAD_PER_LSB 0.000383495f

/** @brief 弧度→度: 180/π，供 arm_sin_cos_f32（角度输入）转换用 */
#define RAD_TO_DEG      57.295779513f

/*==========================================================================*/
/* CTL_Clarke                                                               */
/*==========================================================================*/

/**
 * @brief   Clarke 变换: 三相静止 → 两相静止（幅值不变形式）
 * @param   ia       A 相电流 (A)
 * @param   ib       B 相电流 (A)
 * @param   i_alpha  输出: α 轴电流 (A)
 * @param   i_beta   输出: β 轴电流 (A)
 *
 * @note    归一化: 否。I/O 均为物理电流 (A)。
 *          幅值不变: Iα=Ia, Iβ=(Ia+2Ib)/√3, |Iαβ|_peak=|Iabc|_peak。
 */
void CTL_Clarke(float ia, float ib,
                float *i_alpha, float *i_beta)
{
    /* α 轴 = A 相电流（幅值不变形式的最简情况） */
    /* β 轴 = (Ia+2*Ib)/√3，预计算 ONE_OVER_SQRT3 避免运行时除法 */
    *i_alpha = ia;
    *i_beta  = (ia + 2.0f * ib) * ONE_OVER_SQRT3;
}

/*==========================================================================*/
/* CTL_Park                                                                 */
/*==========================================================================*/

/**
 * @brief   Park 变换: 两相静止 (αβ) → 两相旋转 (dq)
 * @param   i_alpha  α 轴电流 (A)
 * @param   i_beta   β 轴电流 (A)
 * @param   theta    电角度 (rad)
 * @param   id       输出: d 轴电流 (A)
 * @param   iq       输出: q 轴电流 (A)
 *
 * @note    归一化: 否。I/O 均为物理电流 (A)。
 *          Id=Iα*cos(θ)+Iβ*sin(θ), Iq=-Iα*sin(θ)+Iβ*cos(θ)。
 *          d 轴对齐转子 N 极, q 轴超前 90°。
 *          arm_sin_cos_f32 ≈ 0.15µs, 比 sinf+cosf 快 ~10×。
 */
void CTL_Park(float i_alpha, float i_beta, float theta,
              float *id, float *iq)
{
    float s, c;
    /* arm_sin_cos_f32 输入单位为度（非弧度），需转换；
       一次查表同时返回 sin 和 cos，比分别调用快一倍 */
    arm_sin_cos_f32(theta * RAD_TO_DEG, &s, &c);

    *id = i_alpha * c + i_beta * s;
    *iq = i_alpha * (-s) + i_beta * c;
}

/*==========================================================================*/
/* CTL_InvPark                                                              */
/*==========================================================================*/

/**
 * @brief   逆 Park 变换: 两相旋转 (dq) → 两相静止 (αβ)
 * @param   vd       d 轴电压 (V)
 * @param   vq       q 轴电压 (V)
 * @param   theta    电角度 (rad)
 * @param   v_alpha  输出: α 轴电压 (V)
 * @param   v_beta   输出: β 轴电压 (V)
 *
 * @note    归一化: 否。Vd,Vq (V) → Vα,Vβ (V)。
 *          Vα=Vd*cos(θ)-Vq*sin(θ), Vβ=Vd*sin(θ)+Vq*cos(θ)。
 *          不额外钳位——钳位由 PID 和 SVPWM 阶段完成。
 */
void CTL_InvPark(float vd, float vq, float theta,
                 float *v_alpha, float *v_beta)
{
    float s, c;
    /* arm_sin_cos_f32: 输入角度（度），一次查表同时得 sin+cos */
    arm_sin_cos_f32(theta * RAD_TO_DEG, &s, &c);

    *v_alpha = vd * c - vq * s;
    *v_beta  = vd * s + vq * c;
}

/*==========================================================================*/
/* CTL_SVPWM                                                                */
/*==========================================================================*/

/**
 * @brief   SVPWM 七段式调制: Vα,Vβ (V) → 三相占空比 [0,1]
 *
 * @note    归一化: 输入否（物理电压 V）, 输出是（duty ∈ [0,1]）。
 *          这是整个 FOC 链路中唯一做归一化的环节。
 *
 *          算法: min/max 共模注入法（等效七段式 SVPWM）:
 *          Step 1 - 反 Clarke: Va=Vα, Vb=-Vα/2+(√3/2)Vβ, Vc=-Vα/2-(√3/2)Vβ
 *          Step 2 - 共模注入: Vcm = (max+min)/2
 *            注入三次谐波, 电压利用率从 SPWM 50% 提升到 SVPWM 57.7%
 *          Step 3 - 归一化: duty = 0.5 + (Vx-Vcm)/Vbus
 *          Step 4 - 硬钳位: [0,1], 浮点舍入误差保底
 *
 *          七段式 THD 更小, 20kHz 下开关损耗可控, 选用七段式。
 */
void CTL_SVPWM(float  v_alpha, float  v_beta, float vbus,
               float *duty_a,  float *duty_b, float *duty_c)
{
    float va, vb, vc;
    float vmax, vmin, vcm;

    /* Step 1: 反 Clarke → 三相相电压 */
    va = v_alpha;
    vb = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    vc = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    /* Step 2: 共模注入量 = (max + min) / 2 */
    vmax = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    vmin = va;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;
    vcm = 0.5f * (vmax + vmin);

    /* Step 3: 归一化——物理电压 (V) → PWM 占空比 [0,1]
     *         防除零: Vbus < 0.1V 时钳到 0.1V（正常≥12V，仅异常掉电触发） */
    if (vbus < 0.1f) vbus = 0.1f;

    *duty_a = 0.5f + (va - vcm) / vbus;
    *duty_b = 0.5f + (vb - vcm) / vbus;
    *duty_c = 0.5f + (vc - vcm) / vbus;

    /* Step 4: 硬钳位 [0,1]——浮点舍入误差的最后防线 */
    if (*duty_a > 1.0f) *duty_a = 1.0f;
    if (*duty_a < 0.0f) *duty_a = 0.0f;
    if (*duty_b > 1.0f) *duty_b = 1.0f;
    if (*duty_b < 0.0f) *duty_b = 0.0f;
    if (*duty_c > 1.0f) *duty_c = 1.0f;
    if (*duty_c < 0.0f) *duty_c = 0.0f;
}

/*==========================================================================*/
/* CTL_RawToElectrical                                                      */
/*==========================================================================*/

/**
 * @brief   编码器 raw 值 (14-bit) → 电角度 (rad)
 * @param   raw         编码器原始值 (0~16383)
 * @param   pole_pairs  电机极对数（GM3506: 11）
 * @param   enc_offset  编码器零点偏置 (0~16383)
 * @return  电角度 (rad), 范围 [0, 2π × pole_pairs)
 *
 * @note    转换: adjusted=(raw-offset) mod 16384, θ_mech=adjusted/16384×2π,
 *          θ_elec=θ_mech×pole_pairs。环形减法 uint16_t→int32_t 防回绕。
 *          AS5048A: 14-bit, 满量程 16384, 分辨率 0.022°。
 */
float CTL_RawToElectrical(uint16_t raw, uint8_t pole_pairs, uint16_t enc_offset)
{
    int32_t adjusted;
    float   theta_mech;

    /* 环形减法: (raw - offset) mod 16384
     * 转 int32_t 防 uint16_t 减法回绕 */
    adjusted = (int32_t)raw - (int32_t)enc_offset;
    if (adjusted < 0) {
        adjusted += 16384;
    } else if (adjusted >= 16384) {
        adjusted -= 16384;
    }

    /* 机械角度 = adjusted * (2π/16384) */
    theta_mech = (float)adjusted * ENC_RAD_PER_LSB;

    /* 电角度 = 机械角度 × 极对数 */
    return theta_mech * (float)pole_pairs;
}
