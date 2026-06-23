/**
******************************************************************************
* @file     ctl_math.h
* @author   lidongyang
* @version  0.0.3
* @date     23-June-2026
* @brief    FOC 坐标变换 & SVPWM 调制 —— 纯数学函数, 无状态, 可独立测试
*
* @note     归一化: 所有函数使用物理单位（A / V / rad）。
*           仅 SVPWM 输出 duty ∈ [0,1]。详见 ctl_math.c。
*
*           全部为无副作用纯函数, 不依赖 HAL 或全局变量,
*           可在 PC 端用 C 单元测试框架验证后直接移植到 MCU。
******************************************************************************
*/

#ifndef CTL_MATH_H
#define CTL_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*==========================================================================*/
/* 坐标变换                                                                  */
/*==========================================================================*/

/**
 * @brief   Clarke 变换: 三相静止 → 两相静止（幅值不变形式）
 *
 * @note    归一化: 否。I/O 均为物理电流 (A)。
 *          幅值不变: Iα=Ia, Iβ=(Ia+2Ib)/√3
 *          |Iαβ|_peak = |Iabc|_peak, Id/Iq 直读为相电流幅值。
 *          ST FOC 库、SimpleFOC 均用此形式, 调试最直观。
 *          三相电流仅需 Ia、Ib（Ic = -Ia-Ib）。
 *
 * @param   ia       A 相电流 (A)
 * @param   ib       B 相电流 (A)
 * @param   i_alpha  输出: α 轴电流 (A)
 * @param   i_beta   输出: β 轴电流 (A)
 */
void CTL_Clarke(float ia, float ib,
                float *i_alpha, float *i_beta);

/**
 * @brief   Park 变换: 两相静止 (αβ) → 两相旋转 (dq)
 *
 * @note    归一化: 否。I/O 均为物理电流 (A), θ 为电角度 (rad)。
 *          公式: Id=Iα×cos(θ)+Iβ×sin(θ), Iq=-Iα×sin(θ)+Iβ×cos(θ)
 *          物理含义: αβ 坐标系旋转 -θ, d 轴对齐转子 N 极, q 轴超前 90°。
 *          性能: arm_sin_cos_f32（~0.15µs）+ 4 乘加 ≈ 0.3µs。
 *
 * @param   i_alpha   α 轴电流 (A)
 * @param   i_beta    β 轴电流 (A)
 * @param   theta     电角度 (rad)
 * @param   id        输出: d 轴电流 (A)
 * @param   iq        输出: q 轴电流 (A)
 */
void CTL_Park(float i_alpha, float i_beta, float theta,
              float *id, float *iq);

/**
 * @brief   逆 Park 变换: 两相旋转 (dq) → 两相静止 (αβ)
 *
 * @note    归一化: 否。Vd,Vq (V) → Vα,Vβ (V)。
 *          公式: Vα=Vd×cos(θ)-Vq×sin(θ), Vβ=Vd×sin(θ)+Vq×cos(θ)
 *          Vd,Vq 来自 PID 输出, 范围受 PID 限幅约束（±Vbus）。
 *          本函数不额外钳位——钳位由 PID 和 SVPWM 阶段完成。
 *
 * @param   vd       d 轴电压 (V)
 * @param   vq       q 轴电压 (V)
 * @param   theta    电角度 (rad)
 * @param   v_alpha  输出: α 轴电压 (V)
 * @param   v_beta   输出: β 轴电压 (V)
 */
void CTL_InvPark(float vd, float vq, float theta,
                 float *v_alpha, float *v_beta);

/*==========================================================================*/
/* SVPWM                                                                     */
/*==========================================================================*/

/**
 * @brief   SVPWM 七段式调制: Vα,Vβ (V) → 三相占空比 [0,1]
 *
 * @note    归一化: 输入否（物理电压 V）, 输出是（duty ∈ [0,1]）。
 *          这是整个 FOC 链路中唯一做归一化的环节。
 *
 *          算法: min/max 共模注入法（等效七段式 SVPWM）:
 *          Step 1 - 反 Clarke: Va=Vα, Vb=-Vα/2+(√3/2)Vβ, Vc=-Vα/2-(√3/2)Vβ
 *          Step 2 - 共模注入: Vcm=(max+min)/2, 电压利用率 50%→57.7%
 *          Step 3 - 归一化: duty = 0.5 + (Vx-Vcm)/Vbus, 0=下管全通, 0.5=50%, 1=上管全通
 *          Step 4 - 硬钳位: [0,1], 浮点舍入误差保底
 *          七段式 THD 更低, 每次换相只切一个桥臂, 选用七段式。
 *
 * @param   v_alpha  α 轴电压 (V)
 * @param   v_beta   β 轴电压 (V)
 * @param   vbus     直流母线电压 (V), 内部防除零（min=0.1V）
 * @param   duty_a   输出: A 相占空比 [0, 1]
 * @param   duty_b   输出: B 相占空比 [0, 1]
 * @param   duty_c   输出: C 相占空比 [0, 1]
 */
void CTL_SVPWM(float  v_alpha, float  v_beta, float vbus,
               float *duty_a,  float *duty_b, float *duty_c);

/*==========================================================================*/
/* 角度转换                                                                  */
/*==========================================================================*/

/**
 * @brief   编码器 raw 值 (14-bit) → 电角度 (rad)
 *
 * @note    归一化: 部分。raw ∈ [0,16383] → [0, 2π×pole_pairs) rad。
 *          转换: adjusted=(raw-offset) mod 16384, θ_mech=adjusted/16384×2π,
 *          θ_elec=θ_mech×pole_pairs。
 *          环形减法: uint16_t → int32_t 强转防回绕, O(1) ISR 安全。
 *          AS5048A: 14-bit, 满量程 16384, 分辨率 0.022°。
 *          enc_offset 由 FOC_Current_CalibrateEncoder() 获取, 未校准前填 0。
 *
 * @param   raw         编码器原始值 (0~16383)
 * @param   pole_pairs  电机极对数（GM3506: 11）
 * @param   enc_offset  编码器零点偏置 (0~16383)
 * @return  电角度 (rad), 范围 [0, 2π × pole_pairs)
 */
float CTL_RawToElectrical(uint16_t raw, uint8_t pole_pairs, uint16_t enc_offset);

#ifdef __cplusplus
}
#endif

#endif /* CTL_MATH_H */
