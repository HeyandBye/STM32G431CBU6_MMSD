/**
******************************************************************************
* @file     ctl_pid.c
* @author   lidongyang
* @version  0.0.4
* @date     23-June-2026
* @brief    PI/PID 控制器实现 —— 并行式 + setpoint 加权 + 条件积分抗饱和
*           + 微分低通滤波
*
* @note     归一化策略:
*           本 PID 控制器不归一化到 [-1,1], 保留物理单位。
*
*           与通用归一化 PID 库的区别:
*           - 物理量固定（电流 A、电压 V）, Kp/Ki/Kd 有明确量纲
*           - 调试直读 "Id=0.3A, Vd=2.1V" 比归一化值直观
*           - Cortex-M4 FPU 硬件浮点, 无性能损失
*           - 归一化由上层调用者自行处理（除以基准值）, PID 保持纯净
*
*           算法参考:
*           - stm32f10x_pid_fixed: 条件积分抗饱和 + 微分低通滤波
*           - PID_GRANDO_F:       Kr setpoint 加权 + 条件积分抗饱和
*
*           融合设计:
*           1. 比例项 = Kp*(Kr*setpoint - feedback)      —— PID_GRANDO_F
*           2. 积分项 = Ki×Σerror, 未饱和时累积           —— 条件积分抗饱和
*           3. 微分项 = Kd×(e(k)-e(k-1)) + IIR 低通      —— stm32f10x_pid_fixed
*           4. 输出饱和 → 积分+微分+误差历史全部冻结        —— 融合两者
******************************************************************************
*/

#include "ctl_pid.h"
#include <stddef.h>

/*==========================================================================*/
/* 内部默认值                                                                */
/*==========================================================================*/

/** @brief 积分限幅比例 = 输出限幅 × 0.3
 *  积分项通常只占输出一小部分, 钳在 30% 防深度饱和且不削弱积分能力 */
#define DEFAULT_INTEGRAL_RATIO  0.3f

/** @brief 微分低通滤波默认系数: α=0.1 → 约 -20dB/dec 高频衰减 */
#define DEFAULT_DERIV_ALPHA     0.1f

/**
 * @brief   PID 控制器初始化
 * @param   pid          PID 控制器指针
 * @param   kp           比例增益
 * @param   ki           积分增益
 * @param   kd           微分增益（0=PI 模式）
 * @param   kr           setpoint 加权系数 [0, 1]
 * @param   deriv_alpha  微分低通滤波系数 (0, 1]
 * @param   out_min      输出下限
 * @param   out_max      输出上限
 *
 * @note    积分限幅 = output 限幅 × 0.3。Kr 钳位到 [0,1]。
 *          推荐: 电流环 Kr=1.0, 速度/位置环 Kr=0.5~0.8。
 */
void CTL_PID_Init(CTL_PID_t *pid, float kp, float ki, float kd, float kr,
                  float deriv_alpha, float out_min, float out_max)
{
    if (pid == NULL) return;

    pid->Kp           = kp;
    pid->Ki           = ki;
    pid->Kd           = kd;
    pid->Kr           = kr;
    pid->setpoint     = 0.0f;
    pid->feedback     = 0.0f;
    pid->integral     = 0.0f;
    pid->output_max   = out_max;
    pid->output_min   = out_min;
    pid->last_error   = 0.0f;
    pid->deriv_state  = 0.0f;
    pid->deriv_alpha  = deriv_alpha;

    /* Kr 钳位 [0, 1] */
    if (pid->Kr < 0.0f) pid->Kr = 0.0f;
    if (pid->Kr > 1.0f) pid->Kr = 1.0f;

    /* 积分限幅 = 输出限幅 × 0.3——防深度饱和 */
    pid->integral_max = out_max * DEFAULT_INTEGRAL_RATIO;
    pid->integral_min = out_min * DEFAULT_INTEGRAL_RATIO;
    if (pid->integral_min > pid->integral_max) {
        pid->integral_min = pid->integral_max;
    }

    /* 微分滤波系数钳位 */
    if (pid->deriv_alpha <= 0.0f) pid->deriv_alpha = DEFAULT_DERIV_ALPHA;
    if (pid->deriv_alpha >  1.0f) pid->deriv_alpha = 1.0f;
}

/**
 * @brief   写入 setpoint（设定值/目标值）
 * @param   pid       PID 控制器指针
 * @param   setpoint  设定值（物理单位）
 * @note    ISR 中安全调用。error 由 CTL_PID_Update 内部计算。
 */
void CTL_PID_SetSetpoint(CTL_PID_t *pid, float setpoint)
{
    if (pid == NULL) return;
    pid->setpoint = setpoint;
}

/**
 * @brief   在线更新 Kp/Ki/Kd/Kr（不重置积分器/微分器）
 * @param   pid  PID 控制器指针
 * @param   kp   新比例增益
 * @param   ki   新积分增益
 * @param   kd   新微分增益
 * @param   kr   新 setpoint 加权系数 [0, 1]
 * @note    Kr 钳位到 [0,1]。
 */
void CTL_PID_SetGains(CTL_PID_t *pid, float kp, float ki, float kd, float kr)
{
    if (pid == NULL) return;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Kr = kr;
    if (pid->Kr < 0.0f) pid->Kr = 0.0f;
    if (pid->Kr > 1.0f) pid->Kr = 1.0f;
}

/**
 * @brief   调整微分低通滤波强度（不重置滤波状态）
 * @param   pid          PID 控制器指针
 * @param   deriv_alpha  新滤波系数 (0, 1]
 * @note    钳位: ≤0 → 0.1, >1 → 1.0。
 */
void CTL_PID_SetDerivAlpha(CTL_PID_t *pid, float deriv_alpha)
{
    if (pid == NULL) return;
    pid->deriv_alpha = deriv_alpha;
    if (pid->deriv_alpha <= 0.0f) pid->deriv_alpha = DEFAULT_DERIV_ALPHA;
    if (pid->deriv_alpha >  1.0f) pid->deriv_alpha = 1.0f;
}

/**
 * @brief   设置输出限幅（同步更新积分限幅）
 * @param   pid       PID 控制器指针
 * @param   out_min   输出下限
 * @param   out_max   输出上限
 * @note    积分限幅自动 = output 限幅 × 0.3。
 */
void CTL_PID_SetLimits(CTL_PID_t *pid, float out_min, float out_max)
{
    if (pid == NULL) return;

    pid->output_max = out_max;
    pid->output_min = out_min;

    pid->integral_max = out_max * DEFAULT_INTEGRAL_RATIO;
    pid->integral_min = out_min * DEFAULT_INTEGRAL_RATIO;
    if (pid->integral_min > pid->integral_max) {
        pid->integral_min = pid->integral_max;
    }

    /* 立即钳位现有积分到新范围——防止限幅缩小后积分越界 */
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
}

/**
 * @brief   PID 单步更新（并行式 + setpoint 加权 + 条件积分抗饱和 + 微分低通滤波）
 * @param   pid       PID 控制器指针
 * @param   feedback  当前反馈值（物理单位）
 * @return  控制器输出（已钳位到 [out_min, out_max]）
 *
 * @note    5 步: error → 微分(IIR 低通) → 合成(prop+int+deriv) → 钳位 → 抗饱和。
 *          冻结条件: (正饱和 && error>0) || (负饱和 && error<0)。
 */
float CTL_PID_Update(CTL_PID_t *pid, float feedback)
{
    float error;
    float prop_term;
    float u_raw;
    float u_out;
    float deriv_raw;
    float deriv_new;
    int   frozen;

    if (pid == NULL) return 0.0f;

    /* 存储反馈值（供外部诊断查询） */
    pid->feedback = feedback;

    /* ---- Step 1: 误差 = setpoint - feedback ---- */
    error = pid->setpoint - feedback;

    /* ---- Step 2: 微分项 = Kd×(e(k)-e(k-1)) + IIR 低通滤波 ----
     *         Kd=0 时跳过整个微分分支（零开销） */
    if (pid->Kd != 0.0f) {
        /* 后向差分: Δe = e(k) - e(k-1) */
        deriv_raw = pid->Kd * (error - pid->last_error);

        /* 一阶 IIR 低通: 抑制测量噪声对微分项的放大
         * deriv_new = α*raw + (1-α)*prev
         * α=0.1 → 新值占 10%，旧值占 90%，平滑效果强 */
        deriv_new = pid->deriv_alpha * deriv_raw
                  + (1.0f - pid->deriv_alpha) * pid->deriv_state;
    } else {
        deriv_new = 0.0f;
    }

    /* ---- Step 3: 比例项（Kr 加权 setpoint）+ 积分 + 微分 ---- */
    prop_term = pid->Kp * (pid->Kr * pid->setpoint - feedback);
    u_raw     = prop_term + pid->integral + deriv_new;

    /* ---- Step 4: 钳位输出到 [out_min, out_max] ---- */
    u_out = u_raw;
    if (u_out > pid->output_max) u_out = pid->output_max;
    if (u_out < pid->output_min) u_out = pid->output_min;

    /* ---- Step 5: 抗积分饱和——判断是否冻结 ---- */
    frozen = 0;
    if (u_raw > pid->output_max && error > 0.0f) {
        /* 正饱和 (u_raw 超上限) 且误差仍为正 (还需正向调节)
         * → 继续积分只会加剧饱和 → 冻结 */
        frozen = 1;
    }
    if (u_raw < pid->output_min && error < 0.0f) {
        /* 负饱和且误差仍为负 → 冻结 */
        frozen = 1;
    }

    /* 未冻结: 正常更新积分、微分滤波状态、误差历史 */
    if (!frozen) {
        /* 积分累积 + 内部钳位 */
        pid->integral += pid->Ki * error;

        if (pid->integral > pid->integral_max)
            pid->integral = pid->integral_max;
        if (pid->integral < pid->integral_min)
            pid->integral = pid->integral_min;

        /* 微分滤波状态更新 */
        pid->deriv_state = deriv_new;

        /* 误差历史更新（供下一周期差分用） */
        pid->last_error  = error;
    }
    /* 冻结时: integral, deriv_state, last_error 全部保持旧值。
     * 饱和解除后控制器从冻结前状态无缝恢复，
     * 不会因 last_error 跳跃产生虚假微分尖峰。 */

    return u_out;
}

/**
 * @brief   重置 PID 控制器
 * @param   pid  PID 控制器指针
 * @note    清零积分器、微分滤波状态、误差历史。
 *          Kp/Ki/Kd/Kr 和限幅参数保持不变。
 */
void CTL_PID_Reset(CTL_PID_t *pid)
{
    if (pid == NULL) return;
    pid->integral    = 0.0f;
    pid->deriv_state = 0.0f;
    pid->last_error  = 0.0f;
}
