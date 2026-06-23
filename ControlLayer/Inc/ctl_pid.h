/**
******************************************************************************
* @file     ctl_pid.h
* @author   lidongyang
* @version  0.0.4
* @date     23-June-2026
* @brief    PI/PID 控制器 —— 并行式 + setpoint 加权 + 条件积分抗饱和
*           + 微分低通滤波
*
* @note     归一化策略: 不使用 [-1,1] 归一化, 保留物理单位。
*           归一化由调用者在传入前自行处理（除以基准值）。详见 ctl_pid.c。
*
*           参考:
*           - stm32f10x_pid_fixed（条件积分抗饱和 + 微分低通滤波）
*           - PID_GRANDO_F（并行式 PID + Kr setpoint 加权 + 条件积分抗饱和）
*
*           并行式离散 PID（setpoint 加权形式）:
*            e(k)      = setpoint - feedback
*            prop      = Kp×(Kr×setpoint - feedback)
*            u_raw     = prop + integral + deriv_filt
*            u_out     = clamp(u_raw, out_min, out_max)
*            integral += Ki×e(k)                         // 未饱和时
*            deriv    = Kd×(e(k)-e(k-1)) + IIR 低通       // Kd=0 跳过
*
*           适用: 电流环（Kd=0, Kr=1.0 → PI）、速度环（Kd>0, Kr=0.5~0.8 → PID）。
*           CTL_PI_t 是 CTL_PID_t 的别名（Kd=0, Kr=1.0）。
******************************************************************************
*/

#ifndef CTL_PID_H
#define CTL_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/* 结构体 —— PID（PI 是 Kd=0, Kr=1.0 的特例）                               */
/*==========================================================================*/

/**
 * @brief PID 控制器
 *
 * @note    归一化: 否。setpoint/feedback/error/output 均保留物理单位。
 *
 *          setpoint 加权 (Kr):
 *          Kr=1.0 → 比例项 = Kp×(setpoint-feedback), 标准 PID
 *          Kr<1.0 → 比例项 = Kp×(Kr×setpoint-feedback), 减少给定突变冲击
 *          积分和微分始终用完整误差, 保证无静差。推荐: 电流环 1.0, 速度/位置环 0.5~0.8。
 *
 *          微分滤波 (deriv_alpha):
 *          α ∈ (0, 1], 一阶 IIR: df(k)=α×raw+(1-α)×df(k-1)
 *          α=0.01 极强滤波, α=0.1 推荐值, α=1.0 无滤波。
 *          Kd=0 时微分完全跳过, α 无影响。
 *
 *          内存: 15 × float = 60B/实例
 */
typedef struct {
    float Kp;               /**< 比例增益                              */
    float Ki;               /**< 积分增益                              */
    float Kd;               /**< 微分增益，0=PI 模式                    */
    float Kr;               /**< setpoint 加权系数 [0, 1]，默认 1.0    */
    float setpoint;         /**< 设定值/目标值（物理单位）               */
    float feedback;         /**< 最新反馈值（诊断用）                   */
    float integral;         /**< 积分累加器                            */
    float integral_max;     /**< 积分上限（= output_max × 0.3）         */
    float integral_min;     /**< 积分下限（= output_min × 0.3）         */
    float output_max;       /**< 输出上限                              */
    float output_min;       /**< 输出下限                              */
    float last_error;       /**< 上一周期误差 e(k-1)，微分用            */
    float deriv_state;      /**< 微分滤波状态                          */
    float deriv_alpha;      /**< 微分低通滤波系数 (0, 1]，默认 0.1      */
} CTL_PID_t;

/** @brief PI 控制器（PID 别名，Kd=0, Kr=1.0） */
typedef CTL_PID_t CTL_PI_t;

/*==========================================================================*/
/* API —— PID 版本                                                          */
/*==========================================================================*/

/**
 * @brief   PID 控制器初始化
 *
 * @note    将所有字段设为初始值:
 *          - Kp/Ki/Kd/Kr 由参数传入
 *          - setpoint/feedback/integral/deriv_state/last_error 清零
 *          - 积分限幅 = output 限幅 × 0.3
 *          - Kr 钳位到 [0, 1], deriv_alpha 钳位到 (0, 1]
 *
 *          Kr=1.0 标准 PID, Kr<1.0 减少给定突变冲击。
 *          推荐: 电流环 Kr=1.0, 速度/位置环 Kr=0.5~0.8。
 *
 * @param   pid          PID 控制器指针
 * @param   kp           比例增益（物理量纲, 如 V/A）
 * @param   ki           积分增益（Ki = Kp×Ts/Ti）
 * @param   kd           微分增益（0 = PI 模式）
 * @param   kr           setpoint 加权系数 [0, 1]
 * @param   deriv_alpha  微分低通滤波系数 (0, 1], 推荐 0.1
 * @param   out_min      输出下限
 * @param   out_max      输出上限
 */
void CTL_PID_Init(CTL_PID_t *pid, float kp, float ki, float kd, float kr,
                  float deriv_alpha, float out_min, float out_max);

/**
 * @brief   写入 setpoint（设定值/目标值）
 * @note    仅做简单赋值, ISR 中安全调用。
 *          下一周期 CTL_PID_Update 内部自动计算 error。
 * @param   pid       PID 控制器指针
 * @param   setpoint  设定值（物理单位, 与 feedback 同量纲）
 */
void CTL_PID_SetSetpoint(CTL_PID_t *pid, float setpoint);

/**
 * @brief   在线更新 Kp/Ki/Kd/Kr（不重置积分器/微分器）
 * @note    适合在线调参——增益改变后状态保持, 下一周期立即生效。Kr 钳位到 [0,1]。
 * @param   pid  PID 控制器指针
 * @param   kp   新比例增益
 * @param   ki   新积分增益
 * @param   kd   新微分增益
 * @param   kr   新 setpoint 加权系数 [0, 1]
 */
void CTL_PID_SetGains(CTL_PID_t *pid, float kp, float ki, float kd, float kr);

/**
 * @brief   调整微分低通滤波强度（不重置滤波状态）
 * @note    α ∈ (0, 1]: α=0.01 极强滤波, α=0.1 推荐值, α=1.0 无滤波。
 *          钳位: ≤0→0.1, >1→1.0。
 * @param   pid          PID 控制器指针
 * @param   deriv_alpha  新滤波系数 (0, 1]
 */
void CTL_PID_SetDerivAlpha(CTL_PID_t *pid, float deriv_alpha);

/**
 * @brief   设置输出限幅（同步更新积分限幅 = output_limit × 0.3）
 * @note    现有积分若超出新范围则立即钳位, 防止限幅缩小后积分越界。
 * @param   pid       PID 控制器指针
 * @param   out_min   输出下限
 * @param   out_max   输出上限
 */
void CTL_PID_SetLimits(CTL_PID_t *pid, float out_min, float out_max);

/**
 * @brief   PID 单步更新（并行式 + setpoint 加权 + 条件积分抗饱和 + 微分低通滤波）
 *
 * @note    算法（5 步）: error → 微分(IIR 低通) → 合成(prop+int+deriv) → 钳位 → 抗饱和更新
 *          冻结条件: (正饱和 && error>0) || (负饱和 && error<0)
 *          比例项 = Kp×(Kr×setpoint-feedback), Kr=1 标准 PID, Kr<1 减少给定突变冲击
 *
 * @param   pid       PID 控制器指针
 * @param   feedback  当前反馈值（物理单位, 与 setpoint 同量纲）
 * @return  控制器输出（已钳位到 [out_min, out_max]）
 */
float CTL_PID_Update(CTL_PID_t *pid, float feedback);

/**
 * @brief   重置 PID 控制器
 * @note    清零积分器、微分滤波状态、误差历史。Kp/Ki/Kd/Kr 和限幅参数保持不变。
 *          适用于控制模式切换、启动、故障恢复后重新使能。
 * @param   pid  PID 控制器指针
 */
void CTL_PID_Reset(CTL_PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* CTL_PID_H */
