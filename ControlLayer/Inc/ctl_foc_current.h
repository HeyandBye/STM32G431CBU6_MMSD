/**
******************************************************************************
* @file     ctl_foc_current.h
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 电流闭环 —— Id=0 策略, PI 控制, 编码器反馈
*
* @note     依赖 ctl_foc_core.h 中的 FOC_t 结构体。
*           与 ctl_foc_openloop.h 接口对称:
*           - Step:  单步算法（Clarke/Park/PID/InvPark/SVPWM）
*           - Run:   完整周期（读传感器 → Step → 写 PWM → 故障检测）
*
 *           对标 FOC_OpenLoop_Init, FOC_Current_Init 一步完成:
 *           Init（电机参数+PI参数）→ CalibrateEncoder → SetRef → Start
******************************************************************************
*/

#ifndef CTL_FOC_CURRENT_H
#define CTL_FOC_CURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ctl_foc_core.h"

/*==========================================================================*/
/* 配置结构体                                                                */
/*==========================================================================*/

/**
 * @brief 电流闭环初始化配置（对标 FOC_OpenLoop_Init 的参数列表）
 *
 * @note  电机参数 + PI 参数一次性配置, 传入 FOC_Current_Init 一步完成:
 *        FOC_Init → FOC_SetMotorParams → FOC_Current_ConfigPID。
 */
typedef struct {
    uint8_t  pole_pairs;       /**< 极对数（GM3506: 11）               */
    float    rated_current;    /**< 额定电流 (A)（GM3506: 1.0）        */
    float    current_limit;    /**< Id/Iq 限幅 (A)（GM3506: 2.0）      */
    float    vbus_nominal;     /**< 额定母线电压 (V)（GM3506: 12.0）   */
    float    kp_id, ki_id;     /**< d 轴 PI 增益（V/A, 推荐 2.0/0.05）*/
    float    kp_iq, ki_iq;     /**< q 轴 PI 增益（V/A, 推荐 2.0/0.05）*/
} FOC_Current_Config_t;

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   电流闭环初始化（对标 FOC_OpenLoop_Init）
 * @param   foc  FOC 控制器指针
 * @param   cfg  初始化配置（电机参数 + PI 参数）
 * @note    内部调用: FOC_Init → FOC_SetMotorParams → FOC_Current_ConfigPID。
 *          编码器偏置初始为 0, 需后续 CalibrateEncoder 校准。
 */
void FOC_Current_Init(FOC_t *foc, const FOC_Current_Config_t *cfg);

/**
 * @brief   配置 Id/Iq PI 参数
 * @param   foc     FOC 控制器指针
 * @param   kp_id   d 轴比例增益（V/A, 推荐 0.5~2.0）
 * @param   ki_id   d 轴积分增益（推荐 0.01~0.05）
 * @param   kp_iq   q 轴比例增益（V/A, 推荐 0.5~2.0）
 * @param   ki_iq   q 轴积分增益（推荐 0.01~0.05）
 * @note    Kd=0（PI 模式）, Kr=1.0（标准 setpoint 加权）
 */
void FOC_Current_ConfigPID(FOC_t *foc,
                           float kp_id, float ki_id,
                           float kp_iq, float ki_iq);

/**
 * @brief   编码器电角度零点自动校准（阻塞 ~500ms）
 * @param   foc             FOC 控制器指针
 * @param   read_angle_fn   编码器读取函数（如 drv_as5048a_read_angle）
 * @param   set_duty_fn     PWM 占空比设置函数（如 drv_tim_pwm_set_duty_f）
 * @note    注入固定 θ=0 电压矢量, 等待转子对齐, 读取编码器值作为零点偏置。
 *          ⚠️ 阻塞调用, PWM 需已使能。
 */
void FOC_Current_CalibrateEncoder(FOC_t      *foc,
                                  uint16_t (*read_angle_fn)(void),
                                  void     (*set_duty_fn)(float, float, float));

/**
 * @brief   手动设置编码器偏置（跳过自动校准）
 * @param   foc     FOC 控制器指针
 * @param   offset  编码器零点偏置（0~16383）
 * @note    适用于已知偏置值或已校准过的场景, 不检查有效性
 */
void FOC_Current_SetEncoderOffset(FOC_t *foc, uint16_t offset);

/**
 * @brief   启动电流闭环（READY → RUNNING）
 * @param   foc  FOC 控制器指针
 * @note    重置 PID 积分器和微分状态, 同步当前电流给定到 setpoint
 */
void FOC_Current_Start(FOC_t *foc);

/**
 * @brief   停止电流闭环（RUNNING → READY）
 * @param   foc  FOC 控制器指针
 * @note    占空比归 50%（绕组电压为零）, 电机自由滑行。
 *          PWM 保持输出不清 MOE, 方便快速重启。
 */
void FOC_Current_Stop(FOC_t *foc);

/**
 * @brief   设置电流给定（Id/Iq 目标值）
 * @param   foc     FOC 控制器指针
 * @param   id_ref  d 轴电流给定 (A), Id=0 策略固定为 0
 * @param   iq_ref  q 轴电流给定 (A), 正值=正转矩
 * @note    输入钳位到 [-current_limit, +current_limit]
 */
void FOC_Current_SetRef(FOC_t *foc, float id_ref, float iq_ref);

/**
 * @brief   FOC 电流环单步算法（20kHz ISR 内调用）
 * @param   foc        FOC 控制器指针
 * @param   raw_angle  编码器原始值（0~16383）
 * @param   ia         A 相电流 (A)
 * @param   ib         B 相电流 (A)
 * @param   vbus       母线电压 (V)
 * @note    7 步: raw→θ → Clarke → Park → PI → InvPark → SVPWM → 诊断
 */
void FOC_Current_Step(FOC_t   *foc,
                      uint16_t raw_angle,
                      float    ia,
                      float    ib,
                      float    vbus);

/**
 * @brief   FOC 电流闭环完整周期（ADC 回调中调用）
 * @param   foc  FOC 控制器指针
 * @note    封装: 状态检查 → 读传感器 → Step → 写 PWM → 故障检测
 */
void FOC_Current_Run(FOC_t *foc);

/** @brief ADC 回调包装（匹配 DRV_ADC_ConvCplt_Callback_t, 供 FOC_SystemInit 注册） */
void FOC_Current_Run_Callback(void);
/**
 * @brief   电流闭环测试斜坡（主循环中周期性调用）
 * @param   foc      控制器指针
 * @param   tick_ms  系统运行时间 (ms)
 * @note    0~1s: Id=0.5A 锁转子 → 1~1.5s: Iq 0→0.5A → 1.5s后: Id=0, Iq=0.5A 持续。
 */
void FOC_Current_TestRamp(FOC_t *foc, uint32_t tick_ms);
#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_CURRENT_H */
