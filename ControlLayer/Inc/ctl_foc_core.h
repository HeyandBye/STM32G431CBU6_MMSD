/**
******************************************************************************
* @file     ctl_foc_core.h
* @author   lidongyang
* @version  0.0.4
* @date     24-June-2026
* @brief    FOC 基础设施 —— 公共结构体、状态机、系统初始化、故障检测
*
* @note     目标电机: iPower GM3506（24N/22P, 11 对极, 5.6Ω, 1A@12V）
*           编码器: AS5048A 14-bit 磁编码器
*
*           模块分工:
*           - ctl_foc_core.c:      公共基础设施（Init/SystemInit/CheckFault）
*           - ctl_foc_openloop.c:  开环电压拖动
*           - ctl_foc_current.c:   电流闭环（Id=0 + PI）
*
*           初始化顺序（电流闭环为例）:
*           FOC_Current_Init → FOC_Current_CalibrateEncoder
*           → FOC_Current_SetRef → FOC_Current_Start
******************************************************************************
*/

#ifndef CTL_FOC_CORE_H
#define CTL_FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ctl_pid.h"
#include "ctl_foc_openloop.h"

/*==========================================================================*/
/* 枚举 & 结构体                                                             */
/*==========================================================================*/

/** @brief FOC 运行状态 */
typedef enum {
    FOC_STATE_IDLE    = 0,   /**< 未初始化 / 已停止                    */
    FOC_STATE_READY   = 1,   /**< 参数已配置，等待 Start                */
    FOC_STATE_RUNNING = 2,   /**< 控制环路运行中（PWM 有输出）            */
    FOC_STATE_FAULT   = 3    /**< 故障保护（过流/过压/编码器异常）      */
} FOC_State_t;

/** @brief 电机物理参数 */
typedef struct {
    uint8_t  pole_pairs;      /**< 极对数（GM3506: 22极/2 = 11）       */
    uint16_t enc_offset;      /**< 编码器电角度零偏（raw 0~16383）      */
    float    rated_current;   /**< 额定电流 (A)，校准电流 = rated×0.3   */
    float    current_limit;   /**< Id/Iq 最大限幅 (A)，过流保护用       */
    float    vbus_nominal;    /**< 额定母线电压 (V)，PID 输出限幅基准    */
} FOC_Motor_t;

/**
 * @brief FOC 主控制器
 * @note    除 duty_a/b/c ∈ [0,1] 外, 所有字段均为物理单位。
 *          内存约 156B（23 float + 2 PID + 电机参数 + 状态）
 */
typedef struct {
    /* ---- 运行状态 ---- */
    FOC_State_t   state;

    /* ---- 电机参数 ---- */
    FOC_Motor_t   motor;

    /* ---- PID 控制器（Kd=0 即 PI 模式） ---- */
    CTL_PID_t     pid_id;      /**< d 轴电流 PID（默认 Kd=0, Kr=1.0）  */
    CTL_PID_t     pid_iq;      /**< q 轴电流 PID（默认 Kd=0, Kr=1.0）  */

    /* ---- 电流给定 (A) ---- */
    float         id_ref;      /**< d 轴电流给定（Id=0 策略固定为 0）    */
    float         iq_ref;      /**< q 轴电流给定（正值=正转矩）          */

    /* ---- 实时变量（每控制周期更新） ---- */
    uint16_t      raw_angle;   /**< 编码器原始值 (0~16383)             */
    float         theta_elec;  /**< 电角度 (rad)                       */
    float         sin_theta;   /**< sin(θ_elec), 由 arm_sin_cos_f32 算出 */
    float         cos_theta;   /**< cos(θ_elec), 由 arm_sin_cos_f32 算出 */
    float         ia;          /**< A 相电流 (A)                       */
    float         ib;          /**< B 相电流 (A)                       */
    float         i_alpha;     /**< α 轴电流 (A)                       */
    float         i_beta;      /**< β 轴电流 (A)                       */
    float         id;          /**< d 轴电流实测 (A)                    */
    float         iq;          /**< q 轴电流实测 (A)                    */
    float         vd;          /**< d 轴电压输出 (V)                    */
    float         vq;          /**< q 轴电压输出 (V)                    */
    float         v_alpha;     /**< α 轴电压 (V)                       */
    float         v_beta;      /**< β 轴电压 (V)                       */
    float         vbus;        /**< 母线电压 (V)                       */

    /* ---- PWM 输出（归一化 [0,1]） ---- */
    float         duty_a;      /**< A 相占空比 [0, 1]                   */
    float         duty_b;      /**< B 相占空比 [0, 1]                   */
    float         duty_c;      /**< C 相占空比 [0, 1]                   */

    /* ---- 诊断 ---- */
    uint32_t      loop_count;  /**< 控制周期计数（溢出自动回绕）         */
    uint32_t      fault_code;  /**< 故障码（0=无故障，见 FOC_FAULT_*）  */
    uint32_t      fault_consec;/**< 连续故障计数（消抖：N 次才触发）     */
    float         speed_rpm;   /**< 机械转速 (RPM), TIM6 ISR 内 1kHz 更新 */
    float         theta_prev;  /**< 上一周期电角度, 转速计算用            */

    /* ---- 阻尼模式 ---- */
    float         damper_gain; /**< 阻尼增益 (A/RPM), 仅 FOC_MODE_DAMPER  */

    /* ---- 速度环 ---- */
    CTL_PID_t     pid_speed;   /**< 速度 PID 控制器                      */
    float         speed_ref;   /**< 目标转速 (RPM)                       */
    float         speed_iq_limit; /**< 速度环输出 Iq 限幅 (A)            */
    float         speed_limit; /**< 转速硬限幅 (RPM)                     */

    /* ---- 位置环 ---- */
    CTL_PID_t     pid_pos;     /**< 位置 PID 控制器                      */
    uint16_t      pos_ref;     /**< 目标位置 (编码器 raw)                */
    float         pos_speed_limit; /**< 位置环输出限幅 (RPM)              */
    float         unwrapped_pos;   /**< 展开后的累计位置 (LSB, 无回绕)      */
    uint16_t      raw_prev;        /**< 上一周期 raw_angle, 展开计算用       */
} FOC_t;

/*==========================================================================*/
/* 故障码                                                                    */
/*==========================================================================*/

#define FOC_FAULT_NONE          0x00000000U  /**< 无故障                 */
#define FOC_FAULT_OVERCURRENT   0x00000001U  /**< 硬件过流                */
#define FOC_FAULT_OVERVOLTAGE   0x00000002U  /**< 母线过压                */
#define FOC_FAULT_ENCODER       0x00000004U  /**< 编码器通信异常           */
#define FOC_FAULT_UNDERVOLTAGE  0x00000008U  /**< 母线欠压（无法正常调制） */

/*==========================================================================*/
/* FOC 运行模式（编译期切换，修改 FOC_MODE 宏即可）                             */
/*==========================================================================*/

#define FOC_MODE_OPENLOOP  1  /**< 开环电压拖动（虚拟角度 + SVPWM）       */
#define FOC_MODE_CURRENT   2  /**< 电流闭环（Id=0，需要编码器校准）        */
#define FOC_MODE_SPEED     3  /**< 速度闭环（电流内环 + 速度外环）         */
#define FOC_MODE_POSITION  4  /**< 位置闭环（电流内环 + 位置外环）         */
#define FOC_MODE_DAMPER    5  /**< 阻尼模式（转动有阻力, 停手即锁定）       */

/** @brief 当前活跃的 FOC 模式（编译期常量，修改此处切换模式） */
#define FOC_MODE  FOC_MODE_POSITION

/*==========================================================================*/
/* 全局 FOC 实例                                                             */
/*==========================================================================*/

/** @brief 全局 FOC 控制器实例（在 ctl_foc_core.c 中定义，供 main.c 初始化和 HAL 回调使用） */
extern FOC_t g_foc;

/** @brief 全局开环 FOC 实例（仅在 FOC_MODE_OPENLOOP 时使用） */
extern FOC_OpenLoop_t g_fol;

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   初始化 FOC 控制器（清零所有状态, 置 IDLE）
 * @param   foc  FOC 控制器指针
 * @note    初始化链第一步。调用后需依次 SetMotorParams → 模式相关初始化。
 */
void FOC_Init(FOC_t *foc);

/**
 * @brief   设置电机参数（置 READY 状态）
 * @param   foc            FOC 控制器指针
 * @param   pole_pairs     极对数（GM3506: 11）
 * @param   enc_offset     编码器零点偏置（未知填 0, 后续校准）
 * @param   current_rated  额定电流 (A)（GM3506: 1.0）
 * @param   current_limit  Id/Iq 限幅 (A)（GM3506: 2.0）
 * @param   vbus_nominal   额定母线电压 (V)（GM3506: 12.0）
 */
void FOC_SetMotorParams(FOC_t   *foc,
                        uint8_t  pole_pairs,
                        uint16_t enc_offset,
                        float    current_rated,
                        float    current_limit,
                        float    vbus_nominal);

/**
 * @brief   紧急停机（任意状态 → FAULT）
 * @note    占空比归 50%, PID 清零。复位需重新执行完整初始化链。
 */
void FOC_EmergencyStop(FOC_t *foc);

/**
 * @brief   尝试从故障中恢复（FAULT → READY → RUNNING）
 * @param   foc  FOC 控制器指针
 * @return  0=恢复成功, -1=恢复失败
 * @note    重新使能 MOE, 清零故障码, 恢复电流给定, 重置 PID。
 *          主循环中检测到 FAULT 状态后延时调用此函数。
 */
int32_t FOC_RecoverFromFault(FOC_t *foc);

/* ---- 查询 ---- */

/** @brief 获取当前运行状态 */
FOC_State_t FOC_GetState(const FOC_t *foc);

/**
 * @brief   获取 Id/Iq 实测值 (A)
 * @param   foc  FOC 控制器指针
 * @param   id   输出: d 轴电流 (A)
 * @param   iq   输出: q 轴电流 (A)
 * @note    读取上一周期计算结果，实时性 <= 50us。
 */
void FOC_GetCurrents(const FOC_t *foc, float *id, float *iq);

/**
 * @brief   获取当前三相占空比 [0, 1]
 * @note    可用于判断 SVPWM 是否接近饱和边界。
 */
void FOC_GetDuties(const FOC_t *foc, float *da, float *db, float *dc);

/**
 * @brief   故障检测（每 PWM 周期调用一次）
 *
 * @note    检测过压、欠压、过流、编码器异常。
 *          故障码可组合（位或），返回值 0 = 无故障。
 *
 * @param   foc               FOC 控制器指针
 * @param   bus_voltage_max   母线过压阈值 (V)（GM3506: 20V, 3S LiPo 12.6V×1.6）
 * @param   current_max       硬件过流阈值 (A)（GM3506: 2.5A, 限幅 2.0A×1.25）
 * @param   enc_valid         编码器通信正常=1, 异常=0（AS5048A 偶校验/超时）
 * @return  0 = 无故障，非 0 = 故障码（位或: FOC_FAULT_OVERCURRENT|OVERVOLTAGE|...）
 */
uint32_t FOC_CheckFault(const FOC_t *foc,
                        float bus_voltage_max,
                        float current_max,
                        uint8_t enc_valid);

/* ---- 系统初始化 ---- */

/**
 * @brief   FOC 系统总初始化（替代 main.c 中分散的驱动初始化）
 *
 * @note    根据 FOC_MODE 自动完成:
 *          1. 驱动层: AS5048A + PWM + ADC 采样初始化
 *          2. 电机参数配置
 *          3. 模式相关: 开环=Init, 闭环=PID+校准+启动
 *          4. 注册 ADC 转换完成回调（控制环路入口）
 *
 *          main.c 中仅需: MX_xx_Init → FOC_SystemInit → while(1)
 */
void FOC_SystemInit(void);

#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_CORE_H */
