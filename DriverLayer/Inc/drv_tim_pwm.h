/**
******************************************************************************
* @file     drv_tim_pwm.h
* @author   lidongyang
* @version  0.0.1
* @date     18-June-2026
* @brief    TIM1 六路互补 PWM 驱动
*
* @attention    硬件引脚：
*               PA8  = TIM1_CH1  (A 相高侧, AF6)   PB13 = TIM1_CH1N (A 相低侧, AF6)
*               PA9  = TIM1_CH2  (B 相高侧, AF6)   PB14 = TIM1_CH2N (B 相低侧, AF6)
*               PA10 = TIM1_CH3  (C 相高侧, AF6)   PB15 = TIM1_CH3N (C 相低侧, AF4 !)
******************************************************************************
*/

#ifndef DRV_TIM_PWM_H
#define DRV_TIM_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/** @brief PWM 开关频率 20 kHz（人耳听觉上限 20kHz, 无啸叫声） */
#define PWM_FREQ_HZ        20000U

/**
 * @brief 死区宏开关: 1 = 使能 200ns, 0 = 关闭（调试用）
 * @note  调试模式可暂时关闭死区以排查波形异常原因。
 *        但实际驱动电机时必须使能, 否则有直通短路风险!
 */
#define PWM_DEADTIME_ENABLE 0U

/**
 * @brief 互补通道开关: 1 = 使能 CH1N/CH2N/CH3N（六路输出）, 0 = 仅主通道（三路输出）
 * @note  互补通道（CHxN）是相对于主通道（CHx）的反相 PWM 信号（带死区插入），
 *        不是独立的使能引脚。六路输出模式将主/互补通道直接连接到 MOSFET
 *        上下桥臂栅极，由 TIM1 硬件自动生成互补波形。
 *
 *        当前项目采用正弦波/FOC 控制，仅使用三路主通道 PWM 输出，
 *        三相共用一个 EN 使能引脚（DRV8313 的 EN 引脚）同时控制
 *        所有相位的输出通断，不进行六步换向的逐相 Hi-Z 控制。
 *
 *        六路互补输出模式（CHx + CHxN）适合直接驱动 MOSFET 桥臂，
 *        互补调制可降低谐波并减小转矩脉动。当前设为 0（仅主通道），
 *        因使用 DRV8313 集成驱动芯片，其内部已处理上下桥臂驱动逻辑。
 */
#define PWM_ENABLE_N_CHANNELS 0U

/**
 * @brief 自动重载值 = 4249
 * @note  f_PWM = 170MHz / (2 × 4250) ≈ 20.0kHz
 *        ARR 对应 50µs 的 PWM 周期
 */
#define PWM_ARR            4249U

/** @brief 占空比最大值 = ARR（对应 100% 上管占空比） */
#define PWM_DUTY_MAX       4249U

/** @brief 占空比最小值 = 0（对应 0% 上管占空比, 下管 100%） */
#define PWM_DUTY_MIN       0U

/**
 * @brief 死区 DTG 寄存器值: 34 × 5.882ns ≈ 200ns
 * @note  DTG = 死区时间 (ns) / T_dts (ns)
 *        T_dts = 1 / f_timer = 1 / 170MHz ≈ 5.882ns
 */
#define PWM_DTG_200NS      34U

/*==========================================================================*/
/* 调试用: 当前占空比                                                        */
/*==========================================================================*/

extern volatile uint16_t g_pwm_duty_a;  /**< A 相 CCR 值（0 ~ 4249, main 循环 printf 读取） */
extern volatile uint16_t g_pwm_duty_b;  /**< B 相 CCR 值                                   */
extern volatile uint16_t g_pwm_duty_c;  /**< C 相 CCR 值                                   */

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   初始化 PWM 驱动（修复死区时间 + 设初始占空比 50%）
 * @param   htim 已由 CubeMX 初始化的 TIM_HandleTypeDef 结构体指针
 * @note    必须在 MX_TIM1_Init 之后调用, 否则 CubeMX 配置被覆盖。
 *          仅写 BDTR 死区位, 不动 CubeMX 已配好的 GPIO / 时基 / 通道配置。
 */
void drv_tim_pwm_init(TIM_HandleTypeDef *htim);

/**
 * @brief   使能 PWM 输出（启动六路通道 + CEN 计数器 + MOE 总输出）
 * @note    顺序不能乱:
 *          1. HAL_TIM_PWM_Start → 置 CCxE（主通道使能）
 *          2. HAL_TIMEx_PWMN_Start → 置 CCxNE（互补通道使能）
 *          3. __HAL_TIM_ENABLE → 置 CEN（计数器使能）
 *          4. SET_BIT BDTR MOE → 打开输出总闸
 *          MOE 必须最后: 手册要求 MOE 在 CEN=1 之后才能置位。
 */
void drv_tim_pwm_enable(void);

/**
 * @brief   紧急关断 MOE（仅清除总输出使能, ISR 安全, 硬件级快速关断）
 * @note    MOE 清零 → 所有 PWM 输出立即进空闲态（配置为 LOW）。
 *          定时器和通道保持配置, 可在故障解除后快速恢复。
 *          用于 DRV8313 nFAULT 保护路径。
 */
void drv_tim_pwm_moe_off(void);

/**
 * @brief   紧急关断 PWM（清 MOE → 六路立即进空闲态 → 停止计数器）
 * @note    比 drv_tim_pwm_moe_off 更彻底: 计数器一并停掉。
 */
void drv_tim_pwm_disable(void);

/**
 * @brief   设置三相占空比（整数, 0 ~ PWM_DUTY_MAX）
 * @param   a/b/c  A/B/C 相 CCR 比较值, 自动钳位到 [PWM_DUTY_MIN, PWM_DUTY_MAX]
 */
void drv_tim_pwm_set_duty(uint16_t a, uint16_t b, uint16_t c);

/**
 * @brief   设置三相占空比（浮点数, 0.0 ~ 1.0）
 * @param   a/b/c  占空比（0.0 = 下管全通, 0.5 = 50%, 1.0 = 上管全通）
 */
void drv_tim_pwm_set_duty_f(float a, float b, float c);

#ifdef __cplusplus
}
#endif

#endif /* DRV_TIM_PWM_H */
