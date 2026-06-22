/**
******************************************************************************
* @file     drv_tim_pwm.h
* @author   lidongyang
* @version  0.0.1
* @date     18-June-2026
* @brief    TIM1 六路互补 PWM 驱动
*
* @attention    硬件引脚：
*               PA8  = TIM1_CH1  (A 相高侧)   PB13 = TIM1_CH1N (A 相低侧)
*               PA9  = TIM1_CH2  (B 相高侧)   PB14 = TIM1_CH2N (B 相低侧)
*               PA10 = TIM1_CH3  (C 相高侧)   PB15 = TIM1_CH3N (C 相低侧)
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
 * @note  3 路 PWM + 3 路使能引脚（EN_A/B/C）→ 最适合六步换向的 Hi-Z 控制。
 *        六路输出用于正弦波/FOC 模式, 互补调制降低谐波。
 *        当前设为 0（仅主通道）, 因调试阶段使用 DRV8313 EN 引脚控制。
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

/**
 * @brief   更新 TIM1 CCER 寄存器（六步换向用, 读-修改-写原子操作）
 * @param   clr_mask  CCER 位清零掩码（清除要禁能的通道使能位）
 * @param   set_mask  CCER 位置位掩码（设置要使能的通道使能位）
 * @note    CCER 直接更新用于六步换向中动态切换相导通状态。
 *          CCER 更新和 CCR 更新应在同一 ISR 周期内完成。
 */
void drv_tim_pwm_ccer_apply(uint32_t clr_mask, uint32_t set_mask);

/**
 * @brief   设置六步换向 EN 引脚（PC4=A, PC13=B, PC15=C）
 * @param   en_a / en_b / en_c  0 = 关断, 非 0 = 使能
 * @note    使用 GPIO BSRR 寄存器原子写: 低 16-bit 置位, 高 16-bit 复位。
 *          BSRR 的优点: 单次写操作同时完成置位和复位, 无中间态。
 *          避免了 Read-Modify-Write 可能引入的竞态条件。
 */
void drv_tim_pwm_en_pins_set(uint8_t en_a, uint8_t en_b, uint8_t en_c);

#ifdef __cplusplus
}
#endif

#endif /* DRV_TIM_PWM_H */
