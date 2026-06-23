/**
******************************************************************************
* @file     drv_tim_pwm.c
* @author   lidongyang
* @version  0.0.1
* @date     18-June-2026
* @brief    TIM1 六路互补 PWM 驱动实现
*
* @note     本驱动直接操作 TIM1 寄存器以实现高性能 PWM 控制：
*           - 直接写 CCR 寄存器设置占空比（比 HAL 快 ~100 倍）
*           - BDTR 死区配置支持调试模式下临时关闭死区
*           - MOE 紧急关断适用于 nFAULT 硬件保护路径
*
*           使用前需确保已通过 CubeMX MX_TIM1_Init() 完成时基/通道/GPIO 配置。
*           本驱动不会覆盖 CubeMX 已配好的 GPIO 复用功能、时基参数或通道极性。
******************************************************************************
*/

#include "drv_tim_pwm.h"

/** @brief 保存 TIM1 HAL 句柄, 供所有 API 函数使用 */
static TIM_HandleTypeDef *p_pwm_tim = NULL;

/*==========================================================================*/
/* 调试: 当前占空比（全局, ISR 写入, main 循环 printf 读取）                  */
/*==========================================================================*/

volatile uint16_t g_pwm_duty_a = 0U;
volatile uint16_t g_pwm_duty_b = 0U;
volatile uint16_t g_pwm_duty_c = 0U;

/*==========================================================================*/
/* 内部: 浮点 → CCR 转换                                                     */
/*==========================================================================*/

/**
 * @brief   浮点占空比（0.0~1.0）→ CCR 硬件比较值
 * @param   d   归一化占空比（0.0 = 下管全通, 1.0 = 上管全通）
 * @return  uint16_t  CCR 寄存器值（0 ~ PWM_DUTY_MAX）
 * @note    钳位处理: d < 0 → 0, d > 1 → 1, 防浮点误差异常值。
 */
static uint16_t duty_to_ccr(float d)
{
    if (d <= 0.0f)
    {
        return (uint16_t)PWM_DUTY_MIN;
    }
    if (d >= 1.0f)
    {
        return (uint16_t)PWM_DUTY_MAX;
    }
    return (uint16_t)(d * (float)PWM_DUTY_MAX);
}

/*==========================================================================*/
/* drv_tim_pwm_init                                                         */
/*==========================================================================*/

/**
 * @brief   PWM 驱动初始化
 *
 * @note    第 1 步: 死区处理。
 *          CubeMX 默认已配置 DTG=34（≈200ns）。
 *          若 PWM_DEADTIME_ENABLE=1: 保持 CubeMX 配置, 不做修改。
 *          若 PWM_DEADTIME_ENABLE=0: 清零 BDTR[7:0] 关闭死区（调试用）。
 *          不动 BDTR 高 8 位（MOE/BKE/AOE 等）, 保持 CubeMX 原有配置。
 *
 *          第 2 步: 设置三相初始占空比 = 50%。
 *          CCR = ARR / 2 = 2124（整数除法取整）。
 *          50% 时上下桥臂导通时间相等, 绕组两端平均电压为零 → 电机静止。
 *
 *          MISRA 注意: 位操作使用 &= ~(mask) 模式,
 *          避免直接赋值覆盖 BDTR 其他位。
 */
void drv_tim_pwm_init(TIM_HandleTypeDef *htim)
{
    p_pwm_tim = htim;

    /*
     * 第 1 步: 死区处理。
     *
     * CubeMX 默认 DTG=34 → 死区 ≈ 200ns（满足设计要求）。
     * 调试模式（PWM_DEADTIME_ENABLE=0）下清零 DTG 关闭死区,
     * 以排除死区对波形的干扰, 方便示波器排查。
     * 实际驱动电机时必须使能（PWM_DEADTIME_ENABLE=1）。
     */
#if (PWM_DEADTIME_ENABLE)
    /* 保持 CubeMX 默认死区配置 DTG=34 ≈ 200ns, 不做修改 */
#else
    {
        uint32_t bdtr;
        /* 读 BDTR → 清 DTG 位（低 8 位） → 写回（保留 MOE/BKE/AOE 等高位） */
        bdtr = htim->Instance->BDTR;
        bdtr &= ~(uint32_t)0xFFU;
        htim->Instance->BDTR = bdtr;
    }
#endif

    /*
     * 第 2 步: 三相初始占空比 = 50%。
     *
     * CCR = ARR / 2 = 4249 / 2 = 2124。
     * 此时上桥臂和下桥臂导通时间相等（各约 25µs/周期）,
     * 电机绕组两端平均电压为零, 无转矩输出。
     * 此状态为最安全的启动状态: 所有 MOSFET 处于平衡态,
     * 无直通风险, 无意外转矩。
     */
    drv_tim_pwm_set_duty((uint16_t)(PWM_DUTY_MAX / 2U),
                        (uint16_t)(PWM_DUTY_MAX / 2U),
                        (uint16_t)(PWM_DUTY_MAX / 2U));
}

/*==========================================================================*/
/* drv_tim_pwm_enable / drv_tim_pwm_disable / drv_tim_pwm_moe_off           */
/*==========================================================================*/

/**
 * @brief   使能 PWM 输出
 *
 * @note    启动顺序（必须严格按此顺序）:
 *          1. HAL_TIM_PWM_Start      — 置 CCxE（主通道输出使能）
 *          2. HAL_TIMEx_PWMN_Start   — 置 CCxNE（互补通道输出使能, 若使能）
 *          3. __HAL_TIM_ENABLE       — 置 CEN（计数器开始运行）
 *          4. SET_BIT BDTR MOE       — 总闸打开, PWM 信号到达 GPIO 引脚
 *
 *          若 MOE 在 CEN=1 之前置位, TIM1 可能输出异常脉冲（手册明确要求顺序）。
 *          MOE = 1 后, 六路 PWM 立即输出当前 CCR 决定的波形。
 *
 *          互补通道（N 通道）启用后, CHx 和 CHxN 为互补信号（带死区插入）,
 *          适用于驱动上下桥臂 MOSFET。
 */
void drv_tim_pwm_enable(void)
{
    (void)HAL_TIM_PWM_Start(p_pwm_tim, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(p_pwm_tim, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(p_pwm_tim, TIM_CHANNEL_3);
#if (PWM_ENABLE_N_CHANNELS)
    (void)HAL_TIMEx_PWMN_Start(p_pwm_tim, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(p_pwm_tim, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(p_pwm_tim, TIM_CHANNEL_3);
#endif

    __HAL_TIM_ENABLE(p_pwm_tim);
    SET_BIT(p_pwm_tim->Instance->BDTR, TIM_BDTR_MOE);
}

/**
 * @brief   紧急关断 MOE（仅清除总输出使能, ISR 安全, 硬件级快速关断）
 * @note    MOE = 0 后, 所有 PWM 输出立即进入空闲态（配置为 LOW = GPIO 输出低）。
 *          不停止定时器 / 通道, 不清除 CCR 值。
 *          故障恢复后只需重新置位 MOE 即可恢复 PWM 输出。
 *
 *          此函数在 nFAULT ISR 中调用:
 *          EXTI15_10 → app_isr_nfault() → drv_tim_pwm_moe_off()
 *          总延迟 < 1µs（GPIO 中断延迟 12 周期 ≈ 70ns + MOE 清零 1 周期）。
 */
void drv_tim_pwm_moe_off(void)
{
    CLEAR_BIT(p_pwm_tim->Instance->BDTR, TIM_BDTR_MOE);
}

/**
 * @brief   紧急关断 PWM（MOE + 计数器全停）
 * @note    顺序与 enable 严格对称:
 *          1. CLEAR_BIT MOE      — MOSFET 全部关断（最快保护）
 *          2. __HAL_TIM_DISABLE  — 计数器停
 *          3. HAL_TIM_PWM_Stop   — 清 CCxE / CCxNE
 *
 *          比 drv_tim_pwm_moe_off 更彻底, 计数器也停掉。
 *          用于严重故障（如连续过流、长时间未恢复）。
 */
void drv_tim_pwm_disable(void)
{
    CLEAR_BIT(p_pwm_tim->Instance->BDTR, TIM_BDTR_MOE);
    __HAL_TIM_DISABLE(p_pwm_tim);

    (void)HAL_TIM_PWM_Stop(p_pwm_tim, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(p_pwm_tim, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(p_pwm_tim, TIM_CHANNEL_3);
#if (PWM_ENABLE_N_CHANNELS)
    (void)HAL_TIMEx_PWMN_Stop(p_pwm_tim, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(p_pwm_tim, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(p_pwm_tim, TIM_CHANNEL_3);
#endif
}

/*==========================================================================*/
/* drv_tim_pwm_set_duty                                                     */
/*==========================================================================*/

/**
 * @brief   设置三相占空比（整数）
 * @param   a/b/c  A/B/C 相 CCR 值, 范围 [0, PWM_DUTY_MAX]
 *
 * @note    钳位逻辑:
 *          - 若 PWM_DUTY_MIN > 0: 下界钳位（防止 CCR=0 在某些模式下异常）
 *          - 上界钳位: 防止 CCR > ARR（写入 ARR 以上的值会被硬件截断）
 *
 *          使用 __HAL_TIM_SET_COMPARE 宏直接写 CCR 寄存器:
 *            单条 STR 汇编指令, 约 1~2 个 CPU 周期 @ 170MHz。
 *          HAL 等效函数 HAL_TIM_PWM_ConfigChannel 开销 ~100 周期（多次检查 + 多寄存器写）。
 *
 *          ISR 中使用 __HAL_TIM_SET_COMPARE 是安全的:
 *          CCR 寄存器支持 16-bit 原子写（硬件保证完整性）。
 *
 *          调试变量 g_pwm_duty_a/b/c 更新:
 *          供 main 循环中 printf 读取, 不影响 ISR 实时性。
 */
void drv_tim_pwm_set_duty(uint16_t a, uint16_t b, uint16_t c)
{
#if (PWM_DUTY_MIN > 0U)
    if (a < PWM_DUTY_MIN) { a = PWM_DUTY_MIN; }
    if (b < PWM_DUTY_MIN) { b = PWM_DUTY_MIN; }
    if (c < PWM_DUTY_MIN) { c = PWM_DUTY_MIN; }
#endif
    if (a > PWM_DUTY_MAX) { a = PWM_DUTY_MAX; }
    if (b > PWM_DUTY_MAX) { b = PWM_DUTY_MAX; }
    if (c > PWM_DUTY_MAX) { c = PWM_DUTY_MAX; }

    /* 直接写 TIM1 CCR 比较寄存器（无 HAL 开销） */
    __HAL_TIM_SET_COMPARE(p_pwm_tim, TIM_CHANNEL_1, a);
    __HAL_TIM_SET_COMPARE(p_pwm_tim, TIM_CHANNEL_2, b);
    __HAL_TIM_SET_COMPARE(p_pwm_tim, TIM_CHANNEL_3, c);

    /* 更新调试变量（main 循环 printf 可用） */
    g_pwm_duty_a = a;
    g_pwm_duty_b = b;
    g_pwm_duty_c = c;
}

/**
 * @brief   设置三相占空比（浮点）
 * @param   a   A 相占空比, 范围 0.0 ~ 1.0（0.0 = 下管全通, 1.0 = 上管全通）
 * @param   b   B 相占空比, 范围 0.0 ~ 1.0
 * @param   c   C 相占空比, 范围 0.0 ~ 1.0
 * @note    浮点 → 整数转换通过 duty_to_ccr 完成（含钳位）。
 *          float × 4249 → uint16_t, 精度足够（CCR 误差 < 2）。
 */
void drv_tim_pwm_set_duty_f(float a, float b, float c)
{
    drv_tim_pwm_set_duty(duty_to_ccr(a), duty_to_ccr(b), duty_to_ccr(c));
}
