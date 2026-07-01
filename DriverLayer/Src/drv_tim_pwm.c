/**
******************************************************************************
* @file     drv_tim_pwm.c
* @author   lidongyang
* @version  0.0.1
* @date     18-June-2026
* @brief    TIM1 六路互补 PWM 驱动实现
*
* @note     设计决策:
*           1. 直接写 CCR 寄存器设置占空比（单条 STR 指令, ~2 CPU 周期,
*              比 HAL_TIM_PWM_ConfigChannel 快 ~100 倍）
*           2. BDTR 死区配置支持调试模式下临时关闭死区
*           3. MOE 紧急关断适用于 nFAULT 硬件保护路径
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
 * @brief   浮点占空比 [0.0, 1.0] → CCR 寄存器值
 * @param   d   归一化占空比（0.0=下管全通, 1.0=上管全通）
 * @return  CCR 寄存器值 [0, PWM_DUTY_MAX]
 * @note    钳位: d<0→0, d>1→1, 防浮点误差异常值
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
 * @param   htim  已由 CubeMX 初始化的 TIM_HandleTypeDef 指针
 *
 * @note    两步初始化:
 *
 *          第 1 步 - 死区处理:
 *          若 PWM_DEADTIME_ENABLE=1: 保持 CubeMX 默认 DTG=34（≈200ns）。
 *          若 PWM_DEADTIME_ENABLE=0: 清零 BDTR[7:0] 关闭死区（调试用）。
 *
 *          第 2 步 - 三相初始占空比 = 50%:
 *          CCR = ARR/2 = 2124, 绕组两端平均电压为零, 电机静止。
 */
void drv_tim_pwm_init(TIM_HandleTypeDef *htim)
{
    p_pwm_tim = htim;

    /* ---- 第 1 步: 死区处理 ----
     * CubeMX 默认 DTG=34 → 死区 ≈ 200ns。
     * 调试模式（PWM_DEADTIME_ENABLE=0）清零 DTG 关闭死区,
     * 排除死区对波形的干扰, 方便示波器排查。
     * 实际驱动电机时必须使能（PWM_DEADTIME_ENABLE=1）。 */
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

    /* ---- 第 2 步: 三相初始占空比 = 50% ----
     * CCR = ARR/2 = 4249/2 = 2124。
     * 上下桥臂导通时间相等（各约 25µs/周期）,
     * 绕组两端平均电压为零, 电机静止。
     * 所有 MOSFET 处于平衡态, 无直通风险, 无意外转矩。 */
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
 *          若 MOE 在 CEN=1 之前置位, TIM1 可能输出异常脉冲。
 *          MOE=1 后, 六路 PWM 立即输出当前 CCR 决定的波形。
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
 * @brief   紧急关断 MOE（仅清除总输出使能, ISR 安全）
 * @note    MOE=0 → 所有 PWM 立即进空闲态（LOW）。
 *          不停止定时器/通道, 不清 CCR。故障恢复后重设 MOE 即可恢复。
 *          此函数在 nFAULT ISR（EXTI15_10）中调用, 总延迟 < 1µs。
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
 *          3. HAL_TIM_PWM_Stop   — 清 CCxE/CCxNE
 *          用于严重故障（连续过流、长时间未恢复）。
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
 * @param   a  A 相 CCR 值, 范围 [PWM_DUTY_MIN, PWM_DUTY_MAX]
 * @param   b  B 相 CCR 值
 * @param   c  C 相 CCR 值
 *
 * @note    上下界钳位后直接写 CCR 寄存器（__HAL_TIM_SET_COMPARE,
 *          单条 STR 指令, ~2 CPU 周期 @ 170MHz）。
 *          HAL 等效调用 ~100 周期（多次检查 + 多寄存器写）。
 *          CCR 寄存器 16-bit 原子写, ISR 中安全。
 */
void drv_tim_pwm_set_duty(uint16_t a, uint16_t b, uint16_t c)
{
#if (PWM_DUTY_MIN > 0U)
    if (a < PWM_DUTY_MIN)
    {
        a = PWM_DUTY_MIN;
    }
    if (b < PWM_DUTY_MIN)
    {
        b = PWM_DUTY_MIN;
    }
    if (c < PWM_DUTY_MIN)
    {
        c = PWM_DUTY_MIN;
    }
#endif
    if (a > PWM_DUTY_MAX)
    {
        a = PWM_DUTY_MAX;
    }
    if (b > PWM_DUTY_MAX)
    {
        b = PWM_DUTY_MAX;
    }
    if (c > PWM_DUTY_MAX)
    {
        c = PWM_DUTY_MAX;
    }

    /* 直接写 TIM1 CCR 寄存器（无 HAL 开销） */
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
 * @param   a  A 相占空比 [0.0, 1.0]（0.0=下管全通, 1.0=上管全通）
 * @param   b  B 相占空比
 * @param   c  C 相占空比
 * @note    浮点 → 整数转换通过 duty_to_ccr 完成（含钳位）
 */
void drv_tim_pwm_set_duty_f(float a, float b, float c)
{
    drv_tim_pwm_set_duty(duty_to_ccr(a), duty_to_ccr(b), duty_to_ccr(c));
}
