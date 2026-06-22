/**
******************************************************************************
* @file     drv_tim_pwm.c
* @author   lidongyang
* @version  0.0.1
* @date     18-June-2026
* @brief    TIM1 六路互补 PWM 驱动实现
*
* @note     
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
 * @note    第 1 步: 修复死区时间。
 *          CubeMX 默认配置 DTG=1（约 6ns）, 远小于 200ns 设计要求。
 *          重写 BDTR[7:0]（DTG 位）为 34 或 0（取决于 PWM_DEADTIME_ENABLE）。
 *          不动 BDTR 高 8 位（MOE/BKE/AOE 等）, 保持 CubeMX 原有配置。
 *
 *          第 2 步: 设置三相初始占空比 = 50%。
 *          CCR = ARR / 2 = 2124（整数除法取整）。
 *          50% 时上下桥臂导通时间相等, 绕组两端平均电压为零 → 电机静止。
 *
 *          MISRA 注意: 位操作使用 &= ~(mask) | (value) 模式,
 *          避免直接赋值覆盖 BDTR 其他位。
 */
void drv_tim_pwm_init(TIM_HandleTypeDef *htim)
{
    uint32_t bdtr;

    p_pwm_tim = htim;

    /* 读 BDTR → 清 DTG 位（低 8 位） → 写入 DTG 新值 */
    bdtr = htim->Instance->BDTR;
    bdtr &= ~(uint32_t)0xFFU;
#if (PWM_DEADTIME_ENABLE)
    bdtr |= (uint32_t)PWM_DTG_200NS;  /* DTG = 34, DT ≈ 200ns */
#else
    bdtr |= 0U;                       /* DTG = 0,  无死区（调试用） */
#endif
    htim->Instance->BDTR = bdtr;

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
 * @param   a/b/c  占空比, 0.0 ~ 1.0
 * @note    浮点 → 整数转换通过 duty_to_ccr 完成（含钳位）。
 *          float × 4249 → uint16_t, 精度足够（CCR 误差 < 2）。
 */
void drv_tim_pwm_set_duty_f(float a, float b, float c)
{
    drv_tim_pwm_set_duty(duty_to_ccr(a), duty_to_ccr(b), duty_to_ccr(c));
}

/*==========================================================================*/
/* drv_tim_pwm_ccer_apply — TIM1 CCER 原子更新（六步换相动态通道控制）        */
/*==========================================================================*/

/**
 * @brief   更新 TIM1 CCER 寄存器（读-修改-写, 非原子硬件操作）
 * @param   clr_mask  要清零的位（CCxE / CCxNE 禁能）
 * @param   set_mask  要置位的位（CCxE / CCxNE 使能）
 *
 * @note    CCER (Capture/Compare Enable Register) 控制每个通道的使能:
 *          CCER[0]  = CC1E  (CH1 输出使能)
 *          CCER[2]  = CC1NE (CH1N 互补输出使能)
 *          CCER[4]  = CC2E
 *          CCER[6]  = CC2NE
 *          CCER[8]  = CC3E
 *          CCER[10] = CC3NE
 *
 *          六步换向中使用:
 *          电角度 0~60°（扇区 1）: A 相 PWM, B 相 Hi-Z, C 相 ON
 *            → CC1E=CC1NE=1（A 使能）, CC2E=CC2NE=0（B 禁能）, CC3E=0, CC3NE=1（C 下管）
 *
 *          CCER 更新 + EN 引脚控制 → 六步换向完整的 MOSFET 开关控制。
 *          CCER 更新在同一个 ISR 中紧跟 duty 更新, 确保换相时刻一致。
 *
 *          MISRA 注意: 读-修改-写操作在 ISR 上下文中是安全的（单线程）。
 *          clr_mask 和 set_mask 不应有交集, 避免清零后又被置位。
 */
void drv_tim_pwm_ccer_apply(uint32_t clr_mask, uint32_t set_mask)
{
    uint32_t ccer = TIM1->CCER;
    ccer &= ~clr_mask;
    ccer |= set_mask;
    TIM1->CCER = ccer;
}

/*==========================================================================*/
/* drv_tim_pwm_en_pins_set — 六步换向 EN 引脚（GPIO BSRR 原子写）            */
/*==========================================================================*/

/**
 * @brief   设置六步换向 EN 引脚
 * @param   en_a  A 相 EN（0 = PC4 低, 1 = PC4 高）
 * @param   en_b  B 相 EN（0 = PC13 低, 1 = PC13 高）
 * @param   en_c  C 相 EN（0 = PC15 低, 1 = PC15 高）
 *
 * @note    使用 GPIO BSRR 寄存器原子操作:
 *          BSRR[15:0]  写 1 → 对应引脚置高（Set）
 *          BSRR[31:16] 写 1 → 对应引脚置低（Reset）
 *
 *          例: en_a=1, en_b=1, en_c=0:
 *            BSRR = PIN4 | PIN13 | (PIN15 << 16)
 *            结果: PC4→高, PC13→高, PC15→低（单次写操作, 原子完成）
 *
 *          BSRR 的优点:
 *          - 原子性: 单条 32-bit STR 指令, 无法被中断打断
 *          - 确定性: 置位位和复位位同时写入, 无中间态
 *          - 安全性: 不用 Read-Modify-Write, 避免 OD/ID 寄存器竞态
 *
 *          EN 引脚分配:
 *            DRV8313_EN_A  = PC4   → GPIO_PIN_4
 *            SIXSTEP_ENB   = PC13  → GPIO_PIN_13
 *            SIXSTEP_ENC   = PC15  → GPIO_PIN_15
 */
void drv_tim_pwm_en_pins_set(uint8_t en_a, uint8_t en_b, uint8_t en_c)
{
    uint32_t bsrr = 0U;
    if (en_a) { bsrr |= GPIO_PIN_4;  } else { bsrr |= (GPIO_PIN_4  << 16U); }
    if (en_b) { bsrr |= GPIO_PIN_13; } else { bsrr |= (GPIO_PIN_13 << 16U); }
    if (en_c) { bsrr |= GPIO_PIN_15; } else { bsrr |= ((uint32_t)GPIO_PIN_15 << 16U); }
    GPIOC->BSRR = bsrr;
}
