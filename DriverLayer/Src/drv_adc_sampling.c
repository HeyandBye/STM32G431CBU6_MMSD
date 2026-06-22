/**
******************************************************************************
* @file     drv_adc_sampling.c
* @author   lidongyang
* @version  0.0.1
* @date     17-June-2026
* @brief    ADC1/ADC2 电流/电压检测驱动实现
*
* @note     CubeMX 已完成 ADC GPIO + DMA + NVIC 配置,
*           本驱动仅负责：
*           1. ADC 自校准
*           2. 启动 ADC + DMA + 定时器
*           3. 在 HAL_ADC_ConvCpltCallback() 中处理采样数据
******************************************************************************
*/

#include "drv_adc_sampling.h"

/*==========================================================================*/
/* DMA 缓冲区（循环模式, 由 DMA 自动填充）                                   */
/*==========================================================================*/

uint16_t adc1_dma_buf[2U] = {0U, 0U};  /* [Ia_raw(IN1), Ib_raw(IN2)]        */
uint16_t adc2_dma_buf[2U] = {0U, 0U};  /* [母线上电流(IN3), 母线电压(IN4)]   */

/*==========================================================================*/
/* 计算结果（volatile: ISR 回调写入, main 循环读取）                         */
/*==========================================================================*/

volatile float g_curr_ia = 0.0f;  /*  A 相电流 (A)                          */
volatile float g_curr_ib = 0.0f;  /*  B 相电流 (A)                          */
volatile float g_bus_cur = 0.0f;  /*  母线输入电流 (A)                       */
volatile float g_bus_vol = 0.0f;  /*  直流母线电压 (V)                       */


/*==========================================================================*/
/* drv_adc_sampling_init                                                    */
/*==========================================================================*/

/**
 * @brief   ADC 电流/电压检测初始化
 *
 * @note    务必在 MX_ADC1_Init、MX_ADC2_Init、MX_TIM6_Init 之后调用。
 *          执行顺序:
 *          1. ADC1 + ADC2 自校准（补偿内部采样电容差异）
 *          2. 启动 ADC1 DMA + ADC2 DMA（循环模式, 连续采集）
 *          3. 使能 TIM6 计数器（开始产生 1kHz TRGO → ADC2）
 *          4. 使能 TIM1 计数器（开始产生 20kHz TRGO → ADC1）
 *
 *          TIM1 RCR 计算公式:
 *            f_TRGO = (2 × f_PWM) / (RCR + 1)
 *            中心对齐模式: 每 PWM 周期 2 次更新（峰顶 + 谷底）
 *            RCR = 1 → f_TRGO = 40000 / 2 = 20000 Hz
 *            RCR = 0 → f_TRGO = 40000 / 1 = 40000 Hz
 *
 *          STM32G4 ADC 自校准:
 *            每次上电后必须执行, 校准内部比较器和采样电容的加工偏差。
 *            不校准会导致增益误差 ±5%（G4 典型值 12-bit + 自校准后误差 < ±2 LSB）。
 *            校准时间: 约 82 个 ADC 时钟周期 ≈ 2µs @ 42.5MHz.
 */
 void drv_adc_sampling_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                           TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim6)
{
    /* 1. ADC 自校准（STM32G4 上电后精度保证的必要步骤） */
    (void)HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED);

    /* 2. 启动 ADC DMA（循环模式: 转换完成后自动重新开始, 无需 CPU 干预） */
    (void)HAL_ADC_Start_DMA(hadc1, (uint32_t *)adc1_dma_buf, 2U);
    (void)HAL_ADC_Start_DMA(hadc2, (uint32_t *)adc2_dma_buf, 2U);

    /* 3-4. 使能定时器（TIM6 先启动, TIM1 后启动:
           确保 PWM 开始前母线电压/电流采集已就绪） */
    __HAL_TIM_ENABLE_IT(htim6, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(htim6);

    __HAL_TIM_ENABLE_IT(htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(htim1);
}

/*==========================================================================*/
/* HAL_ADC_ConvCpltCallback（ADC 转换完成 HAL 回调）                         */
/*==========================================================================*/

/**
 * @brief   ADC 转换完成回调（由 HAL_DMA_IRQHandler → HAL_ADC_ConvCpltCallback 链调用）
 *
 * @note    ADC1 每 ~50µs（20kHz）触发一次, ADC2 每 ~1ms（1kHz）触发一次。
 *          回调运行在 DMA 中断上下文中（优先级 = 4, 低于 TIM1 ISR 和 SPI DMA）。
 *
 *          int32_t 中间变量原理:
 *          当传感器输出 < 1.65V（负向电流）时, ADC_raw < 2048,
 *          uint16_t - uint16_t 会产生模运算下溢（uint16_t 无符号, 不能表示负值）。
 *          使用 int32_t 转换后可正确表示负差值。
 *
 *          例: raw = 1500（~1.21V）, 零偏 = 2048
 *            uint16_t: 1500 - 2048 = 溢出为 60000+  → 错误!
 *            int32_t:  1500 - 2048 = -548            → 正确!
 *            I = -548 × 0.004028 = -2.21A            → 正确!
 *
 *          MISRA 注意:
 *          显式 (int32_t) 类型转换, 符合 MISRA C 2023 Rule 10.3。
 *          不使用 volatile 变量的多次读取（单次读取存入局部变量 offset）。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    int32_t offset;

    if (hadc->Instance == ADC1) {
        /* ---- ADC1: 相电流 Ia (PA0/IN1), Ib (PA1/IN2) ---- */

        /* Ia = (raw[0] - 2048) × 0.004028 */
        offset    = (int32_t)adc1_dma_buf[0U] - (int32_t)ADC_CURR_OFFSET;
        g_curr_ia = (float)offset * ADC_CURR_SCALE;

        /* Ib = (raw[1] - 2048) × 0.004028 */
        offset    = (int32_t)adc1_dma_buf[1U] - (int32_t)ADC_CURR_OFFSET;
        g_curr_ib = (float)offset * ADC_CURR_SCALE;

        /* 下面是用于查看ADC转换时间的语句 */
        /* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); */
        /* TIM1 更新 ISR 触发时会先将 PA4 拉高, ADC 转换完成后在这里拉低,
           通过示波器观察 PA4 波形即可测量 ADC 转换时间（从 TIM1 更新到 ADC 转换完成的时间）*/
        /* 测试结果耗时约为 6 µs */

    } else if (hadc->Instance == ADC2) {
        /* ---- ADC2: 母线电流 (PA6/IN3, 同 TMCS1107A3B), 母线电压 (PA7/IN4) ---- */

        /* 母线电流（同 TMCS1107A3B 电流公式） */
        offset    = (int32_t)adc2_dma_buf[0U] - (int32_t)ADC_CURR_OFFSET;
        g_bus_cur = (float)offset * ADC_CURR_SCALE;

        /* 母线电压: 无需零偏修正, 直接 scale */
        /* Vbus = raw × (Vref/4096) × K = raw × 0.007381 */
        g_bus_vol = (float)adc2_dma_buf[1U] * ADC_BUS_VOLT_SCALE;

        /* 下面是用于查看ADC转换时间的语句 */
        /* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); */
        /* TIM6 更新 ISR 触发时会先将 PA4 拉高, ADC 转换完成后在这里拉低,
           通过示波器观察 PA4 波形即可测量 ADC 转换时间（从 TIM6 更新到 ADC 转换完成的时间）*/
        /* 测试结果耗时约为 6 µs */

    } else {
        /* 其他 ADC 实例, 不处理（如将来添加的 ADC3/ADC4） */
    }
}
