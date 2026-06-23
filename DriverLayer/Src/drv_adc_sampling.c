/**
******************************************************************************
* @file     drv_adc_sampling.c
* @author   lidongyang
* @version  0.0.1
* @date     17-June-2026
* @brief    ADC1/ADC2 电流/电压检测驱动实现
*
* @note     CubeMX 已完成 ADC GPIO + DMA + NVIC 配置, 本驱动仅负责:
*           - ADC 自校准
*           - 启动 ADC + DMA + 定时器
*           - 在 HAL_ADC_ConvCpltCallback() 中处理采样数据
******************************************************************************
*/

#include "drv_adc_sampling.h"

/*==========================================================================*/
/* ADC 转换完成回调（供 FOC 等控制环路注册）                                   */
/*==========================================================================*/

static DRV_ADC_ConvCplt_Callback_t s_adc_conv_cplt_cb = NULL;

void drv_adc_register_conv_cplt_callback(DRV_ADC_ConvCplt_Callback_t cb)
{
    s_adc_conv_cplt_cb = cb;
}

/*==========================================================================*/
/* DMA 缓冲区（循环模式, 由 DMA 自动填充）                                   */
/*==========================================================================*/

uint16_t adc1_dma_buf[2U] = {0U, 0U};  /**< [Ia_raw(IN1), Ib_raw(IN2)]      */
uint16_t adc2_dma_buf[2U] = {0U, 0U};  /**< [母线电流(IN3), 母线电压(IN4)]  */

/*==========================================================================*/
/* 计算结果（volatile: ISR 回调写入, main 循环读取）                         */
/*==========================================================================*/

volatile float g_curr_ia = 0.0f;  /**< A 相电流 (A)     */
volatile float g_curr_ib = 0.0f;  /**< B 相电流 (A)     */
volatile float g_bus_cur = 0.0f;  /**< 母线输入电流 (A) */
volatile float g_bus_vol = 0.0f;  /**< 直流母线电压 (V) */

/** @brief IIR 滤波状态（一阶低通: y[n] = α*x[n] + (1-α)*y[n-1]） */
static float s_filt_ia = 0.0f;
static float s_filt_ib = 0.0f;


/*==========================================================================*/
/* drv_adc_sampling_init                                                    */
/*==========================================================================*/

/**
 * @brief   ADC 电流/电压检测初始化
 * @param   hadc1  ADC1 HAL 句柄
 * @param   hadc2  ADC2 HAL 句柄
 * @param   htim1  TIM1 HAL 句柄（ADC1 触发源）
 * @param   htim6  TIM6 HAL 句柄（ADC2 触发源）
 *
 * @note    务必在 MX_ADC1_Init、MX_ADC2_Init、MX_TIM6_Init 之后调用。
 *          执行顺序: 自校准 → 启动 ADC DMA → 使能 TIM6 → 使能 TIM1。
 */
void drv_adc_sampling_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                           TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim6)
{
    /* ---- 1. ADC 自校准（STM32G4 上电后精度保证的必要步骤） ---- */
    (void)HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED);

    /* ---- 2. 启动 ADC DMA（循环模式, 无需 CPU 干预） ---- */
    (void)HAL_ADC_Start_DMA(hadc1, (uint32_t *)adc1_dma_buf, 2U);
    (void)HAL_ADC_Start_DMA(hadc2, (uint32_t *)adc2_dma_buf, 2U);

    /* ---- 3-4. 启动定时器 + 更新中断 ----
     *       TIM6 先启动（1kHz TRGO → ADC2；更新中断 → 速度/位置环）
     *       TIM1 后启动（20kHz TRGO → ADC1；更新中断 → 调试/时序测量） */
    __HAL_TIM_ENABLE_IT(htim6, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(htim6);

    __HAL_TIM_ENABLE_IT(htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(htim1);
}

/*==========================================================================*/
/* HAL_ADC_ConvCpltCallback（ADC 转换完成 HAL 回调）                         */
/*==========================================================================*/

/**
 * @brief   ADC 转换完成回调（HAL_DMA_IRQHandler → HAL_ADC_ConvCpltCallback）
 * @param   hadc  HAL ADC 句柄（用于判断 ADC1 或 ADC2）
 *
 * @note    ADC1 每 50µs（20kHz）触发, ADC2 每 1ms（1kHz）触发。
 *          回调运行在 DMA 中断上下文（优先级 4）。
 *          int32_t 中间变量防 uint16_t 减法负值溢出。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    int32_t offset;

    if (hadc->Instance == ADC1) {
        /* ---- ADC1: 相电流 Ia(PA0/IN1), Ib(PA1/IN2) ---- */

        /* Ia = (raw[0] - 2048) × 0.004028 */
        offset    = (int32_t)adc1_dma_buf[0U] - (int32_t)ADC_CURR_OFFSET;
        g_curr_ia = (float)offset * ADC_CURR_SCALE;

        /* Ib = (raw[1] - 2048) × 0.004028 */
        offset    = (int32_t)adc1_dma_buf[1U] - (int32_t)ADC_CURR_OFFSET;
        g_curr_ib = (float)offset * ADC_CURR_SCALE;

        /* 一阶 IIR 低通滤波: fc≈1.1kHz, 抑制传感器噪声 + PWM 纹波 */
        s_filt_ia = ADC_CURR_FILT_ALPHA * g_curr_ia + (1.0f - ADC_CURR_FILT_ALPHA) * s_filt_ia;
        s_filt_ib = ADC_CURR_FILT_ALPHA * g_curr_ib + (1.0f - ADC_CURR_FILT_ALPHA) * s_filt_ib;
        g_curr_ia = s_filt_ia;
        g_curr_ib = s_filt_ib;

        /* 调用注册的控制回调（如 FOC_xxx_Run）——
         * 相电流数据已就绪, 是执行控制环路的最佳时机 */
        if (s_adc_conv_cplt_cb != NULL) {
            s_adc_conv_cplt_cb();
        }

        /* ---- FOC 执行时间测量（PA4 脉冲宽度 = 耗时） ---- */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

        /* ---- ADC 转换时间测量（注释掉, 测试时启用） ----
         * HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
         * TIM1 更新 ISR 拉高 PA4 → ADC 转换完成拉低 → 示波器测脉宽
         * 测试结果: 约 6µs */

    } else if (hadc->Instance == ADC2) {
        /* ---- ADC2: 母线电流(PA6/IN3), 母线电压(PA7/IN4) ---- */

        /* 母线电流（同 TMCS1107A3B 电流公式） */
        offset    = (int32_t)adc2_dma_buf[0U] - (int32_t)ADC_CURR_OFFSET;
        g_bus_cur = (float)offset * ADC_CURR_SCALE;

        /* 母线电压: 无需零偏修正, Vbus = raw × 0.007381 */
        g_bus_vol = (float)adc2_dma_buf[1U] * ADC_BUS_VOLT_SCALE;

        /* ---- ADC 转换时间测量（注释掉, 测试时启用） ----
         * HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
         * TIM6 更新 ISR 拉高 PA4 → ADC 转换完成拉低 → 示波器测脉宽
         * 测试结果: 约 6µs */

    } else {
        /* 其他 ADC 实例不处理（预留 ADC3/ADC4） */
    }
}
