/**
******************************************************************************
* @file     drv_adc_sampling.h
* @author   lidongyang
* @version  0.0.1
* @date     17-June-2026
* @brief    ADC1/ADC2 电流/电压检测驱动
*           电流传感器: TMCS1107A3B（200mV/A, 零电流 = Vcc/2 = 1.65V）
*
* @note     ADC1（TIM1_TRGO 触发, 20kHz）: 相电流 Ia(PA0/IN1) + Ib(PA1/IN2)
*           ADC2（TIM6_TRGO 触发, 1kHz）:  母线电流(PA6/IN3) + 母线电压(PA7/IN4)
*           ADC 时钟 = SYSCLK/4 = 170MHz/4 = 42.5MHz
*           分辨率: 12-bit 右对齐, 结果范围 0~4095
******************************************************************************
*/

#ifndef DRV_ADC_SAMPLING_H
#define DRV_ADC_SAMPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/*==========================================================================*/
/* 电流 / 电压转换参数                                                       */
/*==========================================================================*/

/** @brief ADC 中位值（零电流时的 ADC 读数）
 *  @note  TMCS1107A3B 零电流输出 = Vcc/2 = 1.65V
 *         1.65V / 3.3V × 4096 = 2048 */
#define ADC_CURR_OFFSET    2048U

/** @brief TMCS1107A3B 电流转换系数 (A / ADC LSB)
 *  @note  Vref = VDDA = 3.3V, 灵敏度 S = 200mV/A
 *         刻度 = (Vref/2^N) / S = (3.3/4096) / 0.2 ≈ 0.004028 A/LSB
 *         验证: ADC = 2048 + 248（1A） → I = 248 × 0.004028 ≈ 1.0A ✓ */
#define ADC_CURR_SCALE     0.004028f

/** @brief 母线电压转换系数 (V / ADC LSB)
 *  @note  分压比 K ≈ 9.16（Vbus=12V → V_ADC≈1.31V）
 *         刻度 = (Vref/2^N) × K = (3.3/4096) × 9.16 ≈ 0.007381 V/LSB
 *         分压电阻 ±1% 容差, 需实测校准 K 值 */
#define ADC_BUS_VOLT_SCALE 0.007381f

/** @brief 电流 IIR 低通滤波系数 (0, 1]
 *  @note  α=1.0 无滤波, α=0.3 约 1.1kHz 截止频率 @20kHz 采样
 *         增大 α → 响应更快但噪声更多, 减小 α → 更平滑但相位滞后更大 */
#define ADC_CURR_FILT_ALPHA  0.3f

/*==========================================================================*/
/* DMA 缓冲区（各 2 通道，循环模式）                                         */
/*==========================================================================*/

/** @brief ADC1 DMA 循环缓冲: [Ia_raw, Ib_raw]
 *  @note  DMA 循环模式, 无需 CPU 干预。IN1(PA0)=Ia, IN2(PA1)=Ib */
extern uint16_t adc1_dma_buf[2U];

/** @brief ADC2 DMA 循环缓冲: [bus_current_raw, bus_voltage_raw]
 *  @note  IN3(PA6)=母线电流, IN4(PA7)=母线电压 */
extern uint16_t adc2_dma_buf[2U];

/*==========================================================================*/
/* 电流 / 电压计算结果（float, 单位 A / V）                                  */
/*==========================================================================*/

/** @brief ADC 计算结果（ISR 内更新, main 循环读取）
 *  @note  volatile + float, Cortex-M4 FPU 单周期加载 */
extern volatile float g_curr_ia;    /**< A 相电流 (A)     */
extern volatile float g_curr_ib;    /**< B 相电流 (A)     */
extern volatile float g_bus_cur;    /**< 母线输入电流 (A) */
extern volatile float g_bus_vol;    /**< 直流母线电压 (V) */

/*==========================================================================*/
/* ADC 转换完成回调注册（供 FOC 等控制环路使用）                                */
/*==========================================================================*/

/** @brief ADC 转换完成回调函数类型（ADC1 完成时调用，运行在中断上下文） */
typedef void (*DRV_ADC_ConvCplt_Callback_t)(void);

/**
 * @brief   注册 ADC1 转换完成回调
 * @note    ADC1 每 50µs（20kHz）完成一次双通道转换（Ia, Ib），
 *          回调在 HAL_ADC_ConvCpltCallback 中执行，数据已就绪。
 *          典型用途: FOC 电流环（保证相电流采样数据完整性）。
 * @param   cb  回调函数指针，传 NULL 取消注册
 */
void drv_adc_register_conv_cplt_callback(DRV_ADC_ConvCplt_Callback_t cb);

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   ADC 电流/电压检测初始化
 * @param   hadc1  ADC1 HAL 句柄（CubeMX 已填充时基/通道/DMA 配置）
 * @param   hadc2  ADC2 HAL 句柄
 * @param   htim1  TIM1 HAL 句柄（ADC1 外部触发源）
 * @param   htim6  TIM6 HAL 句柄（ADC2 外部触发源）
 *
 * @note    必须在 MX_ADC1_Init / MX_ADC2_Init / MX_TIM6_Init 之后调用。
 *          初始化顺序（严格按此顺序, 否则 ADC 触发时序异常）:
 *          1. ADC1/ADC2 自校准（上电必须, 否则精度显著下降）
 *          2. 启动 ADC1 DMA + ADC2 DMA（循环模式）
 *          3. 使能 TIM6 + TIM1 计数器（开始产生 TRGO 触发 ADC）
 */
void drv_adc_sampling_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                           TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim6);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ADC_SAMPLING_H */
