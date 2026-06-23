/**
******************************************************************************
* @file     drv_adc_sampling.h
* @author   lidongyang
* @version  0.0.1
* @date     17-June-2026
* @brief    ADC1/ADC2 电流/电压检测驱动
*
* @attention    ADC1（TIM1_TRGO 触发, 20 kHz）: 相电流 Ia(PA0/IN1) + Ib(PA1/IN2)
*               ADC2（TIM6_TRGO 触发, 1 kHz）:  母线电流(PA6/IN3) + 母线电压(PA7/IN4)
*               ADC 时钟 = SYSCLK/4 = 170MHz/4 = 42.5 MHz
*               分辨率：12-bit 右对齐，结果范围 0~4095
*
*               === 电流传感器：TMCS1107A3B ===
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

/**
 * @brief ADC 中位值（零电流时的 ADC 读数）
 * @note  TMCS1107A3B 零电流输出 = Vcc/2 = 1.65V
 *        1.65V / 3.3V × 4096 = 2048
 */
 #define ADC_CURR_OFFSET    2048U

/**
 * @brief TMCS1107A3B 电流转换系数 (A / ADC LSB)
 * @note  ADC 参考电压 Vref = VDDA = 3.3V（与 MCU 供电共用）
 *        推导:
 *        ADC 每 LSB 电压 = Vref / 2^N = 3.3 / 4096 ≈ 0.8057mV
 *        传感器灵敏度 S = 200mV/A
 *        电流刻度 = (Vref / 2^N) / S = 0.8057mV / 200mV/A = 0.004028 A/LSB
 *        验证: ADC = 2048 + 248（1A） → I = 248 × 0.004028 ≈ 1.0A ✓
 */
#define ADC_CURR_SCALE     0.004028f

/**
 * @brief 母线电压转换系数 (V / ADC LSB)
 * @note  推导:
 *        分压比 K = R_total / R_sample = Vbus / V_ADC
 *        实际分压: Vbus = 12V → V_ADC ≈ 1.31V → K ≈ 12/1.31 ≈ 9.16
 *        电压刻度 = (Vref / 2^N) × K = (3.3/4096) × 9.16 ≈ 0.007381 V/LSB
 *
 *        验证: ADC = 178（1.31V） → Vbus = 178 × 0.007381 ≈ 1.31V × 9.16 ≈ 12.0V ✓
 *        注意: 分压电阻存在 ±1% 容差, 需实测校准 K 值。
 */
#define ADC_BUS_VOLT_SCALE 0.007381f

/*==========================================================================*/
/* DMA 缓冲区（各 2 通道，循环模式）                                         */
/*==========================================================================*/

/**
 * @brief ADC1 DMA 循环缓冲: [Ia_raw, Ib_raw]
 * @note  DMA 循环模式: 转换结束后自动重复, 无需 CPU 干预。
 *        2 个通道: IN1(PA0) = Ia, IN2(PA1) = Ib
 */
extern uint16_t adc1_dma_buf[2U];

/**
 * @brief ADC2 DMA 循环缓冲: [bus_current_raw, bus_voltage_raw]
 * @note  IN3(PA6) = 母线电流, IN4(PA7) = 母线电压
 */
extern uint16_t adc2_dma_buf[2U];

/*==========================================================================*/
/* 电流 / 电压计算结果（float, 单位 A / V）                                  */
/*==========================================================================*/

/**
 * @brief ADC 计算结果（由 HAL_ADC_ConvCpltCallback 回调更新）
 * @note  使用 volatile 修饰: ISR 内更新, main 循环中读取/打印。
 *        float 类型 32-bit, ARM Cortex-M4 单周期加载（含 FPU）。
 */
extern volatile float g_curr_ia;    /*  A 相电流 (A)        */
extern volatile float g_curr_ib;    /*  B 相电流 (A)        */
extern volatile float g_bus_cur;    /*  母线输入电流 (A)      */
extern volatile float g_bus_vol;    /*  直流母线电压 (V)      */

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   ADC 电流/电压检测初始化
 * @param   hadc1  ADC1 HAL 句柄（CubeMX MX_ADC1_Init 已填充时基/通道/DMA 配置）
 * @param   hadc2  ADC2 HAL 句柄
 * @param   htim1  TIM1 HAL 句柄（用于 ADC1 外部触发）
 * @param   htim6  TIM6 HAL 句柄（用于 ADC2 外部触发）
 *
 * @note    初始化顺序（严格按此顺序, 否则 ADC 触发时序异常）:
 *          1. ADC1/ADC2 自校准（上电必须, 否则精度显著下降）
 *          2. 启动 ADC1 DMA + ADC2 DMA（循环模式）
 *          3. 使能 TIM6 + TIM1 计数器（开始产生 TRGO 触发 ADC）
 *
 *          MISRA 注意:
 *          直接操作 HAL 句柄 Instance 寄存器以满足时序要求,
 *          不使用 HAL_TIM API（避免多余的参数检查开销）。
 */
void drv_adc_sampling_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                           TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim6);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ADC_SAMPLING_H */
