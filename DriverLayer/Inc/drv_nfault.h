/**
******************************************************************************
* @file     drv_nfault.h
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    DRV8313 nFAULT 保护驱动
*
* @note     PB11 下降沿 → DRV8313 报告过流/过热/欠压 → 紧急关断 MOE + EN。
*           NFAULT_ENABLE 宏控制编译开关。
******************************************************************************
*/

#ifndef DRV_NFAULT_H
#define DRV_NFAULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/** @brief DRV8313 nFAULT 功能开关: 1=启用, 0=关闭 */
#define NFAULT_ENABLE  1

/** @brief nFAULT 触发标志（ISR 置 1，应用层轮询） */
extern volatile uint8_t g_nfault_triggered;

/**
 * @brief   nFAULT 保护初始化（可选）
 * @note    当前硬件 EXTI 配置由 CubeMX 在 MX_GPIO_Init 中完成，
 *          此函数预留用于额外的初始化需求。
 */
void drv_nfault_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_NFAULT_H */
