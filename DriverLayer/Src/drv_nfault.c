/**
******************************************************************************
* @file     drv_nfault.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    DRV8313 nFAULT 保护驱动实现
*
* @note     HAL_GPIO_EXTI_Callback 覆盖 HAL 弱定义。
*           PB11 (nFAULT) 下降沿 → 清 MOE + 关 EN。
******************************************************************************
*/

#include "drv_nfault.h"
#include "main.h"
#include "gpio.h"

#if NFAULT_ENABLE
#include "drv_tim_pwm.h"
#include "ctl_foc_core.h"

/** @brief nFAULT 触发标志（应用层轮询用） */
volatile uint8_t g_nfault_triggered = 0;

/*==========================================================================*/
/* HAL_GPIO_EXTI_Callback —— 覆盖 HAL 弱定义                                 */
/*==========================================================================*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_EXTI11_nFAULT_Pin) {
        g_nfault_triggered = 1;
        if (g_foc.state >= FOC_STATE_READY) {
            drv_tim_pwm_moe_off();
            HAL_GPIO_WritePin(GPIOC, GPIO_Output_EN_Pin, GPIO_PIN_RESET);
        }
    }
}
#endif /* NFAULT_ENABLE */

/*==========================================================================*/
/* drv_nfault_init                                                           */
/*==========================================================================*/

void drv_nfault_init(void)
{
    /* EXTI 配置由 CubeMX MX_GPIO_Init 完成，此处预留 */
}
