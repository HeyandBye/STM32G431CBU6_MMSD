/**
******************************************************************************
* @file     ctl_foc_debug.c
* @author   lidongyang
* @version  0.0.1
* @date     23-June-2026
* @brief    FOC 调试输出实现 —— USART1 printf 重定向 + CSV 数据输出
*
* @note     输出格式: CSV 逗号分隔, 首次调用打印表头, 后续每行一组数据。
*           可直接粘贴到 Excel 或 Python pandas 分析。
*           115200bps 下一行约 200 字符 ≈ 17ms, 建议 500ms 调用间隔。
******************************************************************************
*/

#include "ctl_foc_debug.h"
#include "drv_adc_sampling.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/*==========================================================================*/
/* USART1 printf 重定向                                                      */
/*==========================================================================*/

/**
 * @brief   __io_putchar 重定向到 USART1（覆盖 syscalls.c 弱定义）
 * @param   ch  待发送字符
 * @return  已发送字符
 * @note    printf → _write → __io_putchar → HAL_UART_Transmit
 *          阻塞发送, 单字符约 87µs @ 115200bps。
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/*==========================================================================*/
/* FOC_Debug_Init                                                           */
/*==========================================================================*/

/**
 * @brief   调试输出初始化
 * @note    打印启动横幅, 确认串口工作正常。
 */
void FOC_Debug_Init(void)
{
    printf("\r\n========================================\r\n");
    printf("  FOC Debug Output - GM3506 Motor\r\n");
    printf("  USART1 115200-8-N-1\r\n");
    printf("========================================\r\n\r\n");
}

/*==========================================================================*/
/* FOC_Debug_Print                                                          */
/*==========================================================================*/

/**
 * @brief   打印一行闭环 FOC 调试数据（CSV 格式）
 * @param   foc  FOC 控制器指针
 * @note    首次调用打印表头。
 */
void FOC_Debug_Print(const FOC_t *foc)
{
    static int header_printed = 0;
    uint32_t   tick_ms;

    if (foc == NULL) return;

    tick_ms = HAL_GetTick();

    if (!header_printed) {
        header_printed = 1;
        printf("tick_ms, state, loop, "
               "enc_raw, adc_ia_raw, adc_ib_raw, adc_vbus_raw, adc_bus_cur_raw, "
               "Ia, Ib, bus_cur, Id_ref, Id, Iq_ref, Iq, "
               "Vd, Vq, theta, sin_theta, cos_theta, Vbus, "
               "duty_a, duty_b, duty_c, fault\r\n");
    }

    printf("%lu, %d, %lu, "
           "%u, %u, %u, %u, %u, "
           "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, "
           "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, "
           "%.4f, %.4f, %.4f, 0x%08lX\r\n",
           (unsigned long)tick_ms,
           (int)foc->state,
           (unsigned long)foc->loop_count,
           (unsigned int)foc->raw_angle,
           (unsigned int)adc1_dma_buf[0], (unsigned int)adc1_dma_buf[1],
           (unsigned int)adc2_dma_buf[1],
           (unsigned int)adc2_dma_buf[0],
           (double)foc->ia, (double)foc->ib,
           (double)g_bus_cur,
           (double)foc->id_ref, (double)foc->id,
           (double)foc->iq_ref, (double)foc->iq,
           (double)foc->vd, (double)foc->vq,
           (double)foc->theta_elec,
           (double)foc->sin_theta, (double)foc->cos_theta,
           (double)foc->vbus,
           (double)foc->duty_a, (double)foc->duty_b, (double)foc->duty_c,
           (unsigned long)foc->fault_code);
}

/*==========================================================================*/
/* FOC_Debug_Print_OpenLoop                                                  */
/*==========================================================================*/

/**
 * @brief   打印一行开环 FOC 调试数据（CSV 格式）
 * @param   fol  开环控制器指针
 * @note    首次调用打印表头。
 */
void FOC_Debug_Print_OpenLoop(const FOC_OpenLoop_t *fol)
{
    static int header_printed = 0;
    uint32_t   tick_ms;

    if (fol == NULL) return;

    tick_ms = HAL_GetTick();

    if (!header_printed) {
        header_printed = 1;
        printf("tick_ms, step, freq_hz, amp, v_angle, enc_raw, "
               "Ia, Ib, Vbus, "
               "duty_a, duty_b, duty_c\r\n");
    }

    printf("%lu, %lu, %.3f, %.4f, %.4f, %u, "
           "%.4f, %.4f, %.4f, "
           "%.4f, %.4f, %.4f\r\n",
           (unsigned long)tick_ms,
           (unsigned long)fol->step_count,
           (double)fol->elec_freq_hz,
           (double)fol->amplitude,
           (double)fol->virtual_angle,
           (unsigned int)fol->enc_raw,
           (double)g_curr_ia,
           (double)g_curr_ib,
           (double)g_bus_vol,
           (double)fol->duty_a,
           (double)fol->duty_b,
           (double)fol->duty_c);
}

/*==========================================================================*/
/* FOC_Debug_Print_Auto —— 自动计时 + 模式分发                               */
/*==========================================================================*/

/**
 * @brief   自动调试输出（主循环调用, 内部管理 500ms 计时和 FOC_MODE 分发）
 * @param   tick_ms  系统运行时间 (ms), 由 HAL_GetTick() 提供
 * @note    首次调用自动打印表头。每 500ms 触发一次, 自动根据 FOC_MODE
 *          选择 FOC_Debug_Print_OpenLoop 或 FOC_Debug_Print。
 */
void FOC_Debug_Print_Auto(uint32_t tick_ms)
{
    static uint32_t last_tick = 0;
    if (tick_ms - last_tick < 500U) return;
    last_tick = tick_ms;

#if FOC_MODE == FOC_MODE_OPENLOOP
    FOC_Debug_Print_OpenLoop(&g_fol);
#else
    FOC_Debug_Print(&g_foc);
#endif
}

/*==========================================================================*/
/* FOC_Debug_Print_Compact                                                  */
/*==========================================================================*/

void FOC_Debug_Print_Compact(const FOC_t *foc)
{
    static int header_printed = 0;

    if (foc == NULL) return;

    if (!header_printed) {
        header_printed = 1;
        printf("  tick_ms  Id_ref    Id      Iq_ref   Iq      Vd      Vq      Vbus    duty_a  theta\r\n");
    }

    printf("%8lu  %+.4f  %+.4f  %+.4f  %+.4f  %+.3f  %+.3f  %5.2f  %5.3f  %6.2f\r\n",
           (unsigned long)HAL_GetTick(),
           (double)foc->id_ref, (double)foc->id,
           (double)foc->iq_ref, (double)foc->iq,
           (double)foc->vd, (double)foc->vq,
           (double)foc->vbus, (double)foc->duty_a,
           (double)foc->theta_elec);
}

