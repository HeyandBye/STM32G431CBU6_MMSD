/**
******************************************************************************
* @file     ctl_foc_debug.h
* @author   lidongyang
* @version  0.0.1
* @date     23-June-2026
* @brief    FOC 调试输出 —— 通过 USART1 打印关键参数，辅助调参和故障诊断
*
* @note     用法:
*           1. main.c 初始化末尾调用 FOC_Debug_Init()
*           2. main 循环中周期性调用 FOC_Debug_Print(&g_foc)
*           3. 串口工具 115200-8-N-1 接收
*
*           输出格式: CSV 风格，便于复制到 Excel/Python 绘图
******************************************************************************
*/

#ifndef CTL_FOC_DEBUG_H
#define CTL_FOC_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ctl_foc_core.h"
#include "ctl_foc_openloop.h"

/*==========================================================================*/
/* 调试输出总开关                                                             */
/*==========================================================================*/

/** @brief 调试输出总开关: 1=启用（printf 到 USART1）, 0=关闭（无串口输出） */
#define FOC_DEBUG_ENABLE  1

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   调试输出初始化
 * @note    重定向 printf 到 USART1（覆盖 __io_putchar 弱定义）。
 *          USART1 需已通过 MX_USART1_UART_Init 初始化。调用一次即可。
 */
void FOC_Debug_Init(void);

/**
 * @brief   打印一行闭环 FOC 调试数据（CSV 格式）
 * @note    首次调用自动打印表头。建议每 500ms 调用一次（10Hz）。
 *          输出列: tick, state, loop, Id_ref, Id, Iq_ref, Iq, Vd, Vq,
 *          theta, Ia, Ib, Vbus, duty_a, duty_b, duty_c, fault
 * @param   foc  FOC 控制器指针
 */
void FOC_Debug_Print(const FOC_t *foc);

/**
 * @brief   打印一行开环 FOC 调试数据（CSV 格式）
 * @note    输出列: tick, step, freq_hz, amp, v_angle, enc_raw,
 *          Ia, Ib, Vbus, duty_a, duty_b, duty_c
 * @param   fol  开环控制器指针
 */
void FOC_Debug_Print_OpenLoop(const FOC_OpenLoop_t *fol);

/**
 * @brief   自动调试输出（主循环中周期性调用, 内部管理计时和模式分发）
 * @param   tick_ms  系统运行时间 (ms), 每 500ms 自动触发一次打印
 * @note    替代 main.c 中的 debug_tick + #if FOC_MODE 分支。
 *          必须在 FOC_Debug_Init() 之后调用。
 */
void FOC_Debug_Print_Auto(uint32_t tick_ms);

/**
 * @brief   精简调试输出 —— 仅打印 Id/Iq/Vd/Vq/Vbus/duty（适合锁轴/小电流测试）
 * @param   foc  FOC 控制器指针
 * @note    每行约 120 字符, 比完整版少 60%。首次调用自动打印表头。
 */
void FOC_Debug_Print_Compact(const FOC_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_DEBUG_H */
