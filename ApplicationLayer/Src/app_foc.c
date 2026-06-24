/**
******************************************************************************
* @file     app_foc.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    应用层 —— FOC 系统初始化和主循环任务实现
*
* @note     编译期开关（main.c 中定义，通过 app_foc.h 间接可见）:
*           - POS_AUTO_STEP: 1=自动步进, 0=手动示教
*           - DEBUG_PRINT:   1=串口输出, 0=关闭
******************************************************************************
*/

#include "app_foc.h"
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>

#include "ctl_foc_core.h"
#include "ctl_foc_debug.h"
#include "ctl_foc_position.h"
#include "drv_tim_pwm.h"
#include "drv_adc_sampling.h"

/*==========================================================================*/
/* App_Init —— 系统初始化                                                     */
/*==========================================================================*/

void App_Init(void)
{
    /* FOC 系统总初始化（驱动层 + 控制层 + 模式分发 + ADC 回调注册） */
    FOC_SystemInit();

#if DEBUG_PRINT
    FOC_Debug_Init();
#endif

#if FOC_MODE == FOC_MODE_POSITION
# if POS_AUTO_STEP
    FOC_Position_SetRef(&g_foc, 0U);
#  if DEBUG_PRINT
    printf("\r\n=== Position Loop (auto step 6deg/1s) ===\r\n\r\n");
#  endif
# else
    FOC_Position_SetRef(&g_foc, g_foc.raw_angle);
#  if DEBUG_PRINT
    printf("\r\n=== Position Loop (hold-6s-to-relock) ===\r\n\r\n");
#  endif
# endif
#elif FOC_MODE == FOC_MODE_DAMPER
# if DEBUG_PRINT
    printf("\r\n=== Damper Mode (turn-to-resist, stop-to-lock) ===\r\n\r\n");
# endif
#endif
}

/*==========================================================================*/
/* App_Run —— 主循环任务                                                      */
/*==========================================================================*/

void App_Run(void)
{
    /* ---- 持久状态（static，跨调用保持） ---- */
    static uint32_t tick_led    = 0U;    /* LED 下次翻转时刻 (ms)          */
    static uint32_t fault_tick  = 0U;    /* 进入 FAULT 的时刻, 0=未故障     */

#if FOC_MODE == FOC_MODE_POSITION
# if POS_AUTO_STEP
    /* 自动步进模式 */
    static uint32_t tick_pos    = 0U;    /* 下次步进时刻 (ms)               */
    static float    pos_cmd     = 0.0f;  /* 累计位置指令 (LSB)              */
    static int      pos_step    = 0;     /* 已执行步数                      */
# else
    /* 手动示教模式 */
    static uint32_t tick_hold   = 0U;    /* 持续偏离的起始时刻              */
    static uint8_t  hold_active = 0;     /* 1=正在计时, 0=未偏离或已复位    */
# endif
#endif

    uint32_t tick_now;

    /* ================================================================ */
    /* 1. LED 心跳 (1Hz)                                                 */
    /* ================================================================ */
    tick_now = HAL_GetTick();
    if (tick_now >= tick_led) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_Output_LED_Pin);
        tick_led = tick_now + 1000U;
    }

    /* ================================================================ */
    /* 2. 故障自动恢复                                                    */
    /* ================================================================ */
    if (g_foc.state == FOC_STATE_FAULT) {
        if (fault_tick == 0U) {
            fault_tick = HAL_GetTick();
#if DEBUG_PRINT
            printf("\r\n!!! FAULT: code=0x%08lX", (unsigned long)g_foc.fault_code);
            if (g_foc.fault_code & FOC_FAULT_OVERCURRENT)  printf(" OVERCURRENT");
            if (g_foc.fault_code & FOC_FAULT_OVERVOLTAGE)  printf(" OVERVOLTAGE");
            if (g_foc.fault_code & FOC_FAULT_UNDERVOLTAGE) printf(" UNDERVOLTAGE");
            if (g_foc.fault_code & FOC_FAULT_ENCODER)     printf(" ENCODER");
            printf(" | ia=%.3f ib=%.3f vbus=%.2f\r\n",
                   (double)g_curr_ia, (double)g_curr_ib, (double)g_bus_vol);
#endif
        } else if ((HAL_GetTick() - fault_tick) >= FAULT_RECOVER_DELAY_MS) {
#if DEBUG_PRINT
            printf(">>> Recovering from fault...\r\n");
#endif
            if (FOC_RecoverFromFault(&g_foc) == 0) {
#if DEBUG_PRINT
                printf(">>> Recovery OK\r\n");
#endif
            }
            fault_tick = 0U;
        }
        /* FAULT 状态下跳过位置控制 */
        return;
    }

    /* ================================================================ */
    /* 3. 调试打印 (20ms 周期)                                           */
    /* ================================================================ */
#if DEBUG_PRINT
    {
        static uint32_t tick_prn = 0U;
        tick_now = HAL_GetTick();
        if (tick_now >= tick_prn) {
            FOC_Debug_Print_Compact(&g_foc);
            tick_prn = tick_now + 20U;
        }
    }
#endif

    /* ================================================================ */
    /* 4. 控制模式                                                       */
    /* ================================================================ */
#if FOC_MODE == FOC_MODE_POSITION
# if POS_AUTO_STEP
    /* 自动步进: 每 1s 转 6° (273 LSB), 持续同向旋转 */
    tick_now = HAL_GetTick();
    if (tick_now >= tick_pos) {
        pos_step++;
        pos_cmd += 273.0f;
        /* 转完一圈 (16384 LSB) → 归零, 重置 PID 防跳变 */
        if (pos_cmd >= 16384.0f) {
            pos_cmd -= 16384.0f;
            pos_step = 0;
            CTL_PID_Reset(&g_foc.pid_pos);
        }
        CTL_PID_SetSetpoint(&g_foc.pid_pos, pos_cmd);
#  if DEBUG_PRINT
        printf("\r\n--- Step %d: pos=%.0f LSB (%.0f deg) ---\r\n\r\n",
               pos_step, (double)pos_cmd, (double)(pos_step * 6.0f));
#  endif
        tick_pos = tick_now + 1000U;
    }
# else
    /* 手动示教: 拧偏超过 500 LSB (~11°) 并保持 6 秒 → 锁定新位置 */
    tick_now = HAL_GetTick();
    {
        float sp  = g_foc.pid_pos.setpoint;
        float fb  = g_foc.unwrapped_pos;
        float err = sp - fb;
        if (err < 0.0f) err = -err;
        if (err > 500.0f) {
            if (!hold_active) {
                hold_active = 1;
                tick_hold   = tick_now;
            } else if (tick_now - tick_hold >= 6000U) {
                CTL_PID_SetSetpoint(&g_foc.pid_pos, fb);
#  if DEBUG_PRINT
                printf("\r\n--- Hold 6s: locked at %.0f LSB ---\r\n\r\n", (double)fb);
#  endif
                hold_active = 0;
            }
        } else {
            hold_active = 0;
        }
    }
# endif
#elif FOC_MODE == FOC_MODE_DAMPER
    /* 阻尼模式: 控制由 TIM6 ISR 中的 FOC_Damper_Run 处理, 主循环无需操作 */
#endif
}
