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
#include "usart.h"

#include "ctl_foc_core.h"
#include "ctl_foc_speed.h"
#include "ctl_foc_position.h"
#include "drv_tim_pwm.h"
#include "drv_adc_sampling.h"
#include "diag_tuning.h"

/*==========================================================================*/
/* App_Init —— 系统初始化                                                     */
/*==========================================================================*/

void App_Init(void)
{
    FOC_SystemInit();

#if TUNING_ENABLE
    Tuning_Init(&huart1);
#endif

    switch (g_foc_mode) {
    case FOC_MODE_POSITION:
# if POS_AUTO_STEP
        FOC_Position_SetRef(&g_foc, 0U);
# else
        FOC_Position_SetRef(&g_foc, g_foc.raw_angle);
# endif
        break;
    case FOC_MODE_SPEED:
        FOC_Speed_SetRef(&g_foc, APP_SPEED_RPM);
        break;
    default:
        break;
    }
}

/*==========================================================================*/
/* App_Run —— 主循环任务                                                      */
/*==========================================================================*/

void App_Run(void)
{
    static uint32_t tick_led    = 0U;
    static uint32_t fault_tick  = 0U;

    /* 位置环自动步进 / 手动示教状态 */
#if POS_AUTO_STEP
    static uint32_t tick_pos    = 0U;
    static float    pos_cmd     = 0.0f;
    static int      pos_step    = 0;
#else
    static uint32_t tick_hold   = 0U;
    static uint8_t  hold_active = 0;
#endif

    uint32_t tick_now;

    /* ---- 1. LED 心跳 (1Hz) ---- */
    tick_now = HAL_GetTick();
    if (tick_now >= tick_led) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_Output_LED_Pin);
        tick_led = tick_now + 1000U;
    }

    /* ---- 2. 故障自动恢复 ---- */
    if (g_foc.state == FOC_STATE_FAULT) {
        if (fault_tick == 0U) {
            fault_tick = HAL_GetTick();
        } else if ((HAL_GetTick() - fault_tick) >= FAULT_RECOVER_DELAY_MS) {
            if (FOC_RecoverFromFault(&g_foc) == 0) {
            }
            fault_tick = 0U;
        }
        return;
    }

    /* ---- 3. 在线调参 (VOFA+ JustFloat) ---- */
#if TUNING_ENABLE
    Tuning_Run(&g_foc);
#endif

    /* ---- 4. 控制模式 ---- */
    if (g_foc_mode == FOC_MODE_POSITION) {
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
                    hold_active = 0;
                }
            } else {
                hold_active = 0;
            }
        }
# endif
    }
    /* SPEED/CURRENT/DAMPER: 由 ISR 处理, 主循环无需操作 */
}
