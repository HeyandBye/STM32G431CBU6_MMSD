/**
******************************************************************************
* @file     ctl_foc_speed.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 速度闭环实现 —— 速度 PI, TIM6 1kHz ISR 中运行
******************************************************************************
*/

#include "ctl_foc_speed.h"
#include "ctl_foc_current.h"
#include "ctl_foc_position.h"
#include "ctl_foc_damper.h"
#include "ctl_pid.h"
#include "tim.h"
#include <stddef.h>

/*==========================================================================*/
/* HAL_TIM_PeriodElapsedCallback —— 覆盖弱定义                               */
/*==========================================================================*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        switch (g_foc_mode) {
        case FOC_MODE_SPEED:
        case FOC_MODE_POSITION:
            FOC_Speed_Run(&g_foc);
            break;
        case FOC_MODE_DAMPER:
            FOC_Damper_Run(&g_foc);
            break;
        default:
            break;
        }
    }
    if (htim->Instance == TIM7) {
        if (g_foc_mode == FOC_MODE_POSITION) {
            FOC_Position_Run(&g_foc);
        }
    }
}

void FOC_Speed_Init(FOC_t *foc, const FOC_Speed_Config_t *cfg)
{
    if (foc == NULL || cfg == NULL) return;

    foc->speed_ref = 0.0f;
    CTL_PID_Init(&foc->pid_speed,
                 cfg->speed_kp, cfg->speed_ki, cfg->speed_kd, cfg->speed_kr,
                 cfg->deriv_alpha,
                 -cfg->speed_iq_limit, cfg->speed_iq_limit);
    foc->speed_iq_limit = cfg->speed_iq_limit;
    foc->speed_limit    = cfg->speed_limit;
}

void FOC_Speed_SetRef(FOC_t *foc, float speed_rpm)
{
    if (foc == NULL) return;
    if (speed_rpm >  foc->speed_limit) speed_rpm =  foc->speed_limit;
    if (speed_rpm < -foc->speed_limit) speed_rpm = -foc->speed_limit;
    foc->speed_ref = speed_rpm;
    CTL_PID_SetSetpoint(&foc->pid_speed, speed_rpm);
}

void FOC_Speed_Start(FOC_t *foc)
{
    if (foc == NULL) return;
    CTL_PID_Reset(&foc->pid_speed);
    CTL_PID_SetSetpoint(&foc->pid_speed, foc->speed_ref);
}

void FOC_Speed_Stop(FOC_t *foc)
{
    if (foc == NULL) return;
    foc->speed_ref = 0.0f;
    FOC_Current_SetRef(foc, 0.0f, 0.0f);
    CTL_PID_Reset(&foc->pid_speed);
}

void FOC_Speed_Run(FOC_t *foc)
{
    float dtheta;
    float iq_cmd;

    if (foc == NULL) return;
    if (foc->state != FOC_STATE_RUNNING) return;

    /* 转速计算: 电角度差分 → RPM（每 1ms TIM6 调用一次） */
    dtheta = foc->theta_elec - foc->theta_prev;
    if (dtheta < -34.5f) dtheta += 69.115038379f;
    if (dtheta >  34.5f) dtheta -= 69.115038379f;
    foc->speed_rpm  = dtheta * 868.0f;
    foc->theta_prev = foc->theta_elec;

    /* 速度 PI → Iq 指令 */
    iq_cmd = CTL_PID_Update(&foc->pid_speed, foc->speed_rpm);

    if (iq_cmd >  foc->speed_iq_limit) iq_cmd =  foc->speed_iq_limit;
    if (iq_cmd < -foc->speed_iq_limit) iq_cmd = -foc->speed_iq_limit;

    FOC_Current_SetRef(foc, 0.0f, iq_cmd);
}
