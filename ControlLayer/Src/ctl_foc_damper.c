/**
******************************************************************************
* @file     ctl_foc_damper.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 阻尼模式实现
******************************************************************************
*/

#include "ctl_foc_damper.h"
#include "ctl_foc_current.h"
#include <stddef.h>

void FOC_Damper_Init(FOC_t *foc, float gain)
{
    if (foc == NULL) return;
    foc->damper_gain = gain;
}

void FOC_Damper_Run(FOC_t *foc)
{
    float dtheta;
    float iq_cmd;
    float limit;

    if (foc == NULL) return;
    if (foc->state != FOC_STATE_RUNNING) return;

    /* 转速计算: 电角度差分 → RPM（每 1ms TIM6 调用一次）
     * 与 FOC_Speed_Run 中算法完全一致 */
    dtheta = foc->theta_elec - foc->theta_prev;
    if (dtheta < -34.5f) dtheta += 69.115038379f;
    if (dtheta >  34.5f) dtheta -= 69.115038379f;
    foc->speed_rpm  = dtheta * 868.0f;
    foc->theta_prev = foc->theta_elec;

    /* 阻尼: Iq = -gain × speed（负反馈: 速度越快阻力越大） */
    iq_cmd = -foc->damper_gain * foc->speed_rpm;

    /* 钳位到电流限幅 */
    limit = foc->motor.current_limit;
    if (iq_cmd >  limit) iq_cmd =  limit;
    if (iq_cmd < -limit) iq_cmd = -limit;

    FOC_Current_SetRef(foc, 0.0f, iq_cmd);
}
