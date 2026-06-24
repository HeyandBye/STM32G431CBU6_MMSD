/**
******************************************************************************
* @file     ctl_foc_position.c
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 位置闭环实现 —— 位置 PI, TIM7 100Hz ISR 中运行
******************************************************************************
*/

#include "ctl_foc_position.h"
#include "ctl_foc_speed.h"
#include "ctl_pid.h"
#include <stddef.h>

void FOC_Position_Init(FOC_t *foc, const FOC_Position_Config_t *cfg)
{
    if (foc == NULL || cfg == NULL) return;

    foc->pos_ref = 0U;
    CTL_PID_Init(&foc->pid_pos,
                 cfg->pos_kp, cfg->pos_ki, cfg->pos_kd, cfg->pos_kr,
                 cfg->deriv_alpha,
                 -cfg->speed_limit, cfg->speed_limit);
    foc->pos_speed_limit = cfg->speed_limit;
}

void FOC_Position_SetRef(FOC_t *foc, uint16_t pos_raw)
{
    if (foc == NULL) return;
    foc->pos_ref = pos_raw;
    CTL_PID_SetSetpoint(&foc->pid_pos, (float)pos_raw);
}

void FOC_Position_Start(FOC_t *foc)
{
    if (foc == NULL) return;
    /* 初始化展开位置=当前位置, 避免跨越 wrap 点突变 */
    foc->unwrapped_pos = (float)foc->raw_angle;
    foc->raw_prev      = foc->raw_angle;
    CTL_PID_Reset(&foc->pid_pos);
    CTL_PID_SetSetpoint(&foc->pid_pos, (float)foc->pos_ref);
}

void FOC_Position_Stop(FOC_t *foc)
{
    if (foc == NULL) return;
    foc->pos_ref = 0U;
    FOC_Speed_SetRef(foc, 0.0f);
    CTL_PID_Reset(&foc->pid_pos);
}

void FOC_Position_Run(FOC_t *foc)
{
    float speed_cmd;
    float raw_delta;
    float raw_now;
    float sp, diff;

    if (foc == NULL) return;
    if (foc->state != FOC_STATE_RUNNING) return;

    /* 展开 raw_angle: 消除 0↔16383 回绕, 得到连续累计位置 */
    raw_now  = (float)foc->raw_angle;
    raw_delta = raw_now - (float)foc->raw_prev;
    if (raw_delta >  8192.0f) raw_delta -= 16384.0f;  /* 正向回绕 */
    if (raw_delta < -8192.0f) raw_delta += 16384.0f;  /* 反向回绕 */
    foc->unwrapped_pos += raw_delta;
    foc->raw_prev = (uint16_t)raw_now;

    /* 回绕到 setpoint ±8192 范围内（同时平移 setpoint 保持误差不变）
     * 只平移 unwrapped_pos 会导致 PID 误差符号翻转（越过 180° 时电机助力而非抵抗）。
     * 同步平移 setpoint 后: error = (sp-Δ) - (fb-Δ) = sp-fb, 误差完全不变。
     * 位置环 Kr=1.0, 因此 prop_term = Kp*(Kr*sp - fb) 也保持不变。 */
    sp   = foc->pid_pos.setpoint;
    diff = foc->unwrapped_pos - sp;
    if (diff >  8192.0f) {
        foc->unwrapped_pos    -= 16384.0f;
        foc->pid_pos.setpoint -= 16384.0f;
    }
    if (diff < -8192.0f) {
        foc->unwrapped_pos    += 16384.0f;
        foc->pid_pos.setpoint += 16384.0f;
    }

    /* 位置 PI: 输入展开后的无回绕位置, 输出 RPM 指令 */
    speed_cmd = CTL_PID_Update(&foc->pid_pos, foc->unwrapped_pos);

    if (speed_cmd >  foc->pos_speed_limit) speed_cmd =  foc->pos_speed_limit;
    if (speed_cmd < -foc->pos_speed_limit) speed_cmd = -foc->pos_speed_limit;

    FOC_Speed_SetRef(foc, speed_cmd);
}
