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
    if (foc == NULL || cfg == NULL)
    {
        return;
    }

    foc->pos_ref = 0U;
    CTL_PID_Init(&foc->pid_pos,
                 cfg->pos_kp, cfg->pos_ki, cfg->pos_kd, cfg->pos_kr,
                 cfg->deriv_alpha,
                 -cfg->speed_limit, cfg->speed_limit);
    foc->pos_speed_limit = cfg->speed_limit;
}

/**
 * @brief   折叠 unwrapped_pos 到指定 setpoint 的 ±8192 范围内（最短路径）
 * @note    仅在 setpoint 变更时调用, 不在每周期 Run 中调用。
 *          同步补偿 pid_pos.last_error, 避免微分项突变。
 */
static void fold_unwrapped_to(FOC_t *foc, float target)
{
    float diff = foc->unwrapped_pos - target;
    if (diff > 8192.0f)
    {
        int    wraps = (int)((diff + 8192.0f) / 16384.0f);
        float  shift = (float)wraps * 16384.0f;
        foc->unwrapped_pos    -= shift;
        foc->pid_pos.last_error -= shift;
    }
    else if (diff < -8192.0f)
    {
        int    wraps = (int)((-diff + 8192.0f) / 16384.0f);
        float  shift = (float)wraps * 16384.0f;
        foc->unwrapped_pos    += shift;
        foc->pid_pos.last_error += shift;
    }
}

void FOC_Position_SetRef(FOC_t *foc, uint16_t pos_raw)
{
    if (foc == NULL)
    {
        return;
    }
    foc->pos_ref = pos_raw;
    /* 折叠到新 setpoint 附近, 保证切目标时走最短路径 */
    fold_unwrapped_to(foc, (float)pos_raw);
    CTL_PID_SetSetpoint(&foc->pid_pos, (float)pos_raw);
}

void FOC_Position_Start(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }
    /* 初始化展开位置=当前位置, 避免跨越 wrap 点突变 */
    foc->unwrapped_pos = (float)foc->raw_angle;
    foc->raw_prev      = foc->raw_angle;
    CTL_PID_Reset(&foc->pid_pos);
    /* 折叠到初始 setpoint 附近, 保证启动时走最短路径 */
    fold_unwrapped_to(foc, (float)foc->pos_ref);
    CTL_PID_SetSetpoint(&foc->pid_pos, (float)foc->pos_ref);
}

void FOC_Position_Stop(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }
    foc->pos_ref = 0U;
    FOC_Speed_SetRef(foc, 0.0f);
    CTL_PID_Reset(&foc->pid_pos);
}

void FOC_Position_Run(FOC_t *foc)
{
    float speed_cmd;
    float raw_delta;
    float raw_now;

    if (foc == NULL)
    {
        return;
    }
    if (foc->state != FOC_STATE_RUNNING)
    {
        return;
    }

    /* 展开 raw_angle: 消除 0↔16383 回绕, 得到连续累计位置 */
    raw_now  = (float)foc->raw_angle;
    raw_delta = raw_now - (float)foc->raw_prev;
    if (raw_delta >  8192.0f)
    {
        raw_delta = raw_delta - 16384.0f;
    }  /* 正向回绕 */
    if (raw_delta < -8192.0f)
    {
        raw_delta = raw_delta + 16384.0f;
    }  /* 反向回绕 */
    foc->unwrapped_pos = foc->unwrapped_pos + raw_delta;
    foc->raw_prev = (uint16_t)raw_now;

    /* 位置 PI: unwrapped_pos 如实累积, setpoint 保持不变
     *
     * 运行时不折叠: 外力推开电机 → PID 连续反向抵抗 → 松手回到 setpoint。
     * 最短路径折叠在 SetRef/Start 时已做, 运行时无需再处理。
     * 若推过 180°, PID 取长路径回位——位置锁定下大角度偏移极少, 可接受。 */
    speed_cmd = CTL_PID_Update(&foc->pid_pos, foc->unwrapped_pos);

    if (speed_cmd >  foc->pos_speed_limit)
    {
        speed_cmd =  foc->pos_speed_limit;
    }
    if (speed_cmd < -foc->pos_speed_limit)
    {
        speed_cmd = -foc->pos_speed_limit;
    }

    FOC_Speed_SetRef(foc, speed_cmd);
}
