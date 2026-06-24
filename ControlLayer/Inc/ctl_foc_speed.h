/**
******************************************************************************
* @file     ctl_foc_speed.h
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 速度闭环 —— 速度 PI 控制器
*
* @note     速度环 = 外环（1kHz TIM6），电流环 = 内环（20kHz ADC）
*           速度 PI 输出 → Iq_ref, 钳位到 ±speed_iq_limit。
******************************************************************************
*/

#ifndef CTL_FOC_SPEED_H
#define CTL_FOC_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ctl_foc_core.h"

typedef struct {
    float    speed_kp;
    float    speed_ki;
    float    speed_kd;
    float    speed_kr;
    float    speed_iq_limit;
    float    speed_limit;
    float    deriv_alpha;
} FOC_Speed_Config_t;

void FOC_Speed_Init(FOC_t *foc, const FOC_Speed_Config_t *cfg);
void FOC_Speed_SetRef(FOC_t *foc, float speed_rpm);
void FOC_Speed_Start(FOC_t *foc);
void FOC_Speed_Stop(FOC_t *foc);
void FOC_Speed_Run(FOC_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_SPEED_H */