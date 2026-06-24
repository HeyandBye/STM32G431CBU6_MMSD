/**
******************************************************************************
* @file     ctl_foc_position.h
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 位置闭环 —— 位置 PI 控制器
*
* @note     位置环 = 最外环（1kHz TIM6），速度环 = 中环，电流环 = 内环
*           位置 PI 输出 → speed_ref → 速度 PI → Iq_ref → 电流 PI → PWM
******************************************************************************
*/

#ifndef CTL_FOC_POSITION_H
#define CTL_FOC_POSITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ctl_foc_core.h"

typedef struct {
    float pos_kp;       /**< 位置环比例增益  (RPM/encoder_LSB)   */
    float pos_ki;       /**< 位置环积分增益   (离散, 1kHz)       */
    float pos_kd;       /**< 位置环微分增益   0=PI 模式           */
    float pos_kr;       /**< setpoint 加权 [0,1]                 */
    float speed_limit;  /**< 位置环输出限幅 (RPM)                */
    float deriv_alpha;  /**< 微分滤波系数                        */
} FOC_Position_Config_t;

void FOC_Position_Init(FOC_t *foc, const FOC_Position_Config_t *cfg);
void FOC_Position_SetRef(FOC_t *foc, uint16_t pos_raw);
void FOC_Position_Start(FOC_t *foc);
void FOC_Position_Stop(FOC_t *foc);
void FOC_Position_Run(FOC_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_POSITION_H */
