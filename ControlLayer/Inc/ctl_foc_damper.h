/**
******************************************************************************
* @file     ctl_foc_damper.h
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 阻尼模式 —— 转动有阻力，停手即锁定
*
* @note     原理: 电流环运行 (Id=0)，TIM6 中计算转速，
*           Iq = -damping_gain × speed_rpm（纯比例负反馈）。
*           转动越快阻力越大，静止时 Iq=0 无力矩输出，转子自由停留。
*
*           与速度伺服的区别: 无积分项，不会试图精确维持零速。
******************************************************************************
*/

#ifndef CTL_FOC_DAMPER_H
#define CTL_FOC_DAMPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ctl_foc_core.h"

/**
 * @brief   初始化阻尼模式
 * @param   foc   FOC 控制器指针
 * @param   gain  阻尼增益 (A/RPM)，越大阻力越强
 * @note    推荐初始值 0.03~0.1 A/RPM。
 *          电流环必须先于本函数完成初始化和校准。
 */
void FOC_Damper_Init(FOC_t *foc, float gain);

/**
 * @brief   阻尼模式单步运行（TIM6 1kHz ISR 中调用）
 * @param   foc  FOC 控制器指针
 * @note    计算转速 → Iq = -gain × speed → 写入电流给定。
 *          静止时 Iq=0，转子自由停留。
 */
void FOC_Damper_Run(FOC_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_DAMPER_H */
