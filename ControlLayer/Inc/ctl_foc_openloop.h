/**
******************************************************************************
* @file     ctl_foc_openloop.h
* @author   lidongyang
* @version  0.0.1
* @date     24-June-2026
* @brief    FOC 开环控制 —— 虚拟旋转磁场, 无 PI, 无编码器校准
*
* @note     开环 FOC 用软件生成的虚拟电角度替代编码器角度,
*           固定 Vd=0, Vq=amplitude×Vbus, 产生匀速旋转定子磁场。
*
*           优点: 不需要编码器校准/PI 调参/电流传感器, 启动即转
*           缺点: 无转矩/速度闭环, 负载变化可能失步, 效率低于闭环
*
*           适用: 硬件链路验证、电机首次转动测试、闭环 fallback 方案
******************************************************************************
*/

#ifndef CTL_FOC_OPENLOOP_H
#define CTL_FOC_OPENLOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief FOC 开环控制器
 * @note    虚拟角度每步累加 2π×f×Ts, 经 InvPark→SVPWM 产生 PWM 占空比。
 */
typedef struct {
    float virtual_angle;     /**< 虚拟电角度 (rad), 每步累加 2π×f×Ts     */
    float elec_freq_hz;      /**< 电频率 (Hz), 机械转速 ≈ f×60/P          */
    float amplitude;         /**< 电压幅值比 [0, 1], Vq = amp × Vbus       */
    float duty_a;            /**< A 相占空比 [0, 1]                       */
    float duty_b;            /**< B 相占空比                              */
    float duty_c;            /**< C 相占空比                              */
    uint16_t enc_raw;        /**< 编码器原始值（只读, 仅监控用）           */
    uint32_t step_count;     /**< 步数计数（溢出自动回绕）                 */
} FOC_OpenLoop_t;

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   开环 FOC 初始化
 * @param   fol        控制器指针
 * @param   freq_hz    初始电频率 (Hz), 推荐 2~5Hz
 * @param   amplitude  初始电压幅值比 [0, 1], 推荐 0.10~0.20
 */
void FOC_OpenLoop_Init(FOC_OpenLoop_t *fol, float freq_hz, float amplitude);

/**
 * @brief   设置电频率 (Hz)
 * @note    可在主循环中调用, 实现频率斜坡启动。
 *          机械转速 ≈ freq_hz × 60 / pole_pairs
 *          GM3506 P=11: 5Hz→27RPM, 25Hz→136RPM
 */
void FOC_OpenLoop_SetFreq(FOC_OpenLoop_t *fol, float freq_hz);

/**
 * @brief   设置电压幅值比
 * @note    Vq = amplitude × Vbus, SVPWM 最大线性调制比 ≈ 0.577。
 *          推荐初始值 0.15, 最大 0.40。
 */
void FOC_OpenLoop_SetAmplitude(FOC_OpenLoop_t *fol, float amp);

/**
 * @brief   开环 FOC 单步更新（每 PWM 周期调用一次, 20kHz）
 *
 * @note    算法（4 步）:
 *          1. virtual_angle += 2π × freq_hz × 50µs（含回绕）
 *          2. Vd=0, Vq = amplitude × Vbus
 *          3. InvPark(Vd,Vq, virtual_angle) → Vα,Vβ
 *          4. SVPWM(Vα,Vβ,Vbus) → duty_a,b,c
 *
 * @param   fol      控制器指针
 * @param   vbus     当前母线电压 (V)
 * @param   enc_raw  编码器原始值 (0~16383), 只读取不参与控制
 */
void FOC_OpenLoop_Step(FOC_OpenLoop_t *fol, float vbus, uint16_t enc_raw);

/**
 * @brief   开环 FOC 完整运行周期（供 HAL 回调直接调用）
 * @note    封装: 读编码器 → 读母线电压 → Step → 写 PWM。
 *          HAL 回调中仅需一行: FOC_OpenLoop_Run(&g_fol);
 * @param   fol  控制器指针
 */
void FOC_OpenLoop_Run(FOC_OpenLoop_t *fol);

/**
 * @brief   启动开环控制（对标 FOC_Current_Start）
 * @note    清零虚拟角度和步数计数，使电机从已知状态开始运行。
 */
void FOC_OpenLoop_Start(FOC_OpenLoop_t *fol);

/**
 * @brief   停止开环控制（对标 FOC_Current_Stop）
 * @note    三相占空比全部归 50%（绕组电压为零），电机自由滑行。
 */
void FOC_OpenLoop_Stop(FOC_OpenLoop_t *fol);

/** @brief ADC 回调包装（匹配 DRV_ADC_ConvCplt_Callback_t，供 FOC_SystemInit 注册） */
void FOC_OpenLoop_Run_Callback(void);

/**
 * @brief   开环测试斜坡（主循环中周期性调用）
 * @param   fol      控制器指针
 * @param   tick_ms  系统运行时间 (ms)
 * @note    0~1s: 5Hz/10% → 1~6s: 5→50Hz + 10%→35% → 6s后: 50Hz/35% 稳定。
 */
void FOC_OpenLoop_TestRamp(FOC_OpenLoop_t *fol, uint32_t tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* CTL_FOC_OPENLOOP_H */
