/**
******************************************************************************
* @file     ctl_foc_core.c
* @author   lidongyang
* @version  0.0.4
* @date     24-June-2026
* @brief    FOC 基础设施 —— 公共初始化、故障检测、系统总入口
*
* @note     本文件仅包含 FOC 通用基础设施:
*           - FOC_Init / FOC_SetMotorParams（通用初始化）
*           - FOC_SystemInit（模式分发 + 回调注册）
*           - FOC_CheckFault / FOC_EmergencyStop（通用故障处理）
*           - 查询函数（FOC_GetState / GetCurrents / GetDuties）
*
*           各模式实现在独立文件中:
*           - ctl_foc_openloop.c: 开环电压拖动
*           - ctl_foc_current.c:  电流闭环（Id=0 + PI）
*
*           控制环路入口: ADC1 转换完成回调
*           → FOC_OpenLoop_Run_Callback / FOC_Current_Run_Callback
******************************************************************************
*/

#include "ctl_foc_core.h"
#include "ctl_foc_openloop.h"
#include "ctl_foc_current.h"
#include "ctl_foc_speed.h"
#include "ctl_foc_position.h"
#include "drv_adc_sampling.h"
#include "drv_spi_as5048a.h"
#include "drv_tim_pwm.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "stm32g4xx_hal.h"

/*==========================================================================*/
/* 全局 FOC 实例                                                             */
/*==========================================================================*/

FOC_t g_foc;

#if FOC_MODE == FOC_MODE_OPENLOOP
FOC_OpenLoop_t g_fol;
#endif

/*==========================================================================*/
/* FOC_SystemInit —— 系统总初始化入口                                          */
/*==========================================================================*/

/**
 * @brief   FOC 系统总初始化（驱动层 + 控制层 + 模式分发）
 * @note    初始化顺序:
 *          1. 驱动层: AS5048A → PWM → ADC 采样
 *          2. 等待 ADC 稳定 (100ms)
 *          3. 使能 PWM + DRV8313
 *          4. FOC 核心初始化 (Init + SetMotorParams)
 *          5. 按 FOC_MODE 宏分发模式 + 注册 ADC 回调
 */
void FOC_SystemInit(void)
{
    /* 1. 驱动层初始化 */
    drv_as5048a_init();
    drv_tim_pwm_init(&htim1);
    drv_adc_sampling_init(&hadc1, &hadc2, &htim1, &htim6);

    /* 2. 等待 ADC 稳定 */
    HAL_Delay(100);

    /* 3. 使能 PWM + DRV8313 */
    drv_tim_pwm_enable();
    HAL_GPIO_WritePin(GPIO_Output_EN_GPIO_Port, GPIO_Output_EN_Pin, GPIO_PIN_SET);

    /* 4. 模式分支: Init → Calibrate → SetRef → Start → 注册回调 */
#if FOC_MODE == FOC_MODE_OPENLOOP
    FOC_OpenLoop_Init(&g_fol, 5.0f, 0.10f);
    FOC_OpenLoop_Start(&g_fol);
    drv_adc_register_conv_cplt_callback(FOC_OpenLoop_Run_Callback);
#elif FOC_MODE == FOC_MODE_CURRENT
    {
        FOC_Current_Config_t cfg = {11, 1.0f, 2.0f, 12.0f, 4.0f, 0.3f, 4.0f, 0.3f};
        FOC_Current_Init(&g_foc, &cfg);
    }
    FOC_Current_CalibrateEncoder(&g_foc,
                                 drv_as5048a_read_angle,
                                 drv_tim_pwm_set_duty_f);
    FOC_Current_SetRef(&g_foc, 0.5f, 0.0f);
    FOC_Current_Start(&g_foc);
    drv_adc_register_conv_cplt_callback(FOC_Current_Run_Callback);
#elif FOC_MODE == FOC_MODE_SPEED
    {
        FOC_Current_Config_t cfg_i = {11, 1.0f, 2.0f, 12.0f, 4.0f, 0.3f, 4.0f, 0.3f};
        FOC_Current_Init(&g_foc, &cfg_i);
    }
    FOC_Current_CalibrateEncoder(&g_foc,
                                 drv_as5048a_read_angle,
                                 drv_tim_pwm_set_duty_f);
    FOC_Current_SetRef(&g_foc, 0.5f, 0.0f);
    FOC_Current_Start(&g_foc);
    drv_adc_register_conv_cplt_callback(FOC_Current_Run_Callback);
    {
        FOC_Speed_Config_t cfg_s = {0.002f, 0.001f, 0.0f, 1.0f, 1.0f, 2000.0f, 0.1f};
        FOC_Speed_Init(&g_foc, &cfg_s);
    }
    FOC_Speed_SetRef(&g_foc, 0.0f);
    FOC_Speed_Start(&g_foc);
#elif FOC_MODE == FOC_MODE_POSITION
    {
        FOC_Current_Config_t cfg_i = {11, 1.0f, 2.0f, 12.0f, 4.0f, 0.3f, 4.0f, 0.3f};
        FOC_Current_Init(&g_foc, &cfg_i);
    }
    FOC_Current_CalibrateEncoder(&g_foc,
                                 drv_as5048a_read_angle,
                                 drv_tim_pwm_set_duty_f);
    FOC_Current_SetRef(&g_foc, 0.5f, 0.0f);
    FOC_Current_Start(&g_foc);
    drv_adc_register_conv_cplt_callback(FOC_Current_Run_Callback);
    {
        FOC_Speed_Config_t cfg_s = {0.002f, 0.001f, 0.0f, 1.0f, 1.0f, 2000.0f, 0.1f};
        FOC_Speed_Init(&g_foc, &cfg_s);
    }
    FOC_Speed_SetRef(&g_foc, 0.0f);
    FOC_Speed_Start(&g_foc);
    {
        FOC_Position_Config_t cfg_p = {0.10f, 0.0f, 0.0f, 1.0f, 500.0f, 0.1f};
        FOC_Position_Init(&g_foc, &cfg_p);
    }
    FOC_Position_SetRef(&g_foc, 0U);
    FOC_Position_Start(&g_foc);
    HAL_TIM_Base_Start_IT(&htim7);
#else
    /* SPEED / POSITION: 待实现 */
    drv_adc_register_conv_cplt_callback(NULL);
#endif
}

/**
 * @brief   FOC 控制器初始化（清零所有状态, 置 IDLE）
 * @param   foc  FOC 控制器指针
 * @note    通过局部零结构体复制实现全字段清零（含 PID 内部状态）。
 */
void FOC_Init(FOC_t *foc)
{
    if (foc == NULL) return;
    FOC_t zero = {0};
    *foc = zero;
    foc->state = FOC_STATE_IDLE;
}

/**
 * @brief   设置电机参数（置 READY 状态）
 * @param   foc            FOC 控制器指针
 * @param   pole_pairs     极对数（GM3506: 11）
 * @param   enc_offset     编码器零点偏置（未知填 0）
 * @param   current_rated  额定电流 (A)
 * @param   current_limit  Id/Iq 硬限幅 (A)
 * @param   vbus_nominal   额定母线电压 (V)
 */
void FOC_SetMotorParams(FOC_t   *foc,
                        uint8_t  pole_pairs,
                        uint16_t enc_offset,
                        float    current_rated,
                        float    current_limit,
                        float    vbus_nominal)
{
    if (foc == NULL) return;

    foc->motor.pole_pairs    = pole_pairs;
    foc->motor.enc_offset    = enc_offset;
    foc->motor.rated_current = current_rated;
    foc->motor.current_limit = current_limit;
    foc->motor.vbus_nominal  = vbus_nominal;

    foc->state = FOC_STATE_READY;
}

/**
 * @brief   紧急停机（任意状态 → FAULT）
 * @param   foc  FOC 控制器指针
 * @note    占空比归 50%, Id/Iq 给定清零, PID 积分复位。
 */
void FOC_EmergencyStop(FOC_t *foc)
{
    if (foc == NULL) return;

    foc->state  = FOC_STATE_FAULT;
    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->duty_a = 0.5f;
    foc->duty_b = 0.5f;
    foc->duty_c = 0.5f;
    drv_tim_pwm_set_duty_f(0.5f, 0.5f, 0.5f);

    CTL_PID_Reset(&foc->pid_id);
    CTL_PID_Reset(&foc->pid_iq);
}

/*==========================================================================*/
/* 查询                                                                      */
/*==========================================================================*/

/**
 * @brief   获取当前运行状态
 * @param   foc  FOC 控制器指针
 * @return  当前状态
 */
FOC_State_t FOC_GetState(const FOC_t *foc)
{
    if (foc == NULL) return FOC_STATE_IDLE;
    return foc->state;
}

/**
 * @brief   获取 Id/Iq 实测值 (A)
 * @param   foc  FOC 控制器指针
 * @param   id   输出: d 轴电流 (A)
 * @param   iq   输出: q 轴电流 (A)
 * @note    读取上一周期计算结果, 实时性 ≤ 50µs。
 */
void FOC_GetCurrents(const FOC_t *foc, float *id, float *iq)
{
    if (foc == NULL) return;
    if (id != NULL) *id = foc->id;
    if (iq != NULL) *iq = foc->iq;
}

/**
 * @brief   获取当前三相占空比 [0, 1]
 * @param   foc  FOC 控制器指针
 * @param   da   输出: A 相占空比
 * @param   db   输出: B 相占空比
 * @param   dc   输出: C 相占空比
 */
void FOC_GetDuties(const FOC_t *foc, float *da, float *db, float *dc)
{
    if (foc == NULL) return;
    if (da != NULL) *da = foc->duty_a;
    if (db != NULL) *db = foc->duty_b;
    if (dc != NULL) *dc = foc->duty_c;
}

/**
 * @brief   故障检测（每 PWM 周期调用一次）
 * @param   foc              FOC 控制器指针
 * @param   bus_voltage_max  母线电压上限 (V)
 * @param   current_max      相电流上限 (A)
 * @param   enc_valid        编码器有效标志（0=异常）
 * @return  0=无故障, 非0=故障码（见 FOC_FAULT_*）
 *
 * @note    检测项: 过压/欠压/过流/编码器异常。故障码可组合（位或）。
 */

uint32_t FOC_CheckFault(const FOC_t *foc,
                        float bus_voltage_max,
                        float current_max,
                        uint8_t enc_valid)
{
    uint32_t fault = FOC_FAULT_NONE;

    if (foc == NULL) return FOC_FAULT_NONE;

    if (foc->vbus > bus_voltage_max) fault |= FOC_FAULT_OVERVOLTAGE;
    if (foc->vbus < 0.5f)           fault |= FOC_FAULT_UNDERVOLTAGE;

    if (foc->ia > current_max || foc->ia < -current_max ||
        foc->ib > current_max || foc->ib < -current_max)
        fault |= FOC_FAULT_OVERCURRENT;

    if (enc_valid == 0) fault |= FOC_FAULT_ENCODER;

    return fault;
}
