/**
******************************************************************************
* @file     diag_tuning.h
* @author   lidongyang
* @version  0.0.1
* @date     27-June-2026
* @brief    FOC 在线调试与 PID 调参 —— 通过 VOFA+ JustFloat 双向通信
*
* @note     功能:
*           1. 发送: 周期性将 FOC 实时数据通过 JustFloat 协议发送到 VOFA+
*              不同控制模式发送不同的通道组合, 便于针对性调试。
*           2. 接收: 通过 UART IDLE + DMA 接收 VOFA+ 下发的调参指令,
*              在线修改 PID 增益 / 电流给定 / 转速给定 / 位置给定,
*              无需重新编译烧录。
*
*           发送通道布局（按 FOC_MODE 自动切换，末尾为当前模式编号）:
*           - CURRENT:  Id_ref, Id, Iq_ref, Iq, Vd, Vq, Ia, Ib, speed_rpm, mode (10ch)
*           - SPEED:    speed_ref, speed_rpm, iq_ref, Iq, Vq, Ia, Ib, mode (8ch)
*           - POSITION: pos_cmd, pos_fb, speed_ref, speed_rpm, Iq, Vq, Ia, Ib, mode (9ch)
*           - DAMPER:   同 POSITION (9ch)
*
*           接收指令: 第一个 float = 指令码 (1~17), 第二个 float = 参数值。
*           VOFA+ 发送面板永远填 2 通道, 不会搞混。
*
*           指令列表 (ch0=cmd, ch1=value):
*            1  Set Kp_id   (current D-axis P gain,  V/A)
*            2  Set Ki_id   (current D-axis I gain)
*            3  Set Kp_iq   (current Q-axis P gain,  V/A)
*            4  Set Ki_iq   (current Q-axis I gain)
*            5  Set Speed Kp
*            6  Set Speed Ki
*            7  Set Speed Kd
*            8  Set Speed Kr
*            9  Set Position Kp
*           10  Set Position Ki
*           11  Set Position Kd
*           12  Set Position Kr
*           13  Set Id_ref   (D-axis current ref, A)
*           14  Set Iq_ref   (Q-axis current ref, A)
*           15  Set Speed ref (RPM)
*           16  Set Position ref (encoder raw 0~16383)
*           17  Switch Mode  (2=CURRENT,3=SPEED,4=POSITION,5=DAMPER)
******************************************************************************
*/

#ifndef DIAG_TUNING_H
#define DIAG_TUNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "ctl_foc_core.h"
#include "stm32g4xx_hal.h"

/*==========================================================================*/
/* 配置常量                                                                  */
/*==========================================================================*/

/** @brief 在线调参总开关: 1=启用, 0=禁用（不占用 UART/定时器资源） */
#define TUNING_ENABLE  1

/** @brief 调参数据发送周期 (ms), DMA 非阻塞模式可安全使用 5~10ms (100~200Hz) */
#define TUNING_SEND_PERIOD_MS  5U

/** @brief UART DMA 接收缓冲区大小（足够容纳最大指令帧: 2 + 4×4 = 18 字节） */
#define TUNING_RX_BUF_SIZE  64U

/*==========================================================================*/
/* 调参指令枚举                                                              */
/*==========================================================================*/

/** @brief 调参指令码 —— 每个参数独立一条指令, VOFA+ 永远填 2 通道 */
typedef enum {
    TUNE_CMD_KP_ID       = 1,   /**< 电流环 D 轴 Kp (V/A)            */
    TUNE_CMD_KI_ID       = 2,   /**< 电流环 D 轴 Ki                  */
    TUNE_CMD_KP_IQ       = 3,   /**< 电流环 Q 轴 Kp (V/A)            */
    TUNE_CMD_KI_IQ       = 4,   /**< 电流环 Q 轴 Ki                  */
    TUNE_CMD_SPEED_KP    = 5,   /**< 速度环 Kp                       */
    TUNE_CMD_SPEED_KI    = 6,   /**< 速度环 Ki                       */
    TUNE_CMD_SPEED_KD    = 7,   /**< 速度环 Kd                       */
    TUNE_CMD_SPEED_KR    = 8,   /**< 速度环 Kr                       */
    TUNE_CMD_POS_KP      = 9,   /**< 位置环 Kp                       */
    TUNE_CMD_POS_KI      = 10,  /**< 位置环 Ki                       */
    TUNE_CMD_POS_KD      = 11,  /**< 位置环 Kd                       */
    TUNE_CMD_POS_KR      = 12,  /**< 位置环 Kr                       */
    TUNE_CMD_ID_REF      = 13,  /**< D 轴电流给定 (A)                */
    TUNE_CMD_IQ_REF      = 14,  /**< Q 轴电流给定 (A)                */
    TUNE_CMD_SPEED_REF   = 15,  /**< 转速给定 (RPM)                  */
    TUNE_CMD_POS_REF     = 16,  /**< 位置给定 (encoder raw 0~16383)  */
    TUNE_CMD_MODE        = 17,  /**< 切换控制模式 (2/3/4/5)           */
} TuneCmd_t;

/*==========================================================================*/
/* API                                                                       */
/*==========================================================================*/

/**
 * @brief   初始化在线调参模块
 * @param   huart  UART 句柄指针（&huart1）
 * @note    启动 UART IDLE + DMA 接收, 以接收 VOFA+ 下发的调参指令。
 *          需在 MX_USART1_UART_Init() 之后调用。
 */
void Tuning_Init(void *huart);

/**
 * @brief   在线调参主任务（在主循环中周期性调用）
 * @param   foc       FOC 控制器指针
 * @note    内部管理发送计时（TUNING_SEND_PERIOD_MS 周期）。
 *          1. 按当前 FOC_MODE 构建 JustFloat 帧并发送
 *          2. 检查是否收到调参指令, 若收到则解析并执行
 */
void Tuning_Run(FOC_t *foc);

/**
 * @brief   立即发送当前 FOC 数据（跳过计时, 用于确认指令已接收）
 * @param   foc   FOC 控制器指针
 */
void Tuning_SendNow(const FOC_t *foc);

/**
 * @brief   处理接收到的调参指令（内部使用, 也可外部调用）
 * @param   foc     FOC 控制器指针
 * @param   cmd     指令码 (TuneCmd_t)
 * @param   params  参数数组
 * @param   n       参数个数
 */
void Tuning_ExecuteCommand(FOC_t *foc, uint8_t cmd,
                           const float *params, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_TUNING_H */
