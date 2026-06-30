/**
******************************************************************************
* @file     diag_tuning.c
* @author   lidongyang
* @version  0.0.1
* @date     27-June-2026
* @brief    FOC 在线调试与 PID 调参实现
*
* @note     依赖: prot_justfloat (JustFloat 协议帧构建)
*                  ctl_pid (CTL_PID_SetGains 在线修改 PID)
*                  ctl_foc_current/speed/position (SetRef 在线修改给定)
******************************************************************************
*/

#include "diag_tuning.h"
#include "prot_justfloat.h"
#include "ctl_pid.h"
#include "ctl_foc_current.h"
#include "ctl_foc_speed.h"
#include "ctl_foc_position.h"
#include "ctl_foc_damper.h"
#include "usart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/*==========================================================================*/
/* 模块级静态变量                                                            */
/*==========================================================================*/

/** @brief UART DMA 接收缓冲区 */
static uint8_t  rx_buf[TUNING_RX_BUF_SIZE];

/** @brief 已接收的完整帧暂存（解析后使用） */
static uint8_t  rx_frame[TUNING_RX_BUF_SIZE];
static size_t   rx_frame_len = 0U;

/** @brief 是否有待处理的接收帧 */
static volatile uint8_t rx_ready = 0U;

/** @brief UART 句柄缓存 */
static UART_HandleTypeDef *p_huart = NULL;

/** @brief 下次发送时刻 (ms) */
static uint32_t next_send_tick = 0U;

/** @brief 模块是否已初始化 */
static uint8_t tuning_initialized = 0U;

/*==========================================================================*/
/* Tuning_Init                                                               */
/*==========================================================================*/

void Tuning_Init(void *huart)
{
    if (huart == NULL) return;

    p_huart = (UART_HandleTypeDef *)huart;

    /* HAL_UARTEx_ReceiveToIdle_DMA 内部自动使能 IDLE 中断,
       IDLE 触发后 HAL_UART_IRQHandler 自动调用 HAL_UARTEx_RxEventCallback */
    HAL_UARTEx_ReceiveToIdle_DMA(p_huart, rx_buf, TUNING_RX_BUF_SIZE);

    tuning_initialized = 1U;

    /* printf 会污染 VOFA+ JustFloat 二进制流，暂时注释 */
    /* printf("\r\n[Tuning] VOFA+ JustFloat online tuning initialized\r\n"); */
}

/*==========================================================================*/
/* HAL_UARTEx_RxEventCallback —— 覆盖 HAL 弱定义, 由 HAL_UART_IRQHandler 调用 */
/*==========================================================================*/

/**
 * @brief   UART IDLE + DMA 接收完成回调（HAL 标准接口）
 * @param   huart  UART 句柄
 * @param   Size   本次接收到的字节数（DMA 已传输量）
 * @note    HAL_UARTEx_ReceiveToIdle_DMA 启动接收后,
 *          每收到一帧（IDLE 检测到空闲）HAL 自动调用此函数。
 *          无需在 stm32g4xx_it.c 中手动添加任何代码。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == NULL || huart->Instance != USART1) return;
    if (!tuning_initialized) return;

    /* 有数据且未溢出 → 拷贝到帧缓冲供主循环解析 */
    if (Size > 0U && Size <= TUNING_RX_BUF_SIZE) {
        memcpy(rx_frame, rx_buf, Size);
        rx_frame_len = (size_t)Size;
        rx_ready = 1U;
    }

    /* 重启 DMA 接收下一帧 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, TUNING_RX_BUF_SIZE);
}

/*==========================================================================*/
/* 指令解析: 从 JustFloat 帧中提取指令                                        */
/*==========================================================================*/

/**
 * @brief   从 JustFloat 帧解析调参指令
 * @param   frame    字节帧（含帧尾）
 * @param   len      帧长度
 * @param   cmd      输出: 指令码
 * @param   params   输出: 参数数组
 * @param   max_p    参数数组最大容量
 * @param   n_params 输出: 实际参数个数
 * @return  0=解析成功, -1=无效帧
 */
static int Tuning_ParseCommand(const uint8_t *frame, size_t len,
                               uint8_t *cmd, float *params,
                               size_t max_p, size_t *n_params)
{
    float  all_floats[JF_MAX_CHANNELS];
    size_t ch_count;
    size_t i;

    if (frame == NULL || cmd == NULL || params == NULL ||
        n_params == NULL || len < (JF_TAIL_SIZE + JF_FLOAT_SIZE)) {
        return -1;
    }

    ch_count = JF_ParseFrame(frame, len, all_floats, JF_MAX_CHANNELS);
    if (ch_count < 2U) {
        /* 至少需要 1 个指令 float + 1 个参数 float */
        return -1;
    }

    /* 第一个 float 的整数部分作为指令码 (1.0~17.0 → cmd 1~17) */
    {
        int cmd_int = (int)(all_floats[0] + 0.5f);
        if (cmd_int < 1 || cmd_int > 17) return -1;
        *cmd = (uint8_t)cmd_int;
    }

    /* 后续 float 为参数 */
    *n_params = ch_count - 1U;
    if (*n_params > max_p) {
        *n_params = max_p;
    }
    for (i = 0U; i < *n_params; i++) {
        params[i] = all_floats[i + 1U];
    }

    return 0;
}

/*==========================================================================*/
/* Tuning_ExecuteCommand                                                     */
/*==========================================================================*/

void Tuning_ExecuteCommand(FOC_t *foc, uint8_t cmd,
                           const float *params, size_t n)
{
    float val;

    if (foc == NULL || params == NULL || n < 1U) return;
    val = params[0];  /* 所有指令都是单参数 */

    switch ((TuneCmd_t)cmd) {

    /* ---- 电流环 PID ---- */
    case TUNE_CMD_KP_ID:
        CTL_PID_SetGains(&foc->pid_id, val, foc->pid_id.Ki, 0.0f, 1.0f);
        break;
    case TUNE_CMD_KI_ID:
        CTL_PID_SetGains(&foc->pid_id, foc->pid_id.Kp, val, 0.0f, 1.0f);
        break;
    case TUNE_CMD_KP_IQ:
        CTL_PID_SetGains(&foc->pid_iq, val, foc->pid_iq.Ki, 0.0f, 1.0f);
        break;
    case TUNE_CMD_KI_IQ:
        CTL_PID_SetGains(&foc->pid_iq, foc->pid_iq.Kp, val, 0.0f, 1.0f);
        break;

    /* ---- 速度环 PID ---- */
    case TUNE_CMD_SPEED_KP:
        CTL_PID_SetGains(&foc->pid_speed, val, foc->pid_speed.Ki,
                         foc->pid_speed.Kd, foc->pid_speed.Kr);
        break;
    case TUNE_CMD_SPEED_KI:
        CTL_PID_SetGains(&foc->pid_speed, foc->pid_speed.Kp, val,
                         foc->pid_speed.Kd, foc->pid_speed.Kr);
        break;
    case TUNE_CMD_SPEED_KD:
        CTL_PID_SetGains(&foc->pid_speed, foc->pid_speed.Kp,
                         foc->pid_speed.Ki, val, foc->pid_speed.Kr);
        break;
    case TUNE_CMD_SPEED_KR:
        CTL_PID_SetGains(&foc->pid_speed, foc->pid_speed.Kp,
                         foc->pid_speed.Ki, foc->pid_speed.Kd, val);
        break;

    /* ---- 位置环 PID ---- */
    case TUNE_CMD_POS_KP:
        CTL_PID_SetGains(&foc->pid_pos, val, foc->pid_pos.Ki,
                         foc->pid_pos.Kd, foc->pid_pos.Kr);
        break;
    case TUNE_CMD_POS_KI:
        CTL_PID_SetGains(&foc->pid_pos, foc->pid_pos.Kp, val,
                         foc->pid_pos.Kd, foc->pid_pos.Kr);
        break;
    case TUNE_CMD_POS_KD:
        CTL_PID_SetGains(&foc->pid_pos, foc->pid_pos.Kp,
                         foc->pid_pos.Ki, val, foc->pid_pos.Kr);
        break;
    case TUNE_CMD_POS_KR:
        CTL_PID_SetGains(&foc->pid_pos, foc->pid_pos.Kp,
                         foc->pid_pos.Ki, foc->pid_pos.Kd, val);
        break;

    /* ---- 给定值 ---- */
    case TUNE_CMD_ID_REF:
        FOC_Current_SetRef(foc, val, foc->iq_ref);
        break;
    case TUNE_CMD_IQ_REF:
        FOC_Current_SetRef(foc, foc->id_ref, val);
        break;
    case TUNE_CMD_SPEED_REF:
        FOC_Speed_SetRef(foc, val);
        break;
    case TUNE_CMD_POS_REF:
        FOC_Position_SetRef(foc, (uint16_t)(val + 0.5f));
        break;

    /* ---- 模式切换 ---- */
    case TUNE_CMD_MODE:
        {
            FOC_Mode_t m = (FOC_Mode_t)(val + 0.5f);
            if (m >= FOC_MODE_CURRENT && m <= FOC_MODE_DAMPER) {
                if (FOC_SwitchMode(foc, m) == 0) {
                    printf("[Tune] Mode -> %d (%s)\r\n", (int)m,
                           m == FOC_MODE_CURRENT  ? "CURRENT" :
                           m == FOC_MODE_SPEED    ? "SPEED"   :
                           m == FOC_MODE_POSITION ? "POSITION": "DAMPER");
                }
            }
        }
        break;

    default:
        break;
    }
}

/*==========================================================================*/
/* 构建不同模式的 JustFloat 数据数组                                         */
/*==========================================================================*/

/**
 * @brief   按当前 FOC_MODE 填充发送通道数据
 * @param   foc       FOC 控制器指针
 * @param   data      输出 float 数组
 * @param   max_ch    最大通道数
 * @return  实际通道数
 */
static size_t Tuning_BuildChannels(const FOC_t *foc, float *data, size_t max_ch)
{
    size_t ch = 0U;

    if (foc == NULL || data == NULL || max_ch == 0U) return 0U;

    switch (g_foc_mode) {

    case FOC_MODE_CURRENT:
        /* 10 通道: Id_ref, Id, Iq_ref, Iq, Vd, Vq, Ia, Ib, speed_rpm, mode */
        if (max_ch < 10U) return 0U;
        data[ch++] = foc->id_ref;
        data[ch++] = foc->id;
        data[ch++] = foc->iq_ref;
        data[ch++] = foc->iq;
        data[ch++] = foc->vd;
        data[ch++] = foc->vq;
        data[ch++] = foc->ia;
        data[ch++] = foc->ib;
        data[ch++] = foc->speed_rpm;
        data[ch++] = (float)g_foc_mode;
        break;

    case FOC_MODE_SPEED:
        /* 8 通道: speed_ref, speed_rpm, iq_ref, Iq, Vq, Ia, Ib, mode */
        if (max_ch < 8U) return 0U;
        data[ch++] = foc->speed_ref;
        data[ch++] = foc->speed_rpm;
        data[ch++] = foc->iq_ref;
        data[ch++] = foc->iq;
        data[ch++] = foc->vq;
        data[ch++] = foc->ia;
        data[ch++] = foc->ib;
        data[ch++] = (float)g_foc_mode;
        break;

    case FOC_MODE_POSITION:
    case FOC_MODE_DAMPER:
        /* 9 通道: pos_cmd, pos_fb, speed_ref, speed_rpm, Iq, Vq, Ia, Ib, mode */
        if (max_ch < 9U) return 0U;
        data[ch++] = (float)foc->pid_pos.setpoint;
        data[ch++] = foc->unwrapped_pos;
        data[ch++] = foc->speed_ref;
        data[ch++] = foc->speed_rpm;
        data[ch++] = foc->iq;
        data[ch++] = foc->vq;
        data[ch++] = foc->ia;
        data[ch++] = foc->ib;
        data[ch++] = (float)g_foc_mode;
        break;

    default:
        /* 6 通道: Id, Iq, Vd, Vq, speed_rpm, mode */
        if (max_ch < 6U) return 0U;
        data[ch++] = foc->id;
        data[ch++] = foc->iq;
        data[ch++] = foc->vd;
        data[ch++] = foc->vq;
        data[ch++] = foc->speed_rpm;
        data[ch++] = (float)g_foc_mode;
        break;
    }

    return ch;
}

/*==========================================================================*/
/* Tuning_SendNow                                                            */
/*==========================================================================*/

void Tuning_SendNow(const FOC_t *foc)
{
    float  data[JF_MAX_CHANNELS];
    size_t ch;

    if (!tuning_initialized || foc == NULL || p_huart == NULL) return;

    ch = Tuning_BuildChannels(foc, data, JF_MAX_CHANNELS);
    if (ch > 0U) {
        JF_SendFrame(p_huart, data, ch);
    }
}

/*==========================================================================*/
/* Tuning_Run                                                                */
/*==========================================================================*/

void Tuning_Run(FOC_t *foc)
{
    uint8_t  cmd;
    float    params[8];
    size_t   n_params;

    if (!tuning_initialized || foc == NULL) return;

    /* ---- 1. 处理接收到的调参指令 ---- */
    if (rx_ready) {
        rx_ready = 0U;

        if (Tuning_ParseCommand(rx_frame, rx_frame_len,
                                &cmd, params, 8U, &n_params) == 0) {
            Tuning_ExecuteCommand(foc, cmd, params, n_params);

            /* 确认指令: 立即发送一次当前数据 */
            Tuning_SendNow(foc);
        }

        rx_frame_len = 0U;
    }

    /* ---- 2. 周期性发送 FOC 数据 ---- */
    {
        uint32_t tick = HAL_GetTick();
        if (tick >= next_send_tick) {
            Tuning_SendNow(foc);
            next_send_tick = tick + TUNING_SEND_PERIOD_MS;
        }
    }
}
