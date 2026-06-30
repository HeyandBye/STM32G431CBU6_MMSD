/**
******************************************************************************
* @file     prot_justfloat.h
* @author   lidongyang
* @version  0.0.1
* @date     27-June-2026
* @brief    VOFA+ JustFloat 协议 —— 原始浮点二进制帧收发
*
* @note     JustFloat 是 VOFA+ 的一种数据引擎, 直接发送 float32 原始字节,
*           无需解析文本, 效率远高于 CSV/JSON/printf。
*
*           帧格式: [Ch0][Ch1]...[ChN-1][Tail: 00 00 80 7F]
*           每通道 4 字节（IEEE 754 单精度, little-endian）, 帧尾固定 4 字节。
*
*           VOFA+ 配置: 数据引擎选择 JustFloat, 通道数与帧中通道数一致。
*
*           发送: 阻塞式 HAL_UART_Transmit, 主循环/低优先级 ISR 中调用。
*           接收: 通过 UART IDLE + DMA 接收, 帧解析由 diag_tuning 模块处理。
******************************************************************************
*/

#ifndef PROT_JUSTFLOAT_H
#define PROT_JUSTFLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/*==========================================================================*/
/* 常量                                                                      */
/*==========================================================================*/

/** @brief JustFloat 帧尾标记: 0x00 0x00 0x80 0x7F (VOFA+ 标准帧尾) */
#define JF_TAIL_SIZE  4U

/** @brief 单个 float 的字节数 */
#define JF_FLOAT_SIZE 4U

/** @brief 最大发送通道数（可调整） */
#define JF_MAX_CHANNELS  16U

/** @brief 单帧最大字节数: JF_MAX_CHANNELS × 4 + 4(tail) */
#define JF_MAX_FRAME_BYTES  (JF_MAX_CHANNELS * JF_FLOAT_SIZE + JF_TAIL_SIZE)

/*==========================================================================*/
/* JustFloat 帧构建与发送                                                    */
/*==========================================================================*/

/**
 * @brief   构建 JustFloat 帧到缓冲区
 * @param   buf      输出缓冲区（至少 channels × 4 + 4 字节）
 * @param   data     浮点数据数组
 * @param   channels 通道数量（即 data 数组长度）
 * @return  帧总字节数 = channels × 4 + 4(tail) 或 0（参数无效）
 * @note    帧格式: [f0][f1]...[fN-1][00 00 80 7F], 每 float 为 little-endian。
 */
size_t JF_BuildFrame(uint8_t *buf, const float *data, size_t channels);

/**
 * @brief   构建并发送 JustFloat 帧到 USART
 * @param   huart    UART 句柄
 * @param   data     浮点数据数组
 * @param   channels 通道数量
 * @return  HAL_UART_Transmit 的返回值（HAL_OK=成功）
 * @note    阻塞发送, 耗时 ≈ 帧字节数 × 87µs @ 115200bps。
 *          例如 10 通道: 44 字节 ≈ 3.8ms。建议 <= 200Hz 调用。
 */
int JF_SendFrame(void *huart, const float *data, size_t channels);

/**
 * @brief   快速发送单精度原始缓冲（已包含帧尾）
 * @param   huart     UART 句柄
 * @param   frame     帧数据指针（已含帧尾）
 * @param   frame_len 帧总长度（字节）
 * @return  HAL_UART_Transmit 的返回值
 */
int JF_SendRaw(void *huart, const uint8_t *frame, size_t frame_len);

/*==========================================================================*/
/* JustFloat 帧解析                                                          */
/*==========================================================================*/

/**
 * @brief   从字节缓冲解析 JustFloat 帧为 float 数组
 * @param   raw      原始字节缓冲
 * @param   raw_len  字节长度
 * @param   out      输出 float 数组
 * @param   max_ch   输出数组最大容量
 * @return  解析出的 float 数量, 0=未解析到完整帧（等待更多数据）
 * @note    扫描到帧尾 [00 00 80 7F] 则认为一帧结束，之前每 4 字节为一个 float。
 *          帧尾自身不计入通道。
 */
size_t JF_ParseFrame(const uint8_t *raw, size_t raw_len,
                     float *out, size_t max_ch);

/**
 * @brief   获取 JustFloat 帧尾的字节表示
 * @param   out  输出 4 字节缓冲区
 */
void JF_GetTail(uint8_t out[4]);

#ifdef __cplusplus
}
#endif

#endif /* PROT_JUSTFLOAT_H */
