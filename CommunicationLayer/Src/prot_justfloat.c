/**
******************************************************************************
* @file     prot_justfloat.c
* @author   lidongyang
* @version  0.0.1
* @date     27-June-2026
* @brief    VOFA+ JustFloat 协议实现
******************************************************************************
*/

#include "prot_justfloat.h"
#include "usart.h"
#include <string.h>

/* JustFloat 帧尾: float 角度表示的特殊标记, VOFA+ 用此分隔帧 */
static const uint8_t jf_tail[JF_TAIL_SIZE] = {0x00, 0x00, 0x80, 0x7F};

/*==========================================================================*/
/* JF_BuildFrame                                                             */
/*==========================================================================*/

size_t JF_BuildFrame(uint8_t *buf, const float *data, size_t channels)
{
    size_t i;
    size_t offset;

    if (buf == NULL || data == NULL || channels == 0U || channels > JF_MAX_CHANNELS) {
        return 0U;
    }

    offset = 0U;

    /* 逐通道拷贝 float → little-endian 字节 */
    for (i = 0U; i < channels; i++) {
        uint32_t raw;
        /* 将 float 的位模式安全复制到 uint32_t */
        memcpy(&raw, &data[i], sizeof(raw));

        buf[offset + 0U] = (uint8_t)(raw);
        buf[offset + 1U] = (uint8_t)(raw >> 8U);
        buf[offset + 2U] = (uint8_t)(raw >> 16U);
        buf[offset + 3U] = (uint8_t)(raw >> 24U);

        offset += JF_FLOAT_SIZE;
    }

    /* 追加帧尾 */
    memcpy(&buf[offset], jf_tail, JF_TAIL_SIZE);
    offset += JF_TAIL_SIZE;

    return offset;
}

/*==========================================================================*/
/* DMA 发送 —— 双缓冲 + 非阻塞                                               */
/*==========================================================================*/

/* 双缓冲: ping-pong, 防止构建新帧时覆盖 DMA 正在发送的旧帧 */
static uint8_t  jf_tx_buf[2][JF_MAX_FRAME_BYTES];
static uint8_t  jf_tx_idx = 0U;   /* 当前使用 buf[0] 还是 buf[1] */

/**
 * @brief   构建 JustFloat 帧并通过 DMA 非阻塞发送
 * @param   huart     UART 句柄指针（&huart1）
 * @param   data      浮点数据数组
 * @param   channels  通道数量
 * @return  0=发送已启动, -1=参数无效, -2=DMA 忙（本帧丢弃, 下周期再发）
 * @note    DMA 发送 ≈2µs 启动耗时, 主循环不阻塞。
 *          若上一帧 DMA 未完成则直接返回 -2, 调用方可忽略。
 */
int JF_SendFrame(void *huart, const float *data, size_t channels)
{
    UART_HandleTypeDef *h = (UART_HandleTypeDef *)huart;
    uint8_t  *buf;
    size_t    len;

    if (h == NULL || data == NULL || channels == 0U || channels > JF_MAX_CHANNELS) {
        return -1;
    }

    /* DMA 忙则丢弃本帧, 不等待 */
    if (h->gState != HAL_UART_STATE_READY) {
        return -2;
    }

    /* 切换缓冲 */
    jf_tx_idx = (jf_tx_idx == 0U) ? 1U : 0U;
    buf = jf_tx_buf[jf_tx_idx];

    len = JF_BuildFrame(buf, data, channels);
    if (len == 0U) {
        return -1;
    }

    return (int)HAL_UART_Transmit_DMA(h, buf, (uint16_t)len);
}

/*==========================================================================*/
/* JF_SendRaw                                                                */
/*==========================================================================*/

int JF_SendRaw(void *huart, const uint8_t *frame, size_t frame_len)
{
    UART_HandleTypeDef *h = (UART_HandleTypeDef *)huart;

    if (h == NULL || frame == NULL || frame_len == 0U) {
        return -1;
    }
    if (h->gState != HAL_UART_STATE_READY) {
        return -2;
    }
    return (int)HAL_UART_Transmit_DMA(h, frame, (uint16_t)frame_len);
}

/*==========================================================================*/
/* JF_ParseFrame                                                             */
/*==========================================================================*/

size_t JF_ParseFrame(const uint8_t *raw, size_t raw_len,
                     float *out, size_t max_ch)
{
    size_t scan_pos;
    size_t ch;

    if (raw == NULL || out == NULL || raw_len < JF_TAIL_SIZE || max_ch == 0U) {
        return 0U;
    }

    /* 从尾部向前扫描帧尾 [00 00 80 7F] */
    for (scan_pos = 0U; scan_pos + JF_TAIL_SIZE <= raw_len; scan_pos++) {
        if (raw[scan_pos + 0U] == jf_tail[0] &&
            raw[scan_pos + 1U] == jf_tail[1] &&
            raw[scan_pos + 2U] == jf_tail[2] &&
            raw[scan_pos + 3U] == jf_tail[3]) {

            /* 帧尾之前的数据长度必须是 4 的倍数 */
            if ((scan_pos % JF_FLOAT_SIZE) != 0U) {
                continue;
            }

            ch = scan_pos / JF_FLOAT_SIZE;
            if (ch == 0U || ch > max_ch) {
                continue;
            }

            /* 逐通道解析 float */
            for (size_t i = 0U; i < ch; i++) {
                uint32_t raw_val;
                size_t   off = i * JF_FLOAT_SIZE;

                raw_val  = (uint32_t)raw[off];
                raw_val |= (uint32_t)raw[off + 1U] << 8U;
                raw_val |= (uint32_t)raw[off + 2U] << 16U;
                raw_val |= (uint32_t)raw[off + 3U] << 24U;

                memcpy(&out[i], &raw_val, sizeof(float));
            }

            return ch;
        }
    }

    return 0U;
}

/*==========================================================================*/
/* JF_GetTail                                                                */
/*==========================================================================*/

void JF_GetTail(uint8_t out[4])
{
    if (out != NULL) {
        memcpy(out, jf_tail, JF_TAIL_SIZE);
    }
}
