/**
******************************************************************************
* @file     drv_spi_as5048a.h
* @author   lidongyang
* @version  0.0.1
* @date     22-June-2026
* @brief    AS5048A 14 位磁旋转编码器 SPI 驱动
*           引脚: PB3=SCK  PB4=MISO  PB5=MOSI  CS=PA15（软件 NSS）
*
* @note     SPI 配置由 CubeMX 生成（Master / Full Duplex / 16-bit
*           CPOL=Low / CPHA=2Edge / Prescaler=16 / MSB first）
*           与 AS5048A 芯片要求完全一致, 无需手动修改 SPI 初始化代码。
*
*           用法：
*             main.c → drv_as5048a_init();
*             TIM 中断 / 主循环 → drv_as5048a_read_angle();
******************************************************************************
*/

#ifndef DRV_SPI_AS5048A_H
#define DRV_SPI_AS5048A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/*==========================================================================*/
/* AS5048A 寄存器命令（芯片协议常量）                                         */
/*==========================================================================*/

/** @brief 空操作（用于取回上一帧命令的数据） */
#define CMD_NOP            0x0000U

/** @brief 清除错误标志（bit14 锁存, 必须发此命令才能恢复） */
#define CMD_CLEAR_ERROR    0x0001U

/** @brief 磁场强度（诊断磁铁间距是否合适） */
#define CMD_READ_MAG       0x3FFEU

/** @brief 诊断信息（AGC/溢出等） */
#define CMD_READ_DIAG      0x3FFDU

/** @brief 角度值（14-bit, 0~16383） */
#define CMD_ANGLE          0x3FFFU

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   AS5048A 驱动初始化
 * @note    必须在 MX_SPI1_Init 之后调用一次, 等待芯片上电稳定（1ms）
 */
void     drv_as5048a_init(void);

/**
 * @brief   读取角度值
 * @return  14-bit 角度原始值（0 ~ 16383）
 *          通信异常时返回 0, 可通过 drv_as5048a_get_error() 查询错误状态
 * @note    单次调用耗时约 6µs（2 帧 SPI 传输）
 */
uint16_t drv_as5048a_read_angle(void);

/**
 * @brief   读取磁场强度（诊断磁铁间距是否合适）
 * @return  磁场强度原始值（14-bit, 0 ~ 16383）
 *          数值越大表示磁铁距离越近/对中越好
 *          通信异常时返回 0, 可通过 drv_as5048a_get_error() 查询错误状态
 * @note    用于装配诊断, 判断磁铁是否在有效感应范围内
 */
uint16_t drv_as5048a_read_magnitude(void);

/**
 * @brief   读取诊断信息
 * @return  诊断寄存器值（14-bit）
 *          通信异常时返回 0, 可通过 drv_as5048a_get_error() 查询错误状态
 * @note    包含 AGC 增益、磁场过强/过弱指示等状态位
 */
uint16_t drv_as5048a_read_diag(void);

/** @brief 读取任意寄存器
 *  @param reg_addr  寄存器地址（低 14-bit 有效, 使用 CMD_xxx 宏）
 *  @return 寄存器值（14-bit 数据），出错返回 0
 *  @note  通信异常时 as5048a_error_flag = 1, 可通过 drv_as5048a_get_error() 查询
 */
uint16_t drv_as5048a_read_reg(uint16_t reg_addr);

/** @brief 写入任意寄存器
 *  @param reg_addr  寄存器地址
 *  @param value     写入的 14-bit 数据
 */
void     drv_as5048a_write_reg(uint16_t reg_addr, uint16_t value);

/** @brief 获取通信错误状态
 *  @return 0 = 正常, 非 0 = 最近一次传输有误（偶校验失败/芯片报告错误）
 */
uint8_t  drv_as5048a_get_error(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_SPI_AS5048A_H */
