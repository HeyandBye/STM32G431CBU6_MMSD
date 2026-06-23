/**
******************************************************************************
* @file     drv_spi_as5048a_debug.h
* @author   lidongyang
* @version  0.0.1
* @date     23-June-2026
* @brief    AS5048A 驱动调试与验证模块
*
* @note     提供 DWT 周期计数器宏 + 逐函数可用性/耗时测试框架。
*           所有开关宏集中在本文件, 方便一键启停。
*
*           使用方法:
*           1. main.c 中 #include "drv_spi_as5048a_debug.h"
*           2. 调用 drv_as5048a_init()（内部已初始化 DWT）
*           3. while(1) 中调用 drv_as5048a_debug_run_tests()
*           4. 各测试开关见下方 TEST_ENABLE_xxx 宏
*
*           观测方式:
*            - GPIO 波形: GPIOA_PIN_4 脉冲宽度 = 函数耗时
*            - 变量观察:  test_xxx / test_cycle_xxx（调试器查看）
*            - 串口输出:  UART1(115200) 打印结果表格
******************************************************************************
*/

#ifndef DRV_SPI_AS5048A_DEBUG_H
#define DRV_SPI_AS5048A_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/*==========================================================================*/
/* DWT 周期计数器宏                                                          */
/*==========================================================================*/

/** @brief 初始化 DWT 周期计数器（由 drv_as5048a_init() 自动调用，无需手动执行） */
#define DWT_INIT()         do{ CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
                               DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; }while(0)

/** @brief 读取当前 DWT 周期计数值 */
#define DWT_GET_CYCLES()   (DWT->CYCCNT)

/** @brief CPU 频率（MHz），与 SystemClock_Config 保持一致                   */
#ifndef CPU_FREQ_MHZ
#define CPU_FREQ_MHZ       170U
#endif

/** @brief 将 CPU 周期数转换为微秒 */
#define CYCLES_TO_US(cyc)  ((cyc) / CPU_FREQ_MHZ)

/*==========================================================================*/
/* 测试开关宏                                                                */
/*==========================================================================*/

/** @brief 总开关：1 = 启用所有 AS5048A 测试；0 = 完全跳过 */
#ifndef TEST_AS5048A_ENABLE
#define TEST_AS5048A_ENABLE      1
#endif

/** @brief 串口开关：1 = UART1 打印结果表格；0 = 仅 GPIO + 变量 */
#ifndef TEST_UART_ENABLE
#define TEST_UART_ENABLE         1
#endif

/** @brief 逐函数测试开关 */
#ifndef TEST_ENABLE_READ_ANGLE
#define TEST_ENABLE_READ_ANGLE    1   /* drv_as5048a_read_angle()     */
#endif
#ifndef TEST_ENABLE_READ_MAG
#define TEST_ENABLE_READ_MAG      0   /* drv_as5048a_read_magnitude() */
#endif
#ifndef TEST_ENABLE_READ_DIAG
#define TEST_ENABLE_READ_DIAG     0   /* drv_as5048a_read_diag()      */
#endif
#ifndef TEST_ENABLE_READ_REG
#define TEST_ENABLE_READ_REG      0   /* drv_as5048a_read_reg()       */
#endif
#ifndef TEST_ENABLE_WRITE_REG
#define TEST_ENABLE_WRITE_REG     0   /* drv_as5048a_write_reg()      */
#endif
#ifndef TEST_ENABLE_GET_ERROR
#define TEST_ENABLE_GET_ERROR     0   /* drv_as5048a_get_error()      */
#endif

/*==========================================================================*/
/* 外部变量 — 测试结果（可在调试器中观察）                                    */
/*==========================================================================*/

/** @name 函数返回值 */
/** @{ */
extern volatile uint16_t test_angle;       /**< drv_as5048a_read_angle()     */
extern volatile uint16_t test_mag;         /**< drv_as5048a_read_magnitude() */
extern volatile uint16_t test_diag;        /**< drv_as5048a_read_diag()      */
extern volatile uint16_t test_reg;         /**< drv_as5048a_read_reg()       */
extern volatile uint8_t  test_err;         /**< drv_as5048a_get_error()      */
/** @} */

/** @name 函数耗时（单位：CPU 周期） */
/** @{ */
extern volatile uint32_t test_cycle_angle;
extern volatile uint32_t test_cycle_mag;
extern volatile uint32_t test_cycle_diag;
extern volatile uint32_t test_cycle_read_reg;
extern volatile uint32_t test_cycle_write_reg;
extern volatile uint32_t test_cycle_get_err;
/** @} */

/*==========================================================================*/
/* API                                                                      */
/*==========================================================================*/

/**
 * @brief   初始化调试模块（启用 DWT 周期计数器）
 * @note    在 drv_as5048a_init() 之后、while(1) 之前调用一次
 */
void drv_as5048a_debug_init(void);

/**
 * @brief   运行一轮 AS5048A 全部功能验证测试
 * @note    在 while(1) 中循环调用。
 *          受 TEST_ENABLE_xxx 宏控制，只执行启用的测试项。
 *          每个函数执行时 GPIOA_PIN_4 输出高电平脉冲，脉宽 = 耗时。
 *          若 TEST_UART_ENABLE=1，每轮结束后通过 UART1 打印结果。
 */
void drv_as5048a_debug_run_tests(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_SPI_AS5048A_DEBUG_H */
