/**
******************************************************************************
* @file     drv_spi_as5048a_debug.c
* @author   lidongyang
* @version  0.0.1
* @date     23-June-2026
* @brief    AS5048A 驱动调试与验证模块实现
*
* @note     在 while(1) 中循环调用 drv_as5048a_debug_run_tests(),
*           自动测试所有启用的 API 函数, 通过三种方式反馈:
*           1. GPIOA_PIN_4 脉冲宽度 → 函数耗时（示波器/逻辑分析仪）
*           2. 全局变量 test_xxx / test_cycle_xxx → 调试器查看
*           3. UART1(115200) 表格输出 → 串口助手查看
******************************************************************************
*/

#include "drv_spi_as5048a_debug.h"
#include "drv_spi_as5048a.h"
#include "gpio.h"
#include "usart.h"
#include <stdio.h>

/*==========================================================================*/
/* 测试结果变量（供外部调试器观察）                                           */
/*==========================================================================*/

volatile uint16_t test_angle       = 0;
volatile uint16_t test_mag         = 0;
volatile uint16_t test_diag        = 0;
volatile uint16_t test_reg         = 0;
volatile uint8_t  test_err         = 0;

volatile uint32_t test_cycle_angle     = 0;
volatile uint32_t test_cycle_mag       = 0;
volatile uint32_t test_cycle_diag      = 0;
volatile uint32_t test_cycle_read_reg  = 0;
volatile uint32_t test_cycle_write_reg = 0;
volatile uint32_t test_cycle_get_err   = 0;

/*==========================================================================*/
/* 模块内部变量                                                              */
/*==========================================================================*/

#if (TEST_AS5048A_ENABLE == 1)
/** DWT 临时计时变量 */
static uint32_t test_start = 0;
#endif

#if (TEST_UART_ENABLE == 1)
/** 串口发送缓冲区 */
static uint8_t uart_buf[640];

/** 测试轮次计数器 */
static uint32_t test_round = 0;

/**
 * @brief 编译时统计已启用的测试函数数量
 * @note  用于自动切换输出格式：
 *         =1 → 单函数连续流模式（无表头表尾）
 *         >1 → 多函数表格模式
 */
#define TEST_ENABLED_COUNT  (TEST_ENABLE_READ_ANGLE + \
                             TEST_ENABLE_READ_MAG   + \
                             TEST_ENABLE_READ_DIAG  + \
                             TEST_ENABLE_READ_REG   + \
                             TEST_ENABLE_WRITE_REG  + \
                             TEST_ENABLE_GET_ERROR)
#endif

/*==========================================================================*/
/* API 实现                                                                  */
/*==========================================================================*/

void drv_as5048a_debug_init(void)
{
    /* DWT 初始化已移至 drv_as5048a_init()，本函数保留为空接口兼容 */
}

void drv_as5048a_debug_run_tests(void)
{
#if (TEST_AS5048A_ENABLE == 1)

    /* ---- 测试 1：drv_as5048a_read_angle() ---- */
#if (TEST_ENABLE_READ_ANGLE == 1)
    test_start = DWT_GET_CYCLES();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    test_angle = drv_as5048a_read_angle();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    test_cycle_angle = DWT_GET_CYCLES() - test_start;
#endif

    /* ---- 测试 2：drv_as5048a_read_magnitude() ---- */
#if (TEST_ENABLE_READ_MAG == 1)
    test_start = DWT_GET_CYCLES();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    test_mag = drv_as5048a_read_magnitude();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    test_cycle_mag = DWT_GET_CYCLES() - test_start;
#endif

    /* ---- 测试 3：drv_as5048a_read_diag() ---- */
#if (TEST_ENABLE_READ_DIAG == 1)
    test_start = DWT_GET_CYCLES();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    test_diag = drv_as5048a_read_diag();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    test_cycle_diag = DWT_GET_CYCLES() - test_start;
#endif

    /* ---- 测试 4：drv_as5048a_read_reg(CMD_READ_MAG) ---- */
#if (TEST_ENABLE_READ_REG == 1)
    test_start = DWT_GET_CYCLES();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    test_reg = drv_as5048a_read_reg(CMD_READ_MAG);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    test_cycle_read_reg = DWT_GET_CYCLES() - test_start;
#endif

    /* ---- 测试 5：drv_as5048a_write_reg() ---- */
#if (TEST_ENABLE_WRITE_REG == 1)
    test_start = DWT_GET_CYCLES();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    drv_as5048a_write_reg(CMD_CLEAR_ERROR, 0x0001U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    test_cycle_write_reg = DWT_GET_CYCLES() - test_start;
#endif

    /* ---- 测试 6：drv_as5048a_get_error() ---- */
#if (TEST_ENABLE_GET_ERROR == 1)
    test_start = DWT_GET_CYCLES();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    test_err = drv_as5048a_get_error();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    test_cycle_get_err = DWT_GET_CYCLES() - test_start;
#endif

    /* ---- 串口输出测试结果（自动适配单/多函数模式） ---- */
#if (TEST_UART_ENABLE == 1)
    test_round++;
    {
        int32_t len;
        uint16_t pos = 0;

#if (TEST_ENABLED_COUNT > 1)
        /* ======== 多函数模式：带表头表尾的表格 ======== */
        len = snprintf((char *)uart_buf, (size_t)(sizeof(uart_buf) - pos),
            "\r\n========== AS5048A Test Round %lu ==========\r\n",
            (unsigned long)test_round);
        if (len > 0) pos += (uint16_t)len;
#endif /* TEST_ENABLED_COUNT > 1 */

        /* ---- 逐函数输出（单函数模式无表头表尾，纯数据流） ---- */
#if (TEST_ENABLE_READ_ANGLE == 1)
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            " %-28s = %5u  |  cycles=%5lu  |  ~%lu us\r\n",
            "1) drv_as5048a_read_angle()", test_angle,
            (unsigned long)test_cycle_angle, (unsigned long)CYCLES_TO_US(test_cycle_angle));
        if (len > 0) pos += (uint16_t)len;
#endif

#if (TEST_ENABLE_READ_MAG == 1)
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            " %-28s = %5u  |  cycles=%5lu  |  ~%lu us\r\n",
            "2) drv_as5048a_read_magnitude()", test_mag,
            (unsigned long)test_cycle_mag, (unsigned long)CYCLES_TO_US(test_cycle_mag));
        if (len > 0) pos += (uint16_t)len;
#endif

#if (TEST_ENABLE_READ_DIAG == 1)
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            " %-28s = %5u  |  cycles=%5lu  |  ~%lu us\r\n",
            "3) drv_as5048a_read_diag()", test_diag,
            (unsigned long)test_cycle_diag, (unsigned long)CYCLES_TO_US(test_cycle_diag));
        if (len > 0) pos += (uint16_t)len;
#endif

#if (TEST_ENABLE_READ_REG == 1)
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            " %-28s = %5u  |  cycles=%5lu  |  ~%lu us\r\n",
            "4) drv_as5048a_read_reg()", test_reg,
            (unsigned long)test_cycle_read_reg, (unsigned long)CYCLES_TO_US(test_cycle_read_reg));
        if (len > 0) pos += (uint16_t)len;
#endif

#if (TEST_ENABLE_WRITE_REG == 1)
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            " %-28s |  cycles=%5lu  |  ~%lu us\r\n",
            "5) drv_as5048a_write_reg() (clear err)",
            (unsigned long)test_cycle_write_reg, (unsigned long)CYCLES_TO_US(test_cycle_write_reg));
        if (len > 0) pos += (uint16_t)len;
#endif

#if (TEST_ENABLE_GET_ERROR == 1)
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            " %-28s = %5u  |  cycles=%5lu  |  ~%lu us\r\n",
            "6) drv_as5048a_get_error()", test_err,
            (unsigned long)test_cycle_get_err, (unsigned long)CYCLES_TO_US(test_cycle_get_err));
        if (len > 0) pos += (uint16_t)len;
#endif

#if (TEST_ENABLED_COUNT > 1)
        /* 表尾（仅多函数模式） */
        len = snprintf((char *)(uart_buf + pos), (size_t)(sizeof(uart_buf) - pos),
            "============================================\r\n");
        if (len > 0) pos += (uint16_t)len;
#endif

        if (pos > 0)
        {
            HAL_UART_Transmit(&huart1, uart_buf, pos, 100U);
        }
    }
#endif /* TEST_UART_ENABLE */

#endif /* TEST_AS5048A_ENABLE */
}
