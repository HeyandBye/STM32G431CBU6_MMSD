/**
******************************************************************************
* @file     drv_spi_as5048a.c
* @author   lidongyang
* @version  0.0.1
* @date     22-June-2026
* @brief    AS5048A 14 位磁旋转编码器 SPI 驱动实现
*           引脚: PB3=SCK  PB4=MISO  PB5=MOSI  CS=PA15（软件 NSS）
*
* @note     设计决策：
*           1. 直接操作 SPI1 寄存器（非 HAL/非 DMA）：
*              - 每帧仅 16-bit（~1.5µs @ PCLK/16）, HAL 调用开销占比过高
*              - 直接读写 SPI1->DR + 轮询 SR 标志, 消除 HAL 函数栈/锁/超时检查开销
*              - AS5048A 要求帧间切换 CS, DMA 无法自动完成
*              - 芯片流水线协议要求拿到当前帧返回值才能走下一步
 *           2. CS 时序：
 *              - CS 建立时间、保持时间和帧间隔均 ≥ 350ns
 *              - 用 DWT->CYCCNT 精确延时实现, ±1 CPU 周期精度
 *           3. 上电稳定：HAL_Delay(1) 足够
 *           4. CubeMX 生成的 DMA/中断基础设施原封不动保留
 *           5. DWT 周期计数器由 drv_as5048a_init() 内部自动使能
******************************************************************************
*/

#include "drv_spi_as5048a.h"
#include "spi.h"

/*==========================================================================*/
/* 硬件抽象层 —— CS 控制                                                     */
/*==========================================================================*/

/** @brief CS 引脚：PA15（推挽输出 + 上拉, 低有效） */
#define AS5048A_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define AS5048A_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

/*==========================================================================*/
/* 硬件抽象层 —— 纳秒级延时                                                   */
/*==========================================================================*/

/**
 * @brief   微延时（约 360ns @ 170MHz）
 * @note    基于 DWT->CYCCNT 周期计数器实现，精度 ±1 CPU 周期。
 *          170MHz 下 1 周期 ≈ 5.88ns，等待约 50 周期 ≈ 294ns，
 *          加上函数调用与循环开销后实际约 350~400ns，
 *          满足 AS5048A CS 建立/保持时间 ≥ 350ns（typ.）要求。
 *
 * @note    DWT 周期计数器在 drv_as5048a_init() 中使能, 无需额外初始化。
 */
static inline void AS5048A_Delay360ns(void)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < 50U) { }
}

/*==========================================================================*/
/* 硬件抽象层 —— 单帧 SPI 收发                                               */
/*==========================================================================*/

/**
 * @brief   通过 SPI1 收发一个 16-bit 帧（寄存器版, ~1.8µs @ PCLK/16）
 * @param   TxData  待发送的 16-bit 数据（命令或写入值）
 * @return  接收到的 16-bit 数据（芯片返回的应答帧）
 *
 * @note    直接操作 SPI1 寄存器, 绕过 HAL, 消除 HAL 函数调用开销。
 *          执行顺序:
 *            CS 拉低 → 延时 → 写 DR(启动传输) → 等 RXNE → 读 DR
 *            → 等 BSY → 延时 → CS 拉高 → 延时
 *
 *          相比 HAL_SPI_TransmitReceive 省去了:
 *            - 参数校验/状态机/锁
 *            - while 循环中每次调用 HAL_GetTick()
 *            - SPI_EndRxTxTransaction() 中额外 FIFO/BSY 轮询
 *
 *          AS5048A 用 CS 上升沿触发内部 ADC 采样,
 *          因此每帧之间必须完整切换 CS（拉低→传完→拉高）。
 *          不能像 MT6816 那样连续发多字节不切 CS。
 */
static uint16_t AS5048A_SPI_WriteByte(uint16_t TxData)
{
    uint16_t rxData;

    AS5048A_CS_LOW();
    AS5048A_Delay360ns();   /* CS 建立时间 ≥ 350ns */

    /* 确保 SPI 外设已使能（HAL 在首次传输时自动设置，寄存器版需手动保证）*/
    if ((SPI1->CR1 & SPI_CR1_SPE) == 0U) {
        SPI1->CR1 |= SPI_CR1_SPE;
    }

    /* 等待 TXE（发送缓冲空）→ 可写入新数据 */
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    /* 写 DR — 自动启动 16-bit SPI 传输 */
    SPI1->DR = TxData;
    /* 等待 RXNE（接收缓冲非空）→ 传输完成 */
    while (!(SPI1->SR & SPI_SR_RXNE)) { }
    /* 读 DR — 硬件自动清除 RXNE 标志 */
    rxData = (uint16_t)SPI1->DR;
    /* 等待 BSY 清除 → 最后一帧 SCK 结束 */
    while (SPI1->SR & SPI_SR_BSY) { }

    AS5048A_Delay360ns();   /* CS 保持时间 ≥ 350ns */
    AS5048A_CS_HIGH();
    AS5048A_Delay360ns();   /* 帧间隔 ≥ 350ns */

    return rxData;
}

/*==========================================================================*/
/* 协议层 —— 偶校验                                                          */
/*==========================================================================*/

/**
 * @brief   计算 16-bit 数据的偶校验位
 * @param   v   待计算的 16-bit 数据
 * @return  偶校验位（0 = 1 的个数为偶数, 1 = 1 的个数为奇数）
 *
 * @note    芯片要求每帧 bit15 为偶校验位（1 的个数为偶数 → 0, 奇数 → 1）
 *          算法：16-bit 逐级异或折叠, 末位 = 所有位异或结果
 *
 *          例: v=0x4003 → 100 0000 0000 0011 → 3 个 1（奇数）→ 返回 1
 */
static uint8_t parity_even(uint16_t v)
{
    if (v == 0U) return 0U;
    v ^= v >> 8U;
    v ^= v >> 4U;
    v ^= v >> 2U;
    v ^= v >> 1U;
    return (uint8_t)(v & 1U);
}

/*==========================================================================*/
/* 协议层 —— 通信错误标志                                                     */
/*==========================================================================*/

/** @brief 通信错误标志（0 = 正常, 非 0 = 有误）
 *  @note  在每次 drv_as5048a_read_reg / drv_as5048a_write_reg 后更新 */
static uint8_t as5048a_error_flag = 0U;

/**
 * @brief   获取通信错误状态
 * @return  0 = 最近一次通信正常
 *          非 0 = 偶校验失败或芯片报告错误（需发送 CMD_CLEAR_ERROR 清除锁存）
 * @note    在每次 drv_as5048a_read_reg / drv_as5048a_write_reg 后更新,
 *          可通过此函数查询芯片通信是否正常。
 */
uint8_t drv_as5048a_get_error(void)
{
    return as5048a_error_flag;
}

/*==========================================================================*/
/* 协议层 —— 读寄存器                                                         */
/*==========================================================================*/

/**
 * @brief   读取 AS5048A 寄存器
 * @param   reg_addr  寄存器地址（低 14-bit 有效, 使用 CMD_xxx 宏）
 * @return  寄存器值（14-bit 数据），出错返回 0
 * @note    为什么是两帧？
 *          芯片采用流水线架构——CS 上升沿触发内部 ADC 采样。
 *          第 1 帧发读命令（此时收到的是上一帧的无效数据），
 *          第 2 帧发 NOP（此时收到的才是第 1 帧命令对应的真实数据）。
 *
 *          为什么检查 bit14？
 *          芯片用 bit14 报告通信错误（偶校验失败/无效命令/溢出）。
 *          错误标志是锁存的——不清除则后续所有帧持续报错，
 *          必须发送 CMD_CLEAR_ERROR 清除。
 *
 *          命令格式（16-bit, MSB first）:
 *          | 15:Parity | 14:R/W | 13:0:Register Address |
 *          R/W=1 表示读, =0 表示写
 *          寄存器地址使用 CMD_xxx 宏（已包含正确的 bit14）
 */
uint16_t drv_as5048a_read_reg(uint16_t reg_addr)
{
    uint16_t data = 0U;
    uint16_t res;
    uint16_t cmd;

    /* ---- 帧 1：发送读命令（bit14=1 表示读, 带偶校验） ---- */
    cmd  = 0x4000U | reg_addr;
    cmd |= (uint16_t)(parity_even(cmd) << 15U);
    (void)AS5048A_SPI_WriteByte(cmd);

    /* ---- 帧 2：发送 NOP, 取回帧 1 命令对应的数据 ---- */
    cmd  = 0x4000U | CMD_NOP;
    cmd |= (uint16_t)(parity_even(cmd) << 15U);
    res  = AS5048A_SPI_WriteByte(cmd);

    as5048a_error_flag = 1U;  /* 默认假设有误, 下面校验通过再清零 */

    if ((res & (1U << 14U)) == 0U) {
        /* bit14=0：通信正常, 提取 14-bit 数据并校验偶校验位 */
        data = res & 0x3FFFU;
        as5048a_error_flag = (uint8_t)(parity_even(data) ^ (res >> 15U));
    } else {
        /* bit14=1：芯片报告错误, 清除错误标志 */
        cmd  = 0x4000U | CMD_CLEAR_ERROR;
        cmd |= (uint16_t)(parity_even(cmd) << 15U);
        (void)AS5048A_SPI_WriteByte(cmd);
    }
    return data;
}

/*==========================================================================*/
/* 协议层 —— 写寄存器                                                         */
/*==========================================================================*/

/**
 * @brief   写入 AS5048A 寄存器
 * @param   reg_addr  寄存器地址（低 14-bit 有效, bit14=0 表示写操作）
 *                    注意：与读操作不同, 写命令不需要 bit14=1,
 *                    此处 reg_addr 直接作为写命令帧发送（bit14 保持为 0）。
 *                    可使用 CMD_CLEAR_ERROR 等可写寄存器地址宏。
 * @param   value     待写入的 14-bit 数据（写入前会自动钳位到低 14-bit）
 *
 * @note    三帧协议：
 *          帧 1: 发送写命令（reg_addr, bit14=0, 带偶校验）
 *          帧 2: 发送待写入的数据（value & 0x3FFF, 带偶校验）
 *          帧 3: 发送 NOP, 取回芯片对帧 2 的应答（检查 bit14 错误标志）
 *
 *          注意：写操作与读操作使用不同的帧协议——写操作共 3 帧,
 *          而读操作为 2 帧（发读命令 → 发 NOP 取数据）。
 *
 *          错误检查逻辑与读寄存器完全一致。
 */
void drv_as5048a_write_reg(uint16_t reg_addr, uint16_t value)
{
    uint16_t data;
    uint16_t res;
    uint16_t cmd;

    /* ---- 帧 1：写命令（bit14=0） ---- */
    cmd  = reg_addr;
    cmd |= (uint16_t)(parity_even(cmd) << 15U);
    (void)AS5048A_SPI_WriteByte(cmd);

    /* ---- 帧 2：写入数据 ---- */
    cmd  = value & 0x3FFFU;
    cmd |= (uint16_t)(parity_even(cmd) << 15U);
    (void)AS5048A_SPI_WriteByte(cmd);

    /* ---- 帧 3：NOP 确认 ---- */
    cmd  = 0x4000U | CMD_NOP;
    cmd |= (uint16_t)(parity_even(cmd) << 15U);
    res  = AS5048A_SPI_WriteByte(cmd);

    as5048a_error_flag = 1U;
    if ((res & (1U << 14U)) == 0U) {
        data = res & 0x3FFFU;
        as5048a_error_flag = (uint8_t)(parity_even(data) ^ (res >> 15U));
    } else {
        cmd  = 0x4000U | CMD_CLEAR_ERROR;
        cmd |= (uint16_t)(parity_even(cmd) << 15U);
        (void)AS5048A_SPI_WriteByte(cmd);
    }
}

/*==========================================================================*/
/* 应用层 —— 便捷 API                                                        */
/*==========================================================================*/

/**
 * @brief   读取角度值（0~16383, 14-bit 原始值）
 * @return  14-bit 角度原始值（0 ~ 16383）
 *          通信异常时返回 0, 可通过 drv_as5048a_get_error() 确认
 * @note    在 20kHz TIM 中断或主循环里直接调用此函数。
 *          单次调用耗时约 10~12µs（寄存器版 SPI，待优化），
 *          在 50µs 周期（20kHz）中占比约 20~24%。
 */
uint16_t drv_as5048a_read_angle(void)
{
    uint16_t val = drv_as5048a_read_reg(CMD_ANGLE);
    if (as5048a_error_flag != 0U) {
        return 0U;
    }
    return val;
}

/**
 * @brief   读取磁场强度（诊断磁铁间距是否合适）
 * @return  磁场强度原始值（14-bit, 0 ~ 16383）
 *          数值越大表示磁铁距离越近/对中越好
 *          通信异常时返回 0, 可通过 drv_as5048a_get_error() 确认
 * @note    返回值越小表示磁铁越远/越偏, 可用于装配诊断
 */
uint16_t drv_as5048a_read_magnitude(void)
{
    uint16_t val = drv_as5048a_read_reg(CMD_READ_MAG);
    if (as5048a_error_flag != 0U) {
        return 0U;
    }
    return val;
}

/**
 * @brief   读取诊断信息（AGC/溢出等）
 * @return  诊断寄存器值（14-bit）
 *          通信异常时返回 0, 可通过 drv_as5048a_get_error() 确认
 * @note    各 bit 含义参考 AS5048A 数据手册：
 *          bit13 = AGC 溢出指示
 *          bit12 = 磁场过弱指示
 *          bit11 = 磁场过强指示
 *          低 8 位为 AGC 增益值
 */
uint16_t drv_as5048a_read_diag(void)
{
    uint16_t val = drv_as5048a_read_reg(CMD_READ_DIAG);
    if (as5048a_error_flag != 0U) {
        return 0U;
    }
    return val;
}

/*==========================================================================*/
/* 初始化                                                                    */
/*==========================================================================*/

/**
 * @brief   AS5048A 驱动初始化
 *
 * @note    SPI1 已由 CubeMX 生成的 MX_SPI1_Init() 完成配置：
 *            Master / Full Duplex / 16-bit / CPOL=Low / CPHA=2Edge
 *            Prescaler=16 / MSB first / Software NSS
 *          以上参数与 AS5048A 芯片要求完全一致, 此处无需重复配置。
 *
 *          执行顺序：
 *          1. 使能 DWT 周期计数器（AS5048A_Delay360ns 依赖此功能）
 *          2. CS 拉高（芯片未选中, 默认状态）
 *          3. 等待 1ms 芯片上电稳定
 */
void drv_as5048a_init(void)
{
    /* 使能 DWT 周期计数器（供 AS5048A_Delay360ns 使用）*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    AS5048A_CS_HIGH();
    HAL_Delay(1U);   /* 等待芯片上电稳定 */
}
