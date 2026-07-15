# STM32G431CBU6_MMSD — 无刷电机 FOC 矢量控制

> **版本**: v0.1.0 |
> **MCU**: STM32G431CBU6 (Cortex-M4 @ 170MHz) |
> **驱动**: DRV8313 |
> **编码器**: AS5048A 14-bit SPI |
> **电流传感**: TMCS1107A3B (200mV/A) |
> **电机**: iPower GM3506 (24N22P / 11对极)

> [🇬🇧 English](README_EN.md)

---

## 1. 项目简介

基于 STM32G431CBU6 的 **磁场定向控制（FOC）** 工程，采用 **位置-速度-电流三环串级控制**。

**四种运行模式**：

| 模式 | 说明 |
|------|------|
| 电流闭环 | Id=0 策略，双 PI 控制 |
| 速度闭环 | 速度 PI 外环 + 电流内环 |
| 位置闭环 | 位置 PID 外环 + 速度/电流内环，支持自动步进和手动示教 |
| 阻尼模式 | 纯比例负反馈，转动有阻力、松手自由停 |

---

## 2. 软件分层

```
main.c (HAL 硬件初始化骨架)
  └── ApplicationLayer    应用层 — 系统初始化、主循环任务、模式管理
        ├── CommunicationLayer  通信层 — VOFA+ JustFloat 在线调参
        ├── ControlLayer        控制层 — FOC 算法（坐标变换/PID/SVPWM/环路控制）
        └── DriverLayer         驱动层 — 外设封装（ADC/PWM/SPI/nFAULT）
```

### 2.1 驱动层

| 模块 | 功能 |
|------|------|
| `drv_adc_sampling` | ADC1/ADC2 双通道 DMA 循环采样，IIR 低通滤波 |
| `drv_spi_as5048a` | AS5048A 编码器 SPI 驱动，寄存器直操作，3次重试 |
| `drv_tim_pwm` | TIM1 六路互补 PWM，20kHz，死区可配 |
| `drv_nfault` | DRV8313 nFAULT 中断，紧急关断 MOE |

### 2.2 控制层

| 模块 | 功能 |
|------|------|
| `ctl_math` | Clarke / Park / InvPark 坐标变换 + SVPWM 七段式调制 |
| `ctl_pid` | 并行式 PID + setpoint 加权 + 条件积分抗饱和 + 微分 IIR 低通 |
| `ctl_foc_core` | FOC 主结构体、状态机、全局实例、系统入口 |
| `ctl_foc_current` | 电流闭环 — Id=0 策略，编码器校准 |
| `ctl_foc_speed` | 速度闭环 — 编码器差分测速 |
| `ctl_foc_position` | 位置闭环 — 编码器展开（无回绕） |
| `ctl_foc_damper` | 阻尼模式 — Iq = -gain × speed |
| `ctl_foc_openloop` | 开环 — 虚拟旋转磁场，硬件链路验证 |

### 2.3 应用层 &amp; 通信层

| 模块 | 功能 |
|------|------|
| `app_foc` | `App_Init()` / `App_Run()` — LED 心跳、故障恢复、模式逻辑 |
| `prot_justfloat` | JustFloat 二进制帧构建/解析 |
| `diag_tuning` | UART IDLE + DMA 双向通信，在线修改 PID / 给定 / 模式 |

---

## 3. 编译期配置

### 启动模式 (`ctl_foc_core.h`)

```c
#define FOC_MODE  FOC_MODE_POSITION   /* 默认: 位置闭环 */
```

可选值: `FOC_MODE_OPENLOOP`(1) / `FOC_MODE_CURRENT`(2) / `FOC_MODE_SPEED`(3) / `FOC_MODE_POSITION`(4) / `FOC_MODE_DAMPER`(5)

### 位置控制模式 (`app_foc.h`)

```c
#define POS_AUTO_STEP  0   /* 1=自动步进(每1s转6°), 0=手动示教 */
```

### 在线调参 (`diag_tuning.h`)

```c
#define TUNING_ENABLE         1   /* 1=启用 VOFA+ 在线调参 */
#define TUNING_SEND_PERIOD_MS 5U  /* 数据发送周期 (ms)     */
```

### 速度环目标 (`app_foc.h`)

```c
#define APP_SPEED_RPM  60.0f   /* 仅 FOC_MODE_SPEED 生效 */
```

---

## 4. 引脚映射 &amp; 时序

### 硬件连接

| 功能 | 引脚 | 外设 |
|------|------|------|
| PWM A/B/C 高侧 | PA8/PA9/PA10 | TIM1_CH1/2/3 |
| 相电流 Ia/Ib | PA0/PA1 | ADC1_IN1/2 |
| 母线电压/电流 | PA7/PA6 | ADC2_IN4/3 |
| 编码器 SCK/MISO/MOSI/CS | PB3/PB4/PB5/PA15 | SPI1 + GPIO |
| nFAULT | PB11 | EXTI 下降沿 |
| LED | PC6 | GPIO |

### 控制时序

```
TIM1  20kHz / 50µs   → ADC1 触发 → FOC 电流环 (Clarke→Park→PI→InvPark→SVPWM)
TIM6   1kHz / 1ms    → 速度环 PI (差分测速 → 更新 Iq_ref)
TIM7 100Hz / 10ms    → 位置环 PID (编码器展开 → 更新 speed_ref)
主循环                 → LED 1Hz 心跳 + 故障恢复 + VOFA+ 通信
```

---

## 5. 控制参数

### 电流环 (20kHz)

| 参数 | 值 | 说明 |
|------|----|------|
| 策略 | Id=0 | 最大化转矩电流比 |
| Kp (Id/Iq) | 4.0 V/A | 比例增益 |
| Ki (Id/Iq) | 0.3 | 积分增益 |
| 输出限幅 | ±Vbus (12V) | PID 输出钳位 |
| 电流滤波 | IIR α=0.3 | fc≈1.1kHz |

### 速度环 (1kHz)

| 参数 | 值 | 说明 |
|------|----|------|
| Kp | 0.002 | 比例增益 |
| Ki | 0.001 | 积分增益 |
| 输出限幅 | ±1.0 A | 钳位到 Iq_ref |
| 转速限幅 | ±2000 RPM | 硬限幅 |

### 位置环 (100Hz)

| 参数 | 值 | 说明 |
|------|----|------|
| Kp | 0.10 | 比例增益 |
| Kd | 0.03 | 微分增益（增强阻尼） |
| 输出限幅 | ±500 RPM | 钳位到 speed_ref |

---

## 6. 在线调参 (VOFA+ JustFloat)

- 波特率: **115200-8-N-1** (USART1)
- 协议: JustFloat（float32 原始字节，帧尾 `00 00 80 7F`）
- 发送周期: **5ms** (200Hz)，DMA 非阻塞
- 接收: UART IDLE + DMA

### 上行数据（MCU → VOFA+，自动按模式切换通道）

| 模式 | 通道 (共N个float + 帧尾) |
|------|--------------------------|
| CURRENT | Id_ref, Id, Iq_ref, Iq, Vd, Vq, Ia, Ib, speed_rpm, mode **(10ch)** |
| SPEED | speed_ref, speed_rpm, iq_ref, Iq, Vq, Ia, Ib, mode **(8ch)** |
| POSITION/DAMPER | pos_cmd, pos_fb, speed_ref, speed_rpm, Iq, Vq, Ia, Ib, mode **(9ch)** |

### 下行指令（VOFA+ → MCU，发送面板填 2 通道）

| 指令码 | 参数 | 说明 |
|--------|------|------|
| 1~4 | Kp_id / Ki_id / Kp_iq / Ki_iq | 调整电流环 PI |
| 5~8 | Kp / Ki / Kd / Kr | 调整速度环 PID |
| 9~12 | Kp / Ki / Kd / Kr | 调整位置环 PID |
| 13~16 | Id_ref / Iq_ref / Speed_ref / Pos_ref | 修改给定值 |
| 17 | 2/3/4/5 | 在线切换控制模式 |

---

## 7. 故障保护

| 故障码 | 含义 | 消抖 | 动作 |
|--------|------|------|------|
| `OVERCURRENT` | 相电流 > 2.5A | 5次(250µs) | 关断 MOE |
| `OVERVOLTAGE` | 母线 > 20V | 5次 | 关断 MOE |
| `UNDERVOLTAGE` | 母线 < 0.5V | 5次 | 关断 MOE |
| `ENCODER` | SPI 通信失败 | 100次(5ms) | 关断 MOE |
| DRV8313 nFAULT | 芯片过流/过热/欠压 | 即时 | 关断 MOE+EN |

故障自动恢复: 延时 **500ms** → 清零故障码 → 复位所有 PID → 重新使能 MOE。

---

## 8. 工程结构

```
STM32G431CBU6_MMSD/
├── ApplicationLayer/          # 应用层 (app_foc)
├── CommunicationLayer/        # 通信层 (prot_justfloat, diag_tuning)
├── ControlLayer/              # 控制层 (ctl_math/pid/foc_*)
├── DriverLayer/               # 驱动层 (drv_adc/spi/tim/nfault)
├── Core/                      # CubeMX 生成 (main.c + HAL 初始化)
├── Drivers/                   # CMSIS + STM32G4 HAL 库
├── cmake/                     # CMake 工具链
├── CMakeLists.txt
├── CMakePresets.json
└── document/                  # 文档 &amp; 数据手册
```

---

## 9. 编译 &amp; 烧录

```bash
# Debug
cmake --preset Debug
cmake --build build/Debug

# Release
cmake --preset Release
cmake --build build/Release

# 烧录 (ST-Link)
STM32_Programmer_CLI --connect port=SWD \
  --write build/Debug/STM32G431CBU6_MMSD.hex --verify
```

---

## 10. 编码规范

本项目业务层（ApplicationLayer / CommunicationLayer / ControlLayer / DriverLayer）源码遵循以下规范：

- **大括号**: Allman 风格 — `{` 独占一行
- **运算符**: 禁止 `++` `--` `+=` `-=` `|=` `^=` 等复合赋值，统一 `a = a + 1` 形式
- **三元**: 禁止 `? :`，统一用 `if-else`
- **if 体**: 必须使用 `{}`，即使只有一行

---

## 参考资料

- [STM32G4 参考手册 (RM0440)](https://www.st.com/resource/en/reference_manual/dm00463896.pdf)
- [DRV8313 数据手册](https://www.ti.com/lit/ds/symlink/drv8313.pdf)
- [AS5048A 数据手册](https://ams.com/documents/20143/36005/AS5048A_DS000744.pdf)
- [iPower GM3506 电机规格](document/iPower%20GM3506%20Gimbal%20Motor%20with%20Encoder%20Specifications.txt)

---

> **项目维护**: lidongyang  
> **许可**: 仅供参考学习，未经授权不得用于商业用途
