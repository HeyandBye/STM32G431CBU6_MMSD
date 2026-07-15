# STM32G431CBU6_MMSD — BLDC Motor FOC Vector Control

> **Version**: v0.1.0 |
> **MCU**: STM32G431CBU6 (Cortex-M4 @ 170MHz) |
> **Driver**: DRV8313 |
> **Encoder**: AS5048A 14-bit SPI |
> **Current Sense**: TMCS1107A3B (200mV/A) |
> **Motor**: iPower GM3506 (24N22P / 11 pole pairs)

> [🇨🇳 中文版](README.md)

---

## 1. Overview

**Field-Oriented Control (FOC)** for BLDC motors on STM32G431CBU6, using a **position-speed-current three-loop cascaded control** architecture.

**Four operating modes**:

| Mode | Description |
|------|-------------|
| Current Loop | Id=0 strategy, dual PI control |
| Speed Loop | Speed PI outer loop + current inner loop |
| Position Loop | Position PID outer loop + speed/current inner loops, with auto-step and manual teach |
| Damper Mode | Pure proportional negative feedback — resistance when turning, free stop on release |

---

## 2. Software Layers

```
main.c (HAL hardware init skeleton)
  └── ApplicationLayer    — system init, main loop tasks, mode management
        ├── CommunicationLayer  — VOFA+ JustFloat online tuning
        ├── ControlLayer        — FOC algorithms (transforms/PID/SVPWM/loop control)
        └── DriverLayer         — peripheral wrappers (ADC/PWM/SPI/nFAULT)
```

### 2.1 Driver Layer

| Module | Function |
|--------|----------|
| `drv_adc_sampling` | ADC1/ADC2 dual-channel DMA circular sampling, IIR low-pass filter |
| `drv_spi_as5048a` | AS5048A encoder SPI driver, direct register access, 3 retries |
| `drv_tim_pwm` | TIM1 six-channel complementary PWM, 20kHz, configurable dead-time |
| `drv_nfault` | DRV8313 nFAULT interrupt, emergency MOE shutdown |

### 2.2 Control Layer

| Module | Function |
|--------|----------|
| `ctl_math` | Clarke / Park / InvPark transforms + 7-segment SVPWM |
| `ctl_pid` | Parallel PID + setpoint weighting + conditional integral anti-windup + derivative IIR filter |
| `ctl_foc_core` | FOC main struct, state machine, global instance, system entry |
| `ctl_foc_current` | Current loop — Id=0 strategy, encoder calibration |
| `ctl_foc_speed` | Speed loop — encoder differential speed measurement |
| `ctl_foc_position` | Position loop — encoder unwrapping (no rollover) |
| `ctl_foc_damper` | Damper mode — Iq = -gain × speed |
| `ctl_foc_openloop` | Open-loop — virtual rotating field, hardware chain verification |

### 2.3 Application &amp; Communication Layers

| Module | Function |
|--------|----------|
| `app_foc` | `App_Init()` / `App_Run()` — LED heartbeat, fault recovery, mode logic |
| `prot_justfloat` | JustFloat binary frame build/parse |
| `diag_tuning` | UART IDLE + DMA bidirectional, online PID/ref/mode adjustment |

---

## 3. Compile-Time Configuration

### Startup Mode (`ctl_foc_core.h`)

```c
#define FOC_MODE  FOC_MODE_POSITION   /* default: position loop */
```

Options: `FOC_MODE_OPENLOOP`(1) / `FOC_MODE_CURRENT`(2) / `FOC_MODE_SPEED`(3) / `FOC_MODE_POSITION`(4) / `FOC_MODE_DAMPER`(5)

### Position Mode (`app_foc.h`)

```c
#define POS_AUTO_STEP  0   /* 1=auto-step (6°/s), 0=manual teach */
```

### Online Tuning (`diag_tuning.h`)

```c
#define TUNING_ENABLE         1   /* 1=enable VOFA+ online tuning */
#define TUNING_SEND_PERIOD_MS 5U  /* data send period (ms)       */
```

### Speed Target (`app_foc.h`)

```c
#define APP_SPEED_RPM  60.0f   /* only effective in FOC_MODE_SPEED */
```

---

## 4. Pin Mapping &amp; Timing

### Hardware Connections

| Function | Pin | Peripheral |
|----------|-----|------------|
| PWM A/B/C High | PA8/PA9/PA10 | TIM1_CH1/2/3 |
| Phase Current Ia/Ib | PA0/PA1 | ADC1_IN1/2 |
| Bus Voltage/Current | PA7/PA6 | ADC2_IN4/3 |
| Encoder SCK/MISO/MOSI/CS | PB3/PB4/PB5/PA15 | SPI1 + GPIO |
| nFAULT | PB11 | EXTI falling edge |
| LED | PC6 | GPIO |

### Control Timing

```
TIM1  20kHz / 50µs   → ADC1 trigger → FOC current loop
TIM6   1kHz / 1ms    → speed PI (differential → update Iq_ref)
TIM7 100Hz / 10ms    → position PID (unwrap → update speed_ref)
main loop             → LED 1Hz heartbeat + fault recovery + VOFA+ comm
```

---

## 5. Control Parameters

### Current Loop (20kHz)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Strategy | Id=0 | Max torque per ampere |
| Kp (Id/Iq) | 4.0 V/A | Proportional gain |
| Ki (Id/Iq) | 0.3 | Integral gain |
| Output clamp | ±Vbus (12V) | PID output limit |
| Current filter | IIR α=0.3 | fc≈1.1kHz |

### Speed Loop (1kHz)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Kp | 0.002 | Proportional gain |
| Ki | 0.001 | Integral gain |
| Output clamp | ±1.0 A | Clamped to Iq_ref |
| Speed limit | ±2000 RPM | Hard limit |

### Position Loop (100Hz)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Kp | 0.10 | Proportional gain |
| Kd | 0.03 | Derivative gain (damping) |
| Output clamp | ±500 RPM | Clamped to speed_ref |

---

## 6. Online Tuning (VOFA+ JustFloat)

- Baud: **115200-8-N-1** (USART1)
- Protocol: JustFloat (raw float32 bytes, tail `00 00 80 7F`)
- Send period: **5ms** (200Hz), non-blocking DMA
- Receive: UART IDLE + DMA

### Uplink (MCU → VOFA+, auto-switches by mode)

| Mode | Channels (N floats + tail) |
|------|----------------------------|
| CURRENT | Id_ref, Id, Iq_ref, Iq, Vd, Vq, Ia, Ib, speed_rpm, mode **(10ch)** |
| SPEED | speed_ref, speed_rpm, iq_ref, Iq, Vq, Ia, Ib, mode **(8ch)** |
| POSITION/DAMPER | pos_cmd, pos_fb, speed_ref, speed_rpm, Iq, Vq, Ia, Ib, mode **(9ch)** |

### Downlink (VOFA+ → MCU, sender panel: 2 channels)

| Cmd | Parameter | Description |
|-----|-----------|-------------|
| 1~4 | Kp_id / Ki_id / Kp_iq / Ki_iq | Tune current PI |
| 5~8 | Kp / Ki / Kd / Kr | Tune speed PID |
| 9~12 | Kp / Ki / Kd / Kr | Tune position PID |
| 13~16 | Id_ref / Iq_ref / Speed_ref / Pos_ref | Modify references |
| 17 | 2/3/4/5 | Switch control mode online |

---

## 7. Fault Protection

| Fault | Meaning | Debounce | Action |
|-------|---------|----------|--------|
| `OVERCURRENT` | Phase current > 2.5A | 5× (250µs) | MOE off |
| `OVERVOLTAGE` | Bus > 20V | 5× | MOE off |
| `UNDERVOLTAGE` | Bus < 0.5V | 5× | MOE off |
| `ENCODER` | SPI comm failure | 100× (5ms) | MOE off |
| DRV8313 nFAULT | IC overcurrent/overtemp/UV | Instant | MOE+EN off |

Auto-recovery: **500ms** delay → clear faults → reset all PIDs → re-enable MOE.

---

## 8. Project Structure

```
STM32G431CBU6_MMSD/
├── ApplicationLayer/          # Application (app_foc)
├── CommunicationLayer/        # Communication (prot_justfloat, diag_tuning)
├── ControlLayer/              # Control (ctl_math/pid/foc_*)
├── DriverLayer/               # Driver (drv_adc/spi/tim/nfault)
├── Core/                      # CubeMX generated (main.c + HAL init)
├── Drivers/                   # CMSIS + STM32G4 HAL
├── cmake/                     # CMake toolchain
├── CMakeLists.txt
├── CMakePresets.json
└── document/                  # Docs &amp; datasheets
```

---

## 9. Build &amp; Flash

```bash
# Debug
cmake --preset Debug
cmake --build build/Debug

# Release
cmake --preset Release
cmake --build build/Release

# Flash (ST-Link)
STM32_Programmer_CLI --connect port=SWD \
  --write build/Debug/STM32G431CBU6_MMSD.hex --verify
```

---

## 10. Coding Standards

The business layer source code (ApplicationLayer / CommunicationLayer / ControlLayer / DriverLayer) follows:

- **Braces**: Allman style — `{` on its own line
- **Operators**: No `++` `--` `+=` `-=` `|=` `^=` — always `a = a + 1` form
- **Ternary**: No `? :` — always `if-else`
- **if-body**: Always use `{}`, even for single statements

---

## References

- [STM32G4 Reference Manual (RM0440)](https://www.st.com/resource/en/reference_manual/dm00463896.pdf)
- [DRV8313 Datasheet](https://www.ti.com/lit/ds/symlink/drv8313.pdf)
- [AS5048A Datasheet](https://ams.com/documents/20143/36005/AS5048A_DS000744.pdf)
- [iPower GM3506 Motor Specs](document/iPower%20GM3506%20Gimbal%20Motor%20with%20Encoder%20Specifications.txt)
