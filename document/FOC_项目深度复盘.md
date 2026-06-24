# 🔬 STM32 FOC 电机控制项目 — 系统级深度复盘

> **硬件**：STM32G431CBU6 @ 170MHz · DRV8313 · AS5048A 14-bit · TMCS1107A3B
> **电机**：iPower GM3506 云台电机（24N22P, 11对极, $R=5.6\Omega$, $L\approx 281\mu H$）
> **日期**：2026-06-24

---

## 目录

1. [系统时序全景图](#一系统时序全景图)
2. [数据流与变量血缘图](#二数据流与变量血缘图)
3. [关键变量词典](#三关键变量词典)
4. [状态机全景图](#四状态机全景图)
5. [关键阈值速查表](#五关键阈值速查表)
6. [PI 参数整定速查](#六pi-参数整定速查)
7. [性能预算](#七性能预算)

---

## 一、系统时序全景图

```mermaid
sequenceDiagram
    participant CPU as main() 主循环
    participant T1 as TIM1 20kHz<br/>PWM+ADC触发
    participant T6 as TIM6 1kHz<br/>速度环
    participant T7 as TIM7 100Hz<br/>位置环
    participant ADC as ADC1+ADC2
    participant SPI as SPI1 编码器
    participant PWM as DRV8313

    Note over CPU,PWM: ═══ 初始化阶段 (main.c:117-171) ═══

    CPU->>SPI: drv_as5048a_init()
    CPU->>PWM: drv_tim_pwm_init() → CCR=2124 (50%占空比)
    CPU->>ADC: drv_adc_sampling_init() → 校准+启动DMA
    CPU->>CPU: HAL_Delay(100) 等待ADC稳定
    CPU->>PWM: drv_tim_pwm_enable() → CEN→MOE
    CPU->>CPU: FOC_SystemInit()
    Note over CPU: ① FOC_Init 清零所有状态
    Note over CPU: ② SetMotorParams(pole_pairs=11, enc_ofs=0...)
    Note over CPU: ③ 编码器校准: θ=0矢量锁定500ms→读enc_offset
    Note over CPU: ④ Current_SetRef(Id=0.5A, Iq=0)
    Note over CPU: ⑤ Current_Start → RUNNING
    Note over CPU: ⑥ Speed_Init(Kp=0.002,Ki=0.001) → Speed_Start
    Note over CPU: ⑦ Position_Init(Kp=0.10,Ki=0) → Position_Start
    Note over CPU: ⑧ HAL_TIM_Base_Start_IT(&htim7)

    Note over CPU,PWM: ═══ 稳态运行 ═══

    loop 每 50µs (20kHz)
        T1->>ADC: TRGO触发ADC1双通道采样(Ia,Ib)
        ADC-->>ADC: 转换耗时~6µs
        ADC->>SPI: 读取AS5048A角度 (耗时~10µs)
        SPI-->>ADC: raw_angle (0~16383)
        ADC->>ADC: raw→电流 (ADC-2048)×0.004028
        ADC->>ADC: IIR滤波 α=0.3
        Note over ADC: ★ 控制算法 ~16µs ★
        ADC->>ADC: raw→电角度 (raw-ofs)×2π×11/16384
        ADC->>ADC: arm_sin_cos_f32 → sinθ,cosθ
        ADC->>ADC: Clarke: Iα=Ia, Iβ=(Ia+2Ib)/√3
        ADC->>ADC: Park: Id=Iα·cosθ+Iβ·sinθ
        ADC->>ADC: PI(Id→Vd): Kp=4.0,Ki=0.3,限幅±12V
        ADC->>ADC: PI(Iq→Vq): Kp=4.0,Ki=0.3,限幅±12V
        ADC->>ADC: InvPark: Vα,Vβ
        ADC->>ADC: SVPWM: 共模注入→duty_a/b/c∈[0,1]
        ADC->>PWM: drv_tim_pwm_set_duty_f() → CCR寄存器
    end

    loop 每 1ms (1kHz)
        T6->>T6: Δθ_elec = θ_now - θ_prev
        T6->>T6: 回绕检测: |Δθ|>34.5→±69.115
        T6->>T6: speed_rpm = Δθ × 868.0
        T6->>T6: PI(RPM→Iq): Kp=0.002,Ki=0.001,限幅±1A
        T6->>ADC: FOC_Current_SetRef(Id=0, Iq=新值)
    end

    loop 每 10ms (100Hz)
        T7->>T7: 位置展开第1步: delta检测回绕
        T7->>T7: unrapped_pos += raw_delta
        T7->>T7: 位置展开第2步: 回绕到setpoint±8192
        T7->>T7: P(pos_err→RPM): Kp=0.10,限幅±500RPM
        T7->>T6: FOC_Speed_SetRef(新RPM)
    end

    loop 主循环 (while(1))
        CPU->>CPU: LED心跳 1Hz
        Note over CPU: POS_AUTO_STEP=1: 每500ms pos_cmd+=2731(60°)
        Note over CPU: POS_AUTO_STEP=0: 拧偏>500LSB保持6秒→锁定
    end
```

---

## 二、数据流与变量血缘图

```mermaid
flowchart TB
    subgraph 物理层["🔌 物理层 → ADC 原始值"]
        HALL["TMCS1107A3B<br/>200mV/A<br/>零电流=1.65V"] -->|"ADC 12-bit<br/>Vref=3.3V"| ADC_RAW["adc1_dma_buf[0/1]<br/>uint16_t 0~4095"]
        ENC["AS5048A 14-bit<br/>SPI 流水线读取"] -->|"SPI1 寄存器直操<br/>2帧协议+偶校验"| RAW_ANGLE["raw_angle<br/>uint16_t 0~16383"]
        VBUS["电池12V<br/>电阻分压K≈9.16"] -->|"ADC2 IN4"| VBUS_RAW["adc2_dma_buf[1]"]
    end

    subgraph 转换层["📐 转换层 → 物理量"]
        ADC_RAW -->|"offset=raw-2048(int32)"| CURR_OFFSET["有符号偏差<br/>int32_t"]
        CURR_OFFSET -->|"×0.004028<br/>(3.3/4096)/0.2"| CURR_RAW["g_curr_ia/ib<br/>float (A)"]
        CURR_RAW -->|"IIR α=0.3<br/>y=0.3x+0.7y⁻¹"| CURR_FILT["g_curr_ia/ib<br/>滤波后 ±0.05A"]
        VBUS_RAW -->|"×0.007381<br/>(3.3/4096)×9.16"| VBUS_V["g_bus_vol<br/>float (V)"]
        RAW_ANGLE -->|"环形减法 int32<br/>(raw-offset) mod 16384"| ADJ["adjusted<br/>0~16383"]
        ADJ -->|"×2π/16384≈0.0003835"| THETA_M["θ_mech<br/>rad"]
        THETA_M -->|"×11(极对数)"| THETA_E["theta_elec<br/>float rad<br/>范围[0,69.115)"]
    end

    subgraph 变换层["🔄 坐标变换层"]
        CURR_FILT -->|"Ia, Ib"| CLARKE["Clarke 幅值不变<br/>Iα=Ia<br/>Iβ=(Ia+2Ib)/√3"]
        THETA_E -->|"arm_sin_cos_f32<br/>一次查表"| SINCOS["sin_θ, cos_θ"]
        CLARKE -->|"Iα, Iβ"| PARK["Park<br/>Id=Iα·c+Iβ·s<br/>Iq=-Iα·s+Iβ·c"]
        SINCOS --> PARK
    end

    subgraph 控制层["🎛️ 三环串级控制"]
        PARK -->|"Id, Iq 反馈"| PI_ID["PID_Id<br/>Kp=4.0 Ki=0.3 Kd=0 Kr=1.0<br/>限幅±12V 积分限幅±12V"]
        PARK -->|"Iq 反馈"| PI_IQ["PID_Iq<br/>Kp=4.0 Ki=0.3 Kd=0 Kr=1.0<br/>限幅±12V 积分限幅±12V"]
        PI_ID -->|"Vd (V)"| INVP["InvPark<br/>Vα=Vd·c-Vq·s<br/>Vβ=Vd·s+Vq·c"]
        PI_IQ -->|"Vq (V)"| INVP

        THETA_E -->|"Δθ/1ms 差分<br/>回绕检测±34.5"| SPEED_CALC["speed_rpm<br/>=Δθ×868.0"]
        SPEED_CALC -->|"反馈"| PI_SPD["PID_Speed<br/>Kp=0.002 Ki=0.001<br/>限幅±1A → Iq_ref"]

        RAW_ANGLE -->|"展开第1步<br/>delta>8192→±16384"| UNWRAP["unwrapped_pos<br/>连续累计位置"]
        UNWRAP -->|"展开第2步<br/>回绕到sp±8192"| UNWRAP2["unwrapped_pos<br/>保持精度"]
        UNWRAP2 -->|"反馈"| PI_POS["PID_Pos<br/>Kp=0.10 Ki=0<br/>限幅±500RPM→speed_ref"]
    end

    subgraph 输出层["⚡ PWM 输出层"]
        INVP -->|"Vα, Vβ"| SVPWM_BLK["SVPWM 七段式<br/>共模注入法"]
        VBUS_V -->|"防除零 min=0.1V"| SVPWM_BLK
        SVPWM_BLK -->|"duty∈[0,1]<br/>硬钳位"| DUTY["duty_a/b/c<br/>float"]
        DUTY -->|"d×4249→CCR<br/>uint16_t"| CCR["TIM1 CCR1/2/3<br/>直接寄存器写"]
    end

    style HALL fill:#f9f,stroke:#333
    style ENC fill:#f9f,stroke:#333
    style RAW_ANGLE fill:#ff9,stroke:#333
    style THETA_E fill:#ff9,stroke:#333
    style CURR_FILT fill:#9f9,stroke:#333
    style SVPWM_BLK fill:#9cf,stroke:#333
    style CCR fill:#f99,stroke:#333
```

---

## 三、关键变量词典 — 每个数字的完整来龙去脉

### 🔢 变量: `PWM_ARR = 4249`

```mermaid
flowchart LR
    A["f_PWM 需求<br/>≥20kHz(人耳不可闻)"] --> B["中心对齐模式<br/>f = f_clk / (2×(ARR+1))"]
    B --> C["170MHz / (2×4250)"]
    C --> D["= 20,000 Hz ✓"]
    D --> E["ARR = 4250-1 = 4249"]
    E --> F["DUTY_MAX = ARR = 4249"]
    F --> G["CCR_50% = 4249/2 = 2124"]
```

| 层级 | 问题 | 答案 |
|------|------|------|
| **源头** | 为什么是 20kHz？ | 人耳听觉上限 ~20kHz，低于此频率 PWM 啸叫可闻 |
| **推导** | 为什么中心对齐？ | 对称 PWM 谐波更少，适合电机控制；每个周期两次更新机会 |
| **公式** | 为什么 ARR=4249 而非 4250？ | 计数器 0 开始 → N+1 个步进，频率 = f/(2×(ARR+1)) |
| **影响** | ARR=4249 意味着什么？ | 占空比分辨率 1/4250 ≈ 0.024%，电压精度足够 |

---

### 🔢 变量: `ADC_CURR_OFFSET = 2048`

```mermaid
flowchart LR
    A["TMCS1107A3B<br/>零电流输出"] --> B["Vcc/2 = 1.65V"]
    B --> C["ADC 参考 VDDA = 3.3V"]
    C --> D["1.65V / 3.3V × 4096"]
    D --> E["= 2048"]
    E --> F["Ia = (raw-2048)×0.004028<br/>raw=2048 → Ia=0A ✓"]
```

| 层级 | 问题 | 答案 |
|------|------|------|
| **为什么是 1.65V** | TMCS1107A3B 供电 3.3V，零电流输出 Vcc/2 | 双向电流传感器，正负电流各用一半量程 |
| **为什么用 4096 而非 4095** | $V_{ref}/2^N$ 定义 1 LSB | 虽然 max code=4095，但量化阶梯是 Vref/4096 |
| **为什么转 int32_t** | `(int32_t)raw - 2048` | uint16_t 减法下溢会回绕到 65535；int32_t 得到负值 |

---

### 🔢 变量: `ADC_CURR_SCALE = 0.004028f`

```
每 LSB 对应电流 = (Vref/2^N) / 灵敏度
               = (3.3V/4096) / 0.2V/A
               = 0.0008057 / 0.2
               = 0.004028 A/LSB

验证链:
  raw=2048 → offset=0   → Ia=0.000A  ← 零电流
  raw=2296 → offset=248 → Ia=0.999A  ← ~1A ✓
  raw=1800 → offset=-248→ Ia=-0.999A ← ~-1A ✓
```

**为什么不直接用 `(raw - 2048) * 3.3 / 4096 / 0.2`**：每次 ADC 回调都算一次浮点除法 → 20kHz × 3 操作 = 6 万次/秒浪费。预计算为常量 → 单条 FMUL 指令，1 CPU 周期。

---

### 🔢 变量: `ADC_CURR_FILT_ALPHA = 0.3f`

```mermaid
flowchart TB
    A["原始电流 Ia<br/>噪声 ±0.2A"] --> B["IIR 一阶低通<br/>y=0.3x+0.7y⁻¹"]
    B --> C["滤波后 Ia<br/>噪声 ±0.05A"]
    
    D["α=0.3 的选择"] --> E["截止频率<br/>fc=0.3/(2π×50µs)≈955Hz"]
    E --> F1["✅ > 电流环带宽 ~200Hz<br/>不衰减控制信号"]
    E --> F2["✅ ≪ PWM纹波 20kHz<br/>有效抑制开关噪声"]
    
    G["如果 α=0.1"] --> H["fc≈318Hz<br/>滤波太强→相位滞后→PI不稳"]
    I["如果 α=0.8"] --> J["fc≈2546Hz<br/>滤波太弱→噪声残留→PI抖"]
```

| 对比维度 | α=0.1 | α=0.3✅ | α=0.8 |
|---------|-------|---------|-------|
| 截止频率 | 318Hz | 955Hz | 2546Hz |
| 噪声抑制 | 强 | 中 | 弱 |
| 相位滞后 | ~500µs | ~160µs | ~60µs |
| 电流环影响 | 可能振荡 | 稳定 | Id/Iq 抖动 |

---

### 🔢 变量: `2731.0f` (位置步进量)

```
目标: 每步旋转 60° 机械角

一圈 = 360° = 16384 LSB (AS5048A 14-bit)
 60° = 60/360 × 16384
     = 1/6 × 16384
     = 2730.666...

取 2731 → 实际步进 = 2731/16384×360 = 59.99° ≈ 60°

为什么不是 2730?  
  2730/16384×360 = 59.98° → 累加6步后误差 0.12°
  2731/16384×360 = 59.99° → 累加6步后误差 0.06°
  ↓
  2731 更接近真值，且奇数在连续步进中累积误差更小
```

---

### 🔢 变量: `868.0f` (转速系数)

```
RPM = Δθ_elec / P × (60 / 2π) × (1 / Δt)
     = Δθ_elec / 11 × 9.5493 × 1000
     = Δθ_elec × 868.118...

取 868.0 → 精度 ±0.014%

为什么是电角度差分而非编码器差分?
  编码器: 1ms内Δraw ≈ 20 LSB @500RPM → 分辨率粗
  电角度: 1ms内Δθ_elec ≈ 0.6 rad → 放大11倍 → 分辨率细11倍
```

```mermaid
flowchart LR
    subgraph 电角度差分
        A["theta_elec(t)<br/>=θ_mech×11"] --> B["Δθ=θ(t)-θ(t-1ms)"]
        B --> C{"|Δθ|>34.5?"}
        C -->|"是,Δθ>0"| D["Δθ-=69.115<br/>(正向回绕)"]
        C -->|"是,Δθ<0"| E["Δθ+=69.115<br/>(反向回绕)"]
        C -->|"否"| F["正常差值"]
        D --> G["RPM=Δθ×868.0"]
        E --> G
        F --> G
    end
```

**回绕阈值 34.5 的来历**：
```
电角度范围 = 2π × 11 = 69.115038379 rad
半圈 = 69.115 / 2 = 34.5575... ≈ 34.5

1ms 内正常 Δθ:
  @2000RPM → 2000×11×2π/60×0.001 ≈ 2.3 rad ≪ 34.5 ✓
  
如果 |Δθ| > 34.5 → 一定发生了跨周期回绕，不是真实速度
```

---

### 🔢 变量: `8192.0f` (位置展开阈值)

```mermaid
flowchart TB
    subgraph 第1步["第1步: delta检测回绕(100Hz)"]
        A1["raw_now - raw_prev"] --> B1{"|delta| > 8192?"}
        B1 -->|">8192(正回绕)"| C1["delta -= 16384<br/>例: 0-16383=-16383→修正为+1"]
        B1 -->|"<-8192(负回绕)"| D1["delta += 16384<br/>例: 16383-0=16383→修正为-1"]
        B1 -->|"否"| E1["delta 不变"]
        C1 --> F1["unwrapped_pos += delta"]
        D1 --> F1
        E1 --> F1
    end

    subgraph 第2步["第2步: setpoint重定位(防float精度退化)"]
        F1 --> G2["diff = unwrapped_pos - setpoint"]
        G2 --> H2{"|diff| > 8192?"}
        H2 -->|">8192"| I2["unwrapped_pos -= 16384"]
        H2 -->|"<-8192"| J2["unwrapped_pos += 16384"]
        H2 -->|"否"| K2["保持不变"]
        I2 --> L2["精度保持 <0.002LSB"]
        J2 --> L2
        K2 --> L2
    end
```

**为什么 8192 = 16384/2**：
- 相邻两次采样的角度变化不可能超过半圈（100Hz × 半圈 ≈ 50转/秒 = 3000RPM，远超实际）
- 8192 是**检测阈值**而非精度损失点——超过半圈的变化一定是回绕

**为什么需要第 2 步（float32 精度退化）**：

| 累计圈数 | unwrapped_pos (LSB) | float32 精度 (LSB) | 还能控制吗 |
|---------|---------------------|-------------------|-----------|
| 0 | 0 | 0.001 | ✅ |
| 10 | 163,840 | 0.02 | ✅ |
| 100 | 1,638,400 | 0.2 | ⚠️ 抖动 |
| 1000 | 16,384,000 | 1.95 | ❌ 失效 |

> float32 尾数 23 位 → 相对精度 ≈ 1.2×10⁻⁷。unwrapped_pos 越大，绝对分辨率越差。第 2 步把值保持在 setpoint±8192 范围内 → 绝对精度始终 < 8192/2²³ ≈ 0.001 LSB。

---

### 🔢 变量: `loop_count > 2000U` (故障检测延时)

```
2000 周期 × 50µs/周期 = 100ms

为什么是 100ms？
  ① 电机启动浪涌电流可达 2-3 倍额定值
  ② PI 建立时间约 20-50 个周期(1-2.5ms)
  ③ 电流环从 0→目标需要 ~10ms
  ④ 取 100ms 是 ③ 的 10 倍安全余量

为什么用 loop_count 而非 HAL_GetTick？
  ① loop_count 在 20kHz ISR 中原子递增，精度 50µs
  ② HAL_GetTick 精度 1ms，100ms 内可能因中断延迟误差 ±2ms
  ③ ISR 内用 HAL_GetTick 需要额外函数调用开销
```

---

### 🔢 变量: 电流环 `Kp=4.0, Ki=0.3`

```mermaid
flowchart TB
    subgraph Kp推导["Kp=4.0 的来历"]
        A["电机绕组 R=5.6Ω L≈281µH"] --> B["电气时间常数<br/>τ=L/R=50µs"]
        B --> C["1A 误差需要多大电压修正?"]
        C --> D["稳态: V=I×R → 1A×5.6Ω=5.6V"]
        D --> E["Kp 至少 5.6 才能 1:1 抵消"]
        E --> F["取 Kp=4.0<br/>1A误差→4V修正<br/>+积分补偿剩余1.6V"]
    end

    subgraph Ki推导["Ki=0.3 的来历"]
        G["Ki 经验范围: Kp/10 ~ Kp/100"] --> H["4.0/13.3=0.3"]
        H --> I["Ti = Kp/Ki = 13.3 周期 = 667µs"]
        I --> J["积分建立时间 ~667µs<br/>= 电气时间常数 50µs ×13"]
        J --> K["既足够快消除静差<br/>又不至于振荡"]
    end

    subgraph 失败经历["调参血泪史"]
        L["Kp=4.0 Ki=4.0"] --> M["❌ 剧烈振荡<br/>积分太强,超调>50%"]
        N["Kp=4.0 Ki=2.0"] --> O["❌ 仍然振荡<br/>积分仍太强"]
        P["Kp=4.0 Ki=0.3"] --> Q["✅ 稳定<br/>Iq跟踪良好,超调<5%"]
    end
```

**为什么电流环积分限幅要覆盖为 ±12V**：绕组电感 281µH 在 12V 下的电流上升率 = 12V/281µH = 42.7 A/ms。50µs 内只能建立 2.1A。如果积分限幅只有 30%（±3.6V），积分项不足以补偿电感压降。

---

### 🔢 变量: 位置环 `Kp=0.10, Ki=0`

```
为什么 Kp=0.10 (单位: RPM/LSB)?
  1 LSB 误差 → 输出 0.10 RPM
  500 LSB 误差 (示教死区) → 输出 50 RPM (平缓启动)
  16384 LSB 误差 (半圈) → 输出上限 500 RPM (限幅起作用)

为什么 Ki=0 而非低速积分?
  ① 云台电机摩擦力极低 (轴承+磁滞<0.01Nm)
  ② 纯 P 稳态误差 = 负载转矩/Kp ≈ 0 (空载)
  ③ I 会累积编码器量化误差 → 来回振荡
  ④ 0.10 × 5 LSB = 0.5 RPM → 电流环足以处理

如果强制加 Ki=0.01 会怎样?
  → 转子到达目标后积分未衰减完 → 冲过头
  → 反向积分累积 → 冲回来
  → 极限环振荡
```

---

### 🔢 变量: `500.0f` (位置示教死区) & `6000U` (示教保持时间)

```
死区 500 LSB = 500/16384 × 360° ≈ 10.99° ≈ 11°

为什么是 11° 而非 2° 或 30°?
  太小(2°): 手指轻微触碰就触发 → 误触发严重
  太大(30°): 需要刻意大幅度拧 → 使用体验差
  11°:   有意拧动(>11°)才触发，无意识碰触(<5°)不触发 ✓

保持时间 6 秒:
  ① 区分"路过"和"停留" → 必须稳住 6 秒才算"新目标"
  ② 太短(1s): 拧动过程中就锁定 → 锁定在非期望位置
  ③ 太长(15s): 等待时间过久 → 体验差
  ④ 6s ≈ 位置环稳定时间的 100 倍余量 → 确保已经完全到位
```

---

## 四、状态机全景图

```mermaid
stateDiagram-v2
    [*] --> IDLE: 上电复位

    IDLE --> READY: FOC_SetMotorParams()<br/>写入电机参数(pole_pairs=11...)
    
    READY --> READY: CalibrateEncoder()<br/>θ=0矢量锁轴500ms→读enc_offset
    
    READY --> RUNNING: FOC_Current_Start()<br/>①PID复位 ②同步setpoint ③置RUNNING
    
    RUNNING --> RUNNING: 每50µs: FOC_Current_Run()<br/>每1ms: FOC_Speed_Run()<br/>每10ms: FOC_Position_Run()
    
    RUNNING --> FAULT: 故障检测触发<br/>①过流(>2.5A) ②过压(>20V)<br/>③欠压(<0.5V) ④编码器异常
    
    RUNNING --> READY: FOC_Current_Stop()<br/>占空比归50%+PID复位
    
    FAULT --> IDLE: 复位后重新初始化

    note right of RUNNING
        FOC_MODE=4(POSITION):
        位置环100Hz→速度环1kHz→电流环20kHz
        Id=0策略, Iq由外环级联给定
    end note

    note right of FAULT
        EmergencyStop:
        ①占空比→50%(绕组零压)
        ②Id/Iq给定清零
        ③PID积分复位
        ④MOE关断(可选)
    end note
```

---

## 五、关键阈值速查表

| 阈值 | 值 | 单位 | 物理含义 | 设计理由 |
|------|------|------|----------|----------|
| `8192` | 16384/2 | LSB | 半圈编码器 | 回绕检测最大合理步长 |
| `34.5` | 69.115/2 | rad | 半圈电角度 | 电角度回绕检测阈值 |
| `69.115` | 2π×11 | rad | 一圈电角度 | 11 对极下全电周期 |
| `500` | ~11° | LSB | 示教死区 | 有意拧 vs 无意碰 |
| `6000` | 6s | ms | 示教保持 | 区分路过和停留 |
| `2000` | 100ms | 周期 | 故障检测延时 | 躲过启动浪涌 |
| `2731` | 60° | LSB | 步进步长 | 60°机械角 |
| `0.3` | α | 无量纲 | ADC IIR 滤波 | 截止 955Hz |
| `0.3` | ratio | 无量纲 | 积分限幅比例 | 积分最多占 30% 输出 |
| `0.1` | α | 无量纲 | 微分 IIR 滤波 | 截止 318Hz |
| `100` | ms | ms | ADC 稳定等待 | 电源+传感器建立 |

---

## 六、PI 参数整定速查

| 环路 | Kp | Ki | Kp/Ki 比值 | TI(等效) | 限幅 | Kr |
|------|-----|-----|-----------|----------|------|-----|
| **电流 Id** | 4.0 V/A | 0.3 | 13.3 | 667µs | ±12V | 1.0 |
| **电流 Iq** | 4.0 V/A | 0.3 | 13.3 | 667µs | ±12V | 1.0 |
| **速度** | 0.002 A/RPM | 0.001 | 2.0 | 2ms | ±1A | 1.0 |
| **位置** | 0.10 RPM/LSB | **0** | ∞ | ∞ | ±500RPM | 1.0 |

> Kp/Ki 比值 = 等效积分时间常数（离散周期数）。电流环 ~13 周期快速消除静差，速度环 ~2 周期偏比例主导，位置环纯 P 杜绝积分振荡。

---

## 七、性能预算

| 环节 | 耗时 | 占比(50µs) |
|------|------|-----------|
| ADC 转换 | ~6µs | 12% |
| 编码器 SPI 读取 | ~10µs | 20% |
| sin/cos 查表 ×2 | ~0.3µs | <1% |
| Clarke+Park+PI×2+InvPark | ~1.5µs | 3% |
| SVPWM | ~1.5µs | 3% |
| PWM CCR 写入 | ~0.2µs | <1% |
| **总控制算法** | **~3.5µs** | **~7%** |
| **全流程(含ADC+SPI)** | **~16µs** | **~32%** |

> CPU 余量 ~68%，足够扩展 CAN 通信、更复杂的位置规划、在线参数整定等功能。

---

## 附录：面试要点串联

| 面试问题 | 项目中的体现 |
|---------|------------|
| **为什么用 Id=0 控制** | 表贴式 PMSM，永磁体已提供转子磁场，Id=0 最大化转矩/电流比 |
| **三环串级的带宽关系** | 电流 20kHz > 速度 1kHz > 位置 100Hz，内环带宽 > 外环 10 倍+ |
| **积分抗饱和怎么做** | 条件积分：输出饱和且误差同向时冻结积分，异向时允许退饱和 |
| **位置环为什么纯 P** | 云台低摩擦，P 稳态误差 < 0.2° 可接受，I 导致过冲振荡 |
| **编码器回绕怎么处理** | 两步法：①delta 检测回绕方向累加 ②定期回绕到 setpoint ±8192 防精度退化 |
| **SVPWM 为什么用共模注入** | 比扇区法少 6 路分支，纯算术运算，M4 上更快 |
| **为什么用 float 不用 Q 格式** | M4 有硬件 FPU，float 单周期，物理量可读性碾压定点 |
