# FFB Wheel — RP2040 力反馈方向盘固件

基于 **RP2040 + MCP2515 CAN + SteadyWin GIM6010-8 电机**的 USB HID 力反馈方向盘控制器固件。

游戏把本设备识别为真实的 USB HID FFB 设备(VID `0x1209` / PID `0xFFB0`,与 OpenFFBoard 相同),**无需虚拟 HID、无需装驱动**,DirectInput 原生识别。

---

## 目录
- [特性](#特性)
- [架构](#架构)
- [硬件与接线](#硬件与接线)
- [编译与刷机](#编译与刷机)
- [配置参考(`-D` 标志)](#配置参考-d-标志)
- [上机流程(bring-up)](#上机流程bring-up)
- [工作原理](#工作原理)
- [排障](#排障)
- [已知问题](#已知问题)
- [Changelog](#changelog)
- [文件结构](#文件结构)
- [许可证与致谢](#许可证与致谢)

---

## 特性

- **原生 DirectInput FFB**:11 种效果(恒力、斜坡、方波、正弦、三角、锯齿上/下、弹簧、阻尼、惯性、摩擦),40 效果槽位。
- **6 轴**:方向盘 + 油门/刹车/离合 + 2 路备用。
- **32 按钮**:通过 MCP23017 I²C 扩展(1–2 片),仅占 2 根线。
- **3 路模拟踏板**:片上 12-bit ADC,过采样 + 低通 + 自动量程。
- **双核架构**:USB 与 CAN 完全隔离,CAN 阻塞不影响 USB 时序。
- **掉电恢复**:电机中途掉电/重连自动检测、重进闭环、重设中心。
- **安全**:USB 掉线/挂起、电机故障、离线时自动清零力矩。

---

## 架构

```
游戏 (DirectInput)
  ↕ USB HID FFB (PID 1.0, Usage Page 0x0F)
RP2040 (TinyUSB) ── 双核 ──────────────────────────────────────┐
  core0: USB (tud_task) + FFB 效果引擎 (ffb.c)                  │
         + 输入报告(方向盘轴 / 踏板 ADC / 按钮 I²C)            │
         └─ ffb_output_torque() → g_torque_cmd ──┐(跨核单字)   │
  core1: MCP2515 SPI-CAN (mcp2515.c) ←───────────┘             │
         odrive_poll() → s_odrv 缓存 ─────────────(跨核单字)───┘
       ↕ CAN 500 kbaud (ODrive 协议)
GIM6010-8 电机 (ODrive 固件 v0.5.16)
```

**双核隔离**:所有 SPI/CAN 收发都在 **core1**,USB 独占 **core0**。CAN 再怎么阻塞/出错都拖不住 `tud_task()`。跨核只共享两个对齐单字——`g_torque_cmd`(core0→core1 力矩指令)和 `s_odrv`(core1→core0 编码器/状态缓存)。RP2040 无数据缓存 + 单写者 + 32/16 位对齐 → 天然无撕裂,**无需加锁**。

---

## 硬件与接线

| 部件 | 型号 | 说明 |
|---|---|---|
| MCU | RP2040 (Pico) | USB Full-Speed,~1 kHz FFB 循环 |
| CAN | MCP2515 SPI 模块 | 8 MHz 晶振,500 kbaud,轮询模式(不接 INT) |
| 电机 | SteadyWin GIM6010-8 | 8:1 减速箱,ODrive CAN 协议 |
| 按钮 | MCP23017 ×1–2(可选) | I²C GPIO 扩展,16/32 键 |
| 踏板 | 电位器 ×3(可选) | 片上 ADC |

### GPIO 引脚表(默认)

| 功能 | 信号 | GPIO | 备注 |
|---|---|---|---|
| **MCP2515 (SPI0)** | SCK | GP18 | |
| | MOSI (SI) | GP19 | |
| | MISO (SO) | GP16 | |
| | CS | GP17 | |
| **MCP23017 按钮 (I2C0)** | SDA | GP20 | 需 ~4.7k 上拉到 3V3 |
| | SCL | GP21 | 需 ~4.7k 上拉到 3V3 |
| **踏板 (ADC)** | 油门 (ADC0) | GP26 | → HID 轴 Y |
| | 刹车 (ADC1) | GP27 | → HID 轴 Z |
| | 离合 (ADC2) | GP28 | → HID 轴 Rx |
| **状态灯** | LED | GP25 | 板载;挂载后 1 Hz,未挂载 4 Hz 闪 |
| **USB** | D+/D− | 专用 | 板载 USB 口 |

> 空闲可做直连按钮/其他用途的 GPIO:GP0–15、GP22。所有引脚均可用 `-D` 覆盖(见配置参考)。

### 接线安全 ⚠️

- **CAN 必须共地**:CAN 是差分但**非隔离**。MCP2515 模块地要和电机 CAN 地连一根参考线,否则共模漂移会间歇丢帧/进 bus-off(见[排障](#排障))。
- **CAN 终端 120Ω**:总线两端各一个,H–L 间总阻抗约 60Ω。
- **ADC 脚不耐 5V**(RP2040 全脚、RP2350 的 GP26–29 均不耐):踏板电位器两端接 **3V3(OUT)** 和 GND,滑臂进 ADC;有源 0–5V 传感器要分压或用外部 ADC。
- **MCP23017 若用 5V 供电,I²C 线不要直连 RP2040**(非 5V 耐受)。用 3V3 供电最省心。

---

## 编译与刷机

```bash
# 一次性:ARM 工具链 + CMake + Ninja
brew install arm-none-eabi-gcc cmake ninja      # macOS
# Windows 用 ARM 官方 toolchain(含完整 newlib)+ Ninja

# 编译
cd ffb_wheel
cmake -B build -G Ninja
cmake --build build

# 刷机:按住 Pico 的 BOOTSEL,拷贝 .uf2
cp build/ffb_wheel.uf2 /Volumes/RPI-RP2/
```

- Pico SDK 由 CMake 自动从 git 拉取(tag **2.1.1**)。
- 预编译镜像见 `firmware/ffb_wheel.uf2`(RP2040,默认配置:踏板/按钮关闭)。
- Homebrew 的 `arm-none-eabi-gcc` 不含 newlib;需 ARM 官方 toolchain 13.2.rel1,并设 `PICO_TOOLCHAIN_PATH`。
- 编 RP2350:`cmake -B build2350 -G Ninja -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2`。

---

## 配置参考(`-D` 标志)

所有参数可在 `cmake` 命令行用 `-D` 覆盖,例如:

```bash
cmake -B build -G Ninja -DMAX_NM=0.5 -DBUTTON_CHIPS=1 -DPEDAL_COUNT=3
```

> ✅ CMakeLists 会把这些 `-D` 转发给编译器才真正生效。**标注"源码"的项** 是外设实例/引脚列表,`-D` 传递不便,请直接改对应头文件。

### FFB / 电机

| 标志 | 默认 | 说明 |
|---|---|---|
| `MAX_NM` | `4.0` | FFB 满量程(±32767)对应的电机侧力矩(Nm)。**首次上机务必先设 0.5**,见[已知问题](#已知问题) |
| `WHEEL_MAX_TURNS` | `2.0` | 方向盘 ±圈数(2.0 = ±720°) |
| `ENCODER_SIGN` | `+1` | 编码器方向(方向盘转向反了改 `-1`) |
| `TORQUE_SIGN` | `-1` | 力矩方向(BL72 默认 `-1`,弹簧反向则改 `+1`) |
| `WHEEL_CENTER_AT_BOOT` | `1` | 编码器每次就绪(开机/掉电重连)时把当前位置设为中心;`0` 用 ODrive 编码器零点 |
| `GEAR_RATIO` / `VEL_NORM_TURNS` | `8.0` / `80.0` | 减速比 / 速度归一化(源码常量) |

### CAN / MCP2515

| 标志 | 默认 | 说明 |
|---|---|---|
| `ODRIVE_NODE_ID` | `1` | 电机 CAN 节点 ID(CyberBeast BL72 出厂默认 1) |
| `MCP_BAUD` | `500000` | CAN 波特率 |
| `MCP_OSC_HZ` | `8000000` | MCP2515 晶振 |
| `MCP_SPI_HZ` | `5000000` | SPI 时钟(≤10 MHz) |
| `MCP_SCK_PIN` / `MCP_MOSI_PIN` / `MCP_MISO_PIN` / `MCP_CS_PIN` | `18/19/16/17` | SPI 引脚 |
| `MCP_SPI` | `spi0` | SPI 实例(源码) |

### 踏板

| 标志 | 默认 | 说明 |
|---|---|---|
| `PEDAL_COUNT` | `0` | 模拟踏板数量(0=禁用,接了改 3) |
| `PEDAL_ADC_BASE_GPIO` | `26` | ADC 通道 0 的 GPIO |
| `PEDAL_INVERT_MASK` | `0x00` | 反向位掩码(bit i = 第 i 路反向) |

### 按钮

| 标志 | 默认 | 说明 |
|---|---|---|
| `BUTTON_CHIPS` | `0` | MCP23017 芯片数(0=禁用,1=16 键,2=32 键) |
| `BUTTON_ADDR0` / `BUTTON_ADDR1` | `0x20` / `0x21` | 两片的 I²C 地址(A2/A1/A0 脚设定) |
| `BUTTON_SDA_PIN` / `BUTTON_SCL_PIN` | `20` / `21` | I²C 引脚 |
| `BUTTON_I2C_HZ` | `400000` | I²C 时钟 |
| `BUTTON_DEBOUNCE` | `3` | 去抖采样数(~250 Hz 扫描,3 ≈ 12 ms) |
| `BUTTON_I2C` | `i2c0` | I²C 实例(源码) |
| `BUTTON_GPIOS` | — | 仅直连 GPIO 版用;当前为 MCP23017 版(源码) |

---

## 上机流程(bring-up)

**首次上机务必按顺序做,不要一上来就用默认 `MAX_NM=4.0`。**

### 1. 低力矩起步 + 验证 `MAX_NM` 量纲 ⚠️
`Set_Input_Torque` 一般是减速箱**之前**的电机侧力矩。`4.0 × 8:1` = 盘端最高约 **32 Nm**(足以伤手腕)。**先 `-DMAX_NM=0.5` 编译**,对照手册/测电流确认量纲后再加大。

### 2. 极性校准(`ENCODER_SIGN` / `TORQUE_SIGN`)
1. **先定 `ENCODER_SIGN`**:把盘向右转,看游戏里轴是否同向,反了就 `-1`(它同时决定游戏轴方向和弹簧参考)。
2. **再定 `TORQUE_SIGN`**:开居中弹簧,若盘被**推离中心/失控**而非回中,就 `-1`。低力矩下做最安全。
> 恒力方向也要单独验一次,见[已知问题](#已知问题)。

### 3. 配置 ODrive 编码器广播速率
固件靠电机主动广播的 `Get_Encoder_Estimates (0x09)` 拿位置/速度。**广播关闭 → 轴不动、弹簧/阻尼失效;太慢 → 1kHz 弹簧拿陈旧位置会发涩发抖。** 在 ODrive 侧设到 ~1ms(`axis0.config.can.encoder_msg_rate_ms = 1`)。

### 3.1 CyberBeast BL72 (GIM6010-8) 专项
- **CAN node_id 出厂 = 1**:编译时 `-DODRIVE_NODE_ID=1`(默认已是)。node_id 写死,`sc` 会从闪存恢复默认。
- **CAN 广播默认开**:出厂 `heartbeat=100ms`、`encoder=10ms`,无需额外配置。
- **上电顺序**:随意——固件有心跳超时+1Hz 重试,谁先谁后都能自恢复(见[排障](#排障))。
- **调试完拔掉电机 USB-C**:同时连接某些情况会 CAN 间歇丢帧。

### 4. 开启 ODrive CAN watchdog(安全)
`axis0.config.enable_watchdog = True`,`watchdog_timeout ≈ 0.05`。本固件 1kHz 力矩流持续喂狗;固件一停,狗超时 → 电机进 IDLE,不会顶死盘。

### 5. 方向盘中心(软零点)
中心是软零点:`WHEEL_CENTER_AT_BOOT=1`(默认)时,固件在**每次编码器就绪的瞬间**(开机或电机掉电重连)把当前位置捕获为中心。
- 上电前/上电时把盘摆到机械正中即可。
- 对**绝对编码器**尤其有用(其绝对零点通常不是盘正中)。
- 固件导出 `wheel_recenter()`,可绑按钮做"回中键"。

### 6. 踏板标定
插上踏板后,**每个踩到底再松开一次**,自动量程即完成标定。

### 7. 按钮
接 MCP23017,`-DBUTTON_CHIPS=1`(16 键)或 `2`(32 键);按钮接芯片 GPIO ↔ GND。

---

## 工作原理

### USB FFB 协议
HID 描述符(源自 VNWheel,MIT)声明一个 PID 1.0 设备:
- 32 按钮 + 6 轴(int16)输入报告。
- 11 种效果、40 槽位、设备管理池。
- OUT(EP1)+ IN(EP81)端点,低延迟 FFB 报告。

### 效果引擎(ffb.c,core0)
- `ffb_on_set_report()` — 处理所有 Set Effect/Condition/Periodic、Effect Operation、Device Control 等。
- `ffb_calculate()` — ~1 kHz:读编码器度量,累加活动效果,应用包络/增益/方向,限幅到 ±10000 后换算到 ±32767,调 `ffb_output_torque()`。
- 条件效果(弹簧/阻尼/惯性/摩擦)用归一化到 -10000..10000 的轴度量(匹配 PID 描述符)。

### ODrive CAN(odrive_can.c,core1)
- `odrive_request_closed_loop()` — Set_Controller_Mode(TORQUE, DIRECT) + Set_Axis_State(CLOSED_LOOP) + 清零力矩。
- `odrive_set_torque()` — float Nm 打包为 CAN `0x0E` 帧。
- `odrive_poll()` — 排空 RX,缓存 pos/vel(`0x09`)、追踪闭环/故障(`0x01` 心跳);**心跳/编码器静默 >250ms 判离线**。
- 力矩换算:引擎 ±32767 → Nm 经 `MAX_NM`,`TORQUE_SIGN` 修正方向。

### 编码器归一化(main.c,core0)
- FFB 度量:电机圈数 → -10000..10000(弹簧/阻尼/摩擦/惯性)。
- 方向盘报告:电机圈数 → -32767..32767(游戏读取)。
- 惯性加速度:速度差分 + EMA 低通(粗略)。

### 踏板(pedals.c,core0)
片上 12-bit ADC 读三路,8 次过采样 + 指数低通 + OpenFFBoard 式自动量程,填入 Y/Z/Rx。

### 按钮(buttons.c,core0)
MCP23017 I²C 读 16/32 输入(active-low + 片内上拉),每键 ~12ms 去抖。**I²C 全部带超时**,芯片没接/总线故障降级为"无按钮",不阻塞 USB。

---

## 排障

### USB 偶发掉线
按可能性排查:
1. **CAN 未共地(最常见)**:MCP2515 地 ↔ 电机 CAN 地加一根参考线。无共地 → 共模漂移 → CAN 错误/bus-off → 即使有非阻塞 send 也可能间歇性影响;**掉线是否总在电机大力矩瞬间?** 是则一定是电气问题。
2. **CAN 终端**:量 H–L 断电阻值应约 60Ω。
3. **供电**:电机与 Pico 分开供电、单点共地;Pico 电源就近加电容;USB 线加磁环、用带供电 Hub。
4. **固件已做的硬化**:`mcp2515_send` 非阻塞、整帧 SPI 指令、双核隔离——CAN 侧问题已尽量不传导到 USB。

### CAN 丢帧 / 电机无响应
- 确认 `ODRIVE_NODE_ID` 与电机一致(BL72 = 1)。
- 确认共地 + 终端(同上)。
- 确认 ODrive 编码器广播已开(否则轴不动)。
- LED 常闪但游戏无力:检查是否进闭环(心跳 state=8)、`axis_error` 是否非零。

### 上电顺序 / 掉电恢复
固件对顺序**无要求**:
- 电机后上电 → 1Hz 重发闭环自动 arm。
- 电机运行中掉电 → 250ms 内判离线,清闭环、力矩归零;回来后自动重进闭环 + 重设中心(防增量编码器零点漂移甩盘)。
- Pico 复位 → 重发闭环并先清零力矩;配合 ODrive watchdog 兜底。
- 固件卡死(core0/FFB)→ core1 在 50ms 内改发 0 力矩,盘变软(不会保持力);与 ODrive watchdog 构成双保险。

---

## 已知问题

1. **`MAX_NM` = 输出(盘端)力矩** — GIM6010-8 的 ODrive `torque_constant` 是**输出侧**配置的,`Set_Input_Torque` 单位就是盘端 Nm(8:1 减速已折算进去),固件按原值下发、**不再除以减速比**。默认 `4.0` ≈ 额定输出力矩,安全。
   > ⚠️ 万一某台电机的 `torque_constant` 其实按电机侧配置,`MAX_NM=4.0` 会在盘端放大 8 倍(~32 Nm,伤手腕)。**首次上机花 10 秒验一次**:发一个小力矩,感受手感 / 测电机电流,确认无误再上满量程。

2. **`calc_condition` 负号 / 恒力方向待验** ⚠️ — 当前 condition 公式无负号、靠 `TORQUE_SIGN=-1` 让弹簧回中(欧卡实测 OK)。但去负号 + 全局 `TORQUE_SIGN` 在数学上**无法让弹簧与恒力同时正确**:弹簧对了,恒力/斜坡/带方向的周期力可能整体反向。**恒力主导的竞速游戏(AC/iRacing/rF2)请单独验一次方向**;若反,恢复 `calc_condition` 负号并用恒力游戏重标 `TORQUE_SIGN`。

3. **待实机确认(逻辑已核对,未上硬件)**:MCP2515 整帧 SPI 指令(字节序/自动清标志)、MCP23017 I²C(寄存器/active-low)、双核 + 掉电恢复整链路。

---

## Changelog

### 2026-07-20 独立审查修复(第二轮)
- **失效保护(双核回归修复)**:双核后即使 core0(FFB)卡死,core1 仍会一直重发最后力矩并喂 ODrive 看门狗 → 盘永久保持力。现 core0 每圈打存活时间戳,core1 发现 >50ms 未推进即发 0。
- **掉电重连甩盘窗口收窄**:力矩门控加入 `odrive_has_encoder()`,确保编码器重新就绪(并已重设中心)之前不输出力矩,堵住"心跳先于编码器帧到达"的竞态。
- **按钮 I²C 超时** 1000µs → 300µs:芯片卡死时对 core0 输入报告的阻塞封顶更小。
- **跨核内存屏障**:`odrive_poll` 末尾加 `__dmb()`,显式化 core0 无锁读所依赖的写入顺序(M0+ 本就顺序,属文档化)。
- Inertia EMA 的 `/8` 改为四舍五入,消除小残差死区;修正 PID State 报告 `effectBlockIndex` 注释(与描述符位序一致)。

### 2026-07-20 稳定性 / 掉电恢复 / 按钮
- **双核拆分**:CAN(init/poll/arm/torque)全移到 core1,USB + FFB 留 core0;CAN 阻塞不再影响 USB;MCP 初始化 sleep 不再卡枚举窗口。
- `mcp2515_send` **完全非阻塞**(去 2ms 忙等,TXB0 忙则 abort,最新力矩优先)。
- **整帧 SPI 指令**:LOAD TX / READ RX BUFFER,单次 CS 收发,事务 ~13 → ~2;READ RX 自动清 RXnIF。
- **电机离线检测**:心跳/编码器静默 >250ms 判离线,清闭环/编码器有效、力矩归零、重发 arm。
- **掉电重连自动重设中心**:`encoder_valid` 上升沿重捕获中心(防零点漂移甩盘);力矩门控 `!axis_error && closed_loop`。
- **Inertia 加速度**修复(此前 `>0.001f` + 1ms 量化恒为 0):每帧差分 + EMA。
- **`-D` 调优标志此前全失效**(从未转发给编译器):加 CMake foreach 转发,MAX_NM/TORQUE_SIGN/PEDAL_COUNT/BUTTON_CHIPS 等才真正可覆盖。
- **按钮支持**:HID 按钮 8 → 32;新增 `buttons.c` 走 MCP23017 I²C(1–2 片,带超时去抖)。
- 修 `PEDAL_COUNT=0` 零长数组告警;README / main.c 头注释同步双核。

### 2026-07(前期代码审查)
效果引擎、极性、CAN 驱动、USB 安全等 28 项修复,详见 git 历史。关键项:
- `calc_condition` 系数范围 32767 → 10000(否则弹簧只有 9% 力度)。
- magnitude/包络标度统一到 ±10000;Envelope/Periodic 的 32 位字段修正;相位单位 0..35999;无限时长认 0xFFFF。
- One-Shot TX + RXB1 补读;心跳单字节 state 解析;`odrive_init` 非阻塞。
- USB 掉线/挂起、`axis_error` 非零时清零力矩。

---

## 文件结构

```
ffb_wheel/
├── CMakeLists.txt              # Pico-SDK 构建 + -D 调优转发
├── pico_sdk_import.cmake       # Pico SDK 导入脚本
├── firmware/ffb_wheel.uf2      # 预编译镜像(RP2040,默认配置)
└── src/
    ├── main.c                  # 入口;core0=USB+FFB,core1=CAN
    ├── ffb.{h,c}               # 效果引擎:11 种效果、报告分发
    ├── ffb_types.h             # 报告结构体、常量
    ├── ffb_descriptors.h       # HID FFB 报告描述符(源自 VNWheel)
    ├── tusb_config.h           # TinyUSB 配置
    ├── usb_descriptors.c       # USB 设备/配置/字符串描述符
    ├── mcp2515.{h,c}           # MCP2515 SPI-CAN 驱动(轮询,整帧指令)
    ├── odrive_can.{h,c}        # ODrive CAN 协议(力矩 + 编码器 + 离线检测)
    ├── pedals.{h,c}            # 模拟踏板(片上 ADC)
    └── buttons.{h,c}           # 按钮(MCP23017 I²C,最多 32 键)
```

---

## 许可证与致谢

- HID FFB 描述符源自 **VNWheel**(MIT,Hoan Tran);效果数学参考 **OpenFFBoard**(GPL,Yannick)。
- VID/PID `0x1209/0xFFB0` 沿用 OpenFFBoard,便于按 ID 注册的游戏(Dirt、EA WRC)免改识别。
- 兼容任意 DirectInput FFB 游戏:Assetto Corsa、iRacing、rFactor 2、BeamNG、Forza 等。
