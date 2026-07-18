# FFB Wheel — RP2040 力反馈方向盘固件

基于 RP2040 + MCP2515 CAN + SteadyWin GIM6010-8 电机的 USB HID 力反馈方向盘控制器固件。

游戏会把本设备识别为真实的 USB HID FFB 设备（VID 0x1209 / PID 0xFFB0，与 OpenFFBoard 相同），无需虚拟 HID、无需安装驱动，DirectInput 原生识别。

## 架构

```
游戏 (DirectInput)
  ↕ USB HID FFB (PID 1.0, Usage Page 0x0F)
RP2040 (TinyUSB)
  ├── FFB 效果引擎 (ffb.c) — 11 种效果, 40 槽位池
  └── MCP2515 SPI-CAN (mcp2515.c)
       ↕ CAN 500kbaud (ODrive 协议)
GIM6010-8 电机 (ODrive 固件 v0.5.16)
```

## 硬件

| 部件 | 型号 | 说明 |
|---|---|---|
| MCU | RP2040 (Pico) | USB Full-Speed, ~1 kHz FFB 循环 |
| CAN | MCP2515 SPI 模块 | 8 MHz 晶振, 500 kbaud |
| 电机 | SteadyWin GIM6010-8 | 5 Nm 额定, 8:1 减速箱, ODrive CAN |
| 接线 | SPI0: GP18 SCK, GP19 MOSI, GP16 MISO, GP17 CS | 可用 `-D` 标志覆盖 |

## 编译

```bash
# 一次性安装: ARM 工具链 + Pico SDK 依赖
brew install arm-none-eabi-gcc cmake ninja

# 编译
cd ffb_wheel
cmake -B build -G Ninja
cmake --build build

# 刷机: 按住 Pico 的 BOOTSEL, 拷贝 .uf2 文件
cp build/ffb_wheel.uf2 /Volumes/RPI-RP2/
```

> 注意: Homebrew 的 `arm-none-eabi-gcc` 不含 newlib，需下载 ARM 官方 toolchain 13.2.rel1（含完整 newlib），并设置 `PICO_TOOLCHAIN_PATH` 指向其安装目录。

## 文件结构

```
ffb_wheel/
├── CMakeLists.txt              # Pico-SDK 构建, 链接 TinyUSB + hardware_spi
├── pico_sdk_import.cmake       # Pico SDK 导入脚本 (随 SDK 下载)
└── src/
    ├── main.c                  # 入口, 主循环, ODrive 集成
    ├── ffb.h                   # FFB 引擎接口
    ├── ffb.c                   # 效果引擎: 11 种效果, 报告分发
    ├── ffb_types.h             # 报告结构体, 常量 (无 reportId 字节)
    ├── ffb_descriptors.h       # 完整 HID FFB 报告描述符 (源自 VNWheel)
    ├── tusb_config.h           # TinyUSB 配置 (RP2040, 1 个 HID 接口)
    ├── usb_descriptors.c       # USB 设备/配置/字符串描述符
    ├── mcp2515.h               # MCP2515 SPI-CAN 驱动头文件
    ├── mcp2515.c               # MCP2515 实现 (轮询模式)
    ├── odrive_can.h            # ODrive CAN 协议 API
    ├── odrive_can.c            # ODrive 实现 (力矩 + 编码器轮询)
    ├── pedals.h                # 模拟踏板接口 (油门/刹车/离合)
    └── pedals.c                # 片上 ADC 读踏板, 过采样+低通+自动量程
```

## 工作原理

### USB FFB 协议
HID 描述符（源自 VNWheel, MIT 许可证）声明了一个 PID 1.0 Physical Interface Device，包含:
- 8 按钮 + 6 轴 (int16_t) 输入报告
- 11 种 FFB 效果: 恒力、斜坡、方波、正弦、三角、锯齿上/下、弹簧、阻尼、惯性、摩擦
- 40 个效果槽位, 设备管理的池
- OUT 端点 (EP1) + IN 端点 (EP1), 低延迟 FFB 报告

### 效果引擎 (ffb.c)
- `ffb_on_set_report()` — 处理所有 SET_REPORT/OUT 端点报告 (Set Effect, Set Condition, Set Periodic, Effect Operation, Block Free, Device Control 等)
- `ffb_on_get_report()` — 处理 GET_REPORT 请求 (Block Load 和 PID Pool feature 报告)
- `ffb_calculate()` — 1 kHz 节拍: 读取编码器度量, 累加活动效果, 应用包络/增益, 限幅到 ±32767, 调用 `ffb_output_torque()`
- 条件效果 (Spring/Damper/Friction/Inertia) 使用归一化到 -10000..10000 的轴度量 (匹配 PID 描述符)

### ODrive CAN (odrive_can.c)
- `odrive_init()` — 初始化 MCP2515 并发送首次 Set_Controller_Mode(1=TORQUE, 1=DIRECT) + Set_Axis_State(8=CLOSED_LOOP), 不阻塞; 主循环每秒重发直到心跳确认 CLOSED_LOOP(电机后上电也能自动进闭环)
- `odrive_set_torque()` — 将 float Nm 打包为 CAN 0x0E 帧
- `odrive_poll()` — 排空 RX 缓冲, 缓存 pos/vel (来自 0x09 广播), 追踪 CLOSED_LOOP (来自 0x01 心跳)
- `odrive_get_position/velocity()` — 返回缓存的电机侧圈数和圈/秒

### 力矩换算
FFB 引擎输出 (-32767..32767) → Nm, 通过 `MAX_NM` (默认 4.0)。编译时调整:
```bash
cmake -B build -G Ninja -DMAX_NM=3.0  # 更柔和的方向盘
```

### 编码器归一化
- FFB 度量: 电机圈数 → -10000..10000 (用于 Spring/Damper/Friction/Inertia)
- 方向盘报告: 电机圈数 → -32767..32767 (int16_t, 供游戏读取)
- 可配置: `-DWHEEL_MAX_TURNS=2.5` 实现 ±900° 方向盘

## 踏板 (油门 / 刹车 / 离合)

HID 描述符本就声明了 6 个轴 (X/Y/Z/Rx/Ry/Rz), 现在 `pedals.c` 用 RP2040/RP2350
片上 12-bit ADC 读三路模拟踏板, 填入 Y (油门)、Z (刹车)、Rx (离合):

- 默认引脚: **ADC0/1/2 = GP26/GP27/GP28**
- 每路做 8 次过采样 + 指数低通降噪, 并做 OpenFFBoard 式**自动量程** —— 第一次踩满
  全行程后即完成标定, 释放端映射到轴最小值
- 编译期可调: `-DPEDAL_COUNT=2` (只用两路)、`-DPEDAL_INVERT_MASK=0x2` (第 2 路反向,
  用于踩下时电压下降的接线)

> ⚠️ **接线安全 — ADC 脚不耐 5V** (RP2040 完全不耐; RP2350 的 5V 容忍**不含**
> GP26-29 这几个 ADC 脚)。
> - **被动电位器踏板**: 电位器两端接 **3V3(OUT)** 和 GND, 滑臂进 ADC 脚 —— **不要接 5V**。
> - **有源 0-5V 传感器**: 分压到 0-3.3V, 或外接 ADS1115 (I²C), 切勿把 5V 直连 ADC 脚。
> - 不接的踏板务必同时调低 `PEDAL_COUNT`; 悬空的 ADC 脚会因噪声被自动量程误标定。

## 游戏兼容性
使用 OpenFFBoard 的 VID/PID (0x1209/0xFFB0)，因此按 ID 注册设备的游戏 (Dirt, EA WRC) 无需编辑 XML 即可识别本方向盘。兼容任何 DirectInput FFB 游戏: Assetto Corsa, iRacing, rFactor 2, BeamNG, Forza 等。

## 调优参数
所有关键参数均可用 `-D` 覆盖:

| 标志 | 默认值 | 说明 |
|---|---|---|
| `MAX_NM` | 4.0 | FFB 满量程对应的电机侧力矩 (Nm) |
| `WHEEL_MAX_TURNS` | 2.0 | ±圈数输出 (2.0 = ±720°) |
| `ENCODER_SIGN` | +1 | 编码器方向 (方向盘转向反了改 -1) |
| `TORQUE_SIGN` | +1 | 力矩方向 (弹簧不回中/失控时改 -1) |
| `WHEEL_CENTER_AT_BOOT` | 1 | 开机编码器就绪时把当前位置设为中心 (0 = 用 ODrive 编码器零点) |
| `ODRIVE_NODE_ID` | 0 | 电机 CAN 节点 ID |
| `MCP_BAUD` | 500000 | CAN 波特率 |
| `MCP_SPI_HZ` | 5000000 | MCP2515 的 SPI 时钟 |
| `PEDAL_COUNT` | 3 | 模拟踏板数量 (ADC 通道数) |
| `PEDAL_INVERT_MASK` | 0x00 | 踏板反向位掩码 (bit i = 第 i 路反向) |

## 上机调试 (bring-up)

**首次上机务必按顺序做,不要一上来就用默认 `MAX_NM=4.0`。**

### 1. 低力矩起步 + 验证 `MAX_NM` 量纲 ⚠️
`Set_Input_Torque` 一般是减速箱**之前**的电机侧力矩,`4.0 × 8:1` 意味着盘端最高约
**32 Nm**(足以伤手腕);若手册"5 Nm 额定"指的是输出侧,则电机侧仅 ~0.6 Nm、`4.0`
反而超载 6 倍。**先用 `-DMAX_NM=0.5` 编译**,对照手册 / 测电流确认量纲后再逐步加大。

### 2. 极性校准 (`ENCODER_SIGN` / `TORQUE_SIGN`)
电机/编码器的接线决定了两个固件无法自知的物理方向,弄反了弹簧会**把盘甩到底而非回中**:
1. **先定 `ENCODER_SIGN`**:把盘向右转,看游戏里轴是否同向。反了就 `-DENCODER_SIGN=-1`
   (这同时决定游戏轴方向和弹簧参考,先把它弄对)。
2. **再定 `TORQUE_SIGN`**:开一个居中弹簧,如果盘被**推离中心 / 失控**而不是回中,
   就 `-DTORQUE_SIGN=-1`。
低力矩下做这一步最安全。

### 3. 配置 ODrive 编码器广播速率(否则没手感)
本固件靠电机主动广播的 `Get_Encoder_Estimates (0x09)` 拿位置/速度。
**若该广播关闭,`odrive_get_position` 恒为 0 → 方向盘轴不动、弹簧/阻尼完全失效;
若速率太慢(默认常为 ~10-100ms),1kHz 的弹簧/阻尼拿陈旧位置计算会发涩发抖。**
在 ODrive 侧把编码器广播设到 ~1ms(如 `axis0.config.can.encoder_msg_rate_ms = 1`)。

### 3.1. CyberBeast BL72 (GIM6010-8) 上机注意
- **CAN node_id 出厂默认为 1**: 编译时务必 `-DODRIVE_NODE_ID=1` 匹配电机。电机侧 node_id 写死无法通过 ASCII 修改，`sc` 命令会从闪存恢复默认值。
- **CAN 广播默认开启**: 出厂的 `heartbeat_rate_ms=100` 和 `encoder_rate_ms=10` 已打开, 无需额外配置。
- **上电顺序**: 电机先上电, RP2040 后上电。固件启动时初始化 MCP2515, 如果电机没电 CAN 总线无设备。
- **拔掉电机 USB 有助于 CAN 稳定**: 实际测试发现某些情况下同时连接会导致 CAN 帧间歇丢失, 建议调试完成后拔掉电机 USB-C。

### 4. 开启 ODrive CAN watchdog(安全)
固件若卡死/崩溃,ODrive 会保持最后一次力矩,盘可能被顶死。开启轴看门狗
(`axis0.config.enable_watchdog = True`, `watchdog_timeout ≈ 0.05`),本固件 1kHz 的
力矩流会持续喂狗;固件一停,狗超时 → 电机报错进 IDLE。

### 5. 方向盘中心 (回中/找零)
中心不再是"ODrive 编码器的零点",而是一个软零点:`WHEEL_CENTER_AT_BOOT=1`(默认)时,
固件在**开机收到第一帧编码器数据后**,把当前位置捕获为中心。
- **用法**:上电前 / 上电时把方向盘摆到机械正中即可,弹簧会以此为参考回中。
- 对**绝对编码器**尤其有用 —— 它的绝对零点通常不是方向盘正中,软零点解决了这个错位。
- 想随时重设中心,固件导出了 `wheel_recenter()`,接了按钮后可绑定为"回中键"。
- `WHEEL_CENTER_AT_BOOT=0` 则退回用编码器自身零点(不推荐)。
- 注意:这不是靠限位块的物理 homing;若你的机构有硬限位、想要真正的机械中点自动标定,
  可另外做限位 homing(告诉我即可)。

### 6. 踏板标定
插上踏板后,**每个踏板踩到底再松开一次**,自动量程即完成标定(标定前读"释放"值)。

## 已知问题与修复记录

开发过程中修复的关键 bug:

1. `mcp2515.c` 使用 `MCP_SPI_HZ` 但头文件未定义 → 添加 5MHz 默认值
2. `calc_condition` 除以 32767, 但描述符条件系数范围是 -10000..10000 → 改除 10000, 否则 Spring/Damper 只有 9% 力度
3. `get_next_free_effect` 先存 id 再搜索, 导致重复分配在用槽位 → 改为先搜再分配
4. `ffb.c` 用 `HID_REPORT_TYPE_FEATURE` 但未 include tusb.h → 在 `ffb_types.h` 加 `FFB_REPORT_TYPE_*` 常量保持 USB 栈无关
5. `odrive_init` 用循环计数器做超时 → 改用 `pico/time.h` 真定时器 500ms
6. `tusb_config.h` 不能 `#include tusb_options.h` (应为 `tusb_option.h`, 但实际不需要 include, SDK 自动处理)
7. `CFG_TUSB_MCU`/`CFG_TUSB_OS` 不能在 `tusb_config.h` 里定义, SDK build 系统 `-D` 传入
8. `BOARD_TUD_RHPORT` 不存在, 用 `0` 代替
9. `mcp2515.h` 缺 `TXB0SIDL`/`RXB0SIDL` 寄存器定义 → 补上
10. `usb_descriptors.c` 需 include `bsp/board_api.h` 才能用 `board_usb_get_serial` (static inline)

### 2026-07 代码审查修复

效果引擎 (ffb.c / ffb_types.h):

11. `calc_condition` 负系数错误取反 → 弹簧两侧输出同向力, 方向盘会猛拉向一边; 按 PID 规范公式去掉负号
    > **2026-07-18 修正: 之前去掉负号是错误的, 导致弹簧/阻尼产生正反馈(越偏离中心力越大越同向)。
    > 负号是 OpenFFBoard/VNWheel 原始公式的一部分, 正确公式为 `f = -(metric - cp) * coeff / 10000`,
    > 缺失负号会导致弹簧推离中心而非拉回中心。已将负号加回。
12. 力度标度统一: 描述符 magnitude 范围是 ±10000, 引擎却按 ±32767 满量程 → 恒定力只有 30%; 周期效果的 magnitude 还在包络里被二次相乘 (≈9%)。现在全部效果在 ±10000 单位下计算, `ffb_calculate` 末端统一换算到 ±32767
13. Set Envelope 的 attackTime/fadeTime 描述符是 32 位字段, 结构体误用 uint16 → fadeTime 永远读到 0; Set Periodic 的 period 同理改为 uint32
14. 相位单位: 描述符是 0..35999 (0.01°), 代码原按 0..255 处理
15. 无限时长: Windows 用 0xFFFF (Null 值) 表示无限, 原代码只认 0x7FFF → 无限效果 65 秒后消失。现在 0 和 ≥0x7FFF 都视为无限
16. 周期 offset 范围 ±10000, 去掉遗留自 8 位描述符的 `offset*2`
17. Effect Operation 的 loopCount 不再原地改写 duration (重复 Start 会翻倍), 改为独立的 totalDuration (uint32, 防溢出)
18. 恒定力/斜坡力也应用包络 (PID 规范要求); 斜坡按循环迭代重新爬升
19. 支持 Direction 字段: direction enable 且 direction≠0 时对力效果乘 sin(角度) (东=+X, 西=-X); direction=0 视为符号在 magnitude 里的游戏, 直通
20. Set Condition 只接受 parameterBlockOffset==0, 防止 Y 轴条件块覆盖 X 轴弹簧参数
21. GET_REPORT(Input) 现在会应答 PID State 报告 (部分驱动打开设备时查询)
22. gain==0 不再被偷换成 255; Create Effect 后默认 gain=255

安全性 (main.c / odrive_can.c):

23. USB 拔线/挂起 (`tud_umount_cb`/`tud_suspend_cb`) 时停止所有效果并清零力矩 — 此前游戏崩溃后无限时长效果会永远输出力矩
24. 心跳中 `axis_error` 非零时停发力矩指令
25. ODrive v0.5.x 心跳的 current_state 是单字节 (后随 3 个 flag 字节), 原按 uint32 解析, flag 非零时会误判闭环状态
26. `odrive_init` 不再阻塞 500ms 等心跳 (正值 USB 枚举窗口); 改为主循环 1 Hz 重发闭环请求, 电机晚上电也能进闭环

CAN 驱动 (mcp2515.c):

27. 启用 One-Shot 模式: 电机断电/无 ACK 时 TXREQ 不再永久挂起 (原来每帧阻塞 50ms, 主循环掉到 20Hz 拖死 USB); TX 等待缩短到 2ms
28. 接收路径补上 RXB1: 原来开了 BUKT rollover 但从不读 RXB1/清 RX1IF, 首次 rollover 后 RXB1 永久占用

⚠️ 待实机核实: `MAX_NM` 的量纲 — Set_Input_Torque 通常是减速箱**之前**的电机侧力矩, 4 Nm × 8:1 意味着盘端最高 ~32 Nm; 而如果手册的 "5 Nm 额定" 是输出侧, 电机侧额定仅 ~0.6 Nm。两种解读必有一错, 上机前先用低值 (如 `-DMAX_NM=0.5`) 验证。
