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
    └── odrive_can.c            # ODrive 实现 (力矩 + 编码器轮询)
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
- `odrive_init()` — 发送 Set_Axis_State(8=CLOSED_LOOP) + Set_Controller_Mode(1=TORQUE, 1=DIRECT), 等待 500 ms 心跳
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

## 游戏兼容性
使用 OpenFFBoard 的 VID/PID (0x1209/0xFFB0)，因此按 ID 注册设备的游戏 (Dirt, EA WRC) 无需编辑 XML 即可识别本方向盘。兼容任何 DirectInput FFB 游戏: Assetto Corsa, iRacing, rFactor 2, BeamNG, Forza 等。

## 调优参数
所有关键参数均可用 `-D` 覆盖:

| 标志 | 默认值 | 说明 |
|---|---|---|
| `MAX_NM` | 4.0 | FFB 满量程对应的电机侧力矩 (Nm) |
| `WHEEL_MAX_TURNS` | 2.0 | ±圈数输出 (2.0 = ±720°) |
| `ODRIVE_NODE_ID` | 0 | 电机 CAN 节点 ID |
| `MCP_BAUD` | 500000 | CAN 波特率 |
| `MCP_SPI_HZ` | 5000000 | MCP2515 的 SPI 时钟 |

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
