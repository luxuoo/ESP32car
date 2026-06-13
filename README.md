# ESP32-S3 AI 视觉跟踪小车 — 人形跟随版

> 移植自 STM32F407 方案，运行于 ESP32-S3 DevKitM-1，新增自动人形跟随功能。

---

## 目录

- [系统架构](#系统架构)
- [硬件接线](#硬件接线)
- [软件结构](#软件结构)
- [通信协议](#通信协议)
- [核心算法](#核心算法)
- [人形跟随逻辑](#人形跟随逻辑)
- [可调参数](#可调参数)
- [编译与烧录](#编译与烧录)
- [MaixCAM 端配置](#maixcam-端配置)
- [相比 STM32 版的改进](#相比-stm32-版的改进)
- [故障排查](#故障排查)

---

## 系统架构

```
┌─────────────────┐    UART 115200    ┌──────────────────┐
│    MaixCAM-Pro  │ ───────────────→  │     ESP32-S3     │
│  (视觉检测)     │   二进制协议       │  (运动控制)      │
│                 │ ←───────────────  │                  │
│  · YOLOv8       │   远程手动指令     │  · 云台 PID 跟踪 │
│  · 人脸/人体检测│                   │  · 差速转向      │
│  · TCP 命令服务  │                   │  · 自动跟随决策  │
└─────────────────┘                   └──────────────────┘
```

**工作流程：**
1. MaixCAM 摄像头采集画面，运行 YOLOv8 检测人形
2. 检测结果通过 UART 二进制协议发送给 ESP32-S3
3. ESP32-S3 解析坐标，PID 控制云台舵机跟踪目标
4. 同时根据目标位置和大小自动控制底盘电机跟随移动
5. MaixCAM 也支持 TCP 远程命令，可手动控制电机

---

## 硬件接线

### ESP32-S3 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| **左电机 PWM** | GPIO 4 | TB6612 PWMA |
| **左电机方向1** | GPIO 5 | TB6612 AIN1 |
| **左电机方向2** | GPIO 6 | TB6612 AIN2 |
| **右电机 PWM** | GPIO 7 | TB6612 PWMB |
| **右电机方向1** | GPIO 15 | TB6612 BIN1 |
| **右电机方向2** | GPIO 16 | TB6612 BIN2 |
| **云台水平舵机** | GPIO 17 | SG90 X 轴 |
| **云台俯仰舵机** | GPIO 18 | SG90 Y 轴 |
| **MaixCAM TX→** | GPIO 10 | ESP32-S3 RX (CAM_RX) |
| **MaixCAM ←RX** | GPIO 9 | ESP32-S3 TX (CAM_TX) |

### TB6612FNG 电机驱动真值表

| AIN1 | AIN2 | 电机状态 |
|------|------|----------|
| HIGH | LOW  | 正转 |
| LOW  | HIGH | 反转 |
| LOW  | LOW  | 滑行停止 |
| HIGH | HIGH | 制动 |

### SG90 舵机参数

| 角度 | 脉宽 |
|------|------|
| 0° | 0.5ms |
| 90° | 1.5ms |
| 180° | 2.5ms |

PWM 频率 50Hz，16-bit 分辨率 (65536 级)。

---

## 软件结构

```
src/
├── main.cpp            # 主程序: 状态机 + 人形跟随
├── ringbuffer.c        # 镜像位环形缓冲区 (ISR 安全)
├── comm_protocol.c     # 二进制帧协议 编解码
├── pid.c               # 增量式 / 位置式 PID
├── motor.c             # TB6612 双电机驱动 (LEDC 20kHz)
└── servo.c             # SG90 舵机驱动 (LEDC 50Hz)

include/
├── pin_config.h        # 引脚 + PWM 参数集中定义
├── ringbuffer.h
├── comm_protocol.h
├── pid.h
├── motor.h
└── servo.h
```

### 主循环状态机

```
┌─────────────┐
│ E_PACKET_GET │ ←──────┐
│  协议解析    │         │
└──────┬──────┘         │
       │ cmd_type       │
       ├────────────────┤
       ↓                │
┌─────────────┐         │
│ E_SERVO_CTRL│ ────────┘
│  PID 跟踪   │
│  自动跟随   │
└─────────────┘
       │
       │ cmd_type = 0x01
       ↓
┌─────────────┐
│ E_MOTOR_CTRL│ ────────→ E_PACKET_GET
│  远程手动   │
└─────────────┘
```

---

## 通信协议

### 帧格式

```
┌──────┬──────────┬──────────┬─────────────────┬──────────┬──────┐
│ 0xAA │ len(L)   │ len(H)   │ payload (N字节)  │ checksum │ 0x55 │
│ 帧头 │ 长度低字节│ 长度高字节│      数据        │   校验和  │ 帧尾 │
└──────┴──────────┴──────────┴─────────────────┴──────────┴──────┘
```

- **长度字段**：2 字节小端序，仅表示 payload 长度
- **校验和**：长度字段 + 所有 payload 字节之和 (单字节)
- **帧总长**：3 + payload_len + 2

### 命令类型

| cmd_type | 含义 | payload 格式 |
|----------|------|-------------|
| `0x01` | 电机控制 | `dir` (int32) + `speed` (int32) |
| `0x02` | 目标坐标 | `x` + `y` + `width` + `height` (4×int32) |

### 目标坐标含义 (cmd_type = 0x02)

```
         图像坐标系 (320×224)
    ┌─────────────────────────┐
    │ (0,0)            (319,0)│
    │                         │
    │    ┌───────────┐        │
    │    │  检测目标  │        │
    │    │ (x,y)     │        │
    │    │   w × h   │        │
    │    └───────────┘        │
    │                         │
    │ (0,223)         (319,223)│
    └─────────────────────────┘

    目标中心 = (x + w/2, y + h/2)
    图像中心 = (160, 112)
```

---

## 核心算法

### PID 跟踪 (增量式)

云台采用**增量式 PID** 控制，相比位置式有以下优势：
- 输出增量，不会大幅跳变
- 积分项不会无限累积（但本方案额外加了限幅保护）

```
增量 = Kp × (e[k] - e[k-1]) + Ki × e[k] + Kd × (e[k] - 2×e[k-1] + e[k-2])
target_angle += 增量
```

### PID 参数

| 轴 | Kp | Ki | Kd | 说明 |
|----|----|----|-----|------|
| X (水平) | 0.102 | 0.016 | 0 | 正值 |
| Y (俯仰) | -0.102 | -0.016 | 0 | 反号 (图像 Y 轴与舵机相反) |

### 一阶低通滤波

在 PID 之前对目标坐标做平滑处理，抑制检测抖动：

```
filtered = α × new + (1 - α) × old    (α = 0.01，非常平滑)
```

### 积分限幅 (Bug 修复)

STM32 版的 `target_x` / `target_y` 是纯累加器，长时间偏离中心会饱和到 500+，导致恢复跟踪时严重滞后。

ESP32-S3 版在每次累加后限幅到 **[0, 180]**：

```cpp
if (target_x > 180.0) target_x = 180.0;
if (target_x < 0.0)   target_x = 0.0;
```

---

## 人形跟随逻辑

跟踪和跟随是**两个独立维度**，同时执行：

```
检测到目标
    │
    ├──→ 云台跟踪: PID 控制舵机转向 (始终执行)
    │
    └──→ 底盘跟随: 电机运动决策 (同时执行)
            │
            ├─ 目标偏离中心 > 死区？
            │   ├─ 偏右 → 右转
            │   └─ 偏左 → 左转
            │
            ├─ 目标在中心但框宽 < 阈值？
            │   └─ 目标较远 → 前进
            │
            └─ 目标在中心且框宽 >= 阈值？
                └─ 目标够近 → 停止
```

### 决策参数

| 条件 | 动作 | 速度 |
|------|------|------|
| 水平偏差 > 死区 | 左转 / 右转 | `TURN_SPEED` (100) |
| 水平偏差 ≤ 死区 且 框宽 < 80px | 前进 | `FOLLOW_SPEED` (120) |
| 水平偏差 ≤ 死区 且 框宽 ≥ 80px | 停止 | 0 |
| 丢失目标 > 500ms | 停止 | 0 |

### 为什么用框宽而不是舵机角度判断距离？

- 框宽直接反映目标在画面中的占比，与距离近似反比
- 舵机角度只反映方向，不反映距离
- 框宽阈值简单直观，容易调参

---

## 可调参数

所有参数集中在 [src/main.cpp](src/main.cpp) 顶部：

```cpp
// ===== 自动跟随参数 =====
#define FOLLOW_SPEED       120    // 跟随前进速度 (0~255)
#define TURN_SPEED         100    // 转向速度
#define PAN_DEADZONE       8.0    // 水平死区 (度)
#define BBOX_STOP_WIDTH    80     // 框宽阈值 (像素)
#define LOST_TIMEOUT_MS    500    // 丢失超时 (毫秒)

// ===== PID 参数 =====
// X 轴: Kp=0.102, Ki=0.016, Kd=0
// Y 轴: Kp=-0.102, Ki=-0.016, Kd=0

// ===== 滤波 =====
#define FILTER_FACTOR      0.01   // 低通系数 (越小越平滑)

// ===== 云台初始角度 =====
#define SERVO_X_INIT       90.0   // 水平居中
#define SERVO_Y_INIT       80.0   // 俯仰略低
```

### 调参建议

| 场景 | 调整 |
|------|------|
| 跟踪抖动 | 降低 `Kp`，或增大 `FILTER_FACTOR` |
| 跟踪迟钝 | 增大 `Kp`，或减小 `FILTER_FACTOR` |
| 转向太猛 | 降低 `TURN_SPEED` |
| 跟得太近 | 增大 `BBOX_STOP_WIDTH` |
| 跟得太远 | 减小 `BBOX_STOP_WIDTH` |
| 频繁误停车 | 增大 `PAN_DEADZONE` |
| 丢目标后反应慢 | 减小 `LOST_TIMEOUT_MS` |

---

## 编译与烧录

### 环境要求

- [PlatformIO](https://platformio.org/) (VSCode 插件或 CLI)
- ESP32-S3 DevKitM-1 开发板

### 编译

```bash
cd aicaresop
pio run
```

编译产物位于 `.pio/build/esp32-s3-devkitm-1/firmware.bin`。

### 烧录

```bash
pio run --target upload
```

或直接在 VSCode 中点击 PlatformIO 的 **Upload** 按钮。

### 串口监视

```bash
pio device monitor --baud 115200
```

---

## MaixCAM 端配置

ESP32-S3 版与 STM32 版使用**完全相同的通信协议**，MaixCAM 端代码无需修改。

### MaixCAM 接线

| MaixCAM | ESP32-S3 |
|---------|----------|
| TX | GPIO 10 (CAM_RX) |
| RX | GPIO 9 (CAM_TX) |
| GND | GND |

### MaixCAM 端代码要点

```python
# 发送目标坐标 (cmd_type = 0x02)
payload = struct.pack('<iiiii', 0x02, x, y, w, h)
packet = protocol.encode(payload)
uart.write(packet)

# 发送电机控制 (cmd_type = 0x01)
payload = struct.pack('<iii', 0x01, dir, speed)
packet = protocol.encode(payload)
uart.write(packet)
```

确保 MaixCAM 的 UART 波特率为 **115200**，与 ESP32-S3 一致。

---

## 相比 STM32 版的改进

| # | STM32 版问题 | ESP32-S3 版改进 |
|---|-------------|----------------|
| 1 | 舵机角度累加器无上限，可飙到 500+ | **限幅 [0, 180]**，积分永不饱和 |
| 2 | 无 `motor_stop()` 函数 | **完整实现**，方向引脚 + PWM 全部清零 |
| 3 | 仅支持远程手动控制电机 | 新增**自动人形跟随** |
| 4 | 丢失目标无处理 | **500ms 超时自动停车** |
| 5 | 4 路电机分开控制 (实际同侧联动) | 简化为 2 路 (左/右)，差速转向更直观 |
| 6 | STM32 StdPeriph + 寄存器配置 | ESP32 Arduino，**代码量减少 60%** |
| 7 | 无死区，目标在中心附近频繁微调 | **8° 死区**，减少无意义转向 |

---

## 故障排查

### 编译错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `undefined reference to ledcAttachChannel` | Arduino ESP32 核心版本不匹配 | 使用 `ledcSetup()` + `ledcAttachPin()` (已在代码中修复) |
| `undefined reference to servo_init` | C/C++ 链接问题 | 确保 `servo.h` 包含 `extern "C"` 包裹 (已修复) |

### 运行时问题

| 现象 | 可能原因 | 排查 |
|------|---------|------|
| 舵机不动 | LEDC 通道冲突 | 检查 `pin_config.h` 中通道分配不重复 |
| 电机单向转 | TB6612 方向引脚接反 | 交换 AIN1/AIN2 或 BIN1/BIN2 |
| 收不到 MaixCAM 数据 | 串口接线 / 波特率 | 检查 TX↔RX 交叉连接，确认 115200 |
| 跟踪剧烈抖动 | PID 参数过大 | 降低 `Kp` 或增大 `FILTER_FACTOR` |
| 跟踪迟钝 | 滤波过强 | 增大 `FILTER_FACTOR` (如 0.05) |
| 跟随时来回转 | 死区太小 | 增大 `PAN_DEADZONE` |

---

## 文件清单

```
aicaresop/
├── README.md                   ← 本文档
├── platformio.ini              # PlatformIO 项目配置
├── include/
│   ├── pin_config.h            # 引脚 + 参数集中定义
│   ├── ringbuffer.h            # 环形缓冲区头文件
│   ├── comm_protocol.h         # 通信协议头文件
│   ├── pid.h                   # PID 控制器头文件
│   ├── motor.h                 # 电机驱动头文件
│   └── servo.h                 # 舵机驱动头文件
└── src/
    ├── main.cpp                # 主程序 (状态机 + 跟随)
    ├── ringbuffer.c            # 环形缓冲区实现
    ├── comm_protocol.c         # 通信协议实现
    ├── pid.c                   # PID 算法实现
    ├── motor.c                 # TB6612 电机驱动
    └── servo.c                 # SG90 舵机驱动
```
