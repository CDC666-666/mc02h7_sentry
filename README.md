# H7 机器人控制工程

本仓库是基于 STM32H723 控制板的机器人固件工程，包含底盘、云台、发射机构、遥控器、IMU、CAN 电机和遥测功能。

开发任务和后续硬件适配见 [Document/开发任务清单.md](Document/开发任务清单.md)。

## 快速开始

1. 用 EIDE/VS Code 打开 `MDK-ARM` 目录。
2. 在 `Application/Robot/Inc/robot_param.h` 选择 `ROBOT_PROFILE`。
3. 在所选 `robot_param_*.h` 中核对 CAN、ID、方向、减速比、零位和限位。
4. 执行 Rebuild，烧录前让车轮架空、云台留出安全空间。
5. 保持遥控器安全挡，上电后先确认遥测、CAN 和电机反馈。

当前默认配置：

```c
#define ROBOT_PROFILE ROBOT_PROFILE_OMNI_H723_THREE_AXIS
```

## Profile

| Profile | 配置文件 | 已有代码功能 |
| --- | --- | --- |
| `ROBOT_PROFILE_OMNI_H723_DIRECT` | `robot_param_omni_h723_gimbal.h` | 全向轮、两轴可选云台、发射机构 |
| `ROBOT_PROFILE_OMNI_H723_THREE_AXIS` | `robot_param_omni_h723_big_yaw_small_yaw_pitch.h` | 全向轮、大 Yaw/小 Yaw/Pitch 三轴云台 |
| `ROBOT_PROFILE_MECANUM_INFANTRY` | `robot_param_mecanum_h723_infantry.h` | 麦轮底盘、Yaw/Pitch GM6020、双摩擦轮 |
| `ROBOT_PROFILE_STEERING_FOLD_H723` | `robot_param_steer_fold_h723.h` | 四舵轮和折叠/Yaw/Pitch 云台接口 |

## 代码目录

```text
Application/Robot       Profile 和机器人参数
Application/Chassis     全向轮、麦轮、舵轮、功率限制
Application/Gimbal      两轴、三轴、折叠云台
Application/Shoot       摩擦轮和拨弹盘
Application/RobotCmd    电机抽象、CAN 命令和反馈
Application/Task        FreeRTOS 控制、检测和遥测任务
Components/Device       BMI088、遥控器、裁判系统等设备
BSP                      CAN、UART、SPI、PWM、蜂鸣器等驱动
Tools/ImuVisualizer     串口遥测网页工具
```

## 当前功能

- 三路 FDCAN 均使用经典 CAN、8 字节帧、1 Mbps。
- 支持 M3508、M2006、GM6020、DM4310 和 EVO4310 电机对象、反馈解析与命令发送。
- DJI M3508/M2006 使用电流控制帧；GM6020 使用电压控制帧；DM4310/EVO4310 支持独立 MIT 帧。
- 支持 ET08A/ELRS 的 SBUS 输入，提供摇杆和拨杆状态。
- 支持 BMI088 姿态解算、IMU 加热、启动零偏和控制板安装矩阵。
- 支持 H7 内部温度采样、CAN 收发诊断和电机状态遥测。
- 支持底盘四轮速度 PID 及功率限制模块。
- 支持 USB CDC 和 USART10 遥测链路；网页工具通过串口读取遥测数据。

## 常用硬件映射

实际接线以所选 Profile 中的参数为准。修改 CAN、ID、方向、零位或电机型号后，必须重新进行单电机低速测试。

### 全向轮三轴配置

| 模块 | 总线 | 电机/ID |
| --- | --- | --- |
| 底盘 | CAN1 | M3508，ID1~4 |
| 大 Yaw | CAN1 | DM4310，命令 ID `0x01`，反馈 ID `0x51` |
| 小 Yaw | CAN2 | GM6020，ID1 |
| Pitch | CAN2 | DM4310，命令 ID `0x02`，反馈 ID `0x52` |
| 发射机构 | CAN3 | 由 Profile 中拨弹盘和摩擦轮参数决定 |

### 麦轮步兵配置

| 总线 | 电机 |
| --- | --- |
| CAN1 | 底盘 M3508 ID1~4；Yaw GM6020 ID1 |
| CAN2 | Pitch GM6020 ID1；摩擦轮 M3508 ID1、ID2 |

同一条 CAN 上 GM6020 和 M3508 的数字 ID 可以相同：两类电机使用不同的控制帧和反馈 ID。当前麦轮 profile 未配置拨弹盘。

## 遥测网页

启动脚本：`Tools/ImuVisualizer/start.bat`。

1. 连接控制板 USB 或遥测串口。
2. 运行 `start.bat`。
3. 在浏览器打开 `http://localhost:8000`。
4. 选择对应 COM 口后连接。

网页可显示 IMU、H7 温度、遥控器、电机状态和 CAN 诊断。串口同一时间只能被一个软件占用，使用网页前应关闭 VOFA、串口助手和 EIDE 串口监视器。页面未更新时使用 `Ctrl+F5`，并确认已烧录新固件。

## 安全调试

1. 检查 CANH、CANL、GND 和两端 120 Ω 终端电阻。
2. 上电后先确认遥控器在线、CAN 无总线错误、电机反馈正常。
3. 保持安全挡，确认电机未使能。
4. 先进行单电机低速正反转测试，再接入整车机构。
5. 云台先确认机械零位和限位，再开启闭环。

常见故障优先检查：活动 Profile、CAN/ID、DM 使能与反馈 ID、串口占用和 SWD 连接状态。
