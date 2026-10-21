# SP Vision 串口通信说明书

本文说明本项目 `io/gimbal` 串口实现中，上位机（视觉机）与下位机（云台/C板）之间的通信协议。协议是**固定长度二进制协议**，不是 ASCII 文本协议，也不是本项目中另一套 CAN 协议。

## 1. 适用范围

串口协议由以下代码定义：

- `io/gimbal/gimbal.hpp`：帧结构体和字段顺序。
- `io/gimbal/gimbal.cpp`：串口参数、收发和解析逻辑。
- `tools/crc.cpp`：CRC16 算法。

使用串口版云台接口的程序包括 `standard_mpc`、`auto_aim_debug_mpc`、`mt_auto_aim_debug`、`auto_buff_debug_mpc` 以及 `tests/gimbal_test`、`tests/planner_test` 等。

注意：`standard`、`mt_standard`、`uav`、`sentry` 等部分程序实例化的是 `io::CBoard`，使用 SocketCAN，不使用本文的串口帧。两套协议不能混接。

## 2. 物理串口参数

上位机当前代码固定设置：

| 参数 | 值 |
|---|---|
| 波特率 | 115200 bit/s |
| 数据位 | 8 |
| 校验位 | None（无校验） |
| 停止位 | 1 |
| 流控 | None（无硬件/软件流控） |
| 数据格式 | 原始二进制、无换行、无转义 |
| Linux 设备 | 由 YAML `com_port` 指定，如 `/dev/ttyACM0` 或 `/dev/gimbal` |

因此下位机 UART 配置为 **115200-8-N-1**。USB 虚拟串口仍按此设置；不要在终端工具中开启奇偶校验或自动加 `CR/LF`。

上位机配置示例：

```yaml
com_port: "/dev/ttyACM0"
```

Linux 权限检查：

```bash
ls -l /dev/ttyACM0
groups                         # 当前用户应属于 dialout
sudo usermod -aG dialout "$USER"  # 仅首次设置，需要重新登录
```

## 3. 通信方向和帧长度

通信是全双工、无应答号的周期通信：

```text
下位机  ── GimbalToVision（43 字节，建议 100~200 Hz）──>  上位机
下位机  <── VisionToGimbal（29 字节，建议 100 Hz）──────  上位机
```

每个方向都有独立的固定帧长度：

- `GimbalToVision`：43 字节。
- `VisionToGimbal`：29 字节。

帧没有长度字段、版本字段和序号字段。接收端必须根据方向使用正确的固定长度，不能把两个结构体混用。

## 4. 通用字节规则

1. 帧头固定为两个字节 `0x53 0x50`，即 ASCII 字符 `S`、`P`。
2. 结构体使用 packed 布局，字段之间没有填充字节。
3. `float` 为 IEEE-754 binary32，小端字节序（STM32 大多数 Cortex-M 也使用小端）。
4. `uint16_t` 和 CRC16 均为小端发送：低字节在前，高字节在后。
5. `yaw`、`pitch` 及其速度/加速度使用弧度制：rad、rad/s、rad/s²。代码中的规划器直接把这些值写入串口帧，没有角度制转换。
6. `bullet_speed` 单位为 m/s；四元数顺序为 `w, x, y, z`。
7. 不要发送 C/C++ 对象指针、`double` 或经过文本格式化的数字；必须发送字段的二进制值。

## 5. 下位机到上位机：GimbalToVision

该帧由下位机周期发送，上位机接收后更新云台状态、模式、子弹速度和姿态四元数。

结构体：

```cpp
struct __attribute__((packed)) GimbalToVision {
  uint8_t  head[2];       // 'S', 'P'
  uint8_t  mode;
  float    q[4];           // w, x, y, z
  float    yaw;
  float    yaw_vel;
  float    pitch;
  float    pitch_vel;
  float    bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
}; // sizeof = 43
```

| 偏移 | 长度 | 类型 | 字段 | 含义 |
|---:|---:|---|---|---|
| 0 | 2 | uint8[2] | head | 固定 `53 50` |
| 2 | 1 | uint8 | mode | 下位机当前模式 |
| 3 | 4 | float | q[0] | 四元数 w |
| 7 | 4 | float | q[1] | 四元数 x |
| 11 | 4 | float | q[2] | 四元数 y |
| 15 | 4 | float | q[3] | 四元数 z |
| 19 | 4 | float | yaw | 当前 yaw，rad |
| 23 | 4 | float | yaw_vel | yaw 速度，rad/s |
| 27 | 4 | float | pitch | 当前 pitch，rad |
| 31 | 4 | float | pitch_vel | pitch 速度，rad/s |
| 35 | 4 | float | bullet_speed | 弹速，m/s |
| 39 | 2 | uint16 | bullet_count | 累计发弹次数，小端 |
| 41 | 2 | uint16 | crc16 | 对偏移 0~40 共 41 字节计算 |

### mode 定义

| 值 | 枚举 | 含义 |
|---:|---|---|
| 0 | `IDLE` | 空闲 |
| 1 | `AUTO_AIM` | 自瞄 |
| 2 | `SMALL_BUFF` | 小符 |
| 3 | `BIG_BUFF` | 大符 |

收到合法帧后，上位机代码的行为是：

- 将 `q[0..3]` 按 `wxyz` 构造四元数并放入时间队列。
- 更新 `yaw`、`yaw_vel`、`pitch`、`pitch_vel`、`bullet_speed`、`bullet_count`。
- 根据 `mode` 切换视觉功能组。
- 四元数用于图像时间戳对应的姿态插值；四元数模长应接近 1。

## 6. 上位机到下位机：VisionToGimbal

该帧由上位机周期发送，下位机接收后执行云台控制和开火控制。

结构体：

```cpp
struct __attribute__((packed)) VisionToGimbal {
  uint8_t head[2];       // 'S', 'P'
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
}; // sizeof = 29
```

| 偏移 | 长度 | 类型 | 字段 | 含义 |
|---:|---:|---|---|---|
| 0 | 2 | uint8[2] | head | 固定 `53 50` |
| 2 | 1 | uint8 | mode | 控制/开火模式 |
| 3 | 4 | float | yaw | yaw 目标，rad |
| 7 | 4 | float | yaw_vel | yaw 目标速度，rad/s |
| 11 | 4 | float | yaw_acc | yaw 目标加速度，rad/s² |
| 15 | 4 | float | pitch | pitch 目标，rad |
| 19 | 4 | float | pitch_vel | pitch 目标速度，rad/s |
| 23 | 4 | float | pitch_acc | pitch 目标加速度，rad/s² |
| 27 | 2 | uint16 | crc16 | 对偏移 0~26 共 27 字节计算 |

### mode 定义

| 值 | 含义 | 下位机动作 |
|---:|---|---|
| 0 | 不控制 | 视觉不接管云台，也不允许本帧开火 |
| 1 | 控制但不开火 | 执行 yaw/pitch 目标，禁止开火 |
| 2 | 控制且开火 | 执行 yaw/pitch 目标，并允许开火 |

上位机调用：

```cpp
gimbal.send(control, fire, yaw, yaw_vel, yaw_acc,
            pitch, pitch_vel, pitch_acc);
```

其中 `mode` 由代码自动转换：`control=false` 为 0；`control=true && fire=false` 为 1；`control=true && fire=true` 为 2。程序退出时会发送一帧 mode=0、所有数值为 0 的安全帧。

下位机应将 mode=0 视为安全状态；建议在连续一段时间（例如 100~300 ms）未收到合法帧时也自动退出视觉控制并禁止开火。

## 7. CRC16 算法

项目使用查表 CRC16，等价参数如下：

| 参数 | 值 |
|---|---|
| 初值 | `0xFFFF` |
| 多项式（反射形式） | `0x8408` |
| 输入反射 | 已反射处理 |
| 输出异或 | `0x0000` |
| CRC 覆盖范围 | 不包含最后 2 个 CRC 字节 |
| CRC 在线路上的顺序 | low byte、high byte |

等价的无查表 C 实现：

```c
uint16_t sp_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408)
                            : (uint16_t)(crc >> 1);
    }
    return crc;
}

static inline uint16_t sp_get_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void sp_put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}
```

校验示例：

```c
// rx 是完整帧，frame_len 为 43 或 29
uint16_t expected = sp_crc16(rx, frame_len - 2);
uint16_t received = (uint16_t)rx[frame_len - 2]
                  | ((uint16_t)rx[frame_len - 1] << 8);
bool valid = (expected == received);
```

不要把 CRC 字节按大端读取。上位机的 `check_crc16()` 虽然从数组尾部重组成整数，但它实际要求线路顺序仍是低字节在前。

## 8. 下位机发送 GimbalToVision 的推荐写法

不要依赖未确认的编译器对齐；推荐用字节数组显式写入，或使用 packed 结构体并配合 `static_assert`。

```c
uint8_t tx[43] = {0};
tx[0] = 0x53;
tx[1] = 0x50;
tx[2] = mode;

// 下面的函数按小端复制 float；Cortex-M 小端平台可用 memcpy，
// 也可以自行实现 put_float_le。
memcpy(&tx[3],  &qw, 4);
memcpy(&tx[7],  &qx, 4);
memcpy(&tx[11], &qy, 4);
memcpy(&tx[15], &qz, 4);
memcpy(&tx[19], &yaw, 4);
memcpy(&tx[23], &yaw_vel, 4);
memcpy(&tx[27], &pitch, 4);
memcpy(&tx[31], &pitch_vel, 4);
memcpy(&tx[35], &bullet_speed, 4);
sp_put_u16_le(&tx[39], bullet_count);

uint16_t crc = sp_crc16(tx, 41);
sp_put_u16_le(&tx[41], crc);
HAL_UART_Transmit(&huartX, tx, sizeof(tx), 10);
```

发送周期建议 5~10 ms（100~200 Hz）。上位机启动时会额外发送一次裸的 `0x53 0x50` 两字节唤醒序列；它不是完整数据帧，下位机可以忽略，也可以将其作为链路探测。

## 9. 下位机接收 VisionToGimbal 的推荐状态机

由于串口可能按任意分片返回数据，不能假设一次 UART 接收恰好得到 29 字节。推荐逐字节状态机：

```text
WAIT_HEAD_0: 等待 0x53
WAIT_HEAD_1: 等待 0x50；若不是 0x50，重新判断当前字节是否为 0x53
READ_BODY:   再收 27 字节（包含 mode、6 个 float、CRC）
CHECK:       CRC16(payload[0..26]) 与 payload[27..28] 比较
VALID:       解析并执行；失败则丢弃当前帧并回到 WAIT_HEAD_0
```

注意：有效帧长度是 29 字节，CRC 覆盖前 27 字节。帧头可能出现在浮点数据中，因此不能简单地在任意位置看到 `SP` 就认定是新帧；应以“固定长度 + CRC”共同确认。

收到有效帧后建议：

1. 先校验帧头和 CRC，再解析 float。
2. 检查所有 float 是否为有限值（非 NaN/Inf）。
3. 检查 yaw/pitch 及速度、加速度是否在机械允许范围内。
4. mode=0 时清除视觉控制输出和开火请求。
5. 记录最后合法帧时间，实现通信超时保护。

## 10. 上位机接收 GimbalToVision 的实际行为

上位机 `Gimbal` 对串口启动流程如下：

1. 从 YAML 读取 `com_port`。
2. 以 115200-8-N-1 打开串口。
3. 启动接收线程并发送一次 `53 50`。
4. 等待收到至少一帧合法的 `GimbalToVision`，构造函数才返回。
5. 后续持续解析 43 字节帧，并用时间队列对四元数做球面插值。

因此，如果下位机不上报合法的 43 字节帧，上位机程序会停在 `Gimbal` 构造阶段，看起来像“程序没有继续运行”。联调时应先让下位机以固定值周期发送状态帧，再启动视觉程序。

## 11. 联调顺序

### 第一步：确认设备和权限

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
sudo usermod -aG dialout "$USER"
```

重新登录后确认串口设备名，并写入配置文件。

### 第二步：只接收下位机状态帧

让下位机每 10 ms 发送一帧 43 字节，先固定发送：

```text
head       = 53 50
mode       = 00
q          = 1.0, 0.0, 0.0, 0.0
yaw/pitch  = 0
速度       = 0
bullet     = 0
CRC        = 按本文算法计算
```

启动使用串口版的 `tests/gimbal_test` 或主程序。日志应出现 `First q received`（以及接收帧统计）。

### 第三步：验证上位机下发

使用 `tests/gimbal_test` 时，程序会反复发送 yaw=1 rad、pitch=0 的控制帧；加 `-f` 才会周期性请求开火：

```bash
./build-canuse/gimbal_test configs/ttyACM0.yaml
./build-canuse/gimbal_test -f configs/ttyACM0.yaml
```

下位机抓取到的每帧应为 29 字节，`mode` 分别为 1（控制不打弹）或 2（控制并允许开火）。按 Ctrl-C 或退出时，最后一帧应为 mode=0。

### 第四步：接入视觉算法

确认状态帧稳定后再运行 `standard_mpc` 等程序。下位机应继续周期上报姿态和弹速，同时接收上位机的控制帧。建议先关闭实际发射机构，仅观察 yaw/pitch 响应。

## 12. 常见错误

| 现象 | 常见原因 | 检查方法 |
|---|---|---|
| 上位机启动后卡住 | 没有收到合法 43 字节状态帧 | 检查下位机是否发送、CRC 是否正确 |
| CRC 全部失败 | CRC 覆盖长度或字节序错误 | 43 字节覆盖 0~40；29 字节覆盖 0~26；CRC 低字节在前 |
| 模式始终 IDLE | `mode` 偏移错误或收到 CAN 帧 | 串口帧第 3 字节才是 mode |
| yaw/pitch 数值巨大 | 把 float 当整数、角度制或大小端错误 | 逐字节检查 IEEE-754 小端解析 |
| 帧偶尔错位 | 假设一次 DMA/中断就是一帧 | 使用帧头搜索、固定长度和 CRC 状态机 |
| 云台无动作 | mode=0、下发频率过低或控制超时 | 先验证 mode=1，再检查超时保护 |
| 能收不能发 | 串口 TX/RX 交叉或只接了单向线 | 上位机 TX 接下位机 RX，反之亦然，并共地 |
| 运行权限错误 | 用户无串口设备权限 | `ls -l`、加入 `dialout` |

## 13. 接线

对于 TTL UART/USB-UART：

```text
上位机 TX  -> 下位机 RX
上位机 RX  <- 下位机 TX
上位机 GND <-> 下位机 GND
```

电平必须匹配（常见为 3.3 V TTL）。不要把 RS-232 电平直接接到 MCU UART，也不要在未确认电压前把 5 V 信号接入 3.3 V 引脚。USB 虚拟串口设备通常已完成电平转换，但仍需确认具体转换器和 C板接口定义。

## 14. 与 CAN 协议的区别

如果程序使用 `io::CBoard`，其通信内容是 CAN：

- 四元数 CAN ID 默认 `0x100`。
- 弹速/模式 CAN ID 默认 `0x101`。
- 视觉指令 CAN ID 默认 `0xFF`。
- yaw/pitch 不是 float，而是按 `1e4` 缩放的 int16。

这些字段和本文串口协议完全不同。只有实例化 `io::Gimbal` 的程序才使用 `SP`、43 字节和 29 字节串口帧。

