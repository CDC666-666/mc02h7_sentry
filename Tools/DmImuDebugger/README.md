# DM-IMU-L1 专用调试网页

运行 `start.bat`，浏览器会打开 `http://127.0.0.1:8001`。

选择 H7 Type-C 对应的 **STMicroelectronics Virtual COM Port（当前为 COM11）**，再点击连接。

该网页只适用于已选择 `USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG` 的固件。它显示 CAN3 DM-IMU-L1 的 100 Hz 紧凑调试流，不会发送任何 CAN 控制指令。

若需要恢复综合机器人网页，在 `robot_param_omni_h723_big_yaw_small_yaw_pitch.h` 中将：

```c
#define USB_TELEMETRY_STREAM_MODE USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG
```

改为：

```c
#define USB_TELEMETRY_STREAM_MODE USB_TELEMETRY_STREAM_FULL
```
