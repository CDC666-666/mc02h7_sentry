# H7 IMU Visualizer

This tool reads the H723 UART10 telemetry port through a USB-to-TTL serial adapter in Chrome or Edge.

## Use

1. Build and program the H7 firmware that contains `Telemetry_Task`.
2. Connect a 3.3 V USB-to-TTL adapter: board `PE3 / USART10_TX` to adapter `RX`, and board `GND` to adapter `GND`. Windows should create a COM port. The adapter TX line is optional because this tool only receives telemetry.
3. Double-click `start.bat`. It uses Python 3 when present, otherwise Node.js. No package installation is needed.
4. In the opened Chrome or Edge page, click `选择 UART10 串口`, select the USB-to-TTL COM port, then click `Connect`. The tool opens it at 921600 bit/s.
5. Use `Start Recording` for a stationary ten-minute test and `Download CSV` when finished.

The browser must be Chrome or Edge. The page is served from `localhost` because browser serial access is not allowed from a plain `file://` page.

## Signals

- `raw_gx/raw_gy/raw_gz`: gyro after sensor/board coordinate rotation, before startup bias correction, unit rad/s.
- `gyro_x/gyro_y/gyro_z`: gyro sent into the EKF after bias correction, unit rad/s.
- `bias_x/bias_y/bias_z`: startup bias captured after temperature stabilization, unit rad/s.
- `yaw/pitch/roll`: EKF Euler angles, unit degrees.
- `Spatial attitude`: the red/green/blue needles are the rotated IMU X/Y/Z axes.
  The center point is fixed; the view updates from the same `yaw/pitch/roll` telemetry.
- `ch0` to `ch3`: ET08A main stick channels, normalized to about `-660` to `+660`.
- `sw0/sw1`: ET08A three-position switch state. The page highlights `UP`, `MID`, or `DOWN` in real time.

The firmware sends FireWater-compatible text over UART10 at 25 Hz. VOFA+ can use the same USB-to-TTL COM port with the FireWater protocol, but close this browser tool first because only one program can open a COM port at a time.
