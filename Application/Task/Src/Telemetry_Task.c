#include "Telemetry_Task.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "Bmi088.h"
#include "bsp_mcu.h"
#include "chassis.h"
#include "extra_imu.h"
#include "INS_Task.h"
#include "gimbal.h"
#include "remote_control.h"
#include "robot_param.h"
#include "robot_can.h"
#include "robot_motor.h"
#include "shoot_fric.h"
#include "slam.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* USB 设备句柄在 usb_device.c 中定义；CubeMX 生成的 usb_device.h 未导出它。 */
extern USBD_HandleTypeDef hUsbDeviceHS;

/* USB Type-C CDC 为网页遥测出口。完整 CSV 帧包含全部电机反馈，按 50 Hz 发送。 */
#define TELEMETRY_PERIOD_MS            20U
#define TELEMETRY_DIAGNOSTIC_PERIOD_MS 200U
#define SLAM_DEBUG_PERIOD_MS           10U
#define MCU_TEMP_SAMPLE_PERIOD_MS 100U
#define TELEMETRY_BUFFER_SIZE 3072U
#define MOTOR_FEEDBACK_TIMEOUT_MS 200U

volatile uint32_t Telemetry_FrameCount;
volatile uint32_t Telemetry_DroppedFrameCount;

static uint8_t TelemetryTransmit(uint8_t *tx_buffer, const char *format_buffer, uint16_t length)
{
#if (USB_CDC_MODE == USB_CDC_MODE_TELEMETRY)
    if ((tx_buffer == 0) || (format_buffer == 0) || (length == 0U) ||
        /* 上电后 USB 还未完成枚举时不可启动首次 IN 传输；否则 CDC
         * TxState 可能停在 busy，后续所有遥测帧都会被拒绝。 */
        (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) ||
        (CDC_TxReady_HS() == 0U)) {
        Telemetry_DroppedFrameCount++;
        return 0U;
    }

    memcpy(tx_buffer, format_buffer, length);
    /* USB HS DMA 从 AXI SRAM 读取，发送前清理覆盖范围内的 D-Cache。 */
    SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, (int32_t)((length + 31U) & ~31U));
    if (CDC_Transmit_HS(tx_buffer, length) == USBD_OK) {
        Telemetry_FrameCount++;
        return 1U;
    } else {
        Telemetry_DroppedFrameCount++;
        return 0U;
    }
#else
    if ((tx_buffer == 0) || (format_buffer == 0) || (length == 0U)) {
        Telemetry_DroppedFrameCount++;
        return 0U;
    }
    memcpy(tx_buffer, format_buffer, length);
    if (HAL_UART_Transmit(&huart10, tx_buffer, length, 50U) == HAL_OK) {
        Telemetry_FrameCount++;
        return 1U;
    } else {
        Telemetry_DroppedFrameCount++;
        return 0U;
    }
#endif
}

static void TelemetryAppendMotorFeedback(char *buffer, size_t buffer_size, int *length)
{
    uint8_t motor_count;

    if (buffer == 0 || length == 0 || *length < 0 || (size_t)*length >= buffer_size) return;
    motor_count = RobotMotor_GetRegisteredCount();
    *length += snprintf(&buffer[*length], buffer_size - (size_t)*length,
                        "%smotor_count:%u", *length == 0 ? "" : ",",
                        (unsigned int)motor_count);
    for (uint8_t index = 0U; index < motor_count; index++) {
        const RobotMotor_t *motor = RobotMotor_GetRegistered(index);
        uint32_t feedback_age_ms;

        if (motor == 0) continue;
        feedback_age_ms = HAL_GetTick() - motor->feedback.last_feedback_ms;
        *length += snprintf(&buffer[*length], buffer_size - (size_t)*length,
                            ",m%u_type:%u,m%u_can:%u,m%u_id:%u,m%u_fb_id:%u,"
                            "m%u_online:%u,m%u_age:%lu,m%u_pos:%.4f,m%u_vel:%.4f,"
                            "m%u_current:%.1f,m%u_temp:%.1f",
                            (unsigned int)index, (unsigned int)motor->type,
                            (unsigned int)index, (unsigned int)motor->can_index,
                            (unsigned int)index, (unsigned int)motor->command_id,
                            (unsigned int)index, (unsigned int)motor->feedback_id,
                            (unsigned int)index, (unsigned int)RobotMotor_HasFreshFeedback(
                                motor, MOTOR_FEEDBACK_TIMEOUT_MS),
                            (unsigned int)index, (unsigned long)feedback_age_ms,
                            (unsigned int)index, motor->feedback.position,
                            (unsigned int)index, motor->feedback.velocity,
                            (unsigned int)index, motor->feedback.current,
                            (unsigned int)index, motor->feedback.temperature);
        if (*length < 0 || (size_t)*length >= buffer_size) return;
    }
}

void Telemetry_Task(void const *argument)
{
    /* 遥测组帧与发送缓冲区统一放在 AXI SRAM；DMA 实际只读取 tx_buffer。
     * tx_buffer 按 32 字节 Cache 行对齐，发送前清理其 D-Cache 即可。 */
    __attribute__((section(".AXI_SRAM"), aligned(32))) static uint8_t tx_buffer[TELEMETRY_BUFFER_SIZE];
    __attribute__((section(".AXI_SRAM"), aligned(32))) static char format_buffer[TELEMETRY_BUFFER_SIZE];
    uint32_t mcu_temp_last_sample_ms = 0U;
    uint32_t heartbeat_last_send_ms = 0U;
    uint32_t extra_imu_debug_last_send_ms = 0U;
    uint32_t extra_imu_request_last_send_ms = 0U;
    uint32_t diagnostic_last_send_ms = 0U;
    uint32_t slam_debug_last_send_ms = 0U;
    uint32_t telemetry_last_send_ms = 0U;
    uint8_t motor_frame_pending = 0U;
    SlamPose_t slam_pose;

    (void)argument;

    for (;;) {
        int length;
        uint8_t full_diagnostics;

        /* 独立短心跳：不依赖浮点格式化或 CAN 数据。它用于确认 USB CDC
         * 发送端和遥测任务都已运行；网页会把它识别为一条有效基础帧。 */
        if ((HAL_GetTick() - heartbeat_last_send_ms) >= 1000U) {
            static const char heartbeat[] = "temp:0,yaw:0,telemetry_heartbeat:1\r\n";
            heartbeat_last_send_ms = HAL_GetTick();
            (void)TelemetryTransmit(tx_buffer, heartbeat, (uint16_t)(sizeof(heartbeat) - 1U));
        }

        RobotCan_UpdateDiagnostics();

        /* DM-IMU 请求/应答轮询必须独立于 USB 页面，否则切换页面会停止请求。 */
        if ((HAL_GetTick() - extra_imu_request_last_send_ms) >= EXTRA_IMU_REQUEST_PERIOD_MS) {
            extra_imu_request_last_send_ms = HAL_GetTick();
            (void)ExtraImu_RequestNext();
        }

#if (USB_TELEMETRY_STREAM_MODE == USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG)
        /* 仿照官方例程持续轮询四类只读寄存器。即使模块已经在线也不停止，
         * 否则被动模式只能返回首帧，无法持续更新完整姿态数据。 */
        /* 顶部 DM-IMU-L1 专用调试流：100 Hz 紧凑帧。
         * 与综合遥测互斥，防止大帧占用 USB CDC IN 端点。 */
        if ((HAL_GetTick() - extra_imu_debug_last_send_ms) >= EXTRA_IMU_DEBUG_PERIOD_MS) {
            extra_imu_debug_last_send_ms = HAL_GetTick();
            length = snprintf(format_buffer, sizeof(format_buffer),
                              "dmimu:1,t:%lu,online:%u,can:%u,id:%lu,rx_id:%lu,bus_rx:%lu,bus_last_id:%lu,req_tx:%lu,req_reject:%lu,tx_id:%lu,tx0:%u,tx1:%u,tx2:%u,tx3:%u,tx4:%u,tx5:%u,tx6:%u,tx7:%u,can_tec:%lu,can_rec:%lu,can_lec:%lu,can_activity:%lu,can_ep:%u,can_warn:%u,can_busoff:%u,age:%lu,frames:%lu,invalid:%lu,mask:%u,type:%u,"
                              "temp:%.2f,pitch:%.3f,yaw:%.3f,roll:%.3f,"
                              "gx:%.4f,gy:%.4f,gz:%.4f,ax:%.3f,ay:%.3f,az:%.3f,"
                              "qw:%.5f,qx:%.5f,qy:%.5f,qz:%.5f,"
                              "raw0:%u,raw1:%u,raw2:%u,raw3:%u,raw4:%u,raw5:%u,raw6:%u,raw7:%u\r\n",
                              (unsigned long)HAL_GetTick(),
                              (unsigned int)ExtraImu_IsOnline(),
                              (unsigned int)EXTRA_IMU_CAN,
                              (unsigned long)EXTRA_IMU_CAN_ID,
                              (unsigned long)extra_imu.last_identifier,
                              (unsigned long)RobotCan_RxCount[EXTRA_IMU_CAN],
                              (unsigned long)RobotCan_LastRxId[EXTRA_IMU_CAN],
                              (unsigned long)ExtraImu_RequestCount,
                              (unsigned long)ExtraImu_RequestRejectedCount,
                              (unsigned long)RobotCan_LastTxId[EXTRA_IMU_CAN],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][0],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][1],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][2],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][3],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][4],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][5],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][6],
                              (unsigned int)RobotCan_LastTxData[EXTRA_IMU_CAN][7],
                              (unsigned long)RobotCan_TxErrorCount[EXTRA_IMU_CAN],
                              (unsigned long)RobotCan_RxErrorCount[EXTRA_IMU_CAN],
                              (unsigned long)RobotCan_LastErrorCode[EXTRA_IMU_CAN],
                              (unsigned long)RobotCan_Activity[EXTRA_IMU_CAN],
                              (unsigned int)RobotCan_ErrorPassive[EXTRA_IMU_CAN],
                              (unsigned int)RobotCan_Warning[EXTRA_IMU_CAN],
                              (unsigned int)RobotCan_BusOff[EXTRA_IMU_CAN],
                              (unsigned long)(HAL_GetTick() - extra_imu.last_update_ms),
                              (unsigned long)extra_imu.valid_frames,
                              (unsigned long)extra_imu.invalid_frames,
                              (unsigned int)extra_imu.frame_mask,
                              (unsigned int)extra_imu.last_data_type,
                              extra_imu.temperature_c,
                              extra_imu.pitch_deg, extra_imu.yaw_deg, extra_imu.roll_deg,
                              extra_imu.gyro_rad_s[0], extra_imu.gyro_rad_s[1], extra_imu.gyro_rad_s[2],
                              extra_imu.acceleration_mps2[0], extra_imu.acceleration_mps2[1],
                              extra_imu.acceleration_mps2[2],
                              extra_imu.quaternion[0], extra_imu.quaternion[1],
                              extra_imu.quaternion[2], extra_imu.quaternion[3],
                              (unsigned int)extra_imu.last_data[0], (unsigned int)extra_imu.last_data[1],
                              (unsigned int)extra_imu.last_data[2], (unsigned int)extra_imu.last_data[3],
                              (unsigned int)extra_imu.last_data[4], (unsigned int)extra_imu.last_data[5],
                              (unsigned int)extra_imu.last_data[6], (unsigned int)extra_imu.last_data[7]);
            if ((length > 0) && ((uint32_t)length < sizeof(tx_buffer))) {
                (void)TelemetryTransmit(tx_buffer, format_buffer, (uint16_t)length);
            }
        }
        osDelay(1U);
        continue;
#endif

        if (HAL_GetTick() - mcu_temp_last_sample_ms >= MCU_TEMP_SAMPLE_PERIOD_MS) {
            mcu_temp_last_sample_ms = HAL_GetTick();
            MCU_Temperature_Update();
        }

        full_diagnostics = (HAL_GetTick() - diagnostic_last_send_ms >=
                            TELEMETRY_DIAGNOSTIC_PERIOD_MS) ? 1U : 0U;
        if (full_diagnostics != 0U) diagnostic_last_send_ms = HAL_GetTick();
        Slam_GetPose(&slam_pose);

        /* 专用 SLAM 网页只接收这一类紧凑帧。Type-C 综合调试模式只输出完整 CSV，
         * 避免与高速 SLAM 帧争用同一个 USB CDC 发送端点。 */
#if (USB_CDC_MODE != USB_CDC_MODE_TELEMETRY)
        if ((HAL_GetTick() - slam_debug_last_send_ms) >= SLAM_DEBUG_PERIOD_MS) {
            slam_debug_last_send_ms = HAL_GetTick();
            length = snprintf(format_buffer, sizeof(format_buffer),
                              "slamdbg:1,t:%lu,online:%u,x_count:%d,y_count:%d,z_count:%d,"
                              "x:%.5f,y:%.5f,z:%.5f,roll_count:%d,pitch_count:%d,yaw_count:%d,"
                              "roll:%.2f,pitch:%.2f,yaw:%.2f,"
                              "age:%lu,frames:%lu,format_errors:%lu,uart_errors:%lu,overflow:%lu\\r\\n",
                              (unsigned long)HAL_GetTick(),
                              (unsigned int)slam_pose.online,
                              (int)slam_pose.x_count, (int)slam_pose.y_count,
                              (int)slam_pose.z_count,
                              slam_pose.x_m, slam_pose.y_m, slam_pose.z_m,
                              (int)slam_pose.roll_count, (int)slam_pose.pitch_count,
                              (int)slam_pose.yaw_count,
                              slam_pose.roll_deg, slam_pose.pitch_deg, slam_pose.yaw_deg,
                              (unsigned long)(HAL_GetTick() - slam_pose.last_update_ms),
                              (unsigned long)slam_pose.valid_frames,
                              (unsigned long)slam_pose.format_errors,
                              (unsigned long)slam_pose.uart_errors,
                              (unsigned long)slam_pose.rx_overflow);
            if ((length > 0) && ((uint32_t)length < sizeof(tx_buffer))) {
                TelemetryTransmit(tx_buffer, format_buffer, (uint16_t)length);
            }
        }
#endif

        /* USB CDC 在 IN 传输完成前仍会占用调用者缓冲区。
         * 因此先在独立缓冲区组帧，USB 忙时直接丢弃本次采样。 */
        /* 原综合网页仍保持 50 Hz；只让新增的 SLAM 紧凑帧使用 100 Hz。 */
        if ((HAL_GetTick() - telemetry_last_send_ms) >= TELEMETRY_PERIOD_MS) {
            telemetry_last_send_ms = HAL_GetTick();
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
        if (length > 0 && (size_t)length < sizeof(format_buffer)) {
        length = snprintf(format_buffer, sizeof(format_buffer),
                          "temp:%.2f,mcu_temp:%.2f,mcu_temp_ok:%u,mcu_temp_raw:%u,"
                          "mcu_vref_raw:%u,mcu_vdda_mv:%lu,ready:%u,bias_count:%u,"
                          "imu_gyro_frames:%lu,imu_dma_errors:%lu,"
                          "raw_gx:%.5f,raw_gy:%.5f,raw_gz:%.5f,"
                          "gyro_x:%.5f,gyro_y:%.5f,gyro_z:%.5f,"
                          "bias_x:%.5f,bias_y:%.5f,bias_z:%.5f,"
                          "yaw:%.3f,pitch:%.3f,roll:%.3f,yaw_rate:%.4f,"
                          "rc_lost:%u,ch0:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d,"
                          "sw0:%u,sw1:%u,sbus_frames:%lu,sbus_errors:%lu,"
                          "gimbal_mode:%u,armed:%u,"
                          "yaw_online:%u,pitch_online:%u,yaw_state:%u,pitch_state:%u,"
                          "yaw_fb_pos:%.5f,pitch_fb_pos:%.5f,"
                          "yaw_fb_vel:%.5f,pitch_fb_vel:%.5f,"
                          "yaw_ref:%.5f,pitch_ref:%.5f,"
                          "yaw_target:%.3f,pitch_target:%.3f,"
                          "yaw_err:%.3f,pitch_err:%.3f,"
                          "yaw_cmd:%.5f,pitch_cmd:%.5f,"
                          "yaw_sp_pos:%.5f,pitch_sp_pos:%.5f,"
                          "yaw_sp_vel:%.5f,pitch_sp_vel:%.5f,"
                          "yaw_joint:%.5f,pitch_joint:%.5f,"
                          "pitch_imu_lo:%.3f,pitch_imu_hi:%.3f,"
                          "pitch_motor_lo:%.5f,pitch_motor_hi:%.5f,"
                          "trigger_online:%u,trigger_mode:%u,trigger_target:%.4f,"
                          "trigger_feedback:%.4f,trigger_current:%.1f,trigger_sent:%u,"
                           "can3_rx:%lu,can3_last_id:%lu,can3_tx:%lu,can3_tx_reject:%lu,"
                           "can3_tx_id:%lu,can3_tx_b0:%u,can3_tx_b1:%u,can3_tx_b2:%u,"
                           "can3_tx_b3:%u,can3_tx_b4:%u,can3_tx_b5:%u,can3_tx_b6:%u,can3_tx_b7:%u,"
                           "can3_tec:%lu,can3_rec:%lu,can3_lec:%lu,can3_activity:%lu,"
                           "can3_ep:%u,can3_warning:%u,can3_busoff:%u,"
                           "chassis_mode:%u,chassis_vx:%.3f,chassis_vy:%.3f,chassis_wz:%.3f,"
                           "chassis_i1:%.1f,chassis_i2:%.1f,chassis_i3:%.1f,chassis_i4:%.1f,"
                           "can1_rx:%lu,can1_tx:%lu,can1_tx_reject:%lu,can1_tx_id:%lu",
                          BMI088_Info.Temperature, MCU_Temperature_C,
                          (unsigned int)MCU_TemperatureValid,
                          (unsigned int)MCU_TemperatureRaw,
                          (unsigned int)MCU_VrefIntRaw,
                          (unsigned long)MCU_Vdda_mV,
                          (unsigned int)INS_ImuReady,
                          (unsigned int)INS_GyroBiasSampleCount,
                          (unsigned long)BMI088_DMA_GyroFrameCount,
                          (unsigned long)BMI088_DMA_ErrorCount,
                          INS_GyroRaw[0], INS_GyroRaw[1], INS_GyroRaw[2],
                          INS_Info.Gyro[0], INS_Info.Gyro[1], INS_Info.Gyro[2],
                          INS_GyroBias[0], INS_GyroBias[1], INS_GyroBias[2],
                          INS_Info.Yaw_Angle, INS_Info.Pitch_Angle,
                          INS_Info.Roll_Angle, INS_Info.Yaw_Gyro,
                          (unsigned int)remote_ctrl.rc_lost,
                          (int)remote_ctrl.rc.ch[0], (int)remote_ctrl.rc.ch[1],
                          (int)remote_ctrl.rc.ch[2], (int)remote_ctrl.rc.ch[3],
                          (int)remote_ctrl.rc.ch[4],
                          (unsigned int)remote_ctrl.rc.s[0],
                          (unsigned int)remote_ctrl.rc.s[1],
                          (unsigned long)remote_sbus_frame_count,
                          (unsigned long)remote_sbus_error_count,
                          (unsigned int)gimbal.mode, (unsigned int)robot_command_armed,
                          (unsigned int)gimbal.yaw.online, (unsigned int)gimbal.pitch.online,
                          (unsigned int)gimbal.yaw.feedback.state,
                          (unsigned int)gimbal.pitch.feedback.state,
                          gimbal.yaw.feedback.position, gimbal.pitch.feedback.position,
                          gimbal.yaw.feedback.velocity, gimbal.pitch.feedback.velocity,
                          gimbal_imu_yaw_motor_reference, gimbal_imu_pitch_motor_reference,
                          gimbal_imu_yaw_target_deg, gimbal_imu_pitch_target_deg,
                          gimbal_imu_yaw_error_deg, gimbal_imu_pitch_error_deg,
                          gimbal_imu_yaw_position_cmd, gimbal_imu_pitch_position_cmd,
                          gimbal.yaw.setpoint.position, gimbal.pitch.setpoint.position,
                          gimbal.yaw.setpoint.velocity, gimbal.pitch.setpoint.velocity,
                          gimbal_yaw_joint_position_rad, gimbal_pitch_joint_position_rad,
                          gimbal_imu_pitch_lower_limit_deg,
                          gimbal_imu_pitch_upper_limit_deg,
                          gimbal_pitch_motor_lower_limit,
                          gimbal_pitch_motor_upper_limit,
                          (unsigned int)shoot_fric.trigger_online,
                          (unsigned int)shoot_fric.trigger_mode,
                          shoot_fric.trigger_target_velocity,
                          shoot_fric.trigger_feedback_velocity,
                          shoot_fric.trigger.setpoint.current,
                          (unsigned int)shoot_fric.trigger_command_sent,
                          (unsigned long)RobotCan_RxCount[3],
                          (unsigned long)RobotCan_LastRxId[3],
                          (unsigned long)RobotCan_TxQueuedCount[3],
                          (unsigned long)RobotCan_TxRejectedCount[3],
                          (unsigned long)RobotCan_LastTxId[3],
                          (unsigned int)RobotCan_LastTxData[3][0],
                          (unsigned int)RobotCan_LastTxData[3][1],
                          (unsigned int)RobotCan_LastTxData[3][2],
                          (unsigned int)RobotCan_LastTxData[3][3],
                          (unsigned int)RobotCan_LastTxData[3][4],
                           (unsigned int)RobotCan_LastTxData[3][5],
                           (unsigned int)RobotCan_LastTxData[3][6],
                           (unsigned int)RobotCan_LastTxData[3][7],
                           (unsigned long)RobotCan_TxErrorCount[3],
                           (unsigned long)RobotCan_RxErrorCount[3],
                           (unsigned long)RobotCan_LastErrorCode[3],
                           (unsigned long)RobotCan_Activity[3],
                           (unsigned int)RobotCan_ErrorPassive[3],
                           (unsigned int)RobotCan_Warning[3],
                           (unsigned int)RobotCan_BusOff[3],
                           (unsigned int)chassis.mode,
                           chassis.reference.vx, chassis.reference.vy, chassis.reference.wz,
                           chassis.motor[0].setpoint.current, chassis.motor[1].setpoint.current,
                           chassis.motor[2].setpoint.current, chassis.motor[3].setpoint.current,
                           (unsigned long)RobotCan_RxCount[1],
                           (unsigned long)RobotCan_TxQueuedCount[1],
                           (unsigned long)RobotCan_TxRejectedCount[1],
                           (unsigned long)RobotCan_LastTxId[1]);
        } else {
            /* 50 Hz 紧凑帧：网页实时显示需要的姿态、遥控器和云台字段。 */
            length = snprintf(format_buffer, sizeof(format_buffer),
                              "temp:%.2f,mcu_temp:%.2f,mcu_temp_ok:%u,ready:%u,bias_count:%u,"
                              "raw_gx:%.4f,raw_gy:%.4f,raw_gz:%.4f,"
                              "gyro_x:%.4f,gyro_y:%.4f,gyro_z:%.4f,"
                              "bias_x:%.4f,bias_y:%.4f,bias_z:%.4f,"
                              "yaw:%.3f,pitch:%.3f,roll:%.3f,yaw_rate:%.3f,"
                              "rc_lost:%u,ch0:%d,ch1:%d,ch2:%d,ch3:%d,sw0:%u,sw1:%u,"
                              "gimbal_mode:%u,armed:%u,yaw_online:%u,pitch_online:%u,"
                              "yaw_target:%.3f,pitch_target:%.3f,yaw_err:%.3f,pitch_err:%.3f,"
                              "trigger_online:%u,trigger_mode:%u,trigger_feedback:%.1f,trigger_target:%.1f,"
                              "chassis_mode:%u,chassis_vx:%.2f,chassis_vy:%.2f,chassis_wz:%.2f",
                              BMI088_Info.Temperature, MCU_Temperature_C,
                              (unsigned int)MCU_TemperatureValid,
                              (unsigned int)INS_ImuReady,
                              (unsigned int)INS_GyroBiasSampleCount,
                              INS_GyroRaw[0], INS_GyroRaw[1], INS_GyroRaw[2],
                              INS_Info.Gyro[0], INS_Info.Gyro[1], INS_Info.Gyro[2],
                              INS_GyroBias[0], INS_GyroBias[1], INS_GyroBias[2],
                              INS_Info.Yaw_Angle, INS_Info.Pitch_Angle,
                              INS_Info.Roll_Angle, INS_Info.Yaw_Gyro,
                              (unsigned int)remote_ctrl.rc_lost,
                              (int)remote_ctrl.rc.ch[0], (int)remote_ctrl.rc.ch[1],
                              (int)remote_ctrl.rc.ch[2], (int)remote_ctrl.rc.ch[3],
                              (unsigned int)remote_ctrl.rc.s[0],
                              (unsigned int)remote_ctrl.rc.s[1],
                              (unsigned int)gimbal.mode, (unsigned int)robot_command_armed,
                              (unsigned int)gimbal.yaw.online,
                              (unsigned int)gimbal.pitch.online,
                              gimbal_imu_yaw_target_deg, gimbal_imu_pitch_target_deg,
                              gimbal_imu_yaw_error_deg, gimbal_imu_pitch_error_deg,
                              (unsigned int)shoot_fric.trigger_online,
                              (unsigned int)shoot_fric.trigger_mode,
                              shoot_fric.trigger_feedback_velocity,
                              shoot_fric.trigger_target_velocity,
                              (unsigned int)chassis.mode,
                              chassis.reference.vx, chassis.reference.vy,
                              chassis.reference.wz);
        }
#elif (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
        length = snprintf(format_buffer, sizeof(format_buffer),
                          "temp:%.2f,mcu_temp:%.2f,ready:%u,"
                          "yaw:%.3f,pitch:%.3f,roll:%.3f,yaw_rate:%.4f,"
                          "rc_lost:%u,ch0:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d,sw0:%u,sw1:%u,"
                          "sbus_frames:%lu,sbus_errors:%lu,"
                          "gimbal_mode:%u,armed:%u,"
                          "big_yaw_online:%u,small_yaw_online:%u,pitch_online:%u,"
                          "big_yaw_state:%u,small_yaw_state:%u,pitch_state:%u,"
                          "big_yaw_fb_pos:%.5f,small_yaw_fb_pos:%.5f,pitch_fb_pos:%.5f,"
                          "big_yaw_target:%.3f,big_yaw_follow_target:%.3f,big_yaw_err:%.3f,"
                          "small_yaw_ref:%.5f,pitch_ref:%.5f,big_yaw_cmd:%.5f,"
                          "small_yaw_sp:%.5f,pitch_sp:%.5f,"
                          "extra_imu_enabled:%u,extra_imu_online:%u,extra_imu_can:%u,extra_imu_id:%lu,extra_imu_master_id:%lu,extra_imu_rx_id:%lu,"
                          "extra_imu_age:%lu,extra_imu_frames:%lu,extra_imu_invalid:%lu,"
                          "extra_imu_mask:%u,extra_imu_type:%u,extra_imu_temp:%.2f,"
                          "extra_imu_pitch:%.3f,extra_imu_yaw:%.3f,extra_imu_roll:%.3f,"
                          "extra_imu_gx:%.4f,extra_imu_gy:%.4f,extra_imu_gz:%.4f,"
                          "extra_imu_ax:%.3f,extra_imu_ay:%.3f,extra_imu_az:%.3f,"
                          "extra_imu_qw:%.5f,extra_imu_qx:%.5f,extra_imu_qy:%.5f,extra_imu_qz:%.5f,"
                          "extra_imu_raw0:%u,extra_imu_raw1:%u,extra_imu_raw2:%u,extra_imu_raw3:%u,"
                          "extra_imu_raw4:%u,extra_imu_raw5:%u,extra_imu_raw6:%u,extra_imu_raw7:%u",
                          BMI088_Info.Temperature, MCU_Temperature_C,
                          (unsigned int)INS_ImuReady,
                          INS_Info.Yaw_Angle, INS_Info.Pitch_Angle, INS_Info.Roll_Angle,
                          INS_Info.Yaw_Gyro,
                          (unsigned int)remote_ctrl.rc_lost,
                           (int)remote_ctrl.rc.ch[0], (int)remote_ctrl.rc.ch[1],
                           (int)remote_ctrl.rc.ch[2], (int)remote_ctrl.rc.ch[3],
                          (int)remote_ctrl.rc.ch[4],
                          (unsigned int)remote_ctrl.rc.s[0], (unsigned int)remote_ctrl.rc.s[1],
                          (unsigned long)remote_sbus_frame_count,
                          (unsigned long)remote_sbus_error_count,
                          (unsigned int)gimbal_three_axis.mode,
                          (unsigned int)robot_command_armed,
                          (unsigned int)gimbal_three_axis.big_yaw.online,
                          (unsigned int)gimbal_three_axis.small_yaw.online,
                          (unsigned int)gimbal_three_axis.pitch.online,
                          (unsigned int)gimbal_three_axis.big_yaw.feedback.state,
                          (unsigned int)gimbal_three_axis.small_yaw.feedback.state,
                          (unsigned int)gimbal_three_axis.pitch.feedback.state,
                          gimbal_three_axis.big_yaw.feedback.position,
                          gimbal_three_axis.small_yaw.feedback.position,
                          gimbal_three_axis.pitch.feedback.position,
                          gimbal_three_big_yaw_target_deg,
                          gimbal_three_big_yaw_follow_target_deg,
                          gimbal_three_big_yaw_error_deg,
                          gimbal_three_small_yaw_reference,
                          gimbal_three_pitch_reference,
                          gimbal_three_big_yaw_position_cmd,
                          gimbal_three_axis.small_yaw.setpoint.position,
                          gimbal_three_axis.pitch.setpoint.position,
                          (unsigned int)(EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1),
                          (unsigned int)ExtraImu_IsOnline(),
                          (unsigned int)EXTRA_IMU_CAN,
                          (unsigned long)EXTRA_IMU_CAN_ID,
                          (unsigned long)EXTRA_IMU_MASTER_ID,
                          (unsigned long)extra_imu.last_identifier,
                          (unsigned long)(HAL_GetTick() - extra_imu.last_update_ms),
                          (unsigned long)extra_imu.valid_frames,
                          (unsigned long)extra_imu.invalid_frames,
                          (unsigned int)extra_imu.frame_mask,
                          (unsigned int)extra_imu.last_data_type,
                          extra_imu.temperature_c,
                          extra_imu.pitch_deg, extra_imu.yaw_deg, extra_imu.roll_deg,
                          extra_imu.gyro_rad_s[0], extra_imu.gyro_rad_s[1], extra_imu.gyro_rad_s[2],
                          extra_imu.acceleration_mps2[0], extra_imu.acceleration_mps2[1],
                          extra_imu.acceleration_mps2[2],
                          extra_imu.quaternion[0], extra_imu.quaternion[1],
                          extra_imu.quaternion[2], extra_imu.quaternion[3],
                          (unsigned int)extra_imu.last_data[0], (unsigned int)extra_imu.last_data[1],
                          (unsigned int)extra_imu.last_data[2], (unsigned int)extra_imu.last_data[3],
                          (unsigned int)extra_imu.last_data[4], (unsigned int)extra_imu.last_data[5],
                          (unsigned int)extra_imu.last_data[6], (unsigned int)extra_imu.last_data[7]);
#elif (CHASSIS_TYPE == CHASSIS_EVO4310_TEST)
        length = snprintf(format_buffer, sizeof(format_buffer),
                          "temp:%.2f,mcu_temp:%.2f,ready:%u,"
                          "yaw:%.3f,pitch:%.3f,roll:%.3f,yaw_rate:%.4f,"
                          "rc_lost:%u,ch0:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d,"
                          "sw0:%u,sw1:%u,sbus_frames:%lu,sbus_errors:%lu,"
                          "armed:%u,chassis_mode:%u,chassis_vx:%.3f,chassis_vy:%.3f,chassis_wz:%.3f,"
                          "evo_pos_cmd:%.4f,evo_vel_cmd:%.4f,evo_kp:%.3f,evo_kd:%.3f,evo_torque_cmd:%.4f,"
                          "chassis_i1:%.1f,chassis_i2:%.1f,chassis_i3:%.1f,chassis_i4:%.1f,"
                          "can3_rx:%lu,can3_last_id:%lu,can3_tx:%lu,can3_tx_reject:%lu,can3_tx_id:%lu,"
                          "can3_tec:%lu,can3_rec:%lu,can3_lec:%lu,can3_activity:%lu,"
                          "can3_ep:%u,can3_warning:%u,can3_busoff:%u",
                          BMI088_Info.Temperature, MCU_Temperature_C,
                          (unsigned int)INS_ImuReady,
                          INS_Info.Yaw_Angle, INS_Info.Pitch_Angle, INS_Info.Roll_Angle,
                          INS_Info.Yaw_Gyro,
                          (unsigned int)remote_ctrl.rc_lost,
                          (int)remote_ctrl.rc.ch[0], (int)remote_ctrl.rc.ch[1],
                          (int)remote_ctrl.rc.ch[2], (int)remote_ctrl.rc.ch[3],
                          (int)remote_ctrl.rc.ch[4],
                          (unsigned int)remote_ctrl.rc.s[0],
                          (unsigned int)remote_ctrl.rc.s[1],
                          (unsigned long)remote_sbus_frame_count,
                          (unsigned long)remote_sbus_error_count,
                          (unsigned int)robot_command_armed, (unsigned int)chassis.mode,
                          chassis.reference.vx, chassis.reference.vy, chassis.reference.wz,
                          chassis.motor[0].setpoint.position,
                          chassis.motor[0].setpoint.velocity,
                          EVO4310_TEST_MIT_KP, EVO4310_TEST_MIT_KD,
                          chassis.motor[0].setpoint.torque,
                          chassis.motor[0].setpoint.torque, 0.0f, 0.0f, 0.0f,
                          (unsigned long)RobotCan_RxCount[3],
                          (unsigned long)RobotCan_LastRxId[3],
                          (unsigned long)RobotCan_TxQueuedCount[3],
                          (unsigned long)RobotCan_TxRejectedCount[3],
                          (unsigned long)RobotCan_LastTxId[3],
                          (unsigned long)RobotCan_TxErrorCount[3],
                          (unsigned long)RobotCan_RxErrorCount[3],
                          (unsigned long)RobotCan_LastErrorCode[3],
                          (unsigned long)RobotCan_Activity[3],
                          (unsigned int)RobotCan_ErrorPassive[3],
                          (unsigned int)RobotCan_Warning[3],
                          (unsigned int)RobotCan_BusOff[3]);
#else
        length = snprintf(format_buffer, sizeof(format_buffer),
                          "temp:%.2f,mcu_temp:%.2f,ready:%u",
                          BMI088_Info.Temperature, MCU_Temperature_C,
                          (unsigned int)INS_ImuReady);
#endif

        /* SLAM 遥测独立于车型与云台选型：仅显示定位数据，不会改变任何控制目标。 */
        if (length > 0 && (size_t)length < sizeof(format_buffer)) {
            length += snprintf(&format_buffer[length], sizeof(format_buffer) - (size_t)length,
                               ",slam_online:%u,slam_x:%.3f,slam_y:%.3f,slam_z:%.3f,"
                               "slam_roll:%.2f,slam_pitch:%.2f,slam_yaw:%.2f,slam_age:%lu,"
                               "slam_frames:%lu,slam_errors:%lu,slam_uart_errors:%lu,"
                               "slam_rx_bytes:%lu,slam_overflow:%lu,slam_raw_len:%u,"
                               "slam_raw0:%u,slam_raw1:%u,slam_raw2:%u,slam_raw3:%u,slam_raw4:%u,"
                               "slam_raw5:%u,slam_raw6:%u,slam_raw7:%u,slam_raw8:%u,slam_raw9:%u,"
                               "slam_raw10:%u,slam_raw11:%u,slam_raw12:%u,slam_raw13:%u,slam_raw14:%u",
                               (unsigned int)slam_pose.online,
                               slam_pose.x_m, slam_pose.y_m, slam_pose.z_m,
                               slam_pose.roll_deg, slam_pose.pitch_deg, slam_pose.yaw_deg,
                               (unsigned long)(HAL_GetTick() - slam_pose.last_update_ms),
                               (unsigned long)slam_pose.valid_frames,
                               (unsigned long)slam_pose.format_errors,
                               (unsigned long)slam_pose.uart_errors,
                               (unsigned long)slam_pose.rx_bytes,
                               (unsigned long)slam_pose.rx_overflow,
                               (unsigned int)slam_pose.raw_length,
                               (unsigned int)slam_pose.raw_frame[0], (unsigned int)slam_pose.raw_frame[1],
                               (unsigned int)slam_pose.raw_frame[2], (unsigned int)slam_pose.raw_frame[3],
                               (unsigned int)slam_pose.raw_frame[4], (unsigned int)slam_pose.raw_frame[5],
                               (unsigned int)slam_pose.raw_frame[6], (unsigned int)slam_pose.raw_frame[7],
                               (unsigned int)slam_pose.raw_frame[8], (unsigned int)slam_pose.raw_frame[9],
                               (unsigned int)slam_pose.raw_frame[10], (unsigned int)slam_pose.raw_frame[11],
                               (unsigned int)slam_pose.raw_frame[12], (unsigned int)slam_pose.raw_frame[13],
                               (unsigned int)slam_pose.raw_frame[14]);
        }

#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL) || (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
        /* 底盘诊断统一追加，保证二轴和三轴云台配置都能观察功率限流结果。 */
        if (length > 0 &&
            (size_t)length < sizeof(format_buffer)) {
            length += snprintf(&format_buffer[length], sizeof(format_buffer) - (size_t)length,
                               ",chassis_mode:%u,chassis_vx:%.3f,chassis_vy:%.3f,chassis_wz:%.3f,"
                               "chassis_i1:%.1f,chassis_i2:%.1f,chassis_i3:%.1f,chassis_i4:%.1f,"
                               "chassis_power_enabled:%u,chassis_power_ref:%.1f,"
                               "chassis_power_budget:%.2f,chassis_power_before:%.2f,chassis_power_after:%.2f,"
                               "chassis_power_limit1:%.2f,chassis_power_limit2:%.2f,"
                               "chassis_power_limit3:%.2f,chassis_power_limit4:%.2f,"
                               "chassis_power_k1:%.3f,chassis_power_k2:%.3f,"
                               "chassis_power_k3:%.3f,chassis_power_k4:%.3f,"
                               "can1_rx:%lu,can1_tx:%lu,can1_tx_reject:%lu,can1_tx_id:%lu,"
                                "can1_tec:%lu,can1_rec:%lu,can1_lec:%lu,can1_activity:%lu,"
                                "can1_ep:%u,can1_warning:%u,can1_busoff:%u,"
                               "can2_rx:%lu,can2_rx_id:%lu,can2_tx:%lu,can2_tx_reject:%lu,can2_tx_id:%lu,"
                               "can2_tec:%lu,can2_rec:%lu,can2_lec:%lu,can2_activity:%lu,"
                               "can2_ep:%u,can2_warning:%u,can2_busoff:%u",
                               (unsigned int)chassis.mode,
                               chassis.reference.vx, chassis.reference.vy, chassis.reference.wz,
                               chassis.motor[0].setpoint.current, chassis.motor[1].setpoint.current,
                               chassis.motor[2].setpoint.current, chassis.motor[3].setpoint.current,
                               (unsigned int)chassis.power_manager.enabled,
                               chassis.power_manager.requested_limit_w,
                               chassis.power_manager.budget_w,
                               chassis.power_manager.predicted_before_w,
                               chassis.power_manager.predicted_after_w,
                               chassis.power_manager.motor_limit_w[0],
                               chassis.power_manager.motor_limit_w[1],
                               chassis.power_manager.motor_limit_w[2],
                               chassis.power_manager.motor_limit_w[3],
                               chassis.power_manager.attenuation[0],
                               chassis.power_manager.attenuation[1],
                               chassis.power_manager.attenuation[2],
                               chassis.power_manager.attenuation[3],
                               (unsigned long)RobotCan_RxCount[1],
                               (unsigned long)RobotCan_TxQueuedCount[1],
                               (unsigned long)RobotCan_TxRejectedCount[1],
                               (unsigned long)RobotCan_LastTxId[1],
                                (unsigned long)RobotCan_TxErrorCount[1],
                                (unsigned long)RobotCan_RxErrorCount[1],
                                (unsigned long)RobotCan_LastErrorCode[1],
                                (unsigned long)RobotCan_Activity[1],
                                (unsigned int)RobotCan_ErrorPassive[1],
                                (unsigned int)RobotCan_Warning[1],
                                (unsigned int)RobotCan_BusOff[1],
                               (unsigned long)RobotCan_RxCount[2],
                               (unsigned long)RobotCan_LastRxId[2],
                               (unsigned long)RobotCan_TxQueuedCount[2],
                               (unsigned long)RobotCan_TxRejectedCount[2],
                               (unsigned long)RobotCan_LastTxId[2],
                               (unsigned long)RobotCan_TxErrorCount[2],
                               (unsigned long)RobotCan_RxErrorCount[2],
                               (unsigned long)RobotCan_LastErrorCode[2],
                               (unsigned long)RobotCan_Activity[2],
                               (unsigned int)RobotCan_ErrorPassive[2],
                               (unsigned int)RobotCan_Warning[2],
                               (unsigned int)RobotCan_BusOff[2]);
        }
#endif

        if (length > 0 && (size_t)length < sizeof(format_buffer)) {
            length += snprintf(&format_buffer[length], sizeof(format_buffer) - (size_t)length,
                               "\r\n");
        }

        if ((length > 0) && ((uint32_t)length < sizeof(tx_buffer))) {
            /* 基础遥测和电机表分开发送，避免电机字段把综合帧撑爆。 */
            if (TelemetryTransmit(tx_buffer, format_buffer, (uint16_t)length) != 0U) {
                motor_frame_pending = 1U;
            }
        } else {
            Telemetry_DroppedFrameCount++;
        }

        }

        /* 基础遥测帧完成后，再发送电机表，避免争用 USB CDC 端点。 */
        if (motor_frame_pending != 0U &&
            (HAL_GetTick() - telemetry_last_send_ms) >= 2U &&
            (HAL_GetTick() - telemetry_last_send_ms) < TELEMETRY_PERIOD_MS) {
            length = 0;
            TelemetryAppendMotorFeedback(format_buffer, sizeof(format_buffer), &length);
            if (length > 0 && (size_t)length < sizeof(format_buffer)) {
                length += snprintf(&format_buffer[length], sizeof(format_buffer) - (size_t)length,
                                   "\r\n");
                if ((uint32_t)length < sizeof(tx_buffer) &&
                    TelemetryTransmit(tx_buffer, format_buffer, (uint16_t)length) != 0U) {
                    motor_frame_pending = 0U;
                }
            }
        }

        /* 任务以 10 ms 唤醒，用于输出独立的 SLAM 高速调试帧。 */
        osDelay(SLAM_DEBUG_PERIOD_MS);
    }
}
