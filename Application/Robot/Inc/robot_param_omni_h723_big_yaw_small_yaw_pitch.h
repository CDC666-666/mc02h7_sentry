#ifndef ROBOT_PARAM_OMNI_H723_BIG_YAW_SMALL_YAW_PITCH_H
#define ROBOT_PARAM_OMNI_H723_BIG_YAW_SMALL_YAW_PITCH_H

/* 复用 H723 全向轮、IMU、遥控器和发射机构配置，只替换云台实现。 */
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL
#define GIMBAL_TYPE GIMBAL_BIG_YAW_SMALL_YAW_PITCH
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER

/*
 * 顶部额外 IMU 选择：
 * EXTRA_IMU_NONE      仅使用现有大 Yaw 上的 BMI088；
 * EXTRA_IMU_DM_IMU_L1 使用顶部 DM-IMU-L1（后续驱动和任务按此宏编译）。
 * 默认关闭，关闭时不会改变现有 CAN、任务和电机控制链路。
 */
/* CAN3 台架测试：DM-IMU-L1 默认使用经典 CAN 1 Mbps、标准帧 ID 0x30。 */
#define EXTRA_IMU_TYPE EXTRA_IMU_NONE
/* 小 Yaw 外挂 IMU 作为实际指向反馈；无有效数据时回退到编码器。 */
#define EXTRA_IMU_CONTROL_ENABLE 0U
#define EXTRA_IMU_CAN 3U
#define EXTRA_IMU_CAN_ID 0x30U
#define EXTRA_IMU_MASTER_ID 0x31U
#define EXTRA_IMU_TIMEOUT_MS 200U
/* 仿照官方 MC02_CAN 收发例程：仅在没有新 IMU 回包时发送寄存器读请求。
 * 请求帧为 CAN_ID=0x30、CC/RID/00/DD；绝不写模块寄存器。 */
#define EXTRA_IMU_REQUEST_MODE_ENABLE 0U
#define EXTRA_IMU_REQUEST_PERIOD_MS 10U

/* 外置 IMU 安装在小 yaw 头部，Z 轴陀螺正方向按实际安装确认。 */
#define EXTRA_IMU_YAW_DIRECTION             (1.0f)
#define EXTRA_IMU_SMALL_YAW_OUTER_KP        (0.08f)
#define EXTRA_IMU_SMALL_YAW_OUTER_MAX_RAD   (0.12f)

/* 温度-静置 Z 轴陀螺零偏表，单位分别为摄氏度和 rad/s。 */
#define EXTRA_IMU_YAW_TEMP_BIAS_POINT_COUNT 3U
#define EXTRA_IMU_YAW_TEMP_BIAS_TABLE \
    {25.0f, 0.0f},                    \
    {35.0f, 0.0f},                    \
    {45.0f, 0.0f}

/* Type-C 遥测输出选择。
 * 额外 IMU 调试模式只发送紧凑的 DM-IMU-L1 帧（100 Hz），避免完整机器人
 * 遥测占满 USB 带宽；它只影响网页数据流，不影响 CAN、电机或云台控制。
 * 需要回到综合调试页时，把 USB_TELEMETRY_STREAM_MODE 改为
 * USB_TELEMETRY_STREAM_FULL。 */
#define USB_TELEMETRY_STREAM_FULL             0U
#define USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG  1U
#define USB_TELEMETRY_STREAM_MODE             USB_TELEMETRY_STREAM_FULL
#define EXTRA_IMU_DEBUG_PERIOD_MS             10U

/* 三轴云台 Type-C 接口输出综合 CSV 遥测，供上位机网页调试。 */
#ifndef USB_CDC_MODE
#define USB_CDC_MODE USB_CDC_MODE_TELEMETRY
#endif
#include "robot_param_omni_h723_gimbal.h"

/* 底盘功率控制：只限制 CAN1 上四个 M3508 的电流指令，不影响 CAN2 云台。 */
#define CHASSIS_POWER_LIMITER_ENABLE       1U
/* 当前未接入裁判系统功率数据，固定使用下方回退功率上限。 */
#define CHASSIS_POWER_USE_REFEREE_LIMIT    0U
#define CHASSIS_POWER_FALLBACK_LIMIT_W     (100.0f)
#define CHASSIS_POWER_BUFFER_ATTENUATION   (1.0f)
#define CHASSIS_POWER_COMPENSATION         (0.02f)
#define CHASSIS_POWER_ERROR_EQUAL_RPM      (100.0f)
#define CHASSIS_POWER_RESERVED_BORDER_W    (50.0f)
#define CHASSIS_POWER_RESERVED_PER_MOTOR_W (5.0f)

/* M3508 初始功率模型，来自 PowerManager 示例；实车采样后应替换为本车拟合值。 */
#define CHASSIS_M3508_POWER_K0                 (0.65213f)
#define CHASSIS_M3508_POWER_K1                 (-0.15659f)
#define CHASSIS_M3508_POWER_K2                 (0.00041660f)
#define CHASSIS_M3508_POWER_K3                 (0.00235415f)
#define CHASSIS_M3508_POWER_K4                 (0.20022f)
#define CHASSIS_M3508_POWER_K5                 (0.000000108f)
#define CHASSIS_M3508_POWER_CURRENT_CONVERSION (1000.0f)

/*
 * 三电机云台：控制板安装在大 Yaw 上。
 * 大 Yaw 使用 BMI088 Yaw 姿态闭环；小 Yaw 和 Pitch 仅以自身 DM4310 编码器反馈闭环。
 * 三轴框架已在 robot_param.h 中启用；CAN 与 ID 暂保持为 0，
 * 在实际接线与电机 ID 确认前不会向三台云台电机发送命令。
 */
#define GIMBAL_BIG_YAW_MOTOR_TYPE   DM_4310
#define GIMBAL_SMALL_YAW_MOTOR_TYPE DJI_M6020
#define GIMBAL_THREE_PITCH_MOTOR_TYPE DM_4310

#define GIMBAL_BIG_YAW_CAN            1U
#define GIMBAL_SMALL_YAW_CAN          2U
#define GIMBAL_THREE_PITCH_CAN        2U
#define GIMBAL_BIG_YAW_ID             0x01U
#define GIMBAL_SMALL_YAW_ID           1U
#define GIMBAL_THREE_PITCH_ID         0x02U
#define GIMBAL_BIG_YAW_MASTER_ID      0x51U
#define GIMBAL_SMALL_YAW_MASTER_ID    0U
#define GIMBAL_THREE_PITCH_MASTER_ID  0x52U

#define GIMBAL_BIG_YAW_DIRECTION    (1)
#define GIMBAL_SMALL_YAW_DIRECTION  (1)
#define GIMBAL_THREE_PITCH_DIRECTION (1)
#define GIMBAL_BIG_YAW_REDUCTION_RATIO    (1.0f)
#define GIMBAL_SMALL_YAW_REDUCTION_RATIO  (1.0f)
#define GIMBAL_THREE_PITCH_REDUCTION_RATIO (1.0f)
#define GIMBAL_BIG_YAW_MODE         DM_MODE_MIT
#define GIMBAL_SMALL_YAW_MODE       DJI_VOLTAGE_MODE
#define GIMBAL_THREE_PITCH_MODE     DM_MODE_MIT

/* 2026-10-14 云台中值位置，单位 rad。 */
#define GIMBAL_BIG_YAW_INITIAL_POSITION_RAD     (0.3172f)
#define GIMBAL_SMALL_YAW_INITIAL_POSITION_RAD   (3.2198f)
#define GIMBAL_THREE_PITCH_INITIAL_POSITION_RAD (0.4217f)

/* 小 Yaw/Pitch 编码器参考的机械安全范围，均相对各自上电捕获的位置。 */
#define GIMBAL_SMALL_YAW_LOWER_LIMIT_RAD  (-1.5707963f)
#define GIMBAL_SMALL_YAW_UPPER_LIMIT_RAD  (1.5707963f)
#define GIMBAL_THREE_PITCH_LOWER_LIMIT_RAD (-0.5f)
#define GIMBAL_THREE_PITCH_UPPER_LIMIT_RAD (0.5f)

/* Ch0 控制小 Yaw，Ch1 控制 Pitch；单位是编码器参考位置的弧度每秒。 */
#define GIMBAL_THREE_RC_YAW_CHANNEL       0U
#define GIMBAL_THREE_RC_PITCH_CHANNEL     1U
#define GIMBAL_THREE_RC_DEADBAND           (10.0f)
#define GIMBAL_THREE_RC_YAW_INPUT_DIRECTION (1.0f)
#define GIMBAL_THREE_RC_PITCH_INPUT_DIRECTION (1.0f)
#define GIMBAL_THREE_RC_YAW_RATE_RAD_S     (3.0f)
#define GIMBAL_THREE_RC_PITCH_RATE_RAD_S   (1.5f)
#define GIMBAL_THREE_CONTROL_PERIOD_S      (0.001f)

/* 大 Yaw 同向跟随小 Yaw 偏移，用于验证两轴联动方向。 */
#define GIMBAL_BIG_YAW_FOLLOW_DIRECTION    (1.0f)
#define GIMBAL_BIG_YAW_FOLLOW_RATIO        (1.0f)
#define GIMBAL_BIG_YAW_FOLLOW_MAX_RATE_DEG_S (180.0f)
#define GIMBAL_BIG_YAW_IMU_OUTER_KP        (0.000f)
#define GIMBAL_BIG_YAW_IMU_MAX_POSITION_OFFSET (0.500f)
#define GIMBAL_BIG_YAW_IMU_VELOCITY_KP     (0.080f)
#define GIMBAL_BIG_YAW_IMU_VELOCITY_MAX   (15.0f)
#define GIMBAL_BIG_YAW_IMU_VELOCITY_DIRECTION (-1.0f)

/* 三台 DM4310 共用 MIT 参数，可在实际调试后独立拆分。 */
#define GIMBAL_THREE_DM4310_MIT_KP         (12.0f)
#define GIMBAL_THREE_DM4310_MIT_KD         (0.10f)
#define GIMBAL_THREE_DM4310_MIT_TORQUE     (0.0f)
/* 大 yaw 需要额外的 3 Nm 前馈；Pitch 仍保持零扭矩前馈。 */
#define GIMBAL_BIG_YAW_MIT_TORQUE          (3.0f)

/* 小 Yaw GM6020：编码器位置环 -> 速度环 -> 电压命令。初始值偏保守，
 * 完成单轴正反方向确认后再按实际负载调节。 */
#define GIMBAL_SMALL_YAW_6020_ANGLE_KP       (8.0f)
#define GIMBAL_SMALL_YAW_6020_ANGLE_KI       (0.0f)
#define GIMBAL_SMALL_YAW_6020_ANGLE_KD       (0.0f)
#define GIMBAL_SMALL_YAW_6020_ANGLE_MAX_OUT  (18.0f)
#define GIMBAL_SMALL_YAW_6020_ANGLE_MAX_IOUT (0.0f)
#define GIMBAL_SMALL_YAW_6020_SPEED_KP       (1800.0f)
#define GIMBAL_SMALL_YAW_6020_SPEED_KI       (0.0f)
#define GIMBAL_SMALL_YAW_6020_SPEED_KD       (50.0f)
#define GIMBAL_SMALL_YAW_6020_SPEED_MAX_OUT  (16000.0f)
#define GIMBAL_SMALL_YAW_6020_SPEED_MAX_IOUT (2000.0f)

#endif /* ROBOT_PARAM_OMNI_H723_BIG_YAW_SMALL_YAW_PITCH_H */
