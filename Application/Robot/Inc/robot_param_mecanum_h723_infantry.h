#ifndef ROBOT_PARAM_MECANUM_H723_INFANTRY_H
#define ROBOT_PARAM_MECANUM_H723_INFANTRY_H

#include "robot_typedef.h"

/* 麦轮步兵：复用 H723 板级、IMU、ET08A 和两轴云台默认参数，
 * 仅覆盖本车的 CAN 电机映射与麦轮运动学参数。 */
#define CHASSIS_TYPE CHASSIS_MECANUM_WHEEL
#include "robot_param_omni_h723_gimbal.h"

/* DM-IMU-L1 台架测试：仅监听并请求 CAN3 数据，不参与 BMI088、云台或底盘闭环。
 * 模块请求地址为 0x30，主动发送/应答地址为 0x31，经典 CAN 速率为 1 Mbps。 */
#define EXTRA_IMU_TYPE                   EXTRA_IMU_DM_IMU_L1
#define EXTRA_IMU_CONTROL_ENABLE        0U
#define EXTRA_IMU_CAN                   3U
#define EXTRA_IMU_CAN_ID                0x30U
#define EXTRA_IMU_MASTER_ID             0x31U
#define EXTRA_IMU_TIMEOUT_MS            200U
#define EXTRA_IMU_REQUEST_MODE_ENABLE   1U
#define EXTRA_IMU_REQUEST_PERIOD_MS     10U
#define USB_TELEMETRY_STREAM_MODE       USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG
#define EXTRA_IMU_DEBUG_PERIOD_MS       10U

/* 麦轮步兵安装时控制板相对车体绕 Z 轴旋转 +90°。
 * 若实测 Yaw/横向方向相反，只需将矩阵改为 -90°：
 * {0, 1, 0}, {-1, 0, 0}, {0, 0, 1}。 */
#undef __BOARD_INSTALL_SPIN_MATRIX
#define __BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, -1.0f, 0.0f},            \
    {1.0f,  0.0f, 0.0f},            \
    {0.0f,  0.0f, 1.0f}

/* 固定零偏必须和上面的安装矩阵处于同一个车体坐标系。
 * 基础配置中的零偏来自 180 度安装；麦轮板是 90 度安装，直接复用会把
 * X/Y 零偏方向带错，导致 Yaw 闭环持续修正。 */
#undef __IMU_GYRO_FIXED_BIAS_X
#undef __IMU_GYRO_FIXED_BIAS_Y
#undef __IMU_GYRO_FIXED_BIAS_Z
#define __IMU_GYRO_FIXED_BIAS_X     (0.003609f)
#define __IMU_GYRO_FIXED_BIAS_Y     (-0.003853f)
#define __IMU_GYRO_FIXED_BIAS_Z     (-0.003485f)

/* 麦轮步兵总线分配：
 * CAN1：四个底盘 M3508（ID1~4）和 Yaw GM6020（ID1）；
 * CAN2：Pitch GM6020（ID1）和两个摩擦轮 M3508（ID1、ID2）。
 *
 * DJI 的 M3508/M2006 与 GM6020 即使数字 ID 相同也可共线：M3508/M2006
 * ID1~4 使用 0x200 控制帧、0x201~0x204 反馈；GM6020 ID1~4 使用
 * 0x1FF 控制帧、0x205~0x208 反馈。两类电机的 CAN 帧不会冲突。 */

#undef WHEEL_1_DIRECTION
#undef WHEEL_2_DIRECTION
#undef WHEEL_3_DIRECTION
#undef WHEEL_4_DIRECTION
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (1)
#define WHEEL_4_DIRECTION (1)

#define MECANUM_WHEEL_RADIUS             (0.080f)
#define MECANUM_WHEELBASE_PLUS_TRACK     (0.295f)

/* 麦轮摇杆平移速度映射：ch2 横移、ch3 前后，满杆目标速度提高到 4 m/s。
 * 最终实际速度仍受下方 100 W 底盘功率限制和 M3508 最高转速约束。 */
#undef CHASSIS_RC_MAX_SPEED
#define CHASSIS_RC_MAX_SPEED             (4.0f)

/* 麦轮底盘总功率限制，四个 M3508 共用整车 100 W 预算。 */
#define CHASSIS_POWER_LIMITER_ENABLE       1U
#define CHASSIS_POWER_USE_REFEREE_LIMIT    0U
#define CHASSIS_POWER_FALLBACK_LIMIT_W     (100.0f)
#undef CHASSIS_SPIN_POWER_LIMIT_W
#define CHASSIS_SPIN_POWER_LIMIT_W         (100.0f)
#define CHASSIS_POWER_BUFFER_ATTENUATION   (1.0f)
#define CHASSIS_POWER_COMPENSATION         (0.02f)
#define CHASSIS_POWER_ERROR_EQUAL_RPM      (100.0f)
#define CHASSIS_POWER_RESERVED_BORDER_W    (50.0f)
#define CHASSIS_POWER_RESERVED_PER_MOTOR_W (5.0f)
#define CHASSIS_M3508_POWER_K0                 (0.65213f)
#define CHASSIS_M3508_POWER_K1                 (-0.15659f)
#define CHASSIS_M3508_POWER_K2                 (0.00041660f)
#define CHASSIS_M3508_POWER_K3                 (0.00235415f)
#define CHASSIS_M3508_POWER_K4                 (0.20022f)
#define CHASSIS_M3508_POWER_K5                 (0.000000108f)
#define CHASSIS_M3508_POWER_CURRENT_CONVERSION (1000.0f)

#undef GIMBAL_DIRECT_YAW_MOTOR_PROFILE
#undef GIMBAL_DIRECT_PITCH_MOTOR_PROFILE
#undef GIMBAL_DIRECT_YAW_ID
#undef GIMBAL_DIRECT_PITCH_ID
#undef GIMBAL_DIRECT_YAW_CAN
#undef GIMBAL_DIRECT_PITCH_CAN
#undef GIMBAL_DIRECT_YAW_MOTOR_TYPE
#undef GIMBAL_DIRECT_PITCH_MOTOR_TYPE
#undef GIMBAL_DIRECT_YAW_MODE
#undef GIMBAL_DIRECT_PITCH_MODE
#undef GIMBAL_DIRECT_YAW_MASTER_ID
#undef GIMBAL_DIRECT_PITCH_MASTER_ID
#define GIMBAL_DIRECT_YAW_MOTOR_PROFILE   GIMBAL_MOTOR_PROFILE_GM6020
#define GIMBAL_DIRECT_PITCH_MOTOR_PROFILE GIMBAL_MOTOR_PROFILE_GM6020
#define GIMBAL_DIRECT_YAW_ID              1U
#define GIMBAL_DIRECT_PITCH_ID            1U
#define GIMBAL_DIRECT_YAW_CAN             1U
#define GIMBAL_DIRECT_PITCH_CAN           2U
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE      DJI_M6020
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE    DJI_M6020
#define GIMBAL_DIRECT_YAW_MODE            DJI_VOLTAGE_MODE
#define GIMBAL_DIRECT_PITCH_MODE          DJI_VOLTAGE_MODE
#define GIMBAL_DIRECT_YAW_MASTER_ID       0U
#define GIMBAL_DIRECT_PITCH_MASTER_ID     0U

/* Yaw GM6020 的实际正方向与当前车体坐标相反；方向符号同时作用于
 * 电流命令和编码器反馈，必须成对翻转，否则速度环会形成正反馈。 */
#undef GIMBAL_DIRECT_YAW_DIRECTION
#define GIMBAL_DIRECT_YAW_DIRECTION       (-1)

/* 保留遥控器和 IMU 外环的原始坐标约定；当前只修正 GM6020
 * 的电机/编码器方向。 */
#undef GIMBAL_RC_YAW_INPUT_DIRECTION
#define GIMBAL_RC_YAW_INPUT_DIRECTION     (-1.0f)
#undef GIMBAL_IMU_YAW_OUTPUT_DIRECTION
#define GIMBAL_IMU_YAW_OUTPUT_DIRECTION   (1.0f)

/* ET08A 摇杆回中存在少量量化/校准偏差；中点附近统一视为零输入，
 * 防止 Yaw 目标角度在松杆后继续缓慢积分。 */
#undef GIMBAL_RC_DEADBAND
#define GIMBAL_RC_DEADBAND                (40.0f)

/* 双 GM6020 云台首轮调试采用保守的无积分参数。
 * 板载 BMI088 随 Yaw 轴一起转动，先以较低刚度抑制静止保持抖动，
 * 确认方向和姿态反馈稳定后再逐步提高 KP。 */
#undef KP_GIMBAL_YAW_ANGLE
#undef KI_GIMBAL_YAW_ANGLE
#undef KD_GIMBAL_YAW_ANGLE
#undef MAX_IOUT_GIMBAL_YAW_ANGLE
#undef MAX_OUT_GIMBAL_YAW_ANGLE
#undef KP_GIMBAL_YAW_VELOCITY
#undef KI_GIMBAL_YAW_VELOCITY
#undef KD_GIMBAL_YAW_VELOCITY
#undef MAX_IOUT_GIMBAL_YAW_VELOCITY
#undef MAX_OUT_GIMBAL_YAW_VELOCITY
#define KP_GIMBAL_YAW_ANGLE       (0.20f)
#define KI_GIMBAL_YAW_ANGLE       (0.0f)
#define KD_GIMBAL_YAW_ANGLE       (0.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_YAW_ANGLE  (45.0f)
#define KP_GIMBAL_YAW_VELOCITY       (100.0f)
#define KI_GIMBAL_YAW_VELOCITY       (0.0f)
#define KD_GIMBAL_YAW_VELOCITY       (0.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (0.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY  (4000.0f)

#undef MAX_OUT_GIMBAL_PITCH_ANGLE
#undef KP_GIMBAL_PITCH_VELOCITY
#undef MAX_OUT_GIMBAL_PITCH_VELOCITY
#define MAX_OUT_GIMBAL_PITCH_ANGLE    (45.0f)
#define KP_GIMBAL_PITCH_VELOCITY      (100.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (4000.0f)

#undef FRIC_MOTOR_R_TYPE
#undef FRIC_MOTOR_L_TYPE
#undef FRIC_MOTOR_R_ID
#undef FRIC_MOTOR_L_ID
#undef FRIC_MOTOR_R_CAN
#undef FRIC_MOTOR_L_CAN
#undef TRIGGER_MOTOR_TYPE
#undef TRIGGER_MOTOR_ID
#undef TRIGGER_MOTOR_CAN
#undef SHOOT_TRIGGER_ENABLED
#define FRIC_MOTOR_R_TYPE DJI_M3508
#define FRIC_MOTOR_L_TYPE DJI_M3508
#define FRIC_MOTOR_R_ID   1U
#define FRIC_MOTOR_L_ID   2U
#define FRIC_MOTOR_R_CAN  2U
#define FRIC_MOTOR_L_CAN  2U
/* 当前映射没有拨弹盘，明确关闭，避免向 CAN1 的不存在电机发送命令。 */
#define SHOOT_TRIGGER_ENABLED 0U
#define TRIGGER_MOTOR_TYPE MOTOR_NONE
#define TRIGGER_MOTOR_ID   0U
#define TRIGGER_MOTOR_CAN  0U

#endif /* ROBOT_PARAM_MECANUM_H723_INFANTRY_H */
