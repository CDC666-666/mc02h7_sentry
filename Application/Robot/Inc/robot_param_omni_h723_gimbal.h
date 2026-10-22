#ifndef ROBOT_PARAM_OMNI_H723_GIMBAL_H
#define ROBOT_PARAM_OMNI_H723_GIMBAL_H

#include "robot_typedef.h"

/* H723 全向轮底盘与可选 Yaw/Pitch 云台配置。 */

/*-------------------- 模块类型 --------------------*/
#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE        CHASSIS_OMNI_WHEEL
#endif
#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE         GIMBAL_YAW_PITCH_DIRECT
#endif
#ifndef SHOOT_TYPE
#define SHOOT_TYPE          SHOOT_FRIC_TRIGGER
#endif
#define CONTROL_TYPE        CHASSIS_AND_GIMBAL

/*-------------------- IMU --------------------*/
/* 控制板绕云台 Yaw（Z）轴旋转 180 度安装，故板载 X/Y 轴映射到机器人坐标系时符号相反。 */
#define __HEAT_IMU                 1U
#define __IMU_CONTROL_TEMPERATURE  (40.0f)
#define __IMU_TEMP_PID_KP          (1200.0f)
#define __IMU_TEMP_PID_KI          (20.0f)
#define __IMU_TEMP_PID_KD          (0.0f)
#define __IMU_TEMP_PID_MAX_OUT     (2000.0f)
#define __IMU_TEMP_PID_MAX_IOUT    (0.0f)
#define __IMU_TEMP_STABLE_BAND      (0.5f)
#define __IMU_TEMP_STABLE_SAMPLES   1000U
#define __IMU_GYRO_BIAS_SAMPLES     2000U
#define __IMU_GYRO_BIAS_MAX_RATE    (0.08f)
#define __IMU_GYRO_BIAS_ACCEL_TOL   (0.5f)
#define __IMU_ATTITUDE_WARMUP_SAMPLES 200U

/* 固定零偏来自 h7_imu_2026-08-20T15-04-46-740Z.csv：100 Hz 连续采样 539.4 秒，
 * 已应用本配置的控制板 180 度安装矩阵。 */
#define __IMU_GYRO_USE_FIXED_BIAS   1U
#define __IMU_GYRO_FIXED_BIAS_X     (0.003853f)
#define __IMU_GYRO_FIXED_BIAS_Y     (0.003609f)
#define __IMU_GYRO_FIXED_BIAS_Z     (-0.003485f)

#define __BOARD_INSTALL_SPIN_MATRIX \
    {-1.0f, 0.0f, 0.0f},            \
    {0.0f, -1.0f, 0.0f},            \
    {0.0f, 0.0f, 1.0f}

/* 未接线电机必须使用 ID 0 和 CAN 0，防止被意外发送控制命令。 */
#define MOTOR_ID_UNCONFIGURED 0U

/*-------------------- 底盘 --------------------*/
/* 机械参数 */
#define WHEEL_RADIUS          (0.080f)
#define WHEEL_CENTER_DISTANCE (0.295f)

/* DJI 底盘控制帧。当前 H7 发送器根据电机型号和模式选择帧，本宏用于明确预期配置。 */
#define CHASSIS_CAN   1U
#define CHASSIS_STDID 0x200U

/* 电机 ID */
#define WHEEL_1_ID 1U
#define WHEEL_2_ID 2U
#define WHEEL_3_ID 3U
#define WHEEL_4_ID 4U

/* 电机 CAN 总线 */
#define WHEEL_1_CAN 1U
#define WHEEL_2_CAN 1U
#define WHEEL_3_CAN 1U
#define WHEEL_4_CAN 1U

/* 电机型号 */
#define WHEEL_1_MOTOR_TYPE DJI_M3508
#define WHEEL_2_MOTOR_TYPE DJI_M3508
#define WHEEL_3_MOTOR_TYPE DJI_M3508
#define WHEEL_4_MOTOR_TYPE DJI_M3508

/* 电机方向 */
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (1)
#define WHEEL_4_DIRECTION (1)

/* 减速比 */
#define WHEEL_1_RATIO (19.0f)
#define WHEEL_2_RATIO (19.0f)
#define WHEEL_3_RATIO (19.0f)
#define WHEEL_4_RATIO (19.0f)

/* 电机命令模式 */
#define WHEEL_1_MODE DJI_CURRENT_MODE
#define WHEEL_2_MODE DJI_CURRENT_MODE
#define WHEEL_3_MODE DJI_CURRENT_MODE
#define WHEEL_4_MODE DJI_CURRENT_MODE

/* PID 参数：用于底盘速度闭环控制。 */
#define KP_OMNI_VEL       (20.0f)
#define KI_OMNI_VEL       (0.3f)
#define KD_OMNI_VEL       (0.3f)
#define MAX_IOUT_OMNI_VEL (10000.0f)
#define MAX_OUT_OMNI_VEL  (16000.0f)

#define KP_CHASSIS_FOLLOW_GIMBAL       (2.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL       (0.0f)
#define KD_CHASSIS_FOLLOW_GIMBAL       (0.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL  (2.0f)

/* 遥控器选型：只改此宏即可切换 UART5 帧格式、DMA 长度和解码器。
 * DT7/DR16 为 100 kbit/s、8E1、18 字节 DBUS；其他型号为
 * 100 kbit/s、8E2、25 字节 SBUS。MC-02 的 P6 已有反相电路。 */
#ifndef __RC_TYPE
#define __RC_TYPE RC_DT7
#endif
#define RC_TYPE __RC_TYPE

/* 遥控器参数。 */
#define CHASSIS_RC_DEADLINE     (5.0f)
#define CHASSIS_RC_MAX_RANGE    (660.0f)
#define CHASSIS_RC_MAX_SPEED    (3.0f)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)
#define CHASSIS_SPIN_VELOCITY   (6.0f)
#define CHASSIS_SPIN_POWER_LIMIT_W (100.0f)

/*-------------------- 云台 --------------------*/
/* DJI 云台控制帧。DM4310 始终使用独立的 DM 控制帧。 */
#define GIMBAL_CAN   1U
#define GIMBAL_STDID 0x1FFU

/* 模式开关使用 s[0]；DT7 对应 S1，ET08A 对应 Ch6（Aux2/SC）。 */
#define GIMBAL_RC_MODE_SWITCH           0U
#define GIMBAL_RC_YAW_CHANNEL           0U
#define GIMBAL_RC_PITCH_CHANNEL         1U
#define GIMBAL_RC_YAW_INPUT_DIRECTION   (-1.0f)
#define GIMBAL_RC_PITCH_INPUT_DIRECTION (1.0f)
#define GIMBAL_RC_DEADBAND              (10.0f)
#define GIMBAL_RC_TARGET_RATE_DEG_S     (120.0f)
#define GIMBAL_RC_TARGET_UPDATE_S       (0.001f)
/* Pitch 目标限位由 gimbal.c 根据已标定机械限位和当前 IMU 到机构的偏移实时计算。 */

/* 挡位逻辑：S0 上挡为安全/失能，中挡为手动 IMU 云台，下挡为视觉自瞄。
 * 视觉自瞄仍受 S0 安全挡和有效视觉帧超时保护，绝不允许上位机单独解除安全状态。 */

/*-------------------- 上下机视觉通信 --------------------*/
#define USB_CDC_MODE_TELEMETRY 0U
#define USB_CDC_MODE_VISION    1U

/* USB CDC 只能承载一种协议：视觉模式为 SP 二进制帧；网页模式为 CSV 调试遥测。 */
#ifndef USB_CDC_MODE
#define USB_CDC_MODE USB_CDC_MODE_TELEMETRY
#endif
#ifndef VISION_LINK_ENABLE
#define VISION_LINK_ENABLE 1U
#endif

#define VISION_LINK_TIMEOUT_MS       120U
#define VISION_STATE_PERIOD_MS       10U
#define VISION_MAX_ABS_YAW_RAD       (12.566371f)
#define VISION_MAX_ABS_PITCH_RAD     (3.141593f)
#define VISION_MAX_ABS_RATE_RAD_S    (30.0f)
#define VISION_MAX_ABS_ACCEL_RAD_S2  (300.0f)

/* BMI088 姿态外环目标单位为角度，DM4310 目标单位为弧度。
 * 本测试中两个电机的机械中位均设为 0.0f。 */
#define GIMBAL_IMU_YAW_TARGET_DEG       (0.0f)
#define GIMBAL_IMU_PITCH_TARGET_DEG     (0.0f)
#define GIMBAL_IMU_YAW_MOTOR_ZERO       (0.0f)
#define GIMBAL_IMU_PITCH_MOTOR_ZERO     (0.0f)
#define GIMBAL_IMU_YAW_OUTER_KP         (0.003f)
/* 一个机械弧度对应 57.2958 个 IMU 角度单位，使 IMU 外环保持一一对应的角度映射，
 * 避免目标尚未到达时就因行程限位而饱和。 */
#define GIMBAL_IMU_PITCH_OUTER_KP       (0.003f)
#define GIMBAL_IMU_YAW_OUTPUT_DIRECTION (1.0f)
#define GIMBAL_IMU_PITCH_OUTPUT_DIRECTION (1.0f)
#define GIMBAL_IMU_YAW_MAX_POSITION_OFFSET   (0.080f)
#define GIMBAL_IMU_PITCH_MAX_POSITION_OFFSET (0.040f)
#define GIMBAL_IMU_READY_MIN_GYRO_FRAMES     50U

/* 初始化时间与输入灵敏度。 */
#define GIMBAL_INIT_TIME               1000U
#define MOUSE_SENSITIVITY              (200000.0f)
#define REMOTE_CONTROLLER_SENSITIVITY  (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)

/* 每轴电机选型。只修改对应的 MOTOR_PROFILE 即可在 DM4310 和 GM6020 间切换；
 * 选型会自动带出控制模式和默认反馈 ID。下方的 CAN、ID、方向、减速比和 MID
 * 仍必须按实际安装的硬件填写。 */
#define GIMBAL_MOTOR_PROFILE_DM4310 1U
#define GIMBAL_MOTOR_PROFILE_GM6020 2U
#define GIMBAL_DIRECT_YAW_MOTOR_PROFILE   GIMBAL_MOTOR_PROFILE_DM4310
#define GIMBAL_DIRECT_PITCH_MOTOR_PROFILE GIMBAL_MOTOR_PROFILE_DM4310

/* 两轴 DM4310 测试映射：两台电机共用 CAN2，命令 ID 与反馈 ID 必须不同。
 * DM4310 反馈 ID 配置为从机命令 ID 加 0x50。
 * 物理轴中，Yaw 的反馈为 0x51，Pitch 的反馈为 0x52。 */
#define GIMBAL_DIRECT_YAW_ID        0x01U
#define GIMBAL_DIRECT_PITCH_ID      0x02U

#define GIMBAL_DIRECT_YAW_CAN   2U
#define GIMBAL_DIRECT_PITCH_CAN 2U

/* 电机型号。修改某轴型号时必须同时核对其 ID、CAN 和控制模式。 */
#if (GIMBAL_DIRECT_YAW_MOTOR_PROFILE == GIMBAL_MOTOR_PROFILE_DM4310)
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE DM_4310
#define GIMBAL_DIRECT_YAW_MODE DM_MODE_MIT
#define GIMBAL_DIRECT_YAW_MASTER_ID 0x51U
#elif (GIMBAL_DIRECT_YAW_MOTOR_PROFILE == GIMBAL_MOTOR_PROFILE_GM6020)
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE DJI_M6020
#define GIMBAL_DIRECT_YAW_MODE DJI_VOLTAGE_MODE
#define GIMBAL_DIRECT_YAW_MASTER_ID 0U
#else
#error "Unsupported yaw gimbal motor profile"
#endif

#if (GIMBAL_DIRECT_PITCH_MOTOR_PROFILE == GIMBAL_MOTOR_PROFILE_DM4310)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE DM_4310
#define GIMBAL_DIRECT_PITCH_MODE DM_MODE_MIT
#define GIMBAL_DIRECT_PITCH_MASTER_ID 0x52U
#elif (GIMBAL_DIRECT_PITCH_MOTOR_PROFILE == GIMBAL_MOTOR_PROFILE_GM6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE DJI_M6020
#define GIMBAL_DIRECT_PITCH_MODE DJI_VOLTAGE_MODE
#define GIMBAL_DIRECT_PITCH_MASTER_ID 0U
#else
#error "Unsupported pitch gimbal motor profile"
#endif

/* 转动方向 */
#define GIMBAL_DIRECT_YAW_DIRECTION   (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

/* 减速比 */
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO   (1.0f)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1.0f)

/* 控制模式 */

/* 已标定的机械零点：各轴对准机械中位后记录 DM4310 原始位置并填写到对应 MID。
 * Pitch 限位相对 PITCH_MID 定义，Yaw 没有机械限位。 */
#define GIMBAL_UPPER_LIMIT_PITCH (0.3f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.3f)
#define GIMBAL_PITCH_SOFT_LIMIT_MARGIN (0.015f)
#define GIMBAL_PITCH_LIMIT_REBASE_EPSILON_DEG (0.5f)
#define GIMBAL_DIRECT_YAW_MID    (0.0f)
#define GIMBAL_DIRECT_PITCH_MID  (3.141592654f) /* Pitch 安装零点为 180 度。 */

/* PID 参数：保留给云台闭环控制。 */
#define KP_GIMBAL_YAW_ANGLE       (3.0f)
#define KI_GIMBAL_YAW_ANGLE       (100.0f)
#define KD_GIMBAL_YAW_ANGLE       (2.5f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.05f)
#define MAX_OUT_GIMBAL_YAW_ANGLE  (13.0f)

#define KP_GIMBAL_YAW_VELOCITY       (2000.0f)
#define KI_GIMBAL_YAW_VELOCITY       (140.0f)
#define KD_GIMBAL_YAW_VELOCITY       (80.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (17000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY  (30000.0f)

#define KP_GIMBAL_PITCH_ANGLE       (4.5f)
#define KI_GIMBAL_PITCH_ANGLE       (0.0001f)
#define KD_GIMBAL_PITCH_ANGLE       (3.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (1.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE  (10.0f)

#define KP_GIMBAL_PITCH_VELOCITY       (1200.0f)
#define KI_GIMBAL_PITCH_VELOCITY       (30.0f)
#define KD_GIMBAL_PITCH_VELOCITY       (100.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY  (30000.0f)

/* DM4310 MIT 参数及力矩前馈，与 F407 云台控制帧保持一致。 */
#define GIMBAL_DM4310_MIT_KP (5.0f)
#define GIMBAL_DM4310_MIT_KI (2.0f)
#define GIMBAL_DM4310_MIT_KD (0.05f)
#define GIMBAL_DM4310_MIT_TORQUE (0.0f)

/*-------------------- 发射机构 --------------------*/
#define FRIC_RADIUS             (0.03f)
#define BULLET_NUM              8U
#define GUN_NUM                 1U
#define TRIGGER_REDUCTION_RATIO (1.0f)

/* F407 全向轮配置中两个摩擦轮使用 ID 6 和 ID 5。
 * 拨弹盘 ID 也写为 6，且原工程已注释其 CAN 发送，因此在确认真实且不冲突的 ID 前保持拨弹盘失能。 */
#define SHOOT_RC_MODE_SWITCH 1U
#define TRIGGER_RC_MODE_SWITCH 1U
#define SHOOT_TRIGGER_ENABLED 1U
#define TRIGGER_MOTOR_TYPE DJI_M2006
#define FRIC_MOTOR_TYPE    DJI_M3508
#define FRIC_MOTOR_R_TYPE  FRIC_MOTOR_TYPE
#define FRIC_MOTOR_L_TYPE  FRIC_MOTOR_TYPE
#define TRIGGER_MOTOR_ID   3U
#define FRIC_MOTOR_R_ID    1U
#define FRIC_MOTOR_L_ID    2U
#define TRIGGER_MOTOR_CAN  3U
#define FRIC_MOTOR_R_CAN   3U
#define FRIC_MOTOR_L_CAN   3U

#define TRIGGER_MOTOR_DIRECTION       (1)
#define TRIGGER_MOTOR_REDUCTION_RATIO (36.0f)

#define TRIGGER_SPEED    (600.0f)
#define TRIGGER_START_SPEED_THRESHOLD_RPM (20.0f)
#define TRIGGER_START_CURRENT             (2500.0f)
#define FRIC_R_SPEED     (666.0f)
#define FRIC_L_SPEED     (-666.0f)
#define FRIC_SPEED_LIMIT (600.0f)

#define HALF_ECD_RANGE       4096U
#define ECD_RANGE            8191U
#define MOTOR_RPM_TO_SPEED   (0.0029088821f)
#define MOTOR_ECD_TO_ANGLE   (0.0000213053f)
#define FULL_COUNT           18U

#define BLOCK_TRIGGER_SPEED (5.0f)
#define BLOCK_TIME          1000U
#define REVERSE_TIME        1250U
#define REVERSE_SPEED       (-20.0f)

#define TRIGGER_SPEED_MIT_KD        (0.0f)
#define TRIGGER_SPEED_PID_KP        (2.0f)
#define TRIGGER_SPEED_PID_KI        (0.5f)
#define TRIGGER_SPEED_PID_KD        (0.05f)
#define TRIGGER_SPEED_PID_MAX_OUT   (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT  (1000.0f)
#define TRIGGER_ANGEL_PID_KP        (2.0f)
#define TRIGGER_ANGEL_PID_KI        (0.5f)
#define TRIGGER_ANGEL_PID_KD        (0.05f)
#define TRIGGER_ANGEL_PID_MAX_OUT   (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT  (30.0f)
#define FRIC_SPEED_PID_KP           (666.0f)
#define FIRC_SPEED_PID_KI           (0.6f)
#define FRIC_SPEED_PID_KD           (1.0f)
#define FRIC_PID_MAX_OUT            (16000.0f)
#define FRIC_PID_MAX_IOUT           (1000.0f)
#define SHOOT_HEAT_REMAIN_VALUE     80U

/*-------------------- 板端命令安全配置 --------------------*/
#define ROBOT_CAN1_ENABLED          1U
#define ROBOT_CAN2_ENABLED          1U
#define ROBOT_CAN3_ENABLED          1U
#define ROBOT_COMMAND_ARMED_DEFAULT 1U

#endif /* ROBOT_PARAM_OMNI_H723_GIMBAL_H */
