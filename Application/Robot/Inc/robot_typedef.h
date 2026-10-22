#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

#include <stdint.h>

/* Selectable module implementations. */
#define CHASSIS_NONE       0U
#define CHASSIS_OMNI_WHEEL 1U
#define CHASSIS_EVO4310_TEST 2U
#define CHASSIS_MECANUM_WHEEL 3U
#define CHASSIS_STEERING_WHEEL 4U

#define GIMBAL_NONE             0U
#define GIMBAL_YAW_PITCH_DIRECT 1U
#define GIMBAL_BIG_YAW_SMALL_YAW_PITCH 2U
#define GIMBAL_FOLD_YAW_PITCH 3U

#define SHOOT_NONE         0U
#define SHOOT_FRIC_TRIGGER 1U

/* 可选的额外姿态传感器类型。默认关闭，关闭时不创建任务、不占用 CAN。 */
#define EXTRA_IMU_NONE       0U
#define EXTRA_IMU_DM_IMU_L1  1U

#define CHASSIS_ONLY       0U
#define GIMBAL_ONLY        1U
#define CHASSIS_AND_GIMBAL 2U

/* 遥控器类型：DT7/DR16 使用 18 字节 DJI DBUS，其余三种使用 25 字节 SBUS。 */
#define RC_DT7      0U
#define RC_AT9S_PRO 1U
#define RC_HT8A     2U
#define RC_ET08A    3U

/* Motor types accepted by the H723 motor abstraction. */
typedef enum {
    MOTOR_NONE = 0,
    DJI_M3508,
    DJI_M2006,
    DJI_M6020,
    DM_4310,
    EVO4310,
} MotorType_e;

/* Motor command modes. DJI modes select the matching current/voltage group. */
#define MOTOR_MODE_NONE    0U
#define DJI_CURRENT_MODE   0xAAU
#define DJI_VOLTAGE_MODE   0xBBU
#define DM_MODE_MIT        0x000U
#define DM_MODE_POSITION   0x100U
#define DM_MODE_SPEED      0x200U

#endif /* ROBOT_TYPEDEF_H */
