#ifndef GIMBAL_H
#define GIMBAL_H

#include "robot_param.h"
#include "robot_motor.h"
#include "PID.h"

#if (GIMBAL_TYPE != GIMBAL_NONE)

typedef enum {
    GIMBAL_MODE_SAFE = 0,
    GIMBAL_MODE_IMU,
    GIMBAL_MODE_VISION,
    GIMBAL_MODE_REMOTE,
} GimbalMode_e;

typedef struct {
    RobotMotor_t yaw;
    RobotMotor_t pitch;
    PID_Info_TypeDef yaw_angle_pid;
    PID_Info_TypeDef yaw_velocity_pid;
    PID_Info_TypeDef pitch_angle_pid;
    PID_Info_TypeDef pitch_velocity_pid;
    GimbalMode_e mode;
} Gimbal_t;

extern Gimbal_t gimbal;

/* 供调试观察窗口查看的 IMU 外环状态量。 */
extern volatile float gimbal_imu_yaw_target_deg;
extern volatile float gimbal_imu_pitch_target_deg;
extern volatile float gimbal_imu_yaw_error_deg;
extern volatile float gimbal_imu_pitch_error_deg;
extern volatile float gimbal_imu_yaw_motor_reference;
extern volatile float gimbal_imu_pitch_motor_reference;
extern volatile float gimbal_imu_yaw_position_cmd;
extern volatile float gimbal_imu_pitch_position_cmd;
extern volatile float gimbal_imu_yaw_zero_deg;
extern volatile float gimbal_imu_pitch_zero_deg;
extern volatile float gimbal_yaw_joint_position_rad;
extern volatile float gimbal_pitch_joint_position_rad;
extern volatile float gimbal_imu_pitch_lower_limit_deg;
extern volatile float gimbal_imu_pitch_upper_limit_deg;
extern volatile float gimbal_pitch_motor_lower_limit;
extern volatile float gimbal_pitch_motor_upper_limit;

void GimbalPublish(void);
void GimbalInit(void);
void GimbalObserver(void);
void GimbalHandleException(void);
void GimbalSetMode(void);
void GimbalReference(void);
void GimbalConsole(void);
void GimbalSendCmd(void);
void GimbalSetReference(float yaw_position, float pitch_position);

#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
typedef struct {
    RobotMotor_t big_yaw;
    RobotMotor_t small_yaw;
    RobotMotor_t pitch;
    GimbalMode_e mode;
} GimbalThreeAxis_t;

extern GimbalThreeAxis_t gimbal_three_axis;

/* 三电机云台遥测量：便于确认大 Yaw 跟随目标与两个编码器闭环目标。 */
extern volatile float gimbal_three_big_yaw_target_deg;
extern volatile float gimbal_three_big_yaw_follow_target_deg;
extern volatile float gimbal_three_big_yaw_error_deg;
extern volatile float gimbal_three_small_yaw_reference;
extern volatile float gimbal_three_small_yaw_zero;
extern volatile float gimbal_three_imu_yaw_zero_deg;
extern volatile float gimbal_three_pitch_reference;
extern volatile float gimbal_three_big_yaw_position_cmd;
float GimbalThreeGetActualYawRad(void);
float GimbalThreeGetChassisYawDeltaRad(void);
float GimbalThreeGetActualPitchRad(void);
#endif

#endif /* GIMBAL_TYPE */

#endif /* GIMBAL_H */
