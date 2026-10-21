#ifndef CHASSIS_STEERING_H
#define CHASSIS_STEERING_H

#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#include "PID.h"
#include "robot_motor.h"

#define STEERING_WHEEL_COUNT 4U

typedef enum {
    STEERING_CHASSIS_SAFE = 0U,
    STEERING_CHASSIS_READY,
    STEERING_CHASSIS_ACTIVE,
    STEERING_CHASSIS_ERROR,
} SteeringChassisMode_e;

typedef struct {
    RobotMotor_t steering_motor;
    RobotMotor_t drive_motor;
    PID_Info_TypeDef steering_angle_pid;
    PID_Info_TypeDef drive_velocity_pid;
    float steering_zero_rad;
    float steering_feedback_rad;
    float steering_target_rad;
    float drive_target_mps;
    float drive_feedback_mps;
    float steering_current;
    float drive_current;
    uint8_t reverse_drive;
} SteeringWheel_t;

typedef struct {
    SteeringWheel_t wheel[STEERING_WHEEL_COUNT];
    SteeringChassisMode_e mode;
    uint32_t error_code;
    float vx_reference_mps;
    float vy_reference_mps;
    float wz_reference_rad_s;
} SteeringChassis_t;

extern SteeringChassis_t steering_chassis;

void SteeringChassisSetReference(float vx_mps, float vy_mps, float wz_rad_s);
void SteeringChassisLock(void);

#endif /* CHASSIS_TYPE == CHASSIS_STEERING_WHEEL */
#endif /* CHASSIS_STEERING_H */
