#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_param.h"
#include "robot_motor.h"
#include "PID.h"
#include "chassis_power.h"

#if (CHASSIS_TYPE != CHASSIS_NONE)

typedef enum {
    CHASSIS_MODE_SAFE = 0,
    CHASSIS_MODE_OMNI,
} ChassisMode_e;

typedef struct {
    RobotMotor_t motor[4];
    PID_Info_TypeDef velocity_pid[4];
    PID_Info_TypeDef gimbal_follow_pid;
    ChassisMode_e mode;
    struct {
        float vx;
        float vy;
        float wz;
    } reference;
    float feedback_velocity[4];
    ChassisPowerManager_t power_manager;
} Chassis_t;

extern Chassis_t chassis;

void ChassisPublish(void);
void ChassisInit(void);
void ChassisObserver(void);
void ChassisHandleException(void);
void ChassisSetMode(void);
void ChassisReference(void);
void ChassisConsole(void);
void ChassisSendCmd(void);
void ChassisSetReference(float vx, float vy, float wz);

#endif /* CHASSIS_TYPE */

#endif /* CHASSIS_H */
