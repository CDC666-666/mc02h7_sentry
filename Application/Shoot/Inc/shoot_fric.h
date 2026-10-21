#ifndef SHOOT_FRIC_H
#define SHOOT_FRIC_H

#include <stdint.h>

#include "PID.h"
#include "robot_motor.h"

typedef enum {
    SHOOT_FRIC_STOP = 0,
    SHOOT_FRIC_READY,
} ShootFricMode_e;

typedef struct {
    RobotMotor_t motor[2];
    RobotMotor_t trigger;
    PID_Info_TypeDef velocity_pid[2];
    PID_Info_TypeDef trigger_velocity_pid;
    ShootFricMode_e mode;
    ShootFricMode_e trigger_mode;
    float target_velocity[2];
    float feedback_velocity[2];
    float trigger_target_velocity;
    float trigger_feedback_velocity;
    uint8_t online;
    uint8_t trigger_online;
    uint8_t trigger_command_sent;
} ShootFric_t;

extern ShootFric_t shoot_fric;

void ShootFricInit(void);
void ShootFricObserver(void);
void ShootFricSetMode(void);
void ShootFricReference(void);
void ShootFricConsole(void);
void ShootFricSendCmd(void);

#endif /* SHOOT_FRIC_H */
