#ifndef GIMBAL_FOLD_YAW_PITCH_H
#define GIMBAL_FOLD_YAW_PITCH_H

#include "robot_param.h"

#if (GIMBAL_TYPE == GIMBAL_FOLD_YAW_PITCH)

#include "robot_motor.h"

typedef enum {
    FOLD_GIMBAL_LOCKED = 0U,
    FOLD_GIMBAL_OPENING,
    FOLD_GIMBAL_READY,
    FOLD_GIMBAL_CLOSING,
    FOLD_GIMBAL_ERROR,
} FoldGimbalMode_e;

typedef struct {
    RobotMotor_t fold_motor;
    RobotMotor_t yaw_motor;
    RobotMotor_t pitch_motor;
    FoldGimbalMode_e mode;
    uint32_t error_code;
    uint8_t fold_opened;
    uint8_t reference_captured;
    float fold_target_rad;
    float yaw_target_rad;
    float pitch_target_rad;
} FoldGimbal_t;

extern FoldGimbal_t fold_gimbal;

void FoldGimbalRequestOpen(void);
void FoldGimbalRequestClose(void);
void FoldGimbalLock(void);

#endif /* GIMBAL_TYPE == GIMBAL_FOLD_YAW_PITCH */
#endif /* GIMBAL_FOLD_YAW_PITCH_H */
