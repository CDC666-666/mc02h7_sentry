#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_typedef.h"

typedef enum {
    ROBOT_MOTOR_TYPE_NONE = 0,
    ROBOT_MOTOR_TYPE_DJI_M3508,
    ROBOT_MOTOR_TYPE_DJI_M2006,
    ROBOT_MOTOR_TYPE_DJI_GM6020,
    ROBOT_MOTOR_TYPE_DM4310,
    ROBOT_MOTOR_TYPE_EVO4310,
} RobotMotorType_e;

typedef struct {
    float position;
    float velocity;
    float torque;
    float current;
    float temperature;
    uint8_t state;
    uint32_t last_feedback_ms;
} RobotMotorFeedback_t;

typedef struct {
    float position;
    float velocity;
    float torque;
    float current;
} RobotMotorSetpoint_t;

typedef struct {
    RobotMotorType_e type;
    uint8_t can_index;
    uint16_t command_id;
    uint16_t feedback_id;
    int8_t direction;
    float reduction_ratio;
    uint16_t mode;
    bool online;
    RobotMotorFeedback_t feedback;
    RobotMotorSetpoint_t setpoint;
} RobotMotor_t;

extern volatile uint8_t robot_command_armed;

#define ROBOT_MOTOR_TELEMETRY_MAX_COUNT 10U

RobotMotorType_e RobotMotorTypeFromProfile(MotorType_e type);
void RobotMotor_Init(RobotMotor_t *motor, RobotMotorType_e type, uint8_t can_index,
                     uint16_t command_id, uint16_t feedback_id, int8_t direction,
                     float reduction_ratio, uint16_t mode);
void RobotMotor_Receive(uint8_t can_index, uint32_t identifier, const uint8_t data[8]);
bool RobotMotor_IsConfigured(const RobotMotor_t *motor);
uint8_t RobotMotor_GetRegisteredCount(void);
const RobotMotor_t *RobotMotor_GetRegistered(uint8_t index);
bool RobotMotor_HasFreshFeedback(const RobotMotor_t *motor, uint32_t timeout_ms);
void RobotMotor_SetCommandArmed(bool armed);
bool RobotMotor_SendDjiCurrents(RobotMotor_t *const motors[], uint8_t motor_count);
bool RobotMotor_SendDjiCurrentsWithEnable(RobotMotor_t *const motors[], uint8_t motor_count,
                                          bool command_enabled);
bool RobotMotor_SendDmEnable(const RobotMotor_t *motor);
bool RobotMotor_SendDmDisable(const RobotMotor_t *motor);
bool RobotMotor_SendDmMit(const RobotMotor_t *motor, float kp, float kd);
bool RobotMotor_SendEvoEnable(const RobotMotor_t *motor);
bool RobotMotor_SendEvoDisable(const RobotMotor_t *motor);
bool RobotMotor_SendEvoQuery(const RobotMotor_t *motor);
bool RobotMotor_SendEvoMit(const RobotMotor_t *motor, float kp, float kd);

#endif /* ROBOT_MOTOR_H */
