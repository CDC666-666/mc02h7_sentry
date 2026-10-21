#include "robot_motor.h"

#include "robot_can.h"
#include "robot_param.h"
#include "stm32h7xx_hal.h"

#define ROBOT_MOTOR_MAX_COUNT ROBOT_MOTOR_TELEMETRY_MAX_COUNT
#define DJI_MOTOR_CURRENT_LIMIT 16384.0f
#define DM4310_P_MIN  (-12.5f)
#define DM4310_P_MAX  (12.5f)
#define DM4310_V_MIN  (-30.0f)
#define DM4310_V_MAX  (30.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)
#define DM4310_T_MIN  (-10.0f)
#define DM4310_T_MAX  (10.0f)

/* Motorevo 经典 CAN 单电机 MIT 范围，必须与 Motorevo Studio 中的
 * CAN COM 范围参数保持一致。 */
#define EVO4310_P_MIN  (-12.5f)
#define EVO4310_P_MAX  (12.5f)
#define EVO4310_V_MIN  (-65.0f)
#define EVO4310_V_MAX  (65.0f)
#define EVO4310_KP_MIN (0.0f)
#define EVO4310_KP_MAX (500.0f)
#define EVO4310_KD_MIN (0.0f)
#define EVO4310_KD_MAX (5.0f)
#define EVO4310_T_MIN  (-18.0f)
#define EVO4310_T_MAX  (18.0f)

volatile uint8_t robot_command_armed = ROBOT_COMMAND_ARMED_DEFAULT;
static RobotMotor_t *robot_motors[ROBOT_MOTOR_MAX_COUNT] = {0};
static uint8_t robot_motor_count = 0U;

static float RobotMotor_Constrain(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint16_t RobotMotor_FloatToUint(float value, float minimum, float maximum, uint8_t bits)
{
    const float scale = (float)((1UL << bits) - 1UL);
    value = RobotMotor_Constrain(value, minimum, maximum);
    return (uint16_t)((value - minimum) * scale / (maximum - minimum));
}

static float RobotMotor_UintToFloat(uint16_t value, float minimum, float maximum, uint8_t bits)
{
    const float scale = (float)((1UL << bits) - 1UL);
    return (float)value * (maximum - minimum) / scale + minimum;
}

static uint16_t RobotMotor_GetDjiCommandId(const RobotMotor_t *motor)
{
    if (motor->type == ROBOT_MOTOR_TYPE_DJI_M3508 ||
        motor->type == ROBOT_MOTOR_TYPE_DJI_M2006) return motor->command_id <= 4U ? 0x200U : 0x1FFU;
    if (motor->type == ROBOT_MOTOR_TYPE_DJI_GM6020) return motor->command_id <= 4U ? 0x1FFU : 0x2FFU;
    return 0U;
}

static uint8_t RobotMotor_GetDjiSlot(const RobotMotor_t *motor)
{
    return (uint8_t)((motor->command_id - 1U) % 4U);
}

static void RobotMotor_Register(RobotMotor_t *motor)
{
    if (robot_motor_count < ROBOT_MOTOR_MAX_COUNT) robot_motors[robot_motor_count++] = motor;
}

RobotMotorType_e RobotMotorTypeFromProfile(MotorType_e type)
{
    switch (type) {
        case DJI_M3508: return ROBOT_MOTOR_TYPE_DJI_M3508;
        case DJI_M2006: return ROBOT_MOTOR_TYPE_DJI_M2006;
        case DJI_M6020: return ROBOT_MOTOR_TYPE_DJI_GM6020;
        case DM_4310: return ROBOT_MOTOR_TYPE_DM4310;
        case EVO4310: return ROBOT_MOTOR_TYPE_EVO4310;
        default: return ROBOT_MOTOR_TYPE_NONE;
    }
}

void RobotMotor_Init(RobotMotor_t *motor, RobotMotorType_e type, uint8_t can_index,
                     uint16_t command_id, uint16_t feedback_id, int8_t direction,
                     float reduction_ratio, uint16_t mode)
{
    if (motor == 0) return;
    motor->type = type;
    motor->can_index = can_index;
    motor->command_id = command_id;
    motor->feedback_id = feedback_id;
    motor->direction = direction < 0 ? -1 : 1;
    motor->reduction_ratio = reduction_ratio > 0.0f ? reduction_ratio : 1.0f;
    motor->mode = mode;
    motor->online = false;
    motor->feedback.position = 0.0f;
    motor->feedback.velocity = 0.0f;
    motor->feedback.torque = 0.0f;
    motor->feedback.current = 0.0f;
    motor->feedback.temperature = 0.0f;
    motor->feedback.state = 0U;
    motor->feedback.last_feedback_ms = 0U;
    motor->setpoint.position = 0.0f;
    motor->setpoint.velocity = 0.0f;
    motor->setpoint.torque = 0.0f;
    motor->setpoint.current = 0.0f;
    if (command_id == 0U) return;
    if (motor->feedback_id == 0U) {
        if (type == ROBOT_MOTOR_TYPE_DJI_M3508 || type == ROBOT_MOTOR_TYPE_DJI_M2006)
            motor->feedback_id = 0x200U + command_id;
        else if (type == ROBOT_MOTOR_TYPE_DJI_GM6020) motor->feedback_id = 0x204U + command_id;
        else if (type == ROBOT_MOTOR_TYPE_DM4310) motor->feedback_id = 0x50U + command_id;
        else if (type == ROBOT_MOTOR_TYPE_EVO4310) motor->feedback_id = command_id;
    }
    RobotMotor_Register(motor);
}

bool RobotMotor_IsConfigured(const RobotMotor_t *motor)
{
    return motor != 0 && motor->type != ROBOT_MOTOR_TYPE_NONE && motor->can_index >= 1U &&
           motor->can_index <= 3U && motor->command_id != 0U && motor->feedback_id != 0U;
}

uint8_t RobotMotor_GetRegisteredCount(void)
{
    return robot_motor_count;
}

const RobotMotor_t *RobotMotor_GetRegistered(uint8_t index)
{
    return index < robot_motor_count ? robot_motors[index] : 0;
}

bool RobotMotor_HasFreshFeedback(const RobotMotor_t *motor, uint32_t timeout_ms)
{
    return RobotMotor_IsConfigured(motor) && motor->online &&
           (HAL_GetTick() - motor->feedback.last_feedback_ms <= timeout_ms);
}

void RobotMotor_SetCommandArmed(bool armed)
{
    robot_command_armed = armed ? 1U : 0U;
}

void RobotMotor_Receive(uint8_t can_index, uint32_t identifier, const uint8_t data[8])
{
    if (data == 0) return;
    for (uint8_t i = 0U; i < robot_motor_count; i++) {
        RobotMotor_t *motor = robot_motors[i];
        if (!RobotMotor_IsConfigured(motor) || motor->can_index != can_index ||
            motor->feedback_id != identifier) continue;
        if (motor->type == ROBOT_MOTOR_TYPE_DM4310) {
            uint16_t position = ((uint16_t)data[1] << 8U) | data[2];
            uint16_t velocity = ((uint16_t)data[3] << 4U) | (data[4] >> 4U);
            uint16_t torque = ((uint16_t)(data[4] & 0x0FU) << 8U) | data[5];
            motor->feedback.state = data[0] >> 4U;
            motor->feedback.position = RobotMotor_UintToFloat(position, DM4310_P_MIN, DM4310_P_MAX, 16U);
            motor->feedback.velocity = RobotMotor_UintToFloat(velocity, DM4310_V_MIN, DM4310_V_MAX, 12U);
            motor->feedback.torque = RobotMotor_UintToFloat(torque, DM4310_T_MIN, DM4310_T_MAX, 12U);
            motor->feedback.temperature = (float)data[6];
        } else if (motor->type == ROBOT_MOTOR_TYPE_EVO4310) {
            /* Motorevo 反馈：位置 B0..B1，速度 B2..B3[7:4]，
             * 扭矩 B3[3:0]..B4，温度 B5，状态/故障 B6..B7。 */
            uint16_t position = ((uint16_t)data[0] << 8U) | data[1];
            uint16_t velocity = ((uint16_t)data[2] << 4U) | (data[3] >> 4U);
            uint16_t torque = ((uint16_t)(data[3] & 0x0FU) << 8U) | data[4];

            motor->feedback.position = RobotMotor_UintToFloat(position, EVO4310_P_MIN, EVO4310_P_MAX, 16U);
            motor->feedback.velocity = RobotMotor_UintToFloat(velocity, EVO4310_V_MIN, EVO4310_V_MAX, 12U);
            motor->feedback.torque = RobotMotor_UintToFloat(torque, EVO4310_T_MIN, EVO4310_T_MAX, 12U);
            motor->feedback.temperature = (float)data[5] - 40.0f;
            motor->feedback.state = (uint8_t)(data[6] & 0x01U);
        } else {
            int16_t velocity = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
            int16_t current = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
            motor->feedback.position = (float)(((uint16_t)data[0] << 8U) | data[1]) * 6.283185307f / 8192.0f;
            motor->feedback.velocity = (float)velocity * 6.283185307f / 60.0f /
                                       motor->reduction_ratio * (float)motor->direction;
            motor->feedback.current = (float)current * (float)motor->direction;
            motor->feedback.temperature = (float)data[6];
        }
        motor->feedback.last_feedback_ms = HAL_GetTick();
        motor->online = true;
    }
}

bool RobotMotor_SendDjiCurrentsWithEnable(RobotMotor_t *const motors[], uint8_t motor_count,
                                          bool command_enabled)
{
    bool sent = false;
    for (uint8_t i = 0U; i < motor_count; i++) {
        RobotMotor_t *motor = motors[i];
        uint8_t frame[8] = {0};
        uint16_t command_id;
        if (!RobotMotor_IsConfigured(motor) ||
            (motor->type != ROBOT_MOTOR_TYPE_DJI_M3508 &&
             motor->type != ROBOT_MOTOR_TYPE_DJI_M2006 &&
             motor->type != ROBOT_MOTOR_TYPE_DJI_GM6020)) continue;
        command_id = RobotMotor_GetDjiCommandId(motor);
        for (uint8_t previous = 0U; previous < i; previous++) {
            RobotMotor_t *prior = motors[previous];
            if (RobotMotor_IsConfigured(prior) && prior->can_index == motor->can_index &&
                RobotMotor_GetDjiCommandId(prior) == command_id) {
                command_id = 0U;
                break;
            }
        }
        if (command_id == 0U) continue;
        for (uint8_t j = i; j < motor_count; j++) {
            RobotMotor_t *member = motors[j];
            int16_t current;
            uint8_t slot;
            if (!RobotMotor_IsConfigured(member) || member->can_index != motor->can_index ||
                RobotMotor_GetDjiCommandId(member) != command_id) continue;
            slot = RobotMotor_GetDjiSlot(member);
            current = command_enabled ?
                (int16_t)RobotMotor_Constrain(member->setpoint.current * (float)member->direction,
                                               -DJI_MOTOR_CURRENT_LIMIT, DJI_MOTOR_CURRENT_LIMIT) : 0;
            frame[slot * 2U] = (uint8_t)((uint16_t)current >> 8U);
            frame[slot * 2U + 1U] = (uint8_t)current;
        }
        if (RobotCan_Send(motor->can_index, command_id, frame)) sent = true;
    }
    return sent;
}

bool RobotMotor_SendDjiCurrents(RobotMotor_t *const motors[], uint8_t motor_count)
{
    return RobotMotor_SendDjiCurrentsWithEnable(motors, motor_count,
                                                robot_command_armed != 0U);
}

bool RobotMotor_SendDmEnable(const RobotMotor_t *motor)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
    return RobotMotor_IsConfigured(motor) && motor->type == ROBOT_MOTOR_TYPE_DM4310 &&
           robot_command_armed && RobotCan_Send(motor->can_index, motor->command_id, data);
}

bool RobotMotor_SendDmDisable(const RobotMotor_t *motor)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
    return RobotMotor_IsConfigured(motor) && motor->type == ROBOT_MOTOR_TYPE_DM4310 &&
           RobotCan_Send(motor->can_index, motor->command_id, data);
}

bool RobotMotor_SendDmMit(const RobotMotor_t *motor, float kp, float kd)
{
    uint8_t data[8];
    uint16_t position;
    uint16_t velocity;
    uint16_t gain_p;
    uint16_t gain_d;
    uint16_t torque;
    if (!RobotMotor_IsConfigured(motor) || motor->type != ROBOT_MOTOR_TYPE_DM4310 ||
        motor->mode != DM_MODE_MIT || !robot_command_armed) return false;
    position = RobotMotor_FloatToUint(motor->setpoint.position * motor->direction, DM4310_P_MIN, DM4310_P_MAX, 16U);
    velocity = RobotMotor_FloatToUint(motor->setpoint.velocity * motor->direction, DM4310_V_MIN, DM4310_V_MAX, 12U);
    gain_p = RobotMotor_FloatToUint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12U);
    gain_d = RobotMotor_FloatToUint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12U);
    torque = RobotMotor_FloatToUint(motor->setpoint.torque * motor->direction, DM4310_T_MIN, DM4310_T_MAX, 12U);
    data[0] = (uint8_t)(position >> 8U);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 4U);
    data[3] = (uint8_t)(((velocity & 0x0FU) << 4U) | (gain_p >> 8U));
    data[4] = (uint8_t)gain_p;
    data[5] = (uint8_t)(gain_d >> 4U);
    data[6] = (uint8_t)(((gain_d & 0x0FU) << 4U) | (torque >> 8U));
    data[7] = (uint8_t)torque;
    return RobotCan_Send(motor->can_index, motor->command_id, data);
}

bool RobotMotor_SendEvoEnable(const RobotMotor_t *motor)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};

    return RobotMotor_IsConfigured(motor) && motor->type == ROBOT_MOTOR_TYPE_EVO4310 &&
           robot_command_armed && RobotCan_Send(motor->can_index, 0x100U + motor->command_id, data);
}

bool RobotMotor_SendEvoDisable(const RobotMotor_t *motor)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};

    return RobotMotor_IsConfigured(motor) && motor->type == ROBOT_MOTOR_TYPE_EVO4310 &&
           RobotCan_Send(motor->can_index, 0x100U + motor->command_id, data);
}

bool RobotMotor_SendEvoQuery(const RobotMotor_t *motor)
{
    /* Motorevo 260617 及以后固件支持：只返回当前状态，不产生运动。 */
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFAU};

    return RobotMotor_IsConfigured(motor) && motor->type == ROBOT_MOTOR_TYPE_EVO4310 &&
           RobotCan_Send(motor->can_index, 0x100U + motor->command_id, data);
}

bool RobotMotor_SendEvoMit(const RobotMotor_t *motor, float kp, float kd)
{
    uint8_t data[8];
    uint16_t position;
    uint16_t velocity;
    uint16_t gain_p;
    uint16_t gain_d;
    uint16_t torque;

    if (!RobotMotor_IsConfigured(motor) || motor->type != ROBOT_MOTOR_TYPE_EVO4310 ||
        motor->mode != DM_MODE_MIT || !robot_command_armed) return false;
    position = RobotMotor_FloatToUint(motor->setpoint.position * motor->direction, EVO4310_P_MIN, EVO4310_P_MAX, 16U);
    velocity = RobotMotor_FloatToUint(motor->setpoint.velocity * motor->direction, EVO4310_V_MIN, EVO4310_V_MAX, 12U);
    gain_p = RobotMotor_FloatToUint(kp, EVO4310_KP_MIN, EVO4310_KP_MAX, 12U);
    gain_d = RobotMotor_FloatToUint(kd, EVO4310_KD_MIN, EVO4310_KD_MAX, 12U);
    torque = RobotMotor_FloatToUint(motor->setpoint.torque * motor->direction, EVO4310_T_MIN, EVO4310_T_MAX, 12U);
    data[0] = (uint8_t)(position >> 8U);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 4U);
    data[3] = (uint8_t)(((velocity & 0x0FU) << 4U) | (gain_p >> 8U));
    data[4] = (uint8_t)gain_p;
    data[5] = (uint8_t)(gain_d >> 4U);
    data[6] = (uint8_t)(((gain_d & 0x0FU) << 4U) | (torque >> 8U));
    data[7] = (uint8_t)torque;
    return RobotCan_Send(motor->can_index, 0x200U + motor->command_id, data);
}
