#include "chassis.h"

#include "robot_param.h"
#include "remote_control.h"
#include "stm32h7xx_hal.h"
#include "Referee_System.h"
#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
#include "gimbal.h"
#include "INS_Task.h"
#endif

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

Chassis_t chassis = {0};

enum {
    CHASSIS_WHEEL_ID1_INDEX = 0U,
    CHASSIS_WHEEL_ID2_INDEX = 1U,
    CHASSIS_WHEEL_ID3_INDEX = 2U,
    CHASSIS_WHEEL_ID4_INDEX = 3U,
};

void ChassisPublish(void)
{
}

void ChassisInit(void)
{
    static float wheel_pid[7] = {
        KP_OMNI_VEL, KI_OMNI_VEL, KD_OMNI_VEL, 0.0f, 0.0f,
        MAX_IOUT_OMNI_VEL, MAX_OUT_OMNI_VEL
    };

    RobotMotor_Init(&chassis.motor[0], RobotMotorTypeFromProfile(WHEEL_1_MOTOR_TYPE), WHEEL_1_CAN,
                    WHEEL_1_ID, 0U, WHEEL_1_DIRECTION, WHEEL_1_RATIO, WHEEL_1_MODE);
    RobotMotor_Init(&chassis.motor[1], RobotMotorTypeFromProfile(WHEEL_2_MOTOR_TYPE), WHEEL_2_CAN,
                    WHEEL_2_ID, 0U, WHEEL_2_DIRECTION, WHEEL_2_RATIO, WHEEL_2_MODE);
    RobotMotor_Init(&chassis.motor[2], RobotMotorTypeFromProfile(WHEEL_3_MOTOR_TYPE), WHEEL_3_CAN,
                    WHEEL_3_ID, 0U, WHEEL_3_DIRECTION, WHEEL_3_RATIO, WHEEL_3_MODE);
    RobotMotor_Init(&chassis.motor[3], RobotMotorTypeFromProfile(WHEEL_4_MOTOR_TYPE), WHEEL_4_CAN,
                    WHEEL_4_ID, 0U, WHEEL_4_DIRECTION, WHEEL_4_RATIO, WHEEL_4_MODE);
    for (uint8_t i = 0U; i < 4U; i++) {
        PID_Init(&chassis.velocity_pid[i], PID_POSITION, wheel_pid);
    }
    ChassisPower_Init(&chassis.power_manager);
    chassis.mode = CHASSIS_MODE_SAFE;
}

void ChassisObserver(void)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        chassis.feedback_velocity[i] = chassis.motor[i].feedback.velocity;
    }
}

void ChassisHandleException(void)
{
}

void ChassisSetMode(void)
{
    if (remote_ctrl.rc_lost || remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_UP ||
        !robot_command_armed) {
        chassis.mode = CHASSIS_MODE_SAFE;
    } else {
        chassis.mode = CHASSIS_MODE_OMNI;
    }
}

void ChassisReference(void)
{
    if (chassis.mode == CHASSIS_MODE_SAFE) {
        chassis.reference.vx = 0.0f;
        chassis.reference.vy = 0.0f;
        chassis.reference.wz = 0.0f;
    } else {
        float remote_vx = (float)remote_ctrl.rc.ch[3] / CHASSIS_RC_MAX_RANGE *
                          CHASSIS_RC_MAX_SPEED;
        float remote_vy = -(float)remote_ctrl.rc.ch[2] / CHASSIS_RC_MAX_RANGE *
                          CHASSIS_RC_MAX_SPEED;
#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
        if (INS_IsImuReady() != 0U) {
            float gimbal_heading = (INS_Info.Yaw_Angle - gimbal_three_imu_yaw_zero_deg) *
                                   0.01745329252f;
            float sin_heading = sinf(gimbal_heading);
            float cos_heading = cosf(gimbal_heading);
            chassis.reference.vx = remote_vx * cos_heading - remote_vy * sin_heading;
            chassis.reference.vy = remote_vx * sin_heading + remote_vy * cos_heading;
        } else {
            chassis.reference.vx = remote_vx;
            chassis.reference.vy = remote_vy;
        }
#else
        chassis.reference.vx = remote_vx;
        chassis.reference.vy = remote_vy;
#endif
        chassis.reference.wz = remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN ?
                               CHASSIS_SPIN_VELOCITY : 0.0f;

        if (chassis.reference.vx < CHASSIS_RC_DEADLINE / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED &&
            chassis.reference.vx > -CHASSIS_RC_DEADLINE / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED) {
            chassis.reference.vx = 0.0f;
        }
        if (chassis.reference.vy < CHASSIS_RC_DEADLINE / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED &&
            chassis.reference.vy > -CHASSIS_RC_DEADLINE / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED) {
            chassis.reference.vy = 0.0f;
        }
    }
}

void ChassisConsole(void)
{
    /* 麦轮编号与 F407 chassis_mecanum.c 一致：ID1 左前、ID2 右前、
     * ID3 右后、ID4 左后。 */
    const float translation = 1.0f / MECANUM_WHEEL_RADIUS;
    const float rotation = MECANUM_WHEELBASE_PLUS_TRACK * chassis.reference.wz *
                           translation;

    chassis.motor[CHASSIS_WHEEL_ID1_INDEX].setpoint.velocity =
        (-chassis.reference.vx + chassis.reference.vy - rotation) * WHEEL_1_RATIO;
    chassis.motor[CHASSIS_WHEEL_ID2_INDEX].setpoint.velocity =
        ( chassis.reference.vx + chassis.reference.vy - rotation) * WHEEL_2_RATIO;
    chassis.motor[CHASSIS_WHEEL_ID3_INDEX].setpoint.velocity =
        ( chassis.reference.vx - chassis.reference.vy - rotation) * WHEEL_3_RATIO;
    chassis.motor[CHASSIS_WHEEL_ID4_INDEX].setpoint.velocity =
        (-chassis.reference.vx - chassis.reference.vy - rotation) * WHEEL_4_RATIO;

    for (uint8_t i = 0U; i < 4U; i++) {
        float motor_velocity = chassis.feedback_velocity[i] * chassis.motor[i].reduction_ratio;
        chassis.motor[i].setpoint.current = PID_Calculate(
            &chassis.velocity_pid[i], chassis.motor[i].setpoint.velocity, motor_velocity);
    }
}

void ChassisSendCmd(void)
{
    RobotMotor_t *motors[4] = {&chassis.motor[0], &chassis.motor[1], &chassis.motor[2], &chassis.motor[3]};
#if (CHASSIS_POWER_LIMITER_ENABLE != 0U)
    static const ChassisPowerModel_t m3508_power_model = {
        CHASSIS_M3508_POWER_K0, CHASSIS_M3508_POWER_K1, CHASSIS_M3508_POWER_K2,
        CHASSIS_M3508_POWER_K3, CHASSIS_M3508_POWER_K4, CHASSIS_M3508_POWER_K5,
        CHASSIS_M3508_POWER_CURRENT_CONVERSION
    };
    float desired_current[CHASSIS_POWER_MOTOR_COUNT];
    float target_motor_rad_s[CHASSIS_POWER_MOTOR_COUNT];
    float feedback_motor_rad_s[CHASSIS_POWER_MOTOR_COUNT];
    float total_limit_w = CHASSIS_POWER_FALLBACK_LIMIT_W;

    if (remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN) {
        total_limit_w = CHASSIS_SPIN_POWER_LIMIT_W;
    }

#if (CHASSIS_POWER_USE_REFEREE_LIMIT != 0U)
    if (Referee_System_Info.robot_status.chassis_power_limit > 0U) {
        total_limit_w = (float)Referee_System_Info.robot_status.chassis_power_limit;
    }
#endif
    for (uint8_t i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; i++) {
        desired_current[i] = chassis.motor[i].setpoint.current;
        target_motor_rad_s[i] = chassis.motor[i].setpoint.velocity;
        feedback_motor_rad_s[i] = chassis.feedback_velocity[i] * chassis.motor[i].reduction_ratio;
    }
    ChassisPower_Limit(&chassis.power_manager, &m3508_power_model, desired_current,
                       target_motor_rad_s, feedback_motor_rad_s, total_limit_w,
                       CHASSIS_POWER_BUFFER_ATTENUATION, CHASSIS_POWER_COMPENSATION,
                       CHASSIS_POWER_ERROR_EQUAL_RPM, CHASSIS_POWER_RESERVED_BORDER_W,
                       CHASSIS_POWER_RESERVED_PER_MOTOR_W);
    for (uint8_t i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; i++) {
        chassis.motor[i].setpoint.current = desired_current[i];
    }
#else
    ChassisPower_Init(&chassis.power_manager);
#endif
    (void)RobotMotor_SendDjiCurrents(motors, 4U);
}

void ChassisSetReference(float vx, float vy, float wz)
{
    chassis.reference.vx = vx;
    chassis.reference.vy = vy;
    chassis.reference.wz = wz;
}

#endif /* CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL */
