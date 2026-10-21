#include "chassis.h"

#include "robot_param.h"
#include "remote_control.h"
#include "stm32h7xx_hal.h"

#if (CHASSIS_TYPE == CHASSIS_EVO4310_TEST)

Chassis_t chassis = {0};
static bool evo4310_test_enabled = false;
static uint32_t evo4310_test_last_query_ms = 0U;

void ChassisPublish(void)
{
}

void ChassisInit(void)
{
    RobotMotor_Init(&chassis.motor[0], ROBOT_MOTOR_TYPE_EVO4310,
                    EVO4310_TEST_CAN, EVO4310_TEST_ID, EVO4310_TEST_FEEDBACK_ID,
                    EVO4310_TEST_DIRECTION, EVO4310_TEST_REDUCTION_RATIO, DM_MODE_MIT);
    chassis.mode = CHASSIS_MODE_SAFE;
    evo4310_test_enabled = false;
    evo4310_test_last_query_ms = 0U;
}

void ChassisObserver(void)
{
    chassis.feedback_velocity[0] = chassis.motor[0].feedback.velocity;
}

void ChassisHandleException(void)
{
}

void ChassisSetMode(void)
{
    if (remote_ctrl.rc_lost || remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_UP) {
        chassis.mode = CHASSIS_MODE_SAFE;
    } else {
        chassis.mode = CHASSIS_MODE_OMNI;
    }
}

void ChassisReference(void)
{
    if (chassis.mode == CHASSIS_MODE_SAFE) {
        chassis.reference.vx = 0.0f;
    } else {
        chassis.reference.vx = (float)remote_ctrl.rc.ch[3] / CHASSIS_RC_MAX_RANGE *
                               EVO4310_TEST_RC_MAX_VELOCITY;
        if (chassis.reference.vx < CHASSIS_RC_DEADLINE / CHASSIS_RC_MAX_RANGE *
                                   EVO4310_TEST_RC_MAX_VELOCITY &&
            chassis.reference.vx > -CHASSIS_RC_DEADLINE / CHASSIS_RC_MAX_RANGE *
                                   EVO4310_TEST_RC_MAX_VELOCITY) {
            chassis.reference.vx = 0.0f;
        }
    }
    chassis.reference.vy = 0.0f;
    chassis.reference.wz = 0.0f;
}

void ChassisConsole(void)
{
    if (!chassis.motor[0].online) {
        chassis.motor[0].setpoint.velocity = 0.0f;
        chassis.motor[0].setpoint.torque = 0.0f;
        return;
    }

    chassis.motor[0].setpoint.position = chassis.motor[0].feedback.position *
                                         (float)chassis.motor[0].direction;
    chassis.motor[0].setpoint.velocity = chassis.reference.vx;
    chassis.motor[0].setpoint.torque = EVO4310_TEST_MIT_TORQUE;
}

void ChassisSendCmd(void)
{
    if (chassis.mode == CHASSIS_MODE_SAFE) {
        RobotMotor_SetCommandArmed(false);
        if (evo4310_test_enabled) (void)RobotMotor_SendEvoDisable(&chassis.motor[0]);
        evo4310_test_enabled = false;
        evo4310_test_last_query_ms = 0U;
        return;
    }

    RobotMotor_SetCommandArmed(true);
    if (!evo4310_test_enabled) {
        evo4310_test_enabled = RobotMotor_SendEvoEnable(&chassis.motor[0]);
        return;
    }
    if (!RobotMotor_HasFreshFeedback(&chassis.motor[0], 200U)) {
        if (HAL_GetTick() - evo4310_test_last_query_ms >= 100U) {
            evo4310_test_last_query_ms = HAL_GetTick();
            (void)RobotMotor_SendEvoQuery(&chassis.motor[0]);
        }
        return;
    }
    (void)RobotMotor_SendEvoMit(&chassis.motor[0], EVO4310_TEST_MIT_KP,
                                 EVO4310_TEST_MIT_KD);
}

void ChassisSetReference(float vx, float vy, float wz)
{
    chassis.reference.vx = vx;
    chassis.reference.vy = vy;
    chassis.reference.wz = wz;
}

#endif /* CHASSIS_TYPE == CHASSIS_EVO4310_TEST */
