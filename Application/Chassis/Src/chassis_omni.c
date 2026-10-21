#include "chassis.h"

#include "robot_param.h"
#include "remote_control.h"
#include "stm32h7xx_hal.h"
#include "Referee_System.h"
#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
#include "gimbal.h"
#include "INS_Task.h"
#endif

#if 0 /* EVO4310 测试实现已移至 chassis_evo4310_test.c */

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

    /* Kp 为零时位置字段不参与控制，仍写入当前反馈位置以避免误切换 Kp 后跳变。 */
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

    /* 测试类型独立于云台 IMU 就绪状态，仅由遥控器 S1 和失联状态保证安全。 */
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

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

Chassis_t chassis = {0};
static bool chassis_spin_active = false;

/* 轮序严格跟随 F407 全向轮工程：数组下标、CAN1 电机 ID 与矩阵行一一对应。 */
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
    static float gimbal_follow_pid[7] = {
        KP_CHASSIS_FOLLOW_GIMBAL, KI_CHASSIS_FOLLOW_GIMBAL,
        KD_CHASSIS_FOLLOW_GIMBAL, 0.0f, 0.0f,
        MAX_IOUT_CHASSIS_FOLLOW_GIMBAL, MAX_OUT_CHASSIS_FOLLOW_GIMBAL
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
    PID_Init(&chassis.gimbal_follow_pid, PID_POSITION, gimbal_follow_pid);
    ChassisPower_Init(&chassis.power_manager);
    chassis.mode = CHASSIS_MODE_SAFE;
    chassis_spin_active = false;
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
        chassis_spin_active = false;
        chassis.gimbal_follow_pid.PID_Calc_Clear(&chassis.gimbal_follow_pid);
        for (uint8_t i = 0U; i < 4U; i++) {
            chassis.velocity_pid[i].PID_Calc_Clear(&chassis.velocity_pid[i]);
        }
    } else {
        /* 当前遥控器接线：ch2 为前后，ch3 为左右。 */
        float remote_vx = (float)remote_ctrl.rc.ch[2] / CHASSIS_RC_MAX_RANGE *
                          CHASSIS_RC_MAX_SPEED;
        float remote_vy = -(float)remote_ctrl.rc.ch[3] / CHASSIS_RC_MAX_RANGE *
                          CHASSIS_RC_MAX_SPEED;
#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
        /* 用大小 Yaw 实际反馈的串联偏角旋转底盘平移指令。 */
        float gimbal_heading = GimbalThreeGetChassisYawDeltaRad();
        float sin_heading = sinf(gimbal_heading);
        float cos_heading = cosf(gimbal_heading);
        chassis.reference.vx = remote_vx * cos_heading - remote_vy * sin_heading;
        chassis.reference.vy = remote_vx * sin_heading + remote_vy * cos_heading;
#else
        chassis.reference.vx = remote_vx;
        chassis.reference.vy = remote_vy;
#endif
        {
            const bool spin_requested =
                remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN;

            if (spin_requested) {
                chassis.gimbal_follow_pid.PID_Calc_Clear(&chassis.gimbal_follow_pid);
                chassis.reference.wz = CHASSIS_SPIN_VELOCITY;
            } else if (chassis_spin_active) {
                /* Stop the spin command and drain wheel-loop history before follow resumes. */
                chassis.reference.wz = 0.0f;
                chassis.gimbal_follow_pid.PID_Calc_Clear(&chassis.gimbal_follow_pid);
                for (uint8_t i = 0U; i < 4U; i++) {
                    chassis.velocity_pid[i].PID_Calc_Clear(&chassis.velocity_pid[i]);
                }
            } else {
                chassis.reference.wz = PID_Calculate(&chassis.gimbal_follow_pid,
                                                     0.0f, gimbal_heading);
            }
            chassis_spin_active = spin_requested;
        }

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
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
    /* 完全复现 F407 chassis_omni.c 的车体坐标变换及四轮逆运动学矩阵。 */
    const float chassis_vx = -chassis.reference.vy;
    const float chassis_vy = -chassis.reference.vx;
    const float translation = 1.414213562f / WHEEL_RADIUS;
    const float rotation = WHEEL_CENTER_DISTANCE * chassis.reference.wz / WHEEL_RADIUS;

    /* ID1: (+vx -vy), ID2: (+vx +vy), ID3: (-vx +vy), ID4: (-vx -vy)。 */
    chassis.motor[CHASSIS_WHEEL_ID1_INDEX].setpoint.velocity =
        (translation * (chassis_vx - chassis_vy) - rotation) * WHEEL_1_RATIO;
    chassis.motor[CHASSIS_WHEEL_ID2_INDEX].setpoint.velocity =
        (translation * (chassis_vx + chassis_vy) - rotation) * WHEEL_2_RATIO;
    chassis.motor[CHASSIS_WHEEL_ID3_INDEX].setpoint.velocity =
        (translation * (-chassis_vx + chassis_vy) - rotation) * WHEEL_3_RATIO;
    chassis.motor[CHASSIS_WHEEL_ID4_INDEX].setpoint.velocity =
        (translation * (-chassis_vx - chassis_vy) - rotation) * WHEEL_4_RATIO;
#endif

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

#endif /* CHASSIS_TYPE */
