#include "shoot_fric.h"

#include <math.h>

#include "bsp_buzzer.h"
#include "remote_control.h"
#include "robot_param.h"
#include "vision_link.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

#define SHOOT_RPM_TO_RAD_S (0.1047197551f)
#define SHOOT_RAD_S_TO_RPM (9.5492965869f)

ShootFric_t shoot_fric = {0};
static int8_t trigger_last_switch_state = -1;

static void ShootFricPlayTriggerSwitchSound(void)
{
    uint8_t switch_state;

    if (remote_ctrl.rc_lost) {
        trigger_last_switch_state = -1;
        return;
    }

    switch_state = remote_ctrl.rc.s[TRIGGER_RC_MODE_SWITCH];
    if (trigger_last_switch_state < 0) {
        trigger_last_switch_state = (int8_t)switch_state;
        return;
    }
    if (switch_state == (uint8_t)trigger_last_switch_state) return;

    if (switch_state == RC_SW_UP) {
        BSP_Buzzer_PlayTone(2200U, 140U);
    } else if (switch_state == RC_SW_MID) {
        BSP_Buzzer_PlayTone(3100U, 120U);
    } else if (switch_state == RC_SW_DOWN) {
        BSP_Buzzer_PlayTone(4000U, 120U);
    }
    trigger_last_switch_state = (int8_t)switch_state;
}

void ShootFricInit(void)
{
    static float fric_pid[7] = {
        FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD, 0.0f, 0.0f,
        FRIC_PID_MAX_IOUT, FRIC_PID_MAX_OUT
    };
    static float trigger_pid[7] = {
        TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD, 0.0f, 0.0f,
        TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_MAX_OUT
    };

    RobotMotor_Init(&shoot_fric.motor[0], RobotMotorTypeFromProfile(FRIC_MOTOR_R_TYPE),
                    FRIC_MOTOR_R_CAN, FRIC_MOTOR_R_ID, 0U, 1, 1.0f, DJI_CURRENT_MODE);
    RobotMotor_Init(&shoot_fric.motor[1], RobotMotorTypeFromProfile(FRIC_MOTOR_L_TYPE),
                    FRIC_MOTOR_L_CAN, FRIC_MOTOR_L_ID, 0U, 1, 1.0f, DJI_CURRENT_MODE);
    RobotMotor_Init(&shoot_fric.trigger, RobotMotorTypeFromProfile(TRIGGER_MOTOR_TYPE),
                    TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_ID, 0U, TRIGGER_MOTOR_DIRECTION,
                    TRIGGER_MOTOR_REDUCTION_RATIO, DJI_CURRENT_MODE);
    PID_Init(&shoot_fric.velocity_pid[0], PID_POSITION, fric_pid);
    PID_Init(&shoot_fric.velocity_pid[1], PID_POSITION, fric_pid);
    PID_Init(&shoot_fric.trigger_velocity_pid, PID_POSITION, trigger_pid);
    shoot_fric.mode = SHOOT_FRIC_STOP;
    shoot_fric.trigger_mode = SHOOT_FRIC_STOP;
    trigger_last_switch_state = -1;
}

void ShootFricObserver(void)
{
    shoot_fric.feedback_velocity[0] = shoot_fric.motor[0].feedback.velocity;
    shoot_fric.feedback_velocity[1] = shoot_fric.motor[1].feedback.velocity;
    shoot_fric.online = ((RobotMotor_IsConfigured(&shoot_fric.motor[0]) &&
                          shoot_fric.motor[0].online) ||
                         (RobotMotor_IsConfigured(&shoot_fric.motor[1]) &&
                          shoot_fric.motor[1].online)) ? 1U : 0U;
    /* 与 F407 拨弹盘一致：速度环统一使用 GM2006 电机轴原始 rpm，
       不使用减速后的拨弹盘输出轴角速度。 */
    shoot_fric.trigger_feedback_velocity = shoot_fric.trigger.feedback.velocity *
                                           shoot_fric.trigger.reduction_ratio * SHOOT_RAD_S_TO_RPM;
    shoot_fric.trigger_online = shoot_fric.trigger.online ? 1U : 0U;
}

void ShootFricSetMode(void)
{
    bool vision_fire_allowed = !VisionLink_IsControlSelected() || VisionLink_IsFirePermitted();

    if (remote_ctrl.rc_lost || !robot_command_armed ||
        remote_ctrl.rc.s[SHOOT_RC_MODE_SWITCH] != RC_SW_MID) {
        shoot_fric.mode = SHOOT_FRIC_STOP;
    } else {
        shoot_fric.mode = SHOOT_FRIC_READY;
    }

    /* S1 中档仍是发射机构总许可；S0 下档视觉模式还必须收到 mode=2 的有效视觉帧。 */
    if (SHOOT_TRIGGER_ENABLED && !remote_ctrl.rc_lost && vision_fire_allowed &&
        remote_ctrl.rc.s[TRIGGER_RC_MODE_SWITCH] == RC_SW_MID) {
        shoot_fric.trigger_mode = SHOOT_FRIC_READY;
    } else {
        shoot_fric.trigger_mode = SHOOT_FRIC_STOP;
    }
    ShootFricPlayTriggerSwitchSound();
}

void ShootFricReference(void)
{
    if (shoot_fric.mode == SHOOT_FRIC_READY) {
        shoot_fric.target_velocity[0] = FRIC_R_SPEED * SHOOT_RPM_TO_RAD_S;
        shoot_fric.target_velocity[1] = FRIC_L_SPEED * SHOOT_RPM_TO_RAD_S;
    } else {
        shoot_fric.target_velocity[0] = 0.0f;
        shoot_fric.target_velocity[1] = 0.0f;
    }

    shoot_fric.trigger_target_velocity = shoot_fric.trigger_mode == SHOOT_FRIC_READY ?
                                        TRIGGER_SPEED : 0.0f;
}

void ShootFricConsole(void)
{
    for (uint8_t i = 0U; i < 2U; i++) {
        shoot_fric.motor[i].setpoint.velocity = shoot_fric.target_velocity[i];
        shoot_fric.motor[i].setpoint.current = PID_Calculate(
            &shoot_fric.velocity_pid[i], shoot_fric.target_velocity[i], shoot_fric.feedback_velocity[i]);
    }
    shoot_fric.trigger.setpoint.velocity = shoot_fric.trigger_target_velocity;
    shoot_fric.trigger.setpoint.current = PID_Calculate(
        &shoot_fric.trigger_velocity_pid, shoot_fric.trigger_target_velocity,
        shoot_fric.trigger_feedback_velocity);

    /* 2006 静止时，F407 的速度环起始电流可能不足以克服拨弹机构静摩擦。
       只在低速起动阶段抬高电流，速度建立后立即完全由速度 PID 接管。 */
    if (shoot_fric.trigger_mode == SHOOT_FRIC_READY &&
        fabsf(shoot_fric.trigger_feedback_velocity) < TRIGGER_START_SPEED_THRESHOLD_RPM &&
        fabsf(shoot_fric.trigger.setpoint.current) < TRIGGER_START_CURRENT) {
        shoot_fric.trigger.setpoint.current = shoot_fric.trigger_target_velocity >= 0.0f ?
                                            TRIGGER_START_CURRENT : -TRIGGER_START_CURRENT;
    }
}

void ShootFricSendCmd(void)
{
    RobotMotor_t *shoot_motors[3] = {
        &shoot_fric.motor[0], &shoot_fric.motor[1], &shoot_fric.trigger
    };
    bool shoot_enabled = shoot_fric.mode == SHOOT_FRIC_READY ||
                         shoot_fric.trigger_mode == SHOOT_FRIC_READY;

    /* 拨弹盘仅由 S1 中档和遥控器在线控制，不依赖云台 S0 的全局使能状态。 */
    shoot_fric.trigger_command_sent = RobotMotor_SendDjiCurrentsWithEnable(
        shoot_motors, 3U, shoot_enabled) ? 1U : 0U;
}

#endif /* SHOOT_FRIC_TRIGGER */
