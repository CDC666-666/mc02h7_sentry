#include "gimbal.h"

#include "INS_Task.h"
#include "bsp_buzzer.h"
#include "extra_imu.h"
#include "remote_control.h"
#include "stm32h7xx_hal.h"
#include "vision_link.h"

#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)

#define GIMBAL_THREE_RAD_TO_DEG (57.2957795f)

GimbalThreeAxis_t gimbal_three_axis = {0};
volatile float gimbal_three_big_yaw_target_deg = 0.0f;
volatile float gimbal_three_big_yaw_follow_target_deg = 0.0f;
volatile float gimbal_three_big_yaw_error_deg = 0.0f;
volatile float gimbal_three_small_yaw_reference = 0.0f;
volatile float gimbal_three_imu_yaw_zero_deg = 0.0f;
volatile float gimbal_three_pitch_reference = 0.0f;
volatile float gimbal_three_big_yaw_position_cmd = 0.0f;

static bool big_yaw_enabled = false;
static bool small_yaw_enabled = false;
static bool pitch_enabled = false;
static uint32_t big_yaw_enable_last_ms = 0U;
static uint32_t pitch_enable_last_ms = 0U;
static bool big_yaw_reference_captured = false;
static bool small_yaw_reference_captured = false;
static bool pitch_reference_captured = false;
static float big_yaw_imu_start_deg = 0.0f;
static float big_yaw_zero = 0.0f;
volatile float gimbal_three_small_yaw_zero = 0.0f;
static float pitch_zero = 0.0f;
static int8_t last_switch_state = -1;
static bool gimbal_three_previous_safe = true;
static bool gimbal_three_spin_heading_captured = false;
static float gimbal_three_spin_heading_zero_deg = 0.0f;
static float gimbal_three_spin_heading_last_deg = 0.0f;
static float gimbal_three_spin_heading_accum_deg = 0.0f;
static bool extra_imu_yaw_reference_captured = false;
static float extra_imu_relative_yaw_zero_deg = 0.0f;
static PID_Info_TypeDef small_yaw_angle_pid;
static PID_Info_TypeDef small_yaw_velocity_pid;

static float GimbalThreeConstrain(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float GimbalThreeWrapDegrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float GimbalThreeWrapRadians(float angle)
{
    while (angle > 3.141592654f) angle -= 6.283185307f;
    while (angle < -3.141592654f) angle += 6.283185307f;
    return angle;
}

static float GimbalThreeNearestAngle(float target, float reference)
{
    float error = GimbalThreeWrapRadians(target - reference);
    return reference + error;
}

static bool GimbalThreeMotorUsesDji(const RobotMotor_t *motor)
{
    return motor != 0 && motor->type == ROBOT_MOTOR_TYPE_DJI_GM6020;
}

static void GimbalThreeInitSmallYawPids(void)
{
    static float angle_pid[7] = {
        GIMBAL_SMALL_YAW_6020_ANGLE_KP, GIMBAL_SMALL_YAW_6020_ANGLE_KI,
        GIMBAL_SMALL_YAW_6020_ANGLE_KD, 0.0f, 0.0f,
        GIMBAL_SMALL_YAW_6020_ANGLE_MAX_IOUT, GIMBAL_SMALL_YAW_6020_ANGLE_MAX_OUT
    };
    static float velocity_pid[7] = {
        GIMBAL_SMALL_YAW_6020_SPEED_KP, GIMBAL_SMALL_YAW_6020_SPEED_KI,
        GIMBAL_SMALL_YAW_6020_SPEED_KD, 0.0f, 0.0f,
        GIMBAL_SMALL_YAW_6020_SPEED_MAX_IOUT, GIMBAL_SMALL_YAW_6020_SPEED_MAX_OUT
    };

    PID_Init(&small_yaw_angle_pid, PID_POSITION, angle_pid);
    PID_Init(&small_yaw_velocity_pid, PID_POSITION, velocity_pid);
}

static void GimbalThreeClearSmallYawPids(void)
{
    small_yaw_angle_pid.PID_Calc_Clear(&small_yaw_angle_pid);
    small_yaw_velocity_pid.PID_Calc_Clear(&small_yaw_velocity_pid);
}

static void GimbalThreeDisableMitMotor(const RobotMotor_t *motor)
{
    if (motor == 0) return;
    if (motor->type == ROBOT_MOTOR_TYPE_DM4310) (void)RobotMotor_SendDmDisable(motor);
    else if (motor->type == ROBOT_MOTOR_TYPE_EVO4310) (void)RobotMotor_SendEvoDisable(motor);
}

static float GimbalThreeMoveTowardsDegrees(float current, float target, float maximum_step)
{
    float error = GimbalThreeWrapDegrees(target - current);

    error = GimbalThreeConstrain(error, -maximum_step, maximum_step);
    return GimbalThreeWrapDegrees(current + error);
}

static float GimbalThreeCommandPosition(const RobotMotor_t *motor)
{
    return motor->feedback.position * (float)motor->direction;
}

static bool GimbalThreeUseExtraImuYaw(void)
{
#if (EXTRA_IMU_CONTROL_ENABLE != 0U) && (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    return extra_imu_yaw_reference_captured && ExtraImu_HasYawReference();
#else
    return false;
#endif
}

static float GimbalThreeSmallYawFeedbackPosition(void)
{
    /* 6020 快速位置环使用自身编码器；外挂 IMU 只用于真实指向估计。 */
    return GimbalThreeCommandPosition(&gimbal_three_axis.small_yaw);
}

static float GimbalThreeSmallYawTruePosition(void)
{
    if (GimbalThreeUseExtraImuYaw()) {
        const float relative_yaw_deg = GimbalThreeWrapDegrees(
            (extra_imu.yaw_deg - INS_Info.Yaw_Angle) - extra_imu_relative_yaw_zero_deg) *
            EXTRA_IMU_YAW_DIRECTION;
        return gimbal_three_small_yaw_zero + relative_yaw_deg /
               GIMBAL_THREE_RAD_TO_DEG;
    }

    return GimbalThreeCommandPosition(&gimbal_three_axis.small_yaw);
}

static float GimbalThreeSmallYawFollowPosition(void)
{
    /* 外置 IMU 安装在小 yaw 头部时，使用它测得的小 yaw 实际偏移作为
     * 大 yaw 的参考；大 yaw 通过 FOLLOW_DIRECTION 反向补偿该偏移。 */
    if (GimbalThreeUseExtraImuYaw()) return GimbalThreeSmallYawTruePosition();
    if (small_yaw_reference_captured) {
        /* 6020 反馈是 0..2pi 循环角，先取相对上电零点的最短角差，避免过零跳变。 */
        const float feedback_position = GimbalThreeCommandPosition(&gimbal_three_axis.small_yaw);
        return gimbal_three_small_yaw_zero +
               GimbalThreeWrapRadians(feedback_position - gimbal_three_small_yaw_zero);
    }
    return GimbalThreeSmallYawFeedbackPosition();
}

float GimbalThreeGetActualYawRad(void)
{
    const float small_yaw_delta_rad = GimbalThreeWrapRadians(
        GimbalThreeSmallYawTruePosition() - gimbal_three_small_yaw_zero);
    return (INS_Info.Yaw_Angle * 0.01745329252f) + small_yaw_delta_rad;
}

float GimbalThreeGetChassisYawDeltaRad(void)
{
    float big_yaw_delta_rad;
    float small_yaw_delta_rad;

    if (!big_yaw_reference_captured || !small_yaw_reference_captured) {
        return 0.0f;
    }

    big_yaw_delta_rad = GimbalThreeWrapRadians(
        GimbalThreeCommandPosition(&gimbal_three_axis.big_yaw) - big_yaw_zero);
    small_yaw_delta_rad = GimbalThreeWrapRadians(
        GimbalThreeSmallYawFollowPosition() - gimbal_three_small_yaw_zero);

    return GimbalThreeWrapRadians(big_yaw_delta_rad + small_yaw_delta_rad);
}

float GimbalThreeGetActualPitchRad(void)
{
    const float pitch_delta_rad = GimbalThreeCommandPosition(&gimbal_three_axis.pitch) -
                                  pitch_zero;
    return (INS_Info.Pitch_Angle * 0.01745329252f) + pitch_delta_rad;
}

static void GimbalThreeFreezeVisionTargets(void)
{
    if (small_yaw_reference_captured) {
        gimbal_three_small_yaw_reference = GimbalThreeSmallYawFeedbackPosition();
        gimbal_three_axis.small_yaw.setpoint.position = gimbal_three_small_yaw_reference;
    }
    if (pitch_reference_captured) {
        gimbal_three_pitch_reference = GimbalThreeCommandPosition(&gimbal_three_axis.pitch);
        gimbal_three_axis.pitch.setpoint.position = gimbal_three_pitch_reference;
    }
}

static float GimbalThreeConstrainVelocity(float value)
{
    return GimbalThreeConstrain(value, -GIMBAL_BIG_YAW_IMU_VELOCITY_MAX,
                                GIMBAL_BIG_YAW_IMU_VELOCITY_MAX);
}

static void GimbalThreeCaptureSpinHeading(void)
{
    /* 记录进入小陀螺瞬间的底盘航向，后续大 Yaw 反向补偿底盘旋转。 */
    gimbal_three_spin_heading_zero_deg = INS_Info.Yaw_Angle;
    gimbal_three_spin_heading_last_deg = INS_Info.Yaw_Angle;
}

static void GimbalThreePlaySwitchSound(void)
{
    uint8_t switch_state;

    if (remote_ctrl.rc_lost) {
        last_switch_state = -1;
        return;
    }

    switch_state = remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH];
    if (last_switch_state < 0) {
        last_switch_state = (int8_t)switch_state;
        return;
    }
    if (switch_state == (uint8_t)last_switch_state) return;

    if (switch_state == RC_SW_UP) {
        BSP_Buzzer_PlayTone(2200U, 140U);
    } else if (switch_state == RC_SW_MID) {
        BSP_Buzzer_PlayTone(3100U, 120U);
    } else if (switch_state == RC_SW_DOWN) {
        BSP_Buzzer_PlayTone(4000U, 120U);
    }
    last_switch_state = (int8_t)switch_state;
}

static void GimbalThreeSendMitMotor(RobotMotor_t *motor, bool *enabled, bool reference_captured,
                                    uint32_t *last_enable_ms)
{
    float feedback_position;

    if (motor->type != ROBOT_MOTOR_TYPE_DM4310 && motor->type != ROBOT_MOTOR_TYPE_EVO4310) return;

    if (!robot_command_armed) {
        if (*enabled) {
            if (motor->type == ROBOT_MOTOR_TYPE_DM4310) (void)RobotMotor_SendDmDisable(motor);
            else (void)RobotMotor_SendEvoDisable(motor);
        }
        *enabled = false;
        return;
    }

    /* 发送队列成功不等于电机已收到使能帧；无反馈时周期重发使能。 */
    if (!*enabled || !RobotMotor_HasFreshFeedback(motor, 100U)) {
        if (*last_enable_ms == 0U || (HAL_GetTick() - *last_enable_ms) >= 50U) {
            *enabled = motor->type == ROBOT_MOTOR_TYPE_DM4310 ?
                       RobotMotor_SendDmEnable(motor) : RobotMotor_SendEvoEnable(motor);
            *last_enable_ms = HAL_GetTick();
        }
        return;
    }

    if (!reference_captured) return;

    feedback_position = GimbalThreeCommandPosition(motor);
    if (motor != &gimbal_three_axis.big_yaw) {
        motor->setpoint.velocity = motor->setpoint.position - feedback_position;
    }
    motor->setpoint.torque = motor == &gimbal_three_axis.big_yaw ?
                             GIMBAL_BIG_YAW_MIT_TORQUE :
                             GIMBAL_THREE_DM4310_MIT_TORQUE;
    if (motor->type == ROBOT_MOTOR_TYPE_DM4310) {
        (void)RobotMotor_SendDmMit(motor, GIMBAL_THREE_DM4310_MIT_KP,
                                    GIMBAL_THREE_DM4310_MIT_KD);
    } else {
        (void)RobotMotor_SendEvoMit(motor, GIMBAL_THREE_DM4310_MIT_KP,
                                     GIMBAL_THREE_DM4310_MIT_KD);
    }
}

void GimbalPublish(void)
{
}

void GimbalInit(void)
{
    RobotMotor_Init(&gimbal_three_axis.big_yaw,
                    RobotMotorTypeFromProfile(GIMBAL_BIG_YAW_MOTOR_TYPE),
                    GIMBAL_BIG_YAW_CAN, GIMBAL_BIG_YAW_ID, GIMBAL_BIG_YAW_MASTER_ID,
                    GIMBAL_BIG_YAW_DIRECTION, GIMBAL_BIG_YAW_REDUCTION_RATIO,
                    GIMBAL_BIG_YAW_MODE);
    RobotMotor_Init(&gimbal_three_axis.small_yaw,
                    RobotMotorTypeFromProfile(GIMBAL_SMALL_YAW_MOTOR_TYPE),
                    GIMBAL_SMALL_YAW_CAN, GIMBAL_SMALL_YAW_ID, GIMBAL_SMALL_YAW_MASTER_ID,
                    GIMBAL_SMALL_YAW_DIRECTION, GIMBAL_SMALL_YAW_REDUCTION_RATIO,
                    GIMBAL_SMALL_YAW_MODE);
    RobotMotor_Init(&gimbal_three_axis.pitch,
                    RobotMotorTypeFromProfile(GIMBAL_THREE_PITCH_MOTOR_TYPE),
                    GIMBAL_THREE_PITCH_CAN, GIMBAL_THREE_PITCH_ID,
                    GIMBAL_THREE_PITCH_MASTER_ID, GIMBAL_THREE_PITCH_DIRECTION,
                    GIMBAL_THREE_PITCH_REDUCTION_RATIO, GIMBAL_THREE_PITCH_MODE);

    big_yaw_enabled = false;
    small_yaw_enabled = false;
    pitch_enabled = false;
    big_yaw_enable_last_ms = 0U;
    pitch_enable_last_ms = 0U;
    big_yaw_reference_captured = false;
    small_yaw_reference_captured = false;
    pitch_reference_captured = false;
    last_switch_state = -1;
    gimbal_three_previous_safe = true;
    gimbal_three_spin_heading_captured = false;
    gimbal_three_spin_heading_zero_deg = 0.0f;
    gimbal_three_spin_heading_last_deg = 0.0f;
    gimbal_three_spin_heading_accum_deg = 0.0f;
    extra_imu_yaw_reference_captured = false;
    extra_imu_relative_yaw_zero_deg = 0.0f;
    GimbalThreeInitSmallYawPids();
    gimbal_three_axis.mode = GIMBAL_MODE_SAFE;
}

void GimbalObserver(void)
{
}

void GimbalHandleException(void)
{
}

void GimbalSetMode(void)
{
    if (remote_ctrl.rc_lost || remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_UP) {
        if (!gimbal_three_previous_safe) {
            GimbalThreeDisableMitMotor(&gimbal_three_axis.big_yaw);
            GimbalThreeDisableMitMotor(&gimbal_three_axis.small_yaw);
            GimbalThreeDisableMitMotor(&gimbal_three_axis.pitch);
        }
        gimbal_three_axis.mode = GIMBAL_MODE_SAFE;
        RobotMotor_SetCommandArmed(false);
        VisionLink_SetControlSelected(false);
        big_yaw_enabled = false;
        small_yaw_enabled = false;
        pitch_enabled = false;
        big_yaw_enable_last_ms = 0U;
        pitch_enable_last_ms = 0U;
        big_yaw_reference_captured = false;
        small_yaw_reference_captured = false;
        pitch_reference_captured = false;
        gimbal_three_spin_heading_captured = false;
        gimbal_three_spin_heading_zero_deg = 0.0f;
        gimbal_three_spin_heading_last_deg = 0.0f;
        gimbal_three_spin_heading_accum_deg = 0.0f;
        extra_imu_yaw_reference_captured = false;
        GimbalThreeClearSmallYawPids();
        gimbal_three_previous_safe = true;
    } else {
        /* 小 Yaw/Pitch 不依赖 IMU，可在 IMU 尚未就绪时先完成编码器闭环。 */
        /* 下档只开启底盘小陀螺，云台继续保持遥控器手动控制；避免上位机默认
         * pitch=0 的视觉目标把 Pitch 拉低，同时保留 Pitch 摇杆调节能力。 */
        gimbal_three_axis.mode = GIMBAL_MODE_REMOTE;
        RobotMotor_SetCommandArmed(true);
        VisionLink_SetControlSelected(false);
        if (gimbal_three_previous_safe) {
            big_yaw_reference_captured = false;
            small_yaw_reference_captured = false;
            pitch_reference_captured = false;
            GimbalThreeClearSmallYawPids();
        }
        gimbal_three_previous_safe = false;
    }

    GimbalThreePlaySwitchSound();
}

void GimbalReference(void)
{
    float yaw_input;
    float pitch_input;

    if (gimbal_three_axis.mode == GIMBAL_MODE_SAFE) {
        gimbal_three_axis.big_yaw.setpoint.velocity = 0.0f;
        gimbal_three_axis.small_yaw.setpoint.velocity = 0.0f;
        gimbal_three_axis.pitch.setpoint.velocity = 0.0f;
        gimbal_three_axis.big_yaw.setpoint.torque = 0.0f;
        gimbal_three_axis.small_yaw.setpoint.torque = 0.0f;
        gimbal_three_axis.pitch.setpoint.torque = 0.0f;
        return;
    }

    if (!small_yaw_reference_captured && gimbal_three_axis.small_yaw.online) {
        gimbal_three_small_yaw_zero = GIMBAL_SMALL_YAW_INITIAL_POSITION_RAD;
        gimbal_three_small_yaw_reference = gimbal_three_small_yaw_zero;
        gimbal_three_axis.small_yaw.setpoint.position = gimbal_three_small_yaw_zero;
        small_yaw_reference_captured = true;
    }

#if (EXTRA_IMU_CONTROL_ENABLE != 0U) && (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    /* 外置 IMU 位于小 yaw 头部，记录其相对大 yaw 的初始航向。 */
    if (!extra_imu_yaw_reference_captured && ExtraImu_HasYawReference()) {
        extra_imu_relative_yaw_zero_deg = GimbalThreeWrapDegrees(
            extra_imu.yaw_deg - INS_Info.Yaw_Angle);
        extra_imu_yaw_reference_captured = true;
    }
#endif

    if (!pitch_reference_captured && gimbal_three_axis.pitch.online) {
        pitch_zero = GIMBAL_THREE_PITCH_INITIAL_POSITION_RAD;
        gimbal_three_pitch_reference = pitch_zero;
        gimbal_three_axis.pitch.setpoint.position = pitch_zero;
        pitch_reference_captured = true;
    }

    if (!big_yaw_reference_captured && gimbal_three_axis.big_yaw.online) {
        big_yaw_zero = GIMBAL_BIG_YAW_INITIAL_POSITION_RAD;
        gimbal_three_big_yaw_position_cmd = big_yaw_zero;
        big_yaw_imu_start_deg = INS_Info.Yaw_Angle;
        gimbal_three_imu_yaw_zero_deg = INS_Info.Yaw_Angle;
        gimbal_three_big_yaw_target_deg = big_yaw_imu_start_deg;
        gimbal_three_big_yaw_follow_target_deg = big_yaw_imu_start_deg;
        big_yaw_reference_captured = true;
    }

    if (remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN) {
        if (!gimbal_three_spin_heading_captured && big_yaw_reference_captured &&
            small_yaw_reference_captured) {
            GimbalThreeCaptureSpinHeading();
            gimbal_three_spin_heading_captured = true;
        }
    } else {
        gimbal_three_spin_heading_captured = false;
    }

    if (gimbal_three_axis.mode == GIMBAL_MODE_VISION) {
        VisionCommand_t vision_command;

        /* 自瞄 yaw/pitch 使用绝对姿态目标，协议单位为弧度。 */
        if (VisionLink_GetCommand(&vision_command) &&
            vision_command.mode >= VISION_MODE_CONTROL) {
            float desired_small_yaw_deg =
                GimbalThreeWrapDegrees(vision_command.yaw_rad * GIMBAL_THREE_RAD_TO_DEG -
                                       INS_Info.Yaw_Angle);
            float desired_pitch_deg = vision_command.pitch_rad * GIMBAL_THREE_RAD_TO_DEG;

            desired_small_yaw_deg *= EXTRA_IMU_YAW_DIRECTION;
            gimbal_three_small_yaw_reference = GimbalThreeConstrain(
                gimbal_three_small_yaw_zero + desired_small_yaw_deg / GIMBAL_THREE_RAD_TO_DEG,
                gimbal_three_small_yaw_zero + GIMBAL_SMALL_YAW_LOWER_LIMIT_RAD,
                gimbal_three_small_yaw_zero + GIMBAL_SMALL_YAW_UPPER_LIMIT_RAD);
            gimbal_three_axis.small_yaw.setpoint.position = gimbal_three_small_yaw_reference;
            gimbal_three_pitch_reference = GimbalThreeConstrain(
                pitch_zero + (desired_pitch_deg - INS_Info.Pitch_Angle) /
                GIMBAL_THREE_RAD_TO_DEG,
                pitch_zero + GIMBAL_THREE_PITCH_LOWER_LIMIT_RAD,
                pitch_zero + GIMBAL_THREE_PITCH_UPPER_LIMIT_RAD);
            gimbal_three_axis.pitch.setpoint.position = gimbal_three_pitch_reference;
        } else {
            /* 自瞄帧超时或 mode=0 时冻结当前姿态，不继续执行旧目标。 */
            GimbalThreeFreezeVisionTargets();
        }
    }

    yaw_input = (float)remote_ctrl.rc.ch[GIMBAL_THREE_RC_YAW_CHANNEL];
    pitch_input = (float)remote_ctrl.rc.ch[GIMBAL_THREE_RC_PITCH_CHANNEL];

    if (gimbal_three_axis.mode == GIMBAL_MODE_REMOTE && small_yaw_reference_captured &&
        (yaw_input > GIMBAL_THREE_RC_DEADBAND || yaw_input < -GIMBAL_THREE_RC_DEADBAND)) {
        gimbal_three_small_yaw_reference = GimbalThreeConstrain(
            gimbal_three_small_yaw_reference + yaw_input * GIMBAL_THREE_RC_YAW_INPUT_DIRECTION *
            GIMBAL_THREE_RC_YAW_RATE_RAD_S * GIMBAL_THREE_CONTROL_PERIOD_S / 660.0f,
            gimbal_three_small_yaw_zero + GIMBAL_SMALL_YAW_LOWER_LIMIT_RAD,
            gimbal_three_small_yaw_zero + GIMBAL_SMALL_YAW_UPPER_LIMIT_RAD);
        gimbal_three_axis.small_yaw.setpoint.position = gimbal_three_small_yaw_reference;
    }

    if (gimbal_three_axis.mode == GIMBAL_MODE_REMOTE && pitch_reference_captured &&
        (pitch_input > GIMBAL_THREE_RC_DEADBAND || pitch_input < -GIMBAL_THREE_RC_DEADBAND)) {
        gimbal_three_pitch_reference = GimbalThreeConstrain(
            gimbal_three_pitch_reference + pitch_input * GIMBAL_THREE_RC_PITCH_INPUT_DIRECTION *
            GIMBAL_THREE_RC_PITCH_RATE_RAD_S * GIMBAL_THREE_CONTROL_PERIOD_S / 660.0f,
            pitch_zero + GIMBAL_THREE_PITCH_LOWER_LIMIT_RAD,
            pitch_zero + GIMBAL_THREE_PITCH_UPPER_LIMIT_RAD);
        gimbal_three_axis.pitch.setpoint.position = gimbal_three_pitch_reference;
    }

    /* 大 Yaw 追踪小 Yaw 的期望偏移。目标经过限速器，避免两台电机同时突变。 */
    if (big_yaw_reference_captured && small_yaw_reference_captured) {
        /* 大 Yaw 补偿小 Yaw 的实际机械偏移，而不是遥控器期望值；这样即使
         * 6020 暂时卡住或跟不上，大 Yaw 也不会提前被期望值带走。 */
        float small_yaw_offset_rad = GimbalThreeSmallYawFollowPosition() -
                                     gimbal_three_small_yaw_zero;
        float spin_heading_offset_rad = -gimbal_three_spin_heading_accum_deg /
                                        GIMBAL_THREE_RAD_TO_DEG;
        float maximum_step = GIMBAL_BIG_YAW_FOLLOW_MAX_RATE_DEG_S *
                             GIMBAL_THREE_CONTROL_PERIOD_S;
        {
            const float previous_position_cmd = gimbal_three_big_yaw_position_cmd;
            const float desired_position = big_yaw_zero + (small_yaw_offset_rad + spin_heading_offset_rad) *
                GIMBAL_BIG_YAW_FOLLOW_RATIO * GIMBAL_BIG_YAW_FOLLOW_DIRECTION;
            const float encoder_target = GimbalThreeNearestAngle(
                desired_position, GimbalThreeCommandPosition(&gimbal_three_axis.big_yaw));
            const float encoder_step = maximum_step / GIMBAL_THREE_RAD_TO_DEG;
            float command_error = encoder_target - gimbal_three_big_yaw_position_cmd;

            command_error = GimbalThreeConstrain(command_error, -encoder_step, encoder_step);
            gimbal_three_big_yaw_position_cmd += command_error;
            gimbal_three_axis.big_yaw.setpoint.position = gimbal_three_big_yaw_position_cmd;
            gimbal_three_axis.big_yaw.setpoint.velocity = GimbalThreeConstrainVelocity(
                (gimbal_three_big_yaw_position_cmd - previous_position_cmd) /
                GIMBAL_THREE_CONTROL_PERIOD_S);
        }
    }
}

#if 0

        if (gimbal_three_spin_heading_captured &&
            remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN &&
            INS_IsImuReady() != 0U) {
            /* 用相邻采样的最短角差累计航向，跨越 +/-180 度时不发生 360 度跳变。 */
            const float heading_step_deg = GimbalThreeWrapDegrees(
                INS_Info.Yaw_Angle - gimbal_three_spin_heading_last_deg);
            gimbal_three_spin_heading_accum_deg += heading_step_deg;
            gimbal_three_spin_heading_last_deg = INS_Info.Yaw_Angle;
            spin_heading_offset_rad = -gimbal_three_spin_heading_accum_deg /
                                      GIMBAL_THREE_RAD_TO_DEG;
        }

        if (INS_IsImuReady() == 0U) {
            float encoder_target = big_yaw_zero + (small_yaw_offset_rad + spin_heading_offset_rad) *
                                   GIMBAL_BIG_YAW_FOLLOW_RATIO *
                                   GIMBAL_BIG_YAW_FOLLOW_DIRECTION;
            encoder_target = GimbalThreeNearestAngle(
                encoder_target, GimbalThreeCommandPosition(&gimbal_three_axis.big_yaw));
            float encoder_step = maximum_step / GIMBAL_THREE_RAD_TO_DEG;
            float encoder_error = encoder_target - gimbal_three_big_yaw_position_cmd;
            encoder_error = GimbalThreeConstrain(encoder_error, -encoder_step, encoder_step);
            gimbal_three_big_yaw_position_cmd += encoder_error;
            gimbal_three_axis.big_yaw.setpoint.position = gimbal_three_big_yaw_position_cmd;
            gimbal_three_axis.big_yaw.setpoint.velocity = GimbalThreeConstrainVelocity(
                encoder_error / GIMBAL_THREE_CONTROL_PERIOD_S);
            return;
        }

        gimbal_three_big_yaw_follow_target_deg = GimbalThreeWrapDegrees(
            big_yaw_imu_start_deg + (small_yaw_offset_rad + spin_heading_offset_rad) * GIMBAL_THREE_RAD_TO_DEG *
            GIMBAL_BIG_YAW_FOLLOW_RATIO * GIMBAL_BIG_YAW_FOLLOW_DIRECTION);
        gimbal_three_big_yaw_target_deg = GimbalThreeMoveTowardsDegrees(
            gimbal_three_big_yaw_target_deg, gimbal_three_big_yaw_follow_target_deg, maximum_step);
        gimbal_three_big_yaw_error_deg = GimbalThreeWrapDegrees(
            gimbal_three_big_yaw_target_deg - INS_Info.Yaw_Angle);
        position_offset = GimbalThreeConstrain(
            gimbal_three_big_yaw_error_deg * GIMBAL_BIG_YAW_IMU_OUTER_KP,
            -GIMBAL_BIG_YAW_IMU_MAX_POSITION_OFFSET,
            GIMBAL_BIG_YAW_IMU_MAX_POSITION_OFFSET);
        /* 使用上电绝对位置作为基准，不能每帧加当前反馈位置，否则云台会被底盘带走。 */
        {
            const float previous_position_cmd = gimbal_three_big_yaw_position_cmd;
            const float desired_position = big_yaw_zero + (small_yaw_offset_rad + spin_heading_offset_rad) *
                GIMBAL_BIG_YAW_FOLLOW_RATIO * GIMBAL_BIG_YAW_FOLLOW_DIRECTION +
                position_offset;
            gimbal_three_big_yaw_position_cmd = GimbalThreeNearestAngle(
                desired_position, GimbalThreeCommandPosition(&gimbal_three_axis.big_yaw));
            gimbal_three_axis.big_yaw.setpoint.velocity = GimbalThreeConstrainVelocity(
                (gimbal_three_big_yaw_position_cmd - previous_position_cmd) /
                GIMBAL_THREE_CONTROL_PERIOD_S);
        }
        gimbal_three_axis.big_yaw.setpoint.position = gimbal_three_big_yaw_position_cmd;
        /* 大 yaw 使用 MIT 位置/阻尼环，并加入目标位置变化前馈，减少跟随滞后。 */
#endif

void GimbalConsole(void)
{
    if (gimbal_three_axis.mode == GIMBAL_MODE_SAFE || !robot_command_armed) {
        gimbal_three_axis.small_yaw.setpoint.current = 0.0f;
        GimbalThreeClearSmallYawPids();
        return;
    }

    /* 小 Yaw 使用 GM6020 编码器闭环。目标与反馈均是逻辑坐标，发送函数
     * 会统一处理电机方向；位置误差做一圈环绕，避免跨 0/2pi 时突然满输出。 */
    if (GimbalThreeMotorUsesDji(&gimbal_three_axis.small_yaw) &&
        small_yaw_reference_captured) {
        float feedback_position = GimbalThreeSmallYawFeedbackPosition();
        float position_error = GimbalThreeWrapRadians(
            gimbal_three_axis.small_yaw.setpoint.position -
            feedback_position);

#if 0 /* 外置 IMU 已直接作为小 yaw 实际角度反馈，不再叠加误差修正 */
        if (extra_imu_yaw_reference_captured && ExtraImu_HasYawReference()) {
            const float relative_yaw_deg = GimbalThreeWrapDegrees(
                (extra_imu.yaw_deg - INS_Info.Yaw_Angle) - extra_imu_relative_yaw_zero_deg);
            const float encoder_yaw_deg =
                (feedback_position - gimbal_three_small_yaw_zero) * 57.2957795f;
            const float imu_encoder_error_rad = GimbalThreeConstrain(
                (relative_yaw_deg - encoder_yaw_deg) * 0.01745329252f *
                EXTRA_IMU_SMALL_YAW_OUTER_KP,
                -EXTRA_IMU_SMALL_YAW_OUTER_MAX_RAD,
                EXTRA_IMU_SMALL_YAW_OUTER_MAX_RAD);
            /* IMU 是慢速外环，6020 编码器仍是快速内环和限位依据。 */
            position_error -= imu_encoder_error_rad;
        }
#endif
        gimbal_three_axis.small_yaw.setpoint.velocity = PID_Calculate(
            &small_yaw_angle_pid, 0.0f, -position_error);
        gimbal_three_axis.small_yaw.setpoint.current = PID_Calculate(
            &small_yaw_velocity_pid,
            gimbal_three_axis.small_yaw.setpoint.velocity,
            gimbal_three_axis.small_yaw.feedback.velocity);
    } else {
        gimbal_three_axis.small_yaw.setpoint.current = 0.0f;
    }
}

void GimbalSendCmd(void)
{
    RobotMotor_t *dji_motors[1] = {&gimbal_three_axis.small_yaw};

    (void)RobotMotor_SendDjiCurrents(dji_motors, 1U);
    GimbalThreeSendMitMotor(&gimbal_three_axis.big_yaw, &big_yaw_enabled,
                             big_yaw_reference_captured, &big_yaw_enable_last_ms);
    GimbalThreeSendMitMotor(&gimbal_three_axis.small_yaw, &small_yaw_enabled,
                             small_yaw_reference_captured, 0);
    GimbalThreeSendMitMotor(&gimbal_three_axis.pitch, &pitch_enabled,
                             pitch_reference_captured, &pitch_enable_last_ms);
}

void GimbalSetReference(float yaw_position, float pitch_position)
{
    gimbal_three_small_yaw_reference = yaw_position;
    gimbal_three_pitch_reference = pitch_position;
    gimbal_three_axis.small_yaw.setpoint.position = yaw_position;
    gimbal_three_axis.pitch.setpoint.position = pitch_position;
}

#endif /* GIMBAL_BIG_YAW_SMALL_YAW_PITCH */
