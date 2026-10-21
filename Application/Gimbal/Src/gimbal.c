#include "gimbal.h"
#include "vision_link.h"

#include "Bmi088.h"
#include "INS_Task.h"
#include "bsp_buzzer.h"
#include "robot_param.h"
#include "remote_control.h"

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)

Gimbal_t gimbal = {0};
static bool gimbal_yaw_dm_enabled = false;
static bool gimbal_pitch_dm_enabled = false;
static bool gimbal_yaw_reference_captured = false;
static bool gimbal_pitch_reference_captured = false;
static bool gimbal_imu_yaw_target_captured = false;
static bool gimbal_imu_pitch_target_captured = false;
static bool gimbal_yaw_input_was_active = false;
static int8_t gimbal_last_sound_switch = -1;
volatile float gimbal_imu_yaw_target_deg = GIMBAL_IMU_YAW_TARGET_DEG;
volatile float gimbal_imu_pitch_target_deg = GIMBAL_IMU_PITCH_TARGET_DEG;
volatile float gimbal_imu_yaw_error_deg = 0.0f;
volatile float gimbal_imu_pitch_error_deg = 0.0f;
volatile float gimbal_imu_yaw_motor_reference = GIMBAL_IMU_YAW_MOTOR_ZERO;
volatile float gimbal_imu_pitch_motor_reference = GIMBAL_IMU_PITCH_MOTOR_ZERO;
volatile float gimbal_imu_yaw_position_cmd = GIMBAL_IMU_YAW_MOTOR_ZERO;
volatile float gimbal_imu_pitch_position_cmd = GIMBAL_IMU_PITCH_MOTOR_ZERO;
volatile float gimbal_imu_yaw_zero_deg = 0.0f;
volatile float gimbal_imu_pitch_zero_deg = 0.0f;
volatile float gimbal_yaw_joint_position_rad = 0.0f;
volatile float gimbal_pitch_joint_position_rad = 0.0f;
volatile float gimbal_imu_pitch_lower_limit_deg = 0.0f;
volatile float gimbal_imu_pitch_upper_limit_deg = 0.0f;
volatile float gimbal_pitch_motor_lower_limit = 0.0f;
volatile float gimbal_pitch_motor_upper_limit = 0.0f;

#define GIMBAL_RAD_TO_DEG (57.2957795f)

static bool GimbalMotorUsesDji(const RobotMotor_t *motor)
{
    return motor->type == ROBOT_MOTOR_TYPE_DJI_GM6020;
}

static float GimbalMotorRelativePosition(const RobotMotor_t *motor, float middle)
{
    float relative = motor->feedback.position - middle;

    if (GimbalMotorUsesDji(motor)) {
        while (relative > 3.141592654f) relative -= 6.283185307f;
        while (relative < -3.141592654f) relative += 6.283185307f;
    }
    return relative * (float)motor->direction;
}

static void GimbalInitDjiPids(void)
{
    static float yaw_angle_pid[7] = {
        KP_GIMBAL_YAW_ANGLE, KI_GIMBAL_YAW_ANGLE, KD_GIMBAL_YAW_ANGLE, 0.0f, 0.0f,
        MAX_IOUT_GIMBAL_YAW_ANGLE, MAX_OUT_GIMBAL_YAW_ANGLE
    };
    static float yaw_velocity_pid[7] = {
        KP_GIMBAL_YAW_VELOCITY, KI_GIMBAL_YAW_VELOCITY, KD_GIMBAL_YAW_VELOCITY, 0.0f, 0.0f,
        MAX_IOUT_GIMBAL_YAW_VELOCITY, MAX_OUT_GIMBAL_YAW_VELOCITY
    };
    static float pitch_angle_pid[7] = {
        KP_GIMBAL_PITCH_ANGLE, KI_GIMBAL_PITCH_ANGLE, KD_GIMBAL_PITCH_ANGLE, 0.0f, 0.0f,
        MAX_IOUT_GIMBAL_PITCH_ANGLE, MAX_OUT_GIMBAL_PITCH_ANGLE
    };
    static float pitch_velocity_pid[7] = {
        KP_GIMBAL_PITCH_VELOCITY, KI_GIMBAL_PITCH_VELOCITY, KD_GIMBAL_PITCH_VELOCITY, 0.0f, 0.0f,
        MAX_IOUT_GIMBAL_PITCH_VELOCITY, MAX_OUT_GIMBAL_PITCH_VELOCITY
    };

    PID_Init(&gimbal.yaw_angle_pid, PID_POSITION, yaw_angle_pid);
    PID_Init(&gimbal.yaw_velocity_pid, PID_POSITION, yaw_velocity_pid);
    PID_Init(&gimbal.pitch_angle_pid, PID_POSITION, pitch_angle_pid);
    PID_Init(&gimbal.pitch_velocity_pid, PID_POSITION, pitch_velocity_pid);
}

static void GimbalClearDjiPids(void)
{
    gimbal.yaw_angle_pid.PID_Calc_Clear(&gimbal.yaw_angle_pid);
    gimbal.yaw_velocity_pid.PID_Calc_Clear(&gimbal.yaw_velocity_pid);
    gimbal.pitch_angle_pid.PID_Calc_Clear(&gimbal.pitch_angle_pid);
    gimbal.pitch_velocity_pid.PID_Calc_Clear(&gimbal.pitch_velocity_pid);
}

static float GimbalConstrain(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float GimbalWrapDegrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void GimbalUpdatePitchLimits(void)
{
    float mechanical_pitch_rad;
    float pitch_middle_setpoint;
    float first_limit_rad;
    float second_limit_rad;

    if (!gimbal.pitch.online) return;

    /* 对应 F407 的 Angle_solution()：imu_zero = imu_pitch - motor_delta。
     * 输出方向宏描述本 H7 机构中 IMU 到关节的符号关系；电机方向仍由
     * RobotMotor_SendDmMit() 统一处理。 */
    mechanical_pitch_rad = GIMBAL_IMU_PITCH_OUTPUT_DIRECTION *
                           GimbalMotorRelativePosition(&gimbal.pitch, GIMBAL_DIRECT_PITCH_MID);
    gimbal_pitch_joint_position_rad = mechanical_pitch_rad;
    gimbal_imu_pitch_zero_deg = INS_Info.Pitch_Angle - mechanical_pitch_rad * GIMBAL_RAD_TO_DEG;
    gimbal_imu_pitch_lower_limit_deg = gimbal_imu_pitch_zero_deg +
                                        (GIMBAL_LOWER_LIMIT_PITCH + GIMBAL_PITCH_SOFT_LIMIT_MARGIN) *
                                        GIMBAL_RAD_TO_DEG;
    gimbal_imu_pitch_upper_limit_deg = gimbal_imu_pitch_zero_deg +
                                        (GIMBAL_UPPER_LIMIT_PITCH - GIMBAL_PITCH_SOFT_LIMIT_MARGIN) *
                                        GIMBAL_RAD_TO_DEG;

    /* setpoint.position 先完成方向修正，RobotMotor_SendDmMit() 再应用
     * motor->direction，生成原始 DM MIT 控制帧。 */
    pitch_middle_setpoint = GIMBAL_DIRECT_PITCH_MID * (float)gimbal.pitch.direction;
    first_limit_rad = pitch_middle_setpoint +
                      GIMBAL_LOWER_LIMIT_PITCH * GIMBAL_IMU_PITCH_OUTPUT_DIRECTION;
    second_limit_rad = pitch_middle_setpoint +
                       GIMBAL_UPPER_LIMIT_PITCH * GIMBAL_IMU_PITCH_OUTPUT_DIRECTION;
    gimbal_pitch_motor_lower_limit = first_limit_rad < second_limit_rad ?
                                      first_limit_rad : second_limit_rad;
    gimbal_pitch_motor_upper_limit = first_limit_rad > second_limit_rad ?
                                      first_limit_rad : second_limit_rad;
}

static void GimbalUpdateYawZero(void)
{
    float mechanical_yaw_rad;

    if (!gimbal.yaw.online) return;

    mechanical_yaw_rad = GIMBAL_IMU_YAW_OUTPUT_DIRECTION *
                         GimbalMotorRelativePosition(&gimbal.yaw, GIMBAL_DIRECT_YAW_MID);
    gimbal_yaw_joint_position_rad = mechanical_yaw_rad;
    gimbal_imu_yaw_zero_deg = GimbalWrapDegrees(
        INS_Info.Yaw_Angle - mechanical_yaw_rad * GIMBAL_RAD_TO_DEG);
}

static void GimbalPlaySwitchSound(void)
{
    uint8_t switch_state;

    if (remote_ctrl.rc_lost) {
        gimbal_last_sound_switch = -1;
        return;
    }

    switch_state = remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH];
    if (gimbal_last_sound_switch < 0) {
        gimbal_last_sound_switch = (int8_t)switch_state;
        return;
    }
    if (switch_state == (uint8_t)gimbal_last_sound_switch) return;

    /* 不同音调对应不同的物理挡位。 */
    if (switch_state == RC_SW_UP) {
        BSP_Buzzer_PlayTone(2200U, 140U);
    } else if (switch_state == RC_SW_MID) {
        BSP_Buzzer_PlayTone(3100U, 120U);
    } else if (switch_state == RC_SW_DOWN) {
        BSP_Buzzer_PlayTone(4000U, 120U);
    }
    gimbal_last_sound_switch = (int8_t)switch_state;
}

static void GimbalSendMitMotor(RobotMotor_t *motor, bool *enabled, bool reference_captured)
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

    if (!*enabled) {
        *enabled = motor->type == ROBOT_MOTOR_TYPE_DM4310 ?
                   RobotMotor_SendDmEnable(motor) : RobotMotor_SendEvoEnable(motor);
        return;
    }

    if (!reference_captured) return;

    /* 与 F407 的 MIT 帧一致：速度项取位置误差，并加入固定力矩前馈。
     * 两者均使用与位置目标相同的方向修正坐标系。 */
    feedback_position = motor->feedback.position * (float)motor->direction;
    motor->setpoint.velocity = motor->setpoint.position - feedback_position;
    motor->setpoint.torque = GIMBAL_DM4310_MIT_TORQUE;
    if (motor->type == ROBOT_MOTOR_TYPE_DM4310) {
        (void)RobotMotor_SendDmMit(motor, GIMBAL_DM4310_MIT_KP, GIMBAL_DM4310_MIT_KD);
    } else {
        (void)RobotMotor_SendEvoMit(motor, GIMBAL_DM4310_MIT_KP, GIMBAL_DM4310_MIT_KD);
    }
}

void GimbalPublish(void)
{
}

void GimbalInit(void)
{
    RobotMotor_Init(&gimbal.yaw, RobotMotorTypeFromProfile(GIMBAL_DIRECT_YAW_MOTOR_TYPE),
                    GIMBAL_DIRECT_YAW_CAN, GIMBAL_DIRECT_YAW_ID, GIMBAL_DIRECT_YAW_MASTER_ID,
                    GIMBAL_DIRECT_YAW_DIRECTION, GIMBAL_DIRECT_YAW_REDUCTION_RATIO,
                    GIMBAL_DIRECT_YAW_MODE);
    RobotMotor_Init(&gimbal.pitch, RobotMotorTypeFromProfile(GIMBAL_DIRECT_PITCH_MOTOR_TYPE),
                    GIMBAL_DIRECT_PITCH_CAN, GIMBAL_DIRECT_PITCH_ID, GIMBAL_DIRECT_PITCH_MASTER_ID,
                    GIMBAL_DIRECT_PITCH_DIRECTION, GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,
                    GIMBAL_DIRECT_PITCH_MODE);
    GimbalInitDjiPids();
    gimbal_yaw_dm_enabled = false;
    gimbal_pitch_dm_enabled = false;
    gimbal_yaw_reference_captured = false;
    gimbal_pitch_reference_captured = false;
    gimbal_imu_yaw_target_captured = false;
    gimbal_imu_pitch_target_captured = false;
    gimbal_yaw_input_was_active = false;
    gimbal_imu_yaw_motor_reference = GIMBAL_IMU_YAW_MOTOR_ZERO;
    gimbal_imu_pitch_motor_reference = GIMBAL_IMU_PITCH_MOTOR_ZERO;
    gimbal_last_sound_switch = -1;
    gimbal.mode = GIMBAL_MODE_SAFE;
}

void GimbalObserver(void)
{
}

void GimbalHandleException(void)
{
}

void GimbalSetMode(void)
{
    if (remote_ctrl.rc_lost || remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_UP ||
        INS_IsImuReady() == 0U) {
        gimbal.mode = GIMBAL_MODE_SAFE;
        RobotMotor_SetCommandArmed(false);
        VisionLink_SetControlSelected(false);
        gimbal_yaw_reference_captured = false;
        gimbal_pitch_reference_captured = false;
        gimbal_imu_yaw_target_captured = false;
        gimbal_imu_pitch_target_captured = false;
        gimbal_yaw_input_was_active = false;
    } else if (remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_MID ||
               remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN) {
        gimbal.mode = remote_ctrl.rc.s[GIMBAL_RC_MODE_SWITCH] == RC_SW_DOWN ?
                      GIMBAL_MODE_VISION : GIMBAL_MODE_IMU;
        RobotMotor_SetCommandArmed(true);
        VisionLink_SetControlSelected(gimbal.mode == GIMBAL_MODE_VISION);
        /* 每个轴仅在收到自身反馈后捕获初始值。此前不能使用默认的零值限位窗口，
         * 否则会把当前 IMU 角度错误地钳制为 0 度命令。 */
        if (!gimbal_imu_yaw_target_captured && gimbal.yaw.online) {
            gimbal_imu_yaw_target_deg = INS_Info.Yaw_Angle;
            GimbalUpdateYawZero();
            gimbal_imu_yaw_target_captured = true;
        }
        if (!gimbal_imu_pitch_target_captured && gimbal.pitch.online) {
            GimbalUpdatePitchLimits();
            gimbal_imu_pitch_target_deg = GimbalConstrain(
                INS_Info.Pitch_Angle,
                gimbal_imu_pitch_lower_limit_deg,
                gimbal_imu_pitch_upper_limit_deg);
            gimbal_imu_pitch_target_captured = true;
        }
    } else {
        gimbal.mode = GIMBAL_MODE_SAFE;
        RobotMotor_SetCommandArmed(false);
        VisionLink_SetControlSelected(false);
        gimbal_imu_yaw_target_captured = false;
        gimbal_imu_pitch_target_captured = false;
        gimbal_yaw_input_was_active = false;
    }

    GimbalPlaySwitchSound();
}

void GimbalReference(void)
{
    if (gimbal.mode == GIMBAL_MODE_SAFE) {
        gimbal.yaw.setpoint.velocity = 0.0f;
        gimbal.pitch.setpoint.velocity = 0.0f;
        gimbal.yaw.setpoint.torque = 0.0f;
        gimbal.pitch.setpoint.torque = 0.0f;
        return;
    }

    GimbalUpdateYawZero();
    GimbalUpdatePitchLimits();

    /* 与 F407 一致：Pitch 限位定义在 IMU 坐标系中，每个控制周期根据电机反馈重新计算。 */
    if (gimbal.pitch.online && gimbal_imu_pitch_target_captured) {
        gimbal_imu_pitch_target_deg = GimbalConstrain(
            gimbal_imu_pitch_target_deg,
            gimbal_imu_pitch_lower_limit_deg,
            gimbal_imu_pitch_upper_limit_deg);
    }

    /* 与 F407 一致：工作挡位下的摇杆输入独立更新 IMU 参考，不依赖电机反馈是否已到达。 */
    if (gimbal.mode == GIMBAL_MODE_IMU) {
        float yaw_input = (float)remote_ctrl.rc.ch[GIMBAL_RC_YAW_CHANNEL];
        float pitch_input = (float)remote_ctrl.rc.ch[GIMBAL_RC_PITCH_CHANNEL];
        bool yaw_input_active = yaw_input > GIMBAL_RC_DEADBAND ||
                                yaw_input < -GIMBAL_RC_DEADBAND;
        bool pitch_input_active = pitch_input > GIMBAL_RC_DEADBAND ||
                                  pitch_input < -GIMBAL_RC_DEADBAND;

        /* 摇杆归中时锁定当前姿态，避免继续追逐上一段摇杆累积的旧目标。
         * 之后仍由角度/角速度环保持该位置；释放瞬间也能给速度环一个
         * 明确的零速目标用于刹车。 */
        if (gimbal_imu_yaw_target_captured &&
            gimbal_yaw_input_was_active && !yaw_input_active) {
            gimbal_imu_yaw_target_deg = INS_Info.Yaw_Angle;
            /* PID_Calculate() 在误差落入死区时会保留上一帧输出。
             * 归中必须清掉旧的角度/速度环状态，否则上一段摇杆输入
             * 的电流会继续发送，表现为松杆后仍然旋转。 */
            GimbalClearDjiPids();
            gimbal.yaw.setpoint.velocity = 0.0f;
            gimbal.yaw.setpoint.current = 0.0f;
        }

        if (gimbal_imu_yaw_target_captured && yaw_input_active) {
            gimbal_imu_yaw_target_deg = GimbalWrapDegrees(
                gimbal_imu_yaw_target_deg + yaw_input * GIMBAL_RC_YAW_INPUT_DIRECTION *
                GIMBAL_RC_TARGET_RATE_DEG_S * GIMBAL_RC_TARGET_UPDATE_S / 660.0f);
        }
        gimbal_yaw_input_was_active = yaw_input_active;
        if (gimbal_imu_pitch_target_captured && pitch_input_active) {
            float pitch_delta_deg = pitch_input * GIMBAL_RC_PITCH_INPUT_DIRECTION *
                                    GIMBAL_RC_TARGET_RATE_DEG_S *
                                    GIMBAL_RC_TARGET_UPDATE_S / 660.0f;

            /* 操作者向可用行程方向回推时立即丢弃旧目标，避免限位处积累的目标残留。
             * 这相当于位置目标在机械限位处的抗积分饱和处理。 */
            if ((gimbal_imu_pitch_target_deg <=
                 gimbal_imu_pitch_lower_limit_deg + GIMBAL_PITCH_LIMIT_REBASE_EPSILON_DEG &&
                 pitch_delta_deg > 0.0f) ||
                (gimbal_imu_pitch_target_deg >=
                 gimbal_imu_pitch_upper_limit_deg - GIMBAL_PITCH_LIMIT_REBASE_EPSILON_DEG &&
                 pitch_delta_deg < 0.0f)) {
                gimbal_imu_pitch_target_deg = GimbalConstrain(
                    INS_Info.Pitch_Angle,
                    gimbal_imu_pitch_lower_limit_deg,
                    gimbal_imu_pitch_upper_limit_deg);
            }
            gimbal_imu_pitch_target_deg = GimbalConstrain(
                gimbal_imu_pitch_target_deg + pitch_delta_deg,
                gimbal_imu_pitch_lower_limit_deg, gimbal_imu_pitch_upper_limit_deg);
        }

        /* IMU 未严格位于 Yaw/Pitch 轴交点时，Yaw 转动会投影到欧拉 Pitch。
         * 仅操作 Yaw 时跟随该表观 Pitch，使 Pitch 电机保持不动；一旦有真实 Pitch
         * 摇杆输入，立即恢复 Pitch 的 IMU 闭环控制。 */
        if (gimbal_imu_pitch_target_captured && yaw_input_active && !pitch_input_active) {
            gimbal_imu_pitch_target_deg = GimbalConstrain(
                INS_Info.Pitch_Angle,
                gimbal_imu_pitch_lower_limit_deg, gimbal_imu_pitch_upper_limit_deg);
        }
    } else if (gimbal.mode == GIMBAL_MODE_VISION) {
        gimbal_yaw_input_was_active = false;
        VisionCommand_t vision_command;

        if (VisionLink_GetCommand(&vision_command) && vision_command.mode >= VISION_MODE_CONTROL) {
            if (gimbal_imu_yaw_target_captured) {
                gimbal_imu_yaw_target_deg = GimbalWrapDegrees(
                    vision_command.yaw_rad * RadiansToDegrees);
            }
            if (gimbal_imu_pitch_target_captured) {
                gimbal_imu_pitch_target_deg = GimbalConstrain(
                    vision_command.pitch_rad * RadiansToDegrees,
                    gimbal_imu_pitch_lower_limit_deg, gimbal_imu_pitch_upper_limit_deg);
            }
        } else {
            /* 视觉掉线或 mode=0 时立即冻结到当前姿态，避免保留旧目标继续运动。 */
            if (gimbal_imu_yaw_target_captured) gimbal_imu_yaw_target_deg = INS_Info.Yaw_Angle;
            if (gimbal_imu_pitch_target_captured) gimbal_imu_pitch_target_deg = GimbalConstrain(
                INS_Info.Pitch_Angle, gimbal_imu_pitch_lower_limit_deg,
                gimbal_imu_pitch_upper_limit_deg);
        }
    }

    /* 两个轴独立完成首帧保持。首个 MIT 命令等于各自当前反馈位置，
     * 任一电机缺失反馈都不会阻塞或带动另一电机。 */
    if (!gimbal_yaw_reference_captured && gimbal.yaw.online &&
        BMI088_DMA_GyroFrameCount >= GIMBAL_IMU_READY_MIN_GYRO_FRAMES) {
        gimbal_imu_yaw_motor_reference =
            gimbal.yaw.feedback.position * (float)gimbal.yaw.direction;
        gimbal.yaw.setpoint.position = gimbal_imu_yaw_motor_reference;
        gimbal.yaw.setpoint.velocity = 0.0f;
        gimbal.yaw.setpoint.torque = 0.0f;
        gimbal_yaw_reference_captured = true;
    }

    if (!gimbal_pitch_reference_captured && gimbal.pitch.online &&
        BMI088_DMA_GyroFrameCount >= GIMBAL_IMU_READY_MIN_GYRO_FRAMES) {
        gimbal_imu_pitch_motor_reference =
            gimbal.pitch.feedback.position * (float)gimbal.pitch.direction;
        gimbal.pitch.setpoint.position = gimbal_imu_pitch_motor_reference;
        gimbal.pitch.setpoint.velocity = 0.0f;
        gimbal.pitch.setpoint.torque = 0.0f;
        gimbal_pitch_reference_captured = true;
    }

    if ((gimbal.yaw.type == ROBOT_MOTOR_TYPE_DM4310 ||
         gimbal.yaw.type == ROBOT_MOTOR_TYPE_EVO4310) &&
        gimbal_yaw_reference_captured && gimbal_imu_yaw_target_captured) {
        float yaw_position_offset;

        gimbal_imu_yaw_error_deg = GimbalWrapDegrees(gimbal_imu_yaw_target_deg - INS_Info.Yaw_Angle);
        /* F407 的 Angle_solution() 用于建立参考与限位坐标系，并非 MIT 绝对位置积分器。
         * F407 的串级电流环直接作用于当前姿态误差；此处等效的 DM4310 MIT 外环
         * 在实时反馈位置上叠加一个限幅修正量。 */
        yaw_position_offset = GimbalConstrain(
            gimbal_imu_yaw_error_deg * GIMBAL_IMU_YAW_OUTER_KP,
            -GIMBAL_IMU_YAW_MAX_POSITION_OFFSET,
            GIMBAL_IMU_YAW_MAX_POSITION_OFFSET);
        gimbal_imu_yaw_position_cmd =
            gimbal.yaw.feedback.position * (float)gimbal.yaw.direction +
            yaw_position_offset * GIMBAL_IMU_YAW_OUTPUT_DIRECTION;
        gimbal.yaw.setpoint.position = gimbal_imu_yaw_position_cmd;
        gimbal.yaw.setpoint.velocity = 0.0f;
    }

    if ((gimbal.pitch.type == ROBOT_MOTOR_TYPE_DM4310 ||
         gimbal.pitch.type == ROBOT_MOTOR_TYPE_EVO4310) &&
        gimbal_pitch_reference_captured && gimbal_imu_pitch_target_captured) {
        float pitch_position_offset;

        gimbal_imu_pitch_error_deg = gimbal_imu_pitch_target_deg - INS_Info.Pitch_Angle;
        pitch_position_offset = GimbalConstrain(
            gimbal_imu_pitch_error_deg * GIMBAL_IMU_PITCH_OUTER_KP,
            -GIMBAL_IMU_PITCH_MAX_POSITION_OFFSET,
            GIMBAL_IMU_PITCH_MAX_POSITION_OFFSET);
        gimbal_imu_pitch_position_cmd = GimbalConstrain(
            gimbal.pitch.feedback.position * (float)gimbal.pitch.direction +
            pitch_position_offset * GIMBAL_IMU_PITCH_OUTPUT_DIRECTION,
            gimbal_pitch_motor_lower_limit, gimbal_pitch_motor_upper_limit);
        gimbal.pitch.setpoint.position = gimbal_imu_pitch_position_cmd;
        gimbal.pitch.setpoint.velocity = 0.0f;
    }
}

void GimbalConsole(void)
{
    /* DM4310 的 MIT 目标已在参考阶段生成；GM6020 在此执行双 PID 闭环。 */
    if (gimbal.mode == GIMBAL_MODE_SAFE || !robot_command_armed) {
        gimbal.yaw.setpoint.current = 0.0f;
        gimbal.pitch.setpoint.current = 0.0f;
        GimbalClearDjiPids();
        return;
    }

    /* GM6020 先由姿态角环生成目标角速度，再由陀螺仪角速度环生成 DJI 电压命令。
     * DJI 帧打包器会统一处理 motor->direction，此处不能重复乘方向。 */
    if (GimbalMotorUsesDji(&gimbal.yaw) && gimbal_yaw_reference_captured &&
        gimbal_imu_yaw_target_captured) {
        float yaw_error = GimbalWrapDegrees(gimbal_imu_yaw_target_deg - INS_Info.Yaw_Angle);
        gimbal.yaw.setpoint.velocity = PID_Calculate(&gimbal.yaw_angle_pid, 0.0f, -yaw_error);
        /* 与 F407 稳定版本保持一致：6020 速度环使用 BMI088 角速度。
         * 这里的目标和反馈都使用度/秒，避免编码器量化噪声直接激发高频抖动。 */
        gimbal.yaw.setpoint.current = GIMBAL_IMU_YAW_OUTPUT_DIRECTION * PID_Calculate(
            &gimbal.yaw_velocity_pid, gimbal.yaw.setpoint.velocity,
            INS_Info.Yaw_Gyro);
    } else {
        gimbal.yaw.setpoint.current = 0.0f;
    }

    if (GimbalMotorUsesDji(&gimbal.pitch) && gimbal_pitch_reference_captured &&
        gimbal_imu_pitch_target_captured) {
        gimbal.pitch.setpoint.velocity = PID_Calculate(&gimbal.pitch_angle_pid,
                                                        gimbal_imu_pitch_target_deg,
                                                        INS_Info.Pitch_Angle);
        gimbal.pitch.setpoint.current = GIMBAL_IMU_PITCH_OUTPUT_DIRECTION * PID_Calculate(
            &gimbal.pitch_velocity_pid, gimbal.pitch.setpoint.velocity,
            INS_Info.Pitch_Gyro);
    } else {
        gimbal.pitch.setpoint.current = 0.0f;
    }
}

void GimbalSendCmd(void)
{
    RobotMotor_t *dji_motors[2] = {&gimbal.yaw, &gimbal.pitch};
    (void)RobotMotor_SendDjiCurrents(dji_motors, 2U);
    GimbalSendMitMotor(&gimbal.yaw, &gimbal_yaw_dm_enabled, gimbal_yaw_reference_captured);
    GimbalSendMitMotor(&gimbal.pitch, &gimbal_pitch_dm_enabled, gimbal_pitch_reference_captured);
}

void GimbalSetReference(float yaw_position, float pitch_position)
{
    gimbal.yaw.setpoint.position = yaw_position;
    gimbal.pitch.setpoint.position = pitch_position;
}

#endif /* GIMBAL_YAW_PITCH_DIRECT */
