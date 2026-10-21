#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "robot_typedef.h"

/* USB CDC 通道模式统一定义，避免车型配置缺少常量时被预处理器当成 0。 */
#ifndef USB_CDC_MODE_TELEMETRY
#define USB_CDC_MODE_TELEMETRY 0U
#endif
#ifndef USB_CDC_MODE_VISION
#define USB_CDC_MODE_VISION    1U
#endif

/* 在这里选择当前机器人配置。 */
#define ROBOT_PROFILE_OMNI_H723_DIRECT      1U
#define ROBOT_PROFILE_OMNI_H723_THREE_AXIS  2U
#define ROBOT_PROFILE_MECANUM_INFANTRY       3U
#define ROBOT_PROFILE_STEERING_FOLD_H723      4U

#ifndef ROBOT_PROFILE
#define ROBOT_PROFILE ROBOT_PROFILE_OMNI_H723_THREE_AXIS
#endif

#if (ROBOT_PROFILE == ROBOT_PROFILE_OMNI_H723_DIRECT)
#include "robot_param_omni_h723_gimbal.h"
#elif (ROBOT_PROFILE == ROBOT_PROFILE_OMNI_H723_THREE_AXIS)
#include "robot_param_omni_h723_big_yaw_small_yaw_pitch.h"
#elif (ROBOT_PROFILE == ROBOT_PROFILE_MECANUM_INFANTRY)
#include "robot_param_mecanum_h723_infantry.h"
#elif (ROBOT_PROFILE == ROBOT_PROFILE_STEERING_FOLD_H723)
#include "robot_param_steer_fold_h723.h"
#else
#error "Unsupported robot profile"
#endif

/* Shared board-level defaults. A profile may override these before this point if needed. */
#ifndef __DEVELOP
#define __DEVELOP 1U
#endif

#ifndef __DEBUG
#define __DEBUG 0U
#endif

#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_NONE
#endif

#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE GIMBAL_NONE
#endif

#ifndef SHOOT_TYPE
#define SHOOT_TYPE SHOOT_NONE
#endif

/* 顶部小 Yaw/Pitch 额外 IMU 选择。各机器人配置文件可覆盖。 */
#ifndef EXTRA_IMU_TYPE
#define EXTRA_IMU_TYPE EXTRA_IMU_NONE
#endif

/* 即使安装了额外 IMU，也默认只监测，不允许它直接改变云台控制量。 */
#ifndef EXTRA_IMU_CONTROL_ENABLE
#define EXTRA_IMU_CONTROL_ENABLE 0U
#endif

/* 额外 IMU 的硬件默认值；具体机器人配置文件可覆盖。 */
#ifndef EXTRA_IMU_CAN
#define EXTRA_IMU_CAN 0U
#endif

#ifndef EXTRA_IMU_CAN_ID
#define EXTRA_IMU_CAN_ID 0U
#endif

#ifndef EXTRA_IMU_MASTER_ID
#define EXTRA_IMU_MASTER_ID 0U
#endif

#ifndef EXTRA_IMU_TIMEOUT_MS
#define EXTRA_IMU_TIMEOUT_MS 200U
#endif

/* 额外 IMU 的官方 CAN 请求模式默认关闭；具体机器人配置可按需开启。 */
#ifndef EXTRA_IMU_REQUEST_MODE_ENABLE
#define EXTRA_IMU_REQUEST_MODE_ENABLE 0U
#endif

#ifndef EXTRA_IMU_REQUEST_PERIOD_MS
#define EXTRA_IMU_REQUEST_PERIOD_MS 10U
#endif

/* 遥测输出默认使用完整机器人数据流；带额外 IMU 的配置可覆盖这些宏。 */
#ifndef USB_TELEMETRY_STREAM_FULL
#define USB_TELEMETRY_STREAM_FULL 0U
#endif
#ifndef USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG
#define USB_TELEMETRY_STREAM_EXTRA_IMU_DEBUG 1U
#endif
#ifndef USB_TELEMETRY_STREAM_MODE
#define USB_TELEMETRY_STREAM_MODE USB_TELEMETRY_STREAM_FULL
#endif
#ifndef EXTRA_IMU_DEBUG_PERIOD_MS
#define EXTRA_IMU_DEBUG_PERIOD_MS 10U
#endif

/* 额外 IMU 的默认温飘配置。具体车辆配置可以覆盖这些宏。 */
#ifndef EXTRA_IMU_YAW_TEMP_BIAS_POINT_COUNT
#define EXTRA_IMU_YAW_TEMP_BIAS_POINT_COUNT 3U
#define EXTRA_IMU_YAW_TEMP_BIAS_TABLE \
    {25.0f, 0.0f},                    \
    {35.0f, 0.0f},                    \
    {45.0f, 0.0f}
#endif

#ifndef EXTRA_IMU_YAW_DIRECTION
#define EXTRA_IMU_YAW_DIRECTION 1.0f
#endif

#ifndef EXTRA_IMU_SMALL_YAW_OUTER_KP
#define EXTRA_IMU_SMALL_YAW_OUTER_KP 0.0f
#endif

#ifndef EXTRA_IMU_SMALL_YAW_OUTER_MAX_RAD
#define EXTRA_IMU_SMALL_YAW_OUTER_MAX_RAD 0.0f
#endif

#ifndef CONTROL_TYPE
#define CONTROL_TYPE CHASSIS_AND_GIMBAL
#endif

#endif /* ROBOT_PARAM_H */
