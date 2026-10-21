#include "chassis.h"

#include "chassis_steering.h"
#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

/*
 * 四舵轮框架。每个舵轮使用一个 M3508 舵向电机和一个 M3508 驱动电机。
 * 这里不初始化电机、不发 CAN 命令，直到 CAN/ID、方向、机械零点和尺寸完成确认。
 */
SteeringChassis_t steering_chassis = {0};

void ChassisPublish(void)
{
}

void ChassisInit(void)
{
    steering_chassis.mode = STEERING_CHASSIS_SAFE;
    steering_chassis.error_code = 1U; /* 未配置硬件。 */
    SteeringChassisLock();
}

void ChassisObserver(void)
{
    /* TODO: 读取八个 M3508 反馈，更新舵向角与驱动轮线速度。 */
}

void ChassisHandleException(void)
{
    /* TODO: 远控离线、八电机反馈超时和舵向机械限位保护。 */
}

void ChassisSetMode(void)
{
    /* 默认安全锁定。确认遥控器挡位和急停逻辑后再开放 ACTIVE。 */
    SteeringChassisLock();
}

void ChassisReference(void)
{
    /* TODO: 将遥控器/导航 vx、vy、wz 写入 SteeringChassisSetReference。 */
}

void ChassisConsole(void)
{
    /*
     * TODO:
     * 1. 速度矢量分解至四个轮心；
     * 2. atan2 得到目标舵向角；
     * 3. 角度误差超过 90 度时反转驱动速度；
     * 4. 舵向角 PID 和驱动速度 PID 输出电流。
     */
}

void ChassisSendCmd(void)
{
    /* TODO: 依照最终 CAN 总线和 ID 分组发送两组 DJI 电流帧。 */
}

void SteeringChassisSetReference(float vx_mps, float vy_mps, float wz_rad_s)
{
    steering_chassis.vx_reference_mps = vx_mps;
    steering_chassis.vy_reference_mps = vy_mps;
    steering_chassis.wz_reference_rad_s = wz_rad_s;
}

void SteeringChassisLock(void)
{
    SteeringChassisSetReference(0.0f, 0.0f, 0.0f);
    for (uint8_t i = 0U; i < STEERING_WHEEL_COUNT; ++i) {
        steering_chassis.wheel[i].steering_current = 0.0f;
        steering_chassis.wheel[i].drive_current = 0.0f;
        steering_chassis.wheel[i].drive_target_mps = 0.0f;
    }
    steering_chassis.mode = STEERING_CHASSIS_SAFE;
}

#endif /* CHASSIS_TYPE == CHASSIS_STEERING_WHEEL */
