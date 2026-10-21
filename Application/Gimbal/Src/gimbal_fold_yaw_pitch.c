#include "gimbal.h"

#include "gimbal_fold_yaw_pitch.h"
#include "robot_param.h"

#if (GIMBAL_TYPE == GIMBAL_FOLD_YAW_PITCH)

/* 折叠轴 -> yaw -> pitch。三个 DM4310 的实际映射和限位尚未填写。 */
FoldGimbal_t fold_gimbal = {0};

void GimbalPublish(void)
{
}

void GimbalInit(void)
{
    fold_gimbal.mode = FOLD_GIMBAL_LOCKED;
    fold_gimbal.error_code = 1U; /* 未配置硬件。 */
    FoldGimbalLock();
}

void GimbalObserver(void)
{
    /* TODO: 刷新折叠、yaw、pitch 三台 DM4310 的反馈和折叠限位状态。 */
}

void GimbalHandleException(void)
{
    /* TODO: 反馈超时、折叠机械限位和 DM 故障状态保护。 */
}

void GimbalSetMode(void)
{
    /* 默认锁定。折叠轴未展开到位时 yaw/pitch 必须保持失能。 */
    FoldGimbalLock();
}

void GimbalReference(void)
{
    /* TODO: 根据遥控器挡位请求展开/收回；展开到位后捕获 yaw/pitch 当前角度。 */
}

void GimbalConsole(void)
{
    /* TODO: 完成折叠轴、yaw、pitch 的位置/速度目标与限位约束。 */
}

void GimbalSendCmd(void)
{
    /* TODO: 仅在折叠完成且三台电机均确认在线后，依次发送 DM MIT 使能和控制帧。 */
}

void FoldGimbalRequestOpen(void)
{
    /* TODO: 加入遥控器授权和机械限位确认后切换至 FOLD_GIMBAL_OPENING。 */
}

void FoldGimbalRequestClose(void)
{
    /* TODO: 先失能 yaw/pitch，再切换至 FOLD_GIMBAL_CLOSING。 */
}

void FoldGimbalLock(void)
{
    fold_gimbal.fold_target_rad = 0.0f;
    fold_gimbal.yaw_target_rad = 0.0f;
    fold_gimbal.pitch_target_rad = 0.0f;
    fold_gimbal.reference_captured = 0U;
    fold_gimbal.mode = FOLD_GIMBAL_LOCKED;
}

#endif /* GIMBAL_TYPE == GIMBAL_FOLD_YAW_PITCH */
