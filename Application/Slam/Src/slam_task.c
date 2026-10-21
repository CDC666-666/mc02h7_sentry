#include "slam_task.h"

#include "cmsis_os.h"
#include "slam.h"

/* 定位任务只完成协议解析与在线状态维护，绝不写入底盘或电机目标。 */
void Slam_Task(void const *argument)
{
    (void)argument;

    for (;;) {
        Slam_ProcessRx();
        Slam_Update();
        osDelay(1U);
    }
}
