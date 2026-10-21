#include "chassis_task.h"

#include "chassis.h"
#include "cmsis_os.h"

#if (CHASSIS_TYPE != CHASSIS_NONE)

void Chassis_Task(void const *argument)
{
    (void)argument;
    ChassisPublish();
    ChassisInit();
    for (;;) {
        ChassisObserver();
        ChassisHandleException();
        ChassisSetMode();
        ChassisReference();
        ChassisConsole();
        ChassisSendCmd();
        osDelay(1U);
    }
}

#endif /* CHASSIS_TYPE */
