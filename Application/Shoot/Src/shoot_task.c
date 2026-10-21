#include "shoot_task.h"

#include "cmsis_os.h"
#include "robot_param.h"
#include "shoot_fric.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

void Shoot_Task(void const *argument)
{
    (void)argument;
    osDelay(200U);
    ShootFricInit();

    for (;;) {
        ShootFricObserver();
        ShootFricSetMode();
        ShootFricReference();
        ShootFricConsole();
        ShootFricSendCmd();
        osDelay(1U);
    }
}

#endif /* SHOOT_FRIC_TRIGGER */
