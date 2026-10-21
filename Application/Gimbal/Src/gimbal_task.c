#include "gimbal_task.h"

#include "cmsis_os.h"
#include "gimbal.h"

#if (GIMBAL_TYPE != GIMBAL_NONE)

void Gimbal_Task(void const *argument)
{
    (void)argument;
    GimbalPublish();
    GimbalInit();
    for (;;) {
        GimbalObserver();
        GimbalHandleException();
        GimbalSetMode();
        GimbalReference();
        GimbalConsole();
        GimbalSendCmd();
        osDelay(1U);
    }
}

#endif /* GIMBAL_TYPE */
