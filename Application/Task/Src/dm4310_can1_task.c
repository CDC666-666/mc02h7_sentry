#include "cmsis_os.h"
#include "dm4310_can1.h"

void CAN_Task(void const *argument)
{
    (void)argument;

    /* StandardRobotpp-compatible: slave ID 0x01, FF ... FF FC. */
    DM4310_CAN1_Enable();

    for (;;) {
        /* The enabled motor receives a zero-force MIT command at 1 kHz. */
        DM4310_CAN1_SendMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
    }
}
