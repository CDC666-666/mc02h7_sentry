#include "cmsis_os.h"

void CAN_Task(void const *argument)
{
    (void)argument;
    /* FDCAN reception is interrupt-driven. Motor commands belong to module tasks. */
    for (;;) osDelay(1U);
}
