#include "vision_task.h"

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "robot_param.h"
#include "vision_link.h"

void Vision_Task(void const *argument)
{
    uint32_t last_tx_ms = 0U;

    (void)argument;
    VisionLink_Init();
    for (;;) {
        VisionLink_ProcessRx();
        if (HAL_GetTick() - last_tx_ms >= VISION_STATE_PERIOD_MS) {
            last_tx_ms = HAL_GetTick();
            VisionLink_SendGimbalState();
        }
        osDelay(1U);
    }
}
