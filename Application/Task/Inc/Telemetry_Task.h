#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include <stdint.h>

extern volatile uint32_t Telemetry_FrameCount;
extern volatile uint32_t Telemetry_DroppedFrameCount;

void Telemetry_Task(void const *argument);

#endif /* TELEMETRY_TASK_H */
