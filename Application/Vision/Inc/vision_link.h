#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "vision_protocol.h"

typedef struct {
    uint8_t mode;
    float yaw_rad;
    float yaw_velocity_rad_s;
    float yaw_acceleration_rad_s2;
    float pitch_rad;
    float pitch_velocity_rad_s;
    float pitch_acceleration_rad_s2;
    uint32_t last_valid_ms;
} VisionCommand_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_overflow;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t tx_frames;
    uint32_t tx_dropped;
} VisionLinkStats_t;

extern volatile VisionLinkStats_t vision_link_stats;

void VisionLink_Init(void);
void VisionLink_OnUsbRx(const uint8_t *data, uint32_t length);
void VisionLink_ProcessRx(void);
void VisionLink_SendGimbalState(void);
bool VisionLink_GetCommand(VisionCommand_t *command);
bool VisionLink_IsFirePermitted(void);
void VisionLink_SetControlSelected(bool selected);
bool VisionLink_IsControlSelected(void);

#endif /* VISION_LINK_H */
