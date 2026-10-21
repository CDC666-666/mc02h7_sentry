#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

/* SP 视觉串口协议的固定帧长度。传输使用字节数组显式编解码，避免结构体对齐差异。 */
#define VISION_FRAME_HEAD_0              0x53U
#define VISION_FRAME_HEAD_1              0x50U
#define VISION_TO_GIMBAL_FRAME_SIZE      29U
#define GIMBAL_TO_VISION_FRAME_SIZE      43U

#define VISION_MODE_IDLE                 0U
#define VISION_MODE_CONTROL              1U
#define VISION_MODE_CONTROL_AND_FIRE     2U

uint16_t VisionProtocol_Crc16(const uint8_t *data, uint32_t length);

#endif /* VISION_PROTOCOL_H */
