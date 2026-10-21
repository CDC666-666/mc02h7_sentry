#include "vision_link.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "gimbal.h"
#include "INS_Task.h"
#include "Quaternion.h"
#include "robot_param.h"

#define VISION_RX_RING_SIZE 1024U
#define VISION_RX_RING_MASK (VISION_RX_RING_SIZE - 1U)

typedef enum {
    VISION_RX_WAIT_HEAD_0 = 0U,
    VISION_RX_WAIT_HEAD_1,
    VISION_RX_READ_FRAME,
} VisionRxState_e;

volatile VisionLinkStats_t vision_link_stats = {0};

static volatile uint8_t vision_rx_ring[VISION_RX_RING_SIZE];
static volatile uint16_t vision_rx_head;
static volatile uint16_t vision_rx_tail;
static VisionRxState_e vision_rx_state;
static uint8_t vision_rx_frame[VISION_TO_GIMBAL_FRAME_SIZE];
static uint8_t vision_rx_count;
static VisionCommand_t vision_command;
static volatile uint8_t vision_control_selected;
static uint8_t vision_tx_frame[GIMBAL_TO_VISION_FRAME_SIZE];

static void VisionProtocol_PutFloat(uint8_t *destination, float value)
{
    memcpy(destination, &value, sizeof(value));
}

static float VisionProtocol_GetFloat(const uint8_t *source)
{
    float value;

    memcpy(&value, source, sizeof(value));
    return value;
}

static uint16_t VisionProtocol_GetU16(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8U);
}

static void VisionProtocol_PutU16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)(value >> 8U);
}

uint16_t VisionProtocol_Crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length-- > 0U) {
        uint8_t bit;

        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static bool VisionLink_CommandIsSane(const VisionCommand_t *command)
{
    return command != 0 && command->mode <= VISION_MODE_CONTROL_AND_FIRE &&
           isfinite(command->yaw_rad) && isfinite(command->yaw_velocity_rad_s) &&
           isfinite(command->yaw_acceleration_rad_s2) && isfinite(command->pitch_rad) &&
           isfinite(command->pitch_velocity_rad_s) && isfinite(command->pitch_acceleration_rad_s2) &&
           fabsf(command->yaw_rad) <= VISION_MAX_ABS_YAW_RAD &&
           fabsf(command->pitch_rad) <= VISION_MAX_ABS_PITCH_RAD &&
           fabsf(command->yaw_velocity_rad_s) <= VISION_MAX_ABS_RATE_RAD_S &&
           fabsf(command->pitch_velocity_rad_s) <= VISION_MAX_ABS_RATE_RAD_S &&
           fabsf(command->yaw_acceleration_rad_s2) <= VISION_MAX_ABS_ACCEL_RAD_S2 &&
           fabsf(command->pitch_acceleration_rad_s2) <= VISION_MAX_ABS_ACCEL_RAD_S2;
}

static void VisionLink_AcceptFrame(void)
{
    VisionCommand_t command = {0};
    uint16_t received_crc;
    uint16_t expected_crc;

    received_crc = VisionProtocol_GetU16(&vision_rx_frame[VISION_TO_GIMBAL_FRAME_SIZE - 2U]);
    expected_crc = VisionProtocol_Crc16(vision_rx_frame, VISION_TO_GIMBAL_FRAME_SIZE - 2U);
    if (received_crc != expected_crc) {
        vision_link_stats.crc_errors++;
        return;
    }

    command.mode = vision_rx_frame[2];
    command.yaw_rad = VisionProtocol_GetFloat(&vision_rx_frame[3]);
    command.yaw_velocity_rad_s = VisionProtocol_GetFloat(&vision_rx_frame[7]);
    command.yaw_acceleration_rad_s2 = VisionProtocol_GetFloat(&vision_rx_frame[11]);
    command.pitch_rad = VisionProtocol_GetFloat(&vision_rx_frame[15]);
    command.pitch_velocity_rad_s = VisionProtocol_GetFloat(&vision_rx_frame[19]);
    command.pitch_acceleration_rad_s2 = VisionProtocol_GetFloat(&vision_rx_frame[23]);
    command.last_valid_ms = HAL_GetTick();

    if (!VisionLink_CommandIsSane(&command)) {
        vision_link_stats.format_errors++;
        return;
    }

    taskENTER_CRITICAL();
    vision_command = command;
    taskEXIT_CRITICAL();
    vision_link_stats.valid_frames++;
}

void VisionLink_Init(void)
{
    taskENTER_CRITICAL();
    memset((void *)&vision_link_stats, 0, sizeof(vision_link_stats));
    memset((void *)vision_rx_ring, 0, sizeof(vision_rx_ring));
    memset(&vision_rx_frame, 0, sizeof(vision_rx_frame));
    memset(&vision_command, 0, sizeof(vision_command));
    vision_rx_head = 0U;
    vision_rx_tail = 0U;
    vision_rx_state = VISION_RX_WAIT_HEAD_0;
    vision_rx_count = 0U;
    vision_control_selected = 0U;
    taskEXIT_CRITICAL();
}

void VisionLink_OnUsbRx(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == 0) return;
    for (index = 0U; index < length; index++) {
        uint16_t next_head = (uint16_t)((vision_rx_head + 1U) & VISION_RX_RING_MASK);

        if (next_head == vision_rx_tail) {
            vision_link_stats.rx_overflow++;
            continue;
        }
        vision_rx_ring[vision_rx_head] = data[index];
        vision_rx_head = next_head;
        vision_link_stats.rx_bytes++;
    }
}

void VisionLink_ProcessRx(void)
{
    while (vision_rx_tail != vision_rx_head) {
        uint8_t byte = vision_rx_ring[vision_rx_tail];

        vision_rx_tail = (uint16_t)((vision_rx_tail + 1U) & VISION_RX_RING_MASK);
        if (vision_rx_state == VISION_RX_WAIT_HEAD_0) {
            if (byte == VISION_FRAME_HEAD_0) vision_rx_state = VISION_RX_WAIT_HEAD_1;
            continue;
        }
        if (vision_rx_state == VISION_RX_WAIT_HEAD_1) {
            if (byte == VISION_FRAME_HEAD_1) {
                vision_rx_frame[0] = VISION_FRAME_HEAD_0;
                vision_rx_frame[1] = VISION_FRAME_HEAD_1;
                vision_rx_count = 2U;
                vision_rx_state = VISION_RX_READ_FRAME;
            } else {
                vision_rx_state = byte == VISION_FRAME_HEAD_0 ? VISION_RX_WAIT_HEAD_1 : VISION_RX_WAIT_HEAD_0;
            }
            continue;
        }

        /* 上位机会先发裸 SP 唤醒。若其后立即开始一帧 SP，则丢弃该唤醒并重新同步。 */
        if (vision_rx_count == 2U && byte == VISION_FRAME_HEAD_0) {
            vision_rx_frame[0] = VISION_FRAME_HEAD_0;
            vision_rx_state = VISION_RX_WAIT_HEAD_1;
            continue;
        }
        vision_rx_frame[vision_rx_count++] = byte;
        if (vision_rx_count >= VISION_TO_GIMBAL_FRAME_SIZE) {
            VisionLink_AcceptFrame();
            vision_rx_count = 0U;
            vision_rx_state = VISION_RX_WAIT_HEAD_0;
        }
    }
}

bool VisionLink_GetCommand(VisionCommand_t *command)
{
    VisionCommand_t snapshot;

    if (command == 0) return false;
    taskENTER_CRITICAL();
    snapshot = vision_command;
    taskEXIT_CRITICAL();
    *command = snapshot;
    return snapshot.last_valid_ms != 0U &&
           (HAL_GetTick() - snapshot.last_valid_ms <= VISION_LINK_TIMEOUT_MS);
}

bool VisionLink_IsFirePermitted(void)
{
    VisionCommand_t command;

    return VisionLink_GetCommand(&command) && command.mode == VISION_MODE_CONTROL_AND_FIRE;
}

void VisionLink_SetControlSelected(bool selected)
{
    vision_control_selected = selected ? 1U : 0U;
}

bool VisionLink_IsControlSelected(void)
{
    return vision_control_selected != 0U;
}

void VisionLink_SendGimbalState(void)
{
    VisionCommand_t command;
    float yaw_rad;
    float pitch_rad;
    float yaw_velocity_rad_s = INS_Info.Yaw_Gyro * DegreesToRadians;
    float pitch_velocity_rad_s = INS_Info.Pitch_Gyro * DegreesToRadians;
    uint16_t crc;
    uint8_t robot_mode = VISION_MODE_IDLE;

#if (GIMBAL_TYPE == GIMBAL_BIG_YAW_SMALL_YAW_PITCH)
    yaw_rad = GimbalThreeGetActualYawRad();
    pitch_rad = GimbalThreeGetActualPitchRad();
#else
    yaw_rad = INS_Info.Yaw_Angle * DegreesToRadians;
    pitch_rad = INS_Info.Pitch_Angle * DegreesToRadians;
#endif

    if (VisionLink_IsControlSelected() && VisionLink_GetCommand(&command) &&
        command.mode >= VISION_MODE_CONTROL) {
        robot_mode = VISION_MODE_CONTROL;
    }

    vision_tx_frame[0] = VISION_FRAME_HEAD_0;
    vision_tx_frame[1] = VISION_FRAME_HEAD_1;
    vision_tx_frame[2] = robot_mode;
    VisionProtocol_PutFloat(&vision_tx_frame[3], Quaternion_Info.quat[0]);
    VisionProtocol_PutFloat(&vision_tx_frame[7], Quaternion_Info.quat[1]);
    VisionProtocol_PutFloat(&vision_tx_frame[11], Quaternion_Info.quat[2]);
    VisionProtocol_PutFloat(&vision_tx_frame[15], Quaternion_Info.quat[3]);
    VisionProtocol_PutFloat(&vision_tx_frame[19], yaw_rad);
    VisionProtocol_PutFloat(&vision_tx_frame[23], yaw_velocity_rad_s);
    VisionProtocol_PutFloat(&vision_tx_frame[27], pitch_rad);
    VisionProtocol_PutFloat(&vision_tx_frame[31], pitch_velocity_rad_s);
    VisionProtocol_PutFloat(&vision_tx_frame[35], 0.0f);
    VisionProtocol_PutU16(&vision_tx_frame[39], 0U);
    crc = VisionProtocol_Crc16(vision_tx_frame, GIMBAL_TO_VISION_FRAME_SIZE - 2U);
    VisionProtocol_PutU16(&vision_tx_frame[41], crc);

    if (CDC_Transmit_HS(vision_tx_frame, GIMBAL_TO_VISION_FRAME_SIZE) == USBD_OK) {
        vision_link_stats.tx_frames++;
    } else {
        vision_link_stats.tx_dropped++;
    }
}
