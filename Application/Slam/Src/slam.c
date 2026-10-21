#include "slam.h"

#include <string.h>

#include "stm32h7xx_hal.h"

#define SLAM_FRAME_HEAD             0xAAU
#define SLAM_FRAME_TAIL             0x0AU
#define SLAM_FRAME_SIZE_14          14U
#define SLAM_FRAME_SIZE_15          15U
#define SLAM_RX_RING_SIZE           256U
#define SLAM_ONLINE_TIMEOUT_MS      200U
/* 实物标定：原始位置变化 1000 对应实际移动约 0.1 m，即 0.1 mm/计数。 */
#define SLAM_POSITION_SCALE_M       0.0001f
/* 实机验证 Y 轴显示仍比实际大 10 倍，因此 Y 轴单独缩放为 0.01 mm/计数。 */
#define SLAM_Y_POSITION_SCALE_M     0.00001f

static uint8_t slam_rx_ring[SLAM_RX_RING_SIZE];
static volatile uint16_t slam_rx_write_index;
static volatile uint16_t slam_rx_read_index;
static uint8_t slam_frame[SLAM_RAW_FRAME_MAX_SIZE];
static uint8_t slam_frame_index;
static SlamPose_t slam_pose;

static int16_t Slam_ReadInt16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

static void Slam_PublishFrame(const uint8_t *frame, uint8_t frame_length)
{
    SlamPose_t next_pose;
    uint8_t payload_offset;

    if ((frame == 0) || (frame[0] != SLAM_FRAME_HEAD) ||
        ((frame_length != SLAM_FRAME_SIZE_14) && (frame_length != SLAM_FRAME_SIZE_15)) ||
        (frame[frame_length - 1U] != SLAM_FRAME_TAIL)) {
        slam_pose.format_errors++;
        return;
    }

    /* 已观测到实际模块稳定输出 15 字节帧。
     * 常见扩展格式为 AA 55 + 12 字节位姿 + 0A；若额外字节在帧尾前，
     * 则仍从 byte1 开始读取标准的 12 字节位姿。 */
    payload_offset = ((frame_length == SLAM_FRAME_SIZE_15) && (frame[1] == 0x55U)) ? 2U : 1U;

    next_pose = slam_pose;
    /* 当前 15 字节带校验输出的实测位置单位为 0.1 mm，统一换算为米。 */
    next_pose.x_count = Slam_ReadInt16Be(&frame[payload_offset]);
    next_pose.y_count = Slam_ReadInt16Be(&frame[payload_offset + 2U]);
    next_pose.z_count = Slam_ReadInt16Be(&frame[payload_offset + 4U]);
    next_pose.x_m = (float)next_pose.x_count * SLAM_POSITION_SCALE_M;
    next_pose.y_m = (float)next_pose.y_count * SLAM_Y_POSITION_SCALE_M;
    next_pose.z_m = (float)next_pose.z_count * SLAM_POSITION_SCALE_M;
    /* 实测模块的姿态字段单位为 0.01 度：例如原始 1023 表示 10.23 度。 */
    next_pose.roll_count = Slam_ReadInt16Be(&frame[payload_offset + 6U]);
    next_pose.pitch_count = Slam_ReadInt16Be(&frame[payload_offset + 8U]);
    next_pose.yaw_count = Slam_ReadInt16Be(&frame[payload_offset + 10U]);
    next_pose.roll_deg = (float)next_pose.roll_count * 0.01f;
    next_pose.pitch_deg = (float)next_pose.pitch_count * 0.01f;
    next_pose.yaw_deg = (float)next_pose.yaw_count * 0.01f;
    memcpy(next_pose.raw_frame, frame, frame_length);
    next_pose.raw_length = frame_length;
    next_pose.last_update_ms = HAL_GetTick();
    next_pose.valid_frames++;
    next_pose.online = 1U;

    /* 姿态和位置作为一个快照发布，防止遥测读到半更新的数据。 */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        slam_pose = next_pose;
        if (primask == 0U) {
            __enable_irq();
        }
    }
}

static void Slam_ParseByte(uint8_t byte)
{
    if (slam_frame_index == 0U) {
        if (byte == SLAM_FRAME_HEAD) {
            slam_frame[0] = byte;
            slam_frame_index = 1U;
        }
        return;
    }

    slam_frame[slam_frame_index++] = byte;
    if (slam_frame_index < SLAM_FRAME_SIZE_14) {
        return;
    }

    if (slam_frame_index == SLAM_FRAME_SIZE_14) {
        if (slam_frame[SLAM_FRAME_SIZE_14 - 1U] == SLAM_FRAME_TAIL) {
            Slam_PublishFrame(slam_frame, SLAM_FRAME_SIZE_14);
            slam_frame_index = 0U;
            return;
        }
        /* 14 字节位置不是帧尾，保留本帧并再等待第 15 个字节。 */
        return;
    }

    if (slam_frame[SLAM_FRAME_SIZE_15 - 1U] == SLAM_FRAME_TAIL) {
        Slam_PublishFrame(slam_frame, SLAM_FRAME_SIZE_15);
        slam_frame_index = 0U;
    } else if (byte == SLAM_FRAME_HEAD) {
        /* 错帧中又发现帧头，直接把这个字节作为下一帧的开始。 */
        slam_pose.format_errors++;
        slam_frame[0] = byte;
        slam_frame_index = 1U;
    } else {
        slam_pose.format_errors++;
        slam_frame_index = 0U;
    }
}

void Slam_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    memset(slam_rx_ring, 0, sizeof(slam_rx_ring));
    memset(slam_frame, 0, sizeof(slam_frame));
    memset(&slam_pose, 0, sizeof(slam_pose));
    slam_rx_write_index = 0U;
    slam_rx_read_index = 0U;
    slam_frame_index = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Slam_OnUartRx(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == 0) || (length == 0U)) {
        return;
    }

    slam_pose.rx_bytes += length;
    for (index = 0U; index < length; index++) {
        uint16_t next_write_index = (uint16_t)((slam_rx_write_index + 1U) % SLAM_RX_RING_SIZE);

        if (next_write_index == slam_rx_read_index) {
            slam_pose.rx_overflow++;
            continue;
        }
        slam_rx_ring[slam_rx_write_index] = data[index];
        slam_rx_write_index = next_write_index;
    }
}

void Slam_OnUartError(void)
{
    slam_pose.uart_errors++;
}

void Slam_ProcessRx(void)
{
    while (slam_rx_read_index != slam_rx_write_index) {
        uint8_t byte = slam_rx_ring[slam_rx_read_index];

        slam_rx_read_index = (uint16_t)((slam_rx_read_index + 1U) % SLAM_RX_RING_SIZE);
        Slam_ParseByte(byte);
    }
}

void Slam_Update(void)
{
    if ((slam_pose.online != 0U) &&
        ((HAL_GetTick() - slam_pose.last_update_ms) > SLAM_ONLINE_TIMEOUT_MS)) {
        slam_pose.online = 0U;
    }
}

void Slam_GetPose(SlamPose_t *pose)
{
    uint32_t primask;

    if (pose == 0) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *pose = slam_pose;
    if (primask == 0U) {
        __enable_irq();
    }
}
