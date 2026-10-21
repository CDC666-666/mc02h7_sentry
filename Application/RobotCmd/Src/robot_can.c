#include "robot_can.h"

#include "fdcan.h"
#include "extra_imu.h"
#include "math.h"
#include "robot_motor.h"
#include "robot_param.h"

volatile uint32_t RobotCan_RxCount[4] = {0U};
volatile uint32_t RobotCan_LastRxId[4] = {0U};
volatile uint32_t RobotCan_TxQueuedCount[4] = {0U};
volatile uint32_t RobotCan_TxRejectedCount[4] = {0U};
volatile uint32_t RobotCan_LastTxId[4] = {0U};
volatile uint8_t RobotCan_LastTxData[4][8] = {{0U}};
volatile uint32_t RobotCan_TxErrorCount[4] = {0U};
volatile uint32_t RobotCan_RxErrorCount[4] = {0U};
volatile uint32_t RobotCan_LastErrorCode[4] = {0U};
volatile uint32_t RobotCan_Activity[4] = {0U};
volatile uint8_t RobotCan_ErrorPassive[4] = {0U};
volatile uint8_t RobotCan_Warning[4] = {0U};
volatile uint8_t RobotCan_BusOff[4] = {0U};

volatile ExtraImu_t extra_imu = {0};
volatile uint32_t ExtraImu_RequestCount = 0U;
volatile uint32_t ExtraImu_RequestRejectedCount = 0U;

static float ExtraImu_UintToFloat(uint16_t value, float minimum, float maximum)
{
    return minimum + ((float)value * (maximum - minimum) / 65535.0f);
}

bool ExtraImu_IsOnline(void)
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    return extra_imu.valid_frames != 0U &&
           (HAL_GetTick() - extra_imu.last_update_ms) <= EXTRA_IMU_TIMEOUT_MS;
#else
    return false;
#endif
}

static float ExtraImu_GetYawTemperatureBias(float temperature_c)
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    static const float table[EXTRA_IMU_YAW_TEMP_BIAS_POINT_COUNT][2] = {
        EXTRA_IMU_YAW_TEMP_BIAS_TABLE
    };
    const uint32_t last = EXTRA_IMU_YAW_TEMP_BIAS_POINT_COUNT - 1U;

    if (temperature_c <= table[0][0]) return table[0][1];
    if (temperature_c >= table[last][0]) return table[last][1];

    for (uint32_t index = 1U; index <= last; index++) {
        if (temperature_c <= table[index][0]) {
            const float ratio = (temperature_c - table[index - 1U][0]) /
                                (table[index][0] - table[index - 1U][0]);
            return table[index - 1U][1] +
                   ratio * (table[index][1] - table[index - 1U][1]);
        }
    }
#else
    (void)temperature_c;
#endif
    return 0.0f;
}

bool ExtraImu_HasYawReference(void)
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    const uint32_t now = HAL_GetTick();
    return ExtraImu_IsOnline() &&
           (now - extra_imu.last_gyro_update_ms) <= EXTRA_IMU_TIMEOUT_MS &&
           (now - extra_imu.last_euler_update_ms) <= EXTRA_IMU_TIMEOUT_MS;
#else
    return false;
#endif
}

float ExtraImu_GetYawBias(void)
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    return ExtraImu_GetYawTemperatureBias(extra_imu.temperature_c);
#else
    return 0.0f;
#endif
}

float ExtraImu_GetYawRateCompensated(void)
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    return EXTRA_IMU_YAW_DIRECTION * extra_imu.gyro_rad_s[2] - ExtraImu_GetYawBias();
#else
    return 0.0f;
#endif
}

bool ExtraImu_CanReceive(uint8_t can_index, uint32_t identifier, const uint8_t data[8])
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1)
    uint16_t value0;
    uint16_t value1;
    uint16_t value2;

    /*
     * 说明书区分 CAN_ID（请求地址）和 MST_ID（应答/主动发送地址）。
     * 台架阶段兼容接收两者，网页记录实际收到的 ID；两者均是独立预留 ID，
     * 不会与 CAN3 的 DJI 电机反馈 ID 冲突。
     */
    if (data == 0 || can_index != EXTRA_IMU_CAN ||
        identifier != EXTRA_IMU_MASTER_ID) return false;

    for (uint8_t index = 0U; index < 8U; index++) extra_imu.last_data[index] = data[index];
    extra_imu.last_data_type = data[0];
    extra_imu.last_identifier = identifier;
    value0 = ((uint16_t)data[3] << 8U) | data[2];
    value1 = ((uint16_t)data[5] << 8U) | data[4];
    value2 = ((uint16_t)data[7] << 8U) | data[6];

    switch (data[0]) {
        case 0x01U:
            /* DM-IMU-L1 的 DATA[1] 是整型摄氏温度，不是 16 位映射值。 */
            extra_imu.temperature_c = (float)data[1];
            extra_imu.acceleration_mps2[0] = ExtraImu_UintToFloat(value0, -235.2f, 235.2f);
            extra_imu.acceleration_mps2[1] = ExtraImu_UintToFloat(value1, -235.2f, 235.2f);
            extra_imu.acceleration_mps2[2] = ExtraImu_UintToFloat(value2, -235.2f, 235.2f);
            break;
        case 0x02U:
            /* 陀螺仪范围 ±2000 deg/s，即约 ±34.88 rad/s。 */
            extra_imu.gyro_rad_s[0] = ExtraImu_UintToFloat(value0, -34.88f, 34.88f);
            extra_imu.gyro_rad_s[1] = ExtraImu_UintToFloat(value1, -34.88f, 34.88f);
            extra_imu.gyro_rad_s[2] = ExtraImu_UintToFloat(value2, -34.88f, 34.88f);
            extra_imu.last_gyro_update_ms = HAL_GetTick();
            break;
        case 0x03U:
            /* 官方 CAN 协议欧拉角顺序为 Pitch、Yaw、Roll，均为小端映射值。 */
            extra_imu.pitch_deg = ExtraImu_UintToFloat(value0, -90.0f, 90.0f);
            extra_imu.yaw_deg = ExtraImu_UintToFloat(value1, -180.0f, 180.0f);
            extra_imu.roll_deg = ExtraImu_UintToFloat(value2, -180.0f, 180.0f);
            extra_imu.last_euler_update_ms = HAL_GetTick();
            break;
        case 0x04U: {
            /* W[13:6] 在 DATA[1]，W[5:0] 在 DATA[2] 的 bit7:2。 */
            uint16_t quaternion_w = ((uint16_t)data[1] << 6U) | ((data[2] & 0xF8U) >> 2U);
            uint16_t quaternion_x = ((uint16_t)(data[2] & 0x03U) << 12U) |
                                    ((uint16_t)data[3] << 4U) | ((data[4] & 0xF0U) >> 4U);
            uint16_t quaternion_y = ((uint16_t)(data[4] & 0x0FU) << 10U) |
                                    ((uint16_t)data[5] << 2U) | ((data[6] & 0xC0U) >> 6U);
            uint16_t quaternion_z = ((uint16_t)(data[6] & 0x3FU) << 8U) | data[7];
            extra_imu.quaternion[0] = -1.0f + ((float)quaternion_w * 2.0f / 16383.0f);
            extra_imu.quaternion[1] = -1.0f + ((float)quaternion_x * 2.0f / 16383.0f);
            extra_imu.quaternion[2] = -1.0f + ((float)quaternion_y * 2.0f / 16383.0f);
            extra_imu.quaternion[3] = -1.0f + ((float)quaternion_z * 2.0f / 16383.0f);
            break;
        }
        default:
            extra_imu.invalid_frames++;
            return true;
    }

    extra_imu.frame_mask |= (uint8_t)(1U << (data[0] - 1U));
    extra_imu.valid_frames++;
    extra_imu.last_update_ms = HAL_GetTick();
    return true;
#else
    (void)can_index;
    (void)identifier;
    (void)data;
    return false;
#endif
}

bool ExtraImu_RequestNext(void)
{
#if (EXTRA_IMU_TYPE == EXTRA_IMU_DM_IMU_L1) && (EXTRA_IMU_REQUEST_MODE_ENABLE != 0U)
    static uint8_t register_id = 0x01U;
    uint8_t request_data[8] = {0xCCU, register_id, 0x00U, 0xDDU, 0U, 0U, 0U, 0U};

    /* 达妙 MC02_CAN 官方例程：标准帧 ID=CAN_ID，DLC=8，
     * DATA={0xCC, RID, 读(0x00), 0xDD, 0, 0, 0, 0}。 */
    if (RobotCan_Send(EXTRA_IMU_CAN, EXTRA_IMU_CAN_ID, request_data)) {
        ExtraImu_RequestCount++;
        register_id++;
        if (register_id > 0x04U) register_id = 0x01U;
        return true;
    }

    ExtraImu_RequestRejectedCount++;
    return false;
#else
    return false;
#endif
}

static FDCAN_HandleTypeDef *RobotCan_GetHandle(uint8_t can_index)
{
    switch (can_index) {
        case 1U: return &hfdcan1;
        case 2U: return &hfdcan2;
        case 3U: return &hfdcan3;
        default: return 0;
    }
}

static void RobotCan_Start(uint8_t can_index, uint32_t fifo)
{
    FDCAN_FilterTypeDef filter = {0};
    FDCAN_HandleTypeDef *handle = RobotCan_GetHandle(can_index);

    if (handle == 0) return;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = fifo == FDCAN_RX_FIFO0 ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    (void)HAL_FDCAN_ConfigFilter(handle, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(
        handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    (void)HAL_FDCAN_ActivateNotification(
        handle, fifo == FDCAN_RX_FIFO0 ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
        0U);
    (void)HAL_FDCAN_Start(handle);
}

void RobotCan_Init(void)
{
#if ROBOT_CAN1_ENABLED
    RobotCan_Start(1U, FDCAN_RX_FIFO0);
#endif
#if ROBOT_CAN2_ENABLED
    RobotCan_Start(2U, FDCAN_RX_FIFO1);
#endif
#if ROBOT_CAN3_ENABLED
    RobotCan_Start(3U, FDCAN_RX_FIFO0);
#endif
}

void RobotCan_UpdateDiagnostics(void)
{
    for (uint8_t can_index = 1U; can_index <= 3U; can_index++) {
        FDCAN_HandleTypeDef *handle = RobotCan_GetHandle(can_index);
        FDCAN_ProtocolStatusTypeDef protocol_status = {0};
        FDCAN_ErrorCountersTypeDef error_counters = {0};

        if (handle == 0) continue;
        if (HAL_FDCAN_GetProtocolStatus(handle, &protocol_status) == HAL_OK) {
            RobotCan_LastErrorCode[can_index] = protocol_status.LastErrorCode;
            RobotCan_Activity[can_index] = protocol_status.Activity;
            RobotCan_ErrorPassive[can_index] = protocol_status.ErrorPassive != 0U ? 1U : 0U;
            RobotCan_Warning[can_index] = protocol_status.Warning != 0U ? 1U : 0U;
            RobotCan_BusOff[can_index] = protocol_status.BusOff != 0U ? 1U : 0U;
        }
        if (HAL_FDCAN_GetErrorCounters(handle, &error_counters) == HAL_OK) {
            RobotCan_TxErrorCount[can_index] = error_counters.TxErrorCnt;
            RobotCan_RxErrorCount[can_index] = error_counters.RxErrorCnt;
        }
    }
}

bool RobotCan_Send(uint8_t can_index, uint32_t identifier, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    FDCAN_HandleTypeDef *handle = RobotCan_GetHandle(can_index);
    HAL_StatusTypeDef status;

    if (handle == 0 || data == 0 || HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0U) {
        if (can_index >= 1U && can_index <= 3U) RobotCan_TxRejectedCount[can_index]++;
        return false;
    }
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    status = HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, data);
    if (can_index >= 1U && can_index <= 3U) {
        if (status == HAL_OK) {
            RobotCan_TxQueuedCount[can_index]++;
            RobotCan_LastTxId[can_index] = identifier;
            for (uint8_t index = 0U; index < 8U; index++) {
                RobotCan_LastTxData[can_index][index] = data[index];
            }
        } else RobotCan_TxRejectedCount[can_index]++;
    }
    return status == HAL_OK;
}

void RobotCan_Receive(uint8_t can_index, uint32_t identifier, const uint8_t data[8])
{
    if (can_index >= 1U && can_index <= 3U) {
        RobotCan_RxCount[can_index]++;
        RobotCan_LastRxId[can_index] = identifier;
    }
    /* 顶部 DM-IMU-L1 使用独立 CAN ID；匹配时不再交给电机反馈解析。 */
    if (ExtraImu_CanReceive(can_index, identifier, data)) return;
    RobotMotor_Receive(can_index, identifier, data);
}
