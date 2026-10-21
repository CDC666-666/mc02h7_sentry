#include "dm4310_can1.h"

#include "bsp_can.h"
#include "stm32h7xx_hal.h"

#define DM4310_P_MIN  (-12.5f)
#define DM4310_P_MAX  (12.5f)
#define DM4310_V_MIN  (-30.0f)
#define DM4310_V_MAX  (30.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)
#define DM4310_T_MIN  (-10.0f)
#define DM4310_T_MAX  (10.0f)

volatile uint32_t dm4310_can1_rx_count = 0U;
volatile uint32_t dm4310_can1_tx_count = 0U;
DM4310_CAN1_Feedback_t dm4310_can1_feedback = {0};

static float constrain(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static uint16_t float_to_uint(float value, float min, float max, uint8_t bits)
{
    const float span = max - min;
    const float scale = (float)((1UL << bits) - 1UL);
    value = constrain(value, min, max);
    return (uint16_t)((value - min) * scale / span);
}

static float uint_to_float(uint16_t value, float min, float max, uint8_t bits)
{
    const float span = max - min;
    const float scale = (float)((1UL << bits) - 1UL);
    return (float)value * span / scale + min;
}

static void send_special_command(uint8_t command)
{
    FDCAN1_TxFrame.Header.Identifier = DM4310_CAN1_SLAVE_ID;
    FDCAN1_TxFrame.Data[0] = 0xFFU;
    FDCAN1_TxFrame.Data[1] = 0xFFU;
    FDCAN1_TxFrame.Data[2] = 0xFFU;
    FDCAN1_TxFrame.Data[3] = 0xFFU;
    FDCAN1_TxFrame.Data[4] = 0xFFU;
    FDCAN1_TxFrame.Data[5] = 0xFFU;
    FDCAN1_TxFrame.Data[6] = 0xFFU;
    FDCAN1_TxFrame.Data[7] = command;
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);
    dm4310_can1_tx_count++;
}

void DM4310_CAN1_Enable(void)
{
    send_special_command(0xFCU);
}

void DM4310_CAN1_Disable(void)
{
    send_special_command(0xFDU);
}

void DM4310_CAN1_ClearError(void)
{
    send_special_command(0xFBU);
}

void DM4310_CAN1_SendMIT(float position, float velocity, float kp, float kd, float torque)
{
    const uint16_t position_raw = float_to_uint(position, DM4310_P_MIN, DM4310_P_MAX, 16U);
    const uint16_t velocity_raw = float_to_uint(velocity, DM4310_V_MIN, DM4310_V_MAX, 12U);
    const uint16_t kp_raw = float_to_uint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12U);
    const uint16_t kd_raw = float_to_uint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12U);
    const uint16_t torque_raw = float_to_uint(torque, DM4310_T_MIN, DM4310_T_MAX, 12U);

    FDCAN1_TxFrame.Header.Identifier = DM4310_CAN1_SLAVE_ID;
    FDCAN1_TxFrame.Data[0] = (uint8_t)(position_raw >> 8U);
    FDCAN1_TxFrame.Data[1] = (uint8_t)position_raw;
    FDCAN1_TxFrame.Data[2] = (uint8_t)(velocity_raw >> 4U);
    FDCAN1_TxFrame.Data[3] = (uint8_t)((velocity_raw & 0x0FU) << 4U) | (uint8_t)(kp_raw >> 8U);
    FDCAN1_TxFrame.Data[4] = (uint8_t)kp_raw;
    FDCAN1_TxFrame.Data[5] = (uint8_t)(kd_raw >> 4U);
    FDCAN1_TxFrame.Data[6] = (uint8_t)((kd_raw & 0x0FU) << 4U) | (uint8_t)(torque_raw >> 8U);
    FDCAN1_TxFrame.Data[7] = (uint8_t)torque_raw;
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);
    dm4310_can1_tx_count++;
}

void DM4310_CAN1_Receive(uint32_t identifier, const uint8_t data[8])
{
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;

    if (identifier != DM4310_CAN1_MASTER_ID) return;

    dm4310_can1_feedback.state = data[0] >> 4U;
    position_raw = ((uint16_t)data[1] << 8U) | data[2];
    velocity_raw = ((uint16_t)data[3] << 4U) | (data[4] >> 4U);
    torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8U) | data[5];
    dm4310_can1_feedback.position = uint_to_float(position_raw, DM4310_P_MIN, DM4310_P_MAX, 16U);
    dm4310_can1_feedback.velocity = uint_to_float(velocity_raw, DM4310_V_MIN, DM4310_V_MAX, 12U);
    dm4310_can1_feedback.torque = uint_to_float(torque_raw, DM4310_T_MIN, DM4310_T_MAX, 12U);
    dm4310_can1_feedback.temperature_mos = (float)data[6];
    dm4310_can1_feedback.temperature_rotor = (float)data[7];
    dm4310_can1_feedback.last_feedback_ms = HAL_GetTick();
    dm4310_can1_rx_count++;
}
