#include "bsp_can.h"
#include "fdcan.h"
#include "robot_can.h"

FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = FDCAN_DLC_BYTES_8,
    .Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat = FDCAN_CLASSIC_CAN,
    .Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0U,
};

FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = { .hcan = &hfdcan2 };
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = { .hcan = &hfdcan3 };

void BSP_FDCAN_Init(void)
{
    RobotCan_Init();
}

void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *frame)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(frame->hcan) == 0U) return;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(frame->hcan, &frame->Header, frame->Data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        uint8_t can_index;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }
        if (hfdcan == &hfdcan1) can_index = 1U;
        else if (hfdcan == &hfdcan3) can_index = 3U;
        else continue;
        RobotCan_Receive(can_index, header.Identifier, data);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hfdcan != &hfdcan2 || (rx_fifo1_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U) return;
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, data) != HAL_OK) return;
    RobotCan_Receive(2U, header.Identifier, data);
}
