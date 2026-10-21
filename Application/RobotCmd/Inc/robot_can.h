#ifndef ROBOT_CAN_H
#define ROBOT_CAN_H

#include <stdbool.h>
#include <stdint.h>

/* CAN 诊断数据。数组下标与 CAN 编号一致，0 号下标保留不用。 */
extern volatile uint32_t RobotCan_RxCount[4];
extern volatile uint32_t RobotCan_LastRxId[4];
extern volatile uint32_t RobotCan_TxQueuedCount[4];
extern volatile uint32_t RobotCan_TxRejectedCount[4];
extern volatile uint32_t RobotCan_LastTxId[4];
extern volatile uint8_t RobotCan_LastTxData[4][8];
/* FDCAN 硬件协议状态：用于区分“已入发送队列”和“总线实际发送异常”。 */
extern volatile uint32_t RobotCan_TxErrorCount[4];
extern volatile uint32_t RobotCan_RxErrorCount[4];
extern volatile uint32_t RobotCan_LastErrorCode[4];
extern volatile uint32_t RobotCan_Activity[4];
extern volatile uint8_t RobotCan_ErrorPassive[4];
extern volatile uint8_t RobotCan_Warning[4];
extern volatile uint8_t RobotCan_BusOff[4];

void RobotCan_Init(void);
void RobotCan_UpdateDiagnostics(void);
bool RobotCan_Send(uint8_t can_index, uint32_t identifier, const uint8_t data[8]);
void RobotCan_Receive(uint8_t can_index, uint32_t identifier, const uint8_t data[8]);

#endif /* ROBOT_CAN_H */
