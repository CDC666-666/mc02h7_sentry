#ifndef DM4310_CAN1_H
#define DM4310_CAN1_H

#include <stdint.h>

/* StandardRobotpp-compatible DM4310 test path. */
#define DM4310_CAN1_SLAVE_ID  ((uint32_t)0x01U)
#define DM4310_CAN1_MASTER_ID ((uint32_t)0x51U)

typedef struct {
    uint8_t state;
    float position;
    float velocity;
    float torque;
    float temperature_mos;
    float temperature_rotor;
    uint32_t last_feedback_ms;
} DM4310_CAN1_Feedback_t;

extern volatile uint32_t dm4310_can1_rx_count;
extern volatile uint32_t dm4310_can1_tx_count;
extern DM4310_CAN1_Feedback_t dm4310_can1_feedback;

void DM4310_CAN1_Enable(void);
void DM4310_CAN1_Disable(void);
void DM4310_CAN1_ClearError(void);
void DM4310_CAN1_SendMIT(float position, float velocity, float kp, float kd, float torque);
void DM4310_CAN1_Receive(uint32_t identifier, const uint8_t data[8]);

#endif /* DM4310_CAN1_H */
