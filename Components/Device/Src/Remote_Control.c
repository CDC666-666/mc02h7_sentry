


/* Includes ------------------------------------------------------------------*/
#include "Remote_Control.h"
#include "ramp.h"
#include "robot_param.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};

volatile uint16_t remote_sbus_channels[SBUS_CHANNEL_COUNT] = {0U};
volatile uint8_t remote_sbus_flags = 0U;
volatile uint32_t remote_sbus_frame_count = 0U;
volatile uint32_t remote_sbus_error_count = 0U;
volatile uint8_t remote_dbus_last_frame[DBUS_RX_BUF_NUM] = {0U};
volatile uint16_t remote_dbus_channels[4] = {0U};
volatile uint8_t remote_dbus_switches[2] = {0U};
volatile uint32_t remote_dbus_frame_count = 0U;
volatile uint32_t remote_dbus_error_count = 0U;
volatile uint32_t remote_rx_event_count = 0U;
volatile uint32_t remote_rx_length_error_count = 0U;
volatile uint16_t remote_last_rx_size = 0U;
volatile uint32_t remote_uart_error_count = 0U;
volatile uint32_t remote_uart_last_error = 0U;
volatile uint32_t remote_rx_start_error_count = 0U;

/**
 * @brief remote control usart RxDMA MultiBuffer
 */
__attribute__((section (".AXI_SRAM"), aligned(32))) uint8_t SBUS_MultiRx_Buf[2][REMOTE_RX_DMA_BUF_NUM];
 
/* Private variables ---------------------------------------------------------*/
/**
 * @brief structure that contains the information of keyboard
 */
KeyBoard_Info_Typedef KeyBoard_Info;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Update the status of keyboard
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
#if (__RC_TYPE != RC_DT7)
static uint16_t SbusReadChannel(const uint8_t *frame, uint8_t channel)
{
    const uint8_t *data = &frame[1];
    uint32_t packed;
    uint16_t bit_offset = (uint16_t)channel * 11U;
    uint8_t byte_offset = (uint8_t)(bit_offset >> 3U);
    uint8_t shift = (uint8_t)(bit_offset & 0x07U);

    packed = (uint32_t)data[byte_offset] |
             ((uint32_t)data[byte_offset + 1U] << 8U) |
             ((uint32_t)data[byte_offset + 2U] << 16U);
    return (uint16_t)((packed >> shift) & 0x07FFU);
}

static uint8_t SbusSwitchFromChannel(uint16_t channel, uint16_t center, uint16_t span)
{
    if (channel < center - span / 2U) return RC_SW_UP;
    if (channel > center + span / 2U) return RC_SW_DOWN;
    return RC_SW_MID;
}

static int16_t SbusScaleChannel(uint16_t channel, uint16_t center, uint16_t span,
                                int8_t direction)
{
    int32_t value = ((int32_t)channel - (int32_t)center) * 660 / (int32_t)span;
    return (int16_t)(value * (int32_t)direction);
}

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote)
{
    const uint8_t *frame = (const uint8_t *)sbus_buf;
    if (frame == NULL || remote == NULL) return;
    if (frame[0] != SBUS_FRAME_START ||
        (frame[SBUS_RX_BUF_NUM - 1U] != SBUS_FRAME_END &&
         frame[SBUS_RX_BUF_NUM - 1U] != 0x04U) ||
        (frame[23] & (SBUS_FLAG_FRAME_LOST | SBUS_FLAG_FAILSAFE)) != 0U) {
        remote_sbus_error_count++;
        return;
    }

    for (uint8_t channel = 0U; channel < SBUS_CHANNEL_COUNT; channel++) {
        remote_sbus_channels[channel] = SbusReadChannel(frame, channel);
    }
    remote_sbus_flags = frame[23];

#if (__RC_TYPE == RC_AT9S_PRO)
    remote->rc.ch[0] = SbusScaleChannel(remote_sbus_channels[0], AT9S_PRO_RC_CH_VALUE_OFFSET,
                                         AT9S_PRO_RC_CH_VALUE_SPAN, 1);
    remote->rc.ch[1] = SbusScaleChannel(remote_sbus_channels[1], AT9S_PRO_RC_CH_VALUE_OFFSET,
                                         AT9S_PRO_RC_CH_VALUE_SPAN, -1);
    remote->rc.ch[2] = SbusScaleChannel(remote_sbus_channels[3], AT9S_PRO_RC_CH_VALUE_OFFSET,
                                         AT9S_PRO_RC_CH_VALUE_SPAN, 1);
    remote->rc.ch[3] = SbusScaleChannel(remote_sbus_channels[2], AT9S_PRO_RC_CH_VALUE_OFFSET,
                                         AT9S_PRO_RC_CH_VALUE_SPAN, 1);
    remote->rc.s[0] = SbusSwitchFromChannel(remote_sbus_channels[5], AT9S_PRO_RC_CH_VALUE_OFFSET,
                                             AT9S_PRO_RC_CH_VALUE_SPAN);
    remote->rc.s[1] = SbusSwitchFromChannel(remote_sbus_channels[4], AT9S_PRO_RC_CH_VALUE_OFFSET,
                                             AT9S_PRO_RC_CH_VALUE_SPAN);
#elif (__RC_TYPE == RC_HT8A)
    remote->rc.ch[0] = SbusScaleChannel(remote_sbus_channels[0], HT8A_RC_CH013_VALUE_OFFSET,
                                         HT8A_RC_CH013_VALUE_SPAN, 1);
    remote->rc.ch[1] = SbusScaleChannel(remote_sbus_channels[1], HT8A_RC_CH013_VALUE_OFFSET,
                                         HT8A_RC_CH013_VALUE_SPAN, 1);
    remote->rc.ch[2] = SbusScaleChannel(remote_sbus_channels[3], HT8A_RC_CH013_VALUE_OFFSET,
                                         HT8A_RC_CH013_VALUE_SPAN, 1);
    remote->rc.ch[3] = SbusScaleChannel(remote_sbus_channels[2], HT8A_RC_CH247_VALUE_OFFSET,
                                         HT8A_RC_CH247_VALUE_SPAN, 1);
    remote->rc.s[0] = SbusSwitchFromChannel(remote_sbus_channels[7], HT8A_RC_CH247_VALUE_OFFSET,
                                             HT8A_RC_CH247_VALUE_SPAN);
    remote->rc.s[1] = SbusSwitchFromChannel(remote_sbus_channels[4], HT8A_RC_CH247_VALUE_OFFSET,
                                             HT8A_RC_CH247_VALUE_SPAN);
#elif (__RC_TYPE == RC_ET08A)
    /* 保留本 H7 已实测的 Ch0/Ch1 方向，避免迁入选型功能后改变现有操控方向。 */
    remote->rc.ch[0] = SbusScaleChannel(remote_sbus_channels[0], ET08A_RC_CH_VALUE_OFFSET,
                                         ET08A_RC_CH_VALUE_SPAN, -1);
    remote->rc.ch[1] = SbusScaleChannel(remote_sbus_channels[1], ET08A_RC_CH_VALUE_OFFSET,
                                         ET08A_RC_CH_VALUE_SPAN, 1);
    remote->rc.ch[2] = SbusScaleChannel(remote_sbus_channels[3], ET08A_RC_CH_VALUE_OFFSET,
                                         ET08A_RC_CH_VALUE_SPAN, 1);
    remote->rc.ch[3] = SbusScaleChannel(remote_sbus_channels[2], ET08A_RC_CH_VALUE_OFFSET,
                                         ET08A_RC_CH_VALUE_SPAN, 1);
    remote->rc.s[0] = SbusSwitchFromChannel(remote_sbus_channels[5], ET08A_RC_CH_VALUE_OFFSET,
                                             670U);
    remote->rc.s[1] = SbusSwitchFromChannel(remote_sbus_channels[4], ET08A_RC_CH_VALUE_OFFSET,
                                             670U);
#else
#error "UART5 SBUS only supports RC_AT9S_PRO, RC_HT8A or RC_ET08A"
#endif

    remote->rc.ch[4] = 0;
    remote->mouse.x = 0;
    remote->mouse.y = 0;
    remote->mouse.z = 0;
    remote->mouse.press_l = 0U;
    remote->mouse.press_r = 0U;
    remote->key.v = 0U;

    remote_sbus_frame_count++;
    remote->online_cnt = 0xFAU;
    remote->rc_lost = false;
}
#endif /* SBUS remote */

#if (__RC_TYPE == RC_DT7)
void DBUS_TO_RC(volatile const uint8_t *dbus_buf, Remote_Info_Typedef *remote)
{
    const uint8_t *frame = (const uint8_t *)dbus_buf;
    uint16_t channel[4];
    uint8_t switch_left;
    uint8_t switch_right;

    if (frame == NULL || remote == NULL) return;
    /* Retain even a malformed 18-byte frame for debugger inspection. */
    for (uint8_t i = 0U; i < DBUS_RX_BUF_NUM; i++) remote_dbus_last_frame[i] = frame[i];

    /* DJI DBUS has no frame header or checksum. Validate every channel before
     * publishing the frame and refreshing the receiver-online timer. */
    channel[0] = (uint16_t)(((uint16_t)frame[0] | ((uint16_t)frame[1] << 8U)) & 0x07FFU);
    channel[1] = (uint16_t)((((uint16_t)frame[1] >> 3U) | ((uint16_t)frame[2] << 5U)) & 0x07FFU);
    channel[2] = (uint16_t)((((uint16_t)frame[2] >> 6U) | ((uint16_t)frame[3] << 2U) |
                             ((uint16_t)frame[4] << 10U)) & 0x07FFU);
    channel[3] = (uint16_t)((((uint16_t)frame[4] >> 1U) | ((uint16_t)frame[5] << 7U)) & 0x07FFU);
    switch_left = (uint8_t)((frame[5] >> 4U) & 0x03U);
    switch_right = (uint8_t)((frame[5] >> 6U) & 0x03U);

    for (uint8_t i = 0U; i < 4U; i++) {
        if (channel[i] < 364U || channel[i] > 1684U) {
            remote_dbus_error_count++;
            remote_sbus_error_count++; /* Legacy telemetry field: active RC errors. */
            return;
        }
    }
    if (switch_left < RC_SW_UP || switch_left > RC_SW_MID ||
        switch_right < RC_SW_UP || switch_right > RC_SW_MID) {
        remote_dbus_error_count++;
        remote_sbus_error_count++;
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++) remote_dbus_channels[i] = channel[i];
    remote_dbus_switches[0] = switch_left;
    remote_dbus_switches[1] = switch_right;

    remote->rc.ch[0] = (int16_t)channel[0] - (int16_t)RC_CH_VALUE_OFFSET;
    remote->rc.ch[1] = (int16_t)channel[1] - (int16_t)RC_CH_VALUE_OFFSET;
    /* Keep the H7 chassis convention: ch[2] is forward, ch[3] is lateral. */
    remote->rc.ch[2] = (int16_t)channel[3] - (int16_t)RC_CH_VALUE_OFFSET;
    remote->rc.ch[3] = (int16_t)channel[2] - (int16_t)RC_CH_VALUE_OFFSET;
    remote->rc.ch[4] = 0; /* DBUS bytes 16-17 are reserved, not a fifth stick. */
    remote->rc.s[0] = switch_left;
    remote->rc.s[1] = switch_right;
    remote->mouse.x = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8U));
    remote->mouse.y = (int16_t)((uint16_t)frame[8] | ((uint16_t)frame[9] << 8U));
    remote->mouse.z = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8U));
    remote->mouse.press_l = frame[12];
    remote->mouse.press_r = frame[13];
    remote->key.v = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8U);

    remote_dbus_frame_count++;
    remote_sbus_frame_count++; /* Legacy telemetry field: active RC frames. */
    remote->online_cnt = 0xFAU;
    remote->rc_lost = false;
}
#endif /* DT7 DBUS remote */
//------------------------------------------------------------------------------

/**
  * @brief  clear the remote control data while the device offline
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
{
  /* Juege the device status */
  if(remote_ctrl->online_cnt <= 0x32U)
  {
    /* clear the data */
    memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));

    /* reset the online count */
		
    /* set the lost flag */
		remote_ctrl->rc_lost = true;
		
  }
  else if(remote_ctrl->online_cnt > 0)
  {
    /* online count decrements which reseted in received interrupt  */
    remote_ctrl->online_cnt--;
  }
}
//------------------------------------------------------------------------------
