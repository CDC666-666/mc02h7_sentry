/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Remote_Control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS received data
 */
/* ET08A through its ELRS receiver uses a standard 25-byte SBUS frame. */
#define SBUS_RX_BUF_NUM        25U
#define SBUS_CHANNEL_COUNT     16U
#define SBUS_FRAME_START       0x0FU
#define SBUS_FRAME_END         0x00U
#define SBUS_FLAG_FRAME_LOST   0x04U
#define SBUS_FLAG_FAILSAFE     0x08U

#define ET08A_RC_CH_VALUE_MIN    353U
#define ET08A_RC_CH_VALUE_OFFSET 1024U
#define ET08A_RC_CH_VALUE_MAX    1694U

/* 与 F407 StandardRobotpp 一致的三种标准 SBUS 遥控器标定值。 */
#define AT9S_PRO_RC_CH_VALUE_OFFSET 1000U
#define AT9S_PRO_RC_CH_VALUE_SPAN    800U
#define HT8A_RC_CH013_VALUE_OFFSET   992U
#define HT8A_RC_CH013_VALUE_SPAN     560U
#define HT8A_RC_CH247_VALUE_OFFSET   992U
#define HT8A_RC_CH247_VALUE_SPAN     800U
#define ET08A_RC_CH_VALUE_SPAN       671U

#define RC_SW_UP   1U
#define RC_SW_MID  3U
#define RC_SW_DOWN 2U

#define RC_CH_LEFT_HORIZONTAL  2U
#define RC_CH_LEFT_VERTICAL    3U
#define RC_CH_RIGHT_HORIZONTAL 0U
#define RC_CH_RIGHT_VERTICAL   1U
/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U

/**
 * @brief judgement keyboard set short time
 */
#define KEY_SET_SHORT_TIME		50U
/**
 * @brief judgement keyboard set long time
 */
#define KEY_SET_LONG_TIME		1000U

/**
 * @brief status of keyboard up
 */
#define KEY_UP                    0x00U
/**
 * @brief status of keyboard down
 */
#define KEY_DOWN                  0x01U

/**
 * @brief MAX speed of mouse speed
 */
#define MOUSE_SPEED_MAX		300U

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the status of the keyboard.
 */
typedef enum
{
	UP,			/*!< up */
	SHORT_DOWN,	/*!< short time down */
	DOWN,		/*!< long time down */
	PRESS,		/*!< 0->1 */
	RELAX,		/*!< 1->0 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

typedef struct
{
	uint16_t Count;
	KeyBoard_Status_e Status;
	KeyBoard_Status_e last_Status;
	bool last_KEY_PRESS;
	bool KEY_PRESS;
}KeyBoard_Info_Typedef;

typedef struct
{
	KeyBoard_Info_Typedef press_l;
	KeyBoard_Info_Typedef press_r;
	KeyBoard_Info_Typedef W;
	KeyBoard_Info_Typedef S;
	KeyBoard_Info_Typedef A;
	KeyBoard_Info_Typedef D;
	KeyBoard_Info_Typedef SHIFT;
	KeyBoard_Info_Typedef CTRL;
	KeyBoard_Info_Typedef Q;
	KeyBoard_Info_Typedef E;
	KeyBoard_Info_Typedef R;
	KeyBoard_Info_Typedef F;
	KeyBoard_Info_Typedef G;
	KeyBoard_Info_Typedef Z;
	KeyBoard_Info_Typedef X;
	KeyBoard_Info_Typedef C;
	KeyBoard_Info_Typedef V;
	KeyBoard_Info_Typedef B;
}Remote_Pressed_Typedef;


/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
	/**
	 * @brief structure that contains the information for the lever/Switch.
	 */
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	
	/**
	 * @brief structure that contains the information for the mouse.
	 */
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	/**
	 * @brief structure that contains the information for the keyboard.
	 */
	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* ET08A/ELRS receive diagnostics for the debugger watch window. */
extern volatile uint16_t remote_sbus_channels[SBUS_CHANNEL_COUNT];
extern volatile uint8_t remote_sbus_flags;
extern volatile uint32_t remote_sbus_frame_count;
extern volatile uint32_t remote_sbus_error_count;

/* Mouse Exported defines -----------------------------------------------------*/
#define MOUSE_X_MOVE_SPEED    (remote_ctrl.mouse.x )
#define MOUSE_Y_MOVE_SPEED    (remote_ctrl.mouse.y )
#define MOUSE_Z_MOVE_SPEED    (remote_ctrl.mouse.z )
#define MOUSE_PRESSED_LEFT    (remote_ctrl.mouse.press_l)
#define MOUSE_PRESSED_RIGHT   (remote_ctrl.mouse.press_r)

/* KeyBoard Exported defines --------------------------------------------------*/
#define KeyBoard_W            (remote_ctrl.key.set.W)
#define KeyBoard_S            (remote_ctrl.key.set.S)
#define KeyBoard_A            (remote_ctrl.key.set.A)
#define KeyBoard_D            (remote_ctrl.key.set.D)
#define KeyBoard_SHIFT        (remote_ctrl.key.set.SHIFT)
#define KeyBoard_CTRL         (remote_ctrl.key.set.CTRL)
#define KeyBoard_Q            (remote_ctrl.key.set.Q)
#define KeyBoard_E            (remote_ctrl.key.set.E)
#define KeyBoard_R            (remote_ctrl.key.set.R)
#define KeyBoard_F            (remote_ctrl.key.set.F)
#define KeyBoard_G            (remote_ctrl.key.set.G)
#define KeyBoard_Z            (remote_ctrl.key.set.Z)
#define KeyBoard_X            (remote_ctrl.key.set.X)
#define KeyBoard_C            (remote_ctrl.key.set.C)
#define KeyBoard_V            (remote_ctrl.key.set.V)
#define KeyBoard_B            (remote_ctrl.key.set.B)

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);

/**
  * @brief  report the cover status that acrroding the key_R swicthing
  */
extern bool Key_R(void);
/**
  * @brief  switch the shooter mode that acrroding the key_B
  */
extern bool Key_B(void);

/**
  * @brief  report the auto aim status that acrroding the mouse right swicthing
  */
extern bool Mouse_Pressed_Right(void);
/**
  * @brief  report the fire status that acrroding the mouse left swicthing
  */
extern bool Mouse_Pressed_Left(void);

#endif //REMOTE_CONTROL_H
