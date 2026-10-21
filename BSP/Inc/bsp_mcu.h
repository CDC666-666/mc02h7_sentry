/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.h
  * @brief          : MCU peripheral initialization functions
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_MCU_H
#define BSP_MCU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the MCU.
  */
extern void MCU_Init(void);

/* STM32H723 内部温度传感器调试数据。 */
extern volatile float MCU_Temperature_C;
extern volatile uint16_t MCU_TemperatureRaw;
extern volatile uint16_t MCU_VrefIntRaw;
extern volatile uint32_t MCU_Vdda_mV;
extern volatile uint8_t MCU_TemperatureValid;

void MCU_Temperature_Init(void);
void MCU_Temperature_Update(void);

#endif //BSP_MCU_H
