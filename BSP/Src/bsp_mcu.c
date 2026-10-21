/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.c
  * @brief          : MCU peripheral initialization functions
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_mcu.h"
#include "adc.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bmi088.h"
#include "usb_device.h"

volatile float MCU_Temperature_C = 0.0f;
volatile uint16_t MCU_TemperatureRaw = 0U;
volatile uint16_t MCU_VrefIntRaw = 0U;
volatile uint32_t MCU_Vdda_mV = 0U;
volatile uint8_t MCU_TemperatureValid = 0U;

void MCU_Temperature_Init(void)
{
  /* ADC3 专用于内部温度与 VREFINT 采样。 */
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
                                  ADC_SINGLE_ENDED) != HAL_OK) {
    MCU_TemperatureValid = 0U;
    return;
  }

  /* 内部模拟通道使能后需要至少 26 us 的稳定时间。 */
  HAL_Delay(1U);
  MCU_Temperature_Update();
}

void MCU_Temperature_Update(void)
{
  uint32_t temperature_raw;
  uint32_t vrefint_raw;

  if (HAL_ADC_Start(&hadc3) != HAL_OK) {
    MCU_TemperatureValid = 0U;
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc3, 2U) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc3);
    MCU_TemperatureValid = 0U;
    return;
  }
  temperature_raw = HAL_ADC_GetValue(&hadc3);

  if (HAL_ADC_PollForConversion(&hadc3, 2U) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc3);
    MCU_TemperatureValid = 0U;
    return;
  }
  vrefint_raw = HAL_ADC_GetValue(&hadc3);
  (void)HAL_ADC_Stop(&hadc3);

  if (vrefint_raw == 0U) {
    MCU_TemperatureValid = 0U;
    return;
  }

  MCU_TemperatureRaw = (uint16_t)temperature_raw;
  MCU_VrefIntRaw = (uint16_t)vrefint_raw;
  MCU_Vdda_mV = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(
      vrefint_raw, ADC_RESOLUTION_12B);
  MCU_Temperature_C = (float)__HAL_ADC_CALC_TEMPERATURE(
      MCU_Vdda_mV, temperature_raw, ADC_RESOLUTION_12B);
  MCU_TemperatureValid = 1U;
}

/**
  * @brief Initializes the MCU.
  */
void MCU_Init(void)
{
  /* ----------------------- BSP Init ----------------------- */
  BSP_PWM_Init();
	BSP_GPIO_Init();
  BSP_FDCAN_Init();
  BSP_USART_Init();
	BSP_ADC_Init();
  MCU_Temperature_Init();
  BSP_Buzzer_Init();
	BSP_Buzzer_PlayStartup();
	MX_USB_DEVICE_Init();
  /* ----------------------- Device Init ----------------------- */
  BMI088_Init();
}
//------------------------------------------------------------------------------
