#include "Bmi088.h"
#include "FreeRTOS.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"
#include "main.h"
#include "spi.h"
#include "task.h"

static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data);
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *data);
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *data, uint8_t len);
static void BMI088_DMA_Kick(void);
static void BMI088_DMA_Complete(void);

#define BMI088_Accel_Write_Single_Reg(reg, data) \
    do {                                         \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    } while (0)

#define BMI088_Accel_Read_Single_Reg(reg, data) \
    do {                                        \
        BMI088_ACCEL_NS_L();                    \
        BMI088_Read_Write_Byte((reg) | 0x80U);  \
        BMI088_Read_Write_Byte(0x55U);          \
        (data) = BMI088_Read_Write_Byte(0x55U); \
        BMI088_ACCEL_NS_H();                    \
    } while (0)

#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    do {                                            \
        BMI088_ACCEL_NS_L();                        \
        BMI088_Read_Write_Byte((reg) | 0x80U);      \
        BMI088_Read_Multi_Reg((reg), (data), (len));\
        BMI088_ACCEL_NS_H();                        \
    } while (0)

#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    do {                                        \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    } while (0)

#define BMI088_Gyro_Read_Single_Reg(reg, data) \
    do {                                       \
        BMI088_GYRO_NS_L();                    \
        BMI088_Read_Single_Reg((reg), &(data));\
        BMI088_GYRO_NS_H();                    \
    } while (0)

#define BMI088_Gyro_Read_Multi_Reg(reg, data, len) \
    do {                                           \
        BMI088_GYRO_NS_L();                       \
        BMI088_Read_Multi_Reg((reg), (data), (len));\
        BMI088_GYRO_NS_H();                       \
    } while (0)

#define BMI088_DMA_CACHE_LINE 32U
#define BMI088_GYRO_DMA_LEN   8U
#define BMI088_ACCEL_DMA_LEN  9U
#define BMI088_TEMP_DMA_LEN   4U

typedef enum {
    BMI088_DMA_IDLE = 0U,
    BMI088_DMA_GYRO,
    BMI088_DMA_ACCEL,
    BMI088_DMA_TEMP,
} BMI088_DmaTransfer_e;

static const float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
static const float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

BMI088_Info_Typedef BMI088_Info;
uint32_t BMI088_DMA_ErrorCount;
uint32_t BMI088_DMA_GyroFrameCount;

/* 32-byte alignment is required because the H7 D-cache line is 32 bytes. */
static uint8_t bmi088_gyro_tx[BMI088_DMA_CACHE_LINE] __attribute__((section(".AXI_SRAM"), aligned(BMI088_DMA_CACHE_LINE))) =
    {0x82U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t bmi088_gyro_rx[BMI088_DMA_CACHE_LINE] __attribute__((section(".AXI_SRAM"), aligned(BMI088_DMA_CACHE_LINE)));
static uint8_t bmi088_accel_tx[BMI088_DMA_CACHE_LINE] __attribute__((section(".AXI_SRAM"), aligned(BMI088_DMA_CACHE_LINE))) =
    {0x92U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t bmi088_accel_rx[BMI088_DMA_CACHE_LINE] __attribute__((section(".AXI_SRAM"), aligned(BMI088_DMA_CACHE_LINE)));
static uint8_t bmi088_temp_tx[BMI088_DMA_CACHE_LINE] __attribute__((section(".AXI_SRAM"), aligned(BMI088_DMA_CACHE_LINE))) =
    {0xA2U, 0xFFU, 0xFFU, 0xFFU};
static uint8_t bmi088_temp_rx[BMI088_DMA_CACHE_LINE] __attribute__((section(".AXI_SRAM"), aligned(BMI088_DMA_CACHE_LINE)));

static volatile uint8_t bmi088_dma_enabled;
static volatile uint8_t bmi088_gyro_pending;
static volatile uint8_t bmi088_accel_pending;
static volatile uint8_t bmi088_temp_pending;
static volatile BMI088_DmaTransfer_e bmi088_active_transfer;
static TaskHandle_t bmi088_task_handle;

static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACCEL_PWR_CTRL, BMI088_ACCEL_ENABLE_ACCEL_ON, BMI088_ACCEL_PWR_CTRL_ERROR},
    {BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_PWR_ACTIVE_MODE, BMI088_ACCEL_PWR_CONF_ERROR},
    {BMI088_ACCEL_CONF, (BMI088_ACCEL_NORMAL | BMI088_ACCEL_800_HZ | BMI088_ACCEL_CONF_MUST_Set), BMI088_ACCEL_CONF_ERROR},
    {BMI088_ACCEL_RANGE, BMI088_ACCEL_RANGE_3G, BMI088_ACCEL_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, (BMI088_ACCEL_INT1_IO_ENABLE | BMI088_ACCEL_INT1_GPIO_PP | BMI088_ACCEL_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

static BMI088_Status_e BMI088_Accel_Init(void)
{
    uint8_t res = 0U;
    uint8_t write_reg_num;

    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Accel_Write_Single_Reg(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE);
    Delay_ms(BMI088_LONG_DELAY_TIME);
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACCEL_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0U; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0],
                                      Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]) {
            return (BMI088_Status_e)Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

static BMI088_Status_e BMI088_Gyro_Init(void)
{
    uint8_t res = 0U;
    uint8_t write_reg_num;

    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    Delay_ms(BMI088_LONG_DELAY_TIME);
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0U; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0],
                                     Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_Gyro_Read_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]) {
            return (BMI088_Status_e)Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

void BMI088_Init(void)
{
    BMI088_Status_e status;

    do {
        status = (BMI088_Status_e)(BMI088_Accel_Init() | BMI088_Gyro_Init());
        if (status != BMI088_NO_ERROR) {
            Delay_ms(2U);
        }
    } while (status != BMI088_NO_ERROR);

    /* StandardRobotpp estimates X/Y gyro bias in its EKF. Do not import
     * another board's stored offsets into this H723 profile. */
    BMI088_Info.Offsets_Gyro_X = 0.0f;
    BMI088_Info.Offsets_Gyro_Y = 0.0f;
    BMI088_Info.Offsets_Gyro_Z = 0.0f;
    BMI088_Info.Offsets_Init = true;
}

void BMI088_Info_Update(BMI088_Info_Typedef *info)
{
    (void)info;
    /* Measurements are decoded at DMA completion, before the INS task wakes. */
}

void BMI088_DMA_Start(BMI088_TaskHandle_t task_handle)
{
    bmi088_task_handle = (TaskHandle_t)task_handle;
    bmi088_dma_enabled = 1U;
    /* Start immediately; some board revisions do not route the gyro DRDY pin. */
    bmi088_gyro_pending = 1U;
    BMI088_DMA_Kick();
}

void BMI088_DMA_RequestGyro(void)
{
    if (bmi088_dma_enabled == 0U) return;
    bmi088_gyro_pending = 1U;
    BMI088_DMA_Kick();
}

void BMI088_DMA_DataReadyCallback(uint16_t gpio_pin)
{
    if (bmi088_dma_enabled == 0U) {
        return;
    }

    if (gpio_pin == GYRO_INT_Pin) {
        bmi088_gyro_pending = 1U;
    } else if (gpio_pin == ACCEL_INT_Pin) {
        bmi088_accel_pending = 1U;
        bmi088_temp_pending = 1U;
    }
    BMI088_DMA_Kick();
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    BMI088_DMA_DataReadyCallback(gpio_pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        BMI088_DMA_Complete();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        BMI088_ACCEL_NS_H();
        BMI088_GYRO_NS_H();
        bmi088_active_transfer = BMI088_DMA_IDLE;
        BMI088_DMA_ErrorCount++;
        BMI088_DMA_Kick();
    }
}

static void BMI088_DMA_Kick(void)
{
    uint8_t *tx = NULL;
    uint8_t *rx = NULL;
    uint16_t length = 0U;

    if ((bmi088_dma_enabled == 0U) || (bmi088_active_transfer != BMI088_DMA_IDLE)) {
        return;
    }

    if (bmi088_gyro_pending != 0U) {
        bmi088_gyro_pending = 0U;
        bmi088_active_transfer = BMI088_DMA_GYRO;
        BMI088_GYRO_NS_L();
        tx = bmi088_gyro_tx;
        rx = bmi088_gyro_rx;
        length = BMI088_GYRO_DMA_LEN;
    } else if (bmi088_accel_pending != 0U) {
        bmi088_accel_pending = 0U;
        bmi088_active_transfer = BMI088_DMA_ACCEL;
        BMI088_ACCEL_NS_L();
        tx = bmi088_accel_tx;
        rx = bmi088_accel_rx;
        length = BMI088_ACCEL_DMA_LEN;
    } else if (bmi088_temp_pending != 0U) {
        bmi088_temp_pending = 0U;
        bmi088_active_transfer = BMI088_DMA_TEMP;
        BMI088_ACCEL_NS_L();
        tx = bmi088_temp_tx;
        rx = bmi088_temp_rx;
        length = BMI088_TEMP_DMA_LEN;
    } else {
        return;
    }

    SCB_CleanDCache_by_Addr((uint32_t *)tx, BMI088_DMA_CACHE_LINE);
    SCB_InvalidateDCache_by_Addr((uint32_t *)rx, BMI088_DMA_CACHE_LINE);
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, tx, rx, length) != HAL_OK) {
        BMI088_ACCEL_NS_H();
        BMI088_GYRO_NS_H();
        bmi088_active_transfer = BMI088_DMA_IDLE;
        BMI088_DMA_ErrorCount++;
    }
}

static void BMI088_DMA_Complete(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    int16_t raw_value;
    BMI088_DmaTransfer_e completed_transfer = bmi088_active_transfer;

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();

    if (completed_transfer == BMI088_DMA_GYRO) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)bmi088_gyro_rx, BMI088_DMA_CACHE_LINE);
        raw_value = (int16_t)(((uint16_t)bmi088_gyro_rx[2] << 8) | bmi088_gyro_rx[1]);
        BMI088_Info.MPU_Info.Gyro_X = raw_value;
        BMI088_Info.Gyro[0] = raw_value * BMI088_GYRO_SEN;
        raw_value = (int16_t)(((uint16_t)bmi088_gyro_rx[4] << 8) | bmi088_gyro_rx[3]);
        BMI088_Info.MPU_Info.Gyro_Y = raw_value;
        BMI088_Info.Gyro[1] = raw_value * BMI088_GYRO_SEN;
        raw_value = (int16_t)(((uint16_t)bmi088_gyro_rx[6] << 8) | bmi088_gyro_rx[5]);
        BMI088_Info.MPU_Info.Gyro_Z = raw_value;
        BMI088_Info.Gyro[2] = raw_value * BMI088_GYRO_SEN;
        BMI088_DMA_GyroFrameCount++;
    } else if (completed_transfer == BMI088_DMA_ACCEL) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)bmi088_accel_rx, BMI088_DMA_CACHE_LINE);
        raw_value = (int16_t)(((uint16_t)bmi088_accel_rx[3] << 8) | bmi088_accel_rx[2]);
        BMI088_Info.MPU_Info.Accel_X = raw_value;
        BMI088_Info.Accel[0] = raw_value * BMI088_ACCEL_SEN;
        raw_value = (int16_t)(((uint16_t)bmi088_accel_rx[5] << 8) | bmi088_accel_rx[4]);
        BMI088_Info.MPU_Info.Accel_Y = raw_value;
        BMI088_Info.Accel[1] = raw_value * BMI088_ACCEL_SEN;
        raw_value = (int16_t)(((uint16_t)bmi088_accel_rx[7] << 8) | bmi088_accel_rx[6]);
        BMI088_Info.MPU_Info.Accel_Z = raw_value;
        BMI088_Info.Accel[2] = raw_value * BMI088_ACCEL_SEN;
    } else if (completed_transfer == BMI088_DMA_TEMP) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)bmi088_temp_rx, BMI088_DMA_CACHE_LINE);
        raw_value = (int16_t)(((uint16_t)bmi088_temp_rx[2] << 3) | (bmi088_temp_rx[3] >> 5));
        if (raw_value > 1023) {
            raw_value -= 2048;
        }
        BMI088_Info.MPU_Info.Temperature = raw_value;
        BMI088_Info.Temperature = raw_value * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    bmi088_active_transfer = BMI088_DMA_IDLE;
    BMI088_DMA_Kick();

    if ((completed_transfer == BMI088_DMA_GYRO) && (bmi088_task_handle != NULL)) {
        vTaskNotifyGiveFromISR(bmi088_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    BMI088_Read_Write_Byte(reg);
    BMI088_Read_Write_Byte(data);
}

static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *data)
{
    BMI088_Read_Write_Byte(reg | 0x80U);
    *data = BMI088_Read_Write_Byte(0x55U);
}

static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    (void)reg;
    while (len != 0U) {
        *data = BMI088_Read_Write_Byte(0x55U);
        data++;
        len--;
    }
}
