/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to init the  BSP_USART_Init functions
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"
#include "Referee_System.h"
#include "slam.h"

/* SLAM 固定 14 字节帧的数据率很低。采用单字节中断接收，规避 DMA 空闲重启差异。 */
static uint8_t slam_uart_rx_byte;


void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){
 
    __attribute__((section (".AXI_SRAM"), aligned(32))) static uint8_t Rx_Buf[32];

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;


		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		Rx_Buf[12] =  0x00;
		Rx_Buf[13] =  0x00;
		Rx_Buf[14] =  0x80;
		Rx_Buf[15] =  0x7F;
		SCB_CleanDCache_by_Addr((uint32_t *)Rx_Buf, 32U);
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,16U);
	

}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );
static void USART5_SbusRx_Init(void);
static HAL_StatusTypeDef UART7_SlamRx_Init(void);


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void){

		
	USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);

	USART5_SbusRx_Init();
	Slam_Init();
	UART7_SlamRx_Init();

}

static void USART5_SbusRx_Init(void)
{
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, SBUS_MultiRx_Buf[0], SBUS_RX_BUF_NUM);
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

static HAL_StatusTypeDef UART7_SlamRx_Init(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UART_Receive_IT(&huart7, &slam_uart_rx_byte, 1U);
  if (status != HAL_OK) {
    Slam_OnUartError();
  }
  return status;
}

/* UART7 线路噪声、断线或错误电平后，立即恢复下一帧接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7) {
    Slam_OnUartError();
    (void)HAL_UART_AbortReceive(huart);
    (void)UART7_SlamRx_Init();
  }
}

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxXferSize    = DataLength * 2;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
		
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);	
	
	
}



/**
  * @brief  USER USART5 Reception Event Callback.(SBUS remote_ctrl)
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
  (void)huart;
  if (Size == SBUS_RX_BUF_NUM) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)SBUS_MultiRx_Buf[0], 32U);
    SBUS_TO_RC(SBUS_MultiRx_Buf[0], &remote_ctrl);
  }
  USART5_SbusRx_Init();
}

/**
  * @brief  USER USART1 Reception Event Callback.(Referee_System)
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
	{
		
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
				
				  if(Size >= 10){
						    SCB_InvalidateDCache_by_Addr((uint32_t *)Referee_System_Info_MultiRx_Buf[0],
						                                      (int32_t)((REFEREE_RXFRAME_LENGTH + 31U) & ~31U));
						    Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
				
				        memset(Referee_System_Info_MultiRx_Buf[0],0,REFEREE_RXFRAME_LENGTH);

				        __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
          }
				
	}
	else
	{
				__HAL_DMA_DISABLE(huart->hdmarx);
				
				((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
		    if(Size >= 10){
					 SCB_InvalidateDCache_by_Addr((uint32_t *)Referee_System_Info_MultiRx_Buf[1],
					                                   (int32_t)((REFEREE_RXFRAME_LENGTH + 31U) & ~31U));
			         Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
				
				       memset(Referee_System_Info_MultiRx_Buf[1],0,REFEREE_RXFRAME_LENGTH);

				       __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
      }
					
	}
  
}

/**
  * @brief  USER USART10 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  USER USART3 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  USER USART2 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(huart == &huart5){
	
		  USER_USART5_RxHandler(huart,Size);
		  return;
	 } 

	 if(huart == &huart1){
	 
      USER_USART1_RxHandler(huart,Size);
			
	 }
	
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

/* UART7 的 SLAM 数据逐字节进入解析队列，不会参与电机或底盘控制。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7) {
    Slam_OnUartRx(&slam_uart_rx_byte, 1U);
    (void)UART7_SlamRx_Init();
  }
}
