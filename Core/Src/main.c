/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ymodem.h"
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
#define size 200
#define Rxsize 13000
#define BL_ACK   0XA5
#define BL_NACK  0X7F

uint8_t buffer[size];
uint8_t Rx_buffer[Rxsize];


CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t TxMailBox;


uint8_t Txbuffer[8];
uint8_t Rxbuffer[8];
uint8_t aFileName[FILE_NAME_LENGTH] = "hser_app";
//aFileName = "user";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  pTxHeader.DLC = 8;
  pTxHeader.IDE = CAN_ID_EXT;
  pTxHeader.RTR = CAN_RTR_DATA;
  pTxHeader.StdId = 0xFE;
  pTxHeader.ExtId = 0x01;
  pTxHeader.TransmitGlobalTime = DISABLE;
  Txbuffer[0] = 1;
  Txbuffer[1] = 0x51;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  memset(Rx_buffer, 0, Rxsize);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	bootloader_uart_read_data();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */


  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    sFilterConfig.FilterBank = 0;
  	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	sFilterConfig.FilterIdHigh = 0x0000;
  	sFilterConfig.FilterIdLow = 0x0000;
  	sFilterConfig.FilterMaskIdHigh = 0;
  	sFilterConfig.FilterMaskIdLow = 0;
  	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  	sFilterConfig.FilterActivation = ENABLE;
  	sFilterConfig.SlaveStartFilterBank = 14;

  	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}

  	if (HAL_CAN_Start(&hcan) != HAL_OK)
		Error_Handler();

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING  | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
  		    /* Notification Error */
		Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bootloader_uart_read_data(void){
uint8_t rx_len = 0;
uint16_t temp = 0;
while (1){
	memset(buffer, 0 ,200);

	if (HAL_UART_Receive(&huart2, buffer, 1, 0x200) == HAL_OK)
		rx_len = buffer[0];
	if (HAL_UART_Receive(&huart2, buffer, rx_len, 0x200)== HAL_OK){
		memcpy(Rx_buffer + temp, buffer + 2, (rx_len - 6) * sizeof(uint8_t));
		temp += rx_len - 6;
	}
	switch (buffer[0]){
	case 0x50:
		handle_Get_Version(buffer, rx_len);
		break;
	case 0x51:
		handle_writing(buffer, rx_len);
		break;
	case 0x52:
		handle_start_download(buffer, rx_len);
	case 0x53:
		handle_exit_bootloader(buffer, rx_len);
	default:
		break;

	}

}
}
void handle_start_download(uint8_t *buffer, uint8_t rx_len){
	pTxHeader.ExtId = can_generate_header(0xFE,0x16,0x10,0x01);
	pTxHeader.IDE = CAN_ID_EXT;

	Txbuffer[1] = *(buffer + 1);
	Txbuffer[2] = *(buffer + 2);
	uint16_t NewFirmwareSize = *(Txbuffer+1);
	NewFirmwareSize <<= 8;
	NewFirmwareSize |= *(Txbuffer+2);

	pTxHeader.DLC= 2;
	HAL_CAN_AddTxMessage(&hcan, &pTxHeader, Txbuffer, &TxMailBox);
	pTxHeader.ExtId = can_generate_header(0xFE,0x16,0x10,2);
	pTxHeader.IDE = CAN_ID_EXT;
	pTxHeader.DLC= 1;

	HAL_CAN_AddTxMessage(&hcan, &pTxHeader, Txbuffer, &TxMailBox);
	COM_StatusTypeDef status;
	status = Ymodem_Transmit(&Rx_buffer[0], aFileName, NewFirmwareSize);

}
void handle_Get_Version(uint8_t *buffer, uint8_t rx_len){

	pTxHeader.ExtId = can_generate_header(0xFE,0x16,0x10,0x03);
	pTxHeader.IDE = CAN_ID_EXT;

	pTxHeader.DLC= sizeof(Txbuffer);
    HAL_CAN_AddTxMessage(&hcan, &pTxHeader, Txbuffer, &TxMailBox);

}
void handle_writing(uint8_t * buffer, uint8_t rx_len){
  uint8_t len = rx_len;

  uint32_t total_length = len ;

  uint32_t host_crc = *((uint32_t *) (buffer + total_length - 4));

  if (! bootloader_verify_crc(buffer, total_length - 4, host_crc, rx_len)){
	  bootloader_send_ack(buffer[1], 1);

  }
  else {
	  bootloader_send_nack();
  }
  }

void handle_exit_bootloader(uint8_t *buffer, uint8_t rx_len){

	pTxHeader.ExtId = can_generate_header(0xFE,0x16,0x10,0x06);
	pTxHeader.IDE = CAN_ID_EXT;

	pTxHeader.DLC= sizeof(Txbuffer);
    HAL_CAN_AddTxMessage(&hcan, &pTxHeader, Txbuffer, &TxMailBox);


}
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
 //here we send 2 byte.. first byte is ack and the second byte is len value
uint8_t ack_buf[2];
ack_buf[0] = BL_ACK;
ack_buf[1] = follow_len;
HAL_UART_Transmit(&huart2, ack_buf, 2, 0x2);

}

/*This function sends NACK */
void bootloader_send_nack(void)
{
uint8_t nack = BL_NACK;
HAL_UART_Transmit(&huart2, &nack, 1, 0x2);
}

//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host, uint8_t rx_len)
{
  uint32_t uwCRCValue;
  CRC->CR |= CRC_CR_RESET;
  CRC->DR = rx_len;

  for (uint32_t i=0 ; i < len ; i++)
{
	  CRC->DR = pData[i];
}
  uwCRCValue = CRC->DR;
 /* Reset CRC Calculation Unit */
//__HAL_CRC_DR_RESET(&hcrc);

if( uwCRCValue == crc_host)
{
	return VERIFY_CRC_SUCCESS;
}

return VERIFY_CRC_FAIL;
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//  /* Get RX message */
//  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, Rxbuffer);
//  if(Rxbuffer[0] == 0x12)
//  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//      //uint8_t  buffer1[] = "Fun";
//      //uint8_t ack = 12;
//      //HAL_UART_Transmit(&huart2, &ack, sizeof(ack), 0x2);
//
//}

uint32_t can_generate_header(uint8_t device_ID,uint8_t sensor_ID, uint8_t reply_info_type,uint8_t message_type){
	uint32_t tmp,res;
	tmp=0;
	res=0;
	tmp=device_ID;
	tmp<<=16;
	res|=tmp;
	tmp=0;

	tmp=sensor_ID;
	tmp<<=24;
	res|=tmp;
	tmp=0;

	tmp=reply_info_type;
	res|=tmp;
	tmp=0;

	tmp=message_type;
	tmp<<=8;
	res|=tmp;
	tmp=0;

	return res;
}
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//  /* Get RX message */
//  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, Rxbuffer);
//  if(Rxbuffer[0] == 0x12)
//  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//      //uint8_t  buffer1[] = "Fun";
//      //uint8_t ack = 12;
//      //HAL_UART_Transmit(&huart2, &ack, sizeof(ack), 0x2);
//
//}
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
//{
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}
//void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
//{
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}
//void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
//{
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
