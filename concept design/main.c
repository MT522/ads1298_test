/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ECG_FIR_Filter.h"
#include "ADS1x98.h"
#include "Net_Config_PPP.h"
#include "rl_net.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum  {
	InitDAQ = 0,
	TestDAQ = 1,
	GetDAQInfo = 52, /* "4" for test */
	StartSendECG = 49, /* "1" for test */
	StopSendECG = 48, /* "0" for test */
	EnablePaceDetection = 50, /* "2" for test */
	DisablePaceDetection = 51, /* "3" for test */
}Commands;

typedef enum {
	ACKOK = 0,
	BadCommandNum = 1,
	BadFlag = 2,
	BadDataLength = 3,
	BadData = 4,
	ResendPacket = 5,
	None = 0x80,
}Flags;

typedef struct _CommandPacket {
	Commands Command;
	Flags flag;
	uint8_t *data;
	uint8_t len;
}CommandPacket;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* Definitions for LEDStausTask */
osThreadId_t LEDStausTaskHandle;
const osThreadAttr_t LEDStausTask_attributes = {
  .name = "LEDStausTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for SendECGDataTask */
osThreadId_t SendECGDataTaskHandle;
const osThreadAttr_t SendECGDataTask_attributes = {
  .name = "SendECGDataTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for PrepareECGDataTask */
osThreadId_t PrepareECGDataTaskHandle;
const osThreadAttr_t PrepareECGDataTask_attributes = {
  .name = "PrepareECGDataTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for DoCommandTask */
osThreadId_t DoCommandTaskHandle;
const osThreadAttr_t DoCommandTask_attributes = {
  .name = "DoCommandTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for ToDoCommandBinarySem */
osSemaphoreId_t ToDoCommandBinarySemHandle;
const osSemaphoreAttr_t ToDoCommandBinarySem_attributes = {
  .name = "ToDoCommandBinarySem"
};
/* Definitions for RawECGCountingSem */
osSemaphoreId_t RawECGCountingSemHandle;
const osSemaphoreAttr_t RawECGCountingSem_attributes = {
  .name = "RawECGCountingSem"
};
/* Definitions for FinalECGCountingSem */
osSemaphoreId_t FinalECGCountingSemHandle;
const osSemaphoreAttr_t FinalECGCountingSem_attributes = {
  .name = "FinalECGCountingSem"
};
/* USER CODE BEGIN PV */
extern uint8_t RawECGBuffer[RAW_ECG_BUFFER_SIZE];
uint16_t RawECGBufferReadSampleNum = 0;
uint32_t RawECGBufferReadIndex = 0;
extern uint16_t RawECGBufferWriteSampleNum;
extern uint8_t IsStartedSampling;
uint16_t RawECGBufferdSampleNum;
extern uint32_t RawECGBufferWriteIndex;
netStatus gNetworkStatus = netError;
netStatus gDataSocketStatus = netError;
netStatus gCommandSocketStatus = netError;
netStatus gSendStatus = netError;
uint8_t gWaitedEnoughSocket = 0;
int32_t gDataSocket = 0;
int32_t gCommandSocket = 0;
NET_ADDR4 ECGDataAddress = { NET_ADDR_IP4, DATA_PRT, 180, 180, 1, 1 };
NET_ADDR4 CommandAddress = { NET_ADDR_IP4, COMM_PRT, 180, 180, 1, 1 };
uint8_t FilterBlockSize = (uint8_t)BLOCK_SIZE_FLOAT;
uint8_t DownSamplingBlockSize = (uint8_t)DOWN_SAMPLE_BLOCK_SIZE;
uint8_t SamplePerPacket = SEND_ECG_SAMPLE_PER_PACKET;
extern int8_t FilteredECGBuffer[BLOCK_SIZE_FLOAT*FINAL_ECG_SAMPLE_SIZE/DOWN_SAMPLE_BLOCK_SIZE];
uint8_t FinalECGBuffer[FINAL_ECG_BUFFER_SIZE];
uint16_t FinalECGBufferWriteSampleNum = 0;
uint16_t FinalECGBufferReadSampleNum = 0;
uint32_t FinalECGBufferWriteIndex = 0;
uint32_t FinalECGBufferReadIndex = 0;
uint16_t FinalECGBufferdSampleNum;
uint8_t RecievedData[10];
uint8_t SendData[10];
CommandPacket ToDoCommand;
CommandPacket DoneCommand;
//CommandPacket CommandBuffer[10];
//uint8_t CommandToDoIndex = 0;
//uint8_t CommandDoneIndex = 0;
bool PaceDetection = false;
uint8_t DAQInfo[10];
#ifdef VER1TEST
uint8_t TestBuffer[SEND_ECG_PACKET_SIZE] = {48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 48, 49, 50, 51, 10};
uint16_t TestBufferIndex = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void StartLEDStatusTask(void *argument);
void StartSendECGTask(void *argument);
void StartPrepareECGTask(void *argument);
void StartDoCommandTask(void *argument);

/* USER CODE BEGIN PFP */
void ResetRawECGBuff(void);
void ResetFinalECGBuff(void);
uint32_t udp_data_cb_func (int32_t socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len);
uint32_t udp_command_cb_func (int32_t socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len);
void DoNewCommand (CommandPacket NewCommand);
void AnswerCommand(void);

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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	while(gNetworkStatus != netOK)
	{
		gNetworkStatus = netInitialize();
		osDelay(100);
	}
	#ifdef UART_DEBUGING
	char Message[] = "Network is initialized.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
	ADS1x9x_PowerOn_Init();
	InitFIRFilters();
//	Soft_Start_ADS1x9x();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ToDoCommandBinarySem */
  ToDoCommandBinarySemHandle = osSemaphoreNew(1, 0, &ToDoCommandBinarySem_attributes);

  /* creation of RawECGCountingSem */
  RawECGCountingSemHandle = osSemaphoreNew(RAW_ECG_BUFFER_FULL_SAMPLE_NUM, 0, &RawECGCountingSem_attributes);

  /* creation of FinalECGCountingSem */
  FinalECGCountingSemHandle = osSemaphoreNew(FINAL_ECG_BUFFER_FULL_SAMPLE_NUM, 0, &RawECGCountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDStausTask */
  LEDStausTaskHandle = osThreadNew(StartLEDStatusTask, NULL, &LEDStausTask_attributes);

  /* creation of SendECGDataTask */
  SendECGDataTaskHandle = osThreadNew(StartSendECGTask, NULL, &SendECGDataTask_attributes);

  /* creation of PrepareECGDataTask */
  PrepareECGDataTaskHandle = osThreadNew(StartPrepareECGTask, NULL, &PrepareECGDataTask_attributes);

  /* creation of DoCommandTask */
  DoCommandTaskHandle = osThreadNew(StartDoCommandTask, NULL, &DoCommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_5);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|ADS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADS_CS_Pin|ADS_START_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_DRDY_Pin */
  GPIO_InitStruct.Pin = ADS_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADS_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADS_CS_Pin ADS_START_Pin */
  GPIO_InitStruct.Pin = ADS_CS_Pin|ADS_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_RST_Pin */
  GPIO_InitStruct.Pin = ADS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADS_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Send a message on UART
  * @param  argument: Not used
  * @retval None
  */
void SendDebugMessage(char *pDebugMessage, uint8_t size)
{
	uint8_t data[100];
	memcpy(&data[0], pDebugMessage, size);
	data[size] = 0x0A;	//"\r"
	data[size+1] = 0x0D;	//"\n"
	HAL_UART_Transmit(&huart4, &data[0], size+2, 10);
	/* Send debug info on SWV */
	/* Not working!! */
	for(uint8_t i=0; i<(size+2); i++){
		ITM_SendChar(data[i]);
	}
}

/**
  * @brief  Reset Raw ECG buffer
  * @param  None
  * @retval None
  */
void ResetRawECGBuff(void)
{
	RawECGBufferWriteSampleNum = 0;
	RawECGBufferReadSampleNum = 0;
	RawECGBufferdSampleNum = 0;
	RawECGBufferWriteIndex = 0;
	RawECGBufferReadIndex = 0;
	memset(&RawECGBuffer[0], 0x00, RAW_ECG_BUFFER_SIZE);
	osSemaphoreDelete(RawECGCountingSemHandle);
	RawECGCountingSemHandle = osSemaphoreNew(RAW_ECG_BUFFER_FULL_SAMPLE_NUM, 0, &RawECGCountingSem_attributes);
	IsStartedSampling = 0;
	#ifdef UART_DEBUGING
	char Message[] = "Reset raw ECG buffer!!";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
}

/**
  * @brief  Reset Final ECG buffer
  * @param  None
  * @retval None
  */
void ResetFinalECGBuff(void)
{
	FinalECGBufferWriteSampleNum = 0;
	FinalECGBufferReadSampleNum = 0;
	FinalECGBufferdSampleNum = 0;
	FinalECGBufferWriteIndex = 0;
	FinalECGBufferReadIndex = 0;
	memset(&FinalECGBuffer[0], 0x00, FINAL_ECG_BUFFER_SIZE);
	osSemaphoreDelete(FinalECGCountingSemHandle);
	FinalECGCountingSemHandle = osSemaphoreNew(FINAL_ECG_BUFFER_FULL_SAMPLE_NUM, 0, &FinalECGCountingSem_attributes);
	IsStartedSampling = 0;
	#ifdef UART_DEBUGING
	char Message[] = "Reset Final ECG buffer!! This is so bad!!";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
}

/**
  * @brief  UDP callback
  * @param  socket 
  * @param  addr 
	* @param  buf
	* @param  len
  * @retval 0
  */
uint32_t udp_data_cb_func (int32_t socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len) 
{
	#ifdef UART_DEBUGING
	char Message[] = "Receive packet over UDP.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
  return (0);
}

/**
  * @brief  UDP callback
  * @param  socket 
  * @param  addr 
	* @param  buf
	* @param  len
  * @retval 0
  */
uint32_t udp_command_cb_func (int32_t socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len) 
{
	#ifdef UART_DEBUGING
	char Message[] = "Receive packet over UDP.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
	#ifdef VER2BETA
	memset(RecievedData, 0x00, 10);
	memcpy(RecievedData, buf, len);
	/* simple Stop/Start with 0/1 char recieved */
	if(RecievedData[0] == 0x30){
		Soft_Stop_ADS1x9x();
		/* Reset raw ECG buffer*/
		ResetRawECGBuff();
		ResetFinalECGBuff();
	}
	if(RecievedData[0] == 0x31){
		Soft_Start_ADS1x9x();
	}
	#endif
	#ifdef VER2
	if(len > 2)
	{
		if(osOK == osSemaphoreRelease(ToDoCommandBinarySemHandle))
		{
			memset(RecievedData, 0x00, 10);
			memcpy(RecievedData, buf, len);
			ToDoCommand.Command = (Commands) RecievedData[0];
			ToDoCommand.flag = (Flags) RecievedData[1];
			ToDoCommand.len = len - 2;
			if (ToDoCommand.len){
				ToDoCommand.data = &RecievedData[2];
			}
		}
		else
		{
			#ifdef UART_DEBUGING
			char Message[] = "DAQ is busy!!";
			SendDebugMessage(&Message[0], sizeof(Message)-1);
			#endif
		}
	}
	else
	{
		/* TO DO */
	}
	#endif
  return (0);
}

/**
  * @brief  Do new command.
  * @param  NewCommand: new command structure
  * @retval None
  */
void DoNewCommand (CommandPacket NewCommand)
{
	switch (NewCommand.Command)
	{
		case InitDAQ:
			/* TO DO */
		break;

		case TestDAQ:
			/* TO DO */
		break;
		
		case GetDAQInfo:
			/* TO DO */
			/* ADS ID number cannot get during sampling */
			if(IsStartedSampling == 0)
			{
				DAQInfo[0] = GetADSId();
			}
			DoneCommand.Command = ToDoCommand.Command;
			DoneCommand.flag = ACKOK;
			DoneCommand.data = DAQInfo;
			DoneCommand.len = 1;
		break;
		
		case StartSendECG:
			/* Start sampling ECG*/
			Soft_Start_ADS1x9x();
			DoneCommand.Command = ToDoCommand.Command;
			DoneCommand.flag = ACKOK;
			DoneCommand.data = NULL;
			DoneCommand.len = 0;
		break;
		
		case StopSendECG:
			/* Stop sampling ECG*/
			Soft_Stop_ADS1x9x();
			/* Need a time to really stop ADS */
			osDelay(5);
			/* Reset raw & final ECG buffers*/
			ResetRawECGBuff();
			ResetFinalECGBuff();
			DoneCommand.Command = ToDoCommand.Command;
			DoneCommand.flag = ACKOK;
			DoneCommand.data = NULL;
			DoneCommand.len = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		break;
		
		case EnablePaceDetection:
			/* Enable Pace detection function */
			PaceDetection = true;
			DoneCommand.Command = ToDoCommand.Command;
			DoneCommand.flag = ACKOK;
			DoneCommand.data = NULL;
			DoneCommand.len = 0;
		break;
		
		case DisablePaceDetection:
			/* Disable Pace detection function */
			PaceDetection = false;
			DoneCommand.Command = ToDoCommand.Command;
			DoneCommand.flag = ACKOK;
			DoneCommand.data = NULL;
			DoneCommand.len = 0;
		break;
		
		default:
			DoneCommand.Command = ToDoCommand.Command;
			DoneCommand.flag = BadCommandNum;
			DoneCommand.data = NULL;
			DoneCommand.len = 0;
			#ifdef UART_DEBUGING
			char Message[] = "BadCommandNum!!";
			SendDebugMessage(&Message[0], sizeof(Message)-1);
			#endif
		break;
	}
	AnswerCommand();
}

/**
  * @brief  Send command answer.
  * @param  None
  * @retval None
  */
void AnswerCommand(void)
{
	uint8_t *send_buffer;
	uint8_t len;
	netStatus SendStatus;
	if(netPPP_LinkUp() == true)
	{
		SendData[0] = DoneCommand.Command;
		SendData[1] = DoneCommand.flag;
		if(DoneCommand.len)
		{
			memcpy(&SendData[2], DoneCommand.data, DoneCommand.len);
		}
		len = DoneCommand.len + 2;
		send_buffer = netUDP_GetBuffer(len);
		if (send_buffer != NULL)
		{
			memcpy(send_buffer, &SendData[0], len);
			SendStatus = netUDP_Send(gCommandSocket, (NET_ADDR*)&CommandAddress, send_buffer, len);
			/* have a error in sending UDP packet */
			if(SendStatus)
			{
				#ifdef UART_DEBUGING
				char Message[] = "Error in sending Command packet!!";
				SendDebugMessage(&Message[0], sizeof(Message)-1);
				#endif
			}
			/* Send a packet successfully */
			else
			{
				if(osOK == osSemaphoreAcquire(ToDoCommandBinarySemHandle, osWaitForever))
				{
					#ifdef UART_DEBUGING
					char Message[] = "Send command answer successfully.";
					SendDebugMessage(&Message[0], sizeof(Message)-1);
					#endif
				}
			}
		}
	}
}

/**
  * @brief  Simulate GetDAQInfo command to respond it in the beginning of run.
  * @param  None
  * @retval None
  */
void SendDAQInfoInStarting(void)
{
	if(osOK == osSemaphoreRelease(ToDoCommandBinarySemHandle))
	{
		ToDoCommand.Command = GetDAQInfo;
		ToDoCommand.flag = ACKOK;
		ToDoCommand.len = 0;
		if (ToDoCommand.len){
			ToDoCommand.data = &RecievedData[2];
		}
	}
	else
	{
		#ifdef UART_DEBUGING
		char Message[] = "DAQ is busy!!";
		SendDebugMessage(&Message[0], sizeof(Message)-1);
		#endif
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLEDStatusTask */
/**
  * @brief  Function implementing the LEDStausTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDStatusTask */
void StartLEDStatusTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	#ifdef UART_DEBUGING
	char Message[] = "LEDStatusTask is started.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
  /* Infinite loop */
  for(;;)
  {
		/* check PPP link */
		if(netPPP_LinkUp() == false)
		{
			/* build PPP connection */
			netPPP_Connect("", "", "");
			#ifdef UART_DEBUGING
			char Message[] = "Try to up PPP.";
			SendDebugMessage(&Message[0], sizeof(Message)-1);
			#endif
			/* check socket availability */
			if (gDataSocket != 0)
			{
				netUDP_Close(gDataSocket);
				netUDP_ReleaseSocket(gDataSocket);
				gDataSocket = 0;
				gDataSocketStatus = netError;
				#ifdef UART_DEBUGING
				char Message[] = "Close previous dead data socket.";
				SendDebugMessage(&Message[0], sizeof(Message)-1);
				#endif
			}
			if (gCommandSocket != 0)
			{
				netUDP_Close(gCommandSocket);
				netUDP_ReleaseSocket(gCommandSocket);
				gCommandSocket = 0;
				gCommandSocketStatus = netError;
				#ifdef UART_DEBUGING
				char Message[] = "Close previous dead command socket.";
				SendDebugMessage(&Message[0], sizeof(Message)-1);
				#endif
			}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			osDelay(250);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			osDelay(250);
		}
		else 
		{
			/* build a command socket for a first time*/
			if (gCommandSocket == 0)
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				/* check socket is built OK */
				while (gCommandSocket <= 0)
				{
					gCommandSocket = netUDP_GetSocket(udp_command_cb_func);
					osDelay(50);
				}
				#ifdef UART_DEBUGING
				char Message[] = "PPP is up. Get a new command socket.";
				SendDebugMessage(&Message[0], sizeof(Message)-1);
				#endif
				/* check UDP port is opened OK */
				while (gCommandSocketStatus != netOK)
				{
					/* Open a UDP port */
					gCommandSocketStatus = netUDP_Open(gCommandSocket, COMM_PRT);
					//netUDP_SetOption (socket, netUDP_OptionTTL, 2);
					osDelay(100);
				}			
			}
			/* build a data socket for a first time*/
			if (gDataSocket == 0)
			{
				/* check socket is built OK */
				while (gDataSocket <= 0)
				{
					gDataSocket = netUDP_GetSocket(udp_data_cb_func);
					osDelay(50);
				}
				#ifdef UART_DEBUGING
				char Message[] = "Get a new data socket.";
				SendDebugMessage(&Message[0], sizeof(Message)-1);
				#endif
				/* check UDP port is opened OK */
				while (gDataSocketStatus != netOK)
				{
					/* Open a UDP port */
					gDataSocketStatus = netUDP_Open(gDataSocket, DATA_PRT);
					//netUDP_SetOption (socket, netUDP_OptionTTL, 2);
					osDelay(2000);
				}
				#ifdef VER1TEST
				osDelay(5000);
				#endif
				#ifdef VER1
				/* Start sampling ECG*/
				Soft_Start_ADS1x9x();
				#endif
				gWaitedEnoughSocket = 1;
				/* wait to peer start to listen */
				osDelay(3000);
				/* send DAQ info */
				SendDAQInfoInStarting();
			}
			osDelay(1000);
		} 		
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSendECGTask */
/**
* @brief Function implementing the SendECGDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendECGTask */
void StartSendECGTask(void *argument)
{
  /* USER CODE BEGIN StartSendECGTask */
	#ifdef UART_DEBUGING
	osDelay(10);
	char Message[] = "SendECGTask is started.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
	uint8_t *send_buffer;
	uint16_t extra_sample_num;
	uint8_t extra_packet_num;
  /* Infinite loop */
  for(;;)
  {
		if(netPPP_LinkUp() == false)
		{
			gWaitedEnoughSocket = 0;
			osDelay(50);
		}
		FinalECGBufferdSampleNum = osSemaphoreGetCount(FinalECGCountingSemHandle);
		if(FinalECGBufferdSampleNum >= FINAL_ECG_BUFFER_FULL_SAMPLE_NUM)
		{
			/* Buffer full! */
			/* if we be here, data sending is slow and must become faster!! */
			#ifdef UART_DEBUGING
			char Message[] = "FinalECGBuffer is full!!";
			SendDebugMessage(&Message[0], sizeof(Message)-1);
			#endif
			/* TO DO */
			/* stop samplenig ECG data */
			Soft_Stop_ADS1x9x();
			/* Reset final ECG buffer*/
			ResetFinalECGBuff();
		}
		if(FinalECGBufferdSampleNum >= (FINAL_ECG_L_BUFFER_FULL_SAMPLE_NUM + SEND_ECG_SAMPLE_PER_PACKET))
		{
			/* save ECG data for FINAL_ECG_L_BUFFER_TIME seconds */
//			#ifdef UART_DEBUGING
//			char Message[] = "LittleECGBuffer is full.";
//			SendDebugMessage(&Message[0], sizeof(Message)-1);
//			#endif
			/* TO DO */
			/* remove extra samples. */
			/* Note: extra samples MUST be multiple of SEND_ECG_SAMPLE_PER_PACKET */
			extra_packet_num = (FinalECGBufferdSampleNum - FINAL_ECG_L_BUFFER_FULL_SAMPLE_NUM) / SEND_ECG_SAMPLE_PER_PACKET;
			extra_sample_num = extra_packet_num * SEND_ECG_SAMPLE_PER_PACKET;
			for(uint8_t i=0; i<extra_sample_num; i++)
			{
				if(osOK == osSemaphoreAcquire(FinalECGCountingSemHandle, osWaitForever))
				{
					FinalECGBufferReadIndex = FinalECGBufferReadIndex + FINAL_ECG_SAMPLE_SIZE;
					FinalECGBufferReadSampleNum++;
					FinalECGBufferdSampleNum--;
					if(FinalECGBufferReadIndex == FINAL_ECG_BUFFER_SIZE)
					{
						FinalECGBufferReadIndex = 0; /* reset circular array index */
					}
					if(FinalECGBufferReadSampleNum == FINAL_ECG_BUFFER_FULL_SAMPLE_NUM)
					{
						FinalECGBufferReadSampleNum = 0; /* reset circular array index */
						#ifdef UART_DEBUGING
						char Message[] = "Reset FinalECGBuffer read index.";
						SendDebugMessage(&Message[0], sizeof(Message)-1);
						#endif
						#ifdef VER1TEST
						TestBufferIndex++;
						if(TestBufferIndex == 6) /* testing for N times the size of the final buffer  */
						{
							/* stop samplenig ECG data */
							Soft_Stop_ADS1x9x();
							/* Reset final ECG buffer*/
							ResetFinalECGBuff();
						}
						#endif
					}
				}
			}
		}
		if(IsStartedSampling && gWaitedEnoughSocket)
		{
			/* check enough sample is ready to send in a packet */
			if(FinalECGBufferdSampleNum >= SEND_ECG_SAMPLE_PER_PACKET)
			{
//				HAL_UART_Transmit(&huart5, &RawECGBuffer[RawECGBufferReadIndex], RAW_ECG_SAMPLE_SIZE, 10);
				if(FinalECGBufferdSampleNum == 0)
				{
					#ifdef UART_DEBUGING
					char Message[] = "FinalECGBuffer is Empty!!";
					SendDebugMessage(&Message[0], sizeof(Message)-1);
					#endif
				}
				if( (gWaitedEnoughSocket == 1) && (gDataSocketStatus == netOK) && (netPPP_LinkUp() == true) )
				{
					send_buffer = netUDP_GetBuffer(SEND_ECG_PACKET_SIZE);
					
					if (send_buffer != NULL)
					{
	//					osMutexWait(MutexFillDataID, osWaitForever);
						#ifndef VER1TEST
						memcpy(send_buffer, &FinalECGBuffer[FinalECGBufferReadIndex], SEND_ECG_PACKET_SIZE);
						#endif
						#ifdef VER1TEST
						memcpy(send_buffer, &TestBuffer[0], SEND_ECG_PACKET_SIZE);
						#endif
						gSendStatus = netUDP_Send(gDataSocket, (NET_ADDR*)&ECGDataAddress, send_buffer, SEND_ECG_PACKET_SIZE);
						/* have a error in sending UDP packet */
						if(gSendStatus)
						{
							#ifdef UART_DEBUGING
							char Message[] = "Error in sending data packet!!";
							SendDebugMessage(&Message[0], sizeof(Message)-1);
							#endif
						}
						/* Send a packet successfully */
						else
						{
							FinalECGBufferReadIndex = FinalECGBufferReadIndex + SEND_ECG_PACKET_SIZE;
							FinalECGBufferReadSampleNum = FinalECGBufferReadSampleNum + SEND_ECG_SAMPLE_PER_PACKET;
							if((FinalECGBufferReadSampleNum % 500) == 0)
							{
								HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
							}
							for(uint8_t i=0; i<SamplePerPacket; i++)
							{
								if(osOK == osSemaphoreAcquire(FinalECGCountingSemHandle, osWaitForever))
								{
									FinalECGBufferdSampleNum--;
								}
							}
						}
	//					osMutexRelease(MutexFillDataID);
					}
				}
				else
				{
					FinalECGBufferdSampleNum = 0;
				}
				if(FinalECGBufferReadIndex == FINAL_ECG_BUFFER_SIZE)
				{
					FinalECGBufferReadIndex = 0; /* reset circular array index */
				}
				if(FinalECGBufferReadSampleNum == FINAL_ECG_BUFFER_FULL_SAMPLE_NUM)
				{
					FinalECGBufferReadSampleNum = 0; /* reset circular array index */
					#ifdef UART_DEBUGING
					char Message[] = "Reset FinalECGBuffer read index.";
					SendDebugMessage(&Message[0], sizeof(Message)-1);
					#endif
					#ifdef VER1TEST
					TestBufferIndex++;
					if(TestBufferIndex == 6) /* testing for N times the size of the final buffer  */
					{
						/* stop samplenig ECG data */
						Soft_Stop_ADS1x9x();
						/* Reset final ECG buffer*/
						ResetFinalECGBuff();
					}
					#endif
				}
			}
		}
    osDelay(MIN_DELAY_TO_SEND_ECG_PACKET);
  }
  /* USER CODE END StartSendECGTask */
}

/* USER CODE BEGIN Header_StartPrepareECGTask */
/**
* @brief Function implementing the PrepareECGDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrepareECGTask */
void StartPrepareECGTask(void *argument)
{
  /* USER CODE BEGIN StartPrepareECGTask */
	#ifdef UART_DEBUGING
	osDelay(5);
	char Message[] = "PrepareECGTask is started.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
	int8_t LeadOff[FilterBlockSize];
  /* Infinite loop */
  for(;;)
  {
		if(IsStartedSampling)
		{
			RawECGBufferdSampleNum = osSemaphoreGetCount(RawECGCountingSemHandle);
			/* check for safe threshold (4 samples) for reading from RawECGBuffer */
			while(RawECGBufferdSampleNum >= FilterBlockSize)
			{
				if(RawECGBufferdSampleNum >= RAW_ECG_BUFFER_FULL_SAMPLE_NUM)
				{
					/* Buffer full! */
					#ifdef UART_DEBUGING
					char Message[] = "RawECGBuffer is full!!";
					SendDebugMessage(&Message[0], sizeof(Message)-1);
					#endif
					/* TO DO */
					/* stop samplenig ECG data */
					Soft_Stop_ADS1x9x();
					/* Reset raw ECG buffer*/
					ResetRawECGBuff();
				}
				if(RawECGBufferdSampleNum == 0)
				{
					#ifdef UART_DEBUGING
					char Message[] = "RawECGBuffer is Empty!!";
					SendDebugMessage(&Message[0], sizeof(Message)-1);
					#endif
				}
				memset(&LeadOff[0], 0x00, FilterBlockSize);
				/* Convert raw ECG data to 8 channel float arrays */
				for (int i=0; i<FilterBlockSize; i++ ){
					ConvertIntegerSampleToFloat((uint8_t*)&RawECGBuffer[RawECGBufferReadIndex], i);
					/* TO DO */
					/* extract Lead Off data from raw ECG data */
					LeadOff[i] = ( (RawECGBuffer[RawECGBufferReadIndex]<<4) & 0xFF) | ( (RawECGBuffer[RawECGBufferReadIndex + 1]>>4) & 0xFF) | ( (RawECGBuffer[RawECGBufferReadIndex + 1]<<4) & 0xFF) | ( (RawECGBuffer[RawECGBufferReadIndex + 2]>>4) & 0xFF);
		//			memcpy(&ECGFilteredDataForSend[SAMPLE_SIZE_IN_BYTES*i], &RawSimulatorECG2[26*i], 2);
		//			memset(&ECGFilteredDataForSend[26*i], (pack_num % 250), 2);
					RawECGBufferReadSampleNum++;
					RawECGBufferReadIndex = RawECGBufferReadIndex + RAW_ECG_SAMPLE_SIZE;
					if(osOK == osSemaphoreAcquire(RawECGCountingSemHandle, osWaitForever))
					{
						RawECGBufferdSampleNum--;
					}
				}
				if(RawECGBufferReadIndex == RAW_ECG_BUFFER_SIZE)
				{
					RawECGBufferReadIndex = 0; /* reset circular array index */
				}
				if(RawECGBufferReadSampleNum == RAW_ECG_BUFFER_FULL_SAMPLE_NUM)
				{
					RawECGBufferReadSampleNum = 0; /* reset circular array index */
				}
				/* Filter ECG data */
				FilterECGData();
				for (int8_t i = 0; i < (FilterBlockSize/DownSamplingBlockSize); i++)
				{
					/* check Pace detection function is enabaled */
					if(PaceDetection)
					{
						/* detect PACE */
						FilteredECGBuffer[DOWN_SAMPLED_DATA_SIZE*FINAL_ECG_SAMPLE_SIZE*i] = DetectPaceWithSlewRate(1 /* LEAD II*/,i,0.00018/**4/2.4*/); /* Ch1: LEAD II is a best lead for pace detection. */
					}
					/* detect LeadOff */
					FilteredECGBuffer[(DOWN_SAMPLED_DATA_SIZE*FINAL_ECG_SAMPLE_SIZE*i) + 1] = LeadOff[i] | LeadOff[i+1] | LeadOff[i+2] | LeadOff[i+3];
				}
				/* down sample to 500 sample/s */
				DownSample2000To500();
				/* Arrange data in 19 Bytes every samples */
				Arrange16BitDataForSend();
				/* Store a new sample in FinalECGBuffer */
				memcpy(&FinalECGBuffer[FinalECGBufferWriteIndex], &FilteredECGBuffer[0], FINAL_ECG_SAMPLE_SIZE);
				if(osOK == osSemaphoreRelease(FinalECGCountingSemHandle))
				{
					FinalECGBufferWriteIndex = FinalECGBufferWriteIndex + FINAL_ECG_SAMPLE_SIZE;
					FinalECGBufferWriteSampleNum++;
				}
				if(FinalECGBufferWriteIndex == FINAL_ECG_BUFFER_SIZE)
				{
					FinalECGBufferWriteIndex = 0; /* reset circular array index */
				}
				if(FinalECGBufferWriteSampleNum == FINAL_ECG_BUFFER_FULL_SAMPLE_NUM)
				{
					FinalECGBufferWriteSampleNum = 0; /* reset circular array index */
				}
			}
		}
    osDelay(1);
  }
  /* USER CODE END StartPrepareECGTask */
}

/* USER CODE BEGIN Header_StartDoCommandTask */
/**
* @brief Function implementing the DoCommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDoCommandTask */
void StartDoCommandTask(void *argument)
{
  /* USER CODE BEGIN StartDoCommandTask */
	#ifdef UART_DEBUGING
	char Message[] = "DoCommandTask is started.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
	/* Infinite loop */
  for(;;)
  {
		/* wait for a new command */
		if(osSemaphoreGetCount(ToDoCommandBinarySemHandle))
		{
			DoNewCommand(ToDoCommand);
		}

    osDelay(1);
  }
  /* USER CODE END StartDoCommandTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
