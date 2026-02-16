/**
  ******************************************************************************
  * @file           : ADS1x98.c
  * @brief          : ADS1198 and ADS1298
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include "ADS1x98.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Global variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern osSemaphoreId_t RawECGCountingSemHandle;


/* Private variables ---------------------------------------------------------*/
uint8_t SPI_Tx_buf[10];
uint8_t SPI_Rx_buf[10];
uint8_t ADS_Buffer[30];
uint8_t SPI_SendtoRecieveByte[27] = {0xFF};
uint8_t RawECGBuffer[RAW_ECG_BUFFER_SIZE];
uint16_t RawECGBufferWriteSampleNum = 0;
uint32_t RawECGBufferWriteIndex = 0;
uint8_t IsStartedSampling = 0;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/


/**
  * @brief Enable ADS SPI Chip Select
  * @param None
  * @retval None
  */
void AssertADS_CS(void)
{
  HAL_GPIO_WritePin(GPIOA, ADS_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief Disable ADS SPI Chip Select
  * @param None
  * @retval None
  */
void DeAssertADS_CS(void)
{
  HAL_GPIO_WritePin(GPIOA, ADS_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief Assert ADS Chip
  * @param None
  * @retval None
  */
void AssertADS_Reset(void)
{
  HAL_GPIO_WritePin(GPIOA, ADS_RST_Pin, GPIO_PIN_RESET);
}

/**
  * @brief DeAssert ADS Chip
  * @param None
  * @retval None
  */
void DeAssertADS_Reset(void)
{
  HAL_GPIO_WritePin(GPIOA, ADS_RST_Pin, GPIO_PIN_SET);
}

/**
  * @brief Start Conversion with ADS 
  * @param None
  * @retval None
  */
void ADS1x9x_Enable_Start(void)
{
	HAL_GPIO_WritePin(GPIOA, ADS_START_Pin, GPIO_PIN_SET);
	osDelay(10);
}

/**
  * @brief Stop Conversion with ADS 
  * @param None
  * @retval None
  */
void ADS1x9x_Disable_Start(void)
{
	HAL_GPIO_WritePin(GPIOA, ADS_START_Pin, GPIO_PIN_RESET);
	osDelay(7);
}

/**
  * @brief Reset ADS chip 
  * @param None
  * @retval None
  */
void ADS1x9x_Reset(void)
{
	DeAssertADS_Reset();
	/* Provide suficient dealy*/
	osDelay(1);
	AssertADS_Reset();
	osDelay(1);
	DeAssertADS_Reset();
	osDelay(7);
}

/**
  * @brief Enable ADS Data Ready Interrupts
  * @param None
  * @retval None
  */
void Enable_ADS1x9x_DRDY_Interrupt (void)
{
	/* Enable interrupt in core for  odd gpio interrupts */
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief Disable ADS Data Ready Interrupts
  * @param None
  * @retval None
  */
void Disable_ADS1x9x_DRDY_Interrupt (void)
{
	/* Disable interrupt in core for  odd gpio interrupts */
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief Transmit a Command to ADS
  * @param Command Command for Transmit
  * @retval None
  */
void ADS1x9x_SPI_Command_Data(uint8_t Command)
{
	AssertADS_CS(); // Enable ADS1x9x CS
	HAL_SPI_Transmit(&hspi2, &Command, 1, 10);
	//DeAssertADS_CS(); // Enable ADS1x9x CS
}

/**
  * @brief Wake Up ADS from Standby Mode
  * @param None 
  * @retval None
  */
void Wake_Up_ADS1x9x (void)
{ 
    ADS1x9x_SPI_Command_Data (WAKEUP);	// Send 0x02 to the ADS1x9x                                                      
}

/**
  * @brief Put ADS in Standby Mode
  * @param None
  * @retval None
  */
void Put_ADS1x9x_In_Sleep (void)
{
    ADS1x9x_SPI_Command_Data (STANDBY);	// Send 0x04 to the ADS1x9x
}

/**
  * @brief Reset ADS Chip
  * @param None
  * @retval None
  */
void Soft_Reset_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (RESET);	// Send 0x06 to the ADS1x9x
}

/**
  * @brief Start/Restart (Synchronize) Conversions
  * @param None
  * @retval None
  */
void Soft_Start_ReStart_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (START);	// Send 0x08 to the ADS1x9x
}

/**
  * @brief Start/Restart (Synchronize) Conversions Physically
  * @param None
  * @retval None
  */
void Hard_Start_ReStart_ADS1x9x(void)
{
	ADS1x9x_Enable_Start();	// Set Start pin to High
}

/**
  * @brief Start/Restart (Synchronize) Conversions and Be Ready to Get Data
  * @param None
  * @retval None
  */
void Soft_Start_ADS1x9x (void)
{
	ADS1x9x_SPI_Command_Data (START);	// Send 0x08 to the ADS1x9x
	Enable_ADS1x9x_DRDY_Interrupt();
	#ifdef UART_DEBUGING
	char Message[] = "Start ECG sampling.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
}

/**
  * @brief Stop (Synchronize) Conversions and Be UnReady to Get Data
  * @param None
  * @retval None
  */
void Soft_Stop_ADS1x9x (void)
{
	ADS1x9x_SPI_Command_Data (STOP);	// Send 0x0A to the ADS1x9x
	Disable_ADS1x9x_DRDY_Interrupt();
	#ifdef UART_DEBUGING
	char Message[] = "Stop ECG sampling.";
	SendDebugMessage(&Message[0], sizeof(Message)-1);
	#endif
}

/**
  * @brief Stop (Synchronize) Conversions Physically
  * @param None
  * @retval None
  */
void Hard_Stop_ADS1x9x (void)
{
	ADS1x9x_Disable_Start();	// Set Start pin to Low
	osDelay(14);
}

/**
  * @brief Enable Read Data Continuous Mode.
  * @param None
  * @retval None
  */
void Start_Read_Data_Continuous (void)
{
    ADS1x9x_SPI_Command_Data (RDATAC);	// Send 0x10 to the ADS1x9x
}

/**
  * @brief Stop Read Data Continuously Mode.
  * @param None
  * @retval None
  */
void Stop_Read_Data_Continuous (void)
{
    ADS1x9x_SPI_Command_Data(SDATAC);	// Send 0x11 to the ADS1x9x
}

/**
  * @brief Read Data by Command.
  * @param None
  * @retval None
  */
void Read_Data_by_Command (void)
{
    ADS1x9x_SPI_Command_Data(RDATA);	// Send 0x12 to the ADS1x9x
}

/**
  * @brief Write a Register
  * @param Read_write_address Register Address for Write Opcode
  * @param Data Opcode for Write a Register
  * @retval None
  */
void ADS1x9x_Reg_Write (uint8_t Read_write_address,uint8_t Data)
{ 
	SPI_Tx_buf[0] = Read_write_address | WREG;
	SPI_Tx_buf[1] = 0;						// Write single byte
	SPI_Tx_buf[2] = Data;					// Write single byte
	AssertADS_CS();
	osDelay(10);
	HAL_SPI_Transmit(&hspi2, &SPI_Tx_buf[0], 1, 1);
	for(uint8_t i = 0; i < 100; i++); // wait awhile
	HAL_SPI_Transmit(&hspi2, &SPI_Tx_buf[1], 1, 1);
	for(uint8_t i = 0; i < 100; i++); // wait awhile
	HAL_SPI_Transmit(&hspi2, &SPI_Tx_buf[2], 1, 1);
	//DeAssertADS_CS();
}

/**
  * @brief Read a Register
  * @param Reg_address Register Address for Read
  * @retval Register Data
  */
uint8_t ADS1x9x_Reg_Read(uint8_t Reg_address)
{
	uint8_t retVal;
	SPI_Tx_buf[0] = Reg_address | RREG;
	SPI_Tx_buf[1] = 0;	// Read single register
	AssertADS_CS();	// Set chip select to low
	osDelay(1);
	HAL_SPI_Transmit(&hspi2, &SPI_Tx_buf[0], 1, 1);
	for(uint8_t i = 0; i < 100; i++); // wait awhil
	HAL_SPI_TransmitReceive(&hspi2, &SPI_Tx_buf[1], &SPI_Rx_buf[0], 2, 1);
	retVal = SPI_Rx_buf[1];	// Two first bytes are Transmitted bytes.
//	if(HAL_OK == HAL_SPI_Transmit(&hspi2, &SPI_Tx_buf[0], 2, 10))
//	{
//		if(HAL_OK == HAL_SPI_Receive(&hspi2, &SPI_Rx_buf[0], 1, 10)){
//			retVal = SPI_Rx_buf[0];
//		}
//	}
	//DeAssertADS_CS(); // Set chip select to low
	return 	retVal;

}

/**
  * @brief  Get ADS ID number.
  * @param  None
  * @retval None
  */
uint8_t GetADSId(void)
{
	return (ADS1x9x_Reg_Read(REG_DEVID));
}

/**
  * @brief ADS1x98 Initialization 
  * @param None
  * @retval None
  */
void ADS1x9x_PowerOn_Init(void)
{
	volatile uint8_t i, j;
	uint8_t RegistersData[27];
//	DeAssertADS_PowerDown();
	DeAssertADS_Reset();
	osDelay(1000);
	AssertADS_Reset();
	osDelay(1);
	DeAssertADS_Reset();
	osDelay(7);	
	Soft_Start_ADS1x9x();
	osDelay(1);
	Soft_Stop_ADS1x9x();
	osDelay(1);
	//ADS1x9x_Reset();
	Stop_Read_Data_Continuous(); // SDATAC command
	osDelay(1);
	for(uint8_t i=0; i<26; i++)
	{
		RegistersData[i] = ADS1x9x_Reg_Read(i);
	}
//	TRACE_DEBUG("ADS Device ID is :  0x%X\n", ADS1x9x_Reg_Read(REG_DEVID));
	ADS1x9x_Reg_Write(REG_CONFIG1, 0xA4); //  High-Resolution mode,External Clock Enable, 2000 sample per sec.
	ADS1x9x_Reg_Write(REG_CONFIG2, 0x31); // Internal Test enable, f = 2 Hz
	ADS1x9x_Reg_Write(REG_CONFIG3, 0xCC); // Internal ref buffer,  VREFP is set to 2.4V,  RLDREF signal (AVDD – AVSS)/2 generated internally,  RLD buffer is enabled 
	ADS1x9x_Reg_Write(REG_RLD_SENSP, 0xFF); // RLD as an average of the All channels. 
	ADS1x9x_Reg_Write(REG_RLD_SENSN, 0xFF); // RLD as an average of the All channels.
	ADS1x9x_Reg_Write(REG_PACE, 0x01);     //  PACE detect buffer on,  Channel 1 on PACE_OUT2, Channel 2 on PACE_OUT1
	ADS1x9x_Reg_Write(REG_WCT1, 0x09); //  Power-on WCTA(1) , WCTA(001):RA = IN1N
	ADS1x9x_Reg_Write(REG_WCT2, 0xD0); //  Power-on WCTB(1),Power-on WCTC(1), WCTB(010):LA=IN2P , WCTC(000):LL=IN1P
	
	//Lead off Detection
	ADS1x9x_Reg_Write(REG_LOFF, 0x07); 		 // Comparator threshold at 95% and 5%, Current Source , DC lead-off  
	ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); 	 // Lead-off comparators Enabled ,WCT to RLD connection off,Continuous conversion mode
	ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); // Turn on the P-side of all channels for lead-off sensing
	ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); // Turn off the N-side of all channels for lead-off sensing
	
	// Set All Channels to Internal Test Signal 
	ADS1x9x_Reg_Write(REG_CH1SET, 0x35);  // CH1SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH2SET, 0x35);  // CH2SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH3SET, 0x35);  // CH3SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH4SET, 0x35);  // CH4SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH5SET, 0x35);  // CH5SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH6SET, 0x35);  // CH6SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH7SET, 0x35);  // CH7SET, PGA = 6 , Internal Test Signal
	ADS1x9x_Reg_Write(REG_CH8SET, 0x35);  // CH8SET, PGA = 6 , Internal Test Signal
	
	// Activate Conversion.After This Point Should Toggle at f /16384
//	Init_ADS1x9x_DRDY_Interrupt ();
	Enable_ADS1x9x_DRDY_Interrupt();
	//Enable_PACEn_Interrupt();
	Start_Read_Data_Continuous(); // Put the Device Back in RDATAC Mode
	ADS1x9x_Enable_Start();
	Soft_Start_ReStart_ADS1x9x();
	// Look for ~DRDY and Issue 24 + n * 16 SCLK
	osDelay(500);
	//osDelay(1000);
//	DeAssertADS_Start();
	Soft_Stop_ADS1x9x();
	Stop_Read_Data_Continuous(); // SDATAC command
	osDelay(1);
	//Lead off Detection
	ADS1x9x_Reg_Write(REG_LOFF, 0x07); 		 // Comparator threshold at 95% and 5%, Current Source , DC lead-off  
	ADS1x9x_Reg_Write(REG_CONFIG4, 0x02); 	 // Lead-off comparators Enabled ,WCT to RLD connection off,Continuous conversion mode
	ADS1x9x_Reg_Write(REG_LOFF_SENSP, 0xFF); // Turn on the P-side of all channels for lead-off sensing
	ADS1x9x_Reg_Write(REG_LOFF_SENSN, 0xFF); // Turn off the N-side of all channels for lead-off sensing
	// Chanel Setting
	ADS1x9x_Reg_Write(REG_CH1SET, 0x00);  // CH1SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH2SET, 0x00);  // CH2SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH3SET, 0x00);  // CH3SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH4SET, 0x00);  // CH4SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH5SET, 0x00);  // CH5SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH6SET, 0x00);  // CH6SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH7SET, 0x00);  // CH7SET, PGA = 6 , Normal electrode input 
	ADS1x9x_Reg_Write(REG_CH8SET, 0x00);  // CH8SET, PGA = 6 , Normal electrode input 
//	for(j = 0; j < 10;j++){
//		for(i= 0x12; i < 0x14; i++){
//			TRACE_DEBUG("Reg %02x  : %02X\n", i, ADS1x9x_Reg_Read(i));
//		}
//		Delay(500);
//	}ACR_BYTE0_ADDRESS
	for(uint8_t i=0; i<26; i++)
	{
		RegistersData[i] = ADS1x9x_Reg_Read(i);
	}
	Start_Read_Data_Continuous(); // Put the Device Back in RDATAC Mode

//	ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin){
		case GPIO_PIN_12:
			/* write new sample inside buffer*/
			HAL_SPI_TransmitReceive(&hspi2, &SPI_SendtoRecieveByte[0], &RawECGBuffer[RawECGBufferWriteIndex], RAW_ECG_SAMPLE_SIZE, 1);
			RawECGBufferWriteIndex = RawECGBufferWriteIndex + RAW_ECG_SAMPLE_SIZE;
			RawECGBufferWriteSampleNum++;
			osSemaphoreRelease(RawECGCountingSemHandle);
			if(RawECGBufferWriteIndex == RAW_ECG_BUFFER_SIZE)
			{
				RawECGBufferWriteIndex = 0; /* reset circular array index */
			}
			if(RawECGBufferWriteSampleNum == RAW_ECG_BUFFER_FULL_SAMPLE_NUM)
			{
				RawECGBufferWriteSampleNum = 0; /* reset circular array index */
			}
			if(!IsStartedSampling)
			{
				if(RawECGBufferWriteSampleNum > 1) /* safe threshold to indicate started sampling */
				{
					IsStartedSampling = 1; /* sampling is started */
				}
			}
			break;
		default: 
			break;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	HAL_SPI_Receive_DMA(&hspi2, ADS_Buffer, 27);
}
/******************************END OF FILE************************************/
