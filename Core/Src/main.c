/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */



/*
 * TO DO:
 * 0. EEPROM Value for DAC!!!!!!!!!!!!!!
 *
 *
 * 1. Humidity Sensor
 * 1.1. Temperature sensor
 * 2. PID
 * 3. FAN thresholds definition
 *
 *
 *
 *
 *
 * 6. Changed GPIO from HAL to LL
 *
 *
 *
 */





/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "IO_Expander.h"
#include "DAC_MCP4726.h"
#include "ADC_MCP3464.h"

#include "SSD1306.h"
#include "SSD1306_fonts.h"


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
I2C_HandleTypeDef hi2c1;



SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

ADC_Handle_t MyADC;
DAC_Handle_t MyDAC;
Expander_Handle_t MyExpander;

int flag = 0;

typedef enum
{
	StandBy = 0U,
	Heating,
	Cooling
} Mode_of_Operation;

Mode_of_Operation TCCU_Mode = StandBy;


float Voltage = 0;
float VoltageCh1 = 0;
float VoltageCh2 = 0;
float VoltageCh3 = 0;
float VoltageCh4 = 0;

const float ResistanceCh1 = 1000;			// I will remove it, it is bullshit

float ResistanceCh2 = 0;
float ResistanceCh3 = 0;
float ResistanceCh4 = 0;

float ResistanceADCOffset = 0;

float A = 3.9083E-3;
char TempString[4];


int SetPoint = 10;


uint32_t delay = 100;

//it was float, but for easines of manipulation with the number, int is fine, I do not need more precision than +/- 1 degree C
int TemperatureCh2;
int TemperatureCh3;
int TemperatureCh4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Delay is required, because not all part of the system are powered-on when the code starts to execute the stuff
  HAL_Delay(1000);


//-------------------- IO Expander Initialization ----------------------------------//
	MyExpander.ExpanderAddress = 0x40;			//0b1000000
	MyExpander.ConfigReg = 0xF0;				//0b11110000
	MyExpander.PolarityPortReg = 0xF0;			//0b11110000
	Expander_Init(&MyExpander);

	Expander_Write_Single_Bit(&MyExpander, LED_BLUE, PIN_RESET);
	Expander_Write_Single_Bit(&MyExpander, LED_RED, PIN_RESET);
	Expander_Write_Single_Bit(&MyExpander, LED_WHITE, PIN_RESET);
	Expander_Write_Single_Bit(&MyExpander, LED_AMBER, PIN_RESET);
//----------------------------------------------------------------------------------//




//---------------------------- DAC Initialization ----------------------------------//

	MyDAC.DACAddress = 0x63;
	MyDAC.Command = WRITE_VOLATILE_MEMORY << 5;
	MyDAC.Command += VREF_BUFFERED << 3;
	MyDAC.Command += NO_POWERED_DOWN << 1;
	MyDAC.Command += DAC_GAIN_1 << 0;
	MyDAC.DACValue = 0x0080;    // 0x80 - 1.5V
	HAL_I2C_Master_Transmit(&I2C, MyDAC.DACAddress<<1, &MyDAC.Command, 3, 10);


//----------------------------------------------------------------------------------//


/*

//------------------------ I2C Device Checker --------------------------------------//
	if (HAL_I2C_IsDeviceReady(&I2C, 0x3C<<1, 3, 10) == HAL_OK)
	{
		Expander_Write_Single_Bit(&MyExpander, LED_WHITE, PIN_SET);
	}
	else
	{
		Expander_Write_Single_Bit(&MyExpander, LED_RED, PIN_SET);
	}
//----------------------------------------------------------------------------------//
*/

		ssd1306_Init();
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();

		ssd1306_WriteString("TCCU: StandBy", Font_7x10, White);
		ssd1306_UpdateScreen();
		HAL_Delay(1000);
		ssd1306_SetCursor(0,20);
		ssd1306_WriteString("Set Point:", Font_7x10, White);
		ssd1306_UpdateScreen();




//-------------------------- ADC Initialization ------------------------------------//
	// ADC Configuration:
	MyADC.Config0.ADCMode = ADC_STANDBY_MODE;
	MyADC.Config0.CS_SEL = NO_CURRENT_SOURCE;
	MyADC.Config0.CLK_SEL = EXTERNAL_DIGITAL_CLOCK;

	MyADC.Config1.RESERVED = 0x0;
	MyADC.Config1.OSR = OSR_4096;
	MyADC.Config1.PRE = AMCLK_MCLK_DIV2;

	MyADC.Config2.RESERVED = 0x3;
	MyADC.Config2.AZ_MUX = AZ_MUX_DISABLED;
	MyADC.Config2.GAIN = ADC_GAIN_1;
	MyADC.Config2.BOOST = BOOST_CURRENT_1;

	MyADC.Config3.EN_GAINCAL = GAINCAL_DISABLED;
	MyADC.Config3.EN_OFFCAL = OFFCAL_DISABLED;
	MyADC.Config3.EN_CRCCOM = CRCCOM_DISABLED;
	MyADC.Config3.CRC_FORMAT = CRC_FORMAT_CRC16;
	MyADC.Config3.DATA_FORMAT = ADC_DATA_FORMAT_16BIT;
	MyADC.Config3.CONV_MODE = CONV_MODE_ONE_SHOT_STANDBY;

	MyADC.IRQ.EN_STP = CONVERSATION_START_INTERRUPT_DISABLED;
	MyADC.IRQ.EN_FASTCMD = FAST_COMMAND_ENABLED;
	MyADC.IRQ.IRQ_MODE0 = IRQ_PIN_MODE_LOGIC_HIGH;
	MyADC.IRQ.IRQ_MODE1 = MDAT_PIN_MODE_ALL_SELECTED;

	MyADC.MUX.MUX_VinPlus = MUX_CH0;
	MyADC.MUX.MUX_VinMinus = MUX_CH1;
	MyADC.LOCK = 0xA5;


	ADC_Full_Reset(&MyADC);

	// Put our configuration
	ADC_Init(&MyADC);
//-----------------------------------------------------------------------------------//


	//HAL_GPIO_WritePin(FAN_3_EN_GPIO_Port, FAN_3_EN_Pin, GPIO_PIN_SET);
	//LL_GPIO_SetOutputPin(FAN_3_EN_GPIO_Port, FAN_3_EN_Pin);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



		MyADC.MUX.MUX_VinPlus = MUX_CH0;
		MyADC.MUX.MUX_VinMinus = MUX_CH1;
		ADC_Incremental_Write(&MyADC, MUX_ADDRESS, 1);
		ADC_Start_Conversion(&MyADC);
		HAL_Delay(delay);
		VoltageCh1 = Voltage;


		MyADC.MUX.MUX_VinPlus = MUX_CH1;
		MyADC.MUX.MUX_VinMinus = MUX_CH2;
		ADC_Incremental_Write(&MyADC, MUX_ADDRESS, 1);
		ADC_Start_Conversion(&MyADC);
		HAL_Delay(delay);
		VoltageCh2 = Voltage;
		// Calculating the resistance
		ResistanceCh2 = ResistanceCh1*VoltageCh2/VoltageCh1;


		  // simplified formula for conversion resistance into the temperature:
		  TemperatureCh2 = (ResistanceCh2 - ResistanceCh1) / (ResistanceCh1 * A);

		MyADC.MUX.MUX_VinPlus = MUX_CH2;
		MyADC.MUX.MUX_VinMinus = MUX_CH3;
		ADC_Incremental_Write(&MyADC, MUX_ADDRESS, 1);
		ADC_Start_Conversion(&MyADC);
		HAL_Delay(delay);
		VoltageCh3 = Voltage;
		ResistanceCh3 = ResistanceCh1*VoltageCh3/VoltageCh1;

		  TemperatureCh3 = (ResistanceCh3 - ResistanceCh1) / (ResistanceCh1 * A);

		MyADC.MUX.MUX_VinPlus = MUX_CH3;
		MyADC.MUX.MUX_VinMinus = MUX_AGND;
		ADC_Incremental_Write(&MyADC, MUX_ADDRESS, 1);
		ADC_Start_Conversion(&MyADC);
		HAL_Delay(delay);
		VoltageCh4 = Voltage;
		ResistanceCh4 = ResistanceCh1*VoltageCh4/VoltageCh1;

		  TemperatureCh4 = (ResistanceCh4 - ResistanceCh1) / (ResistanceCh1 * A);


		MyADC.MUX.MUX_VinPlus = MUX_CH3;
		MyADC.MUX.MUX_VinMinus = MUX_CH3;
		ADC_Incremental_Write(&MyADC, MUX_ADDRESS, 1);
		ADC_Start_Conversion(&MyADC);
		HAL_Delay(delay);
		ResistanceADCOffset = ResistanceCh1*Voltage/VoltageCh1;



//---------------------------Display-------------------------//
		ssd1306_SetCursor(90,20);
		ssd1306_WriteString("Temp:", Font_7x10, White);
		ssd1306_Line(82,20,82,60, White);

//------------------------ t Sensor 1 ----------------------//
		ssd1306_SetCursor(115,34);
		ssd1306_WriteString("C", Font_7x10, White);
		ssd1306_SetCursor(90,34);
		intToStr(TemperatureCh2,TempString);
		ssd1306_WriteString(TempString, Font_7x10, White);
//----------------------------------------------------------//

//------------------------ t Sensor 2 ----------------------//
		ssd1306_SetCursor(115,44);
		ssd1306_WriteString("C", Font_7x10, White);
		ssd1306_SetCursor(90,44);
		intToStr(TemperatureCh3,TempString);
		ssd1306_WriteString(TempString, Font_7x10, White);
//----------------------------------------------------------//


//------------------------ t Sensor 3 ----------------------//
		ssd1306_SetCursor(115,54);
		ssd1306_WriteString("C", Font_7x10, White);
		ssd1306_SetCursor(90,54);
		intToStr(TemperatureCh4,TempString);
		ssd1306_WriteString(TempString, Font_7x10, White);
//----------------------------------------------------------//

//------------------------ Set Point ----------------------//
		ssd1306_SetCursor(25,34);
		ssd1306_WriteString("C", Font_7x10, White);
		ssd1306_SetCursor(0,34);
		intToStr(SetPoint,TempString);
		ssd1306_WriteString(TempString, Font_7x10, White);
//----------------------------------------------------------//

		ssd1306_UpdateScreen();


//----------------------- Operation indication----------------//
		if (TCCU_Mode == Heating)
		{
			if (flag == 0)
			{
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("TCCU: ON Heating", Font_7x10, White);
				flag = 1;
			}
			else
			{
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("TCCU: ON        ", Font_7x10, White);
				flag = 0;
			}
			ssd1306_UpdateScreen();
		}
		else if (TCCU_Mode == Cooling)
		{
			if (flag == 0)
			{
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("TCCU: ON Cooling", Font_7x10, White);
				flag = 1;
			}
			else
			{
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("TCCU: ON        ", Font_7x10, White);
				flag = 0;
			}
			ssd1306_UpdateScreen();
		}
//-------------------------------------------------------------------------------//



//----------------Thermal control loop-------------------------------------------//

	if (TCCU_Mode == Heating)
	{
		MyDAC.DACValue = 0x00AA;    // 0xAA - 2.0V
		HAL_I2C_Master_Transmit(&I2C, MyDAC.DACAddress<<1, &MyDAC.Command, 3, 10);
	}
	else if (TCCU_Mode == Cooling)
	{
		MyDAC.DACValue = 0x0055;    // 0x75 - 1.4V
		HAL_I2C_Master_Transmit(&I2C, MyDAC.DACAddress<<1, &MyDAC.Command, 3, 10);
	}
	else
	{
		MyDAC.DACValue = 0x0080;    // 0x80 - 1.5V
		HAL_I2C_Master_Transmit(&I2C, MyDAC.DACAddress<<1, &MyDAC.Command, 3, 10);
	}

	//-------------------------------------------------------------------------------//







	  /*
	  Expander_Write_Single_Bit(&MyExpander, LED_BLUE, PIN_SET);
	  HAL_Delay(500);
	  Expander_Write_Single_Bit(&MyExpander, LED_BLUE, PIN_RESET);
	  HAL_Delay(500);
	  */




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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, FAN_1_EN_Pin|FAN_2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAN_3_EN_GPIO_Port, FAN_3_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : FAN_1_EN_Pin FAN_2_EN_Pin */
  GPIO_InitStruct.Pin = FAN_1_EN_Pin|FAN_2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : IOEXPANDER_INT_L_Pin */
  GPIO_InitStruct.Pin = IOEXPANDER_INT_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IOEXPANDER_INT_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_3_EN_Pin */
  GPIO_InitStruct.Pin = FAN_3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_3_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_nCS_Pin */
  GPIO_InitStruct.Pin = ADC_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADC_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_IRQ_Pin */
  GPIO_InitStruct.Pin = ADC_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADC_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == IOEXPANDER_INT_L_Pin)
	{

		if (Expander_Read_Byte(&MyExpander) & (1<<BUTTON_INCREASE))
		{
			if (SetPoint <85 && TCCU_Mode == StandBy)
			{
				SetPoint += 5;

			}

			ssd1306_SetCursor(0,34);
			intToStr(SetPoint,TempString);
			ssd1306_WriteString(TempString, Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		else if (Expander_Read_Byte(&MyExpander) & (1<<BUTTON_DECREASE))
		{
			if (SetPoint > -40 && TCCU_Mode == StandBy)
			{
				SetPoint -= 5;
			}

			ssd1306_SetCursor(0,34);
			intToStr(SetPoint,TempString);
			ssd1306_WriteString(TempString, Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		else if (Expander_Read_Byte(&MyExpander) & (1<<BUTTON_STARTSTOP))
		{
			if (TCCU_Mode == StandBy)
			{
				// if we are gonna heat
				if (SetPoint > TemperatureCh2)
				{
					TCCU_Mode = Heating;
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("TCCU: ON Heating", Font_7x10, White);
					ssd1306_UpdateScreen();
				}
				else
				{
					TCCU_Mode = Cooling;
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("TCCU: ON Cooling", Font_7x10, White);
					ssd1306_UpdateScreen();
				}
			}
			else
			{
				TCCU_Mode = StandBy;
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("TCCU: StandBy     ", Font_7x10, White);
				ssd1306_UpdateScreen();
			}
		}





	}
	else if (GPIO_Pin == ADC_IRQ_Pin)
	{
		//Expander_Write_Single_Bit(&MyExpander, LED_AMBER, PIN_SET);
		ADC_Get_Measured_DATA(&MyADC);
		ADC_Proccess_Data();
	}


}






void intToStr(int N, char *str) {
    int i = 0;

    if (N == 0)
    {
    	str[i++] = '0';
    	str[i++] = ' ';
    	str[i++] = ' ';
    }
    else
    {

		// Save the copy of the number for sign
		int sign = N;

		// If the number is negative, make it positive
		if (N < 0)
			N = -N;

		// Extract digits from the number and add them to the
		// string
		while (N > 0) {

			// Convert integer digit to character and store
			// it in the str
			str[i++] = N % 10 + '0';
			N /= 10;
		}

		// If the number was negative, add a minus sign to the
		// string
		if (sign < 0) {
			str[i++] = '-';
		}
		else
		{
			str[i++] = '+';
		}
		// Null-terminate the string
		if (i<3)
		{
			str[i++] = ' ';
		}
		str[i] = '\0';
    }
	// Reverse the string to get the correct order
	for (int j = 0, k = i - 1; j < k; j++, k--) {
		char temp = str[j];
		str[j] = str[k];
		str[k] = temp;
	}


}










/* USER CODE END 4 */

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
