/*
 * ADC_MCP3464.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Maksim Inozemtsev
 */
#include "ADC_MCP3464.h"



// Temp variable for the SPI communication
// The precise length will be calculated in the for loop below
// let's make a size of the variable 25 - the maximum possible amount of reading + 1 initial system SPI byte, 26 in total
uint8_t SPIMessage[26] = {0};

// Adding Device address to the bits [7:6];
uint8_t FastCommandByte = 0;

uint8_t ADCReceivedData[30] = {0};

// transfer data to one variable
int16_t ReadData;

extern uint8_t SPIDUMMYData [100];


/**********************************************************************
 * @fn					- ADC_Init
 *
 * @brief				- This function configures the ADC, defines all parameters for measurement
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- none
 */
uint8_t ADC_Init(ADC_Handle_t *pADC)
{
	// Firstly, Reserved registers need to be defined with default values:
	pADC->RESERVED1 = 0x900000;
	pADC->RESERVED2 = 0x50;

#ifdef MCP3464
	pADC->RESERVED3 = 0x000B;
#endif
#ifdef MCP3462
	pADC->RESERVED3 = 0x0009;
#endif
#ifdef MCP3461
	pADC->RESERVED3 = 0x0008;
#endif

	// Secondly, define reserved bits in registers:
	pADC->Config1.RESERVED = 0x00;
	pADC->Config2.RESERVED = 0x03;
	pADC->SCAN.RESERVED = 0x00;

	// Just in case, to avoid full shutdown mode, we will put in CONFIG0-CONFIG0 bits "11"
	pADC->Config0.CONFIG0 = 0x03;

	// Finally, we are applying USER's configurations:
	// To do initialization we will use incremental write to the ADC's registers laid in addresses 0x01 to 0x0D

	ADC_Incremental_Write(pADC, CONFIG0_ADDRESS, INITIALIZATION_AMOUNT_OF_REGISTERS);

	return 0;
}

/**********************************************************************
 * @fn					- ADC_Incremental_Write
 *
 * @brief				- push the device to "Write mode" and starts writing the first data byte to the address given in functions param
 *							The data is taken from ADC_Handle_t structure and sent to ADC via SPI
 *
 * @param[in]			- Address of ADC handler
 * @param[in]			- Address of the register, with which writing process is started
 * @param[in]			- Amount of registers which need to be written
 *
 * @return				- none
 */
uint8_t ADC_Incremental_Write(ADC_Handle_t *pADC, uint8_t RegisterAddress, uint8_t AmountADCRegistersToWrite)
{
	// Check if the parameters which were given to the function are correct
	// 1. Check if initial address is okey
	if (RegisterAddress < ADCDATA_ADDRESS || RegisterAddress > CRCCFG_ADDRESS)
	{
		// Generate Error
		return 1;
	}
	// 2. Check if amount to write is not too long
	if (AmountADCRegistersToWrite > (0xE - RegisterAddress))
	{
		// Generate Error
		return 1;
	}

	// create a temp pointer variable for taking information from the memory to SPI message
	// The pointer has size 8 bits intentionally, so that we move byte by byte instead of words in memory
	uint8_t *pTemp = NULL;

	// Forming Fast Command for incremental writing
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding address (bits [5:2]) of ADC's register address we want to start writing to
	FastCommandByte += RegisterAddress << 2;

	// Adding "Increment writing" command (bits [1:0])
	FastCommandByte += COMMAND_INCREMENTAL_WRITE;

	// Assign first byte for sending SPI writing command with desirable address
	SPIMessage[0] = FastCommandByte;

	// Variable will be responsible for creating exact length of the SPI message
	// Initial value "1" because it will be first byte with system information (previous assignment)
	uint8_t SPIAmountofBytes = 1;

	/*
	 * because all registers have different size, we need to implement quite complicated way
	 * of writing registers.
	 *
	 * for loop will go through register which needs to be processed
	 * Switch will decide amount of bytes for each particular register
	 */

	for (uint8_t CurrentRegister = RegisterAddress; CurrentRegister <= AmountADCRegistersToWrite; CurrentRegister++)
	{
		switch (CurrentRegister)
		{
			case CONFIG0_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->Config0;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case CONFIG1_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->Config1;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case CONFIG2_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->Config2;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case CONFIG3_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->Config3;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case IRQ_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->IRQ;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case MUX_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->MUX;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case SCAN_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->SCAN;

				// dereferencing pointer to fetch information from the memory
				// Because if MSB first, we need to start to put bytes in the reverse order!!
				// the easiest way, to adjuct pointer for this, firstly we will take the highest byte

				SPIMessage[SPIAmountofBytes] = *pTemp;

				// because of SCAN register 24 bits wide, we need to take 2 more bytes from memory
				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				break;

			case TIMER_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->TIMER;

				SPIMessage[SPIAmountofBytes] = *pTemp;

				// because of SCAN register 24 bits wide, we need to take 2 more bytes from memory
				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				break;

			case OFFSETCAL_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->OFFSETCAL;

				SPIMessage[SPIAmountofBytes] = *pTemp;

				// because of SCAN register 24 bits wide, we need to take 2 more bytes from memory
				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				break;

			case GAINCAL_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->GAINCAL;

				SPIMessage[SPIAmountofBytes] = *pTemp;

				// because of SCAN register 24 bits wide, we need to take 2 more bytes from memory
				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				break;

			case RESERVED1_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->RESERVED1;

				SPIMessage[SPIAmountofBytes] = *pTemp;

				// because of SCAN register 24 bits wide, we need to take 2 more bytes from memory
				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				break;

			case RESERVED2_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->RESERVED2;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case LOCK_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->LOCK;

				// dereferencing pointer to fetch information from the memory
				SPIMessage[SPIAmountofBytes] = *pTemp;

				// increment amount of bytes which will be sent to ADC
				SPIAmountofBytes++;
				break;

			case RESERVED3_ADDRESS:
				// Assign to our temporary pointer the address of the desired register
				pTemp = (uint8_t *) &pADC->RESERVED3;

				SPIMessage[SPIAmountofBytes] = *pTemp;

				// because of SCAN register 24 bits wide, we need to take 2 more bytes from memory
				SPIAmountofBytes++;
				pTemp++;
				SPIMessage[SPIAmountofBytes] = *pTemp;

				SPIAmountofBytes++;
				break;


			default:
				return 1;			//???????????????    Return error for the function    ???????????????????
		}

	}

	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPI, SPIMessage, SPIAmountofBytes, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);
	//Start_SPI3_Transmit(SPIMessage, SPIAmountofBytes);
	//HAL_SPI_TransmitReceive(&SPI, SPIMessage, &TestReading, SPIAmountofBytes, 10);
	//HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);




	return 0;
}

/**********************************************************************
 * @fn					- ADC_Static_Read
 *
 * @brief				- This function Read desired register from ADC
 *
 * @param[in]			- Address of ADC handler
 * @param[in]			- Address of the ADC's register, which needs to be read
 *
 * @return				- Content of the register
 */
uint32_t ADC_Static_Read(ADC_Handle_t *pADC, uint8_t RegisterAddress)
{
	// temp variable which we will use for saving data which we will read from ADC
	uint8_t temp[5] = {0};
	uint8_t SPIMessage[5] = {0};

	// Forming Fast Command for incremental reading
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding address (bits [5:2]) of ADC's register address we want to start writing to
	FastCommandByte += RegisterAddress << 2;

	// Adding "Increment writing" command (bits [1:0])
	FastCommandByte += COMMAND_STATIC_READ;
	SPIMessage[0] = FastCommandByte;

	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	//Start_SPI_TransmitReceive(SPIMessage, temp, 5);
	HAL_SPI_TransmitReceive(&SPI, SPIMessage, temp, 5, 10);
	//HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	//HAL_SPI_Receive(&SPI, temp, 4, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);

	ADC_Check_Errors(temp[0]);
	// transfer data to one variable
	uint32_t ReadData;

	//Change it, it is strange:
	ReadData = temp[1] << 24 | temp[2] << 16 | temp[3] << 8 | temp[4];

	return ReadData;
}

/**********************************************************************
 * @fn					- ADC_Start_Conversion
 *
 * @brief				- This function initiates Analog-to-Digital conversion
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- none
 */
void ADC_Start_Conversion(ADC_Handle_t *pADC)
{
	// Forming Fast Command
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding command, bits [5:0]
	FastCommandByte += FC_START_CONVERSION;

	//Start_SPI3_TransmitReceive(&FastCommandByte, SPIDUMMYData, 1);
	// Transfer this via SPI
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	//HAL_SPI_Transmit_IT(&SPI, &FastCommandByte, 1);
	// HAL_SPI_Transmit_DMA(&SPI, &FastCommandByte, 1);
	//__HAL_DMA_DISABLE_IT(&hdma_spi3_tx, DMA_IT_HT);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);
}

/**********************************************************************
 * @fn					- ADC_Standby
 *
 * @brief				- This function puts ADC in stand-by mode
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- none
 */
void ADC_Standby(ADC_Handle_t *pADC)
{
	// Forming Fast Command
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding command, bits [5:0]
	FastCommandByte += FC_STANDBY_MODE;

	// Transfer this via SPI
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	//Start_SPI_Transmit(&FastCommandByte, 1);
	HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);
}

/**********************************************************************
 * @fn					- ADC_Shutdown
 *
 * @brief				- This function puts ADC in Shutdown mode
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- none
 */
void ADC_Shutdown(ADC_Handle_t *pADC)
{
	// Forming Fast Command
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding command, bits [5:0]
	FastCommandByte += FC_SHUTDOWN_MODE;

	// Transfer this via SPI
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	//Start_SPI_Transmit(&FastCommandByte, 1);
	HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);
}

/**********************************************************************
 * @fn					- ADC_Full_Shutdown
 *
 * @brief				- This function puts ADC in Full Shutdown mode
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- none
 */
void ADC_Full_Shutdown(ADC_Handle_t *pADC)
{
	// Forming Fast Command
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding command, bits [5:0]
	FastCommandByte += FC_FULL_SHUTDOWN_MODE;

	// Transfer this via SPI
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	//Start_SPI_Transmit(&FastCommandByte, 1);
	HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);
}

/**********************************************************************
 * @fn					- ADC_Full_Reset
 *
 * @brief				- This function reset all ADC's register contents to default value
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- none
 */
void ADC_Full_Reset(ADC_Handle_t *pADC)
{
	// Forming Fast Command
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding command, bits [5:0]
	FastCommandByte += FC_DEVICE_FULL_RESET;

	// Transfer this via SPI
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	//Start_SPI_Transmit(&FastCommandByte, 1);
	HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);
}

/**********************************************************************
 * @fn					- ADC_Get_Status_Byte
 *
 * @brief				- This function read Status Byte from ADC
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- Status Byte
 */
uint8_t ADC_Get_Status_Byte(ADC_Handle_t *pADC)
{
	// Forming Fast Command, we need to do dummy command
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Adding command, bits [5:0]
	FastCommandByte += COMMAND_DUMMY;

	// Creating a variable which takes value of Status Byte from the ADC
	uint8_t StatusByte = 0;

	// Transfer and getting data via SPI
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	//Start_SPI_TransmitReceive(&FastCommandByte, &StatusByte, 1);
	HAL_SPI_TransmitReceive(&SPI, &FastCommandByte, &StatusByte, 1, 10);
	//HAL_SPI_Transmit(&SPI, &FastCommandByte, 1, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);

	return StatusByte;
}

/**********************************************************************
 * @fn					- ADC_Check_Errors
 *
 * @brief				- This function Check ADC's Status register if an Error occured in the ADC
 *
 * @param[in]			- Address of ADC handler
 * @param[in]			- ADC's Status Byte
 *
 * @return				- Error????????????????????????????
 */
uint8_t ADC_Check_Errors(uint8_t StatusByte)
{
	// Check errors:
	// Error are active Low, so we check if the bit is reset
	// Chech if it was a power disruption
	if (!(StatusByte & ADC_STATUS_POR_MASK))
	{
		//JCU_Error_Handler(ADC_POWER_ERROR);		// TO DO! How to deal with Errors??????????
	}
	// Check if CRC error has occurred:
	if (!(StatusByte & ADC_STATUS_CRC_ERROR_MASK))
	{
		//JCU_Error_Handler(ADC_CRC_ERROR);					// TO DO! How to deal with Errors??????????
	}
	return 0;
}

/**********************************************************************
 * @fn					- ADC_Get_Status_Byte
 *
 * @brief				- This function read measured analog data from ADC.
 * 						This function is used for static read of the ADCData register
 * 						It checks Status byte of the ADC, to understand if new data is available.
 * 						If no new data available, the function returns previous data
 * 						Function is adjusted to 16 bit output data configuration of the ADC!!!
 *
 * @param[in]			- Address of ADC handler
 *
 * @return				- Analog value
 */
float ADC_Get_Measured_DATA(ADC_Handle_t *pADC)
{
	// Forming Fast Command for incremental reading
	// Adding Device address to the bits [7:6];
	FastCommandByte = ADC_DEVICE_ADDRESS << 6;

	// Reading from ADCDATA address Adding address (bits [5:2])
	FastCommandByte += ADCDATA_ADDRESS << 2;


	// Adding "Static Read" command (bits [1:0])
	FastCommandByte += COMMAND_STATIC_READ;

	// Initiate SPI reading

	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&SPI, &FastCommandByte, ADCReceivedData, 3, 10);
	HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET);

	ADC_Proccess_Data();
	return 0;
}




void ADC_Proccess_Data(void)
{
	// Check errors
	ADC_Check_Errors(ADCReceivedData[0]);

	// Check if new data available
	if (!(ADCReceivedData[0] & ADC_STATUS_DATA_READY_MASK))
	{
		//Transfer array of bytes to the 32 bit variable:
		ReadData = ADCReceivedData[1];
		ReadData <<= 8;
		ReadData += ADCReceivedData[2];

		// Calculating real Voltage
		Voltage =  VREF * ReadData / 32768;

	}
	else
	{
		// if no new data available, just return previous value
	}
	// Check if new data available to read from ADC:

}

