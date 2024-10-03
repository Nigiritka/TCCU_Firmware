/*
 * IO_Expander.c
 *
 *  Created on: Apr 30, 2023
 *      Author: Maksim Inozemtsev
 */
#include "IO_Expander.h"

/*
 * Initialization of the IO Expander
 */
/**********************************************************************
 * @fn					- Expander_Init
 *
 * @brief				- This function initialize configuration of the Expander, defines input outputs, defines inversion
 *
 * @param[in]			- Address of Expander handler

 * @return				- none
 */
void Expander_Init(Expander_Handle_t *pExpander)
{
	//temporary variable to create an I2C message
	uint8_t temp[2] = {0};

	// Reset Output register
	temp[0] = COMMAND_OUTPUT_PORT_REGISTER;
	temp[1] = 0;
	HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, temp, 2, 10);

	// Sending configuration register to the Device
	temp[0] = COMMAND_CONFIGURATION_REGISTER;
	temp[1] = pExpander->ConfigReg;
	HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, temp, 2, 10);

	// Sending polarity register to the Device
	temp[0] = COMMAND_POLARITY_INVERSION_REGISTER;
	temp[1] = pExpander->PolarityPortReg;
	HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, temp, 2, 10);



}

/*
 * Writing Single Bit to the IO Expander
 */
/**********************************************************************
 * @fn					- Write_SingleBit
 *
 * @brief				- Set desired output pin 0 or 1, pin should be configure as an output
 *
 * @param[in]			- Address of Expander handler
 * @param[in]			- Number of output pin
 * @param[in]			- Desired state of the output pin

 *
 * @return				- Error if the pin is configure as an input and user try to assign value to this pin
 */
DeviceState Expander_Write_Single_Bit(Expander_Handle_t *pExpander, uint8_t BitNumber, PinState State)
{
	// Before writing to the GPIO pin,
	// we need to make sure that this particular bin was configure as an output
	if (!((pExpander->ConfigReg) & (1 << BitNumber)))
	{
		if (State == PIN_SET)
		{
			//seting desired pin
			pExpander->OutputPortReg |= 1 << BitNumber;
			//temporary variable to create an I2C message
			uint8_t temp[2];
			temp[0] = COMMAND_OUTPUT_PORT_REGISTER;
			temp[1] = pExpander->OutputPortReg;
			HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, temp, 2, 10);
			return DEVICE_OK;
		}
		else
		{
			// reseting desired pin
			pExpander->OutputPortReg &=~(1 << BitNumber);
			//temporary variable to create an I2C message
			uint8_t temp[2];
			temp[0] = COMMAND_OUTPUT_PORT_REGISTER;
			temp[1] = pExpander->OutputPortReg;
			HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, temp, 2, 10);
			return DEVICE_OK;
		}
	}
	else
	{
		return DEVICE_ERROR;
	}

}

/*
 * Read single pin of Expander
 */
/**********************************************************************
 * @fn					- Read_SingleBit
 *
 * @brief				- Read single pin of Expander
 *
 * @param[in]			- Address of Expander handler
 * @param[in]			- Number of pin
 *
 * @return				- State of the pin, which has been requested
 */
PinState Expander_Read_Single_Bit(Expander_Handle_t *pExpander, uint8_t BitNumber)
{
	uint8_t temp = 0;
	// First we will program the command, what we want to read
	// We need to read input port
	temp = COMMAND_INPUT_PORT_REGISTER;
	HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, &temp, 1, 10);

	// Now we can read the content of the Input register:
	HAL_I2C_Master_Receive(&I2C, pExpander->ExpanderAddress, &temp, 1, 10);

	// The function will return 0 or 1 (PIN_RESET or PIN_SET)
	// Apply mask to the received input register value to read bit at desired position
	// Move it to the LSB to have value only 0 or 1
	return ((temp & (1 << BitNumber)) >> BitNumber);
}

/*
 * Read complete input register
 */
/**********************************************************************
 * @fn					- Read_Byte
 *
 * @brief				- Read input register
 *
 * @param[in]			- Address of Expander handler
 *
 * @return				- value of input register of Expander
 */
uint8_t Expander_Read_Byte(Expander_Handle_t *pExpander)
{
	uint8_t temp = 0;
	// First we will program the command, what we want to read
	// We need to read input port
	temp = COMMAND_INPUT_PORT_REGISTER;
	HAL_I2C_Master_Transmit(&I2C, pExpander->ExpanderAddress, &temp, 1, 10);

	// Now we can read the content of the Input register:
	HAL_I2C_Master_Receive(&I2C, pExpander->ExpanderAddress, &temp, 1, 10);

	return temp;
}

/*
 * Check if device functioning and answer Master's requests.
 */
/**********************************************************************
 * @fn					- Check_Device
 *
 * @brief				- Check if Expander on I2C bus is accessible
 *
 * @param[in]			- Address of Expander handler
 *
 * @return				- 0 if device ok, 1 if device is not ok
 */
DeviceState Expander_Check_Device(Expander_Handle_t *pExpander)
{
	if (HAL_I2C_IsDeviceReady(&I2C, pExpander->ExpanderAddress, 1, 10) == HAL_OK)
	{
		return DEVICE_OK;
	}
	else
	{
		return DEVICE_ERROR;
	}
}
