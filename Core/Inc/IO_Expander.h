/*
 * IO_Expander.h
 *
 *  Created on: Apr 30, 2023
 *      Author: Maksim Inozemtsev
 *
 *      Driver is written to operate with PI4IOE5V9554 - 8 bits of
 *      general purpose parallel input/output (GPIO) expansion for
 *      I2C-bus/SMBus applications
 */

#ifndef INC_IO_EXPANDER_H_
#define INC_IO_EXPANDER_H_

#include <stdint.h>
#include "main.h"

//----------------USER, please define your I2C here----------------
extern I2C_HandleTypeDef hi2c1;
#define I2C				hi2c1
//-----------------------------------------------------------------

/*
 * I/O Expander Handle Structure
 */
typedef struct
{
	uint8_t ExpanderAddress;
	uint8_t InputPortReg;
	uint8_t OutputPortReg;
	uint8_t PolarityPortReg;
	uint8_t ConfigReg;

} Expander_Handle_t;

/*
 * Enumeration for PIN state
 */
typedef enum
{
	PIN_RESET = 0U,
	PIN_SET
} PinState;

/*
 * Enumeration for Device state
 */
typedef enum
{
	DEVICE_OK = 0U,
	DEVICE_ERROR,
} DeviceState;


/*
 * GPIO Pin number macros
 */
#define GPIO_PIN_NO_0							0
#define GPIO_PIN_NO_1							1
#define GPIO_PIN_NO_2							2
#define GPIO_PIN_NO_3							3
#define GPIO_PIN_NO_4							4
#define GPIO_PIN_NO_5							5
#define GPIO_PIN_NO_6							6
#define GPIO_PIN_NO_7							7

/*
 * GPIO Pin number macros for this particular project, User can define it for yourself
 */
#define LED_BLUE								GPIO_PIN_NO_0
#define LED_RED									GPIO_PIN_NO_1
#define LED_WHITE								GPIO_PIN_NO_2
#define LED_YELLOW								GPIO_PIN_NO_3
//#define 										GPIO_PIN_NO_4
#define BUTTON_STARTSTOP						GPIO_PIN_NO_5
#define BUTTON_INCREASE							GPIO_PIN_NO_6
#define BUTTON_DECREASE							GPIO_PIN_NO_7


/* Command byte - The command byte is the first byte to follow the address byte during a write transmission.
* It is used as a pointer to determine which of the following registers are written or read.
*
* ----------------------------------------------------------------
* Command	|	Protocol			|	Function
* ----------------------------------------------------------------
* 0			|	Read Byte			|	Input port register
* ----------------------------------------------------------------
* 1			|	Read/Write Byte		|	Output port register
* ----------------------------------------------------------------
* 2			|	Read/Write Byte		|	Polarity Inversion register
* ----------------------------------------------------------------
* 3			|	Read/Write Byte		|	Configuration register
* ----------------------------------------------------------------
*/
#define COMMAND_INPUT_PORT_REGISTER				0
#define COMMAND_OUTPUT_PORT_REGISTER			1
#define COMMAND_POLARITY_INVERSION_REGISTER		2
#define COMMAND_CONFIGURATION_REGISTER			3


/*
 * Function prototype
 */

void Expander_Init(Expander_Handle_t *pExpander);
DeviceState Expander_Write_Single_Bit(Expander_Handle_t *pExpander, uint8_t BitNumber, PinState State);
PinState Expander_Read_Single_Bit(Expander_Handle_t *pExpander, uint8_t BitNumber);
uint8_t Expander_Read_Byte(Expander_Handle_t *pExpander);
DeviceState Expander_Check_Device(Expander_Handle_t *pExpander);

#endif /* INC_IO_EXPANDER_H_ */
