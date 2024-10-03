/*
 * DAC_MCP4726.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Maksim Inozemtsev
 */

#ifndef INC_DAC_MCP4726_H_
#define INC_DAC_MCP4726_H_

#include <stdint.h>
#include "main.h"

//----------------USER, please define your I2C here----------------
extern I2C_HandleTypeDef hi2c1;
#define I2C				hi2c1
//-----------------------------------------------------------------


/*
 * DAC Handle Structure
 */
typedef struct
{
	uint8_t DACAddress;
	uint8_t Command;
	uint16_t DACValue;

} DAC_Handle_t;



/*
 * Resistor Ladder Voltage Reference (V RL) Selection bits
 */

#define VDD_UNBUFFERED							0
#define VREF_UNBUFFERED							2
#define VREF_BUFFERED							3

/*
 * Power-Down Macros
 *
 * Power-Down Selection bits
 * When the DAC is powered down, most of the internal circuits are powered off and the op amp is
 * disconnected from the VOUT pin.
 */
#define NO_POWERED_DOWN							0
#define POWERED_DOWN_1K							1
#define POWERED_DOWN_100K						2
#define POWERED_DOWN_500K						3

/*
 * Gain Selection bit
 */
#define GAIN_1									0
#define GAIN_2									1		// Not applicable when VDD is used as VRL


/*
 * Commands Macros
 */

#define WRITE_VOLATILE_DAC_REG					0
#define WRITE_VOLATILE_MEMORY					2
#define WRITE_ALL_MEMORY						3
#define WRITE_VOLATILE_CONFIG					4




/*
 * Functions prototypes
 */

DeviceState DAC_Check_Device(DAC_Handle_t *pDAC);



#endif /* INC_DAC_MCP4726_H_ */
