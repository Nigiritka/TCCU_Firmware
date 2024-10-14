/*
 * ADC_MCP3464.h
 *
 *  Created on: Mar 21, 2023
 *      Author: Maksim Inozemtsev
 */

/*
 * Some registers of MCP3461/2/4 shall be define to default values which depend on the ADC input number
 * For this you need to define which ADC you exactly use
 * You have 3 options respectively:
 *  - MCP3461
 *  - MCP3462
 *  - MCP3464
 */
#define MCP3462
//-----------------------------------------------------------------------------------------------------



#define INITIALIZATION_AMOUNT_OF_REGISTERS					13

#include <stdint.h>
#include "main.h"

extern float Voltage;

//----------------USER, please define your SPI here----------------
extern SPI_HandleTypeDef hspi1;
#define SPI				hspi1
//-----------------------------------------------------------------


/*
 * 					ADC REGISTERS INFORMATION:
 *
 * -------------------------------------------------------------------------------------
 * 	 Address	|	Register Name	|	No. of Bits	|  R/W	| 		Description
 * -------------------------------------------------------------------------------------
 * 		0x0		|	ADCDATA			|	4/16/32		|	R	|	Latest A/D conversion data output value (16 or 32 bits
 *				|					|				|		|	depending on DATA_FORMAT[1:0]) or modulator output
 *				|					|				|		|	stream (4-bit wide) in MDAT Output mode.
 * -------------------------------------------------------------------------------------
 * 		0x1		|	CONFIG0			|		8		|  R/W	|	ADC Operating mode, Master Clock mode and Input Bias
 *				|					|				|		|	Current Source mode.
 * -------------------------------------------------------------------------------------
 * 		0x2		|	CONFIG1			|		8		|  R/W	|	Prescale and OSR settings.
 * -------------------------------------------------------------------------------------
 * 		0x3		|	CONFIG2			|		8		|  R/W	|	ADC boost and gain settings, auto-zeroing settings for
 *				|					|				|		|	analog multiplexer, voltage reference and ADC.
 * -------------------------------------------------------------------------------------
 * 		0x4		|	CONFIG3			|		8		|  R/W	|	Conversion mode, data and CRC format settings, enable for
 *				|					|				|		|	CRC on communications, enable for digital offset and gain
 *				|					|				|		|	error calibrations.
 * -------------------------------------------------------------------------------------
 * 		0x5		|	IRQ				|		8		|  R/W	|	IRQ Status bits and IRQ mode settings, enable for Fast
 *				|					|				|		|	commands and for conversion start pulse.
 * -------------------------------------------------------------------------------------
 * 		0x6		|	MUX				|		8		|  R/W	|	Analog multiplexer input selection (MUX mode only).
 * -------------------------------------------------------------------------------------
 * 		0x7		|	SCAN			|		24		|  R/W	|	SCAN mode settings.
 * -------------------------------------------------------------------------------------
 * 		0x8		|	TIMER			|		24		|  R/W	|	Delay value for TIMER between each SCAN cycle.
 * -------------------------------------------------------------------------------------
 * 		0x9		|	OFFSETCAL		|		24		|  R/W	|	ADC digital offset calibration value.
 * -------------------------------------------------------------------------------------
 * 		0xA		|	GAINCAL			|		24		|  R/W	|	ADC digital gain calibration value.
 * -------------------------------------------------------------------------------------
 * 		0xB		|	RESERVED		|		24		|  R/W	|
 * -------------------------------------------------------------------------------------
 * 		0xC		|	RESERVED		|		8		|  R/W	|
 * -------------------------------------------------------------------------------------
 * 		0xD		|	LOCK			|		8		|  R/W	|	Password value for SPI Write mode locking.
 * -------------------------------------------------------------------------------------
 * 		0xE		|	RESERVED		|		16		|  R/W	|
 * -------------------------------------------------------------------------------------
 * 		0xF		|	CRCCFG			|		16		|	R	|	CRC checksum for the device configuration.
 * -------------------------------------------------------------------------------------
 *
 */


/*
 * Physical values
 */

// Reference voltage
#define VREF									((float)3.0)

/*
 * Status bits masks
 */
#define ADC_STATUS_POR_POS						0u
#define ADC_STATUS_POR_MASK						(1<<ADC_STATUS_POR_POS)

#define ADC_STATUS_CRC_ERROR_POS				1u
#define ADC_STATUS_CRC_ERROR_MASK				(1<<ADC_STATUS_CRC_ERROR_POS)

#define ADC_STATUS_DATA_READY_POS				2u
#define ADC_STATUS_DATA_READY_MASK				(1<<ADC_STATUS_DATA_READY_POS)




/*
 ****************************** ADC CONFIG0 Structure **********************************
 */
typedef struct
{
	uint8_t ADCMode:2;
	uint8_t CS_SEL:2;
	uint8_t CLK_SEL:2;
	uint8_t CONFIG0:2;							// we do not use it??

}ADC_Config0_t;

/*
 * Config0 Defines:
 */
//ADC modes:
#define ADC_SHUTDOWN_MODE						0x00u
#define ADC_STANDBY_MODE						0x02u
#define ADC_CONVERSION_MODE						0x03u

// Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/Sink on VIN-)
#define NO_CURRENT_SOURCE						0x00u			// No current source is applied to the ADC inputs
#define CURRENT_900_NA							0x01u			// 0.9 μA is applied to the ADC inputs
#define CURRENT_3700_NA							0x02u			// 3.7 μA is applied to the ADC inputs
#define CURRENT_15_UA							0x03u			// 15 μA is applied to the ADC inputs

// Clock Selection:
#define EXTERNAL_DIGITAL_CLOCK					0x00u			// External digital clock
#define INTERNAL_CLOCK_NO_OUTPUT				0x02u			// Internal clock is selected and no clock output is present on the CLK pin
#define INTERNAL_CLOCK_AMCLK_OUTPUT				0x03u			// Internal clock is selected and AMCLK is present on the analog master clock output pin

// TO DO Full Shutdown Mode Enable????????????

/*
 ****************************** ADC CONFIG1 Structure **********************************
 */
typedef struct
{
	uint8_t RESERVED:2;				// Should always be set to ‘00’
	uint8_t OSR:4;					// Oversampling Ratio (OSR) for Delta-Sigma A/D Conversion
	uint8_t PRE:2;					// Prescaler Value Selection for AMCLK

}ADC_Config1_t;

// Oversampling Ratio (OSR) for Delta-Sigma A/D Conversion:
#define OSR_32									0x00u 			// OSR: 32
#define OSR_64									0x01u 			// OSR: 64
#define OSR_128									0x02u 			// OSR: 128
#define OSR_256									0x03u 			// OSR: 256
#define OSR_512									0x04u 			// OSR: 512
#define OSR_1024								0x05u 			// OSR: 1024
#define OSR_2048								0x06u 			// OSR: 2048
#define OSR_4096								0x07u 			// OSR: 4096
#define OSR_8192								0x08u 			// OSR: 8192
#define OSR_16384								0x09u 			// OSR: 16384
#define OSR_20480								0x0Au 			// OSR: 20480
#define OSR_24576								0x0Bu 			// OSR: 24576
#define OSR_40960								0x0Cu 			// OSR: 40960
#define OSR_49152								0x0Du 			// OSR: 49152
#define OSR_81920								0x0Eu 			// OSR: 81920
#define OSR_98304								0x0Fu 			// OSR: 98304

// Prescaler Value Selection for AMCLK:
#define AMCLK_MCLK_DIV0							0x00u			// AMCLK = MCLK
#define AMCLK_MCLK_DIV2							0x01u			// AMCLK = MCLK/2
#define AMCLK_MCLK_DIV4							0x02u			// AMCLK = MCLK/4
#define AMCLK_MCLK_DIV8							0x03u			// AMCLK = MCLK/8

/*
 ****************************** ADC CONFIG2 Structure **********************************
 */
typedef struct
{
	uint8_t RESERVED:2;				// Should always be set to ‘11’
	uint8_t AZ_MUX:1;				// Auto-Zeroing MUX Setting
	uint8_t GAIN:3;					// ADC Gain Selection
	uint8_t BOOST:2;				// ADC Bias Current Selection

}ADC_Config2_t;

// Auto-Zeroing MUX Setting
#define AZ_MUX_DISABLED							0x00u			// Analog input multiplexer auto-zeroing algorithm is disabled
#define AZ_MUX_ENABLED							0x01u			// ADC auto-zeroing algorithm is enabled. This setting multiplies by two the conversion time and
																// does not allow Continuous Conversion mode operation (which is then replaced by a series of
																// consecutive One-Shot mode conversions).
// ADC Gain Selection
#define ADC_GAIN_1_DIV_3							0x00u			// Gain is x1/3
#define ADC_GAIN_1									0x01u			// Gain is x1
#define ADC_GAIN_2									0x02u			// Gain is x2
#define ADC_GAIN_4									0x03u			// Gain is x4
#define ADC_GAIN_8									0x04u			// Gain is x8
#define ADC_GAIN_16									0x05u			// Gain is x16
#define ADC_GAIN_32									0x06u			// Gain is x32 (x16 analog, x2 digital)
#define ADC_GAIN_64									0x07u			// Gain is x64 (x16 analog, x4 digital)

// ADC Bias Current Selection
#define BOOST_CURRENT_05						0x00u			// ADC channel has current x 0.5
#define BOOST_CURRENT_066						0x01u			// ADC channel has current x 0.66
#define BOOST_CURRENT_1		 					0x02u			// ADC channel has current x 1
#define BOOST_CURRENT_2							0x03u			// ADC channel has current x 2

/*
 ****************************** ADC CONFIG3 Structure **********************************
 */
typedef struct
{
	uint8_t EN_GAINCAL:1;			// Enable Digital Gain Calibration
	uint8_t EN_OFFCAL:1;			// Enable Digital Offset Calibration
	uint8_t EN_CRCCOM:1;			// CRC Checksum Selection on Read Communications (does not affect CRCCFG calculations)
	uint8_t CRC_FORMAT:1;			// CRC checksum format selection on read communications (does not affect CRCCFG coding)
	uint8_t	DATA_FORMAT:2;			// ADC Output Data Format Selection
	uint8_t CONV_MODE:2;			// Conversion Mode Selection

}ADC_Config3_t;

// Enable Digital Gain Calibration
#define GAINCAL_DISABLED						0x00u			// Gain Calibration is disabled
#define GAINCAL_ENABLED							0x01u			// Gain Calibration is enabled

// Enable Digital Offset Calibration
#define OFFCAL_DISABLED							0x00u			// Offset Calibration is disabled
#define OFFCAL_ENABLED							0x01u			// Offset Calibration is enabled

// CRC Checksum Selection
#define CRCCOM_DISABLED							0x00u			// CRC on communication is disabled
#define CRCCOM_ENABLED							0x01u			// CRC on communication is enabled

// CRC checksum format selection
#define CRC_FORMAT_CRC16						0x00u			// CRC-16 only (16-bit format)
#define CRC_FORMAT_CRC32						0x01u			// CRC-16 followed by 16 zeros

// ADC Output Data Format
#define ADC_DATA_FORMAT_16BIT					0x00u			// 16-bit (default ADC coding): 16-bit ADC data; does not allow overrange (ADC code locked to 0xFFFF or 0x8000)
#define ADC_DATA_FORMAT_16BIT_LEFT				0x01u			// 32-bit (16-bit left justified data): 16-bit ADC data plus 0x0000 (16-bit); does not allow overrange (ADC code locked to 0xFFFF or 0x8000)
#define ADC_DATA_FORMAT_17BIT_RIGHT				0x02u			// 32-bit (17-bit right justified data): SGN extension (16-bit) plus 16-bit ADC data; allows overrange with the SGN extension
#define ADC_DATA_FORMAT_17BIT_RIGHT_CNL_ID		0x03u			// 32-bit (17-bit right justified data plus Channel ID): CHID[3:0] plus SGN extension (12 bits) plus 16-bit ADC data; allows overrange with the SGN extension

// Conversion Mode
#define CONV_MODE_ONE_SHOT_SHUTDOWN				0x00u			// One-shot conversion or one-shot cycle in SCAN mode and sets ADC_MODE[1:0] to ‘0x’ (ADC
																// Shutdown) at the end of the conversion or at the end of the conversion cycle in SCAN mode (default).
#define CONV_MODE_ONE_SHOT_STANDBY				0x02u			// One-shot conversion or one-shot cycle in SCAN mode and sets ADC_MODE[1:0] to ‘10’ (Standby) at
																// the end of the conversion or at the end of the conversion cycle in SCAN mode
#define CONV_MODE_CONTINUOUS					0x03u			// Continuous Conversion mode or continuous conversion cycle in SCAN mode


/*
 ********************************** IRQ Structure ***************************************
 */
typedef struct
{
	uint8_t EN_STP:1;				// Enable Conversion Start Interrupt Output
	uint8_t EN_FASTCMD:1;			// Enable Fast Commands in the COMMAND Byte
	uint8_t IRQ_MODE0:1;			// IRQ Pin Inactive State Selection
	uint8_t IRQ_MODE1:1;			// IRQ/MDAT Selection
	uint8_t nPOR_STATUS:1;			// POR Status Flag
	uint8_t nCRCCFG_STATUS:1;		// CRC Error Status Flag Bit for Internal Registers
	uint8_t nDR_STATUS:1;			// Data Ready Status Flag
	uint8_t Unimplemented:1;		// not implemented inside ADC, Read as 0
}ADC_IRQ_t;

// Enable Conversion Start Interrupt Output
#define CONVERSATION_START_INTERRUPT_DISABLED	0x00u			// Disable Conversion Start Interrupt Output
#define CONVERSATION_START_INTERRUPT_ENABLED	0x01u			// Enable Conversion Start Interrupt Output

// Enable Fast Commands in the COMMAND Byte
#define FAST_COMMAND_DISABLED					0x00u			// Fast commands are disabled
#define FAST_COMMAND_ENABLED					0x01u			// Fast commands are enabled (

// IRQ Pin Inactive State Selection (IRQ_MODE0)
#define IRQ_PIN_MODE_HIGH_Z						0x00u			// The Inactive state is High-Z (requires a pull-up resistor to DVDD)
#define IRQ_PIN_MODE_LOGIC_HIGH					0x01u			// The Inactive state is logic high (does not require a pull-up resistor to DVDD)

// IRQ Pin Inactive State Selection (IRQ_MODE1) IRQ/MDAT Selection
#define MDAT_PIN_MODE_ALL_SELECTED				0x00u			// IRQ output is selected; all interrupts can appear on the IRQ/MDAT pin
#define MDAT_PIN_MODE_MDAT_SELECTED				0x01u			// MDAT output is selected; only POR and CRC interrupts can be present on this pin
																// and take priority over the MDAT output

// POR (Power-on Reset) Status Flag
#define POR_OCCURED								0x00u			// POR has occurred since the last reading
#define POR_NOT_OCCURED							0x01u			// POR has not occurred since the last reading

// CRC Error Status Flag Bit for Internal Registers
#define CRC_OCCURED								0x00u			// CRC error has occurred for the Configuration registers
#define CRC_NOT_OCCURED							0x01u			// CRC error has not occurred for the Configuration registers

// Data Ready Status Flag
#define	DATA_READY								0x00u			// New ADCDATA ready for reading
#define OLD_DATA								0x01u			// ADCDATA has not been updated since last reading or last Reset


/*
 ********************************** MUX Structure ***************************************
 */
typedef struct
{
	uint8_t MUX_VinMinus:4;
	uint8_t MUX_VinPlus:4;
}ADC_MUX_t;

// Multiplexers defines (for both differential inputs)
#define MUX_CH0									0x00u			// Channel 0
#define MUX_CH1									0x01u			// Channel 1
#define MUX_CH2									0x02u			// Channel 2
#define MUX_CH3									0x03u			// Channel 3
#define MUX_CH4									0x04u			// Channel 4
#define MUX_CH5									0x05u			// Channel 5
#define MUX_CH6									0x06u			// Channel 6
#define MUX_CH7									0x07u			// Channel 7
#define MUX_AGND								0x08u			// Connected to Ground
#define MUX_AVDD								0x09u			// Connected to Analog Power
#define MUX_REFIN_PLUS							0x0Bu			// Connected to Positive Reference Voltage
#define MUX_REFIN_MINUS							0x0Cu			// Connected to Negative Reference Voltage
#define MUX_TEMP_DIODE_P						0x0Du			// Internal Temperature Sensor Diode P (TEMP Diode P)
#define MUX_TEMP_DIODE_M						0x0Eu			// Internal Temperature Sensor Diode M (TEMP Diode M)
#define MUX_VCM									0x0Fu			// Internal VCM

// Selects the internal temperature sensor diode and forces a fixed current through it. For a correct temperature
// reading, the MUX[7:0] selection should be equal to 0xDE.

/*
 ********************************** SCAN Structure **************************************
 */

typedef struct
{
	uint16_t SCAN_Ch_Select;		// ??????????
	uint8_t	Unimplemented:4;
	uint8_t RESERVED:1;				// Shall be set to "0"
	uint8_t DelayTime:3;
}ADC_SCAN_t;

// Delay Time (TDLY_SCAN) Between Each Conversion During a SCAN Cycle
#define ADC_DELAY_0								0x00u			// No Delay
#define ADC_DELAY_8_DMCLK						0x01u			// 8 * DMCLK
#define ADC_DELAY_16_DMCLK						0x02u			// 16 * DMCLK
#define ADC_DELAY_32_DMCLK						0x03u			// 32 * DMCLK
#define ADC_DELAY_64_DMCLK						0x04u			// 64 * DMCLK
#define ADC_DELAY_128_DMCLK						0x05u			// 128 * DMCLK
#define ADC_DELAY_256_DMCLK						0x06u			// 256 * DMCLK
#define ADC_DELAY_512_DMCLK						0x07u			// 512 * DMCLK

/*
 ****************************************************************************************
 ******************************* ADC Handle Structure ***********************************
 ****************************************************************************************
 */
typedef struct
{
	uint32_t ADCData;				// ADC Data Output Code
	ADC_Config0_t Config0;			// CONFIG0 Register
	ADC_Config1_t Config1;			// CONFIG1 Register
	ADC_Config2_t Config2;			// CONFIG2 Register
	ADC_Config3_t Config3;			// CONFIG3 Register
	ADC_IRQ_t IRQ;					// INTERRUPT REQUEST REGISTER
	ADC_MUX_t MUX;					// MULTIPLEXER REGISTER
	ADC_SCAN_t SCAN;				// SCAN MODES SETTINGS REGISTER
	uint32_t TIMER;					// TIMER DELAY VALUE REGISTER
									// Selection Bits for Time Interval (TTIMER_SCAN) Between Two Consecutive SCAN Cycles
	uint32_t OFFSETCAL;				// OFFSET CALIBRATION REGISTER
	uint32_t GAINCAL;				// GAIN CALIBRATION REGISTER
	uint32_t RESERVED1;				// Should be set to 0x900000
	uint8_t RESERVED2;				// Should be set to 0x50
	uint8_t LOCK;					// to unlock all register, keep default value: 0xA5
	uint16_t RESERVED3;				// Should be set for, MCP3461: 0x0008, MCP3462: 0x0009,	MCP3464: 0x000B
	uint16_t CRCCFG;				// CRC-16 Checksum Value
									// CRC-16 checksum is continuously calculated internally based on the register map configuration
									// settings when the device is locked (LOCK[7:0] is different than 0xA5)
}ADC_Handle_t;

/*
 * The last byte of the GAINCAL register
 * (GAINCAL[7:0]) is ignored and internally reset to 0x00
 * during the calibration, therefore, the multiplication just
 * takes into account the GAINCAL[23:8] bits.
 */

// Hardware device address:
// There 4 possible addresses which are defined by manufacturing code on the chip
#define ADC_DEVICE_ADDRESS						0x01u

/*
 * Registers Addresses
 */
#define ADCDATA_ADDRESS							0x00u		// Latest A/D conversion data output value
#define CONFIG0_ADDRESS							0x01u		// ADC Operating mode, Master Clock mode and Input Bias Current Source mode
#define CONFIG1_ADDRESS							0x02u		// Prescale and OSR settings.
#define CONFIG2_ADDRESS							0x03u		// ADC boost and gain settings, auto-zeroing settings for analog multiplexer, voltage reference and ADC.
#define CONFIG3_ADDRESS							0x04u		// Conversion mode, data and CRC format settings, enable for CRC on communications, enable for digital offset and gain error calibrations.
#define IRQ_ADDRESS								0x05u		// IRQ Status bits and IRQ mode settings, enable for Fast commands and for conversion start pulse.
#define MUX_ADDRESS								0x06u		// Analog multiplexer input selection (MUX mode only).
#define SCAN_ADDRESS							0x07u		// SCAN mode settings.
#define TIMER_ADDRESS							0x08u		// Delay value for TIMER between each SCAN cycle.
#define OFFSETCAL_ADDRESS						0x09u		// ADC digital offset calibration value.
#define GAINCAL_ADDRESS							0x0Au		// ADC digital gain calibration value.
#define RESERVED1_ADDRESS						0x0Bu		//
#define RESERVED2_ADDRESS						0x0Cu		//
#define LOCK_ADDRESS							0x0Du		// Password value for SPI Write mode locking.
#define RESERVED3_ADDRESS						0x0Eu		//
#define CRCCFG_ADDRESS							0x0Fu		// CRC checksum for the device configuration.

/*
 * Fast commands Defines
 */
#define FC_START_CONVERSION						0x28u		// ADC Conversion Start/Restart Fast Command (Overwrites ADC_MODE[1:0] = 11)
#define FC_STANDBY_MODE							0x2Cu		// ADC Standby Mode Fast Command (Overwrites ADC_MODE[1:0] = 10)
#define FC_SHUTDOWN_MODE						0x30u		// ADC Shutdown Mode Fast Command (Overwrites ADC_MODE[1:0] = 00)
#define FC_FULL_SHUTDOWN_MODE					0x34u		// Full Shutdown Mode Fast Command (Overwrites CONFIG0[7:0] = 0x00)
#define FC_DEVICE_FULL_RESET					0x38u		// Device Full Reset Fast Command (Resets Whole Register Map to Default Value)

/*
 * Other commands
 */
#define COMMAND_STATIC_READ						0x01u		// Static Read
#define COMMAND_INCREMENTAL_WRITE				0x02u		// Incremental Write
#define COMMAND_INCREMENTAL_READ				0x03u		// Incremental Read


#define COMMAND_DUMMY							0x00u		// Command to send to ADC without pushing to do any action,
															// but to have an opportunity to check status byte of the ADC
/*
 * Functions prototypes:
 */
uint8_t ADC_Init(ADC_Handle_t *pADC);						// ADC initialization function, return ERROR???

uint32_t ADC_Static_Read(ADC_Handle_t *pADC, uint8_t RegisterAddress);
uint8_t ADC_Incremental_Write(ADC_Handle_t *pADC, uint8_t RegisterAddress, uint8_t AmountToWrite);


void ADC_Standby(ADC_Handle_t *pADC);
void ADC_Shutdown(ADC_Handle_t *pADC);
void ADC_Full_Shutdown(ADC_Handle_t *pADC);
void ADC_Full_Reset(ADC_Handle_t *pADC);
void ADC_Start_Conversion(ADC_Handle_t *pADC);

float ADC_Get_Measured_DATA(ADC_Handle_t *pADC);

uint8_t ADC_Get_Status_Byte(ADC_Handle_t *pADC);
uint8_t ADC_Check_Errors(uint8_t StatusByte);
void ADC_Proccess_Data(void);

/*
 * ---------------------------------------- TO BE DONE: ------------------------------------------------
 */
uint8_t ADC_Offset_Calibration(ADC_Handle_t *pADC);
uint8_t ADC_Gain_Calibration(ADC_Handle_t *pADC);

uint8_t * ADC_Incremental_Read(ADC_Handle_t *pADC, uint8_t RegisterAddress, uint8_t AmountToRead);

/*
 * -----------------------------------------------------------------------------------------------------
 */
