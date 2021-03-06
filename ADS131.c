/**
 **********************************************************************************
 * @file   ADS131.c
 * @author Ali Moallem (https://github.com/AliMoal)
 * @brief  
 *         Functionalities of the this file:
 *          + 
 *          + 
 *          + 
 **********************************************************************************
 *
 *! Copyright (c) 2021 Mahda Embedded System (MIT License)
 *!
 *! Permission is hereby granted, free of charge, to any person obtaining a copy
 *! of this software and associated documentation files (the "Software"), to deal
 *! in the Software without restriction, including without limitation the rights
 *! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *! copies of the Software, and to permit persons to whom the Software is
 *! furnished to do so, subject to the following conditions:
 *!
 *! The above copyright notice and this permission notice shall be included in all
 *! copies or substantial portions of the Software.
 *!
 *! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *! SOFTWARE.
 *!
 **********************************************************************************
 **/

//* Private Includes -------------------------------------------------------------- //
#include "ADS131.h"

//* Private Defines and Macros ---------------------------------------------------- //
#define ADS131_RawToAdcValue(oneData) ((int32_t)(((oneData[0] << 16) | (oneData[1] << 8) | (oneData[2])) << 8) / 256)
#define EnableRegistersInContinuousMode() 	ADC_Handler->ADC_CS_LOW(); \
                                            Delay_US(5); \
                                            ADC_Handler->ADC_Transmit(STOP); \
                                            Delay_US(5); \
                                            ADC_Handler->ADC_Transmit(SDATAC); \
                                            Delay_US(5); \
                                            ADC_Handler->ADC_CS_HIGH(); \
                                            Delay_US(5)

#define DisableRegistersInContinuousMode()  Delay_US(5); \
                                            ADC_Handler->ADC_CS_LOW(); \
                                            Delay_US(5); \
                                            ADC_Handler->ADC_Transmit(RDATAC); \
                                            Delay_US(5); \
                                            ADC_Handler->ADC_Transmit(START); \
                                            Delay_US(5); \
                                            ADC_Handler->ADC_CS_HIGH(); \
                                            Delay_US(5)


//* Others ------------------------------------------------------------------------ //
#ifdef Debug_Enable
#include <stdio.h> // for debug
#define PROGRAMLOG(arg...) printf(arg)
#else
#define PROGRAMLOG(arg...)
#endif

/**
 ** ==================================================================================
 **                                ##### Enums #####                               
 ** ==================================================================================
 **/

typedef enum
ADS131Commands_s {
	// SYSTEM COMMANDS
	WAKEUP		 		= 0x02,	// Wake-up from standby mode
	STANDBY		 		=	0x04, // Enter standby mode
	RESET					= 0x06, // Reset the device
	START					= 0x08, // Start or restart (synchronize) conversions
	STOP					=	0x0A, // Stop conversions
	OFFSETCAL			= 0x1A, // Channel offset calibration
	// DATA READ COMMANDS
	RDATAC				= 0x10, // Enable read data continuous mode. This mode is the default mode at power-up. When in RDATAC mode, the RREG command is ignored.
	SDATAC				= 0x11, // Stop read data continuous mode
	RDATA					=	0x12, // Read data by command
	// REGISTER READ COMMANDS
	RREG					= 0x20, // Read registers
	WREG					= 0x40	// Write registers
} ADS131Commands_t;

typedef enum
ADS131Register_s {
	// DEVICE SETTINGS (Read-Only Registers)
	ID						=	0x00, // RESET VALUE: 0xD2
	// GLOBAL SETTINGS ACROSS CHANNELS
	CONFIG1				= 0x01, // RESET VALUE: 0x94
	CONFIG2				=	0x02, // RESET VALUE: 0xE0
	CONFIG3				= 0x03, // RESET VALUE: 0xE0
	FAULT					= 0x04,	// RESET VALUE: 0x00
	// CHANNEL-SPECIFIC SETTINGS
	CH1SET				= 0x05, // RESET VALUE: 0x10
	CH2SET				= 0x06, // RESET VALUE: 0x10
	CH3SET				= 0x07, // RESET VALUE: 0x10
	CH4SET				= 0x08, // RESET VALUE: 0x10
	CH5SET				= 0x09, // RESET VALUE: 0x10
	CH6SET				= 0x0A, // RESET VALUE: 0x10
	CH7SET				= 0x0B, // RESET VALUE: 0x10
	CH8SET				= 0x0C, // RESET VALUE: 0x10
	// FAULT DETECT STATUS REGISTERS (Read-Only Registers)
	FAULT_STATP		= 0x12, // RESET VALUE: 0x00
	FAULT_STATN		= 0x13, // RESET VALUE: 0x00
	// GPIO SETTINGS
	GPIO					= 0x14	// RESET VALUE: 0x0F
} ADS131Register_t;

/**
 *! ==================================================================================
 *!                          ##### Private Functions #####                               
 *! ==================================================================================
 **/

static uint8_t
ADS131_ReadReg (ADS131_Handler_t *ADC_Handler,ADS131Register_t ADS131REG)
{
	uint8_t RecByte = 0;
	ADC_Handler->ADC_CS_LOW();
	Delay_US(5);
	ADC_Handler->ADC_Transmit(RREG | ADS131REG);
	Delay_US(5);
	ADC_Handler->ADC_Transmit(0);
	Delay_US(5);
	RecByte = ADC_Handler->ADC_Receive();
	Delay_US(5);
	ADC_Handler->ADC_CS_HIGH();
	return RecByte;
};

static void
ADS131_WriteReg (ADS131_Handler_t *ADC_Handler,ADS131Register_t ADS131REG, uint8_t RegisterValue)
{
	ADC_Handler->ADC_CS_LOW();
	Delay_US(5);
	ADC_Handler->ADC_Transmit(WREG | ADS131REG);
	Delay_US(5);
	ADC_Handler->ADC_Transmit(0);
	Delay_US(5);
	ADC_Handler->ADC_Transmit(RegisterValue);
	Delay_US(5);
	ADC_Handler->ADC_CS_HIGH();
};

/**
 ** ==================================================================================
 **                           ##### Public Functions #####                               
 ** ==================================================================================
 **/

/**
 * @brief  Initializes The ADC and Library
 * @note   Defaults in Continuous Mode, 1kSPS and VREFF:2.4V, Fault Thershold is set to High-side: 95%, Low-side 5%
 * @param  ADC_Handler:     Pointer Of Library Handler
 * @param  Parameters:      Pointer Of ADC Parameters
 * @param  ChannelsConfig:  Pointer Of Channels Configurations
 * @param  GPIOConfig:      Pointer Of GPIO Configurations
 * @retval None
 */
void
ADS131_Init(ADS131_Handler_t *ADC_Handler, ADS131_Parameters_t *Parameters, ADS131_ChannelsConfig_t *ChannelsConfig, ADS131_GPIOConfig_t *GPIOConfig)
{
  if (!ADC_Handler)
     return;
  
	if(ADC_Handler->ADC_RESET_LOW)
		ADC_Handler->ADC_RESET_LOW();
	if(ADC_Handler->ADC_START_LOW)
		ADC_Handler->ADC_START_LOW();
	
	ADC_Handler->ADC_CS_LOW();
	Delay_US(5);
	ADC_Handler->ADC_Transmit(STOP);
	Delay_US(5);
	ADC_Handler->ADC_Transmit(SDATAC);
	Delay_US(5);
	ADC_Handler->ADC_Transmit(RESET);
	Delay_US(5);
	ADC_Handler->ADC_CS_HIGH();
	Delay_US(100);
  
	EnableRegistersInContinuousMode();
  
  PROGRAMLOG("ID: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,ID));
  
  uint8_t RegVal = 0;
  
  if (Parameters)
  {
    RegVal = (0x90) | ((Parameters->DaisyChain ? 0 : 1) << 6) | (Parameters->OscillatorClkOutput << 5) | (Parameters->DataRate);
    ADS131_WriteReg(ADC_Handler,CONFIG1,RegVal);
    PROGRAMLOG("CONFIG1: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CONFIG1));
    
//    RegVal = 0;                                                              // for Test
//    ADS131_WriteReg(ADC_Handler,CONFIG2,RegVal);                             // for Test
//    PROGRAMLOG("CONFIG2: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CONFIG2));     // for Test  
    
    RegVal = (0xC0) | (Parameters->IntRefVolt << 5) | (Parameters->OpAmpRef << 3) | (Parameters->OpAmpPowerDown << 2);
    ADS131_WriteReg(ADC_Handler,CONFIG3,RegVal);
    PROGRAMLOG("CONFIG3: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CONFIG3));
  }
  else
  {
    RegVal = 0x96; // 1kSPS
    ADS131_WriteReg(ADC_Handler,CONFIG1,RegVal);
    PROGRAMLOG("CONFIG1 (default): 0x%X - Must be 0x96\r\n",ADS131_ReadReg(ADC_Handler,CONFIG1));
    RegVal = 0xC0; // VREFF: 2.4V
    ADS131_WriteReg(ADC_Handler,CONFIG3,RegVal);
    PROGRAMLOG("CONFIG3 (default): 0x%X - Must be 0xC1 or 0xC0\r\n",ADS131_ReadReg(ADC_Handler,CONFIG3));
  }
  
  if (ChannelsConfig)
  {
    RegVal = (ChannelsConfig->Ch1PowerDown << 7) | (ChannelsConfig->Ch1PGA << 4) | (ChannelsConfig->Ch1MUX);
    ADS131_WriteReg(ADC_Handler,CH1SET,RegVal);
    PROGRAMLOG("CH1SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH1SET));
    RegVal = (ChannelsConfig->Ch2PowerDown << 7) | (ChannelsConfig->Ch2PGA << 4) | (ChannelsConfig->Ch2MUX);
    ADS131_WriteReg(ADC_Handler,CH2SET,RegVal);
    PROGRAMLOG("CH2SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH2SET));
    RegVal = (ChannelsConfig->Ch3PowerDown << 7) | (ChannelsConfig->Ch3PGA << 4) | (ChannelsConfig->Ch3MUX);
    ADS131_WriteReg(ADC_Handler,CH3SET,RegVal);
    PROGRAMLOG("CH3SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH1SET));
    RegVal = (ChannelsConfig->Ch4PowerDown << 7) | (ChannelsConfig->Ch4PGA << 4) | (ChannelsConfig->Ch4MUX);
    ADS131_WriteReg(ADC_Handler,CH4SET,RegVal);
    PROGRAMLOG("CH4SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH4SET));
    RegVal = (ChannelsConfig->Ch5PowerDown << 7) | (ChannelsConfig->Ch5PGA << 4) | (ChannelsConfig->Ch5MUX);
    ADS131_WriteReg(ADC_Handler,CH5SET,RegVal);
    PROGRAMLOG("CH5SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH5SET));
    RegVal = (ChannelsConfig->Ch6PowerDown << 7) | (ChannelsConfig->Ch6PGA << 4) | (ChannelsConfig->Ch6MUX);
    ADS131_WriteReg(ADC_Handler,CH6SET,RegVal);
    PROGRAMLOG("CH6SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH6SET));
    RegVal = (ChannelsConfig->Ch7PowerDown << 7) | (ChannelsConfig->Ch7PGA << 4) | (ChannelsConfig->Ch7MUX);
    ADS131_WriteReg(ADC_Handler,CH7SET,RegVal);
    PROGRAMLOG("CH7SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH7SET));
    RegVal = (ChannelsConfig->Ch8PowerDown << 7) | (ChannelsConfig->Ch8PGA << 4) | (ChannelsConfig->Ch8MUX);
    ADS131_WriteReg(ADC_Handler,CH8SET,RegVal);
    PROGRAMLOG("CH8SET: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,CH8SET));
  }
  
  if(GPIOConfig)
  {
    RegVal = 0x50;//(GPIOConfig->GPIO4High << 7) | (GPIOConfig->GPIO3High << 6) | (GPIOConfig->GPIO2High << 5) | (GPIOConfig->GPIO1High << 4) |(GPIOConfig->GPIO4Input << 3) | (GPIOConfig->GPIO3Input << 2) | (GPIOConfig->GPIO2Input << 1) | (GPIOConfig->GPIO1Input);
    ADS131_WriteReg(ADC_Handler,GPIO,RegVal);
    PROGRAMLOG("GPIO: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,GPIO));
  }
  
  DisableRegistersInContinuousMode();
};

/**
 * @brief  Reads ADC Data
 * @note   Call This function when DRDY pin got LOW
 * @param  ADC_Handler:    Pointer Of Library Handler
 * @param  State:          Pointer Of ADC Statement    | 3 Elements ([0]: MSB)
 * @param  ChSamples:      Pointer Of Channels Samples | 8 Elements ([0]: Ch1)
 * @retval None
 */
void
ADS131_ReadData(ADS131_Handler_t *ADC_Handler, uint8_t *State /* 3 Elements ([0]: MSB) */,int32_t *ChSamples /* 8 Elements ([0]: Ch1)*/)
{
  ADC_Handler->ADC_CS_LOW();
  Delay_US(5);
  State[0] = ADC_Handler->ADC_Receive();
  Delay_US(1);
  State[1] = ADC_Handler->ADC_Receive();
  Delay_US(1);
  State[2] = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[0].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[0].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[0].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[1].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[1].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[1].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[2].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[2].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[2].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[3].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[3].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[3].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[4].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[4].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[4].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[5].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[5].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[5].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[6].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[6].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[6].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[7].Part3 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[7].Part2 = ADC_Handler->ADC_Receive();
  Delay_US(1);
  ADC_Handler->ChannelsData[7].Part1 = ADC_Handler->ADC_Receive();
  Delay_US(5);
	ADC_Handler->ADC_CS_HIGH();
  ChSamples[0] = ADC_Handler->ChannelsData[0].INT32 / 256; 
  ChSamples[1] = ADC_Handler->ChannelsData[1].INT32 / 256; 
  ChSamples[2] = ADC_Handler->ChannelsData[2].INT32 / 256; 
  ChSamples[3] = ADC_Handler->ChannelsData[3].INT32 / 256; 
  ChSamples[4] = ADC_Handler->ChannelsData[4].INT32 / 256; 
  ChSamples[5] = ADC_Handler->ChannelsData[5].INT32 / 256; 
  ChSamples[6] = ADC_Handler->ChannelsData[6].INT32 / 256; 
  ChSamples[7] = ADC_Handler->ChannelsData[7].INT32 / 256; 
}

/**
 * @brief  Configures GPIO Settings
 * @param  ADC_Handler:   Pointer Of Library Handler
 * @param  GPIOConfig:    Pointer Of GPIOs Configurations
 * @retval None
 */
void
ADS131_ConfigGPIO(ADS131_Handler_t *ADC_Handler,ADS131_GPIOConfig_t *GPIOConfig)
{
  if((!GPIOConfig) || (!ADC_Handler))
    return;
  
  EnableRegistersInContinuousMode();
  
  uint8_t RegVal = (GPIOConfig->GPIO4High << 7) | (GPIOConfig->GPIO3High << 6) | (GPIOConfig->GPIO2High << 5) | (GPIOConfig->GPIO1High << 4) |(GPIOConfig->GPIO4Input << 3) | (GPIOConfig->GPIO3Input << 2) | (GPIOConfig->GPIO2Input << 1) | (GPIOConfig->GPIO1Input);
  ADS131_WriteReg(ADC_Handler,GPIO,RegVal);
  PROGRAMLOG("GPIO: 0x%X\r\n",ADS131_ReadReg(ADC_Handler,GPIO));
  
  DisableRegistersInContinuousMode();
}

/**
 * @brief  Reads GPIO Data
 * @param  ADC_Handler:   Pointer Of Library Handler
 * @param  GPIOstate:     Pointer Of GPIOs Statements | 4 Element ([0]: GPIO1)
 * @retval None
 */
void
ADS131_ReadGPIO(ADS131_Handler_t *ADC_Handler, bool *GPIOstate /* 4 Element ([0]: GPIO1)*/)
{
  EnableRegistersInContinuousMode();
  uint8_t RegVal = ADS131_ReadReg(ADC_Handler,GPIO);
  PROGRAMLOG("GPIO: 0x%X\r\n",RegVal);
  DisableRegistersInContinuousMode();
  GPIOstate[0] = (RegVal >> 4) & 1;
  GPIOstate[1] = (RegVal >> 5) & 1;
  GPIOstate[2] = (RegVal >> 6) & 1;
  GPIOstate[3] = (RegVal >> 7) & 1;
}
