/**
 **********************************************************************************
 * @file   ADS131.h
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

//* Define to prevent recursive inclusion ---------------------------------------- //
#ifndef ADS131_H
#define ADS131_H

#ifdef __cplusplus
extern "C" {
#endif

//* Includes ---------------------------------------------------------------------- //
#include <stdbool.h>
#include <stdint.h>

//? User Configurations and Notes ------------------------------------------------- //
// SPI Configuration: 8Bits, CPOL=LOW(0), CPHA=2EDGE(1), Max speed: 15MHz (referred to Datasheet)
#define USE_MACRO_DELAY         1           // 0: Use handler delay ,So you have to set ADC_Delay_US in Handler | 1: use Macro delay, So you have to set MACRO_DELAY_US Macro
// #define MACRO_DELAY_US(x)                   // If you want to use Macro delay, place your delay function in microseconds here
#define Debug_Enable                        // Uncomment if you want to use (depends on printf in stdio.h)
// #pragma anon_unions                         // Uncomment if you are using Keil software
//? ------------------------------------------------------------------------------- //

//* Defines and Macros ------------------------------------------------------------ //
#define ADCValueToVoltage(x) (x * 2.4 /*VREFF*/ / 0x7FFFFF) // Use this to conver ADC value to Voltage


//! DO NOT USE OR EDIT THIS BLOCK ------------------------------------------------- //
#if USE_MACRO_DELAY == 0
#define Delay_US(x)   ADS131_Handler->ADC_Delay_US(x)
#else
#define Delay_US(x)   MACRO_DELAY_US(x)
#ifndef MACRO_DELAY_US
#error "MACRO_DELAY_US is not defined. Please Use handler delay or config MACRO_DELAY_US macro, You can choose it on USE_MACRO_DELAY define"
#endif
#endif

typedef union
ADS131_OneSample_u {
  struct {
    uint32_t Zero :8; // Always Zero
    uint32_t Part1:8;
    uint32_t Part2:8;
    uint32_t Part3:8;
  };
  int32_t INT32;
} ADS131_OneSample_t;
//! ------------------------------------------------------------------------------- //

/**
 ** ==================================================================================
 **                                ##### Enums #####                               
 ** ==================================================================================
 **/

/**
 * @brief  Data Rate of ADC
 */
typedef enum
ADS131_DataRate_e {
  _64kSPS_ = 0, // 64 KSPS | RESOLUTION: 16 Bits
  _32kSPS_ = 1, // 32 KSPS | RESOLUTION: 16 Bits
  _16kSPS_ = 2, // 16 KSPS | RESOLUTION: 24 Bits
  _08kSPS_ = 3, //  8 KSPS | RESOLUTION: 24 Bits
  _04kSPS_ = 4, //  4 KSPS | RESOLUTION: 24 Bits
  _02kSPS_ = 5, //  2 KSPS | RESOLUTION: 24 Bits
  _01KSPS_ = 6  //  1 KSPS | RESOLUTION: 24 Bits
} ADS131_DataRate_t;

/**
 * @brief  Reference Voltage of ADC
 */
typedef enum
ADS131_IntRefVolt_e {
  _2V4_ = 0, // VREFP is set to 2.4 V
  _4V__ = 1  // VREFP is set to   4 V
} ADS131_IntRefVolt_t;

/**
 * @brief  Reference of OpAmp of ADC
 */
typedef enum
ADS131_OpAmpRef_e {
  TheOPAMPPpin = 0, // Noninverting input connected to the OPAMPP pin
  AVDD_AVSS_2 = 1   // Noninverting input connected to (AVDD + AVSS) / 2
} ADS131_OpAmpRef_t;

/**
 * @brief  PGA Gain of ADC
 */
typedef enum
ADS131_PGAgain_e {
  _01_ = 1, // Gain  1
  _02_ = 2, // Gain  2
  _04_ = 4, // Gain  4
  _08_ = 5, // Gain  8
  _12_ = 6  // Gain 12
} ADS131_PGAgain_t;

/**
 * @brief  Input Multiplexer of ADC
 */
typedef enum
ADS131_MUXChInput_e {
  NormalInput = 0,  // Normal input
  InputShorted = 1, // Input shorted to (AVDD + AVSS) / 2 (for offset or noise measurements)
  MVDDSupply = 3,   // MVDD for supply measurement
  TempSensor = 4,   // Temperature sensor
  TestSignal = 5    // Test signal
} ADS131_MUXChInput_t;

/**
 ** ==================================================================================
 **                               ##### Structs #####                               
 ** ==================================================================================
 **/

/**
 * @brief  Handling Library
 * @note   User MUST configure This at the begining of the program before ADS1230_Init
 */
typedef struct
ADS131_Handler_s {
  void (*ADC_CS_HIGH)(void);          // Must be initialized
  void (*ADC_CS_LOW)(void);           // Must be initialized
  void (*ADC_Transmit)(uint8_t Data); // Must be initialized
  uint8_t (*ADC_Receive)(void);       // Must be initialized
  void (*ADC_START_HIGH)(void);       // Can be initialized (If you don't want to use software start, the START pin must be pulled down then pass this as NULL)
  void (*ADC_START_LOW)(void);        // Can be initialized (If you don't want to use software start, the START pin must be pulled down then pass this as NULL)
  void (*ADC_RESET_HIGH)(void);       // Can be initialized (If you want to use software reset, the RESET pin must be pulled up then pass this as NULL)
  void (*ADC_RESET_LOW)(void);        // Can be initialized (If you want to use software reset, the RESET pin must be pulled up then pass this as NULL)
  uint8_t (*ADC_DRDY_Read)(void);     // Can be initialized 
  void (*ADC_Delay_US)(uint32_t);     //! Must be initialized If You do not use Macro Delay (Place here your delay in MicroSecond)
  ADS131_OneSample ChannelsData[8];   // !!! DO NOT USE OR EDIT THIS !!!
} ADS131_Handler_t;

/**
 * @brief  ADC Parameters
 * @note   User Can configure This at the begining of the program before ADS1230_Init
 */
typedef struct
ADS131_Parameters_s {
  // This bit determines which mode is enabled.
  // 0: Daisy-chain mode
  // 1: Multiple data readback mode (Standard mode)
  bool DaisyChain;
  // This bit determines if the internal oscillator signal is connected to the CLK pin when the CLKSEL pin = 1.
  // 0: Oscillator clock output disabled
  // 1: Oscillator clock output enabled
  bool OscillatorClkOutput;
  // These bits determine the output data rate and resolution
  // See ADS131_DataRate enum
  ADS131_DataRate DataRate;
  // This bit determines the reference voltage, VREFP.
  // See ADS131_IntRefVolt enum
  ADS131_IntRefVolt IntRefVolt;
  // This bit determines whether the op amp noninverting input connects to the OPAMPP pin or to the internally-derived supply (AVDD + AVSS) / 2.
  // See ADS131_OpAmpRef enum
  ADS131_OpAmpRef OpAmpRef;
  // This bit powers down the op amp.
  // 0: Power-down op amp
  // 1: Enable op amp
  bool OpAmpPowerDown;
} ADS131_Parameters_t;

/**
 * @brief  Channels Configurations
 * @note   User Can configure This at the begining of the program before ADS1230_Init
 */
typedef struct
ADS131_ChannelsConfig_s {
  // Channel 1 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch1PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch1PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch1MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 2 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch2PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch2PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch2MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 3 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch3PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch3PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch3MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 4 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch4PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch4PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch4MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 5 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch5PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch5PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch5MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 6 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch6PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch6PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch6MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 7 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch7PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch7PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch7MUX;
  //----------------------------------------------------------------------------------------------------
  // Channel 8 Config :
  // This bit determines the channel power mode for the corresponding channel
  // 0: Normal operation
  // 1: Channel power-down
  bool Ch8PowerDown;
  // These bits determine the PGA gain setting.
  // See ADS131_PGAgain enum
  ADS131_PGAgain Ch8PGA;
  // These bits determine the channel input selection.
  // See ADS131_MUXChInput enum
  ADS131_MUXChInput Ch8MUX;
} ADS131_ChannelsConfig_t;

/**
 * @brief  GPIO Configurations
 * @note   User Can configure This at the begining of the program before ADS1230_Init
 */
typedef struct
ADS131_GPIOConfig_s {
  // GPIO 1 :
  // GPIO control (corresponding to GPIOD). These bits determine if the corresponding GPIOD pin is an input or output.
  // 0: Output
  // 1: Input
  bool GPIO1Input;
  // These bits are used to read and write data to the GPIO ports. When reading the register, the data returned correspond to the
  // state of the GPIO external pins, whether they are programmed as inputs or outputs. As outputs, a write to the GPIOD sets the
  // output value. As inputs, a write to the GPIOD has no effect.
  // 0: LOW
  // 1: HIGH
  bool GPIO1High;
  //----------------------------------------------------------------------------------------------------
  // GPIO 2 :
  // GPIO control (corresponding to GPIOD). These bits determine if the corresponding GPIOD pin is an input or output.
  // 0: Output
  // 1: Input
  bool GPIO2Input;
  // These bits are used to read and write data to the GPIO ports. When reading the register, the data returned correspond to the
  // state of the GPIO external pins, whether they are programmed as inputs or outputs. As outputs, a write to the GPIOD sets the
  // output value. As inputs, a write to the GPIOD has no effect.
  // 0: LOW
  // 1: HIGH
  bool GPIO2High;
  //----------------------------------------------------------------------------------------------------
  // GPIO 3 :
  // GPIO control (corresponding to GPIOD). These bits determine if the corresponding GPIOD pin is an input or output.
  // 0: Output
  // 1: Input
  bool GPIO3Input;
  // These bits are used to read and write data to the GPIO ports. When reading the register, the data returned correspond to the
  // state of the GPIO external pins, whether they are programmed as inputs or outputs. As outputs, a write to the GPIOD sets the
  // output value. As inputs, a write to the GPIOD has no effect.
  // 0: LOW
  // 1: HIGH
  bool GPIO3High;
  //----------------------------------------------------------------------------------------------------
  // GPIO 4 :
  // GPIO control (corresponding to GPIOD). These bits determine if the corresponding GPIOD pin is an input or output.
  // 0: Output
  // 1: Input
  bool GPIO4Input;
  // These bits are used to read and write data to the GPIO ports. When reading the register, the data returned correspond to the
  // state of the GPIO external pins, whether they are programmed as inputs or outputs. As outputs, a write to the GPIOD sets the
  // output value. As inputs, a write to the GPIOD has no effect.
  // 0: LOW
  // 1: HIGH
  bool GPIO4High;
} ADS131_GPIOConfig_t;

/**
 ** ==================================================================================
 **                          ##### Public Functions #####                               
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
ADS131_Init(ADS131_Handler_t *ADC_Handler, ADS131_Parameters_t *Parameters, ADS131_ChannelsConfig_t *ChannelsConfig, ADS131_GPIOConfig_t *GPIOConfig);

/**
 * @brief  Reads ADC Data
 * @note   Call This function when DRDY pin got LOW
 * @param  ADC_Handler:    Pointer Of Library Handler
 * @param  State:          Pointer Of ADC Statement    | 3 Elements ([0]: MSB)
 * @param  ChSamples:      Pointer Of Channels Samples | 8 Elements ([0]: Ch1)
 * @retval None
 */
void
ADS131_ReadData(ADS131_Handler_t *ADC_Handler, uint8_t *State, int32_t *ChSamples);

/**
 * @brief  Configures GPIO Settings
 * @param  ADC_Handler:   Pointer Of Library Handler
 * @param  GPIOConfig:    Pointer Of GPIOs Configurations
 * @retval None
 */
void
ADS131_ConfigGPIO(ADS131_Handler_t *ADC_Handler, ADS131_GPIOConfig_t *GPIOConfig);

/**
 * @brief  Reads GPIO Data
 * @param  ADC_Handler:   Pointer Of Library Handler
 * @param  GPIOstate:     Pointer Of GPIOs Statements | 4 Element ([0]: GPIO1)
 * @retval None
 */
void
ADS131_ReadGPIO(ADS131_Handler_t *ADC_Handler, bool *GPIOstate);

//! NOT IMPLEMENTED YET ----------------------------------------------------------- //

void ADS131_EnableStartContinuousMode(ADS131_Handler_t *ADC_Handler);
void ADS131_StopDisableContinuousMode(ADS131_Handler_t *ADC_Handler);
void ADS131_Config1(ADS131_Handler_t *ADC_Handler, bool DaisyChain, bool OscillatorClkOutput, ADS131_DataRate_t DataRate);
// void ADS131_Config2(ADS131_Handler *ADC_Handler); // Config2 is for Test!
void ADS131_Config3(ADS131_Handler_t *ADC_Handler, ADS131_IntRefVolt_t IntRefVolt, ADS131_OpAmpRef_t OpAmpRef, bool OpAmpPowerDown);
void ADS131_ChannelConfig(ADS131_Handler_t *ADC_Handler, uint8_t ChannelNumber /* 0xFF for ALL Channels */, ADS131_PGAgain_t PGAgain, ADS131_MUXChInput_t MUXChInput);
void ADS131_StartConversion(ADS131_Handler_t *ADC_Handler);
void ADS131_StopConversion(ADS131_Handler_t *ADC_Handler);
void ADS131_ReadDataCommand(ADS131_Handler_t *ADC_Handler); // For non-Continuous Mode


#ifdef __cplusplus
}
#endif
#endif
