#ifndef ADS131_H
#define ADS131_H

#include <stdbool.h>
#include <stdint.h>

/// SPI Configuration : 8Bits, CPOL=LOW(0), CPHA=2EDGE(1), Max speed: 15MHz
#define USE_MACRO_DELAY         1                           // 0: Use handler delay ,So you have to set ADC_Delay_US in Handler | 1: use Macro delay, So you have to set MACRO_DELAY_US Macro
//#define MACRO_DELAY_US(x)                                   // If you want to use Macro delay, place your delay function in microseconds here
#define Debug_Enable                                        // Uncomment if you want to use (depends on printf in stdio.h)
#define ADCValueToVoltage(x) (x * 2.4 /*VREFF*/ / 0x7FFFFF) // Use this to conver ADC value to Voltage


// Delay configuration: (DO NOT EDIT THIS BLOCK)
#if USE_MACRO_DELAY == 0
#define Delay_US(x)   ADS131_Handler->ADC_Delay_US(x)
#else
#define Delay_US(x)   MACRO_DELAY_US(x)
#ifndef MACRO_DELAY_US
#error "MACRO_DELAY_US is not defined. Please Use handler delay or config MACRO_DELAY_US macro, You can choose it on USE_MACRO_DELAY define"
#endif
#endif

// Input Values :
typedef struct ADS131_Handler_s {
  void (*ADC_CS_HIGH)(void);          // Must be initialized
  void (*ADC_CS_LOW)(void);           // Must be initialized
  void (*ADC_START_HIGH)(void);       // If you don't want to use software start, the START pin must be pulled down then pass this as NULL
  void (*ADC_START_LOW)(void);        // If you don't want to use software start, the START pin must be pulled down then pass this as NULL
  void (*ADC_RESET_HIGH)(void);       // If you want to use software reset, the RESET pin must be pulled up then pass this as NULL
  void (*ADC_RESET_LOW)(void);        // If you want to use software reset, the RESET pin must be pulled up then pass this as NULL
  void (*ADC_Transmit)(uint8_t Data); // Must be initialized
  uint8_t (*ADC_Receive)(void);       // Must be initialized
  uint8_t (*ADC_DRDY_Read)(void);     // Can be initialized
  void (*ADC_Delay_US)(uint32_t);     // If you want to use Macro delay, you have to enable - define, Otherwise This function must be initialized!
} ADS131_Handler;

typedef enum ADS131_DataRate_e {
  _64kSPS_ = 0, // 64 KSPS | RESOLUTION: 16 Bits
  _32kSPS_ = 1, // 32 KSPS | RESOLUTION: 16 Bits
  _16kSPS_ = 2, // 16 KSPS | RESOLUTION: 24 Bits
  _08kSPS_ = 3, //  8 KSPS | RESOLUTION: 24 Bits
  _04kSPS_ = 4, //  4 KSPS | RESOLUTION: 24 Bits
  _02kSPS_ = 5, //  2 KSPS | RESOLUTION: 24 Bits
  _01KSPS_ = 6  //  1 KSPS | RESOLUTION: 24 Bits
} ADS131_DataRate;

typedef enum ADS131_IntRefVolt_e {
  _2V4_ = 0, // VREFP is set to 2.4 V
  _4V__ = 1  // VREFP is set to   4 V
} ADS131_IntRefVolt;

typedef enum ADS131_OpAmpRef_e {
  TheOPAMPPpin = 0, // Noninverting input connected to the OPAMPP pin
  AVDD_AVSS_2 = 1   // Noninverting input connected to (AVDD + AVSS) / 2
} ADS131_OpAmpRef;

typedef struct ADS131_Parameters_s {
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
} ADS131_Parameters;
typedef enum ADS131_PGAgain_e {
  _01_ = 1, // Gain  1
  _02_ = 2, // Gain  2
  _04_ = 4, // Gain  4
  _08_ = 5, // Gain  8
  _12_ = 6  // Gain 12
} ADS131_PGAgain;

typedef enum ADS131_MUXChInput_e {
  NormalInput = 0,  // Normal input
  InputShorted = 1, // Input shorted to (AVDD + AVSS) / 2 (for offset or noise measurements)
  MVDDSupply = 3,   // MVDD for supply measurement
  TempSensor = 4,   // Temperature sensor
  TestSignal = 5    // Test signal
} ADS131_MUXChInput;

typedef struct ADS131_ChannelsConfig_s {
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
} ADS131_ChannelsConfig;

typedef struct ADS131_GPIOConfig_s {
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
} ADS131_GPIOConfig;

// Public Functions :
/* ADS131_Init :
* @note: Default in Continuous Mode, 1kSPS and VREFF:2.4V | Fault Thershold is set to High-side: 95%, Low-side 5% (default)
 */
void ADS131_Init(ADS131_Handler *ADC_Handler, ADS131_Parameters *Parameters, ADS131_ChannelsConfig *ChannelsConfig, ADS131_GPIOConfig *GPIOConfig);

/* ADS131_ReadData :
 * @note: Call This function when DRDY pin got LOW
 */
void ADS131_ReadData(ADS131_Handler *ADC_Handler, uint8_t *State /* 3 Elements ([0]: MSB) */, int32_t *ChSamples /* 8 Element ([0]: Ch1)*/);

/* ADS131_ConfigGPIO :
 */
void ADS131_ConfigGPIO(ADS131_Handler *ADC_Handler, ADS131_GPIOConfig *GPIOConfig);

/* ADS131_ReadGPIO :
 */
void ADS131_ReadGPIO(ADS131_Handler *ADC_Handler, bool *GPIOstate /* 4 Element ([0]: GPIO1)*/);

// ----------------------------------------------------------------------------------------------------
// NOT IMPLEMENTED YET :

void ADS131_EnableStartContinuousMode(ADS131_Handler *ADC_Handler);
void ADS131_StopDisableContinuousMode(ADS131_Handler *ADC_Handler);

void ADS131_Config1(ADS131_Handler *ADC_Handler, bool DaisyChain, bool OscillatorClkOutput, ADS131_DataRate DataRate);
// void ADS131_Config2(ADS131_Handler *ADC_Handler); // Config2 is for Test!
void ADS131_Config3(ADS131_Handler *ADC_Handler, ADS131_IntRefVolt IntRefVolt, ADS131_OpAmpRef OpAmpRef, bool OpAmpPowerDown);
void ADS131_ChannelConfig(ADS131_Handler *ADC_Handler, uint8_t ChannelNumber /* 0xFF for ALL Channels */, ADS131_PGAgain PGAgain, ADS131_MUXChInput MUXChInput);

void ADS131_StartConversion(ADS131_Handler *ADC_Handler);
void ADS131_StopConversion(ADS131_Handler *ADC_Handler);

void ADS131_ReadDataCommand(ADS131_Handler *ADC_Handler); // For non-Continuous Mode

#endif
