/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

//#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
typedef enum {
    UART_0 = (int)UART0_BASE,
    UART_1 = (int)UART1_BASE,
    UART_2 = (int)UART2_BASE
} UARTName;
#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        UART_0

typedef enum {
    I2C_0 = (int)I2C0_BASE,
} I2CName;*/

#define TPM_SHIFT   8
typedef enum {
    PWM_1  = (0 << TPM_SHIFT) | (0),  // FTM0 CH0
    PWM_2  = (0 << TPM_SHIFT) | (1),  // FTM0 CH1
    PWM_3  = (0 << TPM_SHIFT) | (2),  // FTM0 CH2
    PWM_4  = (0 << TPM_SHIFT) | (3),  // FTM0 CH3
    PWM_5  = (0 << TPM_SHIFT) | (4),  // FTM0 CH4
    PWM_6  = (0 << TPM_SHIFT) | (5),  // FTM0 CH5
    PWM_7  = (0 << TPM_SHIFT) | (6),  // FTM0 CH6
    PWM_8  = (0 << TPM_SHIFT) | (7),  // FTM0 CH7
    PWM_9  = (1 << TPM_SHIFT) | (0),  // FTM1 CH0
    PWM_10 = (1 << TPM_SHIFT) | (1),  // FTM1 CH1
	PWM_11 = (2 << TPM_SHIFT) | (0),  // FTM2 CH0
	PWM_12 = (2 << TPM_SHIFT) | (1),  // FTM2 CH1
	PWM_13 = (3 << TPM_SHIFT) | (0),  // FTM3 CH0
	PWM_14 = (3 << TPM_SHIFT) | (1),  // FTM3 CH1
	PWM_15 = (3 << TPM_SHIFT) | (2),  // FTM3 CH2
	PWM_16 = (3 << TPM_SHIFT) | (3),  // FTM3 CH3
	PWM_17 = (3 << TPM_SHIFT) | (4),  // FTM3 CH4
	PWM_18 = (3 << TPM_SHIFT) | (5),  // FTM3 CH5
	PWM_19 = (3 << TPM_SHIFT) | (6),  // FTM3 CH6
	PWM_20 = (3 << TPM_SHIFT) | (7),  // FTM3 CH7
} PWMName;

typedef enum {
	ADC0_DE0  = (0 << TPM_SHIFT) | 0,
	ADC0_DE1  = (0 << TPM_SHIFT) | 1,
    ADC0_DE2  = (0 << TPM_SHIFT) | 2,
	ADC0_DE3  = (0 << TPM_SHIFT) | 3,
	ADC0_SE4b = (0 << TPM_SHIFT) | 4,
    ADC0_SE5b = (0 << TPM_SHIFT) | 5,
    ADC0_SE6b = (0 << TPM_SHIFT) | 6,
    ADC0_SE7b = (0 << TPM_SHIFT) | 7,
    ADC0_SE8  = (0 << TPM_SHIFT) | 8,
    ADC0_SE9  = (0 << TPM_SHIFT) | 9,
	ADC0_SE10 = (0 << TPM_SHIFT) | 10,
	ADC0_SE11 = (0 << TPM_SHIFT) | 11,
    ADC0_SE12 = (0 << TPM_SHIFT) | 12,
    ADC0_SE13 = (0 << TPM_SHIFT) | 13,
    ADC0_SE14 = (0 << TPM_SHIFT) | 14,
    ADC0_SE15 = (0 << TPM_SHIFT) | 15,
	ADC0_SE16 = (0 << TPM_SHIFT) | 16,
	ADC0_SE17 = (0 << TPM_SHIFT) | 17,
	ADC0_SE18 = (0 << TPM_SHIFT) | 18,
	ADC0_DM0  = (0 << TPM_SHIFT) | 19,
	ADC0_DM1  = (0 << TPM_SHIFT) | 20,
	ADC0_SE21 = (0 << TPM_SHIFT) | 21,
	ADC0_SE22 = (0 << TPM_SHIFT) | 22,
	ADC0_SE23_DAC0 = (0 << TPM_SHIFT) | 23,
	ADC0_TEMP = (0 << TPM_SHIFT) | 26,
	ADC0_BANDGAP = (0 << TPM_SHIFT) | 27,
	ADC0_VREFH  = (0 << TPM_SHIFT) | 29,
	ADC0_VREFL  = (0 << TPM_SHIFT) | 30,

	ADC1_DE0  = (1 << TPM_SHIFT) | 0,
	ADC1_DE1  = (1 << TPM_SHIFT) | 1,
	ADC1_DE3  = (1 << TPM_SHIFT) | 3,
	ADC1_SE4a = (1 << TPM_SHIFT) | 4,
	ADC1_SE5a = (1 << TPM_SHIFT) | 5,
	ADC1_SE6a = (1 << TPM_SHIFT) | 6,
	ADC1_SE7a = (1 << TPM_SHIFT) | 7,
    ADC1_SE4b = (1 << TPM_SHIFT) | 4,
    ADC1_SE5b = (1 << TPM_SHIFT) | 5,
    ADC1_SE6b = (1 << TPM_SHIFT) | 6,
    ADC1_SE7b = (1 << TPM_SHIFT) | 7,
	ADC1_SE8  = (1 << TPM_SHIFT) | 8,
	ADC1_SE9  = (1 << TPM_SHIFT) | 9,
	ADC1_SE10 = (1 << TPM_SHIFT) | 10,
	ADC1_SE11 = (1 << TPM_SHIFT) | 11,
	ADC1_SE12 = (1 << TPM_SHIFT) | 12,
	ADC1_SE13 = (1 << TPM_SHIFT) | 13,
	ADC1_SE14 = (1 << TPM_SHIFT) | 14,
	ADC1_SE15 = (1 << TPM_SHIFT) | 15,
	ADC1_SE16 = (1 << TPM_SHIFT) | 16,
	ADC1_SE17 = (1 << TPM_SHIFT) | 17,
	ADC1_SE18_VREF = (1 << TPM_SHIFT) | 18,
	ADC1_DM0  = (1 << TPM_SHIFT) | 19,
	ADC1_DM1  = (1 << TPM_SHIFT) | 20,
	ADC1_SE23_DAC1 = (1 << TPM_SHIFT) | 23,
	ADC1_TEMP = (1 << TPM_SHIFT) | 26,
	ADC1_BANDGAP = (1 << TPM_SHIFT) | 27,
	ADC1_VREFH  = (1 << TPM_SHIFT) | 29,
	ADC1_VREFL  = (1 << TPM_SHIFT) | 30,
} ADCName;

typedef enum {
    DAC_0 = 0,
	DAC_1 = 1
} DACName;

/*
typedef enum {
    SPI_0 = (int)SPI0_BASE,
} SPIName;*/

#ifdef __cplusplus
}
#endif

#endif
