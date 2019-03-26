
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
 
#include <cyg/io/gpio/PeripheralPins.h>

/************ADC***************/
const PinMap PinMap_ADC[] = {
    {PTE0,  ADC1_SE4a, 0},
    {PTE1,  ADC1_SE5a, 0},
    {PTE2,  ADC1_SE6a, 0},
    {PTE3,  ADC1_SE7a, 0},
    {PTE24, ADC0_SE17, 0},
    {PTE25, ADC0_SE18, 0},
    {PTA7,  ADC0_SE10, 0},
    {PTA8,  ADC0_SE11, 0},
    {PTA17, ADC1_SE17, 0},
    {PTB0,  ADC0_SE8,  0},
	{PTB1,  ADC0_SE9,  0},
	{PTB2,  ADC0_SE12, 0},
	{PTB3,  ADC0_SE13, 0},
	{PTB4,  ADC1_SE10, 0},
	{PTB5,  ADC1_SE11, 0},
	{PTB6,  ADC1_SE12, 0},
	{PTB7,  ADC1_SE13, 0},
	{PTB10, ADC1_SE14, 0},
	{PTB11, ADC1_SE15, 0},
    {PTC0,  ADC0_SE14, 0},
	{PTC1,  ADC0_SE15, 0},
	{PTC2,  ADC0_SE4b, 0},
	{PTC8,  ADC1_SE4b, 0},
	{PTC9,  ADC1_SE5b, 0},
	{PTC10, ADC1_SE6b, 0},
	{PTC11, ADC1_SE7b, 0},
	{PTD1,  ADC0_SE5b, 0},
	{PTD5,  ADC0_SE6b, 0},
	{PTD6,  ADC0_SE7b, 0},
    {NC,    NC,        0}
};

/************DAC***************/
const PinMap PinMap_DAC[] = {
    {DAC0_OUT, DAC_0, 0},
	{DAC1_OUT, DAC_1, 0},
    {NC      , NC   , 0}
};

#if 0
/************I2C***************/
const PinMap PinMap_I2C_SDA[] = {
    {PTB1,  I2C_0, 2},
    {PTB3,  I2C_0, 2},
    {PTE0,  I2C_1, 2},
    {PTC11, I2C_1, 2},    
    {NC  ,  NC   , 0}
};

const PinMap PinMap_I2C_SCL[] = {
    {PTB0,  I2C_0, 2},
    {PTB2,  I2C_0, 2},
    {PTE1,  I2C_1, 2},
    {PTC10, I2C_1, 2},
    {NC  ,  NC,    0}
};

/************UART***************/
const PinMap PinMap_UART_TX[] = {
    {PTB17, UART_0, 3},
    {PTC4 , UART_1, 3},
    {PTD3 , UART_2, 3},
    {PTD7 , UART_0, 3},
    {PTE0 , UART_1, 3},
    {NC   , NC    , 0}
};

const PinMap PinMap_UART_RX[] = {
    {PTB16, UART_0, 3},
    {PTC3 , UART_1, 3},
    {PTD2 , UART_2, 3},
    {PTD6 , UART_0, 3},
    {PTE1 , UART_1, 3},
    {NC   , NC    , 0}
};

/************SPI***************/
const PinMap PinMap_SPI_SCLK[] = { // SCK
    {PTC5, SPI_0, 2},
    {PTD1, SPI_0, 2},
    {NC  , NC   , 0}
};

const PinMap PinMap_SPI_MOSI[] = { // DOUT
    {PTD2, SPI_0, 2},
    {PTC6, SPI_0, 2},
    {NC  , NC   , 0}
};

const PinMap PinMap_SPI_MISO[] = { // DIN
    {PTD3, SPI_0, 2},
    {PTC7, SPI_0, 2},
    {NC  , NC   , 0}
};

const PinMap PinMap_SPI_SSEL[] = { // CS
    {PTD0, SPI_0, 2},
    {PTC4, SPI_0, 2},
    {PTD4, SPI_0, 2},
    {PTC3, SPI_0, 2},
    {PTC2, SPI_0, 2},
    {PTD5, SPI_0, 2},
    {PTD6, SPI_0, 2},
    {PTC1, SPI_0, 2},
    {PTC0, SPI_0, 2}
};
#endif

/************PWM***************/
const PinMap PinMap_PWM[] = {
    // LEDs
	{PTB12,  PWM_9,  3}, // PTB12, FTM1 CH0
	{PTB13,  PWM_10, 3}, // PTB13, FTM1 CH1
	{PTB12,  PWM_5,  4}, // PTB12, FTM0 CH4
	{PTB13,  PWM_6,  4}, // PTB13, FTM0 CH5
	{PTE5,  PWM_13, 6}, // PTE5, FTM3 CH0
	{PTE6,  PWM_14, 6}, // PTE6, FTM3 CH1
	{PTE7,  PWM_15, 6}, // PTE7, FTM3 CH2
	{PTE8,  PWM_16, 6}, // PTE8, FTM3 CH3
	{PTE9,  PWM_17, 6}, // PTE9, FTM3 CH4
	{PTE10, PWM_18, 6}, // PTE10, FTM3 CH5
	{PTE11, PWM_19, 6}, // PTE11, FTM3 CH6
	{PTE12, PWM_20, 6}, // PTE12, FTM3 CH7
	{PTA0,  PWM_6 , 3}, // PTA0, FTM0 CH5
	{PTA1,  PWM_7 , 3}, // PTA1, FTM0 CH6
	{PTA2,  PWM_8 , 3}, // PTA2, FTM0 CH7
	{PTA3,  PWM_1 , 3}, // PTA3, FTM0 CH0
	{PTA4,  PWM_2 , 3}, // PTA4, FTM0 CH1
	{PTA5,  PWM_3 , 3}, // PTA5, FTM0 CH2
	{PTA6,  PWM_4 , 3}, // PTA6, FTM0 CH3
	{PTA7,  PWM_5 , 3}, // PTA7, FTM0 CH4
	{PTA8,  PWM_9 , 3}, // PTA8, FTM1 CH0
	{PTA9,  PWM_10, 3}, // PTA9, FTM1 CH1
	{PTA10, PWM_11 ,3}, // PTA10, FTM2 CH0
	{PTA11, PWM_12, 3}, // PTA11, FTM2 CH1
	{PTA12, PWM_9 , 3}, // PTA12, FTM1 CH0
	{PTA13, PWM_10, 3}, // PTA13, FTM1 CH1
	{PTB0,  PWM_9 , 3}, // PTB0, FTM1 CH0
	{PTB1,  PWM_10,  3}, // PTB1, FTM1 CH1
	{PTB18, PWM_11 ,3}, // PTB18, FTM2 CH0
	{PTB19, PWM_12, 3}, // PTB19, FTM2 CH1
	{PTC1,  PWM_1 , 4}, // PTC1, FTM0 CH0
	{PTC2,  PWM_2 , 4}, // PTC2, FTM0 CH1
	{PTC3,  PWM_3 , 4}, // PTC3, FTM0 CH2
	{PTC4,  PWM_4 , 4}, // PTC4, FTM0 CH3
	{PTC5,  PWM_3 , 7}, // PTC5, FTM0 CH2
	{PTC8,  PWM_17, 3}, // PTC8, FTM3 CH4
	{PTC9,  PWM_18, 3}, // PTC9, FTM3 CH5
	{PTC10, PWM_19, 3}, // PTC10, FTM3 CH6
	{PTC11, PWM_20, 3}, // PTC11, FTM3 CH7
	{PTD0,  PWM_13, 4}, // PTD0, FTM3 CH0
	{PTD1,  PWM_14, 4}, // PTD1, FTM3 CH1
	{PTD2,  PWM_15, 4}, // PTD2, FTM3 CH2
	{PTD3,  PWM_16, 4}, // PTD3, FTM3 CH3
	{PTD4,  PWM_5 , 4}, // PTD4, FTM0 CH4
	{PTD5,  PWM_6 , 4}, // PTD5, FTM0 CH5
	{PTD6,  PWM_7 , 4}, // PTD6, FTM0 CH6
	{PTD7,  PWM_8 , 4}, // PTD7, FTM0 CH7
    {NC , NC    , 0}
};
