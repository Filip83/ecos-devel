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
#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

//#include "cmsis.h"
#include <cyg/io/gpio/device.h>
#include <cyg/io/gpio/PortNames.h>
#include <cyg/io/gpio/PeripheralNames.h>
#include <cyg/io/gpio/PinNames.h>
#include <stdint.h>
/*#include <cyg/hal/plf_io.h>
#include <cyg/hal/var_io_adc.h>*/

#ifdef __cplusplus
extern "C" {
#endif

struct gpio_irq_s {
    uint32_t port;
    uint32_t pin;
    uint32_t ch;
};

struct port_s {
    volatile uint32_t *reg_dir;
    volatile uint32_t *reg_out;
    volatile  uint32_t *reg_in;
    PortName port;
    uint32_t mask;
};

struct pwmout_s {
	volatile uint32_t *MOD;
	volatile uint32_t *SYNC;
	volatile uint32_t *CnV;
};

struct analogin_s {
	void *ADC;
    ADCName adc;
};

//#if DEVICE_ANALOGOUT
struct dac_s {
    DACName dac;
    volatile uint8_t *DATH;
    volatile uint8_t *DATL;
};
//#endif

/*
struct serial_s {
    UART_Type *uart;
    int index;
};





struct i2c_s {
    I2C_Type *i2c;
};

struct spi_s {
    SPI_Type *spi;
};*/

#include "gpio_object.h"

#ifdef __cplusplus
}
#endif

#endif
