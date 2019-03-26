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
#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <cyg/infra/diag.h>
#include <cyg/hal/plf_io.h>
#include <cyg/hal/var_io.h>
#include <cyg/io/gpio/gpio_api.h>
#include <cyg/io/gpio/pinmap.h>
#include <cyg/io/gpio/PinNames.h>

void pin_function(PinName pin, int function) {
	CYG_ASSERTC(pin != (PinName)NC);

    uint32_t port_n = (uint32_t)pin >> PORT_SHIFT;
    uint32_t pin_n  = (uint32_t)(pin & 0x7C) >> 2;

    cyghwr_hal_kinetis_sim_t *SIM = CYGHWR_HAL_KINETIS_SIM_P;
    SIM->scgc5 |= 1 << (CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTA_M + port_n);
    volatile uint32_t* pin_pcr = &(((cyghwr_hal_kinetis_port_t *)
    		(((char*)CYGHWR_HAL_KINETIS_PORTA_P) + 0x1000 * port_n)))->pcr[pin_n];

    // pin mux bits: [10:8] -> 11100000000 = (0x700)
    *pin_pcr = (*pin_pcr & ~0x700) | (function << 8);
}

void pin_mode(PinName pin, PinMode mode) {
	CYG_ASSERTC(pin != (PinName)NC);
    volatile uint32_t* pin_pcr = (volatile uint32_t*)(((char*)CYGHWR_HAL_KINETIS_PORTA_P) + pin);

    // pin pullup bits: [1:0] -> 11 = (0x3)
    *pin_pcr = (*pin_pcr & ~0x3) | mode;
}


uint32_t gpio_set(PinName pin) {
	CYG_ASSERTC(pin != (PinName)NC);
    pin_function(pin, 1);
    return 1 << ((pin & 0x7F) >> 2);
}

void gpio_init(gpio_t *obj, PinName pin) {
    obj->pin = pin;
    if (pin == (PinName)NC)
        return;

    obj->mask = gpio_set(pin);

    unsigned int port = (unsigned int)pin >> PORT_SHIFT;

    cyghwr_hal_kinetis_gpio_t *reg = (cyghwr_hal_kinetis_gpio_t *)(((char*)CYGHWR_HAL_KINETIS_GPIO_PORTA_P) + port * 0x40);
    obj->reg_set = &reg->psor;
    obj->reg_clr = &reg->pcor;
    obj->reg_in  = &reg->pdir;
    obj->reg_dir = &reg->pddr;
}

void gpio_mode(gpio_t *obj, PinMode mode) {
    pin_mode(obj->pin, mode);
}

void gpio_dir(gpio_t *obj, PinDirection direction) {
	CYG_ASSERTC(obj->pin != (PinName)NC);
    switch (direction) {
        case PIN_INPUT :
            *obj->reg_dir &= ~obj->mask;
            break;
        case PIN_OUTPUT:
            *obj->reg_dir |=  obj->mask;
            break;
    }
}
