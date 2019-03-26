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
#include <cyg/io/gpio/analogout_api.h>

#if DEVICE_ANALOGOUT

#include <cyg/hal/plf_io.h>
#include <cyg/hal/var_io.h>
#include <cyg/hal/var_io_dac.h>
//#include "cmsis.h"
#include <cyg/io/gpio/pinmap.h>
#include <cyg/io/gpio/PeripheralPins.h>

#define RANGE_12BIT     0xFFF

void analogout_init(dac_t *obj, PinName pin) {
    obj->dac = (DACName)pinmap_peripheral(pin, PinMap_DAC);
    CYG_ASSERTC(obj->dac != (DACName)NC);

    int dac_n = obj->dac;
    cyghwr_hal_kinetis_sim_t *SIM = CYGHWR_HAL_KINETIS_SIM_P;
    SIM->scgc2 |= CYGHWR_HAL_KINETIS_SIM_SCGC2_DAC0_M;

    uint32_t port = (uint32_t)pin >> PORT_SHIFT;
    SIM->scgc5 |= 1 << (CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTA_S + port);

    cyghwr_hal_kinteis_dac_t *DAC = (cyghwr_hal_kinteis_dac_t*)(DAC0_BASE + 0x1000*dac_n);

    DAC->DAT[obj->dac].DATH = 0;
    DAC->DAT[obj->dac].DATL = 0;
    obj->DATH = &DAC->DAT[obj->dac].DATH;
    obj->DATL = &DAC->DAT[obj->dac].DATL;

    DAC->C1 = DAC_C1_DACBFMD_MASK;     // One-Time Scan Mode

    DAC->C0 = DAC_C0_DACEN_MASK      // Enable
             | DAC_C0_DACSWTRG_MASK   // Software Trigger
             | DAC_C0_DACRFS_MASK;    // VDDA selected

    analogout_write_u16(obj, 0);
}

void analogout_free(dac_t *obj) {}

static inline void dac_write(dac_t *obj, int value) {
	obj->DATL = (uint8_t)( value       & 0xFF);
    obj->DATH = (uint8_t)((value >> 8) & 0xFF);
}

static inline int dac_read(dac_t *obj) {
    return ((*obj->DATH << 8) | *obj->DATL);
}

void analogout_write(dac_t *obj, float value) {
    if (value < 0.0) {
        dac_write(obj, 0);
    } else if (value > 1.0) {
        dac_write(obj, RANGE_12BIT);
    } else {
        dac_write(obj, value * (float)RANGE_12BIT);
    }
}

void analogout_write_u16(dac_t *obj, uint16_t value) {
    dac_write(obj, value >> 4); // 12-bit
}

float analogout_read(dac_t *obj) {
    uint32_t value = dac_read(obj);
    return (float)value * (1.0f / (float)RANGE_12BIT);
}

uint16_t analogout_read_u16(dac_t *obj) {
    uint32_t value = dac_read(obj); // 12-bit
    return (value << 4) | ((value >> 8) & 0x003F);
}

const PinMap *analogout_pinmap()
{
    return PinMap_DAC;
}

#endif
