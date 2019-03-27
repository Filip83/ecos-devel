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
#include <cyg/io/gpio/analogin_api.h>
#include <cyg/hal/plf_io.h>
#include <cyg/hal/var_io.h>
#include <cyg/hal/var_io_adc.h>

//#include "cmsis.h"
#include <cyg/io/gpio/pinmap.h>
//#include "clk_freqs.h"
#include <cyg/io/gpio/PeripheralPins.h>

#define MAX_FADC        6000000

#if DEVICE_ANALOGIN

void analogin_init(analogin_t *obj, PinName pin) {
    obj->adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
     (obj->adc != (ADCName)NC);

    cyghwr_hal_kinetis_sim_t *SIM = CYGHWR_HAL_KINETIS_SIM_P;

    uint32_t port = (uint32_t)pin >> PORT_SHIFT;
    SIM->scgc5 |= 1 << (CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTA_S + port);

    cyghwr_hal_kinteis_adc_t *ADC;
    if((obj->adc >> TPM_SHIFT) == 0)
    {
    	ADC = ((cyghwr_hal_kinteis_adc_t *)ADC0_BASE);
    	SIM->scgc6 |= CYGHWR_HAL_KINETIS_SIM_SCGC6_ADC0_M;
    }
    else
    {
    	ADC = ((cyghwr_hal_kinteis_adc_t *)ADC1_BASE);
		SIM->scgc3 |= CYGHWR_HAL_KINETIS_SIM_SCGC3_ADC1_M;
    }

    obj->ADC = ADC;


    // bus clk
    uint32_t PCLK = hal_get_peripheral_clock();//bus_frequency();
    uint32_t clkdiv;
    for (clkdiv = 0; clkdiv < 4; clkdiv++) {
        if ((PCLK >> clkdiv) <= MAX_FADC)
            break;
    }
    if (clkdiv == 4)                    //Set max div
        clkdiv = 0x7;

    ADC->SC1[1] = ADC_SC1_ADCH(obj->adc);

    ADC->CFG1 = ADC_CFG1_ADLPC_MASK    // Low-Power Configuration
               | ADC_CFG1_ADIV(clkdiv & 0x3)       // Clock Divide Select
               | ADC_CFG1_ADLSMP_MASK   // Long Sample Time
               | ADC_CFG1_MODE(3)       // (16)bits Resolution
               | ADC_CFG1_ADICLK(clkdiv >> 2);    // Input Clock

    ADC->CFG2 = ADC_CFG2_MUXSEL_MASK   // ADxxb or ADxxa channels
               | ADC_CFG2_ADHSC_MASK    // High-Speed Configuration
               | ADC_CFG2_ADLSTS(0);    // Long Sample Time Select

    ADC->SC2 = ADC_SC2_REFSEL(0);      // Default Voltage Reference

    ADC->SC3 = ADC_SC3_AVGE_MASK       // Hardware Average Enable
              | ADC_SC3_AVGS(0);        // 4 Samples Averaged

    pinmap_pinout(pin, PinMap_ADC);
}

uint16_t analogin_read_u16(analogin_t *obj) {
    // start conversion
	cyghwr_hal_kinteis_adc_t *ADC = (cyghwr_hal_kinteis_adc_t*)obj->ADC;
    ADC->SC1[0] = ADC_SC1_ADCH(obj->adc);

    // Wait Conversion Complete
    while ((ADC->SC1[0] & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);

    return (uint16_t)ADC->R[0];
}

float analogin_read(analogin_t *obj) {
    uint16_t value = analogin_read_u16(obj);
    return (float)value * (1.0f / (float)0xFFFF);
}

const PinMap *analogin_pinmap()
{
    return PinMap_ADC;
}

#endif