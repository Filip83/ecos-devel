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
#include <cyg/io/gpio/pwmout_api.h>
#include <cyg/io/gpio/PeripheralNames.h>
#include <cyg/io/gpio/pinmap.h>
#include <cyg/io/gpio/PeripheralPins.h>

static float pwm_clock = 0;
externC cyg_uint32 hal_kinetis_busclk;

void pwmout_init(pwmout_t* obj, PinName pin) {
    // determine the channel
    PWMName pwm = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
    CYG_ASSERTC(pwm != (PWMName)NC);

    //uint32_t MGCOUTClock = SystemCoreClock * (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT));
    uint32_t BusClock = hal_get_peripheral_clock();//MGCOUTClock / (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT));

    uint32_t clkdiv = 0;
    float clkval = BusClock / 1000000.0f;

    while (clkval > 1) {
        clkdiv++;
        clkval /= 2.0;
        if (clkdiv == 7)
            break;
    }

    pwm_clock = clkval;
    unsigned int ftm_n = (pwm >> TPM_SHIFT);
    unsigned int ch_n = (pwm & 0xFF);

    cyghwr_hal_kinetis_sim_t *SIM = CYGHWR_HAL_KINETIS_SIM_P;
    SIM->scgc6 |= 1 << (CYGHWR_HAL_KINETIS_SIM_SCGC6_FTM0_M + ftm_n);

    cyghwr_hal_kinteis_ftm_t *ftm = (cyghwr_hal_kinteis_ftm_t *)(CYGADDR_IO_FTM_FREESCALE_FTM0_BASE + 0x1000 * ftm_n);
    ftm->CONF |= FTM_CONF_BDMMODE(3);
    ftm->SC = FTM_SC_CLKS(1) | FTM_SC_PS(clkdiv); // (clock)MHz / clkdiv ~= (0.75)MHz
    ftm->CONTROLS[ch_n].CnSC = (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK); /* No Interrupts; High True pulses on Edge Aligned PWM */
    ftm->MODE = FTM_MODE_FTMEN_MASK;
    ftm->SYNC = FTM_SYNC_CNTMIN_MASK;
    ftm->SYNCONF = FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_SWSOC_MASK | FTM_SYNCONF_SWWRBUF_MASK;
    
    //Without SYNCEN set CnV does not seem to update
    ftm->COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_SYNCEN2_MASK | FTM_COMBINE_SYNCEN3_MASK;

    obj->CnV = (uint32_t*)&ftm->CONTROLS[ch_n].CnV;
    obj->MOD = (uint32_t*)&ftm->MOD;
    obj->SYNC = (uint32_t*)&ftm->SYNC;

    // default to 20ms: standard for servos, and fine for e.g. brightness control
    pwmout_period_ms(obj, 20);
    pwmout_write(obj, 0.0);
    
    // Wire pinout
    pinmap_pinout(pin, PinMap_PWM);
}

void pwmout_free(pwmout_t* obj) {}

void pwmout_write(pwmout_t* obj, float value) {
    if (value < 0.0) {
        value = 0.0;
    } else if (value > 1.0) {
        value = 1.0;
    }
    
    while(*obj->SYNC & FTM_SYNC_SWSYNC_MASK);
    *obj->CnV = (uint32_t)((float)(*obj->MOD + 1) * value);
    *obj->SYNC |= FTM_SYNC_SWSYNC_MASK;    
}

float pwmout_read(pwmout_t* obj) {
    while(*obj->SYNC & FTM_SYNC_SWSYNC_MASK);
    float v = (float)(*obj->CnV) / (float)(*obj->MOD + 1);
    return (v > 1.0) ? (1.0) : (v);
}

void pwmout_period(pwmout_t* obj, float seconds) {
    pwmout_period_us(obj, seconds * 1000000.0f);
}

void pwmout_period_ms(pwmout_t* obj, int ms) {
    pwmout_period_us(obj, ms * 1000);
}

// Set the PWM period, keeping the duty cycle the same.
void pwmout_period_us(pwmout_t* obj, int us) {
    float dc = pwmout_read(obj);
    *obj->MOD = (uint32_t)(pwm_clock * (float)us) - 1;
    *obj->SYNC |= FTM_SYNC_SWSYNC_MASK;
    pwmout_write(obj, dc);
}

void pwmout_pulsewidth(pwmout_t* obj, float seconds) {
    pwmout_pulsewidth_us(obj, seconds * 1000000.0f);
}

void pwmout_pulsewidth_ms(pwmout_t* obj, int ms) {
    pwmout_pulsewidth_us(obj, ms * 1000);
}

void pwmout_pulsewidth_us(pwmout_t* obj, int us) {
    *obj->CnV = (uint32_t)(pwm_clock * (float)us);
    *obj->SYNC |= FTM_SYNC_SWSYNC_MASK;
}

const PinMap *pwmout_pinmap(void)
{
    return PinMap_PWM;
}
