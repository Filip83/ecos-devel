//==========================================================================
//
//      beep_kinetis.c
//
//      Beeper driver for the Kinetis 
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later
// version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License
// along with eCos; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// As a special exception, if other files instantiate templates or use
// macros or inline functions from this file, or you compile this file
// and link it with other works to produce a work based on this file,
// this file does not by itself cause the resulting work to be covered by
// the GNU General Public License. However the source code for this file
// must still be made available in accordance with section (3) of the GNU
// General Public License v2.
//
// This exception does not invalidate any other reasons why a work based
// on this file might be covered by the GNU General Public License.
// -------------------------------------------
// ####ECOSGPLCOPYRIGHTEND####
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Filip <filip.gnu@gmail.com>
// Contributors:
// Date:         2016-11-28
// Purpose:
// Description:  Beeper driver for Kinetis
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>
#include <string.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_cache.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/beep.h>
#include <cyg/io/beeper.h>
#include <cyg/hal/hal_endian.h>

#include <cyg/io/devtab.h>

#include <pkgconf/devs_beep_kinetis.h>

/*******************************************************************************
 * The beeper use FTM timer in compare mode. Where output is toogle on compare *
 * event. The comparret event is set in the midle of timer count interval.     *
 * The timer counts half the configured period and is reset. The duration of   *
 * beping is genertated in compare interupt by counting appropriate number of  *
 * those interrupt. If specified number of those interrupts occure the beeping *
 * id disabled. 
 */

/** Declaration and initialization of beeper data structure.
*
*/
static cyg_beep_kinetis_t cyg_beep_kinetis =
{
#if CYGNUM_DEVS_BEEP_TIMER == 0
    .timer_base             = CYGADDR_IO_FTM_FREESCALE_FTM0_BASE,
#elif CYGNUM_DEVS_BEEP_TIMER == 1
    .timer_base             = CYGADDR_IO_FTM_FREESCALE_FTM1_BASE,
#else
    .timer_base             = CYGADDR_IO_FTM_FREESCALE_FTM2_BASE,
#endif
    .init		    = false,
    .beeping                = false,
    .beep_interrupt         = NULL,
    .beep_interrupt_handle  = NULL,
    #if CYGNUM_DEVS_BEEP_TIMER == 0
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_FTM0,
    #elif CYGNUM_DEVS_BEEP_TIMER == 1
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_FTM1,
    #else
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_FTM2,
    #endif
    .interrupt_prio         = CYGNUM_DEVS_BEEP_KINETIS_ISR_PRI,
};



static cyg_uint32
kinetis_beep_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void       
kinetis_beep_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);


void cyg_beep_nb(int beep_interval_ms)
{
	cyg_beep_kinetis_t * priv = &cyg_beep_kinetis;
	cyg_addrword_t base = priv->timer_base;
	cyg_uint32 regval;

	if (beep_interval_ms <= 0)
	{
		priv->cnt = 0;

		priv->cnt_beep_count = priv->time_to_cnt*
			CYGDAT_DEVS_BEEP_DEFAULT_TIME;

		// read status register
		HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC +
			CYGNUM_DEVS_BEEP_CHANNEL, regval);
		// Enable channel interrupt
		regval |= CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE;
		// and clear interrupt status flgag
		regval &= ~CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF;
		HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC +
			CYGNUM_DEVS_BEEP_CHANNEL, regval);

		// Reset timer count value
		regval = 0;
		HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_CNT, regval);
		// set overflow value
		regval = priv->timer_cmp_value;
		HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_MOD, regval);

		// Clock source system clock divided by 1 slo by i vice
		regval = (1 << 3) | 0;
		HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);

		// set beeping state
		priv->beeping = true;
		// and unmask interrupt in interrupt conteroller
		cyg_drv_interrupt_unmask(priv->interrupt_number);
		cyg_drv_interrupt_acknowledge(priv->interrupt_number);

	}
	else
	{
		priv->cnt = 0;

		priv->cnt_beep_count = priv->time_to_cnt*beep_interval_ms;

		// read status register
		HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC +
			CYGNUM_DEVS_BEEP_CHANNEL, regval);
		// Enable channel interrupt
		regval |= CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE;
		// and clear interrupt status flgag
		regval &= ~CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF;
		HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC +
			CYGNUM_DEVS_BEEP_CHANNEL, regval);

		// Reset timer count value
		regval = 0;
		HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_CNT, regval);
		// set overflow value
		regval = priv->timer_cmp_value;
		HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_MOD, regval);

		// Clock source system clock divided by 1 slo by i vice
		regval = (1 << 3) | 0;
		HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);

		// set beeping state
		priv->beeping = true;
		// and unmask interrupt in interrupt conteroller
		cyg_drv_interrupt_unmask(priv->interrupt_number);
		cyg_drv_interrupt_acknowledge(priv->interrupt_number);
	}
}

void cyg_beep(int beep_interval_ms)
{
	cyg_beep_nb(beep_interval_ms);
	cyg_beep_wait();
}


int  cyg_beep_wait()
{
	cyg_drv_mutex_lock(&cyg_beep_kinetis.lock);
	while(cyg_beep_kinetis.beeping)
		cyg_drv_cond_wait(&cyg_beep_kinetis.wait);
	cyg_drv_mutex_unlock(&cyg_beep_kinetis.lock);
	return cyg_beep_kinetis.beeping;
}

int  cyg_beep_is_beeping()
{
	return cyg_beep_kinetis.beeping;
}

void beeper_init()
{
    cyg_uint32 regval;
    cyg_beep_kinetis_t *beep_dev = &cyg_beep_kinetis;
    cyg_addrword_t base = beep_dev->timer_base;
    // Init beepr timer interrupt 
    cyg_drv_interrupt_create(beep_dev->interrupt_number,
                             beep_dev->interrupt_prio,   // Priority
                             (cyg_addrword_t)beep_dev,   // Data item passed to interrupt handler
                             kinetis_beep_ISR,
                             kinetis_beep_DSR,
                             &beep_dev->beep_interrupt_handle,
                             &beep_dev->beep_interrupt);
    cyg_drv_interrupt_attach(beep_dev->beep_interrupt_handle);

	cyg_drv_mutex_init(&beep_dev->lock);
	cyg_drv_cond_init(&beep_dev->wait, &beep_dev->lock);

    // Enable clocks
#if CYGNUM_DEVS_BEEP_TIMER == 0
    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_FTM0);
#elif CYGNUM_DEVS_BEEP_TIMER == 1
    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_FTM1);
#else
    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_FTM2);
#endif
    
    // Configure pin mux
    hal_set_pin_function(CYGHWR_IO_FREESCALE_BEEP_PIN);
    
    // Disable FTM write protection
    regval = CYGHWR_DEV_FREESCALE_FTM_MODE_WPDIS;
    HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_MODE, regval);
    
    // Set timer initial value
    regval =  0;
    HAL_WRITE_UINT16(base + CYGHWR_DEV_FREESCALE_FTM_CNTIN, regval);
    
    // Enable toogle on compare event
    regval = CYGHWR_DEV_FREESCALE_FTM_CNSC_MSA | 
             CYGHWR_DEV_FREESCALE_FTM_CNSC_ELSA;
    
    HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_C0SC + 
            CYGNUM_DEVS_BEEP_CHANNEL, regval);
    
    beep_dev->timer_cmp_value = 
            (hal_get_peripheral_clock())/(2*CYGDAT_DEVS_BEEP_DEFAULT_FREQUENCY);
    
    beep_dev->time_to_cnt = (2*CYGDAT_DEVS_BEEP_DEFAULT_FREQUENCY*1e-3);
    
    regval = beep_dev->timer_cmp_value >> 2;
    
    // Set comparre value to toggle output
    HAL_WRITE_UINT16(base + CYGHWR_DEV_FREESCALE_FTM_COV +
            CYGNUM_DEVS_BEEP_CHANNEL, regval);
    
    beep_dev->init = true;
        
    return true;
}

cyg_uint32 kinetis_beep_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
	cyg_uint32 ret = CYG_ISR_HANDLED;
    cyg_uint32 regval;
    cyg_beep_kinetis_t * priv = (cyg_beep_kinetis_t*)data;
    cyg_addrword_t base = priv->timer_base;
    
    HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval);
    
    // Chek if channel interrupt
    if(regval & CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF)
    {
        if(priv->cnt == priv->cnt_beep_count)
        {
            // Disable channel interrupt and clear interrupt status falg
            regval &= ~CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE;
            regval &= ~CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF;
            HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC +
                    CYGNUM_DEVS_BEEP_CHANNEL, regval);
            
            // disable clock
            regval = 0;
            HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);
            
            priv->beeping = false;
            cyg_drv_interrupt_mask(priv->interrupt_number);
			ret |= CYG_ISR_CALL_DSR;
        }
        else
        {
            priv->cnt++;
            // Clear interrupt falg
            regval &= ~CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF;
            HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC +
                        CYGNUM_DEVS_BEEP_CHANNEL, regval);
        } 
    }
    else
    {
        CYG_FAIL("Beeper unexpected interrupt.\n");
    }
	
    cyg_drv_interrupt_acknowledge(priv->interrupt_number);
    return ret;
}

void kinetis_beep_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	cyg_beep_kinetis_t * priv = (cyg_beep_kinetis_t*)data;
	cyg_drv_mutex_lock(&priv->lock);
	cyg_drv_cond_signal(&priv->wait);
	cyg_drv_mutex_unlock(&priv->lock);

}

