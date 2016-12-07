//==========================================================================
//
//      beep_avr32.c
//
//      Beeper driver for the AVR32 
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
// Description:  Beeper driver for AVR32
//
//####DESCRIPTIONEND####
//
//==========================================================================


#include <pkgconf/devs_beep_avr32.h>

#include <cyg/kernel/kapi.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/beep_avr32.h>
#include <cyg/hal/gpio.h>
#include <cyg/hal/avr32/io.h>
#include <cyg/io/devtab.h>
#include CYGBLD_HAL_BOARD_H


/** Declaration and initialization of button keyborad data structure.
*
*/
static cyg_beep_avr32_t cyg_beep_avr32 =
{
#if CYGNUM_DEVS_BEEP_TIMER == 0
    .timer_base             = (avr32_tc_t*)AVR32_TC0_ADDRESS,
#else
    .timer_base             = AVR32_TC1,
#endif
    .init		    = false,
    .beeping                = false,
    .beep_interrupt         = NULL,
    .beep_interrupt_handle  = NULL,
#if CYGNUM_DEVS_BEEP_TIMER == 0
    #if CYGNUM_DEVS_BEEP_CHANNEL == 0
    .interrupt_number       = CYGNUM_HAL_VECTOR_TC00,
    #elif CYGNUM_DEVS_BEEP_CHANNEL == 1
    .interrupt_number       = CYGNUM_HAL_VECTOR_TC01,
    #else
    .interrupt_number       = CYGNUM_HAL_VECTOR_TC02,
    #endif
#else
    #if CYGNUM_DEVS_BEEP_CHANNEL == 0
    .interrupt_number       = CYGNUM_HAL_VECTOR_TC10,
    #elif CYGNUM_DEVS_BEEP_CHANNEL == 1
    .interrupt_number       = CYGNUM_HAL_VECTOR_TC11,
    #else
    .interrupt_number       = CYGNUM_HAL_VECTOR_TC12,
    #endif
#endif
    .interrupt_prio         = CYGNUM_DEVS_BEEP_INTERRUPT_PRIO,
};

// Functions in this module



static Cyg_ErrNo beep_set_config(cyg_io_handle_t handle,
                                cyg_uint32 key,
                                const void *buffer,
                                cyg_uint32 *len);
static Cyg_ErrNo beep_get_config(cyg_io_handle_t handle,
                                cyg_uint32 key,
                                void *buffer,
                                cyg_uint32 *len);
static bool      beep_init(struct cyg_devtab_entry *tab);
static Cyg_ErrNo beep_lookup(struct cyg_devtab_entry **tab,
                            struct cyg_devtab_entry *st,
                            const char *name);


DEVIO_TABLE(beep_handlers,
                 NULL,                            // Unsupported write() function
                 NULL,
                 NULL,
                 beep_get_config,
                 beep_set_config);

DEVTAB_ENTRY(beep_device,
                  CYGDAT_DEVS_BEEP_NAME,
                  NULL,                           // Base device name
                  &beep_handlers,
                  beep_init,
                  beep_lookup,
                  &cyg_beep_avr32);                // Private data pointer


static cyg_uint32
avr32_beep_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void       
avr32_beep_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);




static Cyg_ErrNo
beep_set_config(cyg_io_handle_t handle,
               cyg_uint32 key,
               const void *buffer,
               cyg_uint32 *len)
{
    cyg_beep_avr32_t * priv = (cyg_beep_avr32_t*)handle;
    cyg_uint32 ret = ENOERR;
    
    switch(key)
    {
        case CYG_BEEP_IOCTL_SET_BEEP:
        {
            if(*len == 0)
            {
                priv->cnt = 0;
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].rc = 
                        priv->timer_cmp_value;
                
                priv->cnt_beep_count = priv->time_to_cnt*
                        CYGDAT_DEVS_BEEP_DEFAULT_TIME;
                
                // read status register to clear status falgs
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].sr; 
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].ier = 
                        AVR32_TC_IER0_CPCS_MASK;
    
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].ccr 
                        = 4 | 1;
                priv->beeping = true;
                
                
            }
            else if(*len == sizeof(cyg_uint32))
            {
                cyg_uint32 *time = (cyg_uint32*)buffer;
                
                priv->cnt = 0;
                
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].rc = 
                        priv->timer_cmp_value;
                
                priv->cnt_beep_count = priv->time_to_cnt*(*time);
                
                // read status register to clear status falgs
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].sr; 
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].ier = 
                        AVR32_TC_IER0_CPCS_MASK;
                
                priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].ccr 
                        = 4 | 1;
                priv->beeping = true;
            }
            else
                ret = EINVAL;
        }
        break;
        default:
            ret = EINVAL;
    }

    return ret;
}

static Cyg_ErrNo
beep_get_config(cyg_io_handle_t handle,
               cyg_uint32 key,
               void *buffer,
               cyg_uint32 *len)
{
    cyg_beep_avr32_t * priv = (cyg_beep_avr32_t*)handle;
    switch(key)
    {
        case CYG_BEEP_IOCTL_GET_BEEPING:
            if(*len == sizeof(cyg_uint32))
            {
                *((cyg_uint32*)buffer) = priv->beeping;
                return ENOERR;
            }
            break;
    }
    return EINVAL;
}

static bool
beep_init(struct cyg_devtab_entry *tab)
{
    cyg_beep_avr32_t *beep_dev = &cyg_beep_avr32;

    // Init beepr timer interrupt 
    cyg_drv_interrupt_create(beep_dev->interrupt_number,
                             beep_dev->interrupt_prio,   // Priority
                             (cyg_addrword_t)beep_dev,   // Data item passed to interrupt handler
                             avr32_beep_ISR,
                             avr32_beep_DSR,
                             &beep_dev->beep_interrupt_handle,
                             &beep_dev->beep_interrupt);
    cyg_drv_interrupt_attach(beep_dev->beep_interrupt_handle);

    beep_dev->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].cmr =        
                                          AVR32_TC_CMR0_WAVE_MASK   |
                                    (3 << AVR32_TC_CMR0_ACPC_OFFSET)   |
                                    (2 << AVR32_TC_CMR0_WAVSEL_OFFSET) |
                                    (1 << AVR32_TC_CMR0_EEVT_OFFSET)   |
                                    (1 << AVR32_TC_CMR0_TCCLKS_OFFSET);
    
    
    beep_dev->timer_cmp_value = 
            (1e6*CYGHWR_HAL_AVR32_CPU_FREQ)/CYGDAT_DEVS_BEEP_DEFAULT_FREQUENCY;
    
    beep_dev->time_to_cnt = (CYGDAT_DEVS_BEEP_DEFAULT_FREQUENCY*1e-3);
    
    gpio_enable_module_pin(CYG_HAL_AVR32_BEEP_PIN, CYG_HAL_AVR32_BEEP_FUNCTION);
        
    return true;
}

static Cyg_ErrNo
beep_lookup(struct cyg_devtab_entry **tab,
           struct cyg_devtab_entry *st,
           const char *name)
{
    cyg_beep_avr32_t * const priv = (cyg_beep_avr32_t *) (*tab)->priv;
    
    if(!priv->init)
    {
        beep_init(*tab);
        priv->init = true;
    }
    return ENOERR;
}

cyg_uint32 avr32_beep_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_beep_avr32_t * priv = (cyg_beep_avr32_t*)data;
    if(priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].sr & 
            AVR32_TC_SR0_CPCS_MASK)
    {
        if(priv->cnt == priv->cnt_beep_count)
        {
            priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].idr = 
                    AVR32_TC_SR0_CPCS_MASK;
            
            // disable clock
            priv->timer_base->channel[CYGNUM_DEVS_BEEP_CHANNEL].ccr = 2;
            
            priv->beeping = false;
        }
        else
            priv->cnt++;
    }
    else
    {
        CYG_FAIL("Beeper unexpected interrupt.\n");
    }
	
    return CYG_ISR_HANDLED;
}

void avr32_beep_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	
}

