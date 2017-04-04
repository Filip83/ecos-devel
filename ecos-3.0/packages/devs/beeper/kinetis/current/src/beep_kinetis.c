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
static cyg_beep_kinetis_t cyg_beep_kinetis =
{
#if CYGNUM_DEVS_BEEP_CHANNEL == 0
    .timer_base             = CYGADDR_IO_FTM_FREESCALE_FTM0_BASE,
#elif CYGNUM_DEVS_BEEP_CHANNEL == 1
    .timer_base             = CYGADDR_IO_FTM_FREESCALE_FTM0_BASE,
#else
    .timer_base             = CYGADDR_IO_FTM_FREESCALE_FTM2_BASE,
#endif
    .init		    = false,
    .beeping                = false,
    .beep_interrupt         = NULL,
    .beep_interrupt_handle  = NULL,
    #if CYGNUM_DEVS_BEEP_CHANNEL == 0
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_FTM0,
    #elif CYGNUM_DEVS_BEEP_CHANNEL == 1
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_FTM1,
    #else
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_FTM2,
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
kinetis_beep_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void       
kinetis_beep_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static Cyg_ErrNo
beep_set_config(cyg_io_handle_t handle,
               cyg_uint32 key,
               const void *buffer,
               cyg_uint32 *len)
{
    cyg_uint32 regval;
    cyg_devtab_entry_t *hnd = (cyg_devtab_entry_t*)handle;
    cyg_beep_kinetis_t * priv = (cyg_beep_kinetis_t*)hnd->priv;
    cyg_addrword_t base = priv->timer_base;
    cyg_uint32 ret = ENOERR;
    
    switch(key)
    {
        case CYG_BEEP_IOCTL_SET_BEEP:
        {
            if(*len == -1)
            {
                priv->cnt = 0;
                regval = beep_dev->timer_cmp_value;
    
                HAL_WRITE_UINT16(base + CYGHWR_DEV_FREESCALE_FTM_COV, regval);
                
                priv->cnt_beep_count = priv->time_to_cnt*
                        CYGDAT_DEVS_BEEP_DEFAULT_TIME;
                
                // read status register to clear status falgs
                HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval); 
                regval |= CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE;
                HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval); 
                
                // Clock source system clock divided by 1 slo by i vice
                regval = (1 << 3) | 0;
                HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);
                priv->beeping = true;
                cyg_drv_interrupt_unmask(priv->interrupt_number);
                
            }
            else if(*len == sizeof(cyg_uint32))
            {
                cyg_uint32 *time = (cyg_uint32*)buffer;
                
                priv->cnt = 0;
                
                regval = beep_dev->timer_cmp_value;
    
                HAL_WRITE_UINT16(base + CYGHWR_DEV_FREESCALE_FTM_COV, regval);
                
                priv->cnt_beep_count = priv->time_to_cnt*(*time);
                
                // read status register to clear status falgs
                HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval); 
                regval |= CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE;
                HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval); 
                
                // Clock source system clock divided by 1 slo by i vice
                regval = (1 << 3) | 0;
                HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);
                priv->beeping = true;
                cyg_drv_interrupt_unmask(priv->interrupt_number);
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
    cyg_devtab_entry_t *hnd = (cyg_devtab_entry_t*)handle;
    cyg_beep_kinetis_t * priv = (cyg_beep_kinetis_t*)hnd->priv;
    switch(key)
    {
        case CYG_BEEP_IOCTL_GET_BEEPING:
            if(*len == sizeof(cyg_uint32))
            {
                *((cyg_uint32*)buffer) = (cyg_uint32)priv->beeping;
                return ENOERR;
            }
            break;
    }
    return EINVAL;
}

static bool
beep_init(struct cyg_devtab_entry *tab)
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

    // Enable clocks
#if CYGNUM_DEVS_BEEP_CHANNEL == 0
    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_FTM0);
#elif CYGNUM_DEVS_BEEP_CHANNEL == 1
    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_FTM1);
#else
    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_FTM2);
#endif
    
    // Configure pin mux
    CYGHWR_IO_FREESCALE_UART_PIN(CYGHWR_IO_FREESCALE_FTM1_PIN_CH0);
    
    regval = CYGHWR_DEV_FREESCALE_FTM_MODE_WPDIS;
    HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_MODE, regval);
    
    // Clock source system clock divided by 1 slo by i vice
    regval = (1 << 3) | 0;
    HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);
    
    // Enable toogle on compare event
    regval = CYGHWR_DEV_FREESCALE_FTM_CNSC_MSA | 
             CYGHWR_DEV_FREESCALE_FTM_CNSC_ELSA;
    
    HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval);
    
    // Maxk output event 
    regval = CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH0OM;
    
    HAL_WRITE_UINT8(base + CYGHWR_DEV_FREESCALE_FTM_OUTMASK, regval);
    
    // Souce clock is PBA/2
    // On compare pin value is toogle
    beep_dev->timer_cmp_value = 
            (hal_get_peripheral_clock())/(CYGDAT_DEVS_BEEP_DEFAULT_FREQUENCY);
    
    beep_dev->time_to_cnt = (2*CYGDAT_DEVS_BEEP_DEFAULT_FREQUENCY*1e-3);
    
    regval = beep_dev->timer_cmp_value;
    
    HAL_WRITE_UINT16(base + CYGHWR_DEV_FREESCALE_FTM_COV, regval);
    
    beep_dev->init = true;
        
    return true;
}

static Cyg_ErrNo
beep_lookup(struct cyg_devtab_entry **tab,
           struct cyg_devtab_entry *st,
           const char *name)
{
    cyg_beep_kinetis_t * const priv = (cyg_beep_kinetis_t *) (*tab)->priv;
    
    if(!priv->init)
    {
        beep_init(*tab);
    }
    return ENOERR;
}

cyg_uint32 kinetis_beep_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_uint32 regval;
    cyg_addrword_t base = beep_dev->timer_base;
    cyg_beep_kinetis_t * priv = (cyg_beep_kinetis_t*)data;
    
    HAL_READ_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval);
    
    if(regval & CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF)
    {
        if(priv->cnt == priv->cnt_beep_count)
        {
            // Disable channel interrupt
            regval &= ~CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE;
            HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_C0SC, regval);
            
            // disable clock
            regval = 0;
            HAL_WRITE_UINT32(base + CYGHWR_DEV_FREESCALE_FTM_SC, regval);
            
            priv->beeping = false;
            cyg_drv_interrupt_mask(priv->interrupt_number);
        }
        else
            priv->cnt++;
    }
    else
    {
        CYG_FAIL("Beeper unexpected interrupt.\n");
    }
	
    
    cyg_drv_interrupt_acknowledge(priv->interrupt_number);
    return CYG_ISR_HANDLED;
}

void kinetis_beep_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	
}

