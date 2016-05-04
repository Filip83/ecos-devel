//==========================================================================
//
//      i2c_uc3c.c
//
//      I2C driver for ATMEL AVR32UC3C processors
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 2008 Free Software Foundation, Inc.
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
// Author(s):    Filip
// Contributors:
// Date:         2012-11-15
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================


//==========================================================================
//                                 INCLUDES
//==========================================================================
#include <pkgconf/system.h>
#include <pkgconf/devs_i2c_avr32_uc3c.h>
#include <cyg/hal/gpio.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/i2c.h>
#include <cyg/io/i2c_uc3c.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>

#define I2C_FLAG_FINISH  1       // transfer finished
#define I2C_FLAG_ACT     2       // bus still active, no STOP condition send
#define I2C_FLAG_ERROR  (1<<31)  // one of the following errors occured:
#define I2C_FLAG_ADDR   (1<<30)  // - address was not ACKed
#define I2C_FLAG_DATA   (1<<29)  // - data was not ACKed
#define I2C_FLAG_LOST   (1<<28)  // - bus arbitration was lost
#define I2C_FLAG_BUF    (1<<27)  // - no buffer for reading or writing
#define I2C_FLAG_UNK    (1<<26)  // - unknown I2C status
#define I2C_FLAG_BUS    (1<<25)  // - bus error

#if CYGPKG_DEVS_I2C_AVR32_UC3C_DEBUG_LEVEL  > 0
   #define debug1_printf(args...) diag_printf(args)
#else
   #define debug1_printf(args...)
#endif
#if CYGPKG_DEVS_I2C_AVR32_UC3C_DEBUG_LEVEL  > 1
   #define debug2_printf(args...) diag_printf(args)
#else
   #define debug2_printf(args...)
#endif

static void avr32_i2c_set_speed(volatile avr32_twim_t *twi, 
        unsigned int speed, unsigned long pba_hz);
static cyg_uint32 avr32_i2c_isr(cyg_vector_t vec, cyg_addrword_t data);

static void avr32_i2c_set_speed(volatile avr32_twim_t *twi, 
        unsigned int speed, unsigned long pba_hz)
{
    cyg_uint32 f_prescaled;
    cyg_bool   done = false;
    cyg_uint8  cwgr_exp = 0;
    while(!done)
    {
        cwgr_exp = 0;
        f_prescaled = (pba_hz / speed / 2);
        // f_prescaled must fit in 8 bits, cwgr_exp must fit in 3 bits
        while ((f_prescaled > 0xFF) && (cwgr_exp <= 0x7)) 
        {
            // increase clock divider
            cwgr_exp++;
            // divide f_prescaled value
            f_prescaled /= 2;
        }
        if (cwgr_exp > 0x7)
        {
            speed /= 2;
            continue;
        }
        done = true;
        // set clock waveform generator register
        twi->cwgr = ((f_prescaled/2) << AVR32_TWIM_CWGR_LOW_OFFSET)
                    | ((f_prescaled - f_prescaled/2) << AVR32_TWIM_CWGR_HIGH_OFFSET)
                    | (cwgr_exp << AVR32_TWIM_CWGR_EXP_OFFSET)
                    | (0     << AVR32_TWIM_CWGR_DATA_OFFSET)
                    | (f_prescaled << AVR32_TWIM_CWGR_STASTO_OFFSET);
    }
}

//==========================================================================
// The ISR does the actual work. It is not that much work to justify
// putting it in the DSR, and it is also not clear whether this would
// even work.  If an error occurs we try to leave the bus in the same
// state as we would if there was no error.
//==========================================================================
static cyg_uint32 avr32_i2c_isr(cyg_vector_t vec, cyg_addrword_t data)
{
    cyg_avr32_i2c_extra* extra = (cyg_avr32_i2c_extra*)data;
    volatile avr32_twim_t *twi = extra->i2c_dev;
    cyg_uint32  status;

    status = twi->sr & twi->imr;
    if(status & AVR32_TWIM_SR_ARBLST_MASK)
    {
        twi->idr = ~0;
        twi->cr |= AVR32_TWIM_CR_SWRST_MASK;
        extra->i2c_flag = I2C_FLAG_ERROR | I2C_FLAG_LOST;
        return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
    }
    else if(status & AVR32_TWIM_SR_DNAK_MASK)
    {
        extra->i2c_flag = I2C_FLAG_ERROR | I2C_FLAG_DATA;
        twi->idr = ~0;
        twi->cr |= AVR32_TWIM_CR_SWRST_MASK;

        return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
    }
    else if(status & AVR32_TWIM_SR_ANAK_MASK)
    {
        extra->i2c_flag = I2C_FLAG_ERROR | I2C_FLAG_ADDR;
	twi->idr = ~0;
        twi->cr |= AVR32_TWIM_CR_SWRST_MASK;

        return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
    }
    else if((status & AVR32_TWIM_SR_CCOMP_MASK))
    {
        twi->idr =  AVR32_TWIM_IDR_CCOMP_MASK;
        extra->i2c_flag = I2C_FLAG_FINISH;
        return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;

    }
    else if((status & AVR32_TWIM_SR_RXRDY_MASK))
    {
        if(extra->i2c_rxbuf == NULL)
        {
            extra->i2c_flag = I2C_FLAG_ERROR | I2C_FLAG_BUF;
            twi->idr = ~0;
            return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
        }

        *(extra->i2c_rxbuf++)= twi->rhr;
        extra->i2c_count--;
        
        if(extra->i2c_count == 0)
        {
            twi->idr =  AVR32_TWIM_IDR_RXRDY_MASK;
            extra->i2c_flag = I2C_FLAG_FINISH;
            return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
        }
    }
    else if((status & AVR32_TWIM_SR_TXRDY_MASK))
    {
        if(extra->i2c_txbuf == NULL)
        {
           extra->i2c_flag = I2C_FLAG_ERROR | I2C_FLAG_BUF;
            twi->idr = AVR32_TWIM_IDR_TXRDY_MASK;
            return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
        }

        twi->thr = *(extra->i2c_txbuf++);

        extra->i2c_count--;

        if(extra->i2c_count == 0)
        {
            twi->idr =  AVR32_TWIM_IDR_TXRDY_MASK;
        }
    }

    //
    // We need to call the DSR only if there is really something to signal,
    // that means only if extra->i2c_flag != 0
    //
    if (extra->i2c_flag)
    {
        return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
    }
    else
    {
        return CYG_ISR_HANDLED;
    }
}


//==========================================================================
// DSR signals data
//==========================================================================
static void
avr32_i2c_dsr(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_avr32_i2c_extra* extra = (cyg_avr32_i2c_extra*)data;
    if(extra->i2c_flag)
    {
        cyg_drv_cond_signal(&extra->i2c_wait);
    }
}


//==========================================================================
// Initialize driver & hardware state
//==========================================================================
void cyg_avr32_i2c_init(struct cyg_i2c_bus *bus)
{
    cyg_avr32_i2c_extra* extra = (cyg_avr32_i2c_extra*)bus->i2c_extra;
    volatile avr32_twim_t *twi = extra->i2c_dev;

    cyg_drv_mutex_init(&extra->i2c_lock);
    cyg_drv_cond_init(&extra->i2c_wait, &extra->i2c_lock);
    cyg_drv_interrupt_create(extra->i2c_isrvec,
                             extra->i2c_isrpri,
                             (cyg_addrword_t) extra,
                             &avr32_i2c_isr,
                             &avr32_i2c_dsr,
                             &(extra->i2c_interrupt_handle),
                             &(extra->i2c_interrupt_data));
    cyg_drv_interrupt_attach(extra->i2c_interrupt_handle);


    twi->idr = ~0UL;
    twi->sr;
    // Reset TWI
    twi->cr = AVR32_TWIM_CR_SWRST_MASK;

    // Select the speed
    avr32_i2c_set_speed(twi, extra->i2c_bus_freq, extra->i2c_pclk);

    if(twi == AVR32_TWIM0_ADDRESS)
    {
    gpio_enable_module_pin(AVR32_TWIMS0_TWD_PIN,  AVR32_TWIMS0_TWD_FUNCTION);
    gpio_enable_module_pin(AVR32_TWIMS0_TWCK_PIN, AVR32_TWIMS0_TWCK_FUNCTION);
    }
    
    if(twi == AVR32_TWIM1_ADDRESS)
    {
    gpio_enable_module_pin(AVR32_TWIMS0_TWD_PIN,  AVR32_TWIMS0_TWD_FUNCTION);
    gpio_enable_module_pin(AVR32_TWIMS0_TWCK_PIN, AVR32_TWIMS0_TWCK_FUNCTION);
    }

}

//==========================================================================
// transmit a buffer to a device
//==========================================================================
cyg_uint32 cyg_avr32_i2c_tx(const cyg_i2c_device *dev,
                              cyg_bool              send_start,
                              const cyg_uint8      *tx_data,
                              cyg_uint32            count,
                              cyg_bool              send_stop)
{
    cyg_avr32_i2c_extra* extra =
                           (cyg_avr32_i2c_extra*)dev->i2c_bus->i2c_extra;
    volatile avr32_twim_t *twi = extra->i2c_dev;
    cyg_bool tenth_bit = false;
    extra->i2c_addr  = dev->i2c_address;
    extra->i2c_count = count;
    extra->i2c_txbuf = tx_data;

    twi->cr = AVR32_TWIM_CR_MEN_MASK;
    twi->cr = AVR32_TWIM_CR_SWRST_MASK;
    twi->cr = AVR32_TWIM_CR_MDIS_MASK;

    if(extra->i2c_addr >= 128)
        tenth_bit = true;

    twi->cmdr = ((count << AVR32_TWIM_CMDR_NBYTES)                &
                AVR32_TWIM_CMDR_NBYTES_MASK)                      |
                AVR32_TWIM_CMDR_VALID_MASK                        |
                ((send_stop << AVR32_TWIM_CMDR_STOP_OFFSET)       &
                AVR32_TWIM_CMDR_STOP_MASK)                        |
                ((send_start << AVR32_TWIM_CMDR_START_OFFSET)     &
                AVR32_TWIM_CMDR_START_MASK)                       |
                ((tenth_bit  << AVR32_TWIM_CMDR_TENBIT_OFFSET)    &
                AVR32_TWIM_CMDR_TENBIT_MASK)                      |
		((extra->i2c_addr << AVR32_TWIM_CMDR_SADR_OFFSET) &
                AVR32_TWIM_CMDR_SADR_MASK) ;

    extra->i2c_flag  = 0;

    //
    // the isr will do most of the work, and the dsr will signal when an
    // error occured or the transfer finished
    //
    cyg_drv_mutex_lock(&extra->i2c_lock);
    cyg_drv_dsr_lock();
    cyg_drv_interrupt_unmask(extra->i2c_isrvec);

    twi->ier = AVR32_TWIM_IER_ANAK_MASK  | AVR32_TWIM_IER_ARBLST_MASK |
               AVR32_TWIM_IER_DNAK_MASK  | AVR32_TWIM_IER_TXRDY_MASK  |
               AVR32_TWIM_IER_CCOMP_MASK;

    // Enable master transfer, disable slave
    twi->cr =   AVR32_TWIM_CR_MEN_MASK;
    while(!(extra->i2c_flag & (I2C_FLAG_FINISH | I2C_FLAG_ERROR)))
    {
        cyg_drv_cond_wait(&extra->i2c_wait);
    }
    twi->idr = ~0;

    twi->cr =   AVR32_TWIM_CR_MDIS_MASK;
    //cyg_drv_interrupt_mask(extra->i2c_isrvec);
    cyg_drv_dsr_unlock();
    cyg_drv_mutex_unlock(&extra->i2c_lock);

    // too bad we have no way to tell the caller
    if(extra->i2c_flag & I2C_FLAG_ERROR)
    {
        debug1_printf("I2C TX error flag: %x\n", extra->i2c_flag);
        extra->i2c_flag = 0;
    }

    count -= extra->i2c_count;

    extra->i2c_addr  = 0;
    extra->i2c_count = 0;
    extra->i2c_txbuf = NULL;

    return count;
}


//==========================================================================
// receive into a buffer from a device
//==========================================================================
cyg_uint32 cyg_avr32_i2c_rx(const cyg_i2c_device *dev,
                              cyg_bool              send_start,
                              cyg_uint8            *rx_data,
                              cyg_uint32            count,
                              cyg_bool              send_nak,
                              cyg_bool              send_stop)
{
    cyg_bool tenth_bit = false;
    cyg_avr32_i2c_extra* extra =
                           (cyg_avr32_i2c_extra*)dev->i2c_bus->i2c_extra;
    avr32_twim_t *twi = extra->i2c_dev;
    extra->i2c_addr  = dev->i2c_address;
    extra->i2c_count = count;
    extra->i2c_rxbuf = rx_data;
    extra->i2c_rxnak = send_nak;

    twi->cr = AVR32_TWIM_CR_MEN_MASK;
    twi->cr = AVR32_TWIM_CR_SWRST_MASK;
    twi->cr = AVR32_TWIM_CR_MDIS_MASK;

    if(extra->i2c_addr >= 128)
    {
        tenth_bit = true;
        twi->cmdr = ((extra->i2c_addr << AVR32_TWIM_CMDR_SADR_OFFSET))
              & (AVR32_TWIM_CMDR_SADR_MASK
              | (0 << AVR32_TWIM_CMDR_NBYTES_OFFSET)
              | (AVR32_TWIM_CMDR_VALID_MASK)
              | (AVR32_TWIM_CMDR_START_MASK)
              | (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
              | (AVR32_TWIM_CMDR_TENBIT_MASK)
              | (0 << AVR32_TWIM_CMDR_READ_OFFSET));


        twi->ncmdr = ((count << AVR32_TWIM_CMDR_NBYTES)           &
                AVR32_TWIM_CMDR_NBYTES_MASK)                      |
                AVR32_TWIM_CMDR_VALID_MASK                        |
                ((send_stop << AVR32_TWIM_CMDR_STOP_OFFSET)       &
                AVR32_TWIM_CMDR_STOP_MASK)                        |
                ((send_start << AVR32_TWIM_CMDR_START_OFFSET)     &
                AVR32_TWIM_CMDR_START_MASK)                       |
                ((tenth_bit  << AVR32_TWIM_CMDR_TENBIT_OFFSET)    &
                AVR32_TWIM_CMDR_TENBIT_MASK)                      |
		((extra->i2c_addr << AVR32_TWIM_CMDR_SADR_OFFSET) &
                AVR32_TWIM_CMDR_SADR_MASK)                        |
                AVR32_TWIM_CMDR_READ_MASK;
    }
    else
    {
        twi->cmdr = ((count << AVR32_TWIM_CMDR_NBYTES)            &
                AVR32_TWIM_CMDR_NBYTES_MASK)                      |
                AVR32_TWIM_CMDR_VALID_MASK                        |
                ((send_stop << AVR32_TWIM_CMDR_STOP_OFFSET)       &
                AVR32_TWIM_CMDR_STOP_MASK)                        |
                ((send_start << AVR32_TWIM_CMDR_START_OFFSET)     &
                AVR32_TWIM_CMDR_START_MASK)                       |
                ((tenth_bit  << AVR32_TWIM_CMDR_TENBIT_OFFSET)    &
                AVR32_TWIM_CMDR_TENBIT_MASK)                      |
		((extra->i2c_addr << AVR32_TWIM_CMDR_SADR_OFFSET) &
                AVR32_TWIM_CMDR_SADR_MASK)                        |
                AVR32_TWIM_CMDR_READ_MASK;
    }

    extra->i2c_flag  = 0;

    //
    // the isr will do most of the work, and the dsr will signal when an
    // error occurred or the transfer finished
    //
    cyg_drv_mutex_lock(&extra->i2c_lock);
    cyg_drv_dsr_lock();
    //cyg_drv_interrupt_unmask(extra->i2c_isrvec);

    twi->ier = AVR32_TWIM_IER_ANAK_MASK  | AVR32_TWIM_IER_ARBLST_MASK |
               AVR32_TWIM_IER_DNAK_MASK  | AVR32_TWIM_IER_RXRDY_MASK;

    // Enable master transfer, disable slave
    twi->cr =   AVR32_TWIM_CR_MEN_MASK;
    while(!(extra->i2c_flag & (I2C_FLAG_FINISH | I2C_FLAG_ERROR)))
    {
        cyg_drv_cond_wait(&extra->i2c_wait);
    }
    twi->idr = ~0;

    twi->cr =   AVR32_TWIM_CR_MDIS_MASK;
    //cyg_drv_interrupt_mask(extra->i2c_isrvec);
    cyg_drv_dsr_unlock();
    cyg_drv_mutex_unlock(&extra->i2c_lock);

    // too bad we have no way to tell the caller
    if (extra->i2c_flag & I2C_FLAG_ERROR)
    {
        diag_printf("I2C RX error flag: %x\n", extra->i2c_flag);
        extra->i2c_flag = 0;
    }

    count -= extra->i2c_count;

    extra->i2c_addr  = 0;
    extra->i2c_count = 0;
    extra->i2c_rxbuf = NULL;

    return count;
}


//==========================================================================
//  generate a STOP
//==========================================================================
void cyg_avr32_i2c_stop(const cyg_i2c_device *dev)
{
    cyg_avr32_i2c_extra* extra =
                           (cyg_avr32_i2c_extra*)dev->i2c_bus->i2c_extra;
    volatile avr32_twim_t *twi = extra->i2c_dev;

    twi->scr |= (1 << AVR32_TWIM_CMDR_STOP);
    twi->cr  |= (1 << AVR32_TWIM_CMDR_STOP);

    extra->i2c_flag  = 0;
    extra->i2c_count = 0;
}

//---------------------------------------------------------------------------
// eof i2c_avr32.c

