#ifndef CYGONCE_I2C_AVR32_UC3C_H
#define CYGONCE_I2C_AVR32_UC3C_H
//==========================================================================
//
//      devs/i2c/avr32/uc3c/current/src/i2c_uc3c.h
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
// Author(s):     Filip Adamec
// Contributors:
// Date:          2012-11-15
// Description:   I2C driver for AVR32UC3C
//####DESCRIPTIONEND####
//==========================================================================


//==========================================================================
//                               INCLUDES
//==========================================================================
#include <cyg/hal/avr32/io.h>
#include <pkgconf/devs_i2c_avr32_uc3c.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>




//==========================================================================
// Single I2C bus sepecififc data
//==========================================================================
typedef struct cyg_avr32_i2c_extra {
    // Put statically initialized fields first.
    avr32_twim_t     *i2c_dev;    // Per-bus h/w details
    cyg_vector_t     i2c_isrvec;
    cyg_uint32       i2c_isrpri;
    cyg_uint32       i2c_pclk;    // peripheral clock of
    cyg_uint32       i2c_bus_freq;// I2C bus frequency (e.g. 100 kHz, 400 kHz)

    cyg_uint32       i2c_addr;
    cyg_uint32       i2c_count;
    const cyg_uint8* i2c_txbuf;
    cyg_uint8*       i2c_rxbuf;
    cyg_bool         i2c_rxnak;
    
    cyg_uint32       i2c_naddr;
    cyg_uint32       i2c_ncount;
    const cyg_uint8* i2c_ntxbuf;
    cyg_uint8*       i2c_nrxbuf;
    cyg_bool         i2c_nrxnak;
    
    cyg_uint32       i2c_ier;
    
    cyg_uint32       i2c_flag;
    cyg_bool         i2c_chained;
    cyg_bool         i2c_first;


    cyg_drv_mutex_t  i2c_lock; // For synchronizing between DSR and foreground
    cyg_drv_cond_t   i2c_wait;
    cyg_handle_t     i2c_interrupt_handle;// For initializing the interrupt
    cyg_interrupt    i2c_interrupt_data;
} cyg_avr32_i2c_extra;


//==========================================================================
// I2C driver interface
//==========================================================================
externC void        cyg_avr32_i2c_init(struct cyg_i2c_bus*);
externC cyg_uint32  cyg_avr32_i2c_tx(const cyg_i2c_device*,
                                       cyg_bool, const cyg_uint8*,
                                       cyg_uint32, cyg_bool);
externC cyg_uint32  cyg_avr32_i2c_rx(const cyg_i2c_device*,
                                       cyg_bool, cyg_uint8*,
                                       cyg_uint32, cyg_bool, cyg_bool);
externC void        cyg_avr32_i2c_stop(const cyg_i2c_device*);

//==========================================================================
// I2C bus declaration macros
//=========================================================================

# define CYG_AVR32_I2C_BUS(_name_, _init_fn_, _base_, _isr_vec_, _isr_pri_, \
                             _pclk_, _i2c_bus_freq_)                    \
  static cyg_avr32_i2c_extra _name_ ## _extra = {                       \
  i2c_dev      : _base_,                                                \
  i2c_isrvec   : _isr_vec_,                                             \
  i2c_isrpri   : _isr_pri_,                                             \
  i2c_pclk     : _pclk_,                                                \
  i2c_bus_freq : _i2c_bus_freq_,                                        \
  i2c_count    :  0,                                                    \
  i2c_txbuf    :  NULL,                                                 \
  i2c_rxbuf    :  NULL,                                                 \
  i2c_ncount   :  0,                                                    \
  i2c_ntxbuf   :  NULL,                                                 \
  i2c_nrxbuf   :  NULL,                                                 \
  i2c_flag     :  0,                                                    \
  i2c_chained  :  0,                                                    \
  i2c_first    :  0                                                     \
  } ;                                                                   \
  CYG_I2C_BUS(_name_,                                                   \
              _init_fn_,                                                \
              &cyg_avr32_i2c_tx,                                        \
              &cyg_avr32_i2c_rx,                                        \
              &cyg_avr32_i2c_stop,                                      \
              (void*) & ( _name_ ## _extra)) ;



//-----------------------------------------------------------------------------
#endif // #endif CYGONCE_I2C_AVR32_UC3C_H
