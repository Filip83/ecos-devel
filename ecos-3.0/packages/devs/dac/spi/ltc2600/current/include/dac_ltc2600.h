#ifndef CYGONCE_DEVS_LTC2600_H
#define CYGONCE_DEVS_LTC2600_H
//==========================================================================
//
//      dac_ltc2600.h
//
//      Freescale DAC definitions.
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2011, 2013 Free Software Foundation, Inc.                        
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
// Author(s):   Mike Jones <mike@proclivis.com>
// Contributors:
// Date:        2013-06-14
// Purpose:     Freescale FTM and DAC definitions.
// Description:
//
//
//####DESCRIPTIONEND####
//==========================================================================

typedef enum ltc2600_select {
    LTC2600_SELECT_0   = 0x00,     // Chan 0
    LTC2600_SELECT_1   = 0x01,     // Chan 1
    LTC2600_SELECT_2   = 0x02,     // Chan 2
    LTC2600_SELECT_3   = 0x03,     // Chan 3
    LTC2600_SELECT_4   = 0x04,     // Chan 4
    LTC2600_SELECT_5   = 0x05,     // Chan 5
    LTC2600_SELECT_6   = 0x06,     // Chan 6
    LTC2600_SELECT_7   = 0x07,     // Chan 7
    LTC2600_SELECT_ALL = 0x0F      // All channels
} ltc2600_select;

typedef struct ltc2600_settings {
    ltc2600_select      select;
} ltc2600_settings;

//-----------------------------------------------------------------------------
// LTC2600 DAC device setup

typedef struct ltc2600_dac_setup {
    CYG_ADDRESS         dac_base;           // Programmable timer base address
    cyg_vector_t        tmr_int_vector;     // TMR interrupt vector
    cyg_priority_t      tmr_int_pri;        // TMR interrupt priority
    ltc2600_settings    *channel_settings;  // DAC settings by channel
} ltc2600_dac_setup;

//-----------------------------------------------------------------------------
// LTC2600 DAC device

typedef struct ltc2600_dac_info {
    const ltc2600_dac_setup   *setup;         // DAC setup
    cyg_handle_t              tmr_int_handle; // TMR interrupt handle
    cyg_interrupt             tmr_int_data;   // TMR interrupt data
    cyg_dac_channel           *chan[8];       // Channel references by channel no
    cyg_uint32                chan_mask;      // Channel mask
    cyg_spi_device            *spi_dev;       // SPI device
} ltc2600_dac_info;

extern bool ltc2600_dac_init(struct cyg_devtab_entry *tab);
extern Cyg_ErrNo ltc2600_dac_lookup(struct cyg_devtab_entry **tab,
                                  struct cyg_devtab_entry *sub_tab,
                                  const char *name);
extern void ltc2600_dac_enable(cyg_dac_channel *chan);
extern void ltc2600_dac_disable(cyg_dac_channel *chan);
extern void ltc2600_dac_set_rate(cyg_dac_channel *chan, cyg_uint32 rate);
extern void ltc2600_dac_set_gain(cyg_dac_channel *chan, cyg_uint32 gain);
extern void ltc2600_dac_set_polarity(cyg_dac_channel *chan, cyg_uint32 polarity);

#endif // CYGONCE_DEVS_LTC2600_H
