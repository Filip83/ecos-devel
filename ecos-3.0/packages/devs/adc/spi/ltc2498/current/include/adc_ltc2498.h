#ifndef CYGONCE_DEVS_LTC2498_H
#define CYGONCE_DEVS_LTC2498_H
//==========================================================================
//
//      adc_ltc2498.h
//
//      Freescale ADC definitions.
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
// Purpose:     Freescale FTM and ADC definitions.
// Description:
//
//
//####DESCRIPTIONEND####
//==========================================================================

typedef enum ltc2498_mode {
    LTC2498_MODE_DIFF  = 0x00,     // Differential mode
    LTC2498_MODE_SGL   = 0x01      // Single ended mode
} ltc2498_mode;

typedef enum ltc2498_opposite {
    LTC2498_ODD_NORM   = 0x00,     // Positive or even
    LTC2498_ODD_ALT    = 0x01      // Negative or odd
} ltc2498_opposite;

typedef enum ltc2498_select {
    LTC2498_SELECT_0   = 0x00,     // Chan 0
    LTC2498_SELECT_1   = 0x01,     // Chan 1
    LTC2498_SELECT_2   = 0x02,     // Chan 2
    LTC2498_SELECT_3   = 0x03,     // Chan 3
    LTC2498_SELECT_4   = 0x04,     // Chan 4
    LTC2498_SELECT_5   = 0x05,     // Chan 5
    LTC2498_SELECT_6   = 0x06,     // Chan 6
    LTC2498_SELECT_7   = 0x07      // Chan 7
} ltc2498_select;

typedef enum ltc2498_polarity {
    LTC2498_POL_BI     = 0x00,
    LTC2498_POL_UNI    = 0x01
} ltc2498_polarity;

typedef enum ltc2498_gain {
    LTC2498_GAIN_LOW   = 0x01,
    LTC2498_GAIN_HI    = 0x00
} ltc2498_gain;

typedef struct ltc2498_settings {
    ltc2498_mode        mode;
    ltc2498_opposite    opposite;
    ltc2498_select      select;
} ltc2498_settings;

//-----------------------------------------------------------------------------
// LTC2498 ADC device setup

typedef struct ltc2498_adc_setup {
    CYG_ADDRESS         adc_base;           // Programmable Delay Block base address
    cyg_vector_t        tmr_int_vector;     // TMR interrupt vector
    cyg_priority_t      tmr_int_pri;        // TMR interrupt priority
    ltc2498_settings    *channel_settings;  // ADC settings by channel
} ltc2498_adc_setup;

//-----------------------------------------------------------------------------
// LTC2498 ADC device

typedef struct ltc2498_adc_info {
    const ltc2498_adc_setup   *setup;                 // ADC setup
    cyg_handle_t              tmr_int_handle;         // TMR interrupt handle
    cyg_interrupt             tmr_int_data;           // TMR interrupt data
    cyg_adc_channel           *chan[16];              // Channel references by channel no
    cyg_adc_channel           *chan_active[16];       // Channels to be processed
    cyg_uint32                chan_mask;              // Channel mask
    cyg_uint32                chan_pos;               // Channel position
    cyg_uint32                mask_reset;             // True if the enable/disable changed
    cyg_uint32                num_active;             // Number of active channels
    cyg_spi_device            *spi_dev;               // SPI device
} ltc2498_adc_info;

extern bool ltc2498_adc_init(struct cyg_devtab_entry *tab);
extern Cyg_ErrNo ltc2498_adc_lookup(struct cyg_devtab_entry **tab,
                                  struct cyg_devtab_entry *sub_tab,
                                  const char *name);
extern void ltc2498_adc_enable(cyg_adc_channel *chan);
extern void ltc2498_adc_disable(cyg_adc_channel *chan);
extern void ltc2498_adc_set_rate(cyg_adc_channel *chan, cyg_uint32 rate);
extern void ltc2498_adc_set_gain(cyg_adc_channel *chan, cyg_uint32 gain);
extern void ltc2498_adc_set_polarity(cyg_adc_channel *chan, cyg_uint32 polarity);
extern void ltc2498_adc_set_mode(cyg_adc_channel *chan, cyg_uint32 mode);


#endif // CYGONCE_DEVS_LTC2498_H
