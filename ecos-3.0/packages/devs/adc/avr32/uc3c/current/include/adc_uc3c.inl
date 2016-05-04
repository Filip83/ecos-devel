//==========================================================================
//
//      adc_uc3c.inl
//
//      ADC driver for AVR32UC3C on chip ADC
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
//
// Contributors:
// Date:         2012-11-17
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

#ifndef CYGONCE_DEVS_ADC_AVR32_UC3C_INL
#define CYGONCE_DEVS_ADC_AVR32_UC3C_INL

// Declare ADC0
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0

static avr32_adc_info avr32_adc0_info =
{
    .adc_base         = AVR32_ADCIFA_ADDRESS,
    .adc_vector       = CYGNUM_HAL_VECTOR_ADCIF_SEQ0,
    .adc_intprio      = CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_INTPRIO,
    .int_handle       = 0,
    .adc_prescal      = CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_PRESCAL,
    .adc_startup_time = CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_STARTUP_TIME,
#if CYGNUM_IO_ADC_SAMPLE_SIZE > 10
     .resolution      = 0,
#elif CYGNUM_IO_ADC_SAMPLE_SIZE > 8
     .resolution      = 1,
#else
     .resolution      = 2,
#endif
    .num_active_ch    = 0,
    .chan_mask        = 0
};

CYG_ADC_DEVICE( avr32_adc0_device,
                &avr32_adc_funs,
                &avr32_adc0_info,
                CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_DEFAULT_RATE);

#define AVR32_ADC0_CHANNEL( __chan )                                        \
CYG_ADC_CHANNEL( avr32_adc0_channel##__chan,                                \
                 __chan,                                                    \
                 CYGDAT_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL##__chan##_BUFSIZE, \
                 &avr32_adc0_device );                                      \
                                                                            \
DEVTAB_ENTRY( avr32_adc0_channel##__chan##_device,                          \
              CYGDAT_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL##__chan##_NAME,       \
              0,                                                            \
              &cyg_io_adc_devio,                                            \
              avr32_adc_init,                                               \
              avr32_adc_lookup,                                             \
              &avr32_adc0_channel##__chan );

#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL0
AVR32_ADC0_CHANNEL(0);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL1
AVR32_ADC0_CHANNEL(1);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL2
AVR32_ADC0_CHANNEL(2);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL3
AVR32_ADC0_CHANNEL(3);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL4
AVR32_ADC0_CHANNEL(4);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL5
AVR32_ADC0_CHANNEL(5);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL6
AVR32_ADC0_CHANNEL(6);
#endif
#ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL7
AVR32_ADC0_CHANNEL(7);
#endif

#endif // CYGPKG_DEVS_ADC_ARM_AVR32_UC3C

#endif // CYGONCE_DEVS_ADC_AVR32_UC3C_INL
