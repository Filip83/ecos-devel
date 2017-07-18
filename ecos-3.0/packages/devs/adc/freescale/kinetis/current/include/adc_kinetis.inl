//==========================================================================
//
//      adc_kinetis.inl
//
//      ADC driver for Freescale Kinetis on chip ADC
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
// Date:         2017-03-31
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

#ifndef CYGONCE_DEVS_ADC_KINETIS_INL
#define CYGONCE_DEVS_ADC_KINETIS_INL

// Declare ADC
#ifdef CYGPKG_DEVS_ADC_KINETIS_ADC

static kinetis_adc_info kinetis_adc0_info =
{
    .adc_base         = CYGADR_DEVS_ADC_BASE,
    .adc_vector       = CYGNUM_HAL_INTERRUPT_ADC0,
    .adc_intprio      = CYGNUM_DEVS_ADC_KINETIS_ISR_PRI,
    .int_handle       = 0,
    .num_active_ch    = 0,
    .chan_mask        = 0
};

CYG_ADC_DEVICE( kinetis_adc_device,
                &kinetis_adc_funs,
                &kinetis_adc0_info,
                CYGNUM_DEVS_ADC_KINETIS_DEFAULT_RATE);

#define KINETIS_ADC_CHANNEL( __chan )                                     \
CYG_ADC_CHANNEL( kinetis_adc_channel##__chan,                             \
                 __chan,                                                  \
                 CYGDAT_DEVS_ADC_KINETIS_CHANNEL##__chan##_BUFSIZE,       \
                 &kinetis_adc_device );                                   \
                                                                          \
DEVTAB_ENTRY( kinetis_adc_channel##__chan##_device,                       \
              CYGDAT_DEVS_ADC_KINETIS_CHANNEL##__chan##_NAME,             \
              0,                                                          \
              &cyg_io_adc_devio,                                          \
              kinetis_adc_init,                                           \
              kinetis_adc_lookup,                                         \
              &kinetis_adc_channel##__chan );

#ifdef CYGPKG_DEVS_ADC_KINETIS_CHANNEL0
KINETIS_ADC_CHANNEL(0);
#endif


#endif // CYGPKG_DEVS_ADC_KINETIS_ADC

#endif // CYGONCE_DEVS_ADC_KINETIS_INL
