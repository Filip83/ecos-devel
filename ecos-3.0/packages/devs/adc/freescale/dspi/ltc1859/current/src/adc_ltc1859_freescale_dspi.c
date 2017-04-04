//==========================================================================
//
//      adc_ltc1859_freescale_dspi.c
//
//      LTC1859 ADC device over Freescale DSPI
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
// Author(s):      Mike Jones
// Date:           2013-05-18
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================


#include <pkgconf/system.h>
#if defined(CYGPKG_IO_SPI) && defined(CYGPKG_DEVS_ADC_SPI_LTC1859)
// ------------------------------------------------------------------------
// There is a LTC1859 ADC on the SPI bus
//#include <cyg/infra/cyg_type.h>
#include <cyg/io/spi.h>
#include <cyg/io/adc.h>
#include <cyg/io/adc_ltc1859.h>
#include <cyg/io/spi_freescale_dspi.h>
#include <pkgconf/devs_adc_ltc1859_freescale_dspi.h>
#include <pkgconf/devs_adc_spi_ltc1859.h>

// SPI bus device configuration
#define LTC1859_SPI_FRAME_SIZE      8
#define LTC1859_SPI_CLOCK_POL       0
#define LTC1859_SPI_CLOCK_PHASE     0

#ifdef CYGHWR_DEVS_ADC_LTC1859_DEV0_SPI_USE_DBR
# define LTC1859_SPI_DBR_DEV0 1
#else
# define LTC1859_SPI_DBR_DEV0 0
#endif

#define CYGNUM_HAL_INTERRUPT_FTM_BASE_(_base_) \
        CYGADDR_IO_FTM_FREESCALE_FTM ## _base_ ## _BASE

#define CYGADDR_IO_FTM_FREESCALE_FTM_BASE(_base_) \
        CYGNUM_HAL_INTERRUPT_FTM_BASE_(_base_)

#define CYGNUM_HAL_INTERRUPT_FTM_VECTOR_(_vector_) \
        CYGNUM_HAL_INTERRUPT_FTM ## _vector_

#define CYGNUM_HAL_INTERRUPT_FTM_VECTOR(_vector_) \
        CYGNUM_HAL_INTERRUPT_FTM_VECTOR_(_vector_)

// Underlaying Freescale DSPI device
CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    ltc1859_spi_dev0,                        // Device name
    CYGHWR_DEVS_ADC_LTC1859_DEV0_SPI_BUS,    // SPI bus
    CYGHWR_DEVS_ADC_LTC1859_DEV0_SPI_CS,     // Dev num (CS)
    LTC1859_SPI_FRAME_SIZE,                  // Frame size
    LTC1859_SPI_CLOCK_POL,                   // Clock pol
    LTC1859_SPI_CLOCK_PHASE,                 // Clock phase
    CYGHWR_DEVS_ADC_LTC1859_DEV0_SPEED,      // Clock speed (Hz)
    CYGHWR_DEVS_ADC_LTC1859_DEV0_CS_DLY,     // CS assert delay
    CYGHWR_DEVS_ADC_LTC1859_DEV0_CS_DLY,     // CS negate delay
    CYGHWR_DEVS_ADC_LTC1859_DEV0_CS_DLY,     // Delay between transfers
    CYGHWR_DEVS_ADC_LTC1859_DEV0_CS_DLY_UN,  // Delay unit (100 or 1000 ns)
    LTC1859_SPI_DBR_DEV0                     // Use double baud rate
);

//-----------------------------------------------------------------------------
// Instantiate the LTC1859 device driver.

CYG_ADC_FUNCTIONS(ltc1859_adc_funs,
                  ltc1859_adc_enable,
                  ltc1859_adc_disable,
                  ltc1859_adc_set_rate,
                  ltc1859_adc_set_gain,
                  ltc1859_adc_set_polarity,
                  ltc1859_adc_set_mode);

//-----------------------------------------------------------------------------
// LTC1859 ADC channel instance macro

#define LTC1859_ADC_CHANNEL(_device_, _chan_)                                 \
CYG_ADC_CHANNEL(                                                            \
    ltc1859_adc##_device_##_channel##_chan_,                                  \
    _chan_,                                                                 \
    CYGDAT_DEVS_ADC_LTC1859_ADC##_device_##_CHANNEL##_chan_##_BUFSIZE,\
    &ltc1859_adc_device##_device_                                             \
);                                                                          \
DEVTAB_ENTRY(                                                               \
    ltc1859_adc##_device_##_channel##_chan_##_device,                         \
    CYGDAT_DEVS_ADC_LTC1859_ADC##_device_##_CHANNEL##_chan_##_NAME,   \
    0,                                                                      \
    &cyg_io_adc_devio,                                                      \
    ltc1859_adc_init,                                                         \
    ltc1859_adc_lookup,                                                       \
    &ltc1859_adc##_device_##_channel##_chan_                                  \
);

//-----------------------------------------------------------------------------
// LTC1859 ADC device instances

#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0
#include "adc0.inl"
#endif

#endif // defined(CYGPKG_IO_SPI) && defined(CYGPKG_DEVS_ADC_SPI_LTC1859)

//==========================================================================
// EOF adc_ltc1589_freescale_dspi.c
