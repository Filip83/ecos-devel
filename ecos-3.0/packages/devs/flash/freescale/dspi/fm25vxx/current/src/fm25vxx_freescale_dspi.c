//==========================================================================
//
//      fm25vxx_freescale_dspi.c
//
//      FM25VXX FLASH device over Freescale DSPI
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
// Contributor(s): Ilija Kocho
// Date:           2013-05-18
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================


#include <pkgconf/system.h>
#if defined(CYGPKG_IO_SPI) && defined(CYGPKG_DEVS_FLASH_SPI_FM25VXX)
// ------------------------------------------------------------------------
// There is an FM25VXX serial flash on the SPI bus
//#include <cyg/infra/cyg_type.h>
#include <cyg/io/spi.h>
#include <cyg/io/fm25vxx.h>
#include <cyg/io/spi_freescale_dspi.h>
#include <pkgconf/devs_flash_fm25vxx_freescale_dspi.h>

// SPI bus device configuration
#define FM25VXX_SPI_FRAME_SIZE      8
#define FM25VXX_SPI_CLOCK_POL       0
#define FM25VXX_SPI_CLOCK_PHASE     0

#ifdef CYGHWR_DEVS_FLASH_FM25VXX_DEV0_SPI_USE_DBR
# define FM25VXX_SPI_DBR_DEV0 1
#else
# define FM25VXX_SPI_DBR_DEV0 0
#endif

// Underlaying Freescale DSPI device
CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    fm25vxx_spi_dev0,                        // Device name
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_SPI_BUS,  // SPI bus
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_SPI_CS,   // Dev num (CS)
    FM25VXX_SPI_FRAME_SIZE,                  // Frame size
    FM25VXX_SPI_CLOCK_POL,                   // Clock pol
    FM25VXX_SPI_CLOCK_PHASE,                 // Clock phase
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_SPEED,    // Clock speed (Hz)
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_CS_DLY,   // CS assert delay
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_CS_DLY,   // CS negate delay
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_CS_DLY,   // Delay between transfers
    CYGHWR_DEVS_FLASH_FM25VXX_DEV0_CS_DLY_UN,// Delay unit (100 or 1000 ns)
    FM25VXX_SPI_DBR_DEV0                     // Use double baud rate
);

//-----------------------------------------------------------------------------
// Instantiate the FM25Vxx device driver.

CYG_DEVS_FLASH_SPI_FM25VXX_DRIVER(fm25vxx_flash_dev0,
                                  CYGNUM_DEVS_FLASH_SPI_FM25VXX_DEV0_MAP_ADDR,
                                  &fm25vxx_spi_dev0);

#endif // defined(CYGPKG_IO_SPI) && defined(CYGPKG_DEVS_FLASH_SPI_FM25VXX)

//==========================================================================
// EOF fm25vxx_freescale_dspi.c
