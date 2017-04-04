#ifndef CYGONCE_DEVS_SPI_AVR32_UC3C_H
#define CYGONCE_DEVS_SPI_AVR32_UC3C_H
//==========================================================================
//
//      spi_avr32.h
//
//      Atmel AVR32UC3C SPI driver defines
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
// Author(s):      Filip
// Original data:  Savin Zlobec <savin@elatec.si> 
// Contributors:  
// Date:           2012-11-15
//
//####DESCRIPTIONEND####
//
//==========================================================================


#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/spi.h>

//-----------------------------------------------------------------------------
// AVR32UC3C0512 SPI BUS

typedef struct cyg_spi_avr32_bus_s
{
    // ---- Upper layer data ----

    cyg_spi_bus   spi_bus;                  // Upper layer SPI bus data

    // ---- Lower layer data ----
     
    cyg_interrupt               spi_interrupt;        // SPI interrupt object
    cyg_handle_t                spi_interrupt_handle; // SPI interrupt handle
    cyg_drv_mutex_t             transfer_mx;          // Transfer mutex
    cyg_drv_cond_t              transfer_cond;        // Transfer condition
    cyg_bool                    transfer_end;         // Transfer end flag
    cyg_bool                    cs_up;                // Chip Select up flag 
    cyg_vector_t                interrupt_number;     // SPI Interrupt Number
    volatile avr32_spi_t        *spi_dev;             // Base Address of the SPI peripheral
    volatile cyg_uint32         count;
    volatile const cyg_uint8    *tx;
    volatile cyg_uint8          *rx;
} cyg_spi_avr32_bus_t;

//-----------------------------------------------------------------------------
// AVR32UC3C0512 SPI DEVICE

typedef struct cyg_spi_avr32_device_s
{
    // ---- Upper layer data ----

    cyg_spi_device spi_device;  // Upper layer SPI device data

    // ---- Lower layer data (configurable) ----

    cyg_uint8  dev_num;         // Device number
    cyg_uint8  bits;            // Number of bits
    cyg_uint8  cl_pol;          // Clock polarity (0 or 1)
    cyg_uint8  cl_pha;          // Clock phase    (0 or 1)
    cyg_uint32 cl_brate;        // Clock baud rate
    cyg_uint8  cs_up_udly;      // Delay in us between CS up and transfer start
    cyg_uint8  tr_bt_udly;      // Delay in us between two transfers

    // ---- Lower layer data (internal) ----

    cyg_bool   init;            // Is device initialized
    cyg_uint8  cl_scbr;         // Value of SCBR (SPI clock) reg field
} cyg_spi_avr32_device_t;

//-----------------------------------------------------------------------------
// AT91 SPI exported busses

/* For backwards compatability  */
#define cyg_spi_avr32_bus cyg_spi_avr32_bus0

externC cyg_spi_avr32_bus_t cyg_spi_avr32_bus0;
externC cyg_spi_avr32_bus_t cyg_spi_avr32_bus1;

//-----------------------------------------------------------------------------

#endif // CYGONCE_DEVS_SPI_AVR32_UC3C_H 

//-----------------------------------------------------------------------------
// End of spi_avr32.h
