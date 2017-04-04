#ifndef CYGONCE_DEVS_DMA_USART_SPI_AVR32_UC3C_H
#define CYGONCE_DEVS_DMA_USART_SPI_AVR32_UC3C_H
//==========================================================================
//
//      spi_uc3c.h
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
// Author(s):     Filip
// Date:          2012-11-15
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
// AVR32UC3C SPI BUS

typedef struct cyg_dma_usart_spi_avr32_bus_s
{
    // ---- Upper layer data ----

    cyg_spi_bus         spi_bus;                  // Upper layer SPI bus data

    // ---- Lower layer data ----

    cyg_interrupt           spi_interrupt;        // SPI interrupt object
    cyg_handle_t            spi_interrupt_handle; // SPI interrupt handle
    cyg_interrupt           spi_interrupt2;       // SPI interrupt object
    cyg_handle_t            spi_interrupt_handle2;// SPI interrupt handle
    cyg_drv_mutex_t         transfer_mx;          // Transfer mutex
    cyg_drv_cond_t          transfer_cond;        // Transfer condition
    cyg_bool                transfer_end;         // Transfer end flag
    cyg_bool                cs_up;                // Chip Select up flag
    cyg_vector_t            interrupt_number;     // SPI Interrupt Number
    volatile avr32_usart_t *spi_dev;     // Base Address of the SPI peripheral
    cyg_uint32              count;                // Number of bytes to transfer
    cyg_uint8               dma_rx_ch;            // RX PDMA channell number
    cyg_uint8               dma_tx_ch;            // TX PDMA channell number
    cyg_uint8               dma_pid_rx;           // RX PDMA channell identifier
    cyg_uint8               dma_pid_tx;           // RX PDMA channell identifier
    volatile avr32_pdca_channel_t	*pdca_rx_channel; //rx PDMA chanel
    volatile avr32_pdca_channel_t	*pdca_tx_channel; //tx PDMA chanel
} cyg_dma_usart_spi_avr32_bus_t;

//-----------------------------------------------------------------------------
// AVR32UC3C0512 SPI DEVICE

typedef struct cyg_dma_usart_spi_avr32_device_s
{
    // ---- Upper layer data ----

    cyg_spi_device spi_device;  // Upper layer SPI device data

    // ---- Lower layer data (configurable) ----

    cyg_uint8  dev_num;         // Device number
    cyg_uint8  bits;
    cyg_uint8  cl_pol;          // Clock polarity (0 or 1)
    cyg_uint8  cl_pha;          // Clock phase    (0 or 1)
    cyg_uint32 cl_brate;        // Clock baud rate
    cyg_uint8  cs_up_udly;      // Delay in us between CS up and transfer start
    cyg_uint8  tr_bt_udly;      // Delay in us between two transfers

    // ---- Lower layer data (internal) ----

    cyg_bool   init;            // Is device initialized
    cyg_uint32  cl_scbr;         // Value of SCBR (SPI clock) reg field
} cyg_dma_usart_spi_avr32_device_t;

//-----------------------------------------------------------------------------
// AVR32UC3C SPI exported busses

/* For backwards compatability  */
#define cyg_dma_usart_spi_avr32_bus   cyg_dma_usart_spi_avr32_bus0

externC cyg_dma_usart_spi_avr32_bus_t cyg_dma_usart_spi_avr32_bus0;
externC cyg_dma_usart_spi_avr32_bus_t cyg_dma_usart_spi_avr32_bus1;
externC cyg_dma_usart_spi_avr32_bus_t cyg_dma_usart_spi_avr32_bus2;
externC cyg_dma_usart_spi_avr32_bus_t cyg_dma_usart_spi_avr32_bus3;
externC cyg_dma_usart_spi_avr32_bus_t cyg_dma_usart_spi_avr32_bus4;


//-----------------------------------------------------------------------------

#endif // CYGONCE_DEVS_DMA_USART_SPI_AVR32_UC3C_H

//-----------------------------------------------------------------------------
// End of spi_avr32.h
