//==========================================================================
//
//      avr32_ehci_serial.c
//
//      Atmel AV32UC3C Serial I/O Interface Module (PDMA driven)
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
// Contributors:  
// Original data: 
// Date:          2016-10-31
// Purpose:       Atmel AVR32UC3C Serial I/O module (PDMA driven version)
// Description: 
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/hal/gpio.h>
#include <cyg/hal/avr32/io.h>

#include <pkgconf/hal.h>
#include <pkgconf/infra.h>
#include <pkgconf/system.h>
#include <pkgconf/kernel.h>
#include <pkgconf/hal_freescale_edma.h>
#include <cyg/hal/freescale_edma.h>

#include <cyg/io/io.h>
#include <cyg/hal/hal_io.h>
#include <cyg/io/ehci_serial.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/io/devtab.h>
#include <cyg/io/ehci_serial.h>
#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>


#if 1
#ifndef CYGPKG_IO_EHCI_SERIAL_AVR32
#define CYGPKG_IO_EHCI_SERIAL_AVR32

#include "avr32_ehci_serial.h"

#define EHCI_DMA_CHAN_TX_I          0
#define EHCI_DMA_CHAN_RX_I          1

typedef struct uart_pins_s {
    cyg_uint32 rx;
    cyg_uint32 tx;
    cyg_uint32 rts;
    cyg_uint32 cts;
} uart_pins_t;

static const uart_pins_t uart0_pins = {
    rx  : CYGHWR_IO_FREESCALE_UART0_PIN_RX,
    tx  : CYGHWR_IO_FREESCALE_UART0_PIN_TX,
    rts : CYGHWR_IO_FREESCALE_UART0_PIN_RTS,
    cts : CYGHWR_IO_FREESCALE_UART0_PIN_CTS
};

static const cyghwr_hal_freescale_dma_chan_set_t ehci_dma_chan[2] =              \
{                                                                                \
    {                                                                            \
        .dma_src = FREESCALE_DMAMUX_SRC_KINETIS_UART0T                           \
        | FREESCALE_DMAMUX_CHCFG_ENBL_M,                                         \
        .dma_chan_i = CYGHWR_DEVS_EHCI_FREESCALE_DSPI_TX_DMA_CHAN,               \
        .dma_prio   = CYGNUM_DEVS_EHCI_FREESCALE_DSPI_TX_DMA_PRI,                \
    },                                                                           \
    {                                                                            \
        .dma_src = FREESCALE_DMAMUX_SRC_KINETIS_UART0R                           \
        | FREESCALE_DMAMUX_CHCFG_ENBL_M,                                         \
        .dma_chan_i = CYGHWR_DEVS_EHCI_FREESCALE_DSPI_RX_DMA_CHAN,               \
        .dma_prio   = CYGNUM_DEVS_EHCI_FREESCALE_DSPI_RX_DMA_PRI,                \
    }                                                                            \
};                                                                               \
                                                                                 \
static cyghwr_hal_freescale_dma_set_t ehci_dma_set = {                           \
    .chan_p = ehci_dma_chan,                                                     \
    .chan_n = 2                                                                  \
};  

static const cyghwr_hal_freescale_edma_tcd_t ehci_dma_tcd_tx_ini =               \
{                                                                                \
        .saddr = NULL,                                                           \
        .soff  = 1,                                                              \
        .attr = FREESCALE_EDMA_ATTR_SSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          \
                FREESCALE_EDMA_ATTR_DSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          \
                FREESCALE_EDMA_ATTR_SMOD(0) |                                    \
                FREESCALE_EDMA_ATTR_DMOD(0),                                     \
        .daddr = (cyg_uint32 *)                                                  \
                 CYGADDR_IO_SERIAL_FREESCALE_UART0_BASE +                        \
                    CYGHWR_DEV_FREESCALE_UART_D,                                 \
        .doff = 0,                                                               \
        .nbytes.mlno = 1,                                                        \
        .slast = 0,                                                              \
        .citer.elinkno = 1,                                                      \
        .dlast_sga.dlast = 0,                                                    \
        .biter.elinkno = 1,                                                      \
        .csr = DSPI_DMA_BANDWIDTH                                                \
};                                                                               \
                                                                                 \
static const cyghwr_hal_freescale_edma_tcd_t ehci_dma_tcd_rx_ini =               \
{                                                                                \
        .saddr = (cyg_uint32 *)                                                  \
                 CYGADDR_IO_SERIAL_FREESCALE_UART0_BASE +                        \
                    CYGHWR_DEV_FREESCALE_UART_D,                                 \
        .soff = 0,                                                               \
        .attr = FREESCALE_EDMA_ATTR_SSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          \
                FREESCALE_EDMA_ATTR_DSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          \
                FREESCALE_EDMA_ATTR_SMOD(0) |                                    \
                FREESCALE_EDMA_ATTR_DMOD(0),                                     \
        .daddr = NULL,                                                           \
        .doff = 1,                                                               \
        .nbytes.mlno = 1,                                                        \
        .slast = 0,                                                              \
        .citer.elinkno = 1,                                                      \
        .dlast_sga.dlast = 0,                                                    \
        .biter.elinkno = 1,                                                      \
        .csr = DSPI_DMA_BANDWIDTH                                                \
}

typedef struct ehci_serial_info {
    CYG_ADDRWORD        uart_base;          // Base address of the uart port
    CYG_WORD            interrupt_num;      // NVIC interrupt vector
    cyg_priority_t      interrupt_priority; // NVIC interupt priority
    const uart_pins_t   *pins_p;         // Rx, Tx, etc.
    cyg_uint32          clock;              // Clock gate
    cyg_bool            tx_active;
    cyg_interrupt       serial_interrupt;
    cyg_handle_t        serial_interrupt_handle;

    cyghwr_hal_freescale_dma_set_t* dma_set_p; // DMA configuration block.
    // DMA transfer control descriptors
    const cyghwr_hal_freescale_edma_tcd_t* tx_dma_tcd_ini_p; // TCD init.
    const cyghwr_hal_freescale_edma_tcd_t* rx_dma_tcd_ini_p; //     data
    volatile cyghwr_hal_freescale_edma_tcd_t* rx_dma_tcd_p; // DMA TCD (RX)
    volatile cyghwr_hal_freescale_edma_tcd_t* tx_dma_tcd_p; // DMA TCD (TX)
} ehci_serial_info;

static avr32_ehci_serial_info avr32_ehci_serial_info_s = {
    uart_base                : CYGADDR_IO_SERIAL_FREESCALE_UART0_BASE,
    interrupt_num            : CYGNUM_IO_SERIAL_FREESCALE_UART0_INT_VECTOR,
    interrupt_priority       : CYGNUM_IO_SERIAL_FREESCALE_UART0_INT_PRIORITY,
    pins_p                   : &uart0_pins,
    clock                    : CYGHWR_IO_FREESCALE_UART0_CLOCK,
    tx_active                : false,

    dma_set_p                : &ehci_dma_set,
    // DMA transfer control descriptors
    tx_dma_tcd_ini_p         : &ehci_dma_tcd_tx_ini,
    rx_dma_tcd_ini_p         : &ehci_dma_tcd_rx_ini
};

static bool avr32_ehci_serial_init(struct cyg_devtab_entry *tab);

static Cyg_ErrNo avr32_ehci_serial_lookup(struct cyg_devtab_entry **tab, 
                                    struct cyg_devtab_entry *sub_tab,
                                    const char *name);

static Cyg_ErrNo avr32_ehci_serial_set_config(ehci_serial_channel *chan, cyg_uint32 key,
                                        const void *xbuf, cyg_uint32 *len);
static void avr32_ehci_serial_start_xmit(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len);

static void
avr32_ehci_serial_start_recive(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len);

static void
avr32_ehci_serial_close(ehci_serial_channel *chan);

static cyg_uint32 avr32_ehci_serial_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       avr32_ehci_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static cyg_uint32 avr32_ehci_serial_rx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       avr32_ehci_serial_rx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static cyg_uint32 avr32_ehci_serial_tx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       avr32_ehci_serial_tx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);


static EHCI_SERIAL_FUNS(avr32_ehci_serial_funs, 
                   avr32_ehci_serial_start_recive,
                   avr32_ehci_serial_start_xmit,
                   avr32_ehci_serial_set_config,
                   avr32_ehci_serial_close
    );


#define CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL3



static EHCI_SERIAL_CHANNEL(
                    avr32_ehci_serial_channel,
                    avr32_ehci_serial_funs, 
                    avr32_ehci_serial_info_s,
                    CYGNUM_SERIAL_BAUD_115200,
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    (CYGNUM_SERIAL_FLOW_RTSCTS_RX|CYGNUM_SERIAL_FLOW_RTSCTS_TX)
    );

DEVTAB_ENTRY(avr32_ehci_serial_io, 
             "/dev/ehci3",
             0,                     // Does not depend on a lower level interface
             NULL, 
             avr32_ehci_serial_init, 
             avr32_ehci_serial_lookup,   // Serial driver may need initializing
             &avr32_ehci_serial_channel
    );


// Internal function to actually configure the hardware to desired baud rate, etc.
static bool
avr32_ehci_serial_config_port(ehci_serial_channel *chan, 
        cyg_serial_info_t *new_config, bool init)
{
    cyg_uint32 regval;
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    ehci_serial_info * uart_chan = (uart_serial_info *)(chan->dev_priv);
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 baud_rate = select_baud[new_config->baud];

    if(!baud_rate) return false;    // Invalid baud rate selected
    
    // Bring clock to the sevice
    CYGHWR_IO_CLOCK_ENABLE(uart_chan->clock);
    // Configure PORT pins
    CYGHWR_IO_FREESCALE_UART_PIN(uart_chan->pins_p->rx);
    CYGHWR_IO_FREESCALE_UART_PIN(uart_chan->pins_p->tx);
    CYGHWR_IO_FREESCALE_UART_PIN(uart_chan->pins_p->rts);
    CYGHWR_IO_FREESCALE_UART_PIN(uart_chan->pins_p->cts);

    CYGHWR_IO_FREESCALE_UART_BAUD_SET(uart_base, baud_rate);

    if(new_config->word_length != 8)
        return false;

    switch(new_config->parity) {
    case CYGNUM_SERIAL_PARITY_NONE:
        regval = 0;
        break;
    case CYGNUM_SERIAL_PARITY_EVEN:
        regval = CYGHWR_DEV_FREESCALE_UART_C1_PE;
        break;
    case CYGNUM_SERIAL_PARITY_ODD:
        regval = CYGHWR_DEV_FREESCALE_UART_C1_PE |
                 CYGHWR_DEV_FREESCALE_UART_C1_PT;
        break;
    default: return false;
    }

    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C1, regval);
    
    /* Set tx/rx FIFO watermark */
    regval = 1;
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_TWFIFO, regval);
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_RWFIFO, regval);
    
    /* Enable tx/rx FIFO */
    HAL_READ_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_PFIFO, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_PFIFO_TXFE | 
              CYGHWR_DEV_FREESCALE_UART_PFIFO_RXFE;
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_RWFIFO, regval);
    
    /* Flush FIFO */
    regval = CYGHWR_DEV_FREESCALE_UART_CFIFO_TXFLUSH | 
             CYGHWR_DEV_FREESCALE_UART_CFIFO_RXFLUSH;
   
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_CFIFO, regval);
    
    /* Enable haardware flow control */
    regval = CYGHWR_DEV_FREESCALE_UART_MODEM_TXCTSE |
             CYGHWR_DEV_FREESCALE_UART_MODEM_RXRTSE;
    
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_MODEM, regval);
    
    if((dma_set_p=uart_chan->dma_set_p)) {
        // Initialize DMA channels
        hal_freescale_edma_init_chanset(dma_set_p);
#if DEBUG_EHCI >= 1
        hal_freescale_edma_diag(dma_set_p, 0xffff);
        cyghwr_devs_freescale_dspi_diag(spi_bus_p);
#endif
        // Set up DMA transfer control descriptors
        edma_p = dma_set_p->edma_p;
        dma_chan_i = dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].dma_chan_i;
        hal_freescale_edma_transfer_init(edma_p, dma_chan_i,
                                         spi_bus_p->tx_dma_tcd_ini_p);
#if DEBUG_EHCI >= 1
        hal_freescale_edma_transfer_diag(edma_p, dma_chan_i, true);
#endif
        dma_chan_i = dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].dma_chan_i;
        hal_freescale_edma_transfer_init(edma_p, dma_chan_i,
                                         spi_bus_p->rx_dma_tcd_ini_p);
#if DEBUG_EHCI >= 1
        hal_freescale_edma_transfer_diag(edma_p, dma_chan_i, true);
#endif
    }
    
    /*if(init) { // Enable the receiver interrupt
        regval = CYGHWR_DEV_FREESCALE_UART_C2_RIE;
    } else {    // Restore the old interrupt state
        HAL_READ_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    }*/

    // Enable the device
    regval |= CYGHWR_DEV_FREESCALE_UART_C2_TE |
              CYGHWR_DEV_FREESCALE_UART_C2_RE;

    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);

    uart_chan->tx_active = false;

    if(new_config != &chan->config)
        chan->config = *new_config;

    return true;
}

// Function to initialize the device.  Called at bootstrap time.
static bool 
avr32_ehci_serial_init(struct cyg_devtab_entry *tab)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) tab->priv;
    ehci_serial_info * const avr32_chan = (ehci_serial_info *) chan->dev_priv;
    int res;

#ifdef CYGDBG_IO_INIT
    diag_printf("AVR32 SERIAL init - dev: %x.%d\n", avr32_chan->usart_dev, avr32_chan->int_num);
#endif
   
    //avr32_chan->flags = SIFLG_NONE;
    //avr32_chan->stat  = 0;
    //(chan->callbacks->serial_init)(chan);  // Really only required for interrupt driven devices

    //serial interrupt
    cyg_drv_interrupt_create(avr32_chan->int_num,
                             1,                      // Priority
                             (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                             avr32_ehci_serial_ISR,
                             avr32_ehci_serial_DSR,
                             &avr32_chan->serial_interrupt_handle,
                             &avr32_chan->serial_interrupt);
    cyg_drv_interrupt_attach(avr32_chan->serial_interrupt_handle);

    res = avr32_ehci_serial_config_port(chan, &chan->config, true);
    chan->init = true;
    return res;
}

// This routine is called when the device is "looked" up (i.e. attached)
static Cyg_ErrNo 
avr32_ehci_serial_lookup(struct cyg_devtab_entry **tab, 
                  struct cyg_devtab_entry *sub_tab,
                  const char *name)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) (*tab)->priv;
    
    // TODO: Power on bluettoth modlue
    
    if(!chan->init)
    {
        avr32_ehci_serial_init(*tab);
        chan->init = true;
    }
    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(CYGHWR_HAL_KINETIS_GPIO_PORTA_P,BT_NSHUTD_PIN);
    //(chan->callbacks->ehci_init)(chan);  // Really only required for interrupt driven devices
    return ENOERR;
}

static void
avr32_ehci_serial_close(ehci_serial_channel *chan)
{
    // TODO: Power off Bluetooth module
    gpio_set_pin_low(BT_NSHUTD_PIN);
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    
    CYG_PDMA_DISABLE_CHANEL(avr32_chan->pdca_tx_channel);
    CYG_PDMA_DISABLE_CHANEL(avr32_chan->pdca_rx_channel);
    dev->idr = -1;
    //dev->icr = -1;
    dev->cr = AVR32_USART_CR_RXDIS_MASK | AVR32_USART_CR_TXDIS_MASK |
              AVR32_USART_RTSEN_MASK ;
    
    chan->init = 0;
}

// Set up the device characteristics; baud rate, etc.
static Cyg_ErrNo
avr32_ehci_serial_set_config(ehci_serial_channel *chan, cyg_uint32 key,
                       const void *xbuf, cyg_uint32 *len)
{
    switch (key) {
    case CYG_IO_SET_CONFIG_SERIAL_INFO:
    {
        cyg_serial_info_t *config = (cyg_serial_info_t *)xbuf;
        if ( *len < sizeof(cyg_serial_info_t) ) {
            return -EINVAL;
        }
        *len = sizeof(cyg_serial_info_t);
        if ( true != avr32_ehci_serial_config_port(chan, config, false) )
            return -EINVAL;
    }
    break;
    


    case CYG_IO_SET_CONFIG_SERIAL_HW_RX_FLOW_THROTTLE:
    {
        // RX flow control involves just the RTS line. Most of the
        // work is done by the hardware depending on the state of
        // the fifo. This option serves mainly to drop RTS if
        // higher-level code is running out of buffer space, even
        // if the fifo is not yet full.
        avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
        volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
        cyg_uint32 *f = (cyg_uint32 *)xbuf;
        unsigned char mask=0;
        if ( *len < sizeof(*f) )
            return -EINVAL;

        if ( chan->config.flags & CYGNUM_SERIAL_FLOW_RTSCTS_RX )
            mask = AVR32_USART_RTSDIS_MASK;
        else
            mask = AVR32_USART_RTSEN_MASK;

        dev->cr = mask; 
    }
        break;
        
    case CYG_IO_SET_CONFIG_SERIAL_HW_FLOW_CONFIG:
      break;
    default:
        return -EINVAL;
    }
    return ENOERR;
}

char buf[64];
// Enable the transmitter on the device
static void
avr32_ehci_serial_start_xmit(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len)
{
    ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t* edma_p = NULL;
    cyg_uint32 dma_chan_rx_i = 0;
    cyg_uint32 dma_chan_tx_i = 0;

    cyg_drv_dsr_lock();
    
    if(!uart_chan->tx_active) {
        // enable trasmitter
        uart_chan->tx_active = true;
        dma_set_p = avr32_chan->dma_set_p;
        edma_p = dma_set_p->edma_p;
        
        // Set up the DMA channels.
        dma_chan_rx_i = dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].dma_chan_i;
        edma_p->tcd[dma_chan_rx_i].saddr = buffer;
        edma_p->tcd[dma_chan_rx_i].biter.elinkno = len;
        
        hal_freescale_edma_erq_enable(edma_p, dma_chan_rx_i);
        dspi_irq_enable(dspi_p,
                        FREESCALE_DSPI_RSER_TFFF_RE_M   |
                        FREESCALE_DSPI_RSER_RFDF_RE_M   |
                        FREESCALE_DSPI_RSER_TFFF_DIRS_M |
                        FREESCALE_DSPI_RSER_RFDF_DIRS_M);
        
    }            
    cyg_drv_dsr_unlock();
}
char buf[64];
int  inaction = 0;
static void
avr32_ehci_serial_start_recive(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len)
{
    ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t* edma_p = NULL;
    cyg_uint32 dma_chan_rx_i = 0;
    cyg_uint32 dma_chan_tx_i = 0;

    cyg_drv_dsr_lock();
    uart_chan->tx_active = true;
    dma_set_p = avr32_chan->dma_set_p;
    edma_p = dma_set_p->edma_p;

    // Set up the DMA channels.
    dma_chan_rx_i = EHCI_DMA_CHAN_I(dma_set_p, RX);
    dma_chan_tx_i = EHCI_DMA_CHAN_I(dma_set_p, TX);
    rx_dma_channel_setup(dma_set_p, (cyg_uint8*) rx_data,
                         bus_16bit, &edma_p->tcd[dma_chan_rx_i]);
    hal_freescale_edma_erq_enable(edma_p, dma_chan_rx_i);
    dspi_irq_enable(dspi_p,
                    FREESCALE_DSPI_RSER_TFFF_RE_M   |
                    FREESCALE_DSPI_RSER_RFDF_RE_M   |
                    FREESCALE_DSPI_RSER_TFFF_DIRS_M |
                    FREESCALE_DSPI_RSER_RFDF_DIRS_M);
    
                
    cyg_drv_dsr_unlock();
}

// Serial I/O - low level interrupt handler (ISR)
static void       
avr32_ehci_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    ehci_serial_channel *chan = (ehci_serial_channel *)data;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *)chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyg_uint32 _csr, _ier = 0;

    _csr = dev->csr;
    
    //CYG_FAIL("UART FAIL\n");

#ifdef CYGOPT_IO_EHCI_SERIAL_FLOW_CONTROL_HW
    if(_csr & AVR32_USART_CSR_CTSIC_MASK)
    {
        (chan->callbacks->csr_handler)( );
        _ier |= AVR32_USART_IER_CTSIC_MASK;
    }
#endif
    dev->cr = AVR32_USART_CR_RSTSTA_MASK;
    dev->ier = _ier;   
}


// Serial I/O - high level interrupt handler (DSR)
static cyg_uint32
avr32_ehci_serial_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) data;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyg_uint32 ret = CYG_ISR_HANDLED;
    // Check if we have an interrupt pending - note that the interrupt
    // is pending of the low bit of the isr is *0*, not 1.
    
    dev->idr = dev->csr & dev->imr;
    /*diag_printf("Err\n");
    while(1)
    {
        
    }*/
    if(dev->csr & dev->imr)
        ret |= CYG_ISR_CALL_DSR;
    
    //dev->cr = AVR32_USART_STTTO_MASK | AVR32_USART_RETTO_MASK;
    return ret;
}

// Serial tx dma dsr
static void       
avr32_ehci_serial_tx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    ehci_serial_channel *chan = (ehci_serial_channel *)data;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *)chan->dev_priv;

    if (CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_tx_channel)) 
    {
        //CYG_PDMA_DISABLE_INTERRUPT(avr32_chan->pdca_tx_channel);
        //diag_printf("tx\n");
                                               
        (chan->callbacks->block_sent_handler)();
    } 
    else
    {
        CYG_FAIL("TX PDMA\n");
    }
}


// Serial tx dma isr
static cyg_uint32
avr32_ehci_serial_tx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) data;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    
    cyg_uint32 ret = CYG_ISR_HANDLED;

#if 1
   // if(CYG_PDMA_IS_ENABLED(avr32_chan->pdca_tx_channel))
    {
      if(CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_tx_channel))
      {
            
            CYG_PDMA_DISABLE_INTERRUPT(avr32_chan->pdca_tx_channel);
            //avr32_chan->tx_trans_rdy = true;
            ret |= CYG_ISR_CALL_DSR;
             //(chan->callbacks->block_sent_handler)();
      }
    }
#else
    if (CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_tx_channel) &&
	    CYG_PDMA_IS_ENABLED(avr32_chan->pdca_tx_channel)) 
    {
        CYG_PDMA_DISABLE_CHANEL(avr32_chan->pdca_tx_channel);
        //diag_printf("tx\n");
 
        (chan->callbacks->block_sent_handler)();
    } 
    else
    {
        CYG_PDMA_DISABLE_CHANEL(avr32_chan->pdca_tx_channel);
        CYG_FAIL("TX PDMA\n");
    }
#endif

    return ret;
}

///
// Serial rx dma dsr
static void       
avr32_ehci_serial_rx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    ehci_serial_channel *chan = (ehci_serial_channel *)data;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *)chan->dev_priv;
    
    if (CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_rx_channel)) 
    {   
        (chan->callbacks->block_received_handler)();
    }
    else
    {
        CYG_FAIL("RX PDMA\n");
    }
}


// Serial rx dma dsr
static cyg_uint32
avr32_ehci_serial_rx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) data;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    
    cyg_uint32 ret = CYG_ISR_HANDLED;
    //dev->cr = AVR32_USART_CR_RXDIS_MASK;
    dev->cr = AVR32_USART_CR_RTSDIS_MASK;
#if 1
    if(CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_rx_channel))
    {
        CYG_PDMA_DISABLE_INTERRUPT(avr32_chan->pdca_rx_channel);
        dev->cr = AVR32_USART_CR_RTSDIS_MASK;
         avr32_chan->rx_trans_rdy = true;
         ret |= CYG_ISR_CALL_DSR;
        //(chan->callbacks->block_received_handler)();
    }
#else
    if (CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_rx_channel) &&
	    CYG_PDMA_IS_ENABLED(avr32_chan->pdca_rx_channel)) 
    {
        
        CYG_PDMA_DISABLE_INTERRUPT(avr32_chan->pdca_rx_channel);
        //
        
       // diag_printf("rx\n"); 
       (chan->callbacks->block_received_handler)(); 
    }
    else
    {
        CYG_PDMA_DISABLE_CHANEL(avr32_chan->pdca_rx_channel);
        CYG_FAIL("RX PDMA\n");
    }
#endif
    return ret;
}
#endif

#endif