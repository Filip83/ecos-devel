//==========================================================================
//
//      uart_ehci_serial.c
//
//      Freescale Kinetis ehci driver.
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
// Date:          2017-03-28
// Purpose:       Freescale Kinetis ehci driver unsing DMA
// Description: 
//
//####DESCRIPTIONEND####
//
//==========================================================================
#include <pkgconf/hal.h>
#include <pkgconf/io_serial.h>
#include <pkgconf/io.h>
#include <pkgconf/hal_freescale_edma.h>
#include <pkgconf/devs_ehci_serial_freescale.h>

#include <string.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_cache.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/ehci_serial.h>
#include <cyg/io/devtab.h>
#include <cyg/hal/freescale_edma.h>
#include <cyg/io/ser_freescale_uart.h>

//#define CYGPKG_DEVS_EHCI_SERIAL_FREESCALE
#ifdef  CYGPKG_DEVS_EHCI_SERIAL_FREESCALE


#define EHCI_DMA_CHAN_TX_I          0
#define EHCI_DMA_CHAN_RX_I          1

typedef struct uart_pins_s {
    cyg_uint32 rx;
    cyg_uint32 tx;
    cyg_uint32 rts;
    cyg_uint32 cts;
} uart_pins_t;

static const uart_pins_t uart0_pins = {
    rx  : CYGHWR_IO_FREESCALE_EHCI_PIN_RX,
    tx  : CYGHWR_IO_FREESCALE_EHCI_PIN_TX,
    rts : CYGHWR_IO_FREESCALE_EHCI_PIN_RTS,
    cts : CYGHWR_IO_FREESCALE_EHCI_PIN_CTS
};

static const cyghwr_hal_freescale_dma_chan_set_t ehci_dma_chan[2] =              
{                                                                                
    {                                                                            
        .dma_src = FREESCALE_DMAMUX_SRC_KINETIS_EHCI_TX                         
        | FREESCALE_DMAMUX_CHCFG_ENBL_M,                                         
        .dma_chan_i = CYGHWR_DEVS_EHCI_SERIAL_FREESCALE_TX_DMA_CHAN,                    
        .dma_prio   = CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_TX_DMA_PRI,                     
        .isr_prio   = CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_TX_DMA_ISR_PRI,                
        .isr_num    = CYGNUM_HAL_INTERRUPT_DMA0 +                             
                      CYGHWR_DEVS_EHCI_SERIAL_FREESCALE_TX_DMA_CHAN             
    },                                                                           
    {                                                                            
        .dma_src = FREESCALE_DMAMUX_SRC_KINETIS_EHCI_RX                         
        | FREESCALE_DMAMUX_CHCFG_ENBL_M,                                         
        .dma_chan_i = CYGHWR_DEVS_EHCI_SERIAL_FREESCALE_RX_DMA_CHAN,                    
        .dma_prio   = CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_RX_DMA_PRI,                     
        .isr_prio   = CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_RX_DMA_ISR_PRI,                
        .isr_num    = CYGNUM_HAL_INTERRUPT_DMA0 +                             
                      CYGHWR_DEVS_EHCI_SERIAL_FREESCALE_RX_DMA_CHAN             
    }                                                                            
};                                                                               
                                                                                 
static cyghwr_hal_freescale_dma_set_t ehci_dma_set = {                           
    .chan_p = ehci_dma_chan,                                                     
    .chan_n = 2                                                                  
};  

static const cyghwr_hal_freescale_edma_tcd_t ehci_dma_tcd_tx_ini =               
{                                                                                
        .saddr = NULL,                                                           
        .soff  = 1,                                                              
        .attr = FREESCALE_EDMA_ATTR_SSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          
                FREESCALE_EDMA_ATTR_DSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          
                FREESCALE_EDMA_ATTR_SMOD(0) |                                    
                FREESCALE_EDMA_ATTR_DMOD(0),                                     
        .daddr = (cyg_uint32 *)                                                  
                 CYGADDR_DEVS_EHCI_SERIAL_FREESCALE_UART_BASE +                  
                 CYGHWR_DEV_FREESCALE_UART_D,                                 
        .doff = 0,                                                               
        .nbytes = 1,                                                             
        .slast = 0,                                                              
        .citer.elinkno = 0,                                                      
        .dlast_sga.dlast = 0,                                                    
        .biter.elinkno = 0,                                                      
        .csr = FREESCALE_EDMA_CSR_BWC_0                                                
};                                                                               
                                                                                 
static const cyghwr_hal_freescale_edma_tcd_t ehci_dma_tcd_rx_ini =               
{                                                                                
        .saddr = (cyg_uint32 *)                                                  
                 CYGADDR_DEVS_EHCI_SERIAL_FREESCALE_UART_BASE +                  
                 CYGHWR_DEV_FREESCALE_UART_D,                                 
        .soff = 0,                                                               
        .attr = FREESCALE_EDMA_ATTR_SSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          
                FREESCALE_EDMA_ATTR_DSIZE(FREESCALE_EDMA_ATTR_SIZE_8) |          
                FREESCALE_EDMA_ATTR_SMOD(0) |                                    
                FREESCALE_EDMA_ATTR_DMOD(0),                                     
        .daddr = NULL,                                                           
        .doff = 1,                                                               
        .nbytes = 1,                                                             
        .slast = 0,                                                              
        .citer.elinkno = 0,                                                      
        .dlast_sga.dlast = 0,                                                    
        .biter.elinkno = 0,                                                      
        .csr = FREESCALE_EDMA_CSR_BWC_0                                                
};

typedef struct freescale_ehci_serial_info {
    CYG_ADDRWORD        uart_base;          // Base address of the uart port
    CYG_WORD            interrupt_num;      // NVIC interrupt vector
    cyg_priority_t      interrupt_priority; // NVIC interupt priority
    const uart_pins_t   *pins_p;         // Rx, Tx, etc.
    cyg_uint32          clock;              // Clock gate
    cyg_bool            tx_active;
    cyg_interrupt       serial_interrupt;
    cyg_handle_t        serial_interrupt_handle;
    cyg_interrupt       serial_tx_dma_interrupt;
    cyg_handle_t        serial_tx_dma_interrupt_handle;
    cyg_interrupt       serial_rx_dma_interrupt;
    cyg_handle_t        serial_rx_dma_interrupt_handle;

    cyghwr_hal_freescale_dma_set_t* dma_set_p; // DMA configuration block.
    // DMA transfer control descriptors
    const cyghwr_hal_freescale_edma_tcd_t* tx_dma_tcd_ini_p; // TCD init.
    const cyghwr_hal_freescale_edma_tcd_t* rx_dma_tcd_ini_p; //     data
    volatile cyghwr_hal_freescale_edma_tcd_t* rx_dma_tcd_p; // DMA TCD (RX)
    volatile cyghwr_hal_freescale_edma_tcd_t* tx_dma_tcd_p; // DMA TCD (TX)
} freescale_ehci_serial_info;

static freescale_ehci_serial_info freescale_ehci_serial_info_s = {
    uart_base                : CYGADDR_DEVS_EHCI_SERIAL_FREESCALE_UART_BASE,
    interrupt_num            : CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_ISR_NUM,
    interrupt_priority       : CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_ISR_PRI,
    pins_p                   : &uart0_pins,
    clock                    : CYGHWR_IO_FREESCALE_EHCI_UART_CLOCK,
    tx_active                : false,

    dma_set_p                : &ehci_dma_set,
    // DMA transfer control descriptors
    tx_dma_tcd_ini_p         : &ehci_dma_tcd_tx_ini,
    rx_dma_tcd_ini_p         : &ehci_dma_tcd_rx_ini
};

static bool freescale_ehci_serial_init(struct cyg_devtab_entry *tab);

static Cyg_ErrNo freescale_ehci_serial_lookup(struct cyg_devtab_entry **tab, 
                                    struct cyg_devtab_entry *sub_tab,
                                    const char *name);

static Cyg_ErrNo freescale_ehci_serial_set_config(ehci_serial_channel *chan, cyg_uint32 key,
                                        const void *xbuf, cyg_uint32 *len);

static void freescale_ehci_serial_start_xmit(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len);

static void
freescale_ehci_serial_start_recive(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len);

static void
freescale_ehci_serial_close(ehci_serial_channel *chan);

static cyg_uint32 freescale_ehci_serial_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       freescale_ehci_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static cyg_uint32 freescale_ehci_rx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       freescale_ehci_rx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static cyg_uint32 freescale_ehci_tx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       freescale_ehci_tx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);


static EHCI_SERIAL_FUNS(
        freescale_ehci_serial_funs, 
        freescale_ehci_serial_start_recive,
        freescale_ehci_serial_start_xmit,
        freescale_ehci_serial_set_config,
        freescale_ehci_serial_close
    );


static EHCI_SERIAL_CHANNEL(
        freescale_ehci_serial_channel,
        freescale_ehci_serial_funs, 
        freescale_ehci_serial_info_s,
        CYGNUM_SERIAL_BAUD_115200,
        CYG_SERIAL_STOP_DEFAULT,
        CYG_SERIAL_PARITY_DEFAULT,
        CYG_SERIAL_WORD_LENGTH_DEFAULT,
        (CYGNUM_SERIAL_FLOW_RTSCTS_RX|CYGNUM_SERIAL_FLOW_RTSCTS_TX)
    );

DEVTAB_ENTRY(
        freescale_ehci_serial_io, 
        "/dev/ehci",
        0,                     // Does not depend on a lower level interface
        NULL, 
        freescale_ehci_serial_init, 
        freescale_ehci_serial_lookup,   // Serial driver may need initializing
        &freescale_ehci_serial_channel
    );

// Available baud rates
static cyg_int32 select_baud[] = {
    0,      // Unused
    50,     // 50
    75,     // 75
    110,    // 110
    0,      // 134.5
    150,    // 150
    200,    // 200
    300,    // 300
    600,    // 600
    1200,   // 1200
    1800,   // 1800
    2400,   // 2400
    3600,   // 3600
    4800,   // 4800
    7200,   // 7200
    9600,   // 9600
    14400,  // 14400
    19200,  // 19200
    38400,  // 38400
    57600,  // 57600
    115200, // 115200
    230400, // 230400
};

// Internal function to actually configure the hardware to desired baud rate, etc.
static bool
freescale_ehci_serial_config_port(ehci_serial_channel *chan, 
        cyg_serial_info_t *new_config, bool init)
{
    cyg_uint32 regval;
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t *edma_p;
    freescale_ehci_serial_info * uart_chan = (freescale_ehci_serial_info *)(chan->dev_priv);
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 baud_rate;
    cyg_uint32 dma_chan_i;

    // Enable uso of hgner baud rates
    if(new_config->baud > 21)
        baud_rate = new_config->baud;
    else
        baud_rate = select_baud[new_config->baud];
    
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
    
    /* Enable DMA requsts on RX and TX channel */
    regval = CYGHWR_DEV_FREESCALE_UART_C5_TDMAS |
             CYGHWR_DEV_FREESCALE_UART_C5_RDMAS;
    
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C5, regval);
    
    if((dma_set_p=uart_chan->dma_set_p)) {
        // Initialize DMA channels
        hal_freescale_edma_init_chanset(dma_set_p);
#if DEBUG_EHCI >= 1
        hal_freescale_edma_diag(dma_set_p, 0xffff);
        cyghwr_devs_freescale_dspi_diag(uart_chan);
#endif
        // Set up DMA transfer control descriptors
        edma_p = dma_set_p->edma_p;
        dma_chan_i = dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].dma_chan_i;
        hal_freescale_edma_transfer_init(edma_p, dma_chan_i,
                                         uart_chan->tx_dma_tcd_ini_p);
#if DEBUG_EHCI >= 1
        hal_freescale_edma_transfer_diag(edma_p, dma_chan_i, true);
#endif
        dma_chan_i = dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].dma_chan_i;
        hal_freescale_edma_transfer_init(edma_p, dma_chan_i,
                                         uart_chan->rx_dma_tcd_ini_p);
#if DEBUG_EHCI >= 1
        hal_freescale_edma_transfer_diag(edma_p, dma_chan_i, true);
#endif
    }

    // Enable the device
    regval = CYGHWR_DEV_FREESCALE_UART_C2_TE |
             CYGHWR_DEV_FREESCALE_UART_C2_RE;

    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);

    uart_chan->tx_active = false;

    if(new_config != &chan->config)
        chan->config = *new_config;

    return true;
}

// Function to initialize the device.  Called at bootstrap time.
static bool 
freescale_ehci_serial_init(struct cyg_devtab_entry *tab)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) tab->priv;
    freescale_ehci_serial_info * const freescale_chan = (freescale_ehci_serial_info *) chan->dev_priv;
    int res;

#ifdef CYGDBG_IO_INIT
    diag_printf("Freescale EHCI init\n");
#endif
   
    //serial interrupt
    cyg_drv_interrupt_create(freescale_chan->interrupt_num,
                             freescale_chan->interrupt_priority,                      // Priority
                             (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                             freescale_ehci_serial_ISR,
                             freescale_ehci_serial_DSR,
                             &freescale_chan->serial_interrupt_handle,
                             &freescale_chan->serial_interrupt);
    cyg_drv_interrupt_attach(freescale_chan->serial_interrupt_handle);
    
    //DMA interrupt
    cyg_drv_interrupt_create(ehci_dma_chan[EHCI_DMA_CHAN_TX_I].isr_num,
                             ehci_dma_chan[EHCI_DMA_CHAN_TX_I].isr_prio,                      // Priority
                             (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                             freescale_ehci_tx_dma_ISR,
                             freescale_ehci_tx_dma_DSR,
                             &freescale_chan->serial_tx_dma_interrupt_handle,
                             &freescale_chan->serial_tx_dma_interrupt);
    cyg_drv_interrupt_attach(freescale_chan->serial_tx_dma_interrupt_handle);
    
    cyg_drv_interrupt_create(ehci_dma_chan[EHCI_DMA_CHAN_RX_I].isr_num,
                             ehci_dma_chan[EHCI_DMA_CHAN_RX_I].isr_prio,   
                             (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                             freescale_ehci_rx_dma_ISR,
                             freescale_ehci_rx_dma_DSR,
                             &freescale_chan->serial_rx_dma_interrupt_handle,
                             &freescale_chan->serial_rx_dma_interrupt);
    cyg_drv_interrupt_attach(freescale_chan->serial_rx_dma_interrupt_handle);

    res = freescale_ehci_serial_config_port(chan, &chan->config, true);
    chan->init = true;
    return res;
}

// This routine is called when the device is "looked" up (i.e. attached)
static Cyg_ErrNo 
freescale_ehci_serial_lookup(struct cyg_devtab_entry **tab, 
                  struct cyg_devtab_entry *sub_tab,
                  const char *name)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) (*tab)->priv;
    
    // TODO: Power on bluettoth modlue
    
    if(!chan->init)
    {
        freescale_ehci_serial_init(*tab);
        chan->init = true;
    }
    CYGHWR_IO_SET_PIN_NSHUTD;
    return ENOERR;
}

static void
freescale_ehci_serial_close(ehci_serial_channel *chan)
{
    cyg_uint32 regval;
    freescale_ehci_serial_info * uart_chan = (freescale_ehci_serial_info *)(chan->dev_priv);
    cyg_addrword_t uart_base = uart_chan->uart_base;
    
    // Disable ehci uart
    regval = 0;
    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    // Shut down module
    CYGHWR_IO_CLEAR_PIN_NSHUTD;
    
    chan->init = 0;
}

// Set up the device characteristics; baud rate, etc.
static Cyg_ErrNo
freescale_ehci_serial_set_config(ehci_serial_channel *chan, cyg_uint32 key,
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
        if ( true != freescale_ehci_serial_config_port(chan, config, false) )
            return -EINVAL;
    }
    break;
    default:
        return -EINVAL;
    }
    return ENOERR;
}

// Enable the transmitter on the device
static void
freescale_ehci_serial_start_xmit(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len)
{
    freescale_ehci_serial_info * const uart_chan = (freescale_ehci_serial_info *)
                                                chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 regval;
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t* edma_p = NULL;
    cyg_uint32 dma_chan_tx_i = 0;

    cyg_drv_dsr_lock();
    
    if(!uart_chan->tx_active) {
        // enable trasmitter
        uart_chan->tx_active = true;
        dma_set_p = uart_chan->dma_set_p;
        edma_p = dma_set_p->edma_p;
        
        // Set up the DMA channels.
        dma_chan_tx_i = dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].dma_chan_i;
        edma_p->tcd[dma_chan_tx_i].saddr = buffer;
        edma_p->tcd[dma_chan_tx_i].biter.elinkno = len;
        edma_p->tcd[dma_chan_tx_i].citer.elinkno = len;
        edma_p->tcd[dma_chan_tx_i].csr |= FREESCALE_EDMA_CSR_INTMAJOR_M |
                                          FREESCALE_EDMA_CSR_DREQ_M;
        
        // Enable TX dma request
        HAL_READ_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
        regval |=  CYGHWR_DEV_FREESCALE_UART_C2_TIE;
        
        HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
        
        // Enable edma
        hal_freescale_edma_erq_enable(edma_p, dma_chan_tx_i);
        
    }            
    cyg_drv_dsr_unlock();
}

static void
freescale_ehci_serial_start_recive(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len)
{
    freescale_ehci_serial_info * const uart_chan = (freescale_ehci_serial_info *) 
                                                chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 regval;
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t* edma_p = NULL;
    cyg_uint32 dma_chan_rx_i = 0;

    cyg_drv_dsr_lock();
    
    dma_set_p = uart_chan->dma_set_p;
    edma_p = dma_set_p->edma_p;

    // Set up the DMA channels.
    dma_chan_rx_i = dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].dma_chan_i;
    edma_p->tcd[dma_chan_rx_i].saddr = buffer;
    edma_p->tcd[dma_chan_rx_i].biter.elinkno = len;
    edma_p->tcd[dma_chan_rx_i].citer.elinkno = len;
    edma_p->tcd[dma_chan_rx_i].csr |= FREESCALE_EDMA_CSR_INTMAJOR_M |
                                      FREESCALE_EDMA_CSR_DREQ_M;

    // Enable RX DMA request
    HAL_READ_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    regval |=  CYGHWR_DEV_FREESCALE_UART_C2_RIE;

    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);

    // Enable edma
    hal_freescale_edma_erq_enable(edma_p, dma_chan_rx_i);
     
    cyg_drv_dsr_unlock();
}

// Serial I/O - low level interrupt handler (ISR)
static void       
freescale_ehci_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    ehci_serial_channel *chan = (ehci_serial_channel *)data;
    freescale_ehci_serial_info * const uart_chan = (freescale_ehci_serial_info *)
                                                chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 regval;
    
    cyg_drv_interrupt_unmask(uart_chan->interrupt_num);
}


// Serial I/O - high level interrupt handler (DSR)
static cyg_uint32
freescale_ehci_serial_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) data;
    freescale_ehci_serial_info * const uart_chan = (freescale_ehci_serial_info *)
                                                    chan->dev_priv;
    
    cyg_drv_interrupt_mask(uart_chan->interrupt_num);
    cyg_drv_interrupt_acknowledge(uart_chan->interrupt_num);

    return CYG_ISR_CALL_DSR; // cause the DSR to run
}

// Serial tx dma dsr
static void       
freescale_ehci_tx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    ehci_serial_channel *chan = (ehci_serial_channel *)data;
                                     
    (chan->callbacks->block_sent_handler)();
}

// Serial tx dma isr
static cyg_uint32
freescale_ehci_tx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) data;
    freescale_ehci_serial_info * const uart_chan = (freescale_ehci_serial_info *)
                                                    chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 regval;
    
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t* edma_p = NULL;
    cyg_uint32 dma_chan_tx_i = 0;
    
    dma_set_p = uart_chan->dma_set_p;
    edma_p = dma_set_p->edma_p;
    dma_chan_tx_i = dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].dma_chan_i;
    
    cyg_drv_interrupt_mask(dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].isr_num);
    cyg_drv_interrupt_acknowledge(dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].isr_num);
    
    // Clear edma done requsts are automaticaly disable
    // on the completition of major loop
    hal_freescale_edma_cleardone(edma_p,dma_chan_tx_i);
    
    // Disable TX DMA request from uart
    HAL_READ_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    regval &=  ~CYGHWR_DEV_FREESCALE_UART_C2_TIE;

    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    cyg_drv_interrupt_unmask(dma_set_p->chan_p[EHCI_DMA_CHAN_TX_I].isr_num);

    return CYG_ISR_CALL_DSR;
}

///
// Serial rx dma dsr
static void       
freescale_ehci_rx_dma_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    ehci_serial_channel *chan = (ehci_serial_channel *)data;

    (chan->callbacks->block_received_handler)();
}

// Serial rx dma dsr
static cyg_uint32
freescale_ehci_rx_dma_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) data;
    freescale_ehci_serial_info * const uart_chan = (freescale_ehci_serial_info *)
                                                    chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 regval;
    
    cyghwr_hal_freescale_dma_set_t* dma_set_p;
    cyghwr_hal_freescale_edma_t* edma_p = NULL;
    
    cyg_uint32 dma_chan_rx_i = 0;
    
    dma_set_p = uart_chan->dma_set_p;
    edma_p = dma_set_p->edma_p;
    dma_chan_rx_i = dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].dma_chan_i;
    
    cyg_drv_interrupt_mask(dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].isr_num);
    cyg_drv_interrupt_acknowledge(dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].isr_num);
    
    // Clear edma done requsts are automaticaly disable
    // on the completition of major loop
    hal_freescale_edma_cleardone(edma_p,dma_chan_rx_i);
    
    // Disable RX DMA request from uart
    HAL_READ_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    regval &=  ~CYGHWR_DEV_FREESCALE_UART_C2_RIE;

    HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_C2, regval);
    cyg_drv_interrupt_unmask(dma_set_p->chan_p[EHCI_DMA_CHAN_RX_I].isr_num);

    return CYG_ISR_CALL_DSR;
}
#endif