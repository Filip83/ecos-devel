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

#include <cyg/io/io.h>
#include <cyg/hal/hal_io.h>
#include <cyg/io/ehci_serial.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/io/devtab.h>
#include <cyg/io/ehci_serial.h>
#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/pdca.h>
#include <cyg/hal/board_config.h>

#if 1
#ifndef CYGPKG_IO_EHCI_SERIAL_AVR32
#define CYGPKG_IO_EHCI_SERIAL_AVR32

#include "avr32_ehci_serial.h"

#define RCV_TIMEOUT         0


typedef struct avr32_ehci_serial_info {
    volatile                        avr32_usart_t  *usart_dev;
    CYG_WORD                        int_num;
    CYG_WORD                        int_num_dma_rx;
    CYG_WORD                        int_num_dma_tx;
    cyg_interrupt                   serial_interrupt;
    cyg_handle_t                    serial_interrupt_handle;

    cyg_interrupt                   serial_dma_tx_interrupt;
    cyg_handle_t                    serial_dma_tx_interrupt_handle;
    cyg_uint8                       dma_tx_ch;
    cyg_uint8                       dma_pid_tx;
    volatile avr32_pdca_channel_t   *pdca_tx_channel;
    cyg_bool                        tx_trans_rdy;

    cyg_interrupt                   serial_dma_rx_interrupt;
    cyg_handle_t                    serial_dma_rx_interrupt_handle;
    cyg_uint8                       dma_rx_ch;
    cyg_uint8                       dma_pid_rx;
    volatile avr32_pdca_channel_t   *pdca_rx_channel;
    cyg_bool                        rx_trans_rdy;
} avr32_ehci_serial_info;

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

static avr32_ehci_serial_info avr32_ehci_serial_info_s = {
#if defined(CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL0)
    usart_dev       : (avr32_usart_t*) AVR32_USART0_ADDRESS,
#elif defined(CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL1)
    usart_dev       : (avr32_usart_t*) AVR32_USART1_ADDRESS,
#elif defined(CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL2)
    usart_dev       : (avr32_usart_t*) AVR32_USART2_ADDRESS,
#elif defined(CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL3)
    usart_dev       : (avr32_usart_t*) AVR32_USART3_ADDRESS,
#else
    #ifdef AVR32_USART4_ADDRESS
    usart_dev       : (avr32_usart_t*) AVR32_USART4_ADDRESS,
    #endif
#endif
    int_num         : CYGNUM_HAL_VECTOR_USART3,
    int_num_dma_rx  : CYGNUM_HAL_VECTOR_PDMA_CH8,
    dma_rx_ch       : 8,
    dma_pid_rx      : AVR32_PDCA_PID_USART3_RX,
    int_num_dma_tx  : CYGNUM_HAL_VECTOR_PDMA_CH5,
    dma_pid_tx      : AVR32_PDCA_PID_USART3_TX,
    dma_tx_ch       : 5
};

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
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *)chan->dev_priv;

    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyg_uint32 parity = select_parity[new_config->parity];
    cyg_uint32 word_length = 
        select_word_length[new_config->word_length-CYGNUM_SERIAL_WORD_LENGTH_5];
    cyg_uint32 stop_bits = select_stop_bits[new_config->stop - 1]; 
    CYG_ADDRESS usart_addr = (CYG_ADDRESS)avr32_chan->usart_dev;

    if ((word_length == 0xFF) ||
        (parity == 0xFF) ||
        (stop_bits == 0xFF)) {
        return false;  // Unsupported configuration
    }

    if(usart_addr == AVR32_USART0_ADDRESS)
    {
#ifdef CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL0

        gpio_enable_module_pin(CYG_HAL_USART0_TXD_PIN,
                CYG_HAL_USART0_TXD_FUNCTION);

        gpio_enable_module_pin(CYG_HAL_USART0_RXD_PIN, 
                CYG_HAL_USART0_RXD_FUNCTION);

    #ifdef CYGNUM_IO_EHCI_SERIAL_AVR32_SERIAL0_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART0_RTS_PIN, 
                CYG_DEVS_USART0_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART0_CTS_PIN, 
                CYG_DEVS_USART0_CTS_FUNCTION);
    #endif
#endif
    }
    else if(usart_addr == AVR32_USART1_ADDRESS)		
    {
#ifdef CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL1
        gpio_enable_module_pin(CYG_HAL_USART1_TXD_PIN,
                CYG_HAL_USART1_TXD_FUNCTION);

        gpio_enable_module_pin(CYG_HAL_USART1_RXD_PIN, 
                CYG_HAL_USART1_RXD_FUNCTION);

    #ifdef CYGNUM_IO_EHCI_SERIAL_AVR32_SERIAL1_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART1_RTS_PIN, 
                CYG_DEVS_USART1_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART1_CTS_PIN, 
                CYG_DEVS_USART1_CTS_FUNCTION);
    #endif
#endif
    }		
    else if(usart_addr == AVR32_USART2_ADDRESS)
    {
#ifdef CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL2
        gpio_enable_module_pin(CYG_HAL_USART2_TXD_PIN,
                CYG_HAL_USART2_TXD_FUNCTION);

        gpio_enable_module_pin(CYG_HAL_USART2_RXD_PIN, 
                CYG_HAL_USART2_RXD_FUNCTION);

    #ifdef CYGNUM_IO_EHCI_SERIAL_AVR32_SERIAL2_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART2_RTS_PIN, 
                CYG_DEVS_USART2_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART2_CTS_PIN, 
                CYG_DEVS_USART2_CTS_FUNCTION);
    #endif
#endif
    }	
    else if(usart_addr == AVR32_USART3_ADDRESS)
    {
#ifdef CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL3
        gpio_enable_module_pin(CYG_HAL_USART3_TXD_PIN,
                CYG_HAL_USART3_TXD_FUNCTION);

        gpio_enable_module_pin(CYG_HAL_USART3_RXD_PIN, 
                CYG_HAL_USART3_RXD_FUNCTION);


        gpio_enable_module_pin(CYG_HAL_USART3_RTS_PIN, 
                CYG_HAL_USART3_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_USART3_CTS_PIN, 
                CYG_HAL_USART3_CTS_FUNCTION);
#endif
    }	
#ifdef AVR32_USART4_ADDRESS
    else if(usart_addr == AVR32_USART4_ADDRESS)
    {
#ifdef CYGPKG_IO_EHCI_SERIAL_AVR32_SERIAL4
        gpio_enable_module_pin(CYG_HAL_USART4_TXD_PIN,
                CYG_HAL_USART4_TXD_FUNCTION);

        gpio_enable_module_pin(CYG_HAL_USART4_RXD_PIN, 
                CYG_HAL_USART4_RXD_FUNCTION);

    #ifdef CYGNUM_IO_EHCI_SERIAL_AVR32_SERIAL4_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART4_RTS_PIN, 
                CYG_DEVS_USART4_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART4_CTS_PIN, 
                CYG_DEVS_USART4_CTS_FUNCTION);
    #endif
#endif
    }
#endif
    else
    {
        CYG_ASSERT(false,"Unknownd USART address\n");
    }
    // Reset device
    dev->cr = AVR32_USART_CR_RSTRX_MASK | 
              AVR32_USART_CR_RSTTX_MASK | 
              AVR32_USART_CR_RSTSTA_MASK ;

    // Configuration
    if((new_config->flags & CYGNUM_SERIAL_FLOW_RTSCTS_RX) ||
        (new_config->flags & CYGNUM_SERIAL_FLOW_RTSCTS_TX))
    {
        dev->mr = stop_bits | parity | word_length /*| 
                AVR32_USART_MODE_HARDWARE /*| AVR32_USART_MR_OVER_MASK*/;
        dev->cr = AVR32_USART_CR_RTSDIS_MASK;
    }
    else if((new_config->flags & CYGNUM_SERIAL_FLOW_DSRDTR_RX) ||
        (new_config->flags & CYGNUM_SERIAL_FLOW_DSRDTR_TX))
        dev->mr = stop_bits | parity | word_length | 
                AVR32_USART_MODE_MODEM /*| AVR32_USART_MR_OVER_MASK*/;
    else
    {
        dev->mr = stop_bits | parity | word_length | 
                AVR32_USART_MODE_NORMAL /*| AVR32_USART_MR_OVER_MASK*/;
	dev->cr = AVR32_USART_CR_RTSEN_MASK;
    }
 
    // Baud rate
    if(new_config->baud < sizeof(select_baud))
        dev->brgr = select_baud[new_config->baud];
    else
        dev->brgr = DIVISOR(new_config->baud);
    // Disable all interrupts
    dev->idr = 0xFFFFFFFF;

    
    avr32_chan->pdca_tx_channel = CYG_PDMA_GET_CHANEL(avr32_chan->dma_tx_ch);
    avr32_chan->pdca_rx_channel = CYG_PDMA_GET_CHANEL(avr32_chan->dma_rx_ch);



    dev->rtor = RCV_TIMEOUT;

#if 0
    dev->ier  = /*AVR32_USART_RXRDY_MASK |*/ AVR32_USART_OVER_MASK |
                  AVR32_USART_FRAME_MASK | AVR32_USART_PARE_MASK   |
                  AVR32_USART_RXBRK_MASK | AVR32_USART_TIMEOUT_MASK;
#endif

    dev->ier  = /*AVR32_USART_RXRDY_MASK |*/ AVR32_USART_IER_OVRE_MASK |
                  AVR32_USART_IER_FRAME_MASK | AVR32_USART_IER_PARE_MASK   |
                  AVR32_USART_IER_RXBRK_MASK ;
    //dev->ier = AVR32_USART_CTSIC_MASK;
    
    // Enable RX and TX
    dev->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK |
              AVR32_USART_RTSEN_MASK;
				  
     dev->cr = AVR32_USART_CR_RTSDIS_MASK;      
    if (new_config != &chan->config) {
        chan->config = *new_config;
    }

    CYG_PDAM_SET_CHANEL(avr32_chan->pdca_rx_channel,
                             0, 0,
                             avr32_chan->dma_pid_rx,
                             0,
                             0,
                             0);
    
    CYG_PDAM_SET_CHANEL(avr32_chan->pdca_tx_channel,0,
                  0,avr32_chan->dma_pid_tx,0,0,0);
    
    CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_rx_channel);
    
    CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_tx_channel);

    return true;
}

// Function to initialize the device.  Called at bootstrap time.
static bool 
avr32_ehci_serial_init(struct cyg_devtab_entry *tab)
{
    ehci_serial_channel * const chan = (ehci_serial_channel *) tab->priv;
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
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


    //rx pdma interrupt
    cyg_drv_interrupt_create(avr32_chan->int_num_dma_rx,
                             3,                      // Priority
                             (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                             avr32_ehci_serial_rx_dma_ISR,
                             avr32_ehci_serial_rx_dma_DSR,
                             &avr32_chan->serial_dma_rx_interrupt_handle,
                             &avr32_chan->serial_dma_rx_interrupt);
    cyg_drv_interrupt_attach(avr32_chan->serial_dma_rx_interrupt_handle);

    //tx pdma interrupt
    cyg_drv_interrupt_create(avr32_chan->int_num_dma_tx,
                             1,                      // Priority
                             (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                             avr32_ehci_serial_tx_dma_ISR,
                             avr32_ehci_serial_tx_dma_DSR,
                             &avr32_chan->serial_dma_tx_interrupt_handle,
                             &avr32_chan->serial_dma_tx_interrupt);
    cyg_drv_interrupt_attach(avr32_chan->serial_dma_tx_interrupt_handle);
     

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
    gpio_set_pin_high(BT_NSHUTD_PIN);
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
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;

    cyg_drv_dsr_lock();
    //diag_printf("Start transmit\n");
   /* if(CYG_PDMA_IS_ENABLED(avr32_chan->pdca_tx_channel))
    {
        CYG_FAIL("EHCI TX ENABLED\n");
    }*/

    CYG_PDAM_SET_CHANEL(avr32_chan->pdca_tx_channel,buffer,
                  len,avr32_chan->dma_pid_tx,0,0,0);
    CYG_PDMA_ENABLE_INTERRUPT(avr32_chan->pdca_tx_channel,true,
      true, false);
    //CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_tx_channel);

                
    cyg_drv_dsr_unlock();
}
char buf[64];
int  inaction = 0;
static void
avr32_ehci_serial_start_recive(ehci_serial_channel *chan, const cyg_uint8 *buffer,
                                cyg_uint16 len)
{
    avr32_ehci_serial_info * const avr32_chan = (avr32_ehci_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;

    cyg_drv_dsr_lock();
    //diag_printf("Rec %d\n",len);
    //dev->cr = AVR32_USART_CR_RXEN_MASK;
   /* if(CYG_PDMA_IS_ENABLED(avr32_chan->pdca_rx_channel))
    {*/
       /* if(dev->csr & AVR32_USART_CSR_RXRDY_MASK)
        {
            diag_printf("je, %d, 0x%x\n",len,dev->rhr);
            //while(1);
        }*/

        CYG_PDMA_RELOAD(avr32_chan->pdca_rx_channel,
                             buffer, len);
        
        CYG_PDMA_ENABLE_INTERRUPT(avr32_chan->pdca_rx_channel,true,
                                     true, false);
        
        dev->cr = AVR32_USART_CR_RTSEN_MASK;
    
   /* }
    else
    {
        CYG_PDAM_SET_CHANEL(avr32_chan->pdca_rx_channel,
                             buffer, len,
                             avr32_chan->dma_pid_rx,
                             0,
                             0,
                             0);


         CYG_PDMA_ENABLE_INTERRUPT(avr32_chan->pdca_rx_channel,true,
                                     true, false);

         CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_rx_channel);
    }*/
    
                
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