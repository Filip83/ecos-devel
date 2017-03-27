//==========================================================================
//
//      ser_freescale_uart.c
//
//      Freescale UART Serial I/O Interface Module (interrupt driven)
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
// Contributors:Ilija Kocho <ilijak@siva.com.mk>
// Date:        2011-02-10
// Purpose:     Freescale UART Serial I/O module (interrupt driven version)
// Description:
//
//
//####DESCRIPTIONEND####
//==========================================================================

#include <pkgconf/io_serial.h>
#include <pkgconf/io.h>

#include <cyg/io/io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_arbiter.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/var_io.h>
#include <cyg/hal/var_io_devs.h>
#include <cyg/hal/var_intr.h>
#include <cyg/hal/var_io_clkgat.h>
#include <cyg/hal/var_ccm.h>
#include <cyg/io/devtab.h>
#include <cyg/io/serial.h>
#include <cyg/io/ser_freescale_uart.h>
#include <cyg/infra/diag.h>
//#include <cyg/hal/uart/imx_uart.h> // From SDK.
//#include <cyg/hal/utility/system_util.h> // From SDK
#include <cyg/hal/iomux_define.h> // From SDK
#include <cyg/hal/registers/regsiomuxc.h> // From SDK
#include <cyg/hal/registers/regsuart.h> // From SDK

// Only build this driver for if Freescale UART is needed.
#ifdef CYGPKG_IO_SERIAL_FREESCALE_UARTA

typedef struct uart_pins_s {
    cyg_uint32 rx;
    cyg_uint32 tx;
    cyg_uint32 rts;
    cyg_uint32 cts;
} uart_pins_t;


typedef struct uart_serial_info {
    CYG_ADDRWORD   uart_base;          // Base address of the uart port
    CYG_WORD       interrupt_num;      // NVIC interrupt vector
    cyg_priority_t interrupt_priority; // NVIC interupt priority
    const uart_pins_t *pins_p;         // Rx, Tx, etc.
    cyg_uint32     clock;              // Clock gate
    cyg_bool tx_active;
    cyg_interrupt  interrupt_obj;      // Interrupt object
    cyg_handle_t   interrupt_handle;   // Interrupt handle
} uart_serial_info;

static bool uart_serial_init(struct cyg_devtab_entry * tab);
static bool uart_serial_putc(serial_channel * chan, unsigned char c);
static Cyg_ErrNo uart_serial_lookup(struct cyg_devtab_entry ** tab,
                                    struct cyg_devtab_entry * sub_tab,
                                    const char * name);
static unsigned char uart_serial_getc(serial_channel *chan);
static Cyg_ErrNo uart_serial_set_config(serial_channel *chan, cyg_uint32 key,
                                        const void *xbuf, cyg_uint32 *len);
static void uart_serial_start_xmit(serial_channel *chan);
static void uart_serial_stop_xmit(serial_channel *chan);

// Interrupt servers
static cyg_uint32 uart_serial_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       uart_serial_DSR(cyg_vector_t vector, cyg_ucount32 count,
                                  cyg_addrword_t data);

static SERIAL_FUNS(uart_serial_funs,
                   uart_serial_putc,
                   uart_serial_getc,
                   uart_serial_set_config,
                   uart_serial_start_xmit,
                   uart_serial_stop_xmit);

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

#include <cyg/io/ser_freescale_uart_chan.inl>

#define UART_UFCR_RFDIV    BF_UART_UFCR_RFDIV(4) 
//#define UART_UFCR_RFDIV     UART_UFCR_RFDIV_4
//#define UART_UFCR_RFDIV     UART_UFCR_RFDIV_7

cyg_uint32 uart_get_reffreq(cyg_uint32 instance)
{
    cyg_uint32 div = UART_UFCR_RFDIV;
    cyg_uint32 ret = 0;
    cyg_uint32 freq = CYGHWR_IO_CLOCK(UART_MODULE_CLK(instance));

    if (div == BF_UART_UFCR_RFDIV(4))
        ret = freq / 2;
    else if (div == BF_UART_UFCR_RFDIV(2))
        ret = freq / 4;
    else if (div == BF_UART_UFCR_RFDIV(6))
        ret = freq / 7;

    return ret;
}

//----------------------------------------------------------------------------
// Internal function to actually configure the hardware to desired
// baud rate, etc.
//----------------------------------------------------------------------------
static bool
uart_serial_config_port(serial_channel * chan, cyg_serial_info_t * new_config,
                        bool init)
{
    cyg_uint16 regval;
    cyg_uint32 regval32;
    uart_serial_info * uart_chan = (uart_serial_info *)(chan->dev_priv);
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 baud_rate = select_baud[new_config->baud];

    if(!baud_rate) return false;    // Invalid baud rate selected
    
    // Bring clock to the sevice
    CYGHWR_IO_CLOCK_ENABLE(uart_chan->clock);
    // Configure PORT pins
    CYGHWR_IO_UART_PIN_CONFIG (uart_chan->pins_p->rx);
// TODO: Replace 1.
// Removed when moved to CYGHWR_IO_UART_PIN_CONFIG
//    CYGHWR_IO_UART_PIN_CONFIG_DAISY (CYGHWR_HAL_FREESCALE_PERIPH_UART, CYGHWR_HAL_FREESCALE_PIN_FUN_RX, 1, uart_chan->pins_p->rx);
    CYGHWR_IO_UART_PIN_CONFIG (uart_chan->pins_p->tx);

    if(new_config->word_length != 8)
        return false;

    // Setup in the order found in the IMX6DQ reference manual page 5218
    // with a few modifications from ser_freescale_uart.h from the SDK

    // Wait for any data being transfered.
    regval = 0;
    while (regval == 0) {
        HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UTS, regval);
        regval &= CYGHWR_DEV_FREESCALE_UART_UTS_TXEMPTY;
    }

    // Disable before making any changes to the configuration.
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
    regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR1_UARTEN;
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);

    // Configure FIFOs trigger level to half-full and half-empty.
    // Set referrence divider to div 2
    regval = (CYGHWR_DEV_FREESCALE_UART_UFCR_RXTL & (1 << 0)) |
             (CYGHWR_DEV_FREESCALE_UART_UFCR_RFDIV & (4 << 7)) |
             (CYGHWR_DEV_FREESCALE_UART_UFCR_TXTL & (16 << 10));
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UFCR, regval);

    // Setup one millisecond timer
    // Using SDK code
// TODO: replace 1
    regval32 = uart_get_reffreq(1) / 1000;
    HAL_WRITE_UINT32(uart_base + CYGHWR_DEV_FREESCALE_UART_ONEMS, regval32);

    // Set parity
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);
    switch(new_config->parity) {
    case CYGNUM_SERIAL_PARITY_NONE:
        regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_PREN;
        regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_PROE;
        break;
    case CYGNUM_SERIAL_PARITY_EVEN:
        regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_PREN;
        regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_PROE;
        break;
    case CYGNUM_SERIAL_PARITY_ODD:
        regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_PREN;
        regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_PROE;
        break;
    default: return false;
    }
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set stop bit 1
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_STPB;
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set data size 8
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_WS;
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set no flow control
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_IRTS; // Ignore RTS
    regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_CTSC;// CTS controlled by CTS bit
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set this bit because the manual says it must be set
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR3, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR3_RXDMUXSEL;
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR3, regval);

    // Enable the uart
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR1_UARTEN;
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);

    // Enable transmitter and receiver
    // Software reset
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_TXEN |
        CYGHWR_DEV_FREESCALE_UART_UCR2_RXEN |
        CYGHWR_DEV_FREESCALE_UART_UCR2_SRST;
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set baud rate
    CYGHWR_IO_FREESCALE_UART_BAUD_SET(uart_base, baud_rate);

    // Enable receive interrupt
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
    if(init) {
        regval |= CYGHWR_DEV_FREESCALE_UART_UCR1_RRDYEN;
    }
    HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);

    uart_chan->tx_active = false;

    if(new_config != &chan->config)
        chan->config = *new_config;

    return true;
}

//--------------------------------------------------------------
// Function to initialize the device.  Called at bootstrap time.
//--------------------------------------------------------------
static bool
uart_serial_init(struct cyg_devtab_entry * tab)
{
    serial_channel * chan = (serial_channel *)tab->priv;
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;

    // Really only required for interrupt driven devices
    (chan->callbacks->serial_init)(chan);
    if(chan->out_cbuf.len != 0) {
        cyg_drv_interrupt_create(uart_chan->interrupt_num,
                                 uart_chan->interrupt_priority,
                                 // Data item passed to interrupt handler
                                 (cyg_addrword_t)chan,
                                 uart_serial_ISR,
                                 uart_serial_DSR,
                                 &uart_chan->interrupt_handle,
                                 &uart_chan->interrupt_obj);
       
        // This driver only uses CPU 0 for interrupts.
#ifdef CYGPKG_KERNEL_SMP_SUPPORT
        cyg_drv_interrupt_set_cpu(uart_chan->interrupt_num, 0);
#endif
        cyg_drv_interrupt_attach(uart_chan->interrupt_handle);
        cyg_drv_interrupt_unmask(uart_chan->interrupt_num);
    }
    return uart_serial_config_port(chan, &chan->config, true);
}

//----------------------------------------------------------------------
// This routine is called when the device is "looked" up (i.e. attached)
//----------------------------------------------------------------------
static Cyg_ErrNo
uart_serial_lookup(struct cyg_devtab_entry ** tab,
                   struct cyg_devtab_entry * sub_tab, const char * name)
{
    serial_channel * chan = (serial_channel *)(*tab)->priv;
    // Really only required for interrupt driven devices
    (chan->callbacks->serial_init)(chan);

    return ENOERR;
}

//-----------------------------------------------------------------
// Send a character to Tx
//-----------------------------------------------------------------
static bool
uart_serial_putc(serial_channel * chan, unsigned char ch_out)
{
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint16 uart_sr;

    // Send data if below target threshold.
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_USR1, uart_sr);
    if(uart_sr & CYGHWR_DEV_FREESCALE_UART_USR1_TRDY) {
        HAL_WRITE_UINT8(uart_base + CYGHWR_DEV_FREESCALE_UART_UTXD, ch_out);
        return true;
    } else {
        return false;
    }
}


//---------------------------------------------------------------------
// Fetch a character Rx (for polled operation only)
//---------------------------------------------------------------------
static unsigned char
uart_serial_getc(serial_channel * chan)
{
    cyg_uint8 ch_in;
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;

    cyg_uint32 uart_sr;

    do {
        HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_USR2, uart_sr);
    } while(uart_sr & CYGHWR_DEV_FREESCALE_UART_USR2_RDR);

    // Read character and status
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_URXD, ch_in);

    return (unsigned char) (ch_in & 0xFF);
}


//---------------------------------------------------
// Set up the device characteristics; baud rate, etc.
//---------------------------------------------------
static Cyg_ErrNo
uart_serial_set_config(serial_channel * chan, cyg_uint32 key,
                       const void *xbuf, cyg_uint32 * len)
{
    switch(key) {
    case CYG_IO_SET_CONFIG_SERIAL_INFO: {
            cyg_serial_info_t *config = (cyg_serial_info_t *)xbuf;
            if(*len < sizeof(cyg_serial_info_t)) {
                return -EINVAL;
            }
            *len = sizeof(cyg_serial_info_t);
            if(true != uart_serial_config_port(chan, config, false))
            return -EINVAL;
        }
        break;
    default:
        return -EINVAL;
    }
    return ENOERR;
}

//-------------------------------------
// Enable the transmitter on the device
//-------------------------------------
static void uart_serial_start_xmit(serial_channel * chan)
{
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint16 uart_cr1;

    if(!uart_chan->tx_active) {
        uart_chan->tx_active = true;
        // Send interrupts if there is room in the FIFO.
        HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, uart_cr1);
        uart_cr1 |= CYGHWR_DEV_FREESCALE_UART_UCR1_TRDYEN;
        HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, uart_cr1);
    }
}

//--------------------------------------
// Disable the transmitter on the device
//--------------------------------------
static void uart_serial_stop_xmit(serial_channel * chan)
{
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;

    cyg_addrword_t uart_base = uart_chan->uart_base;
    cyg_uint32 uart_cr1;

    if(uart_chan->tx_active) {
        uart_chan->tx_active = false;
        // Stop interrupts.
        HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, uart_cr1);
        uart_cr1 &= ~(CYGHWR_DEV_FREESCALE_UART_UCR1_TRDYEN);
        HAL_WRITE_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_UCR1, uart_cr1);
    }
}

//-----------------------------------------
// The low level interrupt handler
//-----------------------------------------
static
cyg_uint32 uart_serial_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    serial_channel * chan = (serial_channel *)data;
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;

    cyg_drv_interrupt_mask(uart_chan->interrupt_num);
    cyg_drv_interrupt_acknowledge(uart_chan->interrupt_num);

    return CYG_ISR_CALL_DSR | CYG_ISR_HANDLED; // cause the DSR to run
}


//------------------------------------------
// The high level interrupt handler
//------------------------------------------

#define CYGHWR_DEV_FREESCALE_UART_S1_ERRORS \
                                 (CYGHWR_DEV_FREESCALE_UART_S1_OR | \
                                  CYGHWR_DEV_FREESCALE_UART_S1_NF | \
                                  CYGHWR_DEV_FREESCALE_UART_S1_FE | \
                                  CYGHWR_DEV_FREESCALE_UART_S1_PF)

static void
uart_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    serial_channel * chan = (serial_channel *)data;
    uart_serial_info * uart_chan = (uart_serial_info *)chan->dev_priv;
    cyg_addrword_t uart_base = uart_chan->uart_base;
    volatile cyg_uint32 uart_sr1;
    volatile cyg_uint32 uart_sr2;
    volatile cyg_uint16 uart_dr;

    // TODO: Do we have to enable some interrupt for RECV?

    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_USR1, uart_sr1);
    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_USR2, uart_sr2);
    if(((uart_sr1 & (CYGHWR_DEV_FREESCALE_UART_USR1_RRDY |
                  CYGHWR_DEV_FREESCALE_UART_USR1_PARITYERR |
                  CYGHWR_DEV_FREESCALE_UART_USR1_FRAMERR)) > 0) ||
       ((uart_sr2 & CYGHWR_DEV_FREESCALE_UART_USR2_ORE) > 0)) {
        // Receiver with data or errors
        HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_URXD, uart_dr);
        if(((uart_sr1 & (CYGHWR_DEV_FREESCALE_UART_USR1_PARITYERR |
           CYGHWR_DEV_FREESCALE_UART_USR1_FRAMERR)) > 0) ||
           ((uart_sr2 & CYGHWR_DEV_FREESCALE_UART_USR2_ORE)) > 0) {
            // Check for receive error
        } else { // No errors, get the character
            (chan->callbacks->rcv_char)(chan, (cyg_uint8)(uart_dr & 0xFF));
        }
    }
    // Transmitter FIFO below threshold.
    if(uart_chan->tx_active && (uart_sr1 & CYGHWR_DEV_FREESCALE_UART_USR1_TRDY)){
        (chan->callbacks->xmt_char)(chan);
    }

    HAL_READ_UINT16(uart_base + CYGHWR_DEV_FREESCALE_UART_USR1, uart_sr1);

    cyg_drv_interrupt_unmask(uart_chan->interrupt_num);
}

#endif // CYGPKG_IO_SERIAL_FREESCALE_UARTA
// EOF ser_freescale_uart.c
