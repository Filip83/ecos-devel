/*=============================================================================
//
//      hal_diag.c
//
//      HAL diagnostic output code
//
//=============================================================================
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   Ilija Kocho <ilijak@siva.com.mk>
// Contributors:Mike Jones <mike@proclivis.com>
// Date:        2013-08-08
// Purpose:     HAL diagnostic input/output
// Description: Implementations of HAL diagnostic input/output support.
//
//####DESCRIPTIONEND####
//
//===========================================================================
 */

#include <pkgconf/hal.h>
#include CYGBLD_HAL_PLATFORM_H

#include <cyg/infra/cyg_type.h>         // base types

#include <cyg/hal/hal_arch.h>           // SAVE/RESTORE GP macros
#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_if.h>             // interface API
#include <cyg/hal/hal_intr.h>           // HAL_ENABLE/MASK/UNMASK_INTERRUPTS
#include <cyg/hal/hal_misc.h>           // Helper functions
#include <cyg/hal/drv_api.h>            // CYG_ISR_HANDLED
#include <cyg/hal/hal_diag.h>

#include <cyg/hal/var_io.h>             //
#include <cyg/hal/var_io_devs.h>        //
#include <cyg/io/ser_freescale_uart.h>  // UART registers
#include <cyg/hal/iomux_define.h> // From SDK
#include <cyg/hal/registers/regsiomuxc.h> // From SDK

//-----------------------------------------------------------------------------


typedef struct {
    cyg_uint32 uart;
    CYG_ADDRESS base;
    cyg_int32 msec_timeout;
    cyg_int32 isr_vector;
    cyg_uint32 rx_pin;
    cyg_uint32 tx_pin;
    cyg_uint32 clock_gate;
    cyg_int32 baud_rate;
    cyg_int32 irq_state;
} channel_data_t;

channel_data_t plf_ser_channels[] = {
#ifdef CYGINT_HAL_FREESCALE_UART0
    { 0, CYGADDR_IO_SERIAL_FREESCALE_UART0_BASE, 1000,
      CYGNUM_HAL_INTERRUPT_UART0,
      CYGHWR_HAL_IMX6_UART0_PIN_RX, CYGHWR_HAL_IMX6_UART0_PIN_TX,
      CYGHWR_HAL_FREESCALE_UART0_CLOCK, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD,
      1 },
#endif
#ifdef CYGINT_HAL_FREESCALE_UART1
    { 1, CYGADDR_IO_SERIAL_FREESCALE_UART1_BASE, 1000,
      CYGNUM_HAL_INTERRUPT_UART1,
      CYGHWR_HAL_IMX6_UART1_PIN_RX, CYGHWR_HAL_IMX6_UART1_PIN_TX,
      CYGHWR_HAL_FREESCALE_UART1_CLOCK, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD,
      1 },
#endif
#ifdef CYGINT_HAL_FREESCALE_UART2
    { 2, CYGADDR_IO_SERIAL_FREESCALE_UART2_BASE, 1000,
      CYGNUM_HAL_INTERRUPT_UART2,
      CYGHWR_HAL_IMX6_UART2_PIN_RX, CYGHWR_HAL_IMX6_UART2_PIN_TX,
      CYGHWR_HAL_FREESCALE_UART2_CLOCK, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD,
      1 },
#endif
#ifdef CYGINT_HAL_FREESCALE_UART3
    { 3, CYGADDR_IO_SERIAL_FREESCALE_UART3_BASE, 1000,
      CYGNUM_HAL_INTERRUPT_UART3,
      CYGHWR_HAL_IMX6_UART3_PIN_RX, CYGHWR_HAL_IMX6_UART3_PIN_TX,
      CYGHWR_HAL_FREESCALE_UART3_CLOCK, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD,
      1 },
#endif
#ifdef CYGINT_HAL_FREESCALE_UART4
    { 4, CYGADDR_IO_SERIAL_FREESCALE_UART4_BASE, 1000,
      CYGNUM_HAL_INTERRUPT_UART3,
      CYGHWR_HAL_IMX6_UART4_PIN_RX, CYGHWR_HAL_IMX6_UART4_PIN_TX,
      CYGHWR_HAL_FREESCALE_UART4_CLOCK, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD,
      1 },
#endif
};

//-----------------------------------------------------------------------------

void
cyg_hal_plf_serial_putc(void *__ch_data, char c);


static void
cyg_hal_plf_serial_init_channel(void* __ch_data)
{
    cyg_uint16 regval;
    cyg_uint32 regval32;
    channel_data_t* chan = (channel_data_t*)__ch_data;
    CYG_ADDRESS uart_p = chan->base;

   // Bring clock to the device
   CYGHWR_IO_CLOCK_ENABLE(chan->clock_gate);
   // Configure PORT pins
    CYGHWR_IO_UART_PIN_CONFIG (chan->rx_pin);
// TODO: Don't hard code to 1
    CYGHWR_IO_UART_PIN_CONFIG_DAISY (CYGHWR_HAL_FREESCALE_PERIPH_UART, CYGHWR_HAL_FREESCALE_PIN_FUN_RX, 1, chan->rx_pin);
    CYGHWR_IO_UART_PIN_CONFIG (chan->tx_pin);

    // Setup in the order found in the IMX6DQ reference manual page 5218
    // with a few modifications from ser_imx6_uart.h from the SDK

    // Wait for any data being transfered.
    regval = 0;
    while (regval == 0) {
        HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UTS, regval);
        regval &= CYGHWR_DEV_FREESCALE_UART_UTS_TXEMPTY;
    }

    // Disable before making any changes to the configuration.
    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
    regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR1_UARTEN;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);

    // Configure FIFOs trigger level to half-full and half-empty.
    // Set referrence divider to div 2
    regval = (CYGHWR_DEV_FREESCALE_UART_UFCR_RXTL & (1 << 0)) |
             (CYGHWR_DEV_FREESCALE_UART_UFCR_RFDIV & (4 << 7)) |
             (CYGHWR_DEV_FREESCALE_UART_UFCR_TXTL & (16 << 10));
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UFCR, regval);

    // Setup one millisecond timer
    // Using SDK code
// TODO: Don't hard code to 1
    regval32 = uart_get_reffreq(1) / 1000;
    HAL_WRITE_UINT32(uart_p + CYGHWR_DEV_FREESCALE_UART_ONEMS, regval32);

    // No parity
    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);
    regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_PREN;
    regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_PROE;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, 0);

     // Set stop bit 1
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_STPB;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set data size 8
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_WS;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set no flow control
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_IRTS; // Ignore RTS
    regval &= ~CYGHWR_DEV_FREESCALE_UART_UCR2_CTSC;// CTS controlled by CTS bit
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set this bit because the manual says it must be set
    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR3, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR3_RXDMUXSEL;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR3, regval);

    // Enable the uart
    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR1_UARTEN;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);

    // Enable transmitter and receiver
    // Software reset
    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR2_TXEN |
        CYGHWR_DEV_FREESCALE_UART_UCR2_RXEN |
        CYGHWR_DEV_FREESCALE_UART_UCR2_SRST;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR2, regval);

    // Set baud rate
    CYGHWR_IO_FREESCALE_UART_BAUD_SET(uart_p, chan->baud_rate);

    // Enable receive interrupt
    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
    regval |= CYGHWR_DEV_FREESCALE_UART_UCR1_RRDYEN;
    HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, regval);
}

void
cyg_hal_plf_serial_putc(void* __ch_data, char ch_out)
{
    channel_data_t* chan = (channel_data_t*)__ch_data;
    CYG_ADDRESS uart_p = (CYG_ADDRESS) chan->base;
    cyg_uint32 uart_s1;
    cyg_uint32 timeout = 0;

//    while (uart_p != 0x2020000);

    CYGARC_HAL_SAVE_GP();

    do {
       HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_USR1, uart_s1);
       timeout++;
       if (timeout > 1000)
       {
    	    CYGARC_HAL_RESTORE_GP();
    	    return;
       }
    } while (!(uart_s1 & CYGHWR_DEV_FREESCALE_UART_USR1_TRDY));

    HAL_WRITE_UINT8(uart_p + CYGHWR_DEV_FREESCALE_UART_UTXD, ch_out);

    CYGARC_HAL_RESTORE_GP();
}

static cyg_bool
cyg_hal_plf_serial_getc_nonblock(void* __ch_data, cyg_uint8* p_ch_in)
{
    channel_data_t* chan = (channel_data_t*)__ch_data;
    CYG_ADDRESS uart_p = (CYG_ADDRESS) chan->base;
    cyg_uint8 uart_s1;
    cyg_uint16 ch_in;

    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_USR2, uart_s1);
    if (!(uart_s1 & CYGHWR_DEV_FREESCALE_UART_USR2_RDR))
        return false;

    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_URXD, ch_in);;
    *p_ch_in = (cyg_uint8) (ch_in & 0xFF);

    return true;
}

cyg_uint8
cyg_hal_plf_serial_getc(void* __ch_data)
{
    cyg_uint8 ch;
    CYGARC_HAL_SAVE_GP();

    while(!cyg_hal_plf_serial_getc_nonblock(__ch_data, &ch));

    CYGARC_HAL_RESTORE_GP();
    return ch;
}


//=============================================================================
// Virtual vector HAL diagnostics

#if defined(CYGSEM_HAL_VIRTUAL_VECTOR_DIAG)

static void
cyg_hal_plf_serial_write(void* __ch_data, const cyg_uint8* __buf,
                         cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    while(__len-- > 0)
        cyg_hal_plf_serial_putc(__ch_data, *__buf++);

    CYGARC_HAL_RESTORE_GP();
}

static void 
cyg_hal_plf_serial_read(void* __ch_data, cyg_uint8* __buf, cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    while(__len-- > 0)
        *__buf++ = cyg_hal_plf_serial_getc(__ch_data);

    CYGARC_HAL_RESTORE_GP();
}

cyg_bool
cyg_hal_plf_serial_getc_timeout(void* __ch_data, cyg_uint8* p_ch_in)
{
    int delay_count;
    cyg_bool res;
    CYGARC_HAL_SAVE_GP();

    // delay in .1 ms steps
    delay_count = ((channel_data_t*)__ch_data)->msec_timeout * 10;

    for(;;) {
        res = cyg_hal_plf_serial_getc_nonblock(__ch_data, p_ch_in);
        if (res || 0 == delay_count--)
            break;

        CYGACC_CALL_IF_DELAY_US(100);
    }

    CYGARC_HAL_RESTORE_GP();
    return res;
}

static int
cyg_hal_plf_serial_control(void *__ch_data, __comm_control_cmd_t __func, ...)
{
    channel_data_t* chan = (channel_data_t*)__ch_data;
    CYG_ADDRESS uart_p = ((channel_data_t*)__ch_data)->base;
    cyg_uint16 ser_port_reg;
    int ret = 0;
    va_list ap;

    CYGARC_HAL_SAVE_GP();
    va_start(ap, __func);

    switch (__func) {
    case __COMMCTL_IRQ_ENABLE:
        chan->irq_state = 1;
        HAL_INTERRUPT_ACKNOWLEDGE(chan->isr_vector);
        HAL_INTERRUPT_UNMASK(chan->isr_vector);

        HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, ser_port_reg);
        ser_port_reg |= CYGHWR_DEV_FREESCALE_UART_UCR1_RRDYEN;
        HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, ser_port_reg);

        break;
    case __COMMCTL_IRQ_DISABLE:
        ret = chan->irq_state;
        chan->irq_state = 0;
        HAL_INTERRUPT_MASK(chan->isr_vector);

        HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, ser_port_reg);
        ser_port_reg &= ~(cyg_uint16)CYGHWR_DEV_FREESCALE_UART_UCR1_RRDYEN;
        HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UCR1, ser_port_reg);
        break;
    case __COMMCTL_DBG_ISR_VECTOR:
        ret = chan->isr_vector;
        break;
    case __COMMCTL_SET_TIMEOUT:
        ret = chan->msec_timeout;
        chan->msec_timeout = va_arg(ap, cyg_uint32);
    case __COMMCTL_GETBAUD:
        ret = chan->baud_rate;
        break;
    case __COMMCTL_SETBAUD:
        chan->baud_rate = va_arg(ap, cyg_int32);
        // Should we verify this value here?
        cyg_hal_plf_serial_init_channel(chan);
        ret = 0;
        break;
    default:
        break;
    }

    va_end(ap);
    CYGARC_HAL_RESTORE_GP();
    return ret;
}

static int
cyg_hal_plf_serial_isr(void *__ch_data, int* __ctrlc,
                       CYG_ADDRWORD __vector, CYG_ADDRWORD __data)
{

    channel_data_t* chan = (channel_data_t*)__ch_data;
//    CYG_ADDRESS uart_p = (CYG_ADDRESS) chan->base;
//    cyg_uint8 uart_s1;
//    cyg_uint8 uart_s2;
//    cyg_uint16 uart_dr;
    int res = 0;
//    cyg_uint8 ch_in;
    CYGARC_HAL_SAVE_GP();

    *__ctrlc = 0;

// TODO: Does not really belong here. This is a debugger interrupt.
//    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_USR1, uart_s1);
//    HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_USR2, uart_s2);
//    if(((uart_s1 & (CYGHWR_DEV_FREESCALE_UART_USR1_RRDY |
//                  CYGHWR_DEV_FREESCALE_UART_USR1_PARITYERR |
//                  CYGHWR_DEV_FREESCALE_UART_USR1_FRAMERR)) > 0) ||
//       ((uart_s2 & CYGHWR_DEV_FREESCALE_UART_USR2_ORE) > 0)) {
//        // Receiver with data or errors
//        HAL_READ_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_URXD, uart_dr);
//        if(((uart_s1 & (CYGHWR_DEV_FREESCALE_UART_USR1_PARITYERR |
//           CYGHWR_DEV_FREESCALE_UART_USR1_FRAMERR)) >0) ||
//           ((uart_s2 & CYGHWR_DEV_FREESCALE_UART_USR2_ORE)) > 0) {
//            // Check for receive error
//        } else { // No errors, get the character
//            ch_in = (cyg_uint8)(uart_dr & 0xFF);
//            if( cyg_hal_is_break( (char *) &ch_in , 1 ) )
//                *__ctrlc = 1;
//
//            res = CYG_ISR_HANDLED;
//        }
//    }
    res = CYG_ISR_HANDLED;

    HAL_INTERRUPT_ACKNOWLEDGE(chan->isr_vector);

    CYGARC_HAL_RESTORE_GP();
    return res;
}

static void
cyg_hal_plf_serial_init(void)
{
    hal_virtual_comm_table_t* comm;
    int cur;
    int chan_i;

    cur = CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT);

    // Init channels
    for(chan_i=0; chan_i<CYGNUM_HAL_VIRTUAL_VECTOR_COMM_CHANNELS; chan_i++) {
        cyg_hal_plf_serial_init_channel(&plf_ser_channels[chan_i]);

        // Setup procs in the vector table
        CYGACC_CALL_IF_SET_CONSOLE_COMM(chan_i);
        comm = CYGACC_CALL_IF_CONSOLE_PROCS();
        CYGACC_COMM_IF_CH_DATA_SET(*comm, &plf_ser_channels[chan_i]);
        CYGACC_COMM_IF_WRITE_SET(*comm, cyg_hal_plf_serial_write);
        CYGACC_COMM_IF_READ_SET(*comm, cyg_hal_plf_serial_read);
        CYGACC_COMM_IF_PUTC_SET(*comm, cyg_hal_plf_serial_putc);
        CYGACC_COMM_IF_GETC_SET(*comm, cyg_hal_plf_serial_getc);
        CYGACC_COMM_IF_CONTROL_SET(*comm, cyg_hal_plf_serial_control);
        CYGACC_COMM_IF_DBG_ISR_SET(*comm, cyg_hal_plf_serial_isr);
        CYGACC_COMM_IF_GETC_TIMEOUT_SET(*comm, cyg_hal_plf_serial_getc_timeout);
    }
    // Restore original console
    CYGACC_CALL_IF_SET_CONSOLE_COMM(cur);
#if (CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD != CYGNUM_HAL_VIRTUAL_VECTOR_DEBUG_CHANNEL_BAUD)
    plf_ser_channels[CYGNUM_HAL_VIRTUAL_VECTOR_DEBUG_CHANNEL].baud_rate =
        CYGNUM_HAL_VIRTUAL_VECTOR_DEBUG_CHANNEL_BAUD;
// TODO: Where is this implemented? I can't even find the cortex/kinetis version of this.
//    update_baud_rate( &plf_ser_channels[CYGNUM_HAL_VIRTUAL_VECTOR_DEBUG_CHANNEL] );
#endif
}

void
cyg_hal_plf_comms_init(void)
{
    static int initialized = 0;

    if (initialized)
        return;
    initialized = 1;
    cyg_hal_plf_serial_init();
}

#else // !defined(CYGSEM_HAL_VIRTUAL_VECTOR_DIAG)
//=============================================================================
// Non-Virtual vector HAL diagnostics

// #if !defined(CYGSEM_HAL_VIRTUAL_VECTOR_DIAG)

void hal_plf_diag_init(void)
{
    cyg_hal_plf_serial_init( &plf_ser_channels[CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL] );
}

void hal_plf_diag_putc(char c)
{
    cyg_hal_plf_serial_putc( &plf_ser_channels[CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL], c);
}

cyg_uint8 hal_plf_diag_getc(void)
{
    return cyg_hal_plf_serial_getc( &plf_ser_channels[CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL] );
}

#endif // defined(CYGSEM_HAL_VIRTUAL_VECTOR_DIAG)

//-----------------------------------------------------------------------------
// End of hal_diag.c
