//=============================================================================
//
//      hal_diag.c
//
//      Simple polling driver for the AVR32UC3C serial controller(s),
//      to be used for diagnostic I/O and gdb remote debugging.
//
//=============================================================================
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   Filip
// Contributors:
// Original data: sfurman
// Date:        2014-04-28
// Description: Simple polling driver for the AVR32 serial controller(s)
//               in the ORP, to be used for diagnostic I/O and gdb remote 
//               debugging.
//      
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>
#include <pkgconf/hal.h>
#include <pkgconf/system.h>
#include CYGBLD_HAL_PLATFORM_H

#include <cyg/hal/hal_arch.h>           // SAVE/RESTORE GP macros
#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_if.h>             // interface API
#include <cyg/hal/hal_intr.h>           // HAL_ENABLE/MASK/UNMASK_INTERRUPTS
#include <cyg/hal/hal_misc.h>           // Helper functions
#include <cyg/hal/drv_api.h>            // CYG_ISR_HANDLED
#include <cyg/infra/cyg_ass.h>          // assertion macros
#include <cyg/hal/board_config.h>       // pin multipex settings
 
// Assume the UART is driven 1/16 CPU frequency
#define UART_CLOCK    ((CYGHWR_HAL_AVR32_CPU_FREQ)*1.0e6)

#define DIVISOR(baud) ((int)((UART_CLOCK)/(16.0*baud)+0.5))


#ifdef CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD
#define CYG_DEV_SERIAL_BAUD_DIVISOR   \
    DIVISOR(CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD)
#else
#error Missing/incorrect serial baud rate defined - CDL error?
#endif

//-----------------------------------------------------------------------------
typedef struct {
    volatile avr32_usart_t* usart_dev;
    cyg_int32 msec_timeout;
    int isr_vector;
    int id;
} channel_data_t;

static channel_data_t channels[] = {
    { (avr32_usart_t*) AVR32_USART0_ADDRESS,
      1000,
      CYGNUM_HAL_INTERRUPT_SERIAL_CONSOLE,
      0
    },
    { (avr32_usart_t*) AVR32_USART1_ADDRESS,
      1000,
      CYGNUM_HAL_INTERRUPT_SERIAL_DEBUGGER,
      1
    },
    { (avr32_usart_t*) AVR32_USART2_ADDRESS,
      1000,
      CYGNUM_HAL_INTERRUPT_SERIAL_CONSOLE,
      2
    },
    { (avr32_usart_t*) AVR32_USART3_ADDRESS,
      1000,
      CYGNUM_HAL_INTERRUPT_SERIAL_DEBUGGER,
      3
    }
#ifdef AVR32_USART4_ADDRESS
    ,
    { (avr32_usart_t*) AVR32_USART4_ADDRESS,
      1000,
      CYGNUM_HAL_INTERRUPT_SERIAL_DEBUGGER,
      4
    }
#endif
};

//-----------------------------------------------------------------------------
// Set the baud rate

static void
cyg_hal_plf_serial_set_baud(cyg_uint8* port, cyg_uint16 baud_divisor)
{
    volatile avr32_usart_t  *dev = (avr32_usart_t*)port;

    //reset usart
    dev->cr = AVR32_USART_CR_RSTRX_MASK | 
              AVR32_USART_CR_RSTTX_MASK | 
              AVR32_USART_CR_RSTSTA_MASK ;

    // Disable all interrupts
    dev->idr = 0xFFFFFFFF;

    dev->mr = (AVR32_USART_NBSTOP_1 << AVR32_USART_NBSTOP_OFFSET) | 
              (AVR32_USART_PAR_NONE << AVR32_USART_PARE_OFFSET)   | 
              (AVR32_USART_CHRL_8 << AVR32_USART_CHRL_OFFSET)     |
               AVR32_USART_MODE_NORMAL;

    // Set baud rate.
    dev->brgr = baud_divisor;

    // Enable RX and TX
    dev->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK;
}

//-----------------------------------------------------------------------------
// The minimal init, get and put functions. All by polling.

void
cyg_hal_plf_serial_init_channel(void* __ch_data)
{
    volatile avr32_usart_t  *dev;

    // Some of the diagnostic print code calls through here with no idea what the ch_data is.
    // Go ahead and assume it is channels[0].
    if (__ch_data == 0)
         __ch_data = (void*)&channels[0];
	  
    dev = ((channel_data_t*) __ch_data)->usart_dev;
    // Set port multipexing
    if(((channel_data_t*) __ch_data)->id == 0)
    {
        gpio_enable_module_pin(CYG_HAL_USART0_TXD_PIN, 
                CYG_HAL_USART0_TXD_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_USART0_RXD_PIN, 
                CYG_HAL_USART0_RXD_FUNCTION);
    }
    if(((channel_data_t*) __ch_data)->id == 1)
    {
        gpio_enable_module_pin(CYG_HAL_USART1_TXD_PIN, 
                CYG_HAL_USART1_TXD_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_USART1_RXD_PIN, 
                CYG_HAL_USART1_RXD_FUNCTION);
    }
    if(((channel_data_t*) __ch_data)->id == 2)
    {
        gpio_enable_module_pin(CYG_HAL_USART2_TXD_PIN, 
                CYG_HAL_USART2_TXD_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_USART2_RXD_PIN, 
                CYG_HAL_USART2_RXD_FUNCTION);
    }
    if(((channel_data_t*) __ch_data)->id == 3)
    {
        gpio_enable_module_pin(CYG_HAL_USART3_TXD_PIN, 
                CYG_HAL_USART3_TXD_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_USART3_RXD_PIN, 
                CYG_HAL_USART3_RXD_FUNCTION);
    }
    #ifdef AVR32_USART4_ADDRESS
    if(((channel_data_t*) __ch_data)->id == 4)
    {
        gpio_enable_module_pin(CYG_HAL_USART4_TXD_PIN, 
                CYG_HAL_USART4_TXD_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_USART4_RXD_PIN, 
                CYG_HAL_USART4_RXD_FUNCTION);
    }
    #endif
    //reset usart
    dev->cr = AVR32_USART_CR_RSTRX_MASK | 
              AVR32_USART_CR_RSTTX_MASK | 
              AVR32_USART_CR_RSTSTA_MASK ;

    // Disable all interrupts
    dev->idr = 0xFFFFFFFF;

    dev->mr = (AVR32_USART_NBSTOP_1 << AVR32_USART_NBSTOP_OFFSET) | 
              (AVR32_USART_PAR_NONE << AVR32_USART_PARE_OFFSET)   | 
              (AVR32_USART_CHRL_8 << AVR32_USART_CHRL_OFFSET)     |
               AVR32_USART_MODE_NORMAL;

    // Set baud rate.
    dev->brgr = CYG_DEV_SERIAL_BAUD_DIVISOR;

    // Enable RX and TX
    dev->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK;
}

void
cyg_hal_plf_serial_putc(void* __ch_data, cyg_uint8 __ch)
{
    volatile avr32_usart_t  *dev;

    // Some of the diagnostic print code calls through here with no idea
    // what the ch_data is. Go ahead and assume it is channels[0].
    if (__ch_data == 0)
        __ch_data = (void*)&channels[0];

    dev = ((channel_data_t*) __ch_data)->usart_dev;

    CYGARC_HAL_SAVE_GP();

    while (!dev->CSR.usart_mode.txrdy)
    {
    }

    // Now, the transmit buffer is empty
    dev->thr = __ch;

    // Hang around until the character has been safely sent.
    while (!dev->CSR.usart_mode.txrdy)
    {
    }

    CYGARC_HAL_RESTORE_GP();
}

static cyg_bool
cyg_hal_plf_serial_getc_nonblock(void* __ch_data, cyg_uint8* ch)
{
    volatile avr32_usart_t  *dev;

    // Some of the diagnostic print code calls through here with no idea
    // what the ch_data is. Go ahead and assume it is channels[0].
    if (__ch_data == 0)
        __ch_data = (void*)&channels[0];

    dev = ((channel_data_t*) __ch_data)->usart_dev;

    if(!dev->CSR.usart_mode.rxrdy)
        return false;
		
    CYG_ASSERT((dev->csr & AVR32_USART_CSR_OVRE_MASK) == 0 , 
            "UART receiver overrun error");
    *ch = dev->rhr;

    return true;
}

cyg_uint8
cyg_hal_plf_serial_getc(void* __ch_data)
{
    cyg_uint8 ch;
    CYGARC_HAL_SAVE_GP();

    // Some of the diagnostic print code calls through here with no idea
    // what the ch_data is.Go ahead and assume it is channels[0].
    if (__ch_data == 0)
        __ch_data = (void*)&channels[0];

    while(!cyg_hal_plf_serial_getc_nonblock(__ch_data, &ch));

    CYGARC_HAL_RESTORE_GP();
    return ch;
}

static void
cyg_hal_plf_serial_write(void* __ch_data, const cyg_uint8* __buf, 
                         cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    // Some of the diagnostic print code calls through here with no idea
    // what the ch_data is. Go ahead and assume it is channels[0].
    if (__ch_data == 0)
        __ch_data = (void*)&channels[0];

    while(__len-- > 0)
        cyg_hal_plf_serial_putc(__ch_data, *__buf++);

    CYGARC_HAL_RESTORE_GP();
}

static void
cyg_hal_plf_serial_read(void* __ch_data, cyg_uint8* __buf, cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    // Some of the diagnostic print code calls through here with no idea
    // what the ch_data is.Go ahead and assume it is channels[0].
    if (__ch_data == 0)
        __ch_data = (void*)&channels[0];

    while(__len-- > 0)
        *__buf++ = cyg_hal_plf_serial_getc(__ch_data);

    CYGARC_HAL_RESTORE_GP();
}

cyg_bool
cyg_hal_plf_serial_getc_timeout(void* __ch_data, cyg_uint8* ch)
{
    int delay_count;
    channel_data_t* chan;
    cyg_bool res;
    CYGARC_HAL_SAVE_GP();

    // Some of the diagnostic print code calls through here with no idea what the ch_data is.
    // Go ahead and assume it is channels[0].
    if (__ch_data == 0)
      __ch_data = (void*)&channels[0];

    chan = (channel_data_t*)__ch_data;

    delay_count = chan->msec_timeout; // delay in 1000 us steps

    for(;;) {
        res = cyg_hal_plf_serial_getc_nonblock(__ch_data, ch);
        if (res || 0 == delay_count--)
            break;
        CYGACC_CALL_IF_DELAY_US(1000);
    }

    CYGARC_HAL_RESTORE_GP();
    return res;
}

static int
cyg_hal_plf_serial_control(void *__ch_data, __comm_control_cmd_t __func, ...)
{
    channel_data_t* chan;
    int ret = 0;
    CYGARC_HAL_SAVE_GP();

    // Some of the diagnostic print code calls through here with no idea what the ch_data is.
    // Go ahead and assume it is channels[0].
    if (__ch_data == 0)
        __ch_data = (void*)&channels[0];

    chan = (channel_data_t*)__ch_data;

    switch (__func) {
    case __COMMCTL_IRQ_ENABLE:
        // IRQ mode not supported
        return -1;
        break;
    case __COMMCTL_IRQ_DISABLE:
        return -1;
        break;
    case __COMMCTL_DBG_ISR_VECTOR:
        ret = chan->isr_vector;
        break;
    case __COMMCTL_SET_TIMEOUT:
    {
        va_list ap;

        va_start(ap, __func);

        ret = chan->msec_timeout;
        chan->msec_timeout = va_arg(ap, cyg_uint32);

        va_end(ap);
    }        
    break;
    case __COMMCTL_SETBAUD:
    {
        cyg_uint32 baud_rate;
        cyg_uint16 baud_divisor;
        cyg_uint8* port = (cyg_uint8*)chan->usart_dev;
        va_list ap;

        va_start(ap, __func);
        baud_rate = va_arg(ap, cyg_uint32);
        va_end(ap);

        switch (baud_rate)
        {
        case 110:    baud_divisor = DIVISOR(110);    break;
        case 150:    baud_divisor = DIVISOR(150);    break;
        case 300:    baud_divisor = DIVISOR(300);    break;
        case 600:    baud_divisor = DIVISOR(600);    break;
        case 1200:   baud_divisor = DIVISOR(1200);   break;
        case 2400:   baud_divisor = DIVISOR(2400);   break;
        case 4800:   baud_divisor = DIVISOR(4800);   break;
        case 7200:   baud_divisor = DIVISOR(7200);   break;
        case 9600:   baud_divisor = DIVISOR(9600);   break;
        case 14400:  baud_divisor = DIVISOR(14400);  break;
        case 19200:  baud_divisor = DIVISOR(19200);  break;
        case 38400:  baud_divisor = DIVISOR(38400);  break;
        case 57600:  baud_divisor = DIVISOR(57600);  break;
        case 115200: baud_divisor = DIVISOR(115200); break;
        case 230400: baud_divisor = DIVISOR(230400); break;
        default:     return -1;                      break; // Invalid baud rate selected
        }

        // Set baud rate.
        cyg_hal_plf_serial_set_baud(port, baud_divisor);
    }
    break;

    case __COMMCTL_GETBAUD:
        break;
    default:
        break;
    }
    CYGARC_HAL_RESTORE_GP();
    return ret;
}


static int
cyg_hal_plf_serial_isr(void *__ch_data, int* __ctrlc, 
                       CYG_ADDRWORD __vector, CYG_ADDRWORD __data)
{
    // Interrupt version not supported 
    // If you can use interrupt version add ISR routine code hear
    return 0;//res;
}

static void
cyg_hal_plf_serial_init(void)
{
    int i;
    hal_virtual_comm_table_t* comm;
    int cur = CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT);

    #define NUM_CHANNELS CYGNUM_HAL_VIRTUAL_VECTOR_COMM_CHANNELS
    for (i = 0; i < NUM_CHANNELS; i++) {

	// Disable interrupts.
	HAL_INTERRUPT_MASK(channels[i].isr_vector);

	// Init channels
	cyg_hal_plf_serial_init_channel((void*)&channels[i]);
    
	// Setup procs in the vector table

	// Set COMM callbacks for channel
	CYGACC_CALL_IF_SET_CONSOLE_COMM(i);
	comm = CYGACC_CALL_IF_CONSOLE_PROCS();
	CYGACC_COMM_IF_CH_DATA_SET(*comm, &channels[i]);
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

//-----------------------------------------------------------------------------
// end of hal_diag.c