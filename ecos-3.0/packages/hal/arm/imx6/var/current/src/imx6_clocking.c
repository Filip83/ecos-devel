//==========================================================================
//
//      imx6_clocking.c
//
//      iMX6 HAL functions
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2010 Free Software Foundation, Inc.                        
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
// Author(s):    Mike Jones <mike@proclivis.com>
// Contributors  Ilija kocho <ilijak@siva.com.mk>
// Date:         2013-08-08
// Description:
//
//####DESCRIPTIONEND####
//
//========================================================================

#include <pkgconf/hal.h>
#include <pkgconf/hal_arm.h>
#include <pkgconf/hal_arm_imx6.h>
#ifdef CYGPKG_KERNEL
#include <pkgconf/kernel.h>
#endif

#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_arch.h>           // HAL header
#include <cyg/hal/hal_intr.h>           // HAL header
#include <cyg/hal/hal_if.h>             // HAL header

#include <cyg/hal/var_ccm.h>

#include <cyg/io/ser_freescale_uart.h>

//==========================================================================
// UART baud rate
//
// Set the baud rate divider of a UART based on the requested rate and
// the current clock settings.


void
hal_imx6_uart_setbaud(cyg_uint32 uart_p, cyg_uint32 baud)
{
//    cyg_uint32 sbir, ubmr;
    cyg_uint32 regval;

/*
    switch(uart_p) {
    case CYGADDR_IO_SERIAL_IMX6_UART0_BASE:
    case CYGADDR_IO_SERIAL_IMX6_UART1_BASE:
        sbir = hal_get_peripheral_clock/baud;
        break;
    case CYGADDR_IO_SERIAL_IMX6_UART2_BASE:
    case CYGADDR_IO_SERIAL_IMX6_UART3_BASE:
    case CYGADDR_IO_SERIAL_IMX6_UART4_BASE:
        sbir = hal_get_peripheral_clock/baud;
        break;
    default:
        sbir=0;
        break;
    }
*/
//    if(sbir) {
        regval = (baud / 100) - 1;
        HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UBIR, regval);
// TODO: Don't use fixed value of 1
        regval = (uart_get_reffreq(1) / 1600) - 1;
        HAL_WRITE_UINT16(uart_p + CYGHWR_DEV_FREESCALE_UART_UBMR, regval);
//    }
}

cyg_uint32 hal_get_peripheral_clock(void)
{
    return hal_get_main_clock(IPG_PER_CLK);
}

//==========================================================================
// EOF imx6_clocking.c
