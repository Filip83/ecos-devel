#ifndef CYGONCE_HAL_PLATFORM_INTS_H
#define CYGONCE_HAL_PLATFORM_INTS_H
//==========================================================================
//
//      hal_platform_ints.h
//
//      HAL Interrupt and clock assignments for AT91SAM7
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2006 Free Software Foundation, Inc.
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
// Author(s):    Mike Jones
// Contributors: 
// Date:         2013-08-08
// Purpose:      Define Interrupt support
// Description:  The interrupt specifics for the Freescale iMX6 platform are
//               defined here.
//
// Usage:        #include <cyg/hal/hal_platform_ints.h>
//               ...
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/hal/var_intr.h>

// TODO:
// These remarked peripheral imterrupts came from mars. I have not checked
// that the CPU timer is actually INT 29. My guess it it should be using
// CYGNUM_HAL_INTERRUPT_EPIT1. The timers available are EPIT, GPT, WDOG, 
// SRTC.

// CPU Private Peripheral Interrupts (PPI)
//#define CYGNUM_HAL_INTERRUPT_GLOBALTMR          27  /* SCU Global Timer interrupt */
//#define CYGNUM_HAL_INTERRUPT_FIQ                28  /* FIQ from FPGA fabric */
//#define CYGNUM_HAL_INTERRUPT_SCU_CPUTMR         29  /* SCU Private Timer interrupt */
//#define CYGNUM_HAL_INTERRUPT_SCU_WDTTMR         30  /* SCU Private WDT interrupt */
//#define CYGNUM_HAL_INTERRUPT_IRQ                31  /* IRQ from FPGA fabric */ 


//#define CYGNUM_HAL_ISR_MIN                      0
//#define CYGNUM_HAL_ISR_MAX                      94
//#define CYGNUM_HAL_ISR_COUNT                    (CYGNUM_HAL_ISR_MAX + 1)

#define CYGNUM_HAL_INTERRUPT_RTC                CYGNUM_HAL_INTERRUPT_EPIT2

externC void hal_reset(void);
#define HAL_PLATFORM_RESET()                    hal_reset()

#define HAL_PLATFORM_RESET_ENTRY                0x0000000

#endif // CYGONCE_HAL_PLATFORM_INTS_H
