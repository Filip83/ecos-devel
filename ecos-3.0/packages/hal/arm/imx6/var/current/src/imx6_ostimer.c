/**************************************************************************/
/**
*
* @file     imx6_ostimer.c
*
* @brief    iMX6 Cortex-A9 OS timer [Private Timer] functions
*
***************************************************************************/
/*==========================================================================
//
//      imx6_ostimer.c
//
//      HAL timer code using the Private Timer Counter
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003 Free Software Foundation, Inc.
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
// Author(s):      Mike Jones
// Contributor(s): ITR-GmbH
// Date:           2013-08-08
// Purpose:        HAL board support
// Description:    Implementations of HAL board interfaces
//
//####DESCRIPTIONEND####
//
//========================================================================
*/

/*
 * Notes:
 *
 * 1) This has been macroized so that it does not depends on imx6_misc.c code and can be moved elsewhere.
 *    It is not clear where it would go. Sabre is for particular boards. Perhaps if things shift to Cortex A
 *    HAL it would go there.
 */

#include <pkgconf/hal.h>

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/infra/cyg_ass.h>          // assertion macros
#include <cyg/infra/diag.h>

#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_arch.h>           // Register state info
#include <cyg/hal/hal_intr.h>           // necessary?
#include <cyg/hal/var_timer.h>

/* Internal tick units */
static cyg_uint32 _period;

/****************************************************************************/
/**
*
* HAL clock initialize: Initialize OS timer [A9 Private timer]
*
* @param    period - value for load to private timer.
*
* @return   none
*
*****************************************************************************/

void hal_clock_initialize(cyg_uint32 period)
{
    // Period should be 1000 so that 1000ns is 1ms.
	HAL_TIMER_ENABLE(period);
    _period = period;
}

/****************************************************************************/
/**
*
* HAL clock reset handler: Reset OS timer [A9 Private timer]
*
* @param    vector - interrupt number of private timer.
* @param    period - value for load to private timer.
*
* @return   none
*
*****************************************************************************/
void hal_clock_reset(cyg_uint32 vector, cyg_uint32 period)
{
    HAL_TIMER_WAIT_COMPARE();

    if (_period != period) {
        HAL_TIMER_DISABLE();
        HAL_TIMER_ENABLE(period);
        _period = period;
   }
}

/****************************************************************************/
/**
*
* HAL clock read
*
* @param    pvalue - pointer to cyg_uint32 variable for filling current timer value.
*
* @return   none
*
*****************************************************************************/
void hal_clock_read(cyg_uint32 *pvalue)
{
    cyg_uint32 i;

    i = HAL_TIMER_VALUE();
    *pvalue = _period - i;
}

/****************************************************************************/
/**
*
* HAL us delay
*
* @param    usecs - number of usecs for delay.
*
* @return   none
*
*****************************************************************************/
// This will only work up to 1000us and then it will wrap twice and fail to be accurate.
// The 1000 comes from the counter value used by EPIT2. Because that value also sets
// the 1ms RTC, it can't be changed.
void hal_delay_us(cyg_int32 usecs)
{
    cyg_uint32 stat;
    cyg_int32 first, second, limit;

    limit = (cyg_int32) usecs;

    hal_clock_read(&stat);
    first = (cyg_int32) stat;
    do {
        hal_clock_read(&stat);
        second = (cyg_int32) stat;
    } while ((second - first) < (_period - 1) ? (second - first) < limit : ((_period - 1 - first) + second) < limit);


}

// imx6_ostimer.c
