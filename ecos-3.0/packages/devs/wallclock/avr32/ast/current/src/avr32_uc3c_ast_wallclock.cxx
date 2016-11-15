//==========================================================================
//
//      avr32_uc3c_wallclock_wallclock.cxx
//
//      Wallclock implementation for AVR32UC3C CPUs
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003, 2004 Free Software Foundation, Inc.
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
// Date:          2012-11-19
// Purpose:       Wallclock driver for AVR32UC3C CPUs
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/hal/avr32/io.h>
#include <pkgconf/hal.h>
#include <pkgconf/wallclock.h>
#include <pkgconf/devices_wallclock_avr32_ast.h>

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/wallclock.hxx>
#include <cyg/io/wallclock/wallclock.inl>

#define HW_ERROR_OSC32_NOT_RUNNING			0x0000000000000004

externC cyg_uint64 _hw_error;
           
               
int ast_is_busy(volatile avr32_ast_t *ast)
{
  return (ast->sr & AVR32_AST_SR_BUSY_MASK) != 0;
}

int ast_is_clkbusy(volatile avr32_ast_t *ast)
{
  return (ast->sr & AVR32_AST_SR_CLKBUSY_MASK) != 0;
}

void
Cyg_WallClock::init_hw_seconds(void)
{
  cyg_uint32 wait_loop = 50000;
  #ifdef CYGDBG_IO_INIT
  diag_printf("Init wallclock AST\n");
  #endif

  if(!(_hw_error & HW_ERROR_OSC32_NOT_RUNNING))
  {
      //cyg_uint32 year, month, mday, hour, minute, second;
      //enabl AST clock source as 32k external oscilator
      while((AVR32_AST.sr & AVR32_AST_SR_CLKBUSY_MASK))
      {
        wait_loop--;
        if(wait_loop == 0)
        {
            CYG_ASSERT(false,"AST failure 1\n");
            _hw_error |= HW_ERROR_OSC32_NOT_RUNNING;
            return;
        }
      }
      //TODOO: Make it smarter
#ifdef AVR32_AST_CLOCK_CSSEL_32_KHZ_CLOCK
      AVR32_AST.clock = 
           (AVR32_AST_CLOCK_CSSEL_32_KHZ_CLOCK << AVR32_AST_CLOCK_CSSEL_OFFSET);
#endif
#ifdef AVR32_AST_CLOCK_CSSEL_32KHZCLK
      AVR32_AST.clock = 
           (AVR32_AST_CLOCK_CSSEL_32KHZCLK << AVR32_AST_CLOCK_CSSEL_OFFSET);
#endif
      wait_loop = 50000;
      while((AVR32_AST.sr & AVR32_AST_SR_CLKBUSY_MASK))
      {
        wait_loop--;
        if(wait_loop == 0)
        {
            CYG_ASSERT(false,"AST failure 2\n");
            _hw_error |= HW_ERROR_OSC32_NOT_RUNNING;
            return;
        }
      }

      AVR32_AST.clock |= AVR32_AST_CEN_MASK;

      while((AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK))
      {
      }

      //enabling ast counter in callender mode and prescaler 14 e.g. 32768
      AVR32_AST.cr = (14 << AVR32_AST_CR_PSEL_OFFSET) |
                                 AVR32_AST_CR_EN_MASK | 
                                 AVR32_AST_CR_PCLR_MASK;

      while((AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK))
      {
      }

      AVR32_AST.cv = 0;
  }
  else
    CYG_ASSERT(false,"AST failure xx\n");
  
}

cyg_uint64
Cyg_WallClock::get_hw_seconds(void)
{
  cyg_uint64 naw;
  //and get new time
  naw = AVR32_AST.calv;
  // The AST time counter counts time from 1.1.2010 00:00
  naw += _simple_mktime(2010,1,1,0,0,0);
  return naw;
}

#ifdef CYGSEM_WALLCLOCK_SET_GET_MODE
void
Cyg_WallClock::set_hw_seconds(cyg_uint64 secs)
{
  /* halt clock, reset counter */
  while((AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK))
  {
  }

  //reset rtc prescaler
  AVR32_AST.cr |= AVR32_AST_CR_PCLR_MASK;


  
  while((AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK))
  {
  }
 
  AVR32_AST.cv = (cyg_uint32)(secs - _simple_mktime(2010,1,1,0,0,0));

}
#endif // CYGSEM_WALLCLOCK_SET_GET_MODE

//==========================================================================
// EOF avr32_uc3c_ast_wallclock.cxx
