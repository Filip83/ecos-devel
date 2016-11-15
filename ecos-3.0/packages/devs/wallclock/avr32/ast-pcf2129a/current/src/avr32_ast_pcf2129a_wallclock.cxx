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
#include <pkgconf/devices_wallclock_avr32_ast_pcf2129a.h>
#include <string.h>                     // memcpy()
#include CYGBLD_HAL_BOARD_H

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/wallclock.hxx>
#include <cyg/io/wallclock/wallclock.inl>

#include <cyg/io/i2c.h>

#if 1
# define DEBUG(_format_, ...)
#else
# define DEBUG(_format_, ...) diag_printf(_format_, ## __VA_ARGS__)
#endif

#define PCF_CONTROL1	   	0x00
#define PCF_CONTROL2	   	0x01
#define PCF_CONTROL3	   	0x02
#define PCF_SECONDS        	0x03
#define PCF_MINUTES        	0x04
#define PCF_HOURS          	0x05
#define PCF_DAYS           	0x06
#define PCF_DOW            	0x07
#define PCF_MONTH          	0x08
#define PCF_YEAR           	0x09
#define PCF_SECOND_ALARM        0x0a
#define PCF_MINUTE_ALARM        0x0b
#define PCF_HOUR_ALARM          0x0c
#define PCF_DAY_ALARM           0x0d
#define PCF_DOW_ALARM           0x0e
#define PCF_CLKOUT_CTL          0x0f
#define PCF_WATCHDOG_TIM_CTL    0x10
#define PCF_WATCHDOG_TIM_VAL    0x11
#define PCF_TIMESTP_CTL         0x12
#define PCF_SECOND_TIMESTP      0x13
#define PCF_MINUTE_TIMESTP      0x14
#define PCF_HOUR_TIMESTP        0x15
#define PCF_DAY_TIMESTP         0x16
#define PCF_MONTH_TIMESTP       0x17
#define PCF_YEAR_TIMESTP        0x18
#define PCF_AGING_OFFSET        0x19
#define PCF_IREG1           	0x1a
#define PCF_IREG2           	0x1b


#define PCF_REGS_SIZE       	0x0a   // Size of register space

#define PCF_SECONDS_OSF      	0x80   // Clock Halt
#define PCF_HOURS_24        	0x04   // 24 hour clock mode

#define PCF_I2C_ADDRES		0x51

#define HW_ERROR_OSC32_NOT_RUNNING			0x0000000000000004

externC cyg_uint64 _hw_error;
// I2C device
externC cyg_i2c_device cyg_i2c_wallclock_pcf2129a;

static void
get_pcf_hwclock(cyg_uint32* year, cyg_uint32* month, cyg_uint32* mday,
               cyg_uint32* hour, cyg_uint32* minute, cyg_uint32* second);
           
static void
pcf_write_registers(cyg_uint8 register_offset, cyg_uint8 cnt, cyg_uint8* regs)
{
    cyg_uint8 tx_data[PCF_REGS_SIZE + 1];
    tx_data[0] = register_offset;
    memcpy(&(tx_data[1]), regs, cnt);
    cyg_i2c_transaction_begin(&cyg_i2c_wallclock_pcf2129a);
    cyg_i2c_transaction_tx(&cyg_i2c_wallclock_pcf2129a, true, 
            tx_data, cnt + 1, true);
    cyg_i2c_transaction_end(&cyg_i2c_wallclock_pcf2129a);
}

static int
pcf_read_registers(cyg_uint8 register_offset, cyg_uint8 cnt, cyg_uint8* regs)
{
    int ret = 0;
    cyg_uint8 tx_data[1];
    tx_data[0] = register_offset;

    cyg_i2c_transaction_begin(&cyg_i2c_wallclock_pcf2129a);

    if(cyg_i2c_transaction_tx(&cyg_i2c_wallclock_pcf2129a, true, 
                tx_data, 1, true) != 1)
        ret = 1;
    else
        cyg_i2c_transaction_rx(&cyg_i2c_wallclock_pcf2129a, true, 
                regs, cnt, true, true);

    cyg_i2c_transaction_end(&cyg_i2c_wallclock_pcf2129a);
    return ret;
}

static void
DS_PUT(cyg_uint8* regs)
{
    pcf_write_registers(PCF_CONTROL1,10,regs);
}

static void
DS_GET(cyg_uint8* regs)
{
    cyg_bool    ok = true;

    if (pcf_read_registers(PCF_CONTROL1,10,regs) != 0) 
    {
        // The device has not responded to the address byte.
         ok = false;
	SET_HW_ERRROR(HW_ERROR_RTC_NO_RESPONSE);
    } 
    else
    {
        // Verify that there are reasonable default settings. The
        // register values can be used as array indices so bogus
        // values can lead to bus errors or similar problems.
        
        // Years: 00 - 99, with 70-99 interpreted as 1970 onwards.
        if ((regs[PCF_YEAR] & 0x0F) > 0x09) {
            ok = false;
        }
        // Month: 1 - 12
        if ((regs[PCF_MONTH] == 0x00) ||
            ((regs[PCF_MONTH] > 0x09) && (regs[PCF_MONTH] < 0x10)) ||
            (regs[PCF_MONTH] > 0x12)) {
            ok = false;
        }
        // Day: 1 - 31. This check does not allow for 28-30 day months.
        if ((regs[PCF_DAYS] == 0x00) ||
            ((regs[PCF_DAYS] & 0x0F) > 0x09) ||
            (regs[PCF_DAYS] > 0x31)) {
            ok = false;
        }
        // Hours: 0 - 23. Always run in 24-hour mode
        if (/*(0 != (regs[PCF_CONTROL1] & PCF_HOURS_24)) ||*/
            ((regs[PCF_HOURS] & 0x0F) > 0x09) ||
            ((regs[PCF_HOURS] & 0x3F) > 0x023)) {
            ok = false;
        }
        // Ignore the DOW field. The wallclock code does not need it, and
        // it is hard to calculate.
        // Minutes: 0 - 59
        if (((regs[PCF_MINUTES] & 0x0F) > 0x09) ||
            (regs[PCF_MINUTES] > 0x59)) {
            ok = false;
        }
        // Seconds: 0 - 59
        if (((regs[PCF_SECONDS] & 0x0F) > 0x09) ||
            (regs[PCF_SECONDS] > 0x59)) {
            ok = false;
        }
		
        if((0 != (regs[PCF_SECONDS] & PCF_SECONDS_OSF)))
                ok = false;
    }
    if (! ok) {
        // Any problems, return Jan 1 1970 but do not update the hardware.
        // Leave it to the user or other code to set the clock to a sensible
        // value.
        regs[PCF_SECONDS]  = 0x00;
        regs[PCF_MINUTES]  = 0x00;
        regs[PCF_HOURS]    = 0x00;
        regs[PCF_DOW]      = 0x00;
        regs[PCF_DAYS]     = 0x01;                                                         
        regs[PCF_MONTH]    = 0x01;                                                        
        regs[PCF_YEAR]     = 0x13;
        regs[PCF_CONTROL1] = 0x00;
        regs[PCF_CONTROL2] = 0x00;
        regs[PCF_CONTROL3] = 0x00;

        SET_HW_ERRROR(HW_ERROR_RTC_CHECK_TIME);	
    }
}


//----------------------------------------------------------------------------
// Accessor functions

static cyg_uint32
init_pcf_hwclock(void)
{
    cyg_uint32 year, month, mday, hour, minute, second, now;
    cyg_uint8 regs[PCF_REGS_SIZE+1];
    cyg_bool    ok = true;

    if (pcf_read_registers(PCF_CONTROL1,10,regs) != 0)
    {
	// The device has not responded to the address byte.
	ok = false;
	SET_HW_ERRROR(HW_ERROR_RTC_NO_RESPONSE);
	DEBUG("Cannot connect to PCF2129A\n");
    }
    else
    {
        // Verify that there are reasonable default settings. The
        // register values can be used as array indices so bogus
        // values can lead to bus errors or similar problems.

        // Years: 00 - 99, with 70-99 interpreted as 1970 onwards.
        if ((regs[PCF_YEAR] & 0x0F) > 0x09) {
            ok = false;
        }
        // Month: 1 - 12
        if ((regs[PCF_MONTH] == 0x00) ||
           ((regs[PCF_MONTH] > 0x09) && (regs[PCF_MONTH] < 0x10)) ||
            (regs[PCF_MONTH] > 0x12)) {
            ok = false;
        }
        // Day: 1 - 31. This check does not allow for 28-30 day months.
        if ((regs[PCF_DAYS] == 0x00) ||
           ((regs[PCF_DAYS] & 0x0F) > 0x09) ||
            (regs[PCF_DAYS] > 0x31)) {
            ok = false;
        }
        // Hours: 0 - 23. Always run in 24-hour mode
        if (/*(0 != (regs[PCF_CONTROL1] & PCF_HOURS_24)) ||*/
                   ((regs[PCF_HOURS] & 0x0F) > 0x09) ||
                   ((regs[PCF_HOURS] & 0x3F) > 0x023)) {
            ok = false;
        }
        // Ignore the DOW field. The wallclock code does not need it, and
        // it is hard to calculate.
        // Minutes: 0 - 59
        if (((regs[PCF_MINUTES] & 0x0F) > 0x09) ||
             (regs[PCF_MINUTES] > 0x59)) {
            ok = false;
        }
        // Seconds: 0 - 59
        if (((regs[PCF_SECONDS] & 0x0F) > 0x09) ||
             (regs[PCF_SECONDS] > 0x59)) {
            ok = false;
        }

        if((0 != (regs[PCF_SECONDS] & PCF_SECONDS_OSF)))
              ok = false;
    }
    
    if (! ok) {
        // Any problems, return Jan 1 2013 but do not update the hardware.
        // Leave it to the user or other code to set the clock to a sensible
        // value.
        regs[PCF_SECONDS]  = 0x00;
        regs[PCF_MINUTES]  = 0x00;
        regs[PCF_HOURS]    = 0x00;
        regs[PCF_DOW]      = 0x00;
        regs[PCF_DAYS]     = 0x01;
        regs[PCF_MONTH]    = 0x01;
        regs[PCF_YEAR]     = 0x13;
        regs[PCF_CONTROL1] = 0x00;
        regs[PCF_CONTROL2] = 0x00;
        regs[PCF_CONTROL3] = 0x00;
	    
	DEBUG("Time in the PCF2129A corrupted\n");
	SET_HW_ERRROR(HW_ERROR_RTC_CHECK_TIME);
	    
    }
	
    if(regs[PCF_CONTROL1]  != 0 || regs[PCF_CONTROL2] || regs[PCF_CONTROL3]  )
    {
        regs[PCF_CONTROL1] = 0x00;
        regs[PCF_CONTROL2] = 0x00;
        regs[PCF_CONTROL3] = 0x00;	

        pcf_write_registers(PCF_CONTROL1,3,regs);
    }

    get_pcf_hwclock(&year, &month, &mday, &hour, &minute, &second);
    now = _simple_mktime(year, month, mday, hour, minute, second);
    return now;
}


void
set_pcf_hwclock(cyg_uint32 year, cyg_uint32 month, cyg_uint32 mday,
               cyg_uint32 hour, cyg_uint32 minute, cyg_uint32 second)
{
    cyg_uint8 regs[PCF_REGS_SIZE];

    // Set up the registers
    //regs[PCF_CONTROL]    = 0x00;
    regs[PCF_YEAR]       = TO_BCD((cyg_uint8)(year % 100));
    regs[PCF_MONTH]      = TO_BCD((cyg_uint8)month);
    regs[PCF_DAYS]       = TO_BCD((cyg_uint8)mday);
    regs[PCF_DOW]        = TO_BCD(0x01);     // Not accurate, but not used by this driver either
    regs[PCF_HOURS]      = TO_BCD((cyg_uint8)hour);
    regs[PCF_MINUTES]    = TO_BCD((cyg_uint8)minute);
    // This also starts the clock
    regs[PCF_SECONDS]    = TO_BCD((cyg_uint8)second);

    // Send the register set to the hardware
    DS_PUT(regs);

    // These debugs will cause the test to eventually fail due to
    // the printouts causing timer interrupts to be lost...
    DEBUG("PCF2129A set -------------\n");
    DEBUG("regs %02x %02x %02x %02x %02x %02x %02x %02x\n",
          regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
    DEBUG("year %02d\n", year);
    DEBUG("month %02d\n", month);
    DEBUG("mday %02d\n", mday);
    DEBUG("hour %02d\n", hour);
    DEBUG("minute %02d\n", minute);
    DEBUG("second %02d\n", second);
}

static void
get_pcf_hwclock(cyg_uint32* year, cyg_uint32* month, cyg_uint32* mday,
               cyg_uint32* hour, cyg_uint32* minute, cyg_uint32* second)
{
    cyg_uint8 regs[PCF_REGS_SIZE];

    // Fetch the current state
    DS_GET(regs);

    *year = (cyg_uint32)TO_DEC(regs[PCF_YEAR]);
    // The year field only has the 2 least significant digits :-(
   /* if (*year >= 70) {
        *year += 1900;
    } else {*/
        *year += 2000;
    //}
    *month = (cyg_uint32)TO_DEC(regs[PCF_MONTH]);
    *mday = (cyg_uint32)TO_DEC(regs[PCF_DAYS]);
    *hour = (cyg_uint32)TO_DEC(regs[PCF_HOURS] & 0x3F);
    *minute = (cyg_uint32)TO_DEC(regs[PCF_MINUTES]);
    *second = (cyg_uint32)TO_DEC(regs[PCF_SECONDS] & 0x7F);

    // These debugs will cause the test to eventually fail due to
    // the printouts causing timer interrupts to be lost...
    DEBUG("PCF2129A get -------------\n");
    DEBUG("regs %02x %02x %02x %02x %02x %02x %02x %02x\n",
          regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
    DEBUG("year %02d\n", *year);
    DEBUG("month %02d\n", *month);
    DEBUG("mday %02d\n", *mday);
    DEBUG("hour %02d\n", *hour);
    DEBUG("minute %02d\n", *minute);
    DEBUG("second %02d\n", *second);
}
               
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
      
      AVR32_AST.clock = 
           (AVR32_AST_CLOCK_CSSEL_32_KHZ_CLOCK << AVR32_AST_CLOCK_CSSEL_OFFSET);

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

      // RTC timer can not be initialized from 
      // I2C RTC because I2C driver need running kernel 
      AVR32_AST.cv = 0;//init_pcf_hwclock();
  }
  else
    CYG_ASSERT(false,"AST failure xx\n");
  
}

cyg_uint32
Cyg_WallClock::get_hw_seconds(void)
{
  cyg_uint32 naw;
  //and get new time
  naw = AVR32_AST.calv;
  return naw;
}

#ifdef CYGSEM_WALLCLOCK_SET_GET_MODE
void
Cyg_WallClock::set_hw_seconds(cyg_uint32 secs)
{

  cyg_uint32 year, month, mday, hour, minute, second;

  /* halt clock, reset counter */
  while((AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK))
  {
  }

  //reset rtc prescaler
  AVR32_AST.cr |= AVR32_AST_CR_PCLR_MASK;

   //and set new time
  _simple_mkdate(secs, &year, &month, &mday, &hour, &minute, &second);
  
  while((AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK))
  {
  }
 
  AVR32_AST.cv = secs;
  set_pcf_hwclock(year, month, mday, hour, minute, second);

}
#endif // CYGSEM_WALLCLOCK_SET_GET_MODE

//==========================================================================
// EOF avr32_uc3c_ast_wallclock.cxx
