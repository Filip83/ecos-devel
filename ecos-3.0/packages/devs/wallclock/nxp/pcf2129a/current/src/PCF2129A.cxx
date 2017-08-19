//==========================================================================
//
//      pcf2129a.inl
//
//      Wallclock implementation for PCF2129A
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
// Original Data: gthomas
// Date:          2003-09-19
// Purpose:       Wallclock driver for PCF2129A
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>                // Platform specific configury
#include <pkgconf/wallclock.h>          // Wallclock device config
#include <pkgconf/devices_wallclock_nxp_pcf2129a.h>

#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_intr.h>           // interrupt enable/disable
#include <cyg/infra/cyg_type.h>         // Common type definitions and support
#include <string.h>                     // memcpy()

#include <cyg/io/wallclock.hxx>         // The WallClock API
#include <cyg/io/wallclock/wallclock.inl> // Helpers

#include <cyg/infra/diag.h>

#if defined(CYGNUM_DEVS_WALLOCK_NXP_PCF2129A_INTERFACE) && \
            CYGNUM_DEVS_WALLOCK_NXP_PCF2129A_INTERFACE == 1
#include <cyg/io/i2c.h>
externC cyg_i2c_device cyg_i2c_wallclock_pcf2129a;
#else
#include <cyg/io/spi.h>
#include <cyg/io/spi_freescale_dspi.h>
externC cyg_spi_device cyg_spi_wallclock_pcf2129a;
#endif


#if CYGPKG_DEVICES_WALLCLOCK_NXP_PCF2129A_DEBUG_LEVEL == 0
# define DEBUG(_format_, ...)
#else
# define DEBUG(_format_, ...) diag_printf(_format_, ## __VA_ARGS__)
#endif

// Registers.
// FIXME: there is no need to include the control register here, it
// controls a square wave output which is independent from the wallclock.
// However fixing it would require changing any platforms that use the
// old DS_GET()/DS_PUT() functionality.
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

#define PCF_SPI_WRITE           0x20
#define PCF_SPI_READ            0x20 | 0x80


static void
pcf_write_registers(cyg_uint8 register_offset, cyg_uint8 cnt, cyg_uint8* regs)
{
    cyg_uint8 tx_data[PCF_REGS_SIZE + 1];
    memcpy(&(tx_data[1]), regs, cnt);
#if defined(CYGNUM_DEVS_WALLOCK_NXP_PCF2129A_INTERFACE) && \
            CYGNUM_DEVS_WALLOCK_NXP_PCF2129A_INTERFACE == 1
    tx_data[0] = register_offset;
    cyg_i2c_transaction_begin(&cyg_i2c_wallclock_pcf2129a);
    cyg_i2c_transaction_tx(&cyg_i2c_wallclock_pcf2129a, true, tx_data, cnt + 1, true);
    cyg_i2c_transaction_end(&cyg_i2c_wallclock_pcf2129a)
#else
    tx_data[0] = PCF_SPI_WRITE | (register_offset&0x1f);
    cyg_spi_transaction_begin(&cyg_spi_wallclock_pcf2129a);
    cyg_spi_transaction_transfer(&cyg_spi_wallclock_pcf2129a,true,cnt + 1,
            tx_data,NULL,true);
    cyg_spi_transaction_end(&cyg_spi_wallclock_pcf2129a);
#endif
}

static int
pcf_read_registers(cyg_uint8 register_offset, cyg_uint8 cnt, cyg_uint8* regs)
{
    int ret = 0;
    cyg_uint8 tx_data[1];
#if defined(CYGNUM_DEVS_WALLOCK_NXP_PCF2129A_INTERFACE) && \
            CYGNUM_DEVS_WALLOCK_NXP_PCF2129A_INTERFACE == 1
    tx_data[0] = register_offset;

    cyg_i2c_transaction_begin(&cyg_i2c_wallclock_pcf2129a);

    if(cyg_i2c_transaction_tx(&cyg_i2c_wallclock_pcf2129a, true, tx_data, 1, true) != 1)
        ret = 1;
    else
        cyg_i2c_transaction_rx(&cyg_i2c_wallclock_pcf2129a, true, regs, cnt, true, true);

    cyg_i2c_transaction_end(&cyg_i2c_wallclock_pcf2129a);
#else
    tx_data[0] = PCF_SPI_READ | (register_offset&0x1f);
    cyg_spi_transaction_begin(&cyg_spi_wallclock_pcf2129a);
    cyg_spi_transaction_transfer(&cyg_spi_wallclock_pcf2129a,true,1,
            tx_data,NULL,false);
    
    cyg_spi_transaction_transfer(&cyg_spi_wallclock_pcf2129a,true,cnt,
            NULL,regs,true);
    cyg_spi_transaction_end(&cyg_spi_wallclock_pcf2129a);
#endif
    return ret;
}

static void
DS_GET(cyg_uint8* regs)
{
    cyg_bool    ok = true;

    // Now fetch the data
    // if transfer not correst report, continue otherwise
    if (pcf_read_registers(PCF_CONTROL1,10,regs) != 0) 
    {
        // The device has not responded.
        ok = false;
	SET_HW_ERRROR(HW_ERROR_RTC_NO_RESPONSE);
        DEBUG("Cannto conntect to PCF2129A\n");
    } 
    else 
    {
        // Verify that there are reasonable default settings. The
        // register values can be used as array indices so bogus
        // values can lead to bus errors or similar problems.
        
        if ((regs[PCF_YEAR] & 0x0F) > 0x09)
        {
            ok = false;
        }
        // Month: 1 - 12
        if ((regs[PCF_MONTH] == 0x00) ||
            ((regs[PCF_MONTH] > 0x09) && (regs[PCF_MONTH] < 0x10)) ||
            (regs[PCF_MONTH] > 0x12)) 
        {
            ok = false;
        }
        // Day: 1 - 31. This check does not allow for 28-30 day months.
        if ((regs[PCF_DAYS] == 0x00) ||
            ((regs[PCF_DAYS] & 0x0F) > 0x09) ||
            (regs[PCF_DAYS] > 0x31)) 
        {
            ok = false;
        }
        // Hours: 0 - 23. Always run in 24-hour mode
        if (/*(0 != (regs[PCF_CONTROL1] & PCF_HOURS_24)) ||*/
            ((regs[PCF_HOURS] & 0x0F) > 0x09) ||
            ((regs[PCF_HOURS] & 0x3F) > 0x023)) 
        {
            ok = false;
        }
        // Minutes: 0 - 59
        if (((regs[PCF_MINUTES] & 0x0F) > 0x09) ||
            (regs[PCF_MINUTES] > 0x59)) 
        {
            ok = false;
        }
        // Seconds: 0 - 59
        if (((regs[PCF_SECONDS] & 0x0F) > 0x09) ||
            (regs[PCF_SECONDS] > 0x59)) 
        {
            ok = false;
        }
    }
    
    if (! ok) 
    {
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
        
        SET_HW_ERRROR(HW_ERROR_RTC_CHECK_TIME);
    }
}

static void
DS_PUT(cyg_uint8* regs)
{
    pcf_write_registers(PCF_CONTROL1,10,regs);
}

//----------------------------------------------------------------------------
// Accessor functions

static inline void
init_pcf_hwclock(void)
{
    cyg_uint8 regs[PCF_REGS_SIZE];

    // Fetch the current state
    DS_GET(regs);
    
    // If the clock is not currently running or is not in 24-hours mode,
    // update it. Otherwise skip the update because the clock may have
    // ticked between DS_GET() and DS_PUT() and we could be losing the
    // occasional second.
    if ((0 != (regs[PCF_CONTROL1] & PCF_HOURS_24)) ||
        (0 != (regs[PCF_SECONDS] & PCF_SECONDS_OSF))) 
    {
        regs[PCF_SECONDS]  &= ~PCF_SECONDS_OSF;
        regs[PCF_CONTROL1] &= ~PCF_HOURS_24;
        DS_PUT(regs);
    }
}

static inline void
set_pcf_hwclock(cyg_uint32 year, cyg_uint32 month, cyg_uint32 mday,
               cyg_uint32 hour, cyg_uint32 minute, cyg_uint32 second)
{
    cyg_uint8 regs[PCF_REGS_SIZE];

    // Set up the registers
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

static inline void
get_pcf_hwclock(cyg_uint32* year, cyg_uint32* month, cyg_uint32* mday,
               cyg_uint32* hour, cyg_uint32* minute, cyg_uint32* second)
{
    cyg_uint8 regs[PCF_REGS_SIZE];

    // Fetch the current state
    DS_GET(regs);

    *year = (cyg_uint32)TO_DEC(regs[PCF_YEAR]);
    // The year field only has the 2 least significant digits :-(
    *year += 2000;
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

//-----------------------------------------------------------------------------
// Functions required for the hardware-driver API.

// Returns the number of seconds elapsed since 1970-01-01 00:00:00.
cyg_uint64 
Cyg_WallClock::get_hw_seconds(void)
{
    cyg_uint32 year, month, mday, hour, minute, second;

    get_pcf_hwclock(&year, &month, &mday, &hour, &minute, &second);
    cyg_uint64 now = _simple_mktime(year, month, mday, hour, minute, second);
    return now;
}

#ifdef CYGSEM_WALLCLOCK_SET_GET_MODE

// Sets the clock. Argument is seconds elapsed since 1970-01-01 00:00:00.
void
Cyg_WallClock::set_hw_seconds( cyg_uint64 secs )
{
    cyg_uint32 year, month, mday, hour, minute, second;

    _simple_mkdate(secs, &year, &month, &mday, &hour, &minute, &second);
    set_pcf_hwclock(year, month, mday, hour, minute, second);
}

#endif

void
Cyg_WallClock::init_hw_seconds(void)
{
#ifdef CYGSEM_WALLCLOCK_SET_GET_MODE
    init_pcf_hwclock();
#else
    // This is our base: 2013-01-01 00:00:00
    // Set the HW clock - if for nothing else, just to be sure it's in a
    // legal range. Any arbitrary base could be used.
    // After this the hardware clock is only read.
    set_pcf_hwclock(2013,1,1,0,0,0);
#endif
}

//-----------------------------------------------------------------------------
// End of pcf2129a.cxx
