/*
 * pcf_wallclok.c
 *
 * Created: 5.6.2013 15:59:00
 *  Author: Filip
 */ 
#ifndef BOOT_LOADER
#include <cyg/kernel/kapi.h>

#include <time.h>
#include <cyg/libc/time/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <pkgconf/hal.h>


#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_type.h>

#include <cyg/hal/board_config.h>
#include <cyg/hal/pcf_wallclock.h>

#include <string.h>                     // memcpy()

#include <cyg/infra/diag.h>
#include <cyg/io/wallclock/wallclock.inl>

#if 1
# define DEBUG(_format_, ...)
#else
# define DEBUG(_format_, ...) diag_printf(_format_, ## __VA_ARGS__)
#endif

#define SPI             0
#define IIC             1
#define PCF_INTERFACE   SPI

#if PCF_INTERFACE == SPI
#include <cyg/io/spi.h>
extern cyg_spi_device cyg_spi_wallclock_pcf2129a;
#else
#include <cyg/io/i2c.h>
extern cyg_i2c_device cyg_i2c_wallclock_pcf2129a;
#endif
extern cyg_uint64	  _hw_error;


static void
pcf_write_registers(cyg_uint8 register_offset, cyg_uint8 cnt, cyg_uint8* regs)
{
    cyg_uint8 tx_data[PCF_REGS_SIZE + 1];
    memcpy(&(tx_data[1]), regs, cnt);
#if PCF_INTERFACE == SPI
    tx_data[0] = PCF_SPI_WRITE | (register_offset&0x1f);
    cyg_spi_transaction_begin(&cyg_spi_wallclock_pcf2129a);
    cyg_spi_transaction_transfer(&cyg_spi_wallclock_pcf2129a,false,cnt + 1,
            tx_data,NULL,true);
    cyg_spi_transaction_end(&cyg_spi_wallclock_pcf2129a);
#else	
    tx_data[0] = register_offset;
    cyg_i2c_transaction_begin(&cyg_i2c_wallclock_pcf2129a);
    cyg_i2c_transaction_tx(&cyg_i2c_wallclock_pcf2129a, true, tx_data, cnt + 1, true);
    cyg_i2c_transaction_end(&cyg_i2c_wallclock_pcf2129a);
#endif
}

static int
pcf_read_registers(cyg_uint8 register_offset, cyg_uint8 cnt, cyg_uint8* regs)
{
    int ret = 0;
    cyg_uint8 tx_data[1];
#if PCF_INTERFACE == SPI
    tx_data[0] = PCF_SPI_READ | (register_offset&0x1f);
    cyg_spi_transaction_begin(&cyg_spi_wallclock_pcf2129a);
    cyg_spi_transaction_transfer(&cyg_spi_wallclock_pcf2129a,false,1,
            tx_data,NULL,false);
    
    cyg_spi_transaction_transfer(&cyg_spi_wallclock_pcf2129a,false,cnt,
            NULL,regs,true);
    cyg_spi_transaction_end(&cyg_spi_wallclock_pcf2129a);
#else   
    tx_data[0] = register_offset;

    cyg_i2c_transaction_begin(&cyg_i2c_wallclock_pcf2129a);

    if(cyg_i2c_transaction_tx(&cyg_i2c_wallclock_pcf2129a, true, tx_data, 1, true) != 1)
        ret = 1;
    else
        cyg_i2c_transaction_rx(&cyg_i2c_wallclock_pcf2129a, true, regs, cnt, true, true);

    cyg_i2c_transaction_end(&cyg_i2c_wallclock_pcf2129a);
#endif
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
		
		//DS_PUT(regs);
		
    }
}


//----------------------------------------------------------------------------
// Accessor functions

void
init_pcf_hwclock2(void)
{
    cyg_uint32 year, month, mday, hour, minute, second;
    cyg_uint64 now;
    cyg_uint8 regs[PCF_REGS_SIZE+1];
    cyg_bool    ok = true;

    if (pcf_read_registers(PCF_CONTROL1,10,regs) != 0)
    {
        // The device has not responded to the address byte.
        ok = false;
        SET_HW_ERRROR(HW_ERROR_RTC_NO_RESPONSE);
        DEBUG("Cannto conntect to PCF2129A\n");
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

            DEBUG("Time int the PCF2129A corrupted\n");
        SET_HW_ERRROR(HW_ERROR_RTC_CHECK_TIME);
	    #include <cyg/io/spi.h>

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
    cyg_libc_time_settime(now);	
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

void
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
#endif