/*
 * pcf_wallclock.h
 *
 * Created: 5.6.2013 15:59:18
 *  Author: Filip
 */ 


#ifndef PCF_WALLCLOCK_H_
#define PCF_WALLCLOCK_H_
#ifdef __cplusplus
extern "C" {
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

void init_pcf_hwclock2(void);

void
set_pcf_hwclock(cyg_uint32 year, cyg_uint32 month, cyg_uint32 mday,
cyg_uint32 hour, cyg_uint32 minute, cyg_uint32 second);

void
get_pcf_hwclock(cyg_uint32* year, cyg_uint32* month, cyg_uint32* mday,
cyg_uint32* hour, cyg_uint32* minute, cyg_uint32* second);


#ifdef __cplusplus
}
#endif
#endif /* PCF_WALLCLOCK_H_ */