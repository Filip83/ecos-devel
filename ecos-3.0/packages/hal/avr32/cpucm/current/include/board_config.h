//=============================================================================
//
//      board_config.h
//
//      Board configuration file.
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
// Date:        2012-11-17
// Purpose:     Board startup and configuration.
// Description: The file contains board initialisation.
// 
// Usage:
//              #include <cyg/hal/board_config.h>
//              ...
//
//
//####DESCRIPTIONEND####
//
//=============================================================================
#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_


/**
/* cphol and ncpha configuration see documentation page 660
*/
// N25Q SPI flash
#define FLASH_N25QXX_SPI_DEV_CS_NUM			0
#define FLASH_N25QXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FLASH_N25QXX_SPI_DEV_CPOL			0
#define FLASH_N25QXX_SPI_DEV_NCPHA			1
#define FLASH_N25QXX_SPI_DEV_BUD_RATE			16000000
#define FLASH_N25QXX_SPI_DEV_CSUP_DLY			0
#define FLASH_N25QXX_SPI_DEV_BYTE_DLY			0
#define FLASH_N25QXX_BASE_ADDRESS			0x20008000

// FM25W SPI FRAM
#define FRAM_FM25WXX_SPI_DEV_CS_NUM			2
#define FRAM_FM25WXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FRAM_FM25WXX_SPI_DEV_CPOL			0
#define FRAM_FM25WXX_SPI_DEV_NCPHA			1
#define FRAM_FM25WXX_SPI_DEV_BUD_RATE			16000000
#define FRAM_FM25WXX_SPI_DEV_CSUP_DLY			0
#define FRAM_FM25WXX_SPI_DEV_BYTE_DLY			0
#define FRAM_FM25WXX_BASE_ADDRESS			0x20000000

// AT48DB SPI FPGA configuration FLASH
#define FLASH_AT45DBXX_SPI_DEV_CS_NUM			0
#define FLASH_AT45DBXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FLASH_AT45DBXX_SPI_DEV_CPOL			0
#define FLASH_AT45DBXX_SPI_DEV_NCPHA			1
#define FLASH_AT45DBXX_SPI_DEV_BUD_RATE			16000000
#define FLASH_AT45DBXX_SPI_DEV_CSUP_DLY			0
#define FLASH_AT45DBXX_SPI_DEV_BYTE_DLY			0
#define FLASH_AT45DBXX_BASE_ADDRESS			0x30000000

// SPI LCD Display
#define LCD_SPI_DEV_CS_NUM                              0
#define LCD_SPI_DEV_BITS_NUM                            8 // USART SPI
#define LCD_SPI_DEV_CPOL                                0
#define LCD_SPI_DEV_NCPHA                               1
#define LCD_SPI_DEV_BUD_RATE                            20000000
#define LCD_SPI_DEV_CSUP_DLY                            0
#define LCD_SPI_DEV_BYTE_DLY                            0


/**
/* I2C module settings
*/
#define I2C_MODULE_CLOCK_FREQUENCY		(CYGHWR_HAL_AVR32_CPU_FREQ*1e6)
#define I2C_MODULE_INTERRUPT_PRIO		0
#define I2C_MODULE_BUSS_FREQ			10e3

#define WALLCLOCK_RTC_I2C_BUS_ADDRESS		0x51
#define AUDO_ADC_CONTROL_IF_I2C_BUS_ADDRESS     0x18


/**
/* GPIO pin definition macros
*/
#define BT_PW_EN_PIN			AVR32_PIN_PB13
#define LCD_BACKLIGHT_PIN		AVR32_PIN_PB22
#define HV_PWEN_PIN			AVR32_PIN_PB23
#define FPGA_RESET_PIN			AVR32_PIN_PC00
#define FPGA_RDY_PIN 			AVR32_PIN_PC17
#define SW_POWER_DOWN_PIN		AVR32_PIN_PD22
#define FPGA_PWEN_PIN			AVR32_PIN_PD23
#define ANL_PW_EN_PIN			AVR32_PIN_PB21
#define FPGA_SUSPEND_PIN		AVR32_PIN_PB26
#define FPGA_DUAL_BOOT_PIN		AVR32_PIN_PD30
#define DAC_AMPLIFIER_EN_PIN            AVR32_PIN_PB19


/**
/* Pin access and manipulation macros
*/


#define SYSTEM_POWER_DOWN					\
			gpio_set_pin_low(SW_POWER_DOWN_PIN)
			
#define LCD_BACKLIGHT_ENABLE					\
			gpio_set_pin_high(LCD_BACKLIGHT_PIN)
			
#define LCD_BACKLIGHT_DISABLE					\
			gpio_set_pin_low(LCD_BACKLIGHT_PIN)
			
#define BLUETOOTH_POWER_ENABLE					\
			gpio_set_pin_high(BT_PW_EN_PIN)
			
#define BLUETOOTH_POWER_DISABLE					\
			gpio_set_pin_low(BT_PW_EN_PIN)
						
#define HIGH_VOLTAGE_POWER_ENABLE				\
			gpio_set_pin_high(HV_PWEN_PIN)
			
#define HIGH_VOLTAGE_POWER_DISABLE				\
			gpio_set_pin_low(HV_PWEN_PIN)
			
#define ANALOG_VOLTAGE_POWER_ENABLE				\
			gpio_set_pin_high(ANL_PW_EN_PIN)
			
#define ANALOG_VOLTAGE_POWER_DISABLE				\
			gpio_set_pin_low(ANL_PW_EN_PIN)
			
#define FPGA_SUSPEND_ENABLE				        \
			gpio_set_pin_high(FPGA_SUSPEND_PIN)
			
#define FPGA_SUSPEND_DISABLLE				        \
			gpio_set_pin_low(FPGA_SUSPEND_PIN)

#define FPGA_POWER_ENABLE					\
			gpio_set_pin_low(FPGA_PWEN_PIN)
			
#define FPGA_POWER_DISABLE					\
			gpio_set_pin_high(FPGA_PWEN_PIN)
			
#define FPGA_DUAL_BOOT_START					\
			gpio_set_pin_high(FPGA_DUAL_BOOT_PIN)
			
#define FPGA_DUAL_BOOT_STOP					\
			gpio_set_pin_low(FPGA_DUAL_BOOT_PIN)
			
#define FPGA_OSC_ENABLE	

#define FPGA_OCS_DISABLE
			
#define FPGA_RESET_ENABLE					\
			gpio_set_pin_low(FPGA_RESET_PIN)
			
			
#define FPGA_RESET_DISABLE					\
			gpio_set_pin_high(FPGA_RESET_PIN)
			
#define BEEPER_AMPLIFIER_DISABLE		                \
		       gpio_set_pin_low(DAC_AMPLIFIER_EN_PIN)
		 
#define BEEPER_AMPLIFIER_ENABLE		                        \
		       gpio_set_pin_high(DAC_AMPLIFIER_EN_PIN)
			
#define FPGA_RESET(reset) (reset) ? FPGA_RESET_ENABLE : FPGA_RESET_DISABLE

#define IS_FPGA_CONFIGURED	gpio_get_pin_value(FPGA_RDY_PIN)

#define FPGA_SMC_CHIP_SELCET_BASE_ADDRESS 0xD0000000

void hal_board_init();

#define SET_HW_ERRROR(_code_) _hw_error |= _code_

#define HW_ERROR_OSC0_NOT_RUNNING		0x0000000000000001
#define HW_ERROR_OSC1_NOT_RUNNING		0x0000000000000002
#define HW_ERROR_OSC32_NOT_RUNNING		0x0000000000000004
#define HW_ERROR_RC8M_NOT_RUNNING		0x0000000000000008
#define HW_ERROR_MAIN_CLOCK_NOT_RUNNIGN         0x0000000000000010

#define HW_ERROR_RTC_NO_RESPONSE		0x0000000000000020
#define HW_ERROR_RTC_CHECK_TIME			0x0000000000000040

#define HW_ERROR_MAIN_FRAM_NOT_INIT             0x0000000000000080ll
#define HW_ERROR_MAIN_FLASH_NOT_INIT            0x0000000000000100ll
#define HW_ERROR_MOUNT                          0x0000000000000200ll


#endif /* BOARD_CONFIG_H_ */
