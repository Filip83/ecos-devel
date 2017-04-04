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
#define FLASH_AT45DBXX_SPI_DEV_CS_NUM			1
#define FLASH_AT45DBXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FLASH_AT45DBXX_SPI_DEV_CPOL			0
#define FLASH_AT45DBXX_SPI_DEV_NCPHA			1
#define FLASH_AT45DBXX_SPI_DEV_BUD_RATE			1000000
#define FLASH_AT45DBXX_SPI_DEV_CSUP_DLY			0
#define FLASH_AT45DBXX_SPI_DEV_BYTE_DLY			0
#define FLASH_AT45DBXX_BASE_ADDRESS			0x30000000

/**
/* cphol and ncpha configuration see documentation page 660
*/
#define SD_MMC_SPI_DEV_CS_NUM			3
#define SD_MMC_SPI_DEV_BITS_NUM			0 //0 -> 8
#define SD_MMC_SPI_DEV_CPOL			0
#define SD_MMC_SPI_DEV_NCPHA			1
#define SD_MMC_SPI_DEV_BUD_RATE			1000000
#define SD_MMC_SPI_DEV_CSUP_DLY			0
#define SD_MMC_SPI_DEV_BYTE_DLY			0

/**
/* I2C module settings
*/
#define I2C_MODULE_CLOCK_FREQUENCY		(CYGHWR_HAL_AVR32_CPU_FREQ*1e6)
#define I2C_MODULE_INTERRUPT_PRIO		0
#define I2C_MODULE_BUSS_FREQ			10e3

#define AT24C128_EEPROM_I2C_BUS_ADDRESS		0x50
#define AT42QT1060_I2C_BUS_ADDRESS              0x12


/**
/* GPIO pin definition macros
*/
#define LED0_PIN			AVR32_PIN_PA08
#define LED1_PIN			AVR32_PIN_PD23
#define LED2_PIN			AVR32_PIN_PC13
#define LED3_PIN			AVR32_PIN_PD22

#define PB0_PIN         		AVR32_PIN_PA14
#define PB1_PIN         		AVR32_PIN_PA29

#define KBD_EVENT_PIN                   AVR32_PIN_PC07


/**
/* Pin access and manipulation macros
*/

#define LED0_ENABLE               \
                        gpio_set_pin_low(LED0_PIN)

#define LED0_DISABLE               \
                        gpio_set_pin_high(LED0_PIN)

#define LED1_ENABLE               \
                        gpio_set_pin_low(LED1_PIN)

#define LED1_DISABLE               \
                        gpio_set_pin_high(LED1_PIN)

#define LED2_ENABLE               \
                        gpio_set_pin_low(LED2_PIN)

#define LED2_DISABLE               \
                        gpio_set_pin_high(LED2_PIN)

#define LED3_ENABLE               \
                        gpio_set_pin_low(LED3_PIN)

#define LED3_DISABLE               \
                        gpio_set_pin_high(LED3_PIN)

#define PB_ACTIVE       0

#define GET_PB0_STATE	gpio_get_pin_value(PB0_PIN)
#define GET_PB1_STATE	gpio_get_pin_value(PB1_PIN)

#define IS_PB0_ACTIVE   (GET_PB0_STATE == PB_ACTIVE) ? true:false
#define IS_PB1_ACTIVE   (GET_PB1_STATE == PB_ACTIVE) ? true:false
#define IS_KBD_ACTIVE   (gpio_get_pin_value(KBD_EVENT_PIN) == 0) ? true:false


void hal_board_init();

#define SET_HW_ERRROR(_code_) _hw_error |= _code_

#define HW_ERROR_OSC0_NOT_RUNNING		0x0000000000000001
#define HW_ERROR_OSC1_NOT_RUNNING		0x0000000000000002
#define HW_ERROR_OSC32_NOT_RUNNING		0x0000000000000004
#define HW_ERROR_RC8M_NOT_RUNNING		0x0000000000000008
#define HW_ERROR_MAIN_CLOCK_NOT_RUNNIGN         0x0000000000000010

#define HW_ERROR_RTC_NO_RESPONSE		0x0000000000000020
#define HW_ERROR_RTC_CHECK_TIME			0x0000000000000040

#endif /* BOARD_CONFIG_H_ */
