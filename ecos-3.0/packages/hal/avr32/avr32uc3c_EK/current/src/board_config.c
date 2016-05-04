//=============================================================================
//
//      board_config.c
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
// Purpose:     Board startup and configuration
// Description: The file contains board initialisation.
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <pkgconf/system.h>
#include <pkgconf/hal_avr32_uc3c_ek.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/testcase.h>           // Test macros
#include <cyg/infra/cyg_ass.h>            // Assertion macros
#include <cyg/infra/diag.h>               // Diagnostic output

#include <cyg/hal/hal_arch.h>             // CYGNUM_HAL_STACK_SIZE_TYPICAL
#include <cyg/hal/hal_if.h>

#include <cyg/io/i2c.h>
#include <cyg/io/i2c_uc3c.h>

#include <cyg/io/spi.h>
#include <cyg/io/spi_avr32.h>
#include <cyg/io/flash.h>

#include <cyg/io/dataflash.h>

#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>
#include <cyg/hal/board_config.h>

//! Unlock PM register macro
#define PM_UNLOCK(reg)                                             \
        (AVR32_PM.unlock = (AVR32_PM_UNLOCK_KEY_VALUE <<           \
                            AVR32_PM_UNLOCK_KEY_OFFSET) | (reg))

//! CPU start up error flags
cyg_uint64	_hw_error = 0;


/**
/* Definition of I2C bus
*/
CYG_AVR32_I2C_BUS(i2c_bus0,&cyg_avr32_i2c_init,
        AVR32_TWIM0_ADDRESS,CYGNUM_HAL_VECTOR_TWIM0,
	I2C_MODULE_INTERRUPT_PRIO,I2C_MODULE_CLOCK_FREQUENCY,
        I2C_MODULE_BUSS_FREQ);
/**
/* Definition of I2C device EEPROM.
*/
CYG_I2C_DEVICE(cyg_i2c_eeprom_at24c128,&i2c_bus0,
        AT24C128_EEPROM_I2C_BUS_ADDRESS,0,0);

/**
* Definition of touch keyboard
*/
CYG_I2C_DEVICE(i2c_dev_touch, &i2c_bus0, AT42QT1060_I2C_BUS_ADDRESS, 0, 1);


cyg_spi_avr32_device_t data_flash_spi_device =
{
    .spi_device     = &cyg_spi_avr32_bus1,
    .dev_num        = FLASH_AT45DBXX_SPI_DEV_CS_NUM,
    .bits           = FLASH_AT45DBXX_SPI_DEV_BITS_NUM,
    .cl_pol         = FLASH_AT45DBXX_SPI_DEV_CPOL,
    .cl_pha         = FLASH_AT45DBXX_SPI_DEV_NCPHA,
    .cl_brate       = FLASH_AT45DBXX_SPI_DEV_BUD_RATE,
    .cs_up_udly     = FLASH_AT45DBXX_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = FLASH_AT45DBXX_SPI_DEV_BYTE_DLY
};

cyg_spi_avr32_device_t cyg_spi_mmc_dev0 =
{
    .spi_device     = &cyg_spi_avr32_bus1,
    .dev_num        = SD_MMC_SPI_DEV_CS_NUM,
    .bits           = SD_MMC_SPI_DEV_BITS_NUM,
    .cl_pol         = SD_MMC_SPI_DEV_CPOL,
    .cl_pha         = SD_MMC_SPI_DEV_NCPHA,
    .cl_brate       = SD_MMC_SPI_DEV_BUD_RATE,
    .cs_up_udly     = SD_MMC_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = SD_MMC_SPI_DEV_BYTE_DLY
};

CYG_DATAFLASH_FLASH_DRIVER(at45dbxx_spi_flash, &data_flash_spi_device, 
		FLASH_AT45DBXX_BASE_ADDRESS, 0, 31);



void hal_board_init()
{
    int i;

    // Initialize LED pins
    gpio_configure_pin(LED0_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(LED1_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(LED2_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(LED3_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    // Initialize push buttons pins
    gpio_configure_pin(PB0_PIN, GPIO_DIR_INPUT);

    gpio_configure_pin(PB1_PIN, GPIO_DIR_INPUT);
    // Initialize keyboard event pin
    gpio_configure_pin(KBD_EVENT_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);
    
    // Disable clock for unused peripherals can be pleaced hear
}					

