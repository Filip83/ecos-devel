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
#include CYGBLD_HAL_PLATFORM_H

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
#include <cyg/io/dma_usart_spi_avr32.h>
#include <cyg/io/flash.h>

#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>
#include <cyg/hal/board_config.h>

//! Unlock PM register macro
#define PM_UNLOCK(reg)                                             \
        (AVR32_PM.unlock = (AVR32_PM_UNLOCK_KEY_VALUE <<           \
                            AVR32_PM_UNLOCK_KEY_OFFSET) | (reg))

//! CPU start up error flags
cyg_uint64	_hw_error = 0;


#ifndef BOOT_LOADER

#include <cyg/io/i2c.h>
#include <cyg/io/i2c_uc3c.h>

#include <cyg/io/fm25vxx.h>
#include <cyg/io/n25qxx.h>
/**
/* Definition of SPI device on chip select 0
/* for NX25QXX FLASH chip.
*/
cyg_spi_avr32_device_t flash_spi_device =
{
    .spi_device     = &(cyg_spi_avr32_bus1),
    .dev_num        = FLASH_N25QXX_SPI_DEV_CS_NUM,
    .bits           = FLASH_N25QXX_SPI_DEV_BITS_NUM,
    .cl_pol         = FLASH_N25QXX_SPI_DEV_CPOL,
    .cl_pha         = FLASH_N25QXX_SPI_DEV_NCPHA,
    .cl_brate       = FLASH_N25QXX_SPI_DEV_BUD_RATE,
    .cs_up_udly     = FLASH_N25QXX_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = FLASH_N25QXX_SPI_DEV_BYTE_DLY
};

/**
/* Definition of SPI device on chip select 2
/* for FM25WXX FRAM chip.
*/
cyg_spi_avr32_device_t fram_spi_device =
{
    .spi_device     = &(cyg_spi_avr32_bus1),
    .dev_num        = FRAM_FM25WXX_SPI_DEV_CS_NUM,
    .bits           = FRAM_FM25WXX_SPI_DEV_BITS_NUM,
    .cl_pol         = FRAM_FM25WXX_SPI_DEV_CPOL,
    .cl_pha         = FRAM_FM25WXX_SPI_DEV_NCPHA,
    .cl_brate       = FRAM_FM25WXX_SPI_DEV_BUD_RATE,
    .cs_up_udly     = FRAM_FM25WXX_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = FRAM_FM25WXX_SPI_DEV_BYTE_DLY
};

/**
/* Definition of SPI device on chip select 0
/* for ADC chip.
*/
cyg_spi_avr32_device_t adc_spi_device =
{
    .spi_device     = &(cyg_spi_avr32_bus0),
    .dev_num        = 0,
    .bits           = FRAM_FM25WXX_SPI_DEV_BITS_NUM,
    .cl_pol         = 1,
    .cl_pha         = 0,
    .cl_brate       = 1000000,
    .cs_up_udly     = 0,
    .tr_bt_udly     = 0
};

/**
/* Definition of SPI LCD on USART SPI with DMA.
*/
cyg_dma_usart_spi_avr32_device_t EADOGXL240W_7_spi_device =
{
    .spi_device     = &(cyg_dma_usart_spi_avr32_bus0),
    .dev_num        = LCD_SPI_DEV_CS_NUM,
    .bits           = LCD_SPI_DEV_BITS_NUM,
    .cl_pol         = LCD_SPI_DEV_CPOL,
    .cl_pha         = LCD_SPI_DEV_NCPHA,
    .cl_brate       = LCD_SPI_DEV_BUD_RATE,
    .cs_up_udly     = LCD_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = LCD_SPI_DEV_BYTE_DLY
};

/**
* Definition of FLASH drivers structure for FLASH and FRAM
*/
CYG_DEVS_FLASH_SPI_FM25VXX_DRIVER(fm25wxx_spi_fram,FRAM_FM25WXX_BASE_ADDRESS,&fram_spi_device);
CYG_DEVS_FLASH_SPI_N25QXX_DRIVER(n25qxx_spi_flash,FLASH_N25QXX_BASE_ADDRESS,&flash_spi_device);

/**
/* Definition of I2C bus
*/
CYG_AVR32_I2C_BUS(i2c_bus0,&cyg_avr32_i2c_init,AVR32_TWIM0_ADDRESS,
                                CYGNUM_HAL_VECTOR_TWIM0,
				I2C_MODULE_INTERRUPT_PRIO,
                                I2C_MODULE_CLOCK_FREQUENCY,
                                I2C_MODULE_BUSS_FREQ);
/**
/* Definition of I2C device for external RCT.
*/
CYG_I2C_DEVICE(cyg_i2c_wallclock_pcf2129a,(struct cyg_i2c_bus*)&i2c_bus0,WALLCLOCK_RTC_I2C_BUS_ADDRESS,0,0);

/**
/* Definition of I2C device for audio ADC configuration.
*/
CYG_I2C_DEVICE(cyg_i2c_adc_control,(struct cyg_i2c_bus*)&i2c_bus0,AUDO_ADC_CONTROL_IF_I2C_BUS_ADDRESS,0,0);
#endif

#ifdef BOOT_LOADER
#include <cyg/io/dataflash.h>

cyg_spi_avr32_device_t data_flash_spi_device =
{
    .spi_device     = &cyg_spi_avr32_bus0,
    .dev_num        = FLASH_AT45DBXX_SPI_DEV_CS_NUM,
    .bits           = FLASH_AT45DBXX_SPI_DEV_BITS_NUM,
    .cl_pol         = FLASH_AT45DBXX_SPI_DEV_CPOL,
    .cl_pha         = FLASH_AT45DBXX_SPI_DEV_NCPHA,
    .cl_brate       = FLASH_AT45DBXX_SPI_DEV_BUD_RATE,
    .cs_up_udly     = FLASH_AT45DBXX_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = FLASH_AT45DBXX_SPI_DEV_BYTE_DLY
};

CYG_DATAFLASH_FLASH_DRIVER(at45dbxx_spi_flash, &data_flash_spi_device, 
		FLASH_AT45DBXX_BASE_ADDRESS, 0, 15);

#endif

void hal_board_init()
{
    cyg_uint32 *clrreg = (cyg_uint32*)0xA0000000;
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[1];
    //Na vývodu je externí pull down
    //a pull-up je zpanuty při spuštění proto se njprve vypne
    //a potom nastaví jako výstup a naství na nulu
    gpio_configure_pin(BT_PW_EN_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    gpio_configure_pin(FPGA_PWEN_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(ANL_PW_EN_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    //gpio_configure_pin(FPGA_SUSPEND_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW); 

    gpio_configure_pin(FPGA_DUAL_BOOT_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    gpio_configure_pin(HV_PWEN_PIN,   GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    gpio_configure_pin(FPGA_RESET_PIN,GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    gpio_configure_pin(LCD_BACKLIGHT_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(DAC_AMPLIFIER_EN_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    //Vzpnutí celého napájení v nule
    gpio_configure_pin(SW_POWER_DOWN_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(FPGA_RDY_PIN, GPIO_PULL_DOWN | GPIO_DIR_INPUT | GPIO_DRIVE_LOW);

    //gpio_configure_pin(OSC_FPGA_EN, GPIO_PULL_DOWN | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    //Bluetooth USART
    gpio_configure_pin(AVR32_PIN_PB11, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
    gpio_configure_pin(AVR32_PIN_PB10, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
    gpio_configure_pin(AVR32_USART1_TXD_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
    gpio_configure_pin(AVR32_USART1_RXD_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
    gpio_configure_pin(AVR32_USART1_RTS_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
    gpio_configure_pin(AVR32_USART1_CTS_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);

    //natavení vývodu zběrnice. Vývody jsou ponechány jako vstupy po resetu, ale jsou
    //zapnuty pull-down odpory aby se nemohlo FPGA napájet z vstupu. 11000101

    gpio_port = &AVR32_GPIO.port[2];
    gpio_port->puerc = 0xfff80000;
    gpio_port->pders = 0xfff80000;

    gpio_port = &AVR32_GPIO.port[3];

    gpio_port->puerc = 0x1e0fdfff;
    gpio_port->pders = 0x1e0fdfff;
    
    FPGA_POWER_ENABLE;
    FPGA_OSC_ENABLE;
    FPGA_RESET_DISABLE;
	
#if 0
#ifdef BOOT_LOADER
    PM_UNLOCK(AVR32_PM_HSBMASK);
    AVR32_PM.hsbmask = 0x7ea;

    PM_UNLOCK(AVR32_PM_PBAMASK);
    AVR32_PM.pbamask = 0x1baf;

    PM_UNLOCK(AVR32_PM_PBBMASK);
    AVR32_PM.pbbmask = 0x1f;

    PM_UNLOCK(AVR32_PM_PBCMASK);
#ifdef DEBUG_IO
	AVR32_PM.pbcmask = 	0xcd;//0x45;
#else
    AVR32_PM.pbcmask = 	0x4d;//0x45;
#endif
    FPGA_POWER_ENABLE;
    FPGA_OSC_ENABLE;
    FPGA_RESET_DISABLE;
#else
    //disable clock for unused peripherals
    PM_UNLOCK(AVR32_PM_HSBMASK);
    AVR32_PM.hsbmask = 0x7ea;

    PM_UNLOCK(AVR32_PM_PBAMASK);   
    AVR32_PM.pbamask = 0x1baf | 0x800000 | 0x1000000;

    PM_UNLOCK(AVR32_PM_PBBMASK);
    AVR32_PM.pbbmask = 0x1f;

    PM_UNLOCK(AVR32_PM_PBCMASK);
#ifdef DEBUG_IO
    AVR32_PM.pbcmask = 	0xc5;//0x45; //poak asi vypnout i dbg usart
#else
    AVR32_PM.pbcmask = 	0x45;//0x45;
#endif
#endif
#endif
}					

