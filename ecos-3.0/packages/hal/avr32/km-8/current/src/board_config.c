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

#include <cyg/io/spi.h>
#include <cyg/io/spi_avr32.h>
#include <cyg/io/usart_spi_avr32.h>

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

#include <cyg/io/fm25vxx.h>


/**
/* Definition of SPI device on chip select 1
/* for FM25WXX FRAM chip.
*/
cyg_spi_avr32_device_t fram_spi_device =
{
    .spi_device     = &(cyg_spi_avr32_bus0),
    .dev_num        = FRAM_FM25WXX_SPI_DEV_CS_NUM,
    .bits           = FRAM_FM25WXX_SPI_DEV_BITS_NUM,
    .cl_pol         = FRAM_FM25WXX_SPI_DEV_CPOL,
    .cl_pha         = FRAM_FM25WXX_SPI_DEV_NCPHA,
    .cl_brate       = FRAM_FM25WXX_SPI_DEV_BUD_RATE,
    .cs_up_udly     = FRAM_FM25WXX_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = FRAM_FM25WXX_SPI_DEV_BYTE_DLY
};

/**
/* Definition of SPI device on chip select 3
/* for RTC chip.
*/
cyg_spi_avr32_device_t rtc_spi_device =
{
    .spi_device     = &(cyg_spi_avr32_bus0),
    .dev_num        = RTC_SPI_DEV_CS_NUM,
    .bits           = RTC_SPI_DEV_BITS_NUM,
    .cl_pol         = RTC_SPI_DEV_CPOL,
    .cl_pha         = RTC_SPI_DEV_NCPHA,
    .cl_brate       = RTC_SPI_DEV_BUD_RATE,
    .cs_up_udly     = RTC_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = RTC_SPI_DEV_BYTE_DLY
};

/**
/* Definition of SPI device on chip select 2
/* for LCD.
*/
cyg_spi_avr32_device_t lcd_spi_device =
{
    .spi_device     = &(cyg_spi_avr32_bus0),
    .dev_num        = LCD_SPI_DEV_CS_NUM,
    .bits           = LCD_SPI_DEV_BITS_NUM,
    .cl_pol         = LCD_SPI_DEV_CPOL,
    .cl_pha         = LCD_SPI_DEV_NCPHA,
    .cl_brate       = LCD_SPI_DEV_BUD_RATE,
    .cs_up_udly     = LCD_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = LCD_SPI_DEV_BYTE_DLY
};

externC cyg_usart_spi_avr32_bus_t cyg_usart_spi_avr32_bus1;
/**
/* Definition of SPI for DAC on USART1 SPI.
*/
cyg_usart_spi_avr32_device_t dac_control_spi_device =
{
    .spi_device     = &(cyg_usart_spi_avr32_bus1),
    .dev_num        = DAC_SPI_DEV_CS_NUM,
    .bits           = DAC_SPI_DEV_BITS_NUM,
    .cl_pol         = DAC_SPI_DEV_CPOL,
    .cl_pha         = DAC_SPI_DEV_NCPHA,
    .cl_brate       = DAC_SPI_DEV_BUD_RATE,
    .cs_up_udly     = DAC_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = DAC_SPI_DEV_BYTE_DLY
};

/**
/* Definition of SPI for ADC on USART1 SPI.
*/
cyg_usart_spi_avr32_device_t adc_spi_device =
{
    .spi_device     = &(cyg_usart_spi_avr32_bus1),
    .dev_num        = ADC_SPI_DEV_CS_NUM,
    .bits           = ADC_SPI_DEV_BITS_NUM,
    .cl_pol         = ADC_SPI_DEV_CPOL,
    .cl_pha         = ADC_SPI_DEV_NCPHA,
    .cl_brate       = ADC_SPI_DEV_BUD_RATE,
    .cs_up_udly     = ADC_SPI_DEV_CSUP_DLY,
    .tr_bt_udly     = ADC_SPI_DEV_BYTE_DLY
};


/**
* Definition of FLASH drivers structure for FLASH and FRAM
*/
CYG_DEVS_FLASH_SPI_FM25VXX_DRIVER(fm25wxx_spi_fram,FRAM_FM25WXX_BASE_ADDRESS,&fram_spi_device);

#endif


void hal_board_init(void)
{
    //Vzpnutí celého napájení v jednice
    gpio_configure_pin(SW_POWER_DOWN_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
    
    //LCD backlight 
    gpio_configure_pin(LCD_BACKLIGHT_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    
    //LCD address pin
    gpio_configure_pin(LCD_A0_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    //Reset displeje
    gpio_configure_pin(LCD_RESET_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
    
    //Analog power
    gpio_configure_pin(ANL_PW_EN_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
    
    //Bluetooth modlue shut down
    gpio_configure_pin(BT_NSHUTD_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
    
    
#if 0	
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
}					

