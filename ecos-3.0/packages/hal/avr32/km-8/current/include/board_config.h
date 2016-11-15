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
// FM25W SPI FRAM
#define FRAM_FM25WXX_SPI_DEV_CS_NUM			1
#define FRAM_FM25WXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FRAM_FM25WXX_SPI_DEV_CPOL			0
#define FRAM_FM25WXX_SPI_DEV_NCPHA			1
#define FRAM_FM25WXX_SPI_DEV_BUD_RATE			16000000
#define FRAM_FM25WXX_SPI_DEV_CSUP_DLY			0
#define FRAM_FM25WXX_SPI_DEV_BYTE_DLY			0
#define FRAM_FM25WXX_BASE_ADDRESS			0x20000000

// SPI RTC
#define RTC_SPI_DEV_CS_NUM                              3
#define RTC_SPI_DEV_BITS_NUM                            0 //0 -> 8
#define RTC_SPI_DEV_CPOL                                0
#define RTC_SPI_DEV_NCPHA                               1
#define RTC_SPI_DEV_BUD_RATE                            16000000
#define RTC_SPI_DEV_CSUP_DLY                            0
#define RTC_SPI_DEV_BYTE_DLY                            0

// SPI LCD Display
#define LCD_SPI_DEV_CS_NUM                              2
#define LCD_SPI_DEV_BITS_NUM                            0 //0 -> 8
#define LCD_SPI_DEV_CPOL                                0
#define LCD_SPI_DEV_NCPHA                                1
#define LCD_SPI_DEV_BUD_RATE                            16000000
#define LCD_SPI_DEV_CSUP_DLY                            0
#define LCD_SPI_DEV_BYTE_DLY                            0
#define DISPLAY_NAME                                    lcd_spi_device


// Analog board DAC SPI on USART1
#define DAC_SPI_DEV_CS_NUM                              0
#define DAC_SPI_DEV_BITS_NUM                            8 // USART SPI
#define DAC_SPI_DEV_CPOL                                1
#define DAC_SPI_DEV_NCPHA                               1
#define DAC_SPI_DEV_BUD_RATE                            2000000
#define DAC_SPI_DEV_CSUP_DLY                            0
#define DAC_SPI_DEV_BYTE_DLY                            0

// Analog board ADC config SPI on USART1
#define ADC_SPI_DEV_CS_NUM                              1
#define ADC_SPI_DEV_BITS_NUM                            8 // USART SPI
#define ADC_SPI_DEV_CPOL                                1
#define ADC_SPI_DEV_NCPHA                               1
#define ADC_SPI_DEV_BUD_RATE                            2000000
#define ADC_SPI_DEV_CSUP_DLY                            0
#define ADC_SPI_DEV_BYTE_DLY                            0


/**
/* GPIO pin definition macros
*/

#define LCD_BACKLIGHT_PIN		AVR32_PIN_PB00
#define LCD_A0_PIN                      AVR32_PIN_PB27
#define SW_POWER_DOWN_PIN		AVR32_PIN_PA19
#define LCD_RESET_PIN                   AVR32_PIN_PA11
#define ANL_PW_EN_PIN                   AVR32_PIN_PA17
#define BT_NSHUTD_PIN                   AVR32_PIN_PB10


/**
/* Pin access and manipulation macros
*/


#define SYSTEM_POWER_DOWN					\
			gpio_set_pin_low(SW_POWER_DOWN_PIN)
			
#define LCD_BACKLIGHT_ENABLE					\
			gpio_set_pin_high(LCD_BACKLIGHT_PIN)
			
#define LCD_BACKLIGHT_DISABLE					\
			gpio_set_pin_low(LCD_BACKLIGHT_PIN)
			
#define ANALOG_VOLTAGE_POWER_ENABLE				\
			gpio_set_pin_high(ANL_PW_EN_PIN)
			
#define ANALOG_VOLTAGE_POWER_DISABLE				\
			gpio_set_pin_low(ANL_PW_EN_PIN)


			


void hal_board_init(void);

#define SET_HW_ERRROR(_code_) _hw_error |= _code_

#define HW_ERROR_OSC0_NOT_RUNNING		0x0000000000000001
#define HW_ERROR_OSC1_NOT_RUNNING		0x0000000000000002
#define HW_ERROR_OSC32_NOT_RUNNING		0x0000000000000004
#define HW_ERROR_RC8M_NOT_RUNNING		0x0000000000000008
#define HW_ERROR_MAIN_CLOCK_NOT_RUNNIGN         0x0000000000000010
#define HW_ERROR_DFLL_NO_LOCK                   0x0000000000000400

#define HW_ERROR_RTC_NO_RESPONSE		0x0000000000000020
#define HW_ERROR_RTC_CHECK_TIME			0x0000000000000040

#define HW_ERROR_MAIN_FRAM_NOT_INIT             0x0000000000000080ll
#define HW_ERROR_MAIN_FLASH_NOT_INIT            0x0000000000000100ll
#define HW_ERROR_MOUNT                          0x0000000000000200ll


// Board IO multiplexing configureation
#define PIN_ENABLE                              1
#define PIN_DISABLE                             0
// SPI 0
#define CYG_HAL_AVR32_SPI0_MISO_PIN             AVR32_SPI_MISO_0_2_PIN
#define CYG_HAL_AVR32_SPI0_MISO_FUNCTION        AVR32_SPI_MISO_0_2_FUNCTION

#define CYG_HAL_AVR32_SPI0_MOSI_PIN             AVR32_SPI_MOSI_0_2_PIN
#define CYG_HAL_AVR32_SPI0_MOSI_FUNCTION        AVR32_SPI_MOSI_0_2_FUNCTION

#define CYG_HAL_AVR32_SPI0_SCK_PIN              AVR32_SPI_SCK_0_1_PIN
#define CYG_HAL_AVR32_SPI0_SCK_FUNCTION         AVR32_SPI_SCK_0_1_FUNCTION

#define CYG_HAL_AVR32_SPI0_NPCS_0_ENABLED       PIN_DISABLE
#define CYG_HAL_AVR32_SPI0_NPCS_0_PIN           AVR32_SPI0_NPCS_0_PIN
#define CYG_HAL_AVR32_SPI0_NPCS_0_FUNCTION      AVR32_SPI0_NPCS_0_FUNCTION

#define CYG_HAL_AVR32_SPI0_NPCS_1_ENABLED       PIN_ENABLE
#define CYG_HAL_AVR32_SPI0_NPCS_1_PIN           AVR32_SPI_NPCS_1_1_PIN
#define CYG_HAL_AVR32_SPI0_NPCS_1_FUNCTION      AVR32_SPI_NPCS_1_1_FUNCTION

#define CYG_HAL_AVR32_SPI0_NPCS_2_ENABLED       PIN_ENABLE
#define CYG_HAL_AVR32_SPI0_NPCS_2_PIN           AVR32_SPI_NPCS_2_2_PIN
#define CYG_HAL_AVR32_SPI0_NPCS_2_FUNCTION      AVR32_SPI_NPCS_2_2_FUNCTION

#define CYG_HAL_AVR32_SPI0_NPCS_3_ENABLED       PIN_ENABLE
#define CYG_HAL_AVR32_SPI0_NPCS_3_PIN           AVR32_SPI_NPCS_3_2_PIN
#define CYG_HAL_AVR32_SPI0_NPCS_3_FUNCTION      AVR32_SPI_NPCS_3_2_FUNCTION

// SPI 1
#define CYG_HAL_AVR32_SPI1_MISO_PIN             AVR32_SPI1_MISO_PIN
#define CYG_HAL_AVR32_SPI1_MISO_FUNCTION        AVR32_SPI1_MISO_FUNCTION

#define CYG_HAL_AVR32_SPI1_MOSI_PIN             AVR32_SPI1_MOSI_PIN
#define CYG_HAL_AVR32_SPI1_MOSI_FUNCTION        AVR32_SPI1_MOSI_FUNCTION

#define CYG_HAL_AVR32_SPI1_SCK_PIN              AVR32_SPI1_SCK_PIN
#define CYG_HAL_AVR32_SPI1_SCK_FUNCTION         AVR32_SPI1_SCK_FUNCTION

#define CYG_HAL_AVR32_SPI1_NPCS_0_ENABLED       PIN_DISABLE
#define CYG_HAL_AVR32_SPI1_NPCS_0_PIN           AVR32_SPI1_NPCS_0_PIN
#define CYG_HAL_AVR32_SPI1_NPCS_0_FUNCTION      AVR32_SPI1_NPCS_0_FUNCTION

#define CYG_HAL_AVR32_SPI1_NPCS_1_ENABLED       PIN_DISABLE
#define CYG_HAL_AVR32_SPI1_NPCS_1_PIN           AVR32_SPI1_NPCS_1_PIN
#define CYG_HAL_AVR32_SPI1_NPCS_1_FUNCTION      AVR32_SPI1_NPCS_1_FUNCTION

#define CYG_HAL_AVR32_SPI1_NPCS_2_ENABLED       PIN_DISABLE
#define CYG_HAL_AVR32_SPI1_NPCS_2_PIN           AVR32_SPI1_NPCS_2_PIN
#define CYG_HAL_AVR32_SPI1_NPCS_2_FUNCTION      AVR32_SPI1_NPCS_2_FUNCTION

#define CYG_HAL_AVR32_SPI1_NPCS_3_ENABLED       PIN_DISABLE
#define CYG_HAL_AVR32_SPI1_NPCS_3_PIN           AVR32_SPI1_NPCS_3_PIN
#define CYG_HAL_AVR32_SPI1_NPCS_3_FUNCTION      AVR32_SPI1_NPCS_3_FUNCTION


// USART
// USART0
#define CYG_HAL_USART0_TXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART0_TXD_PIN                  AVR32_USART2_TXD_0_0_PIN
#define CYG_HAL_USART0_TXD_FUNCTION             AVR32_USART2_TXD_0_0_FUNCTION

#define CYG_HAL_USART0_RXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART0_RXD_PIN                  AVR32_USART2_RXD_0_0_PIN
#define CYG_HAL_USART0_RXD_FUNCTION             AVR32_USART2_RXD_0_0_FUNCTION

#define CYG_HAL_USART0_CLK_PIN                  AVR32_USART0_CLK_PIN
#define CYG_HAL_USART0_CLK_FUNCTION             AVR32_USART0_CLK_FUNCTION

#define CYG_HAL_USART0_NPCS_0_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART0_NPCS_0_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART0_NPCS_1_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART0_NPCS_1_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART0_NPCS_2_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART0_NPCS_2_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART0_NPCS_3_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART0_NPCS_3_GPIO              AVR32_PIN_PC10

// handshace signals
#define CYG_HAL_USART0_CTS_PIN                  AVR32_USART0_CTS_PIN
#define CYG_HAL_USART0_CTS_FUNCTION             AVR32_USART0_CTS_FUNCTION

#define CYG_HAL_USART0_RTS_PIN                  AVR32_USART0_RTS_PIN
#define CYG_HAL_USART0_RTS_FUNCTION             AVR32_USART0_RTS_FUNCTION

// USART1 ADC DAC SPI
#define CYG_HAL_USART1_TXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART1_TXD_PIN                  AVR32_USART1_TXD_0_3_PIN
#define CYG_HAL_USART1_TXD_FUNCTION             AVR32_USART1_TXD_0_3_FUNCTION

#define CYG_HAL_USART1_RXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART1_RXD_PIN                  AVR32_USART1_RXD_0_4_PIN
#define CYG_HAL_USART1_RXD_FUNCTION             AVR32_USART1_RXD_0_4_FUNCTION

#define CYG_HAL_USART1_CLK_PIN                  AVR32_USART1_CLK_0_0_PIN
#define CYG_HAL_USART1_CLK_FUNCTION             AVR32_USART1_CLK_0_0_FUNCTION

#define CYG_HAL_USART1_NPCS_0_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART1_NPCS_0_GPIO              AVR32_PIN_PB11

#define CYG_HAL_USART1_NPCS_1_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART1_NPCS_1_GPIO              AVR32_PIN_PB12

#define CYG_HAL_USART1_NPCS_2_GPIO_EABLED       PIN_DISABLE
#define CYG_HAL_USART1_NPCS_2_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART1_NPCS_3_GPIO_EABLED       PIN_DISABLE
#define CYG_HAL_USART1_NPCS_3_GPIO              AVR32_PIN_PC10

// handshace signals
#define CYG_HAL_USART1_CTS_PIN                  AVR32_USART1_CTS_PIN
#define CYG_HAL_USART1_CTS_FUNCTION             AVR32_USART1_CTS_FUNCTION

#define CYG_HAL_USART1_RTS_PIN                  AVR32_USART1_RTS_PIN
#define CYG_HAL_USART1_RTS_FUNCTION             AVR32_USART1_RTS_FUNCTION

// modem mode signals only for USART1 in UC3C
#define CYG_HAL_USART1_DTR_PIN                  AVR32_USART1_DTR_PIN
#define CYG_HAL_USART1_DTR_FUNCTION             AVR32_USART1_DTR_FUNCTION

#define CYG_HAL_USART1_DSR_PIN                  AVR32_USART1_DSR_PIN
#define CYG_HAL_USART1_DSR_FUNCTION             AVR32_USART1_DSR_FUNCTION

#define CYG_HAL_USART1_DCD_PIN                  AVR32_USART1_DCD_PIN
#define CYG_HAL_USART1_DCD_FUNCTION             AVR32_USART1_DCD_FUNCTION

#define CYG_HAL_USART1_RI_PIN                   AVR32_USART1_RI_PIN
#define CYG_HAL_USART1_RI_FUNCTION              AVR32_USART1_RI_FUNCTION

// USART2 DBG
#define CYG_HAL_USART2_TXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART2_TXD_PIN                  AVR32_USART2_TXD_0_0_PIN
#define CYG_HAL_USART2_TXD_FUNCTION             AVR32_USART2_TXD_0_0_FUNCTION

#define CYG_HAL_USART2_RXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART2_RXD_PIN                  AVR32_USART2_RXD_0_0_PIN
#define CYG_HAL_USART2_RXD_FUNCTION             AVR32_USART2_RXD_0_0_FUNCTION

#define CYG_HAL_USART2_CLK_PIN                  AVR32_USART2_CLK_PIN
#define CYG_HAL_USART2_CLK_FUNCTION             AVR32_USART2_CLK_FUNCTION

#define CYG_HAL_USART2_NPCS_0_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART2_NPCS_0_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART2_NPCS_1_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART2_NPCS_1_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART2_NPCS_2_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART2_NPCS_2_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART2_NPCS_3_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART2_NPCS_3_GPIO              AVR32_PIN_PC10

// handshace signals
#define CYG_HAL_USART2_CTS_PIN                  AVR32_USART2_CTS_PIN
#define CYG_HAL_USART2_CTS_FUNCTION             AVR32_USART2_CTS_FUNCTION

#define CYG_HAL_USART2_RTS_PIN                  AVR32_USART2_RTS_PIN
#define CYG_HAL_USART2_RTS_FUNCTION             AVR32_USART2_RTS_FUNCTION

// USART3 BT
#define CYG_HAL_USART3_TXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART3_TXD_PIN                  AVR32_USART3_TXD_0_1_PIN
#define CYG_HAL_USART3_TXD_FUNCTION             AVR32_USART3_TXD_0_1_FUNCTION

#define CYG_HAL_USART3_RXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART3_RXD_PIN                  AVR32_USART3_RXD_0_1_PIN
#define CYG_HAL_USART3_RXD_FUNCTION             AVR32_USART3_RXD_0_1_FUNCTION

#define CYG_HAL_USART3_CLK_PIN                  AVR32_USART3_CLK_PIN
#define CYG_HAL_USART3_CLK_FUNCTION             AVR32_USART3_CLK_FUNCTION

#define CYG_HAL_USART3_NPCS_0_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART3_NPCS_0_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART3_NPCS_1_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART3_NPCS_1_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART3_NPCS_2_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART3_NPCS_2_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART3_NPCS_3_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART3_NPCS_3_GPIO              AVR32_PIN_PC10

// handshace signals
#define CYG_HAL_USART3_CTS_PIN                  AVR32_USART3_CLK_0_2_PIN
#define CYG_HAL_USART3_CTS_FUNCTION             AVR32_USART3_CLK_0_2_FUNCTION

#define CYG_HAL_USART3_RTS_PIN                  AVR32_USART3_RTS_0_1_PIN
#define CYG_HAL_USART3_RTS_FUNCTION             AVR32_USART3_RTS_0_1_FUNCTION

// USART4
#define CYG_HAL_USART4_TXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART4_TXD_PIN                  AVR32_USART4_TXD_PIN
#define CYG_HAL_USART4_TXD_FUNCTION             AVR32_USART4_TXD_FUNCTION

#define CYG_HAL_USART4_RXD_ENABLED              PIN_ENABLE
#define CYG_HAL_USART4_RXD_PIN                  AVR32_USART4_RXD_PIN
#define CYG_HAL_USART4_RXD_FUNCTION             AVR32_USART4_RXD_FUNCTION

#define CYG_HAL_USART4_CLK_PIN                  AVR32_USART4_CLK_PIN
#define CYG_HAL_USART4_CLK_FUNCTION             AVR32_USART4_CLK_FUNCTION

#define CYG_HAL_USART4_NPCS_0_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART4_NPCS_0_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART4_NPCS_1_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART4_NPCS_1_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART4_NPCS_2_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART4_NPCS_2_GPIO              AVR32_PIN_PC10

#define CYG_HAL_USART4_NPCS_3_GPIO_EABLED       PIN_ENABLE
#define CYG_HAL_USART4_NPCS_3_GPIO              AVR32_PIN_PC10

// handshace signals
#define CYG_HAL_USART4_CTS_PIN                  AVR32_USART4_CTS_PIN
#define CYG_HAL_USART4_CTS_FUNCTION             AVR32_USART4_CTS_FUNCTION

#define CYG_HAL_USART4_RTS_PIN                  AVR32_USART4_RTS_PIN
#define CYG_HAL_USART4_RTS_FUNCTION             AVR32_USART4_RTS_FUNCTION

// TWIM IIC
#define CYG_HAL_AVR32_TWIMS0_TWD_PIN            AVR32_TWIMS0_TWD_PIN
#define CYG_HAL_AVR32_TWIMS0_TWD_FUNCTION       AVR32_TWIMS0_TWD_FUNCTION

#define CYG_HAL_AVR32_TWIMS0_TWCK_PIN           AVR32_TWIMS0_TWCK_PIN
#define CYG_HAL_AVR32_TWIMS0_TWCK_FUNCTION      AVR32_TWIMS0_TWCK_FUNCTION
#endif /* BOARD_CONFIG_H_ */
