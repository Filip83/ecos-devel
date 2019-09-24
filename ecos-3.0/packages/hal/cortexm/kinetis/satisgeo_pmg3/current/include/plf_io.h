#ifndef CYGONCE_HAL_PLF_IO_H
#define CYGONCE_HAL_PLF_IO_H
//=============================================================================
//
//      plf_io.h
//
//      Platform specific registers
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2011 Free Software Foundation, Inc.                        
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
// Date:        2018-03-14
// Purpose:     FRDM-K64F platform specific registers
// Description:
// Usage:       #include <cyg/hal/plf_io.h>
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <pkgconf/hal.h>
#include <pkgconf/hal_cortexm_kinetis_sg_pmg3.h>

//===========================================================================
// Cortex-M architecture
//---------------------------------------------------------------------------
//--------------------------------------------------------------------------
// Cortex-M architecture overrides
//---------------------------------------------------------------------------
// VTOR - Vector Table Offset Register

#define CYGARC_REG_NVIC_VTOR_TBLBASE_SRAM (0x1FFF0000)

// DMA MUX ------------------------------------------------------------------
// DMAMUX DMA request sources
#define KinetisDMAMUXDefined
#define FREESCALE_DMAMUX_SRC_KINETIS_DISABLE      0
#define FREESCALE_DMAMUX_SRC_KINETIS_RESERVE      1
#define FREESCALE_DMAMUX_SRC_KINETIS_UART0R       2
#define FREESCALE_DMAMUX_SRC_KINETIS_UART0T       3
#define FREESCALE_DMAMUX_SRC_KINETIS_UART1R       4
#define FREESCALE_DMAMUX_SRC_KINETIS_UART1T       5
#define FREESCALE_DMAMUX_SRC_KINETIS_UART2R       6
#define FREESCALE_DMAMUX_SRC_KINETIS_UART2T       7
#define FREESCALE_DMAMUX_SRC_KINETIS_UART3R       8
#define FREESCALE_DMAMUX_SRC_KINETIS_UART3T       9
#define FREESCALE_DMAMUX_SRC_KINETIS_UART4TR     10
#define FREESCALE_DMAMUX_SRC_KINETIS_UART5TR     11
#define FREESCALE_DMAMUX_SRC_KINETIS_I2S0R       12
#define FREESCALE_DMAMUX_SRC_KINETIS_I2S0T       13
#define FREESCALE_DMAMUX_SRC_KINETIS_SPI0R       14
#define FREESCALE_DMAMUX_SRC_KINETIS_SPI0T       15
#define FREESCALE_DMAMUX_SRC_KINETIS_SPI1TR      16
#define FREESCALE_DMAMUX_SRC_KINETIS_SPI2TR      17
#define FREESCALE_DMAMUX_SRC_KINETIS_I2C0        18
#define FREESCALE_DMAMUX_SRC_KINETIS_I2C1        19 // Either I2C1 or I2C2
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C0      20
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C1      21
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C2      22
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C3      23
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C4      24
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C5      25
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C6      26
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM0C7      27
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM1C0      28
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM1C1      29
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM2C0      30
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM2C1      31
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C0      32
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C1      33
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C2      34
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C3      35
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C4      36
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C5      37
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C6      38
#define FREESCALE_DMAMUX_SRC_KINETIS_FTM3C7      39
#define FREESCALE_DMAMUX_SRC_KINETIS_ADC0        40
#define FREESCALE_DMAMUX_SRC_KINETIS_ADC1        41
#define FREESCALE_DMAMUX_SRC_KINETIS_CMP0        42
#define FREESCALE_DMAMUX_SRC_KINETIS_CMP1        43
#define FREESCALE_DMAMUX_SRC_KINETIS_CMP2        44
#define FREESCALE_DMAMUX_SRC_KINETIS_DAC0        45
#define FREESCALE_DMAMUX_SRC_KINETIS_DAC1        46
#define FREESCALE_DMAMUX_SRC_KINETIS_CMT         47
#define FREESCALE_DMAMUX_SRC_KINETIS_PDB         48
#define FREESCALE_DMAMUX_SRC_KINETIS_PORTA       49
#define FREESCALE_DMAMUX_SRC_KINETIS_PORTB       50
#define FREESCALE_DMAMUX_SRC_KINETIS_PORTC       51
#define FREESCALE_DMAMUX_SRC_KINETIS_PORTD       52
#define FREESCALE_DMAMUX_SRC_KINETIS_PORTE       53
#define FREESCALE_DMAMUX_SRC_KINETIS_1588T0      54
#define FREESCALE_DMAMUX_SRC_KINETIS_1588T1      55
#define FREESCALE_DMAMUX_SRC_KINETIS_1588T2      56
#define FREESCALE_DMAMUX_SRC_KINETIS_1588T3      57
#define FREESCALE_DMAMUX_SRC_KINETIS_DMAMUX0     58
#define FREESCALE_DMAMUX_SRC_KINETIS_DMAMUX1     59
#define FREESCALE_DMAMUX_SRC_KINETIS_DMAMUX2     60
#define FREESCALE_DMAMUX_SRC_KINETIS_DMAMUX3     61
#define FREESCALE_DMAMUX_SRC_KINETIS_DMAMUX4     62
#define FREESCALE_DMAMUX_SRC_KINETIS_DMAMUX5     63



// UART PINs
#ifndef CYGHWR_HAL_FREESCALE_UART3_PIN_RX
# define CYGHWR_HAL_FREESCALE_UART3_PIN_RX CYGHWR_HAL_KINETIS_PIN(C, 16, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART3_PIN_TX CYGHWR_HAL_KINETIS_PIN(C, 17, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART3_PIN_RTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE
# define CYGHWR_HAL_FREESCALE_UART3_PIN_CTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE

# define CYGHWR_IO_FREESCALE_UART3_PIN_RX CYGHWR_HAL_FREESCALE_UART3_PIN_RX
# define CYGHWR_IO_FREESCALE_UART3_PIN_TX CYGHWR_HAL_FREESCALE_UART3_PIN_TX
# define CYGHWR_IO_FREESCALE_UART3_PIN_RTS CYGHWR_HAL_FREESCALE_UART3_PIN_RTS
# define CYGHWR_IO_FREESCALE_UART3_PIN_CTS CYGHWR_HAL_FREESCALE_UART3_PIN_CTS
#endif

#ifndef CYGHWR_HAL_FREESCALE_UART4_PIN_RX
# define CYGHWR_HAL_FREESCALE_UART4_PIN_RX CYGHWR_HAL_KINETIS_PIN(C, 14, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART4_PIN_TX CYGHWR_HAL_KINETIS_PIN(C, 15, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART4_PIN_RTS CYGHWR_HAL_KINETIS_PIN(C, 12, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART4_PIN_CTS CYGHWR_HAL_KINETIS_PIN(C, 13, 3, 0)

# define CYGHWR_IO_FREESCALE_UART4_PIN_RX CYGHWR_HAL_FREESCALE_UART4_PIN_RX
# define CYGHWR_IO_FREESCALE_UART4_PIN_TX CYGHWR_HAL_FREESCALE_UART4_PIN_TX
# define CYGHWR_IO_FREESCALE_UART4_PIN_RTS CYGHWR_HAL_FREESCALE_UART4_PIN_RTS
# define CYGHWR_IO_FREESCALE_UART4_PIN_CTS CYGHWR_HAL_FREESCALE_UART4_PIN_CTS
#endif

// EHCI pins
#define CYGHWR_IO_FREESCALE_EHCI_PIN_RX CYGHWR_HAL_FREESCALE_UART4_PIN_RX
#define CYGHWR_IO_FREESCALE_EHCI_PIN_TX CYGHWR_HAL_FREESCALE_UART4_PIN_TX
#define CYGHWR_IO_FREESCALE_EHCI_PIN_RTS CYGHWR_HAL_FREESCALE_UART4_PIN_RTS
#define CYGHWR_IO_FREESCALE_EHCI_PIN_CTS CYGHWR_HAL_FREESCALE_UART4_PIN_CTS
// EHCI
#define CYGADDR_DEVS_EHCI_SERIAL_FREESCALE_UART_BASE CYGADDR_IO_SERIAL_FREESCALE_UART4_BASE
#define CYGHWR_IO_FREESCALE_EHCI_UART_CLOCK CYGHWR_IO_FREESCALE_UART4_CLOCK
#define CYGNUM_DEVS_EHCI_SERIAL_FREESCALE_ISR_NUM CYGNUM_HAL_INTERRUPT_UART4_ERR

#define FREESCALE_DMAMUX_SRC_KINETIS_EHCI_TX CYGNUM_HAL_INTERRUPT_UART4_RX_TX
#define FREESCALE_DMAMUX_SRC_KINETIS_EHCI_RX CYGNUM_HAL_INTERRUPT_UART4_RX_TX

// DSPI
// DSPI0 Pins
#define CYGHWR_IO_FREESCALE_SPI0_PIN_SIN  CYGHWR_HAL_KINETIS_PIN(A, 17, 2, KINETIS_PIN_SPI1_IN_OPT)
#define CYGHWR_IO_FREESCALE_SPI0_PIN_SOUT CYGHWR_HAL_KINETIS_PIN(A, 16, 2, KINETIS_PIN_SPI1_OUT_OPT)
#define CYGHWR_IO_FREESCALE_SPI0_PIN_SCK  CYGHWR_HAL_KINETIS_PIN(A, 15, 2, KINETIS_PIN_SPI1_OUT_OPT)

// FLASH
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS0  CYGHWR_HAL_KINETIS_PIN(A, 14, 2, KINETIS_PIN_SPI1_CS_OPT)
// FRAM
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS1  CYGHWR_HAL_KINETIS_PIN(C, 3, 2, KINETIS_PIN_SPI1_CS_OPT) 
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS2  CYGHWR_HAL_KINETIS_PIN(C, 2, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS3  CYGHWR_HAL_KINETIS_PIN(C, 1, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS4  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS5  CYGHWR_HAL_KINETIS_PIN_NONE

// DSPI2 Pins
#define CYGHWR_IO_FREESCALE_SPI2_PIN_SIN  CYGHWR_HAL_KINETIS_PIN(B, 23, 2, KINETIS_PIN_SPI1_IN_OPT)
#define CYGHWR_IO_FREESCALE_SPI2_PIN_SOUT CYGHWR_HAL_KINETIS_PIN(B, 22, 2, KINETIS_PIN_SPI1_OUT_OPT)
#define CYGHWR_IO_FREESCALE_SPI2_PIN_SCK  CYGHWR_HAL_KINETIS_PIN(B, 21, 2, KINETIS_PIN_SPI1_OUT_OPT)

#define CYGHWR_IO_FREESCALE_SPI2_PIN_CS0  CYGHWR_HAL_KINETIS_PIN(B, 20, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI2_PIN_CS1  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI2_PIN_CS2  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI2_PIN_CS3  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI2_PIN_CS4  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI2_PIN_CS5  CYGHWR_HAL_KINETIS_PIN_NONE

// I2C
// I2C Pins RTC
# define CYGHWR_IO_I2C_FREESCALE_I2C1_PIN_SDA CYGHWR_HAL_KINETIS_PIN(C, 11, 2, 0)
# define CYGHWR_IO_I2C_FREESCALE_I2C1_PIN_SCL CYGHWR_HAL_KINETIS_PIN(C, 10, 2, 0)

// Matrix keyboard
// KBOUT0
#define CYGHWR_IO_FREESCALE_KBOUT0      CYGHWR_HAL_KINETIS_PIN(B, 2,  1, 0)
#define CYGHWR_IO_DIR_KBDOUT0           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(B, 2)
#define CYGNUM_IO_MATRIX_KBDOUT0_SET    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B,2)
#define CYGNUM_IO_MATRIX_KBDOUT0_CLR    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B,2)
// KBOUT1
#define CYGHWR_IO_FREESCALE_KBOUT1      CYGHWR_HAL_KINETIS_PIN(B, 1,  1, 0)
#define CYGHWR_IO_DIR_KBDOUT1           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(B, 1)
#define CYGNUM_IO_MATRIX_KBDOUT1_SET    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B,1)
#define CYGNUM_IO_MATRIX_KBDOUT1_CLR    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B,1)
// KBOUT2
#define CYGHWR_IO_FREESCALE_KBOUT2      CYGHWR_HAL_KINETIS_PIN(B, 0,  1, 0)
#define CYGHWR_IO_DIR_KBDOUT2           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(B, 0)
#define CYGNUM_IO_MATRIX_KBDOUT2_SET    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B,0)
#define CYGNUM_IO_MATRIX_KBDOUT2_CLR    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B,0)
// KBOUT3
#define CYGHWR_IO_FREESCALE_KBOUT3      CYGHWR_HAL_KINETIS_PIN(A, 19,  1, 0)
#define CYGHWR_IO_DIR_KBDOUT3           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(A, 19)
#define CYGNUM_IO_MATRIX_KBDOUT3_SET    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(A,19)
#define CYGNUM_IO_MATRIX_KBDOUT3_CLR    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(A,19)

// KBREAD0
// Interrupt is generated on falling-edge
#define CYGHWR_IO_FREESCALE_KBREAD0     CYGHWR_HAL_KINETIS_PIN_CFG(B, 11,  1, 0x0A, 0)
#define CYGHWR_IO_DIR_KBREAD0           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(B, 11)
#define CYGNUM_IO_MATRIX_GET_KBREAD0    CYGHWR_HAL_KINETIS_GPIO_GET_PIN(B,11)
#define CYGNUM_IO_MATRIX_CLR_KBREAD0_ISR CYGHWR_HAL_KINETIS_PORT_ISFR_CLEAR(B,11);
// KBREAD1
// Interrupt is generated on falling-edge
#define CYGHWR_IO_FREESCALE_KBREAD1     CYGHWR_HAL_KINETIS_PIN_CFG(B, 10,  1, 0x0A, 0)
#define CYGHWR_IO_DIR_KBREAD1           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(B, 10)
#define CYGNUM_IO_MATRIX_GET_KBREAD1    CYGHWR_HAL_KINETIS_GPIO_GET_PIN(B,10)
#define CYGNUM_IO_MATRIX_CLR_KBREAD1_ISR CYGHWR_HAL_KINETIS_PORT_ISFR_CLEAR(B,10);
// KBREAD2
// Interrupt is generated on falling-edge
#define CYGHWR_IO_FREESCALE_KBREAD2     CYGHWR_HAL_KINETIS_PIN_CFG(B, 9,  1, 0x0A, 0)
#define CYGHWR_IO_DIR_KBREAD2           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(B, 9)
#define CYGNUM_IO_MATRIX_GET_KBREAD2    CYGHWR_HAL_KINETIS_GPIO_GET_PIN(B,9)
#define CYGNUM_IO_MATRIX_CLR_KBREAD2_ISR CYGHWR_HAL_KINETIS_PORT_ISFR_CLEAR(B,9);
// KBREAD3
// Interrupt is generated on falling-edge
#define CYGHWR_IO_FREESCALE_KBREAD3     CYGHWR_HAL_KINETIS_PIN_CFG(B, 3,  1, 0x0A, 0)
#define CYGHWR_IO_DIR_KBREAD3           CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(B, 3)
#define CYGNUM_IO_MATRIX_GET_KBREAD3    CYGHWR_HAL_KINETIS_GPIO_GET_PIN(B,3)
#define CYGNUM_IO_MATRIX_CLR_KBREAD3_ISR CYGHWR_HAL_KINETIS_PORT_ISFR_CLEAR(B,3);

// LCD
// LCD Pins
#define CYGHWR_IO_FREESCALE_LCD_A0  CYGHWR_HAL_KINETIS_PIN(B, 19,  1, 0)
#define CYGHWR_IO_DIR_LCD_A0        CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(B, 19)
#define CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_HIGH CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B,19)
#define CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW  CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B,19)

// BUZ
#define CYGHWR_IO_FREESCALE_FTM2_PIN_CH0 CYGHWR_HAL_KINETIS_PIN(B, 18, 3, 0)
#define CYGHWR_IO_FREESCALE_BEEP_PIN CYGHWR_IO_FREESCALE_FTM2_PIN_CH0

// Miscelcium GPIO pins
#define CYGHWR_IO_FRESCALE_PIN_NSHUTD CYGHWR_HAL_KINETIS_PIN(C, 9,  1, 0)
#define CYGHWR_IO_DIR_PIN_NSHUTD      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(C, 9)
#define CYGHWR_IO_SET_PIN_NSHUTD      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(C,9)
#define CYGHWR_IO_CLEAR_PIN_NSHUTD    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(C,9)

#define CYGHWR_IO_FRESCALE_PIN_LCD_BL CYGHWR_HAL_KINETIS_PIN(C, 4,  1, 0)
#define CYGHWR_IO_DIR_PIN_LCD_BL      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(C, 4)
#define CYGHWR_IO_SET_PIN_LCD_BL      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(C,4)
#define CYGHWR_IO_CLEAR_PIN_LCD_BL    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(C,4)

#define CYGHWR_IO_FRESCALE_PIN_PWOFF CYGHWR_HAL_KINETIS_PIN(D, 7,  1, KINETIS_PIN_PD_ENABLE)
#define CYGHWR_IO_DIR_PIN_PWOFF      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(D, 7)
#define CYGHWR_IO_SET_PIN_PWOFF      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(D,7)
#define CYGHWR_IO_CLEAR_PIN_PWOFF    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(D,7)

// SPI LCD Display
#define CYGHWR_DEVS_LCD_DEV1_SPI_CS                     0
#define LCD_SPI_FRAME_SIZE                              8
#define LCD_SPI_CLOCK_POL                               0
#define LCD_SPI_CLOCK_PHASE                             0
#define CYGHWR_DEVS_LCD_DEV1_SPEED                      8000000l
#define CYGHWR_DEVS_LCD_DEV1_CS_DLY                     1
#define CYGHWR_DEVS_LCD_DEV1_CS_DLY_UN                  100
#define LCD_SPI_DBR_DEV1                                0l


// FM25W SPI FRAM
#define CYGHWR_DEVS_FRAM_FM25VXX_DEV1_SPI_CS            1
#define FM25VXX_SPI_FRAME_SIZE                          8
#define FM25VXX_SPI_CLOCK_POL                           0
#define FM25VXX_SPI_CLOCK_PHASE                         0
#define CYGHWR_DEVS_FRAM_FM25VXX_DEV2_SPEED             20000000l
#define CYGHWR_DEVS_FRAM_FM25VXX_DEV2_CS_DLY            1
#define CYGHWR_DEVS_FRAM_FM25VXX_DEV2_CS_DLY_UN         100
#define FM25VXX_SPI_DBR_DEV2                            0l
#define FRAM_FM25WXX_BASE_ADDRESS						0xe0000000

// MT25QL SPI FLASH
#define CYGHWR_DEVS_FLASH_MT25QL_DEV2_SPI_CS            0
#define MT25QL_SPI_FRAME_SIZE                          8
#define MT25QL_SPI_CLOCK_POL                           0
#define MT25QL_SPI_CLOCK_PHASE                         0
#define CYGHWR_DEVS_FLASH_MT25QL_DEV2_SPEED             50000000l
#define CYGHWR_DEVS_FLASH_MT25QL_DEV2_CS_DLY            1
#define CYGHWR_DEVS_FLASH_MT25QL_DEV2_CS_DLY_UN         100
#define MT25QL_SPI_DBR_DEV2                            0l
#define FRAM_MT25QL_BASE_ADDRESS					   0x80000000

externC cyg_uint64 _hw_error;
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

//=============================================================================
// Memory access checks.
//
// Accesses to areas not backed by real devices or memory can cause
// the CPU to hang. These macros allow the GDB stubs to avoid making
// accidental accesses to these areas.

__externC int cyg_hal_stub_permit_data_access( CYG_ADDRESS addr, cyg_uint32 count );

#define CYG_HAL_STUB_PERMIT_DATA_READ(_addr_, _count_) cyg_hal_stub_permit_data_access( _addr_, _count_ )

#define CYG_HAL_STUB_PERMIT_DATA_WRITE(_addr_, _count_ ) cyg_hal_stub_permit_data_access( _addr_, _count_ )

//=============================================================================


//-----------------------------------------------------------------------------
// end of plf_io.h
#endif // CYGONCE_HAL_PLF_IO_H
