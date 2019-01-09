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
#include <pkgconf/hal_cortexm_kinetis_frdm_k64f1024.h>

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
#ifndef CYGHWR_HAL_FREESCALE_UART0_PIN_RX
# define CYGHWR_HAL_FREESCALE_UART0_PIN_RX CYGHWR_HAL_KINETIS_PIN(B, 16, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART0_PIN_TX CYGHWR_HAL_KINETIS_PIN(B, 17, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART0_PIN_RTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE
# define CYGHWR_HAL_FREESCALE_UART0_PIN_CTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE

# define CYGHWR_IO_FREESCALE_UART0_PIN_RX CYGHWR_HAL_FREESCALE_UART1_PIN_RX
# define CYGHWR_IO_FREESCALE_UART0_PIN_TX CYGHWR_HAL_FREESCALE_UART1_PIN_TX
# define CYGHWR_IO_FREESCALE_UART0_PIN_RTS CYGHWR_HAL_FREESCALE_UART1_PIN_RTS
# define CYGHWR_IO_FREESCALE_UART0_PIN_CTS CYGHWR_HAL_FREESCALE_UART1_PIN_CTS
#endif

#ifndef CYGHWR_HAL_FREESCALE_UART4_PIN_RX
# define CYGHWR_HAL_FREESCALE_UART4_PIN_RX CYGHWR_HAL_KINETIS_PIN(C, 14, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART4_PIN_TX CYGHWR_HAL_KINETIS_PIN(C, 15, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART4_PIN_RTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE
# define CYGHWR_HAL_FREESCALE_UART4_PIN_CTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE

# define CYGHWR_IO_FREESCALE_UART4_PIN_RX CYGHWR_HAL_FREESCALE_UART4_PIN_RX
# define CYGHWR_IO_FREESCALE_UART4_PIN_TX CYGHWR_HAL_FREESCALE_UART4_PIN_TX
# define CYGHWR_IO_FREESCALE_UART4_PIN_RTS CYGHWR_HAL_FREESCALE_UART4_PIN_RTS
# define CYGHWR_IO_FREESCALE_UART4_PIN_CTS CYGHWR_HAL_FREESCALE_UART4_PIN_CTS
#endif

// ENET PINs

// MDIO
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_MDIO    CYGHWR_HAL_KINETIS_PIN(B, 0, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_MDC     CYGHWR_HAL_KINETIS_PIN(B, 1, 4, 0)
// Both RMII and MII interface
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXER    CYGHWR_HAL_KINETIS_PIN(A, 5, 4, \
                                                    CYGHWR_HAL_KINETIS_PORT_PCR_PE_M)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXD1    CYGHWR_HAL_KINETIS_PIN(A, 12, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXD0    CYGHWR_HAL_KINETIS_PIN(A, 13, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXEN    CYGHWR_HAL_KINETIS_PIN(A, 15, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXD0    CYGHWR_HAL_KINETIS_PIN(A, 16, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXD1    CYGHWR_HAL_KINETIS_PIN(A, 17, 4, 0)
// RMII interface only
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_CRS_DV  CYGHWR_HAL_KINETIS_PIN(A, 14, 4, 0)
// MII interface only
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXD3     CYGHWR_HAL_KINETIS_PIN(A, 9, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXD2     CYGHWR_HAL_KINETIS_PIN(A, 10, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXCLK    CYGHWR_HAL_KINETIS_PIN(A, 11, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXD2     CYGHWR_HAL_KINETIS_PIN(A, 24, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXCLK    CYGHWR_HAL_KINETIS_PIN(A, 25, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXD3     CYGHWR_HAL_KINETIS_PIN(A, 26, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_CRS      CYGHWR_HAL_KINETIS_PIN(A, 27, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MIIO_TXER     CYGHWR_HAL_KINETIS_PIN(A, 28, 4, 0)
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_COL      CYGHWR_HAL_KINETIS_PIN(A, 29, 4, 0)
// IEEE 1588 timers
#define CYGHWR_IO_FREESCALE_ENET0_PIN_1588_CLKIN    CYGHWR_HAL_KINETIS_PIN(E, 26, 4, 0)

#if defined(CYGHWR_HAL_FREESCALE_ENET0_E0_1588_PORT_B)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR0  CYGHWR_HAL_KINETIS_PIN(B, 2, 4, 0)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR1  CYGHWR_HAL_KINETIS_PIN(B, 3, 4, 0)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR2  CYGHWR_HAL_KINETIS_PIN(B, 4, 4, 0)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR3  CYGHWR_HAL_KINETIS_PIN(B, 5, 4, 0)
#elif defined(CYGHWR_HAL_FREESCALE_ENET0_E0_1588_PORT_C)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR0  CYGHWR_HAL_KINETIS_PIN(C, 16, 4, 0)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR1  CYGHWR_HAL_KINETIS_PIN(C, 17, 4, 0)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR2  CYGHWR_HAL_KINETIS_PIN(C, 18, 4, 0)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR3  CYGHWR_HAL_KINETIS_PIN(C, 19, 4, 0)
#endif


// DSPI
// DSPI Pins

#define CYGHWR_IO_FREESCALE_SPI1_PIN_SIN  CYGHWR_HAL_KINETIS_PIN(E, 3, 2, KINETIS_PIN_SPI1_IN_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_SOUT CYGHWR_HAL_KINETIS_PIN(E, 1, 2, KINETIS_PIN_SPI1_OUT_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_SCK  CYGHWR_HAL_KINETIS_PIN(E, 2, 2, KINETIS_PIN_SPI1_OUT_OPT)

#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS0  CYGHWR_HAL_KINETIS_PIN(E, 4, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS1  CYGHWR_HAL_KINETIS_PIN(E, 0, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS2  CYGHWR_HAL_KINETIS_PIN(E, 5, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS3  CYGHWR_HAL_KINETIS_PIN(E, 6, 2, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS4  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS5  CYGHWR_HAL_KINETIS_PIN_NONE

// I2C
// I2C Pins ACCELEROMETER AND MAGNETOMETER

# define CYGHWR_IO_I2C_FREESCALE_I2C0_PIN_SDA CYGHWR_HAL_KINETIS_PIN(E, 25, 5, 0)
# define CYGHWR_IO_I2C_FREESCALE_I2C0_PIN_SCL CYGHWR_HAL_KINETIS_PIN(E, 24, 5, 0)

// RGB LED
// Red
#define CYGHWR_IO_FREESCALE_RED_LED  CYGHWR_HAL_KINETIS_PIN(B, 22,  1, 0)
#define CYGHWR_IO_DIR_RED_LED        CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(B, 22)
#define CYGNUM_IO_RED_LED_ON         CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B,22)
#define CYGNUM_IO_RED_LED_OFF        CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B,22)

// Blue
#define CYGHWR_IO_FREESCALE_BLUE_LED  CYGHWR_HAL_KINETIS_PIN(B, 21,  1, 0)
#define CYGHWR_IO_DIR_BLUE_LED        CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(B, 21)
#define CYGNUM_IO_RED_BLUE_ON         CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B,21)
#define CYGNUM_IO_RED_BLUE_OFF        CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B,21)

// Green
#define CYGHWR_IO_FREESCALE_GREEN_LED  CYGHWR_HAL_KINETIS_PIN(E, 26,  1, 0)
#define CYGHWR_IO_DIR_GREEN_LED        CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(E, 26)
#define CYGNUM_IO_RED_GREEN_ON         CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(E,26)
#define CYGNUM_IO_RED_GREEN_OFF        CYGHWR_HAL_KINETIS_GPIO_SET_PIN(E,26)

// Buttons
// SW2
#define CYGHWR_IO_FREESCALE_BUTTON_SW2  CYGHWR_HAL_KINETIS_PIN(C, 6,  1, 0)
#define CYGHWR_IO_DIR_BUTTON_SW2        CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(C, 6)
#define CYGNUM_IO_BUTTON_SW2_STATE      CYGHWR_HAL_KINETIS_GPIO_GET_PIN(C,6)

// SW3
#define CYGHWR_IO_FREESCALE_BUTTON_SW3  CYGHWR_HAL_KINETIS_PIN(A, 4,  1, 0)
#define CYGHWR_IO_DIR_BUTTON_SW3        CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(A, 4)
#define CYGNUM_IO_BUTTON_SW3_STATE      CYGHWR_HAL_KINETIS_GPIO_GET_PIN(A,4)


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
