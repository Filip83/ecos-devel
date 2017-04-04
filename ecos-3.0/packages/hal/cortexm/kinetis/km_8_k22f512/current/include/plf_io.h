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
// Copyright (C) 2012 Free Software Foundation, Inc.                        
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
// Author(s):   Ilija Kocho <ilijak@siva.com.mk>
// Contrib(s):  Mike Jones <mjones@proclivis.com>
// Date:        2013-06-02
// Purpose:     TWR-K60F120M platform specific registers
// Description:
// Usage:       #include <cyg/hal/plf_io.h>
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <pkgconf/hal.h>
#include <pkgconf/hal_cortexm_kinetis_twr_k60f120m.h>


// UART PINs
#ifndef CYGHWR_HAL_FREESCALE_UART1_PIN_RX
# define CYGHWR_HAL_FREESCALE_UART1_PIN_RX CYGHWR_HAL_KINETIS_PIN(E, 1, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART1_PIN_TX CYGHWR_HAL_KINETIS_PIN(E, 0, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART1_PIN_RTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE
# define CYGHWR_HAL_FREESCALE_UART1_PIN_CTS CYGHWR_HAL_KINETIS_PORT_PIN_NONE

# define CYGHWR_IO_FREESCALE_UART1_PIN_RX CYGHWR_HAL_FREESCALE_UART1_PIN_RX
# define CYGHWR_IO_FREESCALE_UART1_PIN_TX CYGHWR_HAL_FREESCALE_UART1_PIN_TX
# define CYGHWR_IO_FREESCALE_UART1_PIN_RTS CYGHWR_HAL_FREESCALE_UART1_PIN_RTS
# define CYGHWR_IO_FREESCALE_UART1_PIN_CTS CYGHWR_HAL_FREESCALE_UART1_PIN_CTS
#endif

#ifndef CYGHWR_HAL_FREESCALE_UART0_PIN_RX
# define CYGHWR_HAL_FREESCALE_UART0_PIN_RX CYGHWR_HAL_KINETIS_PIN(D, 6, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART0_PIN_TX CYGHWR_HAL_KINETIS_PIN(D, 7, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART0_PIN_RTS CYGHWR_HAL_KINETIS_PIN(B, 2, 3, 0)
# define CYGHWR_HAL_FREESCALE_UART0_PIN_CTS CYGHWR_HAL_KINETIS_PIN(B, 3, 3, 0)

# define CYGHWR_IO_FREESCALE_UART0_PIN_RX CYGHWR_HAL_FREESCALE_UART0_PIN_RX
# define CYGHWR_IO_FREESCALE_UART0_PIN_TX CYGHWR_HAL_FREESCALE_UART0_PIN_TX
# define CYGHWR_IO_FREESCALE_UART0_PIN_RTS CYGHWR_HAL_FREESCALE_UART0_PIN_RTS
# define CYGHWR_IO_FREESCALE_UART0_PIN_CTS CYGHWR_HAL_FREESCALE_UART0_PIN_CTS
#endif

// EHCI pins
#define CYGHWR_IO_FREESCALE_EHCI_PIN_RX CYGHWR_HAL_FREESCALE_UART1_PIN_RX
#define CYGHWR_IO_FREESCALE_EHCI_PIN_TX CYGHWR_HAL_FREESCALE_UART1_PIN_TX
#define CYGHWR_IO_FREESCALE_EHCI_PIN_RTS CYGHWR_HAL_FREESCALE_UART1_PIN_RTS
#define CYGHWR_IO_FREESCALE_EHCI_PIN_CTS CYGHWR_HAL_FREESCALE_UART1_PIN_CTS
// EHCI
#define CYGADDR_DEVS_EHCI_SERIAL_FREESCALE_UART_BASE CYGADDR_IO_SERIAL_FREESCALE_UART1_BASE
#define CYGHWR_IO_FREESCALE_EHCI_UART_CLOCK CYGHWR_IO_FREESCALE_UART1_CLOCK

// DSPI
// DSPI0 Pins

#define KINETIS_PIN_SPI0_OUT_OPT (CYGHWR_HAL_KINETIS_PORT_PCR_PS_M | \
                                  CYGHWR_HAL_KINETIS_PORT_PCR_PE_M)
#define KINETIS_PIN_SPI0_SCK_OPT (0)
#define KINETIS_PIN_SPI0_CS_OPT  (CYGHWR_HAL_KINETIS_PORT_PCR_PS_M | \
                                  CYGHWR_HAL_KINETIS_PORT_PCR_PE_M)

#define KINETIS_PIN_SPI0_IN_OPT  (CYGHWR_HAL_KINETIS_PORT_PCR_PE_M | CYGHWR_HAL_KINETIS_PORT_PCR_PS_M)
#define KINETIS_PIN_SPI0_0_OPT  (CYGHWR_HAL_KINETIS_PORT_PCR_PE_M | CYGHWR_HAL_KINETIS_PORT_PCR_PS_M)

#define CYGHWR_IO_FREESCALE_SPI0_PIN_SIN  CYGHWR_HAL_KINETIS_PIN(D, 3, 2, KINETIS_PIN_SPI0_IN_OPT)
#define CYGHWR_IO_FREESCALE_SPI0_PIN_SOUT CYGHWR_HAL_KINETIS_PIN(D, 2, 2, KINETIS_PIN_SPI0_OUT_OPT)
#define CYGHWR_IO_FREESCALE_SPI0_PIN_SCK  CYGHWR_HAL_KINETIS_PIN(D, 1, 2, KINETIS_PIN_SPI0_SCK_OPT)

//RTC_CS
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS0  CYGHWR_HAL_KINETIS_PIN(D, 0, 1, KINETIS_PIN_SPI0_CS_OPT)
#define CYGHWR_IO_GPIO_PIN_CS_RTC_CLR     CYGHWR_HAL_KINETIS_GPIO_SET_PIN(D,0)
#define CYGHWR_IO_GPIO_PIN_CS_RTC_SET     CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(D,0)
//LCD_CS
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS1  CYGHWR_HAL_KINETIS_PIN(C, 11, 1, KINETIS_PIN_SPI0_CS_OPT)
#define CYGHWR_IO_GPIO_PIN_CS_LCD_CLR     CYGHWR_HAL_KINETIS_GPIO_SET_PIN(C,11)
#define CYGHWR_IO_GPIO_PIN_CS_LCD_SET     CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(C,11)
//FRAM_CS
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS2  CYGHWR_HAL_KINETIS_PIN(D, 4, 1, KINETIS_PIN_SPI0_CS_OPT)
#define CYGHWR_IO_GPIO_PIN_CS_FRAM_CLR     CYGHWR_HAL_KINETIS_GPIO_SET_PIN(D, 4)
#define CYGHWR_IO_GPIO_PIN_CS_FRAM_SET     CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(D, 4)

#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS3  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS4  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI0_PIN_CS5  CYGHWR_HAL_KINETIS_PIN_NONE

// DSPI1 Pins

#define KINETIS_PIN_SPI1_OUT_OPT ( CYGHWR_HAL_KINETIS_PORT_PCR_PS_M | \
                                  CYGHWR_HAL_KINETIS_PORT_PCR_PE_M)
#define KINETIS_PIN_SPI1_SCK_OPT (0)
#define KINETIS_PIN_SPI1_CS_OPT  (CYGHWR_HAL_KINETIS_PORT_PCR_PS_M | \
                                  CYGHWR_HAL_KINETIS_PORT_PCR_PE_M)

#define KINETIS_PIN_SPI1_IN_OPT  (CYGHWR_HAL_KINETIS_PORT_PCR_PE_M | CYGHWR_HAL_KINETIS_PORT_PCR_PS_M)
#define KINETIS_PIN_SPI1_0_OPT   (CYGHWR_HAL_KINETIS_PORT_PCR_PE_M | CYGHWR_HAL_KINETIS_PORT_PCR_PS_M)

#define CYGHWR_IO_FREESCALE_SPI1_PIN_SIN  CYGHWR_HAL_KINETIS_PIN(B, 17, 2, KINETIS_PIN_SPI1_IN_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_SOUT CYGHWR_HAL_KINETIS_PIN(B, 16, 2, KINETIS_PIN_SPI1_OUT_OPT)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_SCK  CYGHWR_HAL_KINETIS_PIN(D, 5,  7, KINETIS_PIN_SPI1_SCK_OPT)

// ADC_CS
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS0  CYGHWR_HAL_KINETIS_PIN(B, 0, 1, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_GPIO_PIN_CS_ADC_CLR     CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B, 0)
#define CYGHWR_IO_GPIO_PIN_CS_ADC_SET     CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B, 0)
// DAC_CS
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS1  CYGHWR_HAL_KINETIS_PIN(B, 1, 1, KINETIS_PIN_SPI1_CS_OPT)
#define CYGHWR_IO_GPIO_PIN_CS_DAC_CLR     CYGHWR_HAL_KINETIS_GPIO_SET_PIN(B, 1)
#define CYGHWR_IO_GPIO_PIN_CS_DAC_SET     CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(B, 1)
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS2  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS3  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS4  CYGHWR_HAL_KINETIS_PIN_NONE
#define CYGHWR_IO_FREESCALE_SPI1_PIN_CS5  CYGHWR_HAL_KINETIS_PIN_NONE

// SAI
// SAI Pins
# define CYGHWR_IO_FREESCALE_SAI_PIN_MCLK    CYGHWR_HAL_KINETIS_PIN(C, 6,  6, 0)
# define CYGHWR_IO_FREESCALE_SAI_PIN_TX_BCLK CYGHWR_HAL_KINETIS_PIN(B, 18, 4, 0)
# define CYGHWR_IO_FREESCALE_SAI_PIN_TX_SYNC CYGHWR_HAL_KINETIS_PIN(B, 19, 4, 0)
# define CYGHWR_IO_FREESCALE_SAI_PIN_TX_DATA CYGHWR_HAL_KINETIS_PIN(C, 1,  6, 0)

# define CYGHWR_IO_FREESCALE_SAI_PIN_RX_BCLK CYGHWR_HAL_KINETIS_PIN_NONE
# define CYGHWR_IO_FREESCALE_SAI_PIN_RX_SYNC CYGHWR_HAL_KINETIS_PIN_NONE
# define CYGHWR_IO_FREESCALE_SAI_PIN_RX_DATA CYGHWR_HAL_KINETIS_PIN_NONE

// LCD
// LCD Pins
#define CYGHWR_IO_FREESCALE_LCD_A0  CYGHWR_HAL_KINETIS_PIN(C, 10,  1, 0)
#define CYGHWR_IO_DIR_PIN_PWOFF     CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(C, 10)
#define CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_HIGH CYGHWR_HAL_KINETIS_GPIO_SET_PIN(C,10)
#define CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW  CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(C,10)

// XTAL
// XTAL Pins
#define CYGHWR_IO_FREESCALE_XTAL_PIN_EXTAL0  CYGHWR_HAL_KINETIS_PIN(A, 18,  0, 0)
#define CYGHWR_IO_FREESCALE_XTAL_PIN_XTAL0   CYGHWR_HAL_KINETIS_PIN(A, 19,  0, 0)

// Kyboard
// Kyboard pins
#define CYGHWR_DEVS_KBD_BUTTON_PORT CYGHWR_HAL_KINETIS_PORTC_P
#define CYGHWR_DEVS_KB0_PIN         4
#define CYGHWR_DEVS_KB1_PIN         5
#define CYGHWR_DEVS_KB2_PIN         7
#define CYGHWR_DEVS_KB3_PIN         8

// FTM
// FTM Pins
#define KINETIS_PIN_PD_ENABLE (CYGHWR_HAL_KINETIS_PORT_PCR_PE_M)
// SIG0 and SIG90
# define CYGHWR_IO_FREESCALE_FTM0_PIN_CH1 CYGHWR_HAL_KINETIS_PIN(C, 2,  4, KINETIS_PIN_PD_ENABLE)
# define CYGHWR_IO_FREESCALE_FTM0_PIN_CH2 CYGHWR_HAL_KINETIS_PIN(C, 3, 4, KINETIS_PIN_PD_ENABLE)
// BUZ
# define CYGHWR_IO_FREESCALE_FTM1_PIN_CH0 CYGHWR_HAL_KINETIS_PIN(A, 12, 3, 0)

// Miscelcium GPIO pins
#define CYGHWR_IO_FRESCALE_PIN_NSHUTD CYGHWR_HAL_KINETIS_PIN(A, 4,  1, 0)
#define CYGHWR_IO_DIR_PIN_NSHUTD      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(A, 4)
#define CYGHWR_IO_SET_PIN_NSHUTD      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(A,4)
#define CYGHWR_IO_CLEAR_PIN_NSHUTD    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(A,4)

#define CYGHWR_IO_FRESCALE_PIN_ANL_ON CYGHWR_HAL_KINETIS_PIN(A, 5,  1, KINETIS_PIN_PD_ENABLE)
#define CYGHWR_IO_DIR_PIN_ANL_ON      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(A, 5)
#define CYGHWR_IO_SET_PIN_ANL_ON      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(A,5)
#define CYGHWR_IO_CLEAR_PIN_ANL_ON    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(A,5)

#define CYGHWR_IO_FRESCALE_PIN_LCD_BL CYGHWR_HAL_KINETIS_PIN(A, 13,  1, 0)
#define CYGHWR_IO_DIR_PIN_LCD_BL      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(A, 13)
#define CYGHWR_IO_SET_PIN_LCD_BL      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(A,13)
#define CYGHWR_IO_CLEAR_PIN_LCD_BL    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(A,13)

#define CYGHWR_IO_FRESCALE_PIN_PWOFF CYGHWR_HAL_KINETIS_PIN(C, 9,  1, KINETIS_PIN_PD_ENABLE)
#define CYGHWR_IO_DIR_PIN_PWOFF      CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(C, 9)
#define CYGHWR_IO_SET_PIN_PWOFF      CYGHWR_HAL_KINETIS_GPIO_SET_PIN(C,9)
#define CYGHWR_IO_CLEAR_PIN_PWOFF    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(C,9)

//=============================================================================
// Memory access checks.
//
// Accesses to areas not backed by real devices or memory can cause
// the CPU to hang. These macros allow the GDB stubs to avoid making
// accidental accesses to these areas.

__externC int cyg_hal_stub_permit_data_access( void* addr, cyg_uint32 count );

#define CYG_HAL_STUB_PERMIT_DATA_READ(_addr_, _count_) cyg_hal_stub_permit_data_access( _addr_, _count_ )

#define CYG_HAL_STUB_PERMIT_DATA_WRITE(_addr_, _count_ ) cyg_hal_stub_permit_data_access( _addr_, _count_ )

//=============================================================================


//-----------------------------------------------------------------------------
// end of plf_io.h
#endif // CYGONCE_HAL_PLF_IO_H

