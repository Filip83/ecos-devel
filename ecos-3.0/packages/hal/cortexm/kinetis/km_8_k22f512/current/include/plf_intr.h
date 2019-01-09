#ifndef CYGONCE_HAL_PLF_INTR_H
#define CYGONCE_HAL_PLF_INTR_H
//=============================================================================
//
//      plf_intr.h
//
//      Platform specific interrupt overrides
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
// Purpose:     TWR-K60F120M platform specific interrupt overrides
// Description: 
// Usage:       #include <cyg/hal/plf_intr.h>
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <pkgconf/hal.h>
#include <pkgconf/hal_cortexm_kinetis_km_8_k22f512.h>

#define KinetisInterruptsDefined
typedef enum {
    CYGNUM_HAL_INTERRUPT_DMA0
        = CYGNUM_HAL_INTERRUPT_EXTERNAL,  //0 DMA Channel 0 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA1,            //1 DMA Channel 1 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA2,            //2 DMA Channel 2 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA3,            //3 DMA Channel 3 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA4,            //4 DMA Channel 4 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA5,            //5 DMA Channel 5 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA6,            //6 DMA Channel 6 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA7,            //7 DMA Channel 7 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA8,            //8 DMA Channel 8 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA9,            //9 DMA Channel 9 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA10,           //10 DMA Channel 10 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA11,           //11 DMA Channel 11 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA12,           //12 DMA Channel 12 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA13,           //13 DMA Channel 13 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA14,           //14 DMA Channel 14 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA15,           //15 DMA Channel 15 Transfer Complete
    CYGNUM_HAL_INTERRUPT_DMA_ERROR,       //16 DMA Error Int
    CYGNUM_HAL_INTERRUPT_MCM,             //17 Normal Int
    CYGNUM_HAL_INTERRUPT_FTFL,            //18 FTFL Int
    CYGNUM_HAL_INTERRUPT_READ_COLLISION,  //19 Read Collision Int
    CYGNUM_HAL_INTERRUPT_LVD_LVW,         //20 Low Volt Detect, Low Volt Warn
    CYGNUM_HAL_INTERRUPT_LLW,             //21 Low Leakage Wakeup
    CYGNUM_HAL_INTERRUPT_WDOG,            //22 WDOG Int
    CYGNUM_HAL_INTERRUPT_RNGB,            //23 RNGB Int
    CYGNUM_HAL_INTERRUPT_I2C0,            //24 I2C0 int
    CYGNUM_HAL_INTERRUPT_I2C1,            //25 I2C1 int
    CYGNUM_HAL_INTERRUPT_SPI0,            //26 SPI0 Int
    CYGNUM_HAL_INTERRUPT_SPI1,            //27 SPI1 Int
    CYGNUM_HAL_INTERRUPT_I2S0_TX,         //28 I2S0 transmit interrupt
    CYGNUM_HAL_INTERRUPT_I2S0_RX,         //29 I2S0 receive interrupt
    CYGNUM_HAL_INTERRUPT_LPUART0_IRQ,     //30 LPUART0 status/error interrupt
    CYGNUM_HAL_INTERRUPT_UART0_RX_TX,     //31 UART0 Receive/Transmit int
    CYGNUM_HAL_INTERRUPT_UART0_ERR,       //32 UART0 Error int
    CYGNUM_HAL_INTERRUPT_UART1_RX_TX,     //33 UART1 Receive/Transmit int
    CYGNUM_HAL_INTERRUPT_UART1_ERR,       //34 UART1 Error int
    CYGNUM_HAL_INTERRUPT_UART2_RX_TX,     //35 UART2 Receive/Transmit int
    CYGNUM_HAL_INTERRUPT_UART2_ERR,       //36 UART2 Error int
    CYGNUM_HAL_INTERRUPT_Reserved53,      //37 Reserved interrupt 53
    CYGNUM_HAL_INTERRUPT_Reserved54,      //38 Reserved interrupt 54
    CYGNUM_HAL_INTERRUPT_ADC0,            //39 ADC0 int
    CYGNUM_HAL_INTERRUPT_CMP0,            //40 CMP0 int
    CYGNUM_HAL_INTERRUPT_CMP1,            //41 CMP1 int
    CYGNUM_HAL_INTERRUPT_FTM0,            //42 FTM0 fault, overflow and channels int
    CYGNUM_HAL_INTERRUPT_FTM1,            //43 FTM1 fault, overflow and channels int
    CYGNUM_HAL_INTERRUPT_FTM2,            //44 FTM2 fault, overflow and channels int
    CYGNUM_HAL_INTERRUPT_Reserved61,      //45 Reserved interrupt 61    
    CYGNUM_HAL_INTERRUPT_RTC_RTC,         //46 RTC int
    CYGNUM_HAL_INTERRUPT_RTC_SECONDS,     //47 RTC seconds interrupt
    CYGNUM_HAL_INTERRUPT_PIT0,            //48 PIT timer channel 0 int
    CYGNUM_HAL_INTERRUPT_PIT1,            //49 PIT timer channel 1 int
    CYGNUM_HAL_INTERRUPT_PIT2,            //50 PIT timer channel 2 int
    CYGNUM_HAL_INTERRUPT_PIT3,            //51 PIT timer channel 3 int
    CYGNUM_HAL_INTERRUPT_PDB0,            //52 PDB0 Int
    CYGNUM_HAL_INTERRUPT_USB0,            //53 USB0 int
    CYGNUM_HAL_INTERRUPT_Reserved70,      //54 Reserved interrupt 70    
    CYGNUM_HAL_INTERRUPT_Reserved71,      //55 Reserved interrupt 71    
    CYGNUM_HAL_INTERRUPT_DAC0,            //56 DAC0 int
    CYGNUM_HAL_INTERRUPT_MCG,             //57 MCG Int           
    CYGNUM_HAL_INTERRUPT_LPTIMER,         //58 LPTimer int
    CYGNUM_HAL_INTERRUPT_PORTA,           //59 Port A int
    CYGNUM_HAL_INTERRUPT_PORTB,           //60 Port B int
    CYGNUM_HAL_INTERRUPT_PORTC,           //61 Port C int
    CYGNUM_HAL_INTERRUPT_PORTD,           //62 Port D int
    CYGNUM_HAL_INTERRUPT_PORTE,           //63 Port E int        
    CYGNUM_HAL_INTERRUPT_SWI,             // Software interrupt
    CYGNUM_HAL_INTERRUPT_Reserved81,      // Reserved interrupt 81    
    CYGNUM_HAL_INTERRUPT_Reserved82,      // Reserved interrupt 82   
    CYGNUM_HAL_INTERRUPT_Reserved83,      // Reserved interrupt 83   
    CYGNUM_HAL_INTERRUPT_Reserved84,      // Reserved interrupt 84   
    CYGNUM_HAL_INTERRUPT_Reserved85,      // Reserved interrupt 85   
    CYGNUM_HAL_INTERRUPT_Reserved86,      // Reserved interrupt 86   
    CYGNUM_HAL_INTERRUPT_FTM3,            // FTM3 fault, overflow and channels interrupt
    CYGNUM_HAL_INTERRUPT_DAC1,            // DAC1 int        
    CYGNUM_HAL_INTERRUPT_ADC1,            // ADC1 int
    CYGNUM_HAL_INTERRUPT_Reserved90,      // Reserved interrupt 90
    CYGNUM_HAL_INTERRUPT_Reserved91,      // Reserved interrupt 91  
    CYGNUM_HAL_INTERRUPT_Reserved92,      // Reserved interrupt 92  
    CYGNUM_HAL_INTERRUPT_Reserved93,      // Reserved interrupt 93  
    CYGNUM_HAL_INTERRUPT_Reserved94,      // Reserved interrupt 94  
    CYGNUM_HAL_INTERRUPT_Reserved95,      // Reserved interrupt 95  
    CYGNUM_HAL_INTERRUPT_Reserved96,      // Reserved interrupt 96  
    CYGNUM_HAL_INTERRUPT_Reserved97,      // Reserved interrupt 97  
    CYGNUM_HAL_INTERRUPT_Reserved98,      // Reserved interrupt 98  
    CYGNUM_HAL_INTERRUPT_Reserved99,      // Reserved interrupt 99  
    CYGNUM_HAL_INTERRUPT_Reserved100,     // Reserved interrupt 100  
    CYGNUM_HAL_INTERRUPT_Reserved101      // Reserved interrupt 101  
    
} KinetisExtInterrupt_e;

#define CYGNUM_HAL_INTERRUPT_NVIC_MAX (CYGNUM_HAL_INTERRUPT_Reserved101)

#define CYGNUM_HAL_ISR_MIN            0
#define CYGNUM_HAL_ISR_MAX            CYGNUM_HAL_INTERRUPT_Reserved101
#define CYGNUM_HAL_ISR_COUNT          (CYGNUM_HAL_ISR_MAX + 1)

#define CYGNUM_HAL_VSR_MIN            0
#ifndef CYGNUM_HAL_VSR_MAX
# define CYGNUM_HAL_VSR_MAX           (CYGNUM_HAL_VECTOR_SYS_TICK+ \
                                       CYGNUM_HAL_INTERRUPT_NVIC_MAX)
#endif
//=============================================================================

//-----------------------------------------------------------------------------
// end of plf_intr.h
#endif // CYGONCE_HAL_PLF_INTR_H
