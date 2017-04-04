//==========================================================================
//
//      avr32_L3U_L4U_ISR_Vectors.h
//
//      HAL Chip Interrupt support
//
//==========================================================================
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Filip
// Contributors:
// Date:         2016-10-26
// Purpose:      Define Chip Interrupt Vectors
// Description:  The macros defined here provide the HAL APIs for handling
//               both external interrupts and clock interrupts.
//
//
//
//####DESCRIPTIONEND####
//
//==========================================================================

#ifndef AVR32_L3U_L4U_ISR_VECTORS_H
#define AVR32_L3U_L4U_ISR_VECTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#define CYGHWR_HAL_INTERRUPT_VECTORS_DEFINED
#define CYGNUM_HAL_VECTOR_SYSTEM_TIMER          0x00
#define CYGNUM_HAL_VECTOR_OCD_DIRTY             0x20
#define CYGNUM_HAL_VECTOR_OCD_READ              0x21
    
#define CYGNUM_HAL_VECTOR_FLASHCDW              0x40
    
#define CYGNUM_HAL_VECTOR_SAU                   0x60

#define CYGNUM_HAL_VECTOR_PDMA_CH0              0x80
#define CYGNUM_HAL_VECTOR_PDMA_CH1          	0x81
#define CYGNUM_HAL_VECTOR_PDMA_CH2          	0x82
#define CYGNUM_HAL_VECTOR_PDMA_CH3          	0x83

#define CYGNUM_HAL_VECTOR_PDMA_CH4          	0xa0
#define CYGNUM_HAL_VECTOR_PDMA_CH5          	0xa1
#define CYGNUM_HAL_VECTOR_PDMA_CH6          	0xa2
#define CYGNUM_HAL_VECTOR_PDMA_CH7          	0xa3

#define CYGNUM_HAL_VECTOR_PDMA_CH8          	0xc0
#define CYGNUM_HAL_VECTOR_PDMA_CH9          	0xc1
#define CYGNUM_HAL_VECTOR_PDMA_CH10         	0xc2
#define CYGNUM_HAL_VECTOR_PDMA_CH11         	0xc3
    
#define CYGNUM_HAL_VECTOR_PMC           	0xe0
    
#define CYGNUM_HAL_VECTOR_SCIF           	0x100    
    
#define CYGNUM_HAL_VECTOR_AST_ALARM        	0x120
#define CYGNUM_HAL_VECTOR_AST_PER        	0x140
#define CYGNUM_HAL_VECTOR_AST_OVF       	0x141
#define CYGNUM_HAL_VECTOR_AST_READY        	0x142
#define CYGNUM_HAL_VECTOR_AST_CLKREDY      	0x143

#define CYGNUM_HAL_VECTOR_EIC_1         	0x160
#define CYGNUM_HAL_VECTOR_EIC_2         	0x161
#define CYGNUM_HAL_VECTOR_EIC_3         	0x162
#define CYGNUM_HAL_VECTOR_EIC_4         	0x163
#define CYGNUM_HAL_VECTOR_EIC_5         	0x180

#define CYGNUM_HAL_VECTOR_FREQM         	0x1a0
    
#define CYGNUM_HAL_VECTOR_GPIO_0        	0x1c0
#define CYGNUM_HAL_VECTOR_GPIO_1        	0x1c1
#define CYGNUM_HAL_VECTOR_GPIO_2        	0x1c2
#define CYGNUM_HAL_VECTOR_GPIO_3        	0x1c3
#define CYGNUM_HAL_VECTOR_GPIO_4        	0x1c4
#define CYGNUM_HAL_VECTOR_GPIO_5        	0x1c5
#define CYGNUM_HAL_VECTOR_GPIO_6        	0x1c6
#define CYGNUM_HAL_VECTOR_GPIO_7        	0x1c7

#define CYGNUM_HAL_VECTOR_USART0        	0x1e0
#define CYGNUM_HAL_VECTOR_USART1        	0x200
#define CYGNUM_HAL_VECTOR_USART2        	0x220
#define CYGNUM_HAL_VECTOR_USART3        	0x240    
    
#define CYGNUM_HAL_VECTOR_SPI0          	0x260
    
#define CYGNUM_HAL_VECTOR_TWIM0          	0x280
#define CYGNUM_HAL_VECTOR_TWIM1          	0x2a0

#define CYGNUM_HAL_VECTOR_TWIS0          	0x2c0
#define CYGNUM_HAL_VECTOR_TWIS1          	0x2e0
    
#define CYGNUM_HAL_VECTOR_PWM           	0x300
    
#define CYGNUM_HAL_VECTOR_TC00          	0x320
#define CYGNUM_HAL_VECTOR_TC01          	0x321
#define CYGNUM_HAL_VECTOR_TC02          	0x322

#define CYGNUM_HAL_VECTOR_TC10          	0x340
#define CYGNUM_HAL_VECTOR_TC11          	0x341
#define CYGNUM_HAL_VECTOR_TC12          	0x342    
   
#define CYGNUM_HAL_VECTOR_ADCIFB          	0x360    
   
#define CYGNUM_HAL_VECTOR_ACIFB         	0x380
    
#define CYGNUM_HAL_VECTOR_CAT             	0x3a0    
    
#define CYGNUM_HAL_VECTOR_AW                    0x3c0    
    
#define CYGNUM_HAL_VECTOR_ABDACB                0x3e0    
    
#define CYGNUM_HAL_VECTOR_USB                	0x400

#define CYGNUM_HAL_VECTOR_IISC          	0x420    
    
    



// The interrupt vector used by the RTC, aka tick timer
#define CYGNUM_HAL_INTERRUPT_RTC                CYGNUM_HAL_VECTOR_SYSTEM_TIMER

// Min/Max ISR numbers and how many there are
#define CYGNUM_HAL_ISR_MIN                      0
#define CYGNUM_HAL_ISR_MAX                      CYGNUM_HAL_VECTOR_IISC
#define CYGNUM_HAL_ISR_COUNT                    33


#ifdef __cplusplus
}
#endif

#endif /* AVR32_L3U_L4U_ISR_VECTORS_H */

