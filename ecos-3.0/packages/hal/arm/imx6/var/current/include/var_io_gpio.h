#ifndef CYGONCE_HAL_VAR_IO_GPIO_H
#define CYGONCE_HAL_VAR_IO_GPIO_H
//===========================================================================
//
//      var_io_gpio.h
//
//      iMX6 GPIO
//
//===========================================================================
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
//===========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):     Mike Jones <mike@proclivis.com>
// Origonal:	  Tomas Frydrych <tomas@sleepfive.com>
// Date:          2014-03-29
// Purpose:       iMX6 variant specific registers
// Description:
// Usage:         #include <cyg/hal/var_io.h>  // var_io.h includes this file
//
//####DESCRIPTIONEND####
//
//===========================================================================


//---------------------------------------------------------------------------
// GPIO
typedef volatile struct cyghwr_hal_imx6_gpio_s {
  cyg_uint32 dr;
  cyg_uint32 gdir;
  cyg_uint32 psr;
  cyg_uint32 icr1;
  cyg_uint32 icr2;
  cyg_uint32 imr;
  cyg_uint32 isr;
} cyghwr_hal_imx6_gpio_t;

// PTA-PTE base pointers
#define CYGHWR_HAL_IMX6_GPIO_PORTA_P ((cyghwr_hal_imx6_gpio_t*)0x0209C000u)
#define CYGHWR_HAL_IMX6_GPIO_PORTB_P ((cyghwr_hal_imx6_gpio_t*)0x020A0000u)
#define CYGHWR_HAL_IMX6_GPIO_PORTC_P ((cyghwr_hal_imx6_gpio_t*)0x020A4000u)
#define CYGHWR_HAL_IMX6_GPIO_PORTD_P ((cyghwr_hal_imx6_gpio_t*)0x020A8000u)
#define CYGHWR_HAL_IMX6_GPIO_PORTE_P ((cyghwr_hal_imx6_gpio_t*)0x020AC000u)
#define CYGHWR_HAL_IMX6_GPIO_PORTF_P ((cyghwr_hal_imx6_gpio_t*)0x020B0000u)
#define CYGHWR_HAL_IMX6_GPIO_PORTG_P ((cyghwr_hal_imx6_gpio_t*)0x020B4000u)

// GPIO register on a given port (register name is lower case)
#define CYGHWR_HAL_IMX6_GPIO(__port, __reg)           \
  (CYGHWR_HAL_IMX6_GPIO_PORT##__port##_P)->__reg

// Get values for entire port
#define CYGHWR_HAL_IMX6_GPIO_GET(__port)              \
  CYGHWR_HAL_IMX6_GPIO(__port, psr)

// Output values for entire port
#define CYGHWR_HAL_IMX6_GPIO_PUT(__port, __val)       \
  CYGHWR_HAL_IMX6_GPIO(__port, dr) = __val

// Set values for entire port based on bitmask
#define CYGHWR_HAL_IMX6_GPIO_SET(__port, __val)       \
  CYGHWR_HAL_IMX6_GPIO(__port, dr) |= __val

// Clear values for entire port based on bitmask
#define CYGHWR_HAL_IMX6_GPIO_CLEAR(__port, __val)     \
  CYGHWR_HAL_IMX6_GPIO(__port, dr) &= ~__val

// Toggle values for entire port based on bitmask
#define CYGHWR_HAL_IMX6_GPIO_TOGGLE(__port, __val)    \
  CYGHWR_HAL_IMX6_GPIO(__port, dr) ^= __val

// Get value for a single pin on given port
#define CYGHWR_HAL_IMX6_GPIO_GET_PIN(__port, __pin)   \
  (BIT_(__pin) &  CYGHWR_HAL_IMX6_GPIO_GET(__port))

// Set a single pin on a given register
#define CYGHWR_HAL_IMX6_GPIO_SET_PIN(__port, __pin)   \
  CYGHWR_HAL_IMX6_GPIO_SET(__port, BIT_(__pin))

// Clear a single pin on a given register
#define CYGHWR_HAL_IMX6_GPIO_CLEAR_PIN(__port, __pin) \
  CYGHWR_HAL_IMX6_GPIO_CLEAR(__port, BIT_(__pin))

// Toggle a single pin on a given register
#define CYGHWR_HAL_IMX6_GPIO_TOGGLE_PIN(__port, __pin)  \
  CYGHWR_HAL_IMX6_GPIO_TOGGLE(__port, BIT_(__pin))

// Set pin data direction
#define CYGHWR_HAL_IMX6_GPIO_PIN_DDR_OUT(__port, __pin) \
  CYGHWR_HAL_IMX6_GPIO(__port, gdir) |= BIT_(__pin)

#define CYGHWR_HAL_IMX6_GPIO_PIN_DDR_IN(__port, __pin)  \
  CYGHWR_HAL_IMX6_GPIO(__port, gdir) &= ~BIT_(__pin)

// Set interrupt configuration
#define CYGHWR_HAL_IMX6_GPIO_PIN_INT_CONF(__port, __pin, __con) \
	if (__pin < 16) { \
		CYGHWR_HAL_IMX6_GPIO(__port, icr1) &= ~(0x3 << (__pin * 2)); \
		CYGHWR_HAL_IMX6_GPIO(__port, icr1) |= __con << (__pin * 2); \
		} \
	else { \
		CYGHWR_HAL_IMX6_GPIO(__port, icr2) &= ~(0x3 << ((__pin - 16) * 2)); \
		CYGHWR_HAL_IMX6_GPIO(__port, icr2) |= __con << ((__pin - 16) * 2); \
		}

// Set interrupt mask
#define CYGHWR_HAL_IMX6_GPIO_PIN_MASK_SET(__port, __pin) \
		CYGHWR_HAL_IMX6_GPIO(__port, imr) |= BIT_(__pin)

#define CYGHWR_HAL_IMX6_GPIO_PIN_MASK_CLEAR(__port, __pin) \
		CYGHWR_HAL_IMX6_GPIO(__port, imr) &= ~BIT_(__pin)

// Status register
#define CYGHWR_HAL_IMX6_GPIO_PIN_STATUS_GET(__port, __pin) \
		BIT_(__pin) & CYGHWR_HAL_IMX6_GPIO(__port, isr)

#define CYGHWR_HAL_IMX6_GPIO_PIN_STATUS_CLEAR(__port, __pin) \
		CYGHWR_HAL_IMX6_GPIO(__port, isr) |= BIT_(__pin)


//-----------------------------------------------------------------------------
// end of var_io_gpio.h
#endif // CYGONCE_HAL_VAR_IO_GPIO_H
