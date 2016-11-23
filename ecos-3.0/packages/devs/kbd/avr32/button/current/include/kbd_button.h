#ifndef CYGONCE_DEVS_KBD_BUTTON_UC3C_H
#define CYGONCE_DEVS_KBD_BUTTON_UC3C_H
//==========================================================================
//
//      kbd_button.h
//
//      Atmel AVR32 button keyboard driver defines
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
// Author(s):     Filip Adamec
// Date:          2013-04-05
//
//####DESCRIPTIONEND####
//
//==========================================================================

/**
 * @addtogroup Drivers
 *
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif
	
#include <pkgconf/hal.h>
#include <pkgconf/devs_kbd_button.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>


/** @} */

//#define BUFFERED_KEYBOARD
//-----------------------------------------------------------------------------
// AVR32 KBD driver data structure

/** Matrix keyboard row pin interrupt data
*
*/
typedef struct cyg_pins_isr_s
{
    cyg_interrupt     kbd_pin_interrupt;	/**< Pin interrupt object. */  
    cyg_handle_t      kbd_pin_interrupt_handle; /**< Pin interrupt handle. */ 
    cyg_vector_t      kbd_pin_interrupt_number; /**< Pin interrupt number. */ 
}cyg_pins_isr_t;

typedef struct cyg_kbd_key_s
{
    cyg_uint16 key;
    cyg_uint16 param;
}cyg_kbd_key_t;

/** Matrix keyboard driver data structure
*
*/
typedef struct cyg_kbd_avr32_s
{
    cyg_uint32        repeat_interval;	/**< Interval between keyboard scans. */ 
    cyg_uint32	      push_cnt;		/**< Interval counter to count start of scan interval. */ 
    cyg_uint16        last_scan_code;	/**< Latest scan code. */ 
    cyg_uint8         scan_line;	/**< Current scanned line/column. */ 
    cyg_uint8         glitch_cnt;       /**< Glitch filter number of counts. */
    cyg_uint32	      scan_code;	/**< Current scan code. */ 
    cyg_bool          is_open;		/**< True if matrix keyboard is initialized. */
    cyg_bool          enabled;          /**< Enable/Disable key messages. */
    cyg_interrupt     kbd_interrupt;        /**< Keyboard timer interrupt object. */
    cyg_handle_t      kbd_interrupt_handle; /**< Keyboard timer interrupt handle. */
    cyg_vector_t      interrupt_number;     /**< Keyboard timer interrupt number. */
    cyg_uint32        interrupt_prio;	    /**< Keyboard interrupts priority. */
    cyg_pins_isr_t    kb_pins_isr[CYGNUM_DEVS_KBD_MATRIX_ISR_PINS]; /**< Keyboard rows pins interrupts. \see cyg_pins_isr_t */
    (void (*call_back)(cyg_uint16, cyg_uint16)) kbd_callback;
#if CYGNUM_DEVS_KBD_MATRIX_CALLBACK_MODE == 0
    cyg_kbd_key_t     key_buffer[CYGNUM_DEVS_KBD_MATRIX_EVENT_BUFFER_SIZE];
    cyg_uint16        num_events;
    cyg_uint16        event_put;
    cyg_bool          kbd_select_active;
    cyg_selinfo       kbd_select_info; 
#endif
}cyg_kbd_avr32_t;


#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif // CYGONCE_DEVS_KBD_BUTTON_UC3C_H

/** @} */
//-----------------------------------------------------------------------------
// End of matrix_kbd.h
