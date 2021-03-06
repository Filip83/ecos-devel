#ifndef CYGONCE_DEVS_KBD_BUTTON_KINETIS_H
#define CYGONCE_DEVS_KBD_BUTTON_KINETIS_H
//==========================================================================
//
//      kbd_button.h
//
//      Freescale Kinetis button keyboard driver defines
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
// Date:          2017-03-29
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

#include <pkgconf/devs_kbd_button_kinetis.h>
#include <cyg/io/kbd_io.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>


/** Matrix keyboard row pin interrupt data
*
*/
typedef struct cyg_pins_isr_s
{
    cyg_interrupt     kbd_pin_interrupt;	/**< Pin interrupt object. */  
    cyg_handle_t      kbd_pin_interrupt_handle; /**< Pin interrupt handle. */ 
    cyg_vector_t      kbd_pin_interrupt_number; /**< Pin interrupt number. */ 
}cyg_pins_isr_t;

#if CYGNUM_DEVS_KBD_BUTTON_LINUX_KEYBOARD == 1
typedef cyg_uint16 cyg_kbd_key_t;
#else
typedef cyg_uint8 cyg_kbd_key_t;
#endif

/** Matrix keyboard driver data structure
*
*/
typedef struct cyg_kbd_kinetis_s
{
    cyg_uint32        scan_interval;    /**< Interval in which keys are scaned. */
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
    cyg_pins_isr_t    kb_pins_isr[1]; /**< Keyboard rows pins interrupts. \see cyg_pins_isr_t */
    kbd_call_back_t   kbd_call_back;
#ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
    cyg_kbd_key_t     key_buffer[CYGNUM_DEVS_KBD_BUTTON_EVENT_BUFFER_SIZE];
    cyg_uint16        num_events;
    cyg_uint16        event_put;
    cyg_uint16        event_get;
    cyg_bool          kbd_select_active;
#endif
    cyg_selinfo       kbd_select_info; 
}cyg_kbd_kinetis_t;


#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif // CYGONCE_DEVS_KBD_BUTTON_KINETIS_H

/** @} */
//-----------------------------------------------------------------------------
// End of kbd_button.h
