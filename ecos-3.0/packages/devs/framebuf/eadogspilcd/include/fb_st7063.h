#ifndef CYGONCE_DEVS_FB_ST7063_H
#define CYGONCE_DEVS_FB_ST7063_H
//==========================================================================
//
//      fb_st7063.h
//
//      ST7063 LCD driver defines
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
// Date:          2016-11-23
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
#include <pkgconf/devs_fb_st7063.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
    
#define LCD_SET_COL_ADDR_LSB		0x00
#define LCD_SET_COL_ADDR_MSB		0x10
#define LCD_SET_TEMP_COMPENSATION	0x24
#define LCD_SET_PANEL_LOADING		0x28
#define LCD_SET_PUMP_CONTROL		0x2c
#define LCD_SET_ADV_CONTROL		0x30
#define LCD_SET_SCROLL_LINE_LSB		0x40
#define LCD_SET_SCROLL_LINE_MSB		0x50
#define LCD_SET_PAGE_ADDR		0x60
#define LCD_SET_PAGE_ADDR_MSB		0x70

#define LCD_SET_BIAS_POT		0x81
#define LCD_SET_PARTIAL_CONTROL		0x84
#define LCD_SET_RAM_ADDR_CONTROL	0x88
#define LCD_SET_FIXED_LINES		0x90
#define LCD_SET_LINE_RATE		0xa0
#define LCD_SET_ALL_PX_ON		0xa4
#define LCD_SET_INVERS_DISPLAY		0xa6
#define LCD_SET_DISPLAY_ENABLE		0xa8
#define LCD_SET_MAPPING_CONTROL		0xc0
#define LCD_GRAY_SCALE			0xd0
#define LCD_SYSTEM_RESET		0xe2
#define LCD_NOP				0xe3
#define LCD_SET_TEST_CONTROL		0xe4
#define LCD_SET_BIAS_RATIO		0xe8
#define LCD_RESET_CURSOR_UPDATE_MODE    0xee
#define LCD_SET_CURSOR_UPDATE_MODE	0xef
#define LCD_SET_COM_END			0xf1
#define LCD_SET_PATRIAL_DISP_START	0xf2
#define LCD_SET_PATRIAL_DISP_END	0xf3
#define LCD_SET_START_COL_ADDR		0xf4
#define LCD_SET_START_PAGE_ADDR		0xf5
#define LCD_SET_ENDING_COL_ADDR		0xf6
#define LCD_SET_ENDING_PAGE_ADDR	0xf7
#define LCD_ENABLE_WND_PROGRAM		0xf8

#define LCD_COL_ADDR_MASK		0x0f
#define LCD_SCROLL_LINE_MASK		0x0f
#define LCD_PAGE_ADDR_MASK		0x1f
#define LCD_FIXE_LINE_MASK		0x0f

#define LCD_TEMP_COMP_005		0x00
#define LCD_TEMP_COMP_01		0x01
#define LCD_TEMP_COMP_015		0x02
#define LCD_TEMP_COMP_02		0x03

#define LCD_PANEL_LOADING_16nF		0x00
#define LCD_PANEL_LOADING_16_21nF	0x01
#define LCD_PANEL_LOADING_21_28nF	0x02
#define LCD_PANEL_LOADING_28_39nF	0x03

#define LC_PUMP_CONTROL_EXTERNAL	0x00
#define LC_PUMP_CONTROL_7X		0x01
#define LC_PUMP_CONTROL_6X		0x02
#define LC_PUMP_CONTROL_8X		0x03

#define LCD_PATRIL_CONTROL_DISABLE	0x00
#define LCD_PATRIL_CONTROL_CEN_1	0x02
#define LCD_PATRIL_CONTROL_DEN_DST	0x03

#define LCD_ADDR_CONTROL_AC0		0x01
#define LCD_ADDR_CONTROL_AC1		0x02
#define LCD_ADDR_CONTROL_AC2		0x04

#define LCD_LINE_RATE_12K		0x00
#define LCD_LINE_RATE_13K		0x01
#define LCD_LINE_RATE_14K		0x02
#define LCD_LINE_RATE_16K		0x03

#define LCD_ALL_PIXELS_ON		0x01
#define LCD_ENABLE_INVERS_DISP		0x01
#define LCD_SET_DISPLAY_ON		0x01

#define LCD_SET_GRAY_SCALE_24		0x00
#define LCD_SET_GRAY_SCALE_29		0x01
#define LCD_SET_GRAY_SCALE_36		0x02
#define LCD_SET_GRAY_SCALE_40		0x03

#define LCD_BIAS_RATIO_5		0x00
#define LCD_BIAS_RATIO_10		0x01
#define LCD_BIAS_RATIO_11		0x02
#define LCD_BIAS_RATIO_12		0x03


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

#endif // CYGONCE_DEVS_FB_ST7063_H

/** @} */
//-----------------------------------------------------------------------------
// End of fb_st7063.h
