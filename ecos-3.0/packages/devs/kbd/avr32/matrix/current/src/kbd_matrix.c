//==========================================================================
//
//      kbd_matrix.c
//
//      Keyboard driver for the AVR32 matrix keyboard
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
// Author(s):    Filip <filip.gnu@gmail.com>
// Contributors:
// Date:         2016-11-23
// Purpose:
// Description:  Matrix keyboardd driver for AVR32
//
//####DESCRIPTIONEND####
//
//==========================================================================


#include <pkgconf/devs_kbd_matrix_avr32.h>

#include <cyg/kernel/kapi.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/linux_keyboard.h>
#include <cyg/io/kbd_io.h>

#include <cyg/hal/gpio.h>

#include <cyg/fileio/fileio.h>  // For select() functionality

#include <cyg/io/kbd_matrix.h>

#include <cyg/io/devtab.h>
#include CYGBLD_HAL_BOARD_H

#define MAX_EVENTS CYGNUM_DEVS_KBD_MATRIX_EVENT_BUFFER_SIZE

/** Declaration and initialization of matrix keyborad data structure.
*
*/
static cyg_kbd_avr32_t cyg_kbd_avr32 =
{
    .repeat_interval        = 5,
    .push_cnt		    = 0,
    .last_scan_code         = 0,
    .scan_code		    = 0,
    .scan_line              = 0,
    .is_open                = false,
    .enabled                = true,
    .kbd_interrupt          = NULL,
    .kbd_interrupt_handle   = NULL,
    .interrupt_number       = CYGNUM_HAL_VECTOR_AST_PER,
    .interrupt_prio         = CYGNUM_DEVS_KBD_MATRIX_INTERRUPT_PRIO,
    .kb_pins_isr[0].kbd_pin_interrupt		= NULL,
    .kb_pins_isr[0].kbd_pin_interrupt_handle	= NULL,
    .kb_pins_isr[0].kbd_pin_interrupt_number	= CYGNUM_HAL_VECTOR_EIC_1,

    .kb_pins_isr[1].kbd_pin_interrupt		= NULL,
    .kb_pins_isr[1].kbd_pin_interrupt_handle	= NULL,
    .kb_pins_isr[1].kbd_pin_interrupt_number	= CYGNUM_HAL_VECTOR_EIC_2,

    .kb_pins_isr[2].kbd_pin_interrupt		= NULL,
    .kb_pins_isr[2].kbd_pin_interrupt_handle	= NULL,
    .kb_pins_isr[2].kbd_pin_interrupt_number	= CYGNUM_HAL_VECTOR_EIC_3,

    .kb_pins_isr[3].kbd_pin_interrupt		= NULL,
    .kb_pins_isr[3].kbd_pin_interrupt_handle	= NULL,
    .kb_pins_isr[3].kbd_pin_interrupt_number	= CYGNUM_HAL_VECTOR_EIC_4,
    .kbd_call_back                              = NULL,
#ifdef CYGNUM_DEVS_KBD_MATRIX_CALLBACK_MODE
    .num_events                                 = 0,
    .event_put                                  = 0,
    .event_get                                  = 0,
    .kbd_select_active                          = false,
#endif
};

// Functions in this module

static Cyg_ErrNo kbd_read(cyg_io_handle_t handle,
                          void *buffer,
                          cyg_uint32 *len);
static cyg_bool  kbd_select(cyg_io_handle_t handle,
                            cyg_uint32 which,
                            cyg_addrword_t info);
static Cyg_ErrNo kbd_set_config(cyg_io_handle_t handle,
                                cyg_uint32 key,
                                const void *buffer,
                                cyg_uint32 *len);
static Cyg_ErrNo kbd_get_config(cyg_io_handle_t handle,
                                cyg_uint32 key,
                                void *buffer,
                                cyg_uint32 *len);
static bool      kbd_init(struct cyg_devtab_entry *tab);
static Cyg_ErrNo kbd_lookup(struct cyg_devtab_entry **tab,
                            struct cyg_devtab_entry *st,
                            const char *name);


CHAR_DEVIO_TABLE(kbd_handlers,
                 NULL,                            // Unsupported write() function
                 kbd_read,
                 kbd_select,
                 kbd_get_config,
                 kbd_set_config);

CHAR_DEVTAB_ENTRY(kbd_device,
                  CYGDAT_DEVS_KBD_MATRIX_NAME,
                  NULL,                           // Base device name
                  &kbd_handlers,
                  kbd_init,
                  kbd_lookup,
                  &cyg_kbd_avr32);                // Private data pointer


static cyg_uint32
avr32_matrix_kbd_timer_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void       
avr32_matrix_kbd_timer_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static cyg_uint32
avr32_matrix_kbd_pin_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void
avr32_matrix_kbd_pin_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

/** Function to convert scan code to linux keyboard key mapping.
*
* \param scan_code is keyboard scan code.
* \return linux keyboard key code.
*/
static cyg_uint16 kbd_scan_code_to_key_gs4x5(cyg_uint32 scan_code)
{
#if CYGDAT_DEVS_KBD_MATRIX_DEBUG_OUTPUT > 0
    diag_printf("Kbd dev scan Code: 0x%X\n",scan_code);
#endif
    switch(scan_code)
    {
        case 0x10000:
        return K_ENTER;
        break;
        case 0x20000:
        return K_SAK;
        break;
        case 0x40000:
        return K_PGDN;
        break;
        case 0x04:
        return K_UP;
        break;
        case 0x008:
        return K_DOWN;
        break;
        /*case 0x0400:
        return K_F3;
        break;*/
        case 0x8000:
        return K_RIGHT;//'6';
        break;
        /*case 0x0020:
        return K_F3;//'5';*/
        break;
        case 0x0080:
        return K_LEFT;//'4';
        break;
        case 0x01:
        return K_F1;//'9';
        break;
        case 0x0002:
        return K_F2;//'7';
        break;
        case (0x008 | 0x0004):
        return K_F10;
        break;

        case 0x0800:
        return '0';
        break;
        case 0x00040:
        return '1';
        break;
        case 0x00400:
        return '2';
        break;
        case 0x4000:
        return '3';
        break;
        case 0x00020:
        return '4';
        break;
        case 0x00200:
        return '5';
        break;
        case 0x2000:
        return '6';
        break;
        case 0x00010:
        return '7';
        break;
        case 0x00100:
        return '8';
        break;
        case 0x1000:
        return '9';
        break;
    }

    return 0;
}

/** Function to convert scan code to linux keyboard key mapping.
*
* \param scan_code is keyboard scan code.
* \return linux keyboard key code.
*/
static cyg_uint16 kbd_scan_code_to_key_pmg4x4(cyg_uint32 scan_code)
{
#if CYGDAT_DEVS_KBD_MATRIX_DEBUG_OUTPUT > 0
    diag_printf("Kbd dev scan Code: 0x%X\n",scan_code);
#endif
    switch(scan_code)
    {
        case 0x1000:
        return K_ENTER;
        break;
        case 0x2000:
        return K_SAK;
        break;
        case 0x4000:
        return K_PGDN;
        break;
        case 0x0800:
        return K_UP;
        break;
        case 0x008:
        return K_DOWN;
        break;
        case (0x008 | 0x0800):
        return K_F10;
        break;
        case 0x080:
        return '0';
        break;
        case 0x0004:
        return '1';
        break;
        case 0x0040:
        return '2';
        break;
        case 0x0400:
        return '3';
        break;
        case 0x00002:
        return '4';
        break;
        case 0x00020:
        return '5';
        break;
        case 0x200:
        return '6';
        break;
        case 0x0001:
        return '7';
        break;
        case 0x0010:
        return '8';
        break;
        case 0x100:
        return '9';
        break;
    }

    return 0;
}

static Cyg_ErrNo
kbd_read(cyg_io_handle_t handle,
         void *buffer,
         cyg_uint32 *len)
{
#ifdef CYGNUM_DEVS_KBD_MATRIX_CALLBACK_MODE
    cyg_kbd_avr32_t * priv = (cyg_kbd_avr32_t*)handle;

    cyg_kbd_key_t *ev;
    int tot = *len;
    cyg_kbd_key_t *bp = (cyg_kbd_key_t*)buffer;

    cyg_scheduler_lock();  // Prevent interaction with DSR code
    while (tot >= sizeof(*ev)) {
        if (priv->num_events > 0) {
            ev = &_events[priv->event_get++];
            if (priv->event_get == MAX_EVENTS) {
                priv->event_get = 0;
            }
            memcpy(bp, ev, sizeof(*ev));
            bp += sizeof(*ev);
            tot -= sizeof(*ev);
            priv->num_events--;
        } else {
            break;  // No more events
        }
    }
    cyg_scheduler_unlock(); // Allow DSRs again
    diag_dump_buf(buffer, tot);
    *len -= tot;
    return ENOERR;
#else
    return EINVAL;
#endif
}

static cyg_bool
kbd_select(cyg_io_handle_t handle,
           cyg_uint32 which,
           cyg_addrword_t info)
{
#ifdef CYGNUM_DEVS_KBD_MATRIX_CALLBACK_MODE
    cyg_kbd_avr32_t * priv = (cyg_kbd_avr32_t*)handle;
    if (which == CYG_FREAD) {
        cyg_scheduler_lock();  // Prevent interaction with DSR code
        if (priv->num_events > 0) {
            cyg_scheduler_unlock();  // Reallow interaction with DSR code
            return true;
        }
        if (!priv->kbd_select_active) {
            priv->kbd_select_active = true;
            cyg_selrecord(info, &priv->kbd_select_info);
        }
        cyg_scheduler_unlock();  // Reallow interaction with DSR code
    }
    return false;
#else
    return false;
#endif
}

static Cyg_ErrNo
kbd_set_config(cyg_io_handle_t handle,
               cyg_uint32 key,
               const void *buffer,
               cyg_uint32 *len)
{
    cyg_kbd_avr32_t * priv = (cyg_kbd_avr32_t*)handle;
    cyg_uint32 ret = ENOERR;
    cyg_scheduler_lock();  // Prevent interaction with DSR code
    switch(key)
    {
        case CYG_KBD_SET_CONFIG_INTERVAL:
        {
            if(*len == sizeof(cyg_uint32))
            {
                cyg_uint32 *repeat = (cyg_uint32*)buffer;
                if(repeat > 2 && repeat < 50)
                    priv->repeat_interval = *repeat;
                else
                    ret = EINVAL;
            }
            else
                ret = EINVAL;
        }
        break;
        case CYG_KBD_SET_CONFIG_ENABLE:
            priv->enabled = true;
            break;
        case CYG_KBD_SET_CONFIG_DISABLE:
            priv->enabled = false;
            break;
        case CYG_KBD_SET_CONFIG_CALLBACK:
        {
            kbd_call_back_t kbd_callback =
                    (kbd_call_back_t)buffer;
            priv->kbd_call_back = kbd_callback;
        }
        break;
        case CYG_KBD_SET_CONFIG_CALLBACK_REMOVE:
            priv->kbd_call_back = NULL;
            break;
        case CYG_KBD_SET_CONFIG_DEFAULT_INTERVAL:
            priv->repeat_interval = 20;
        default:
            ret = EINVAL;
    }
    cyg_scheduler_unlock();  // Reallow interaction with DSR code
    return ret;
}

static Cyg_ErrNo
kbd_get_config(cyg_io_handle_t handle,
               cyg_uint32 key,
               void *buffer,
               cyg_uint32 *len)
{
    cyg_kbd_avr32_t * priv = (cyg_kbd_avr32_t*)handle;
    switch(key)
    {
        case CYG_KBD_GET_CONFIG_INTERVAL:
            if(*len == sizeof(cyg_uint32))
            {
                *((cyg_uint32*)buffer) = priv->repeat_interval;
                return ENOERR;
            }
            break;
        case CYG_KBD_GET_CONFIG_ENABLED:
            if(*len == sizeof(cyg_uint32))
            {
                *((cyg_uint32*)buffer) = priv->enabled;
                return ENOERR;
            }
            break;
    }
    return EINVAL;
}

static bool
kbd_init(struct cyg_devtab_entry *tab)
{
    int i;
    cyg_kbd_avr32_t *kbd_dev = &cyg_kbd_avr32;
    // set first line to scan
    kbd_dev->scan_line          = 0;
    //
    while(AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK)
    {
    }		
    // set pir interrupt interval = fc/2^(pir0 + 1)
    AVR32_AST.pir0 = CYGNUM_DEVS_KBD_MATRIX_SCAN_INTERVAL;

    // Configure GPIO pins used by keyboard
    gpio_configure_pin(AVR32_PIN_PA21, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(AVR32_PIN_PA22, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(AVR32_PIN_PA23, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(AVR32_PIN_PA24, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
    gpio_configure_pin(AVR32_PIN_PA25, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
#endif

    gpio_configure_pin(AVR32_PIN_PA26, GPIO_DIR_INPUT );
    gpio_configure_pin(AVR32_PIN_PA27, GPIO_DIR_INPUT);
    gpio_configure_pin(AVR32_PIN_PA28, GPIO_DIR_INPUT);
    gpio_configure_pin(AVR32_PIN_PA29, GPIO_DIR_INPUT);

    // The keyboard rows are driving external interrupt controller
    gpio_enable_module_pin(AVR32_EIC_EXTINT_1_2_PIN, AVR32_EIC_EXTINT_1_2_FUNCTION);
    gpio_enable_module_pin(AVR32_EIC_EXTINT_2_PIN ,  AVR32_EIC_EXTINT_2_FUNCTION);
    gpio_enable_module_pin(AVR32_EIC_EXTINT_3_PIN,   AVR32_EIC_EXTINT_3_FUNCTION);
    gpio_enable_module_pin(AVR32_EIC_EXTINT_4_PIN,   AVR32_EIC_EXTINT_4_FUNCTION);

    // Init keyboard timer interrupt 
    cyg_drv_interrupt_create(kbd_dev->interrupt_number,
                             kbd_dev->interrupt_prio,   // Priority
                             (cyg_addrword_t)kbd_dev,   // Data item passed to interrupt handler
                             avr32_matrix_kbd_timer_ISR,
                             avr32_matrix_kbd_timer_DSR,
                             &kbd_dev->kbd_interrupt_handle,
                             &kbd_dev->kbd_interrupt);
    cyg_drv_interrupt_attach(kbd_dev->kbd_interrupt_handle);

    // Init keyboard rows interrupt
    for(i = 0; i < CYGNUM_DEVS_KBD_MATRIX_ISR_PINS; i++)
    {
        cyg_drv_interrupt_create(kbd_dev->kb_pins_isr[i].kbd_pin_interrupt_number,
                        kbd_dev->interrupt_prio,                      // Priority
                        (cyg_addrword_t)kbd_dev,   // Data item passed to interrupt handler
                        avr32_matrix_kbd_pin_ISR,
                        avr32_matrix_kbd_pin_DSR,
                        &kbd_dev->kb_pins_isr[i].kbd_pin_interrupt_handle,
                        &kbd_dev->kb_pins_isr[i].kbd_pin_interrupt);
        cyg_drv_interrupt_attach(kbd_dev->kb_pins_isr[i].kbd_pin_interrupt_handle);
    }

    //Configure pin interupt
    AVR32_EIC.mode &= ~(AVR32_EIC_MODE_INT4_MASK | AVR32_EIC_MODE_INT3_MASK |
                        AVR32_EIC_MODE_INT2_MASK | AVR32_EIC_MODE_INT1_MASK);
    //spadovou
    AVR32_EIC.edge |= (AVR32_EIC_EDGE_INT4_MASK  | AVR32_EIC_EDGE_INT3_MASK	|
                       AVR32_EIC_EDGE_INT2_MASK  | AVR32_EIC_EDGE_INT1_MASK);

    AVR32_EIC.en    = AVR32_EIC_EN_INT4_MASK	 | AVR32_EIC_EN_INT3_MASK	|
                      AVR32_EIC_EN_INT2_MASK      | AVR32_EIC_EN_INT1_MASK;

    AVR32_EIC.icr   = AVR32_EIC_ICR_INT4_MASK | AVR32_EIC_ICR_INT3_MASK |
                      AVR32_EIC_ICR_INT2_MASK | AVR32_EIC_ICR_INT1_MASK;

    AVR32_EIC.ier   = AVR32_EIC_IER_INT4_MASK | AVR32_EIC_IER_INT3_MASK |
                      AVR32_EIC_IER_INT2_MASK | AVR32_EIC_IER_INT1_MASK;
        
    cyg_selinit(&kbd_dev->kbd_select_info);
 
    return true;
}

static Cyg_ErrNo
kbd_lookup(struct cyg_devtab_entry **tab,
           struct cyg_devtab_entry *st,
           const char *name)
{
    cyg_kbd_avr32_t * const kbd = (cyg_kbd_avr32_t *) (*tab)->priv;
    
    if(!kbd->is_open)
    {
        kbd_init(*tab);
        kbd_dev->is_open = true;
    }
    return ENOERR;
}


/** Start keyboard scan timer.
*
* The pir0 interrupt is used to driver columns/lines scanning.
* \param kdb_dev is pointer to keyboard data structure.
*/
static void avr32_matrix_kbd_start_scan(cyg_kbd_avr32_t *kbd_dev)
{
    kbd_dev->push_cnt   = 0;
    kbd_dev->scan_line  = 0;
    AVR32_AST.ier       = AVR32_AST_IER_PER0_MASK;
}

/** Stop keyboard columns/lines scanning.
*
* The pir0 interrupt is used to driver columns/lines scanning.
* \param kdb_dev is pointer to keyboard data structure.
*/
static void avr32_matrix_kbd_stop_scan(cyg_kbd_avr32_t *kbd_dev)
{
    kbd_dev->scan_line  = 0;
    AVR32_AST.idr       = AVR32_AST_IDR_PER0_MASK;
}

/** Keyboard timer interrupt.
*
* The pir0 interrupt is handled in this function.
* In this function the rows scanning based on current
* line selection and next line selection is handled hear.
* \param vector is interrupt vector.
* \param data is pointer to driver data structure.
* \return value indicating to OS that DSR nead to be called.
*/
static cyg_uint32
avr32_matrix_kbd_timer_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_kbd_avr32_t *kbd_dev = (cyg_kbd_avr32_t*)data;
    cyg_uint32           ret = CYG_ISR_HANDLED;
    
    // is pir0 interrupt
    if(AVR32_AST.sr&AVR32_AST_SR_PER0_MASK)
    {
        // Clear pir0 interupt
        while(AVR32_AST.sr & AVR32_AST_SR_BUSY_MASK)
        {
        }
        AVR32_AST.scr  = AVR32_AST_SCR_PER0_MASK;
        
        if(kbd_dev->scan_line == 0)
        {
            // If scan line is zero set line to scan
            gpio_set_pin_high(AVR32_PIN_PA21);
            gpio_set_pin_low(AVR32_PIN_PA22);
            gpio_set_pin_low(AVR32_PIN_PA23);
            gpio_set_pin_low(AVR32_PIN_PA24);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            gpio_set_pin_low(AVR32_PIN_PA25);
#endif
            ++kbd_dev->scan_line;
            kbd_dev->scan_code = 0;
        }
        else if(kbd_dev->scan_line == 1)
        {
            // If scan line is non zero scan 
            // for active rows and update scan code
            if(gpio_get_pin_value(AVR32_PIN_PA26))
            {
                kbd_dev->scan_code |= 0x0001;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA27))
            {
                kbd_dev->scan_code |= 0x0002;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA28))
            {
                kbd_dev->scan_code |= 0x0004;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA29))
            {
                kbd_dev->scan_code |= 0x0008;
            }
            // Set next line to scan
            gpio_set_pin_low(AVR32_PIN_PA21);
            gpio_set_pin_high(AVR32_PIN_PA22);
            gpio_set_pin_low(AVR32_PIN_PA23);
            gpio_set_pin_low(AVR32_PIN_PA24);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            gpio_set_pin_low(AVR32_PIN_PA25);
#endif

            ++kbd_dev->scan_line;
        }
        else if(kbd_dev->scan_line == 2)
        {
            if(gpio_get_pin_value(AVR32_PIN_PA26))
            {
                 kbd_dev->scan_code |= 0x0010;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA27))
            {
                 kbd_dev->scan_code |= 0x0020;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA28))
            {
                 kbd_dev->scan_code |= 0x0040;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA29))
            {
                 kbd_dev->scan_code |= 0x0080;
            }

            gpio_set_pin_low(AVR32_PIN_PA21);
            gpio_set_pin_low(AVR32_PIN_PA22);
            gpio_set_pin_high(AVR32_PIN_PA23);
            gpio_set_pin_low(AVR32_PIN_PA24);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            gpio_set_pin_low(AVR32_PIN_PA25);
#endif
            
            ++kbd_dev->scan_line;
        }
        else if(kbd_dev->scan_line == 3)
        {
            if(gpio_get_pin_value(AVR32_PIN_PA26))
            {
                kbd_dev->scan_code |=  0x0100;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA27))
            {
                kbd_dev->scan_code |= 0x0200;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA28))
            {
                kbd_dev->scan_code |= 0x0400;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA29))
            {
                kbd_dev->scan_code |= 0x0800;
            }

            gpio_set_pin_low(AVR32_PIN_PA21);
            gpio_set_pin_low(AVR32_PIN_PA22);
            gpio_set_pin_low(AVR32_PIN_PA23);
            gpio_set_pin_high(AVR32_PIN_PA24);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            gpio_set_pin_low(AVR32_PIN_PA25);
#endif
            ++kbd_dev->scan_line;
        }
        else if(kbd_dev->scan_line == 4)
        {
            if(gpio_get_pin_value(AVR32_PIN_PA26))
            {
                kbd_dev->scan_code |=  0x1000;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA27))
            {
                kbd_dev->scan_code |= 0x2000;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA28))
            {
                kbd_dev->scan_code |= 0x4000;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA29))
            {
                kbd_dev->scan_code |= 0x8000;
            }

            gpio_set_pin_low(AVR32_PIN_PA21);
            gpio_set_pin_low(AVR32_PIN_PA22);
            gpio_set_pin_low(AVR32_PIN_PA23);
            gpio_set_pin_low(AVR32_PIN_PA24);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            gpio_set_pin_low(AVR32_PIN_PA25);
#endif

            ++kbd_dev->scan_line;
        }
        else if(kbd_dev->scan_line == 5)
        {
            if(gpio_get_pin_value(AVR32_PIN_PA26))
            {
                kbd_dev->scan_code |= 0x10000;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA27))
            {
                kbd_dev->scan_code |= 0x20000;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA28))
            {
                kbd_dev->scan_code |= 0x40000;
            }
            if(gpio_get_pin_value(AVR32_PIN_PA29))
            {
               kbd_dev->scan_code |= 0x80000;
            }

            // All keyboard lines was scanned 
            // put all lines to inactive state
            gpio_set_pin_high(AVR32_PIN_PA21);
            gpio_set_pin_high(AVR32_PIN_PA22);
            gpio_set_pin_high(AVR32_PIN_PA23);
            gpio_set_pin_high(AVR32_PIN_PA24);
#ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            gpio_set_pin_low(AVR32_PIN_PA25);
#endif
            // Disable scan timer interrupt and
            // indicate that DSR nead to be processed
            AVR32_AST.idr       = AVR32_AST_IDR_PER0_MASK;
            kbd_dev->scan_line	= 0;
            ret                |= CYG_ISR_CALL_DSR;            
        }
        else
        {
            CYG_ASSERT(false,"Unexpected scan position");
        }
    }
    
    return ret;
}

/** Keyboard timer DSR.
*
* The DSR is called after complete keyboard scanning is done.
* The repeating scanning of the keyboard is handled hear and
* as well sending keyboard event to the application.
* \param vector is interrupt vector.
* \param count is ??
* \param data is pointer to driver data structure.
*/
static void       
avr32_matrix_kbd_timer_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_kbd_avr32_t *kbd_dev = (cyg_kbd_avr32_t*)data;
    
    // If scan code is non zero do some stuff
    // This DSR is called every ca. 0.0390625s
    if(kbd_dev->scan_code != 0)
    {
        // If the repeat interval for pushed button is reached
        if(kbd_dev->push_cnt == 0)
        {
            // Convert scan code to linux keyboard code
            #ifdef CYGNUM_DEVS_KBD_MATRIX_4x5
            cyg_uint16 key = kbd_scan_code_to_key_gs4x5(kbd_dev->scan_code);    
            #else
            cyg_uint16 key = kbd_scan_code_to_key_pmg4x4(kbd_dev->scan_code);
            #endif
            // if latest key is different from current
            // use longer interval
            if(kbd_dev->scan_code != kbd_dev->last_scan_code)
            {
                kbd_dev->push_cnt = 25;
                if(kbd_dev->enabled)
                {
                    #ifdef CYGNUM_DEVS_KBD_MATRIX_CALLBACK_MODE 
                    if (kbd_dev->num_events < CYGNUM_DEVS_KBD_BUFFER_LEN) 
                    {
                        cyg_kbd_key_t *ev;
                        kbd_dev->num_events++;
                        ev = &kbd_dev->key_buffer[kbd_dev->event_put++];
                        if (kbd_dev->event_put == CYGNUM_DEVS_KBD_BUFFER_LEN) {
                            kbd_dev->event_put = 0;
                        }
                        *ev = (cyg_kbd_key_t)key;
                        if (kbd_dev->kbd_select_active)
                        {
                            kbd_dev->kbd_select_active = false;
                            cyg_selwakeup(&kbd_dev->kbd_select_info);
                        }
                    }
                    #endif
                    if(kbd_dev->kbd_call_back != NULL)
                    {
                        kbd_dev->kbd_call_back(key, CYG_KBD_KEY_DOWNO);
                    }
                }
            }
            else
            {
                // otherwise use default repeat interval
                kbd_dev->push_cnt = kbd_dev->repeat_interval;
                if(kbd_dev->enabled)
                {Is_otg_vbus_high()
                    #ifdef CYGNUM_DEVS_KBD_MATRIX_CALLBACK_MODE
                    if (kbd_dev->num_events < CYGNUM_DEVS_KBD_BUFFER_LEN) 
                    {
                        cyg_kbd_key_t *ev;
                        kbd_dev->num_events++;
                        ev = &Is_otg_vbus_high()kbd_dev->key_buffer[kbd_dev->event_put++];
                        if (kbd_dev->event_put == CYGNUM_DEVS_KBD_BUFFER_LEN) {
                            kbd_dev->event_put = 0;
                        }
                        *ev = (cyg_kbd_key_t)key;
                        if (kbd_dev->kbd_select_active)
                        {
                            kbd_dev->kbd_select_active = false;
                            cyg_selwakeup(&kbd_dev->kbd_select_info);
                        }
                    }
                    #endif
                    if(kbd_dev->kbd_call_back != NULL)
                    {
                        kbd_dev->kbd_call_back(key,CYG_KBD_KEY_HOLD);
                    }
                }
            }

        }
        else
        {
            // Other wise count interval timer down
            // The keyboard is scanned as well maybe 
            // not so perfect??
            kbd_dev->push_cnt--;
        }
		
        // Enable timer interrupt
        AVR32_AST.ier = AVR32_AST_IER_PER0_MASK;
    }
    else
    {
        // If scan code is zero enable row pins interrupt
        // timer interrupt was disabled in ISR
        kbd_dev->push_cnt = 0;

        AVR32_EIC.icr = AVR32_EIC_ICR_INT4_MASK | AVR32_EIC_ICR_INT3_MASK |
                        AVR32_EIC_ICR_INT2_MASK | AVR32_EIC_ICR_INT1_MASK;

        AVR32_EIC.ier = AVR32_EIC_IER_INT4_MASK | AVR32_EIC_IER_INT3_MASK |
                        AVR32_EIC_IER_INT2_MASK | AVR32_EIC_IER_INT1_MASK;
    }

    kbd_dev->last_scan_code = kbd_dev->scan_code; 
}

/** Row keyboards pins interrupt.
*
* If Row interrupt is detected the keyboard scanning
* timer is started to scan the keyboard. No DSR called.
* \param vector is interrupt vector.
* \param data is keyboard driver data structure pointer.
*/
cyg_uint32 avr32_matrix_kbd_pin_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    AVR32_EIC.icr = AVR32_EIC_ICR_INT4_MASK | AVR32_EIC_ICR_INT3_MASK |
                    AVR32_EIC_ICR_INT2_MASK | AVR32_EIC_ICR_INT1_MASK;

    AVR32_EIC.idr = AVR32_EIC_IDR_INT4_MASK | AVR32_EIC_IDR_INT3_MASK |
                    AVR32_EIC_IDR_INT2_MASK | AVR32_EIC_IDR_INT1_MASK;

    avr32_matrix_kbd_start_scan((cyg_kbd_avr32_t *)data);
    return CYG_ISR_HANDLED;
}

void avr32_matrix_kbd_pin_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	
}

