//==========================================================================
//
//      kbd_button.c
//
//      Keyboard driver for the Freescale Kinetis button keyboard
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
// Description:  Button keyboardd driver for Kinetis
//
//####DESCRIPTIONEND####
//
//==========================================================================


#include <pkgconf/devs_kbd_button_kinetis.h>

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

#include <cyg/hal/var_io_gpio.h>
#include <cyg/hal/var_io.h>
#include <cyg/hal/plf_io.h>
#include <cyg/fileio/fileio.h>  // For select() functionality
#include <cyg/io/kbd_button.h>
#include <cyg/io/devtab.h>


#define MAX_EVENTS CYGNUM_DEVS_KBD_BUTTON_EVENT_BUFFER_SIZE


#if CYGNUM_DEVS_KBD_BUTTON_INTERRUPTS_EDGE == RISING_EDGE
#define PORT_IRQC_EDGE  (0x09l)
#else
#define PORT_IRQC_EDGE  (0x0al)
#endif

#define PIT    ((cyghwr_hal_kinteis_pit_t *)CYGHWR_HAL_KINETIS_PIT_BASE)


/** Declaration and initialization of button keyborad data structure.
*
*/
static cyg_kbd_kinetis_t cyg_kbd_kinetis =
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
    .interrupt_number       = CYGNUM_HAL_INTERRUPT_PIT0,
    .interrupt_prio         = CYGNUM_IO_KBD_BUTTON_INT_PRIORITY,
    .kb_pins_isr[0].kbd_pin_interrupt		= NULL,
    .kb_pins_isr[0].kbd_pin_interrupt_handle	= NULL,
    .kb_pins_isr[0].kbd_pin_interrupt_number	= CYGNUM_DEVS_KBD_BUTTON_INTERRUPTS_GROUP0_VECTOR,
#ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
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
                  CYGDAT_DEVS_KBD_BUTTON_NAME,
                  NULL,                           // Base device name
                  &kbd_handlers,
                  kbd_init,
                  kbd_lookup,
                  &cyg_kbd_kinetis);                // Private data pointer


static cyg_uint32
kinetis_button_kbd_timer_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void       
kinetis_button_kbd_timer_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

static cyg_uint32
kinetis_button_kbd_pin_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void
kinetis_button_kbd_pin_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

/** Function to convert scan code to linux keyboard key mapping.
*
* \param scan_code is keyboard scan code.
* \return linux keyboard key code.
*/
static cyg_uint16 kbd_scan_code_to_key(cyg_uint32 scan_code)
{
#if CYGDAT_DEVS_KBD_BUTTON_DEBUG_OUTPUT > 0
    diag_printf("Kbd dev scan Code: 0x%X\n",scan_code);
#endif
    switch(scan_code)
    {
        case 0x00001:
        return K_ENTER;
        break;
        case 0x00004:
        return K_SAK;
        break;
        case 0x02:
        return K_UP;
        break;
        case 0x008:
        return K_DOWN;
        break;
    }

    return 0;
}

static Cyg_ErrNo
kbd_read(cyg_io_handle_t handle,
         void *buffer,
         cyg_uint32 *len)
{
#ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
    cyg_devtab_entry_t *hnd = (cyg_devtab_entry_t*)handle;
    cyg_kbd_kinetis_t * priv = (cyg_kbd_kinetis_t*)hnd->priv;;

    cyg_kbd_key_t *ev;
    int tot = *len;
    cyg_kbd_key_t *bp = (cyg_kbd_key_t*)buffer;

    cyg_scheduler_lock();  // Prevent interaction with DSR code
    while (tot >= sizeof(*ev)) {
        if (priv->num_events > 0) {
            ev = &priv->key_buffer[priv->event_get++];
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
#ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
    cyg_devtab_entry_t *hnd = (cyg_devtab_entry_t*)handle;
    cyg_kbd_kinetis_t * priv = (cyg_kbd_kinetis_t*)hnd->priv;;
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
    cyg_devtab_entry_t *hnd = (cyg_devtab_entry_t*)handle;
    cyg_kbd_kinetis_t * priv = (cyg_kbd_kinetis_t*)hnd->priv;;
    cyg_uint32 ret = ENOERR;
    cyg_scheduler_lock();  // Prevent interaction with DSR code
    switch(key)
    {
        case CYG_KBD_SET_CONFIG_INTERVAL:
        {
            if(*len == sizeof(cyg_uint32))
            {
                cyg_uint32 *repeat = (cyg_uint32*)buffer;
                if(*repeat > 2 && *repeat < 50)
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
                    *((kbd_call_back_t*)buffer);
            priv->kbd_call_back = kbd_callback;
        }
        break;
        case CYG_KBD_SET_CONFIG_CALLBACK_REMOVE:
            priv->kbd_call_back = NULL;
            break;
        case CYG_KBD_SET_CONFIG_DEFAULT_INTERVAL:
            priv->repeat_interval = 20;
            break;
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
    cyg_devtab_entry_t *hnd = (cyg_devtab_entry_t*)handle;
    cyg_kbd_kinetis_t * priv = (cyg_kbd_kinetis_t*)hnd->priv;;
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
                *((cyg_uint32*)buffer) = (cyg_uint32)priv->enabled;
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
    cyg_uint32 regval;
    cyg_kbd_kinetis_t *kbd_dev = &cyg_kbd_kinetis;
    // set first line to scan
    kbd_dev->scan_line          = 0;

    //Configure pin interupt
    regval = CYGHWR_HAL_KINETIS_PIN_CFG(C,CYGHWR_DEVS_KB0_PIN,1,
                          PORT_IRQC_EDGE,CYGHWR_HAL_KINETIS_PORT_PCR_ISF_M);
    CYGHWR_IO_FREESCALE_UART_PIN(regval);
    
    regval = CYGHWR_HAL_KINETIS_PIN_CFG(C,CYGHWR_DEVS_KB1_PIN,1,
                          PORT_IRQC_EDGE,CYGHWR_HAL_KINETIS_PORT_PCR_ISF_M);
    CYGHWR_IO_FREESCALE_UART_PIN(regval);
    
    regval = CYGHWR_HAL_KINETIS_PIN_CFG(C,CYGHWR_DEVS_KB2_PIN,1,
                          PORT_IRQC_EDGE,CYGHWR_HAL_KINETIS_PORT_PCR_ISF_M);
    CYGHWR_IO_FREESCALE_UART_PIN(regval);
    
    regval = CYGHWR_HAL_KINETIS_PIN_CFG(C,CYGHWR_DEVS_KB3_PIN,1,
                          PORT_IRQC_EDGE,CYGHWR_HAL_KINETIS_PORT_PCR_ISF_M);
    CYGHWR_IO_FREESCALE_UART_PIN(regval);
    
    // Configure GPIO pins used by keyboard
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(C,CYGHWR_DEVS_KB0_PIN);
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(C,CYGHWR_DEVS_KB1_PIN);
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(C,CYGHWR_DEVS_KB2_PIN);
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_IN(C,CYGHWR_DEVS_KB3_PIN);

    // Init keyboard timer interrupt 
    cyg_drv_interrupt_create(kbd_dev->interrupt_number,
                             kbd_dev->interrupt_prio,   // Priority
                             (cyg_addrword_t)kbd_dev,   // Data item passed to interrupt handler
                             kinetis_button_kbd_timer_ISR,
                             kinetis_button_kbd_timer_DSR,
                             &kbd_dev->kbd_interrupt_handle,
                             &kbd_dev->kbd_interrupt);
    cyg_drv_interrupt_attach(kbd_dev->kbd_interrupt_handle);

    // Init keyboard rows interrupt
    cyg_drv_interrupt_create(kbd_dev->kb_pins_isr[0].kbd_pin_interrupt_number,
                    kbd_dev->interrupt_prio,                      // Priority
                    (cyg_addrword_t)kbd_dev,   // Data item passed to interrupt handler
                    kinetis_button_kbd_pin_ISR,
                    kinetis_button_kbd_pin_DSR,
                    &kbd_dev->kb_pins_isr[0].kbd_pin_interrupt_handle,
                    &kbd_dev->kb_pins_isr[0].kbd_pin_interrupt);
    cyg_drv_interrupt_attach(kbd_dev->kb_pins_isr[0].kbd_pin_interrupt_handle);

        
#ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
    cyg_selinit(&kbd_dev->kbd_select_info);
#endif
    kbd_dev->is_open = true;
    return true;
}

static Cyg_ErrNo
kbd_lookup(struct cyg_devtab_entry **tab,
           struct cyg_devtab_entry *st,
           const char *name)
{
    cyg_kbd_kinetis_t * const kbd = (cyg_kbd_kinetis_t *) (*tab)->priv;
    
    if(!kbd->is_open)
    {
        kbd_init(*tab);
        kbd->is_open = true;
    }
    return ENOERR;
}

/** Start keyboard scan timer.
*
* The pir0 interrupt is used to driver columns/lines scanning.
* \param kdb_dev is pointer to keyboard data structure.
*/
static void avr32_button_kbd_start_scan(cyg_kbd_avr32_t *kbd_dev)
{
    kbd_dev->push_cnt   = 0;
    kbd_dev->scan_line  = 0;
    kbd_dev->glitch_cnt = 0;
    kbd_dev->scan_code  = 0;
    PIT.CHANNEL[0].TFLG = CYGHWR_HAL_KINETIS_PIT_TFLG_TIF;
    PIT.CHANNEL[0].LDVAL = CYGNUM_DEVS_KBD_BUTTON_PIT_CNT;
    PIT.CHANNEL[0].TCTRL = CYGHWR_HAL_KINETIS_PIT_TCTRL_TIE |
                           CYGHWR_HAL_KINETIS_PIT_TCTRL_TEN;
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
kinetis_button_kbd_timer_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_kbd_kinetis_t *kbd_dev = (cyg_kbd_kinetis_t*)data;
    cyg_uint32           ret = CYG_ISR_HANDLED;
    
	// is pir0 interrupt
    if(PIT.CHANNEL[0].TFLG)
    {
        // Clear pir0 interupt
        PIT.CHANNEL[0].TFLG = CYGHWR_HAL_KINETIS_PIT_TFLG_TIF;
         
        cyg_uint32 scan_code = 0;
        if(CYGHWR_HAL_KINETIS_GPIO_GET_PIN(C, CYGHWR_DEVS_KB0_PIN) == 
                CYGNUM_DEVS_KBD_BUTTON_ACTIVE_VALUE)
        {
            scan_code |= 0x0001;
        }
        if(CYGHWR_HAL_KINETIS_GPIO_GET_PIN(C, CYGHWR_DEVS_KB1_PIN) == 
                CYGNUM_DEVS_KBD_BUTTON_ACTIVE_VALUE)
        {
            scan_code |= 0x0002;
        }
        if(CYGHWR_HAL_KINETIS_GPIO_GET_PIN(C, CYGHWR_DEVS_KB2_PIN) == 
                CYGNUM_DEVS_KBD_BUTTON_ACTIVE_VALUE)
        {
            scan_code |= 0x0004;
        }
        if(CYGHWR_HAL_KINETIS_GPIO_GET_PIN(C, CYGHWR_DEVS_KB3_PIN) == 
                CYGNUM_DEVS_KBD_BUTTON_ACTIVE_VALUE)
        {
            scan_code |= 0x0008;
        }

        kbd_dev->glitch_cnt++;
        
        if(kbd_dev->scan_code == 0)
            kbd_dev->scan_code = scan_code;
        else
        {
            if((kbd_dev->scan_code != scan_code))
            {
                kbd_dev->scan_code = scan_code;
                kbd_dev->glitch_cnt = 0;
            }
        }
        
        if(kbd_dev->glitch_cnt == CYGNUM_DEVS_KBD_BUTTON_GLITCH_CNT)
        {
            // Disable timer interrupt
            //PIT.CHANNEL[0].TCTRL &= ~CYGHWR_HAL_KINETIS_PIT_TCTRL_TIE;
            cyg_drv_interrupt_mask(kbd_dev->interrupt_number);
            cyg_drv_interrupt_acknowledge(kbd_dev->interrupt_number);
            kbd_dev->glitch_cnt = 0;
            ret |= CYG_ISR_CALL_DSR;
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
kinetis_button_kbd_timer_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_kbd_kinetis_t *kbd_dev = (cyg_kbd_kinetis_t*)data;
    
    // If scan code is non zero do some stuff
    // This DSR is called every ca. 0.0390625s
    if(kbd_dev->scan_code != 0)
    {
        // If the repeat interval for pushed button is reached
        if(kbd_dev->push_cnt == 0)
        {
            // Convert scan code to linux keyboard code
            cyg_uint16 key = kbd_scan_code_to_key(kbd_dev->scan_code);
            // if latest key is different from current
            // use longer interval
            if(kbd_dev->scan_code != kbd_dev->last_scan_code)
            {
                kbd_dev->push_cnt = 25;
                if(kbd_dev->enabled)
                {
                    #ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
                    if (kbd_dev->num_events < CYGNUM_DEVS_KBD_BUTTON_EVENT_BUFFER_SIZE) 
                    {
                        cyg_kbd_key_t *ev;
                        kbd_dev->num_events++;
                        ev = &kbd_dev->key_buffer[kbd_dev->event_put++];
                        if (kbd_dev->event_put == CYGNUM_DEVS_KBD_BUTTON_EVENT_BUFFER_SIZE) {
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
                        kbd_dev->kbd_call_back(key,CYG_KBD_KEY_DOWNO);
                    }
                }
            }
            else
            {
                // otherwise use default repeat interval
                kbd_dev->push_cnt = kbd_dev->repeat_interval;
                if(kbd_dev->enabled)
                {
                    #ifndef CYGNUM_DEVS_KBD_BUTTON_CALLBACK_MODE
                    if (kbd_dev->num_events < CYGNUM_DEVS_KBD_BUTTON_EVENT_BUFFER_SIZE) 
                    {
                        cyg_kbd_key_t *ev;
                        kbd_dev->num_events++;
                        ev = &kbd_dev->key_buffer[kbd_dev->event_put++];
                        if (kbd_dev->event_put == CYGNUM_DEVS_KBD_BUTTON_EVENT_BUFFER_SIZE) {
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
        //PIT.CHANNEL[0].TCTRL |= CYGHWR_HAL_KINETIS_PIT_TCTRL_TIE;
        cyg_drv_interrupt_unmask(kbd_dev->interrupt_number);
    }
    else
    {
        // If scan code is zero enable row pins interrupt
        // timer interrupt was disabled in ISR
        kbd_dev->push_cnt = 0;
        
        // disable pit timer
        PIT.CHANNEL[0].TFLG = CYGHWR_HAL_KINETIS_PIT_TFLG_TIF;
        PIT.CHANNEL[0].TCTRL = 0;
    
        KBD_PORT->isfr = -1;
        cyg_drv_interrupt_unmask(kbd_dev->kb_pins_isr[0].kbd_pin_interrupt);
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
cyg_uint32 kinetis_button_kbd_pin_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_kbd_kinetis_t *kbd_dev = (cyg_kbd_kinetis_t*)data;
    cyg_drv_interrupt_mask(kbd_dev->kb_pins_isr[0].kbd_pin_interrupt);
    cyg_drv_interrupt_acknowledge(kbd_dev->kb_pins_isr[0].kbd_pin_interrupt);
    
    KBD_PORT->isfr = -1;
	
    avr32_button_kbd_start_scan((cyg_kbd_avr32_t *)data);
    return CYG_ISR_HANDLED;
}

void kinetis_button_kbd_pin_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	
}

