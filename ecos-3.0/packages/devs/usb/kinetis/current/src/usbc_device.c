//==========================================================================
//
//      usbs_uc3c.c
//
//      Driver for the AVR32UC3C USB device
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 2006, 2010 Free Software Foundation, Inc.
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
// Author(s):    Oliver Munz,
// Contributors: Andrew Lunn, bartv, ccoutand
// Date:         2006-02-22
//
// This code implements support for the on-chip USB port on the AT91
// family of processors. The code has been developed on the AT91SAM7S
// and may or may not work on other members of the AT91 family.
//
//####DESCRIPTIONEND####
//==========================================================================

#include <cyg/io/usb/usbc_device.h>
#include <pkgconf/system.h>
#include <pkgconf/devs_usb_kinetis.h>
#include <cyg/io/usb/usb.h>
#include <cyg/io/usb/usbs.h>
#include <cyg/io/usb/usbs_kinetis.h>

#include CYGBLD_HAL_PLATFORM_H
#include <cyg/hal/hal_io.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_io.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/cyg_trac.h>
#include <cyg/infra/diag.h>
#include <stdio.h>

#include <cyg/kernel/kapi.h>

#define WM_ON_USB_PLUG					0x02000000
#define WM_ON_USB_UNPLUG				0x04000000

#define BOOT_LOADER

#ifndef BOOT_LOADER
extern int queue_put(unsigned int msg, unsigned int lparam, unsigned int rparam);
#endif


#define USB_DEVICE_MAX_EP 16
#define UDD_NO_SLEEP_MGR
#define USB_DEVICE_EP_CTRL_SIZE 8


#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#	define UDC_DATA(x)              __attribute__((__aligned__(x)))
#	define UDC_BSS(x)               __attribute__((__aligned__(x)))

#define USB_DEVICE_EP_CTRL_SIZE    8

cyg_bool  ep0_start = false;



//Endpoints control banks
UDC_BSS(512)
static volatile usb_buffer_descritpor_t udd_bd_table[4*USB_DEVICE_MAX_EP];

//control endpoint buffer
UDC_BSS(4) cyg_uint8 udd_ctrl_buffer[USB_DEVICE_EP_CTRL_SIZE];
//endpoints chace buffers
UDC_BSS(4) cyg_uint8 udd_ep_out_cache_buffer[USB_DEVICE_MAX_EP][64];

static cyg_interrupt usbs_kinetis_intr_data;
static cyg_handle_t  usbs_kinetis_intr_handle;

static cyg_interrupt usbs_plug_intr_data;
static cyg_handle_t  usbs_plug_intr_handle;

static void usbs_kinetis_ep0_start(usbs_control_endpoint *);
static void usbs_kinetis_poll(usbs_control_endpoint *);

static void usbs_kinetis_endpoint_start(usbs_rx_endpoint * pep);
static void usbs_kinetis_endpoint_set_halted(usbs_rx_endpoint * pep,
                                          cyg_bool new_value);

void usbs_kinetis_endpoint_abort(usbs_rx_endpoint * pep);

// Endpoint 0, the control endpoint, structure.
usbs_control_endpoint usbs_kinetis_ep0 = {
  // The hardware does not distinguish  between detached, attached and powered.
  state:                  USBS_STATE_POWERED,
  enumeration_data:       (usbs_enumeration_data *) 0,
  start_fn:               usbs_kinetis_ep0_start,
  poll_fn:                usbs_kinetis_poll,
  interrupt_vector:       CYGNUM_HAL_INTERRUPT_USB0,
  control_buffer:         {0, 0, 0, 0, 0, 0, 0, 0},
  state_change_fn:        (void (*) (usbs_control_endpoint *,
                                     void *, usbs_state_change, int)) 0,
  state_change_data:      (void *) 0,
  standard_control_fn:    (usbs_control_return (*)
                          (usbs_control_endpoint *, void *)) 0,
  standard_control_data:  (void *) 0,
  class_control_fn:       (usbs_control_return (*)
                           (usbs_control_endpoint *, void *)) 0,
  class_control_data:     (void *) 0,
  vendor_control_fn:      (usbs_control_return (*)
                           (usbs_control_endpoint *, void *)) 0,
  vendor_control_data:    (void *) 0,
  reserved_control_fn:    (usbs_control_return (*)
                           (usbs_control_endpoint *, void *)) 0,
  reserved_control_data:  (void *) 0,
  buffer:                 (unsigned char *) 0,
  buffer_size:            0,
  fill_buffer_fn:         (void (*)(usbs_control_endpoint *)) 0,
  fill_data:              (void *) 0,
  fill_index:             0,
  complete_fn:            (usbs_control_return (*)(usbs_control_endpoint *,
                                                   int)) 0
};

// Endpoint 1 receive control structure
usbs_rx_endpoint usbs_kinetis_ep1 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};

// Endpoint 2 Receive control structure
usbs_rx_endpoint usbs_kinetis_ep2 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};

// Endpoint 3 Receive control structure
usbs_rx_endpoint usbs_kinetis_ep3 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};

#if (KINETIS_USB_ENDPOINTS > 4)
// Endpoint 4 Receive control structure
usbs_rx_endpoint usbs_kinetis_ep4 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};
#endif

#if (KINETIS_USB_ENDPOINTS > 5)
// Endpoint 5 Receive control structure
usbs_rx_endpoint usbs_kinetis_ep5 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};
#endif

#if (KINETISUSB_ENDPOINTS > 6)
// Endpoint 6 Receive control structure
usbs_rx_endpoint usbs_kinetis_ep6 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};
#endif

#if (KINETIS_USB_ENDPOINTS > 7)
// Endpoint 7 Receive control structure
usbs_rx_endpoint usbs_kinetis_ep7 = {
  start_rx_fn:    usbs_kinetis_endpoint_start,
  set_halted_fn:  usbs_kinetis_endpoint_set_halted,
  complete_fn:    (void (*)(void *, int)) 0,
  complete_data:  (void *) 0,
  buffer:         (unsigned char *) 0,
  buffer_size:    0,
  halted:         0,
};
#endif

// Array of end points. Used for translating end point pointer to an
// end point number
static const void *usbs_kinetis_endpoints[KINETIS_USB_ENDPOINTS] = {
  (void *) &usbs_kinetis_ep0,
  (void *) &usbs_kinetis_ep1,
  (void *) &usbs_kinetis_ep2,
  (void *) &usbs_kinetis_ep3
#if (KINETIS_USB_ENDPOINTS > 4)
  ,(void *) &usbs_kinetis_ep4, (void *) &usbs_kinetis_ep5
#if (KINETIS_USB_ENDPOINTS > 6)
  ,(void *) &usbs_kinetis_ep6, (void *) &usbs_kinetis_ep7
#endif
#endif
};

static usb_device_khci_state_struct_t kinetis_usb_device
{
    
    registerBase:               USB0,
    dmaAlignBuffer:             NULL,
    isDmaAlignBufferInusing:    0,
    isResetting:                0,
    controllerId:               0,
};

    


// Convert an endpoint pointer to an endpoint number, using the array
// of endpoint structures
static int
usbs_kinetis_pep_to_number(const usbs_rx_endpoint * pep)
{
    int epn;

    for(epn=0; epn < UC3C_USB_ENDPOINTS; epn++) {
        if (pep == usbs_kinetis_endpoints[epn])
            return epn;
    }
    CYG_FAIL("Unknown endpoint");
    return 0;
}

typedef enum ep0_low_level_status_t 
{
    UDD_EPCTRL_SETUP                  = 0, //!< Wait a SETUP packet
    UDD_EPCTRL_DATA_OUT               = 1, //!< Wait a OUT data packet
    UDD_EPCTRL_DATA_IN                = 2, //!< Wait a IN data packet
    UDD_EPCTRL_HANDSHAKE_WAIT_IN_ZLP  = 3, //!< Wait a IN ZLP packet
    UDD_EPCTRL_HANDSHAKE_WAIT_OUT_ZLP = 4, //!< Wait a OUT ZLP packet
    UDD_EPCTRL_STALL_REQ              = 5, //!< STALL enabled on IN & OUT packet
    UDD_EPCTRL_SET_ADDRESS            = 6
} ep0_low_level_status_t;


static ep0_low_level_status_t
usbs_kinetis_control_data_recv(ep0_low_level_status_t status);

void usbs_kinetis_endpoint_init (const usb_endpoint_descriptor *ep);

static ep0_low_level_status_t
usbs_kinetis_control_data_sent(ep0_low_level_status_t status);

static cyg_bool
usbs_kinetis_endpoint_isr_rx(cyg_uint8 epn);
static cyg_bool
usbs_kinetis_endpoint_isr_tx(cyg_uint8 epn);

static ep0_low_level_status_t udd_ctrl_send_zlp_in(void);

static void
usbs_kinetis_reset_device (void);

static cyg_uint8 usb_address = 0;


// Stop all transfers that are currently active.
static void
usbs_end_all_transfers (usbs_control_return returncode)
{
    cyg_uint32 epn;
    usbs_rx_endpoint *pep;

    for (epn = 1; epn < UC3C_USB_ENDPOINTS; epn++) 
    {
        // Stop transfer
        kinetis_usb_device.endpointState[index].stateUnion.stateBitField.transferring = 0U;
        // If the end point is transmitting, call the complete function
        // to terminate to transfer
        pep = (usbs_rx_endpoint *) usbs_kinetis_endpoints[epn];

        if (pep->complete_fn) 
        {
            (*pep->complete_fn) (pep->complete_data, returncode);
        }

    }
}

void usbs_ep_alloc(void)
{
    int epn;
    const usb_endpoint_descriptor *usb_endpoints;

    // Now walk the endpoints configuring them correctly. This only
    // works if there is one interface.
    usb_endpoints = usbs_kinetis_ep0.enumeration_data->endpoints;

    for (epn = 0;
    epn < usbs_kinetis_ep0.enumeration_data->total_number_endpoints;
    epn++) 
    {
        if ( epn < KINETIS_USB_ENDPOINTS ) 
        {
            usbs_kinetis_endpoint_init(&usb_endpoints[epn]);
        }
    }
}

// There has been a change in state. Update the end point.
static void
usbs_state_notify (usbs_control_endpoint * pcep)
{
    static int old_state = USBS_STATE_CHANGE_POWERED;
    int state = pcep->state & USBS_STATE_MASK;

    if (pcep->state != old_state) {
        usbs_end_all_transfers (-EPIPE);
        switch (state) {
            case USBS_STATE_DETACHED:
            case USBS_STATE_ATTACHED:
            case USBS_STATE_POWERED:
                // Nothing to do
                break;
            case USBS_STATE_DEFAULT:
                break;
            case USBS_STATE_ADDRESSED:
                usbs_ep_alloc();
                break;
            case USBS_STATE_CONFIGURED:
                break;
            default:
                CYG_FAIL("Unknown endpoint state");
        }

        if (pcep->state_change_fn) {
            (*pcep->state_change_fn) (pcep, pcep->state_change_data,
                                                    pcep->state, old_state);
        }

        old_state = pcep->state;
    }
}

static usbs_control_return usb_set_address(usbs_control_endpoint *cep,
                                                int val)
{
    // TODO: Divene mela by byt soucasti volani
    kinetis_usb_device.registerBase->ADDR = usb_address;
    cep->state = USBS_STATE_ADDRESSED;
    usbs_state_notify(cep);
    return USBS_CONTROL_RETURN_HANDLED;
}

usbs_control_return
usbs_parse_host_get_command (usbs_control_endpoint * pcep)
{
  usbs_control_return retcode;
  cyg_uint8 dev_req_type =
    (((usb_devreq *) pcep->control_buffer)->type) & USB_DEVREQ_TYPE_MASK;

  switch (dev_req_type) {
    case USB_DEVREQ_TYPE_STANDARD:
      if (!pcep->standard_control_fn) {
        return usbs_handle_standard_control (pcep);
      }

      retcode =
        (*pcep->standard_control_fn) (pcep, pcep->standard_control_data);

      if (retcode == USBS_CONTROL_RETURN_UNKNOWN) {
        return usbs_handle_standard_control (pcep);
      }
      return retcode;

    case USB_DEVREQ_TYPE_CLASS:
      if (!pcep->class_control_fn) {
        return USBS_CONTROL_RETURN_STALL;
      }
      return (*pcep->class_control_fn) (pcep, pcep->class_control_data);

    case USB_DEVREQ_TYPE_VENDOR:
      if (!pcep->class_control_fn) {
        return USBS_CONTROL_RETURN_STALL;
      }
      return (*pcep->class_control_fn) (pcep, pcep->vendor_control_data);

    case USB_DEVREQ_TYPE_RESERVED:
      if (!pcep->reserved_control_fn) {
        return USBS_CONTROL_RETURN_STALL;
      }
      return (*pcep->reserved_control_fn) (pcep, pcep->reserved_control_data);
    default:
      return USBS_CONTROL_RETURN_STALL;
  }
}

void
usbs_kinetis_endpoint_set_halted (usbs_rx_endpoint * pep, cyg_bool new_value)
{
  int epn = usbs_kinetis_pep_to_number(pep);

  cyg_drv_dsr_lock ();

  if (pep->halted != new_value) {
    /* There is something is to do */
    pep->halted = new_value;

    if ( new_value ) {

      /* Halt endpoint */
      udd_enable_stall_handshake(epn);

    } else if(Is_udd_endpoint_stall_requested(epn)){
        // Remove stall request
        udd_disable_stall_handshake(epn);

        if (Is_udd_stall(epn)) {
           udd_ack_stall(epn);
           // The Stall has occured, then reset data toggle
           udd_reset_data_toggle(epn);
        }

         // Ready to use
         if (pep->complete_fn) {
           (*pep->complete_fn) (pep->complete_data, ENOERR);
          }
    }
  }
  cyg_drv_dsr_unlock ();
}

void
usbs_kinetis_endpoint_init (const usb_endpoint_descriptor *ep)
{
  int epn = ep->endpoint & (~USB_EP_DIR_IN);

  CYG_ASSERT (UC3C_USB_ENDPOINTS > epn, "Invalid end point");

   if (Is_udd_endpoint_enabled(epn)) {
      CYG_ASSERT(true,"Endpoint is enabled");
      return;
   }

   
   udd_configure_endpoint(epn, ep->attributes,
           ((ep->endpoint & USB_EP_DIR_IN) ? 1 : 0), 
		   ((int)ep->max_packet_hi << 8) | ep->max_packet_lo,
            AVR32_USBC_UECFG0_EPBK_SINGLE);

   udd_enable_busy_bank0(epn);
   udd_enable_endpoint(epn);
}

void usbs_kinetis_endpoint_abort(usbs_rx_endpoint * pep)
{
    cyg_uint32 irq;
    int epn = usbs_kinetis_pep_to_number(pep);

    CYG_ASSERT (UC3C_USB_ENDPOINTS > epn, "Invalid end point");

    HAL_DISABLE_INTERRUPTS(irq);
    udd_disable_endpoint_interrupt(epn);
    HAL_RESTORE_INTERRUPTS(irq);
    
    // Stop transfer
    udd_enable_busy_bank0(epn);
    //dsable endpoint
    udd_disable_endpoint(epn);
}

void
usbs_kinetis_reset_device (void)
{
    usbs_end_all_transfers (-EPIPE);

    // Reset USB address to 0
    udd_configure_address(0);
    udd_enable_address();

    udd_disable_endpoint(0);

    // Alloc and configure control endpoint
    udd_configure_endpoint(0,
                    USB_EP_TYPE_CONTROL,
                    0,
                    USB_DEVICE_EP_CTRL_SIZE, AVR32_USBC_UECFG0_EPBK_SINGLE);

    // Use internal buffer for endpoint control
    udd_udesc_set_buf0_addr(0, udd_ctrl_buffer);

    // don't use multipacket on endpoint control
    udd_udesc_rst_buf0_size(0);
    udd_enable_endpoint(0);
    udd_disable_busy_bank0(0);

    udd_enable_setup_received_interrupt(0);
    udd_enable_out_received_interrupt(0);
    udd_enable_endpoint_interrupt(0);

    udd_disable_in_send_interrupt(0);
    // In case of OUT ZLP event is no processed before Setup event occurs
    udd_ack_out_received(0);
}

void
usbs_kinetis_handle_reset (void)
{
    usbs_kinetis_reset_device ();
}

void
usbs_kinetis_ep0_start (usbs_control_endpoint * endpoint)
{
    int irq;
    
    HAL_DISABLE_INTERRUPTS(irq);
    // At startup the USB bus state is unknown,
    // therefore the state is considered IDLE to not miss any USB event
    //udd_sleep_mode(true);
    usbs_kinetis_handle_reset ();
    otg_unfreeze_clock();
    while( !Is_otg_clock_usable() );

    // Authorize attach if Vbus is present
    udd_attach_device();

    // Enable USB line events
    udd_enable_reset_interrupt();
    udd_enable_suspend_interrupt();
    udd_enable_wake_up_interrupt();
    //udd_enable_sof_interrupt();

    // Reset following interupts flag
    udd_ack_reset();
    udd_ack_sof();

    // The first suspend interrupt must be forced
    udd_raise_suspend();
    udd_ack_wake_up();
    otg_freeze_clock();
    ep0_start = true;
    HAL_RESTORE_INTERRUPTS(irq);
}

void
usbs_kinetis_endpoint_start (usbs_rx_endpoint * pep)
{
    int epn = usbs_kinetis_pep_to_number(pep);
    int epsize;
    ep_job *ptr_job = &ep_job_str[epn];

    CYG_ASSERT (pep->complete_fn, "No complete_fn()");

    cyg_drv_dsr_lock ();
    if (usbs_kinetis_ep0.state != USBS_STATE_CONFIGURED) {
        /* If not configured it means there is nothing to do */
        cyg_drv_dsr_unlock ();

        if (pep->complete_fn) {
            (*pep->complete_fn) (pep->complete_data, -EPIPE);
        }
        return;
    }

    // Is this endpoint currently stalled? If so then a size of 0 can
    // be used to block until the stall condition is clear, anything
    // else should result in an immediate callback.
    if (pep->halted) {
        /* Halted means nothing to do */
        cyg_drv_dsr_unlock ();

        if (pep->complete_fn && pep->buffer_size != 0) {
            (*pep->complete_fn) (pep->complete_data, -EAGAIN);
        }
        return;
    }

    if (/*udd_nb_busy_bank(epn) != 0 ||*/ !Is_udd_endpoint_enabled(epn)) {
        cyg_drv_dsr_unlock ();

        if (pep->complete_fn) {
            (*pep->complete_fn) (pep->complete_data, -EIO);
        }

        return;
    }
  
    ptr_job->nb_trans = 0;
    epsize = udd_get_endpoint_size(epn);
    if(ptr_job->endpoint->buffer_size % epsize == 0)
    {
        ptr_job->shortpacket = true;
    }
    else
    {
        ptr_job->shortpacket = false;
    }
    
    ptr_job->shortpacket = false;
    ptr_job->use_out_cache_buffer = false;

    // Initialize value to simulate a empty transfer
    udd_udesc_rst_buf0_ctn(epn);
    udd_udesc_rst_buf0_size(epn);
    
    
    // Request next transfer
    if(Is_udd_endpoint_in(epn))
    {
        usbs_kinetis_endpoint_isr_tx(epn);
    }
    else
    {
        usbs_kinetis_endpoint_isr_rx(epn);
    }

    cyg_drv_dsr_unlock ();
}

// Perform transmit handling on an endpoint
cyg_bool
usbs_kinetis_endpoint_isr_tx(cyg_uint8 epn)
{
    cyg_uint32 irq;
    cyg_uint16 ep_size, nb_trans;
    cyg_uint16 next_trans;
    ep_job *ptr_job = &ep_job_str[epn];

    ep_size = udd_get_endpoint_size(epn);

    HAL_DISABLE_INTERRUPTS(irq);
    udd_disable_endpoint_interrupt(epn);
    HAL_RESTORE_INTERRUPTS(irq);
    // Transfer complete on IN
    nb_trans = udd_udesc_get_buf0_size(epn);
    // Lock emission of new IN packet
    udd_enable_busy_bank0(epn);

    // Ack interrupt
    udd_ack_in_send(epn);
    
 
    if (0 == nb_trans) 
    {
        if (0 == udd_nb_busy_bank(epn)) 
        {
            // All byte are transfered than take nb byte requested
            nb_trans = udd_udesc_get_buf0_ctn(epn);
        }
    }
    
    // Update number of data transfered
    ptr_job->nb_trans += nb_trans;

    // Need to send other data
    if ((ptr_job->nb_trans != ptr_job->endpoint->buffer_size)
            || ptr_job->shortpacket) 
    {
        next_trans = ptr_job->endpoint->buffer_size - ptr_job->nb_trans;
        if (UDD_ENDPOINT_MAX_TRANS < next_trans)
        {
            // The USB hardware support a maximum
            // transfer size of UDD_ENDPOINT_MAX_TRANS Bytes
            next_trans = UDD_ENDPOINT_MAX_TRANS -
                    (UDD_ENDPOINT_MAX_TRANS % ep_size);
            udd_udesc_set_buf0_autozlp(epn, false);
        } 
        else
        {
            // Need ZLP, if requested and last packet is not a short packet
            udd_udesc_set_buf0_autozlp(epn, ptr_job->shortpacket);
            ptr_job->shortpacket = false; // No need to request another ZLP
        }

        udd_udesc_set_buf0_ctn(epn, next_trans);
        udd_udesc_rst_buf0_size(epn);

        // Link the user buffer directly on USB hardware DMA
        udd_udesc_set_buf0_addr(epn, &ptr_job->endpoint->buffer[ptr_job->nb_trans]);

        // Start transfer
        udd_ack_fifocon(epn);
        udd_disable_busy_bank0(epn);
        // Enable interrupt
	HAL_DISABLE_INTERRUPTS(irq);
        udd_enable_in_send_interrupt(epn);
        udd_enable_endpoint_interrupt(epn);
        HAL_RESTORE_INTERRUPTS(irq);
        
        return false;
    }
    return true;
}

cyg_bool
usbs_kinetis_endpoint_isr_rx(cyg_uint8 epn)
{
    cyg_uint32 irq;
    cyg_uint16 ep_size, nb_trans;
    cyg_uint16 next_trans;
    ep_job *ptr_job = &ep_job_str[epn];

    ep_size = udd_get_endpoint_size(epn);

    HAL_DISABLE_INTERRUPTS(irq);
    udd_disable_endpoint_interrupt(epn);
    HAL_RESTORE_INTERRUPTS(irq);
    // Transfer complete on OUT
    nb_trans = udd_udesc_get_buf0_ctn(epn);

    // Lock reception of new OUT packet
    udd_enable_busy_bank0(epn);

    // Ack interrupt
    udd_ack_out_received(epn);
    udd_ack_fifocon(epn);

    // Can be necessary to copy data receiv from cache buffer to user buffer
    if (ptr_job->use_out_cache_buffer) 
    {
        memcpy(&ptr_job->endpoint->buffer[ptr_job->nb_trans],
                udd_ep_out_cache_buffer[epn],
                ptr_job->endpoint->buffer_size % ep_size);
        //diag_printf("coppy casche\n");
    }

    // Update number of data transfered
    ptr_job->nb_trans += nb_trans;
    if (ptr_job->nb_trans > ptr_job->endpoint->buffer_size) {
        ptr_job->nb_trans = ptr_job->endpoint->buffer_size;
    }

    // If all previous data requested are received and user buffer not full
    // then need to receiv other data
    if ((nb_trans == udd_udesc_get_buf0_size(epn))
        && (ptr_job->nb_trans != ptr_job->endpoint->buffer_size)) 
    {
	//diag_printf("Configure transfer\n");
        next_trans = ptr_job->endpoint->buffer_size - ptr_job->nb_trans;
        if (UDD_ENDPOINT_MAX_TRANS < next_trans)
        {
            // The USB hardware support a maximum transfer size
            // of UDD_ENDPOINT_MAX_TRANS Bytes
            next_trans = UDD_ENDPOINT_MAX_TRANS
                    - (UDD_ENDPOINT_MAX_TRANS % ep_size);
        }
        else 
        {
            next_trans -= next_trans % ep_size;
        }

        udd_udesc_rst_buf0_ctn(epn);
        if (next_trans < ep_size) 
        {
            // Use the cache buffer for Bulk or Interrupt size endpoint
            ptr_job->use_out_cache_buffer = true;
            udd_udesc_set_buf0_addr(epn,
                    udd_ep_out_cache_buffer[epn       ]);
            udd_udesc_set_buf0_size(epn, ep_size);
        } 
        else
        {
            // Link the user buffer directly on USB hardware DMA
            udd_udesc_set_buf0_addr(epn, &ptr_job->endpoint->buffer[ptr_job->nb_trans]);
            udd_udesc_set_buf0_size(epn, next_trans);
        }
        // Start transfer
        udd_disable_busy_bank0(epn);
        // Enable interrupt
        HAL_DISABLE_INTERRUPTS(irq);
        udd_enable_out_received_interrupt(epn);
        udd_enable_endpoint_interrupt(epn);
        HAL_RESTORE_INTERRUPTS(irq);
        return false;
    }

    return true;//call dsr
}

// Handle a DSR for an endpoint
 void
usbs_kinetis_endpoint_dsr (cyg_uint8 epn)
{
    ep_job *ptr_job = &ep_job_str[epn];
    usbs_rx_endpoint *pep = (usbs_rx_endpoint *) usbs_kinetis_endpoints[epn];

    CYG_ASSERT (UC3C_USB_ENDPOINTS > epn && epn, "Invalid end point");
    CYG_ASSERT (pep->complete_fn, "No complete_fn()");
    
    if (pep->complete_fn)
    {
        /* Do not check on pep->buffer_size != 0, user should
        * be allowed to send empty packet */
        if (!pep->halted) 
        {
            (*pep->complete_fn) (pep->complete_data, ptr_job->nb_trans);
        } 
        else 
        {
            (*pep->complete_fn) (pep->complete_data, -EAGAIN);
        }
    }
    ptr_job->call_dsr = false;

}

// ISR for an endpoint. Handle receive and transmit interrupts.
cyg_bool
usbs_kinetis_endpoint_isr (cyg_uint8 epn)
{
    CYG_ASSERT (UC3C_USB_ENDPOINTS > epn && epn, "Invalid end point");

    // For each endpoint different of control endpoint (0)

    if(Is_udd_endpoint_in(epn))
    {
        if(usbs_kinetis_endpoint_isr_tx(epn))
        {
            return true;
        }
    }
    else
    {
        if(usbs_kinetis_endpoint_isr_rx(epn))
        {
            return true;
        }
    }
    return false;
}

// Handle an error condition on the control endpoint
ep0_low_level_status_t
usbs_kinetis_control_error(ep0_low_level_status_t status)
{

    usbs_kinetis_ep0.buffer_size = 0;
    usbs_kinetis_ep0.fill_buffer_fn = 0;
    usbs_kinetis_ep0.complete_fn = 0;


    if (status == UDD_EPCTRL_SETUP ) {
        if (usbs_kinetis_ep0.complete_fn) {
        (*usbs_kinetis_ep0.complete_fn) (&usbs_kinetis_ep0,
                                    USBS_CONTROL_RETURN_STALL);
        }
    }

    status = UDD_EPCTRL_SETUP ;

    if (Is_udd_nak_out(0)) {
		// Overflow on OUT packet
		udd_ack_nak_out(0);
		status = udd_ctrl_overflow(status);

	}
	if (Is_udd_nak_in(0)) {
		// Underflow on IN packet
		udd_ack_nak_in(0);
		status = udd_ctrl_underflow(status);
	}

  return status;
}

// Handle a get status setup message on the control end point
 ep0_low_level_status_t
usbs_kinetis_control_setup_get_status(void)
{
  ep0_low_level_status_t status;
  usb_devreq *req = (usb_devreq *)usbs_kinetis_ep0.control_buffer;
  cyg_uint8 recipient = req->type & USB_DEVREQ_RECIPIENT_MASK;
  cyg_uint16 word = 0;

  status = UDD_EPCTRL_DATA_IN;

  switch (recipient) {
    case USB_DEVREQ_RECIPIENT_DEVICE:
    case USB_DEVREQ_RECIPIENT_INTERFACE:
      // Nothing to do
      break;
    case USB_DEVREQ_RECIPIENT_ENDPOINT:
      if ((usbs_kinetis_ep0.state == USBS_STATE_CONFIGURED) &&
          (req->index_lo > 0) &&
          (req->index_lo < UC3C_USB_ENDPOINTS)) {

        if(Is_udd_stall(req->index_lo))
            word = 1;
      } else {
        status = UDD_EPCTRL_STALL_REQ;
      }
      break;
    default:
      status = UDD_EPCTRL_STALL_REQ;
  }

  *((cyg_uint16*)usbs_kinetis_ep0.control_buffer) = word;
  usbs_kinetis_ep0.buffer_size = sizeof (word);
  return status;
}


// Handle a get status set feature message on the control endpoint
ep0_low_level_status_t
usbs_kinetis_control_setup_set_feature(void)
{
  ep0_low_level_status_t status;
  usb_devreq *req = (usb_devreq *)usbs_kinetis_ep0.control_buffer;
  cyg_uint8 recipient = req->type & USB_DEVREQ_RECIPIENT_MASK;
  usbs_rx_endpoint * pep;
  cyg_uint8 epn = req->index_lo & 0x0F;

  //usbs_kinetis_control_setup_send_ack();
  status = UDD_EPCTRL_SETUP;

  switch(recipient) {
    case USB_DEVREQ_RECIPIENT_DEVICE:
      status = UDD_EPCTRL_STALL_REQ  ;
      break;
    case USB_DEVREQ_RECIPIENT_INTERFACE:
      // Nothing to do
      break;
    case USB_DEVREQ_RECIPIENT_ENDPOINT:
      if ((usbs_kinetis_ep0.state == USBS_STATE_CONFIGURED) &&
            (epn > 0) &&
            (epn < UC3C_USB_ENDPOINTS)) {
      /*  cyg_uint32 CSR;

        HAL_READ_UINT32(pCSRn(epn), CSR);*/
        pep = (usbs_rx_endpoint *) usbs_kinetis_endpoints[epn];
        if ( !Is_udd_stall(epn) ) {
          usbs_kinetis_endpoint_set_halted ( pep , true );
        }
        else
          status = UDD_EPCTRL_STALL_REQ ;

      }
      else {
        status = UDD_EPCTRL_STALL_REQ  ;
      }
      break;
    default:
      status = UDD_EPCTRL_STALL_REQ  ;
  }
  return status;
}

// Handle a get status clear feature message on the control endpoint
 ep0_low_level_status_t
usbs_kinetis_control_setup_clear_feature(void)
{
  ep0_low_level_status_t status;
  usb_devreq *req = (usb_devreq *)usbs_kinetis_ep0.control_buffer;
  cyg_uint8 recipient = req->type & USB_DEVREQ_RECIPIENT_MASK;
  usbs_rx_endpoint * pep;
  cyg_uint8 epn = req->index_lo & 0x0F;

  udd_ctrl_send_zlp_in();
  status = UDD_EPCTRL_SETUP;

  switch (recipient) {
    case USB_DEVREQ_RECIPIENT_DEVICE:
      status = UDD_EPCTRL_STALL_REQ;
      break;
    case USB_DEVREQ_RECIPIENT_INTERFACE:
      // Nothing to do
      break;
    case USB_DEVREQ_RECIPIENT_ENDPOINT:
      if ((usbs_kinetis_ep0.state == USBS_STATE_CONFIGURED) &&
          (epn > 0) &&
          (epn < UC3C_USB_ENDPOINTS)) {
        //cyg_uint32 CSR;

        //HAL_READ_UINT32(pCSRn(epn), CSR);
        pep = (usbs_rx_endpoint *) usbs_kinetis_endpoints[epn];
        if ( Is_udd_stall(epn) && pep->halted ) {
          usbs_kinetis_endpoint_set_halted ( pep , false );
        }
        else
          status = UDD_EPCTRL_STALL_REQ;

      }
      else {
        status = UDD_EPCTRL_STALL_REQ;
      }
      break;
    default:
      status = UDD_EPCTRL_STALL_REQ;
  }
  return status;
}

// Handle a setup message from the host
 ep0_low_level_status_t
usbs_kinetis_control_setup(ep0_low_level_status_t status)
{
  usb_devreq *req = (usb_devreq *) usbs_kinetis_ep0.control_buffer;
  cyg_uint8   protocol;
  cyg_uint16 length;
  cyg_bool dev_to_host;
  cyg_uint32 irq;
  usbs_control_return usbcode;
  cyg_bool handled = false;

  CYG_TRACE0( true, "Control Setup\n" );
  

  if (UDD_EPCTRL_SETUP != status) {
	  // May be a hidden DATA or ZLP phase
	  // or protocol abort
	  if (usbs_kinetis_ep0.complete_fn) {
		  (*usbs_kinetis_ep0.complete_fn) (&usbs_kinetis_ep0,
		  USBS_CONTROL_RETURN_HANDLED);
	  }

	  // Reinitializes control endpoint management
	  // In case of abort of IN Data Phase:
	  // No need to abort IN transfer (rise TXINI),
	  // because it is automatically done by hardware when a Setup packet is received.
	  // But the interrupt must be disabled to don't generate interrupt TXINI
	  // after SETUP reception.
	  udd_disable_in_send_interrupt(0);
	  // In case of OUT ZLP event is no processed before Setup event occurs
	  udd_ack_out_received(0);
	  CYG_TRACE0( true, "Control Setup state no setup\n" );
	  status = UDD_EPCTRL_SETUP;
  }
  
  usbs_kinetis_ep0.buffer_size = 0;
  usbs_kinetis_ep0.fill_buffer_fn = 0;
  usbs_kinetis_ep0.complete_fn = 0;
  	// Fill setup request structure
  if (8 != udd_udesc_get_buf0_ctn(0)) {
		status = UDD_EPCTRL_STALL_REQ;
		udd_enable_stall_handshake(0);
		udd_ack_setup_received(0);
		return status; // Error data number doesn't correspond to SETUP packet
  }
  memcpy((cyg_uint8 *)req, udd_ctrl_buffer, 8);

  length = (req->length_hi << 8) | req->length_lo;;
  dev_to_host = req->type & USB_DEVREQ_DIRECTION_IN;

  //CLEAR_BITS (pCSR0, AT91_UDP_CSR_DTGLE);

  status = UDD_EPCTRL_SETUP;

  protocol = req->type & (USB_DEVREQ_TYPE_MASK);

  if (protocol == USB_DEVREQ_TYPE_STANDARD) {
    handled = true;
    switch (req->request) {
      case USB_DEVREQ_GET_STATUS:
        status = usbs_kinetis_control_setup_get_status();
        break;
      case USB_DEVREQ_SET_ADDRESS:
		CYG_TRACE0( true, "Control Setup Set Addr\n" );
		usb_address = req->value_lo;
		usbs_kinetis_ep0.complete_fn = &usb_set_address;
        break;
      case USB_DEVREQ_SET_FEATURE:
        status = usbs_kinetis_control_setup_set_feature();
        break;
      case USB_DEVREQ_CLEAR_FEATURE:
        status = usbs_kinetis_control_setup_clear_feature();
        break;
      default:
        handled = false;
    }
  }
    if ((protocol != USB_DEVREQ_TYPE_STANDARD) || !handled) 
    {
      // Ask the layer above to process the message
      usbcode = usbs_parse_host_get_command (&usbs_kinetis_ep0);
      usbs_kinetis_ep0.buffer_size = MIN (usbs_kinetis_ep0.buffer_size, length);
          if (usbcode == USBS_CONTROL_RETURN_HANDLED)
          {
              handled = true;
              if(usbs_kinetis_ep0.fill_buffer_fn)
              {
                  cyg_uint32 pos = 0;
                  memcpy(buf,usbs_kinetis_ep0.buffer,usbs_kinetis_ep0.buffer_size);
                  pos += usbs_kinetis_ep0.buffer_size;
                  while(usbs_kinetis_ep0.fill_buffer_fn)
                  {
                      if((usbs_kinetis_ep0.buffer_size + pos) < 100)
                      {
                          (*usbs_kinetis_ep0.fill_buffer_fn) (&usbs_kinetis_ep0);
                          memcpy(buf + pos,usbs_kinetis_ep0.buffer,usbs_kinetis_ep0.buffer_size);
                          pos += usbs_kinetis_ep0.buffer_size;
                      }
                      else
                      {
                          CYG_ASSERT(false,"USB control buffer overflow\n");
                      }					
                  }		

                  usbs_kinetis_ep0.buffer = buf;
                  usbs_kinetis_ep0.buffer_size = pos;
              }			

          }
          else
              handled = false;
    }
      
    udd_ack_setup_received(0);

    if (handled)
    { /* OK */
        if (dev_to_host)
        {
            status = UDD_EPCTRL_DATA_IN;
        } 
        else
        {
            status = UDD_EPCTRL_DATA_OUT;
        }

        if(dev_to_host)
        {
            udd_ctrl_prev_payload_nb_trans = 0;
            udd_ctrl_payload_nb_trans = 0;
            status = usbs_kinetis_control_data_sent(status);
        }		  
        else
        {
            if(length == 0)
            {
                status = udd_ctrl_send_zlp_in();
                return status;
            }			  

            // OUT data phase requested
            udd_ctrl_prev_payload_nb_trans = 0;
            udd_ctrl_payload_nb_trans = 0;
            status = UDD_EPCTRL_DATA_OUT;

            // To detect a protocol error, enable nak interrupt on data IN phase
            udd_ack_nak_in(0);

            HAL_DISABLE_INTERRUPTS(irq);
            udd_enable_nak_in_interrupt(0);
            HAL_RESTORE_INTERRUPTS(irq);
        }		  
    }
    else
    {
        status = UDD_EPCTRL_STALL_REQ;
    }
 
    return status;
}

ep0_low_level_status_t
usbs_kinetis_control_data_recv(ep0_low_level_status_t status)
{
    cyg_uint32 irq;
    cyg_uint16 nb_data;
    usbs_control_return usbcode;
    //usb_devreq *req = (usb_devreq *) usbs_kinetis_ep0.control_buffer;

    CYG_TRACE0( true, "Control Data recv\n" );
    if (UDD_EPCTRL_DATA_OUT != status) 
    {
        if ((UDD_EPCTRL_DATA_IN == status) ||
                        (UDD_EPCTRL_HANDSHAKE_WAIT_OUT_ZLP == status)) 
        {
            // End of SETUP request:
            // - Data IN Phase aborted,
            // - or last Data IN Phase hidden by ZLP OUT sending quiclky,
            // - or ZLP OUT received normaly.
            if (usbs_kinetis_ep0.complete_fn) 
            {
                    (*usbs_kinetis_ep0.complete_fn) (&usbs_kinetis_ep0,
                    USBS_CONTROL_RETURN_HANDLED);
            }
        }
        else 
        {
            // Protocol error during SETUP request
            CYG_TRACE0( true, "Control Data recv pro err\n" );
            status = UDD_EPCTRL_STALL_REQ;
            udd_enable_stall_handshake(0);

        }
        status = UDD_EPCTRL_SETUP;
        // Reinitializes control endpoint management
        udd_disable_in_send_interrupt(0);
        // In case of OUT ZLP event is no processed before Setup event occurs
        udd_ack_out_received(0);
        return status;
    }

    // Read data received during OUT phase
    nb_data = udd_udesc_get_buf0_ctn(0);

    memcpy((cyg_uint8 *) (usbs_kinetis_ep0.buffer + usbs_kinetis_ep0.buffer_size),
                    udd_ctrl_buffer, nb_data);
    usbs_kinetis_ep0.buffer_size  += nb_data;

    if(nb_data < USB_DEVICE_EP_CTRL_SIZE)
    {
        usbcode = USBS_CONTROL_RETURN_STALL;

        if (usbs_kinetis_ep0.complete_fn)
        {
            usbcode = (*usbs_kinetis_ep0.complete_fn) (&usbs_kinetis_ep0, 0);
        }

        if (usbcode == USBS_CONTROL_RETURN_HANDLED)
        {
            status = UDD_EPCTRL_SETUP;
        }
        else 
        {
            status = UDD_EPCTRL_STALL_REQ;
        }
        CYG_TRACE0( true, "Control Data recv cmpl\n" );
        // Send IN ZLP to ACK setup request
        udd_ack_out_received(0);
        udd_ctrl_send_zlp_in();
    }

    // Free buffer of control endpoint to authorize next reception
    udd_ack_out_received(0);

    // To detect a protocol error, enable nak interrupt on data IN phase
    udd_ack_nak_in(0);
    HAL_DISABLE_INTERRUPTS(irq);
    udd_enable_nak_in_interrupt(0);
    HAL_RESTORE_INTERRUPTS(irq);

    return status;
}

 ep0_low_level_status_t
usbs_kinetis_control_data_sent(ep0_low_level_status_t status)
{
    //static cyg_uint32 mb_transferred = 0;
    //cyg_uint32 bytes_to_write = 0;
    //cyg_uint8 nb_write = 0;
    //usb_devreq *req = (usb_devreq *)usbs_kinetis_ep0.control_buffer;

    static cyg_bool b_shortpacket = false;
    cyg_uint16 nb_remain;
    cyg_uint32 irq;

    HAL_DISABLE_INTERRUPTS(irq);
    udd_disable_in_send_interrupt(0);
    HAL_RESTORE_INTERRUPTS(irq);

    CYG_TRACE0( true, "Control Data send\n" );
    if (UDD_EPCTRL_HANDSHAKE_WAIT_IN_ZLP == status) 
    {
        // ZLP on IN is sent, then valid end of setup request
        //udd_ctrl_endofrequest();
        // Reinitializes control endpoint management
         if (usbs_kinetis_ep0.complete_fn) 
         {
            (*usbs_kinetis_ep0.complete_fn) (&usbs_kinetis_ep0,
            USBS_CONTROL_RETURN_HANDLED);
            //status = UDD_EPCTRL_DATA_IN;
        }
        //else
        {
            CYG_TRACE0( true, "Control Data send cmpl\n" );
            udd_disable_in_send_interrupt(0);
            // In case of OUT ZLP event is no processed before Setup event occurs
            udd_ack_out_received(0);
            return UDD_EPCTRL_SETUP;
        }			
    }
    Assert(status == UDD_EPCTRL_DATA_IN);
	/////
    nb_remain = usbs_kinetis_ep0.buffer_size - udd_ctrl_payload_nb_trans;
    if (0 == nb_remain) 
    {
        // All content of current buffer payload are sent
        // Update number of total data sending by previous playlaod buffer
        udd_ctrl_prev_payload_nb_trans += udd_ctrl_payload_nb_trans;
		
		
        if ((usbs_kinetis_ep0.buffer_size == udd_ctrl_prev_payload_nb_trans)
                        || b_shortpacket) 
        {
            // All data requested are transfered or a short packet has been sent
            // then it is the end of data phase.
            udd_ctrl_send_zlp_out();
            return status;
        }
    }
    // Continue transfer and send next data
    if (nb_remain >= USB_DEVICE_EP_CTRL_SIZE)
    {
        nb_remain = USB_DEVICE_EP_CTRL_SIZE;
        b_shortpacket = false;
    } 
    else 
    {
        b_shortpacket = true;
    }
    //** Critical section
    // Only in case of DATA IN phase abort without USB Reset signal after.
    // The IN data don't must be written in endpoint 0 DPRAM during
    // a next setup reception in same endpoint 0 DPRAM.
    // Thereby, an OUT ZLP reception must check before IN data write
    // and if no OUT ZLP is recevied the data must be written quickly (800us)
    // before an eventually ZLP OUT and SETUP reception
    HAL_DISABLE_INTERRUPTS(irq);
    if (Is_udd_out_received(0)) {
            // IN DATA phase aborted by OUT ZLP
            HAL_RESTORE_INTERRUPTS(irq);
            status = UDD_EPCTRL_HANDSHAKE_WAIT_OUT_ZLP;
            return status; // Exit of IN DATA phase
    }
    // Write quickly the IN data
    // Continue transfer and send next data
    memcpy(udd_ctrl_buffer,
            usbs_kinetis_ep0.buffer + udd_ctrl_payload_nb_trans,
            nb_remain);

    udd_ctrl_payload_nb_trans += nb_remain;
    udd_udesc_set_buf0_ctn(0, nb_remain);

    // Validate and send the data available in the control endpoint buffer
    udd_ack_in_send(0);
    udd_enable_in_send_interrupt(0);

    // In case of abort of DATA IN phase, no need to enable nak OUT interrupt
    // because OUT endpoint is already free and ZLP OUT accepted.
    HAL_RESTORE_INTERRUPTS(irq);
    return status;
}

 void
usbs_kinetis_control_dsr (void)
{
    static ep0_low_level_status_t status = UDD_EPCTRL_SETUP;
    // By default disable overflow and underflow interrupt
    udd_disable_nak_in_interrupt(0);
    udd_disable_nak_out_interrupt(0);

    // Check for a setup message and handle it
    if (Is_udd_setup_received(0))
    {
        status = usbs_kinetis_control_setup(status);
        goto control_dsr_end;
    }

    // Check for received data on the control endpoint
    if (Is_udd_out_received(0))
    {
        status = usbs_kinetis_control_data_recv(status);
        goto control_dsr_end;
    }

    // Check if the last packet has been sent
    if (Is_udd_in_send(0) && Is_udd_in_send_interrupt_enabled(0)) 
    {
        status = usbs_kinetis_control_data_sent(status);
        goto control_dsr_end;
    }

    if (Is_udd_nak_out(0)) 
    {
         // Overflow on OUT packet
         CYG_TRACE0( true, "Nak out\n" );
         udd_ack_nak_out(0);
         status = udd_ctrl_overflow(status);
         return;
    }
    if (Is_udd_nak_in(0)) 
    {
         // Underflow on IN packet
         CYG_TRACE0( true, "Nak in\n" );
         udd_ack_nak_in(0);
         status = udd_ctrl_underflow(status);
         return;
    }

control_dsr_end:
    if (status == UDD_EPCTRL_STALL_REQ)
    {
        udd_enable_stall_handshake(0);
    }
}

///
void usbs_kinetis_attach(void)
{
  cyg_uint32 irq;
  HAL_DISABLE_INTERRUPTS(irq);

  // At startup the USB bus state is unknown,
  // therefore the state is considered IDLE to not miss any USB event
  otg_unfreeze_clock();
  while (!Is_otg_clock_usable());

  // Authorize attach if Vbus is present
  udd_attach_device();

  // Enable USB line events
  udd_enable_reset_interrupt();
  udd_enable_suspend_interrupt();
  udd_enable_wake_up_interrupt();
  udd_enable_sof_interrupt();

  // Reset following interrupts flag
  udd_ack_reset();
  udd_ack_sof();

  // The first suspend interrupt must be forced
  udd_raise_suspend();
  udd_ack_wake_up();
  otg_freeze_clock();
  HAL_RESTORE_INTERRUPTS(irq);
}


void usbs_kinetis_detach(void)
{
  otg_unfreeze_clock();

  // Detach device from the bus
  udd_detach_device();
  otg_freeze_clock();
}
//

 void
usbs_kinetis_dsr (cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    int n;
    CYG_ASSERT (CYGNUM_HAL_VECTOR_USB == vector, "Wrong interrupts");
    CYG_ASSERT (0 == data, "DSR needs no data");
	
    if (ep_job_str[0].call_dsr)
    {
        usbs_kinetis_control_dsr ();
        udd_enable_endpoint_interrupt(0);
    }

    for (n = 1; n < UC3C_USB_ENDPOINTS; n++) 
    {
        if (ep_job_str[n].call_dsr)
        {
            usbs_kinetis_endpoint_dsr (n);
            ep_job_str[n].call_dsr = false;
        }
    }
		
    while(which_dsr)
    {

        // USB bus reset detection
        if (which_dsr&DSR_USB_RESET)
        {
            usbs_kinetis_ep0.state = USBS_STATE_POWERED;
            usbs_state_notify (&usbs_kinetis_ep0);
            //otg_unfreeze_clock();
            usbs_kinetis_handle_reset ();
            //otg_freeze_clock();
            usbs_kinetis_ep0.state = USBS_STATE_DEFAULT;
            usbs_state_notify (&usbs_kinetis_ep0);
            CYG_TRACE0( true, "USB reset\n" );
            udd_ack_reset();
            udd_enable_reset_interrupt();
            which_dsr &= ~DSR_USB_RESET;
        }
		
        if (which_dsr&DSR_USB_SUSPEND)
        {
            usbs_kinetis_ep0.state = usbs_kinetis_ep0.state | USBS_STATE_SUSPENDED;
            usbs_state_notify (&usbs_kinetis_ep0);
            //otg_unfreeze_clock();
            // The suspend interrupt is automatic acked when a wakeup occur
            //udd_disable_suspend_interrupt();
            udd_enable_wake_up_interrupt();
            otg_freeze_clock(); // Mandatory to exit of sleep mode after a wakeup event
            CYG_TRACE0( true, "Suspend\n" );
            //udd_sleep_mode(false); // Enter in SUSPEND mode
            which_dsr &= ~DSR_USB_SUSPEND;
        }
		
        if (which_dsr&DSR_USB_WAKE_UP) 
        {
            usbs_kinetis_ep0.state = USBS_STATE_DEFAULT;
            usbs_state_notify (&usbs_kinetis_ep0);
            // Ack wakeup interrupt and enable suspend interrupt
            //otg_unfreeze_clock(); v ISR

            // Check USB clock ready after suspend and eventually sleep USB clock
            while( !Is_otg_clock_usable() );
            CYG_TRACE0( true, "Wakeup\n" );
            // The wakeup interrupt is automatic acked when a suspend occur
            //udd_disable_wake_up_interrupt();
            udd_enable_suspend_interrupt();
            which_dsr &= ~DSR_USB_WAKE_UP;
        }
		
		
        if (which_dsr&DSR_USB_VBUS)
        {
#ifndef BOOT_LOADER
            if(!Is_otg_vbus_high())
	    {
                    queue_put(WM_ON_USB_UNPLUG,0,0);
	    }
            else
	    {
                    queue_put(WM_ON_USB_PLUG,0,0);
	    }
#endif
	    usbs_kinetis_ep0.state = USBS_STATE_POWERED;
            usbs_state_notify (&usbs_kinetis_ep0);
            which_dsr &= ~DSR_USB_VBUS;
        }
    }	
}

cyg_uint32
usbs_kinetis_isr (cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_uint32 ret = CYG_ISR_HANDLED;
    cyg_uint8 n;

    CYG_ASSERT (CYGNUM_HAL_VECTOR_USB == vector, "Wrong interrupts");
    CYG_ASSERT (0 == data, "ISR needs no data");

    if (Is_udd_sof() && Is_udd_sof_interrupt_enabled()) 
    {
        udd_ack_sof();
        goto udd_interrupt_end;
    }
    
    for (n = 1; n < UC3C_USB_ENDPOINTS; n++) 
    {
        if (Is_udd_endpoint_interrupt_enabled(n) && Is_udd_endpoint_interrupt(n))
        {
            if(usbs_kinetis_endpoint_isr (n))
            {
                ep_job_str[n].call_dsr = true;
                ret |= CYG_ISR_CALL_DSR;
            }
            goto udd_interrupt_end;
        }
    }
				
    if (Is_udd_endpoint_interrupt(0)) 
    {
        udd_disable_endpoint_interrupt(0);
        ep_job_str[0].call_dsr = true;
        ret |= CYG_ISR_CALL_DSR;
        goto udd_interrupt_end;
    }
    // USB bus reset detection
    if (Is_udd_reset()) 
    {
        if(Is_otg_clock_frozen())
            otg_unfreeze_clock();

        udd_disable_reset_interrupt();
        ret |= CYG_ISR_CALL_DSR;
        which_dsr |= DSR_USB_RESET;
        goto udd_interrupt_end;
    }
	
    if (Is_udd_suspend_interrupt_enabled() && Is_udd_suspend()) 
    {
        otg_unfreeze_clock();
        udd_disable_suspend_interrupt();
        ret |= CYG_ISR_CALL_DSR;
        which_dsr |= DSR_USB_SUSPEND;
        goto udd_interrupt_end;
    }
	
    if (Is_udd_wake_up_interrupt_enabled() && Is_udd_wake_up()) 
    {
        otg_unfreeze_clock();
        udd_disable_wake_up_interrupt();
        ret |= CYG_ISR_CALL_DSR;
        which_dsr |= DSR_USB_WAKE_UP;
        goto udd_interrupt_end;
        //udd_sleep_mode(true); // Enter in IDLE mode
    }
	
    if (Is_otg_vbus_transition()) 
    {
        // Ack Vbus transition and send status to high level
        otg_unfreeze_clock();
	otg_ack_vbus_transition();
	otg_freeze_clock();
        if (Is_otg_vbus_high() && ep0_start == true) 
	{
	  usbs_kinetis_attach();
	} 
	else
	{
	  usbs_kinetis_detach();
	}

        ret |= CYG_ISR_CALL_DSR;
        which_dsr |= DSR_USB_VBUS;
        goto udd_interrupt_end;
    }
    
udd_interrupt_end:
    otg_data_memory_barrier();
  return ret;
}

cyg_uint32
usbs_plug_isr (cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_uint32 ret = CYG_ISR_HANDLED;

    //CYG_ASSERT (CYGNUM_HAL_VECTOR_GPIO_4 == vector, "Wrong interrupts");
    CYG_ASSERT (0 == data, "ISR needs no data");
    
    if (gpio_get_pin_interrupt_flag(AVR32_PIN_PB05)) 
    {
        gpio_clear_pin_interrupt_flag(AVR32_PIN_PB05);
        // Ack Vbus transition and send status to high level
        otg_unfreeze_clock();
	otg_ack_vbus_transition();
	otg_freeze_clock();
        if (gpio_get_pin_value(AVR32_PIN_PB05) && ep0_start == true) 
	{
	  usbs_kinetis_attach();
	} 
	else
	{
	  usbs_kinetis_detach();
	}

        ret |= CYG_ISR_CALL_DSR;
        which_dsr |= DSR_USB_VBUS;
    }
    return ret;
}

// ----------------------------------------------------------------------------
// Polling support. It is not clear that this is going to work particularly
// well since according to the documentation the hardware does not generate
// NAKs automatically - instead the ISR has to set the appropriate bits
// sufficiently quickly to avoid confusing the host.
//
// Calling the isr directly avoids duplicating code, but means that
// cyg_drv_interrupt_acknowledge() will get called when not inside a
// real interrupt handler. This should be harmless.

void
usbs_kinetis_poll (usbs_control_endpoint * endpoint)
{
  CYG_ASSERT (endpoint == &usbs_kinetis_ep0, "Wrong endpoint");
  if (CYG_ISR_CALL_DSR == usbs_kinetis_isr (CYGNUM_HAL_VECTOR_USB, 0)) {
    usbs_kinetis_dsr (CYGNUM_HAL_VECTOR_USB, 0, 0);
  }
}

// ----------------------------------------------------------------------------
// Initialization
//
// This routine gets called from a prioritized static constructor during
// eCos startup.
//! Unlock SCIF register macro
#define SCIF_UNLOCK(reg)  (AVR32_SCIF.unlock = (AVR32_SCIF_UNLOCK_KEY_VALUE << AVR32_SCIF_UNLOCK_KEY_OFFSET)|(reg))

void usbs_kinetis_init (void)
{
    int i;
    cyg_uint32 old_intr;
    SCIF_UNLOCK(AVR32_SCIF_GCCTRL);
    //TODOO: set this by configtool
    AVR32_SCIF.gcctrl[7] = (12 << 8) | 1 | /*2 |*/ (1 << 16);

    HAL_DISABLE_INTERRUPTS(old_intr);
    // ID pin not used then force device mode
    otg_disable_id_pin();
    otg_force_device_mode();

    // Enable USB hardware
    otg_enable_pad();
    otg_enable();
    otg_unfreeze_clock();
    Is_otg_clock_frozen();

    memset((cyg_uint8 *) udd_g_ep_table, 0, sizeof(udd_g_ep_table));
    otg_register_desc_tab(udd_g_ep_table);

    for (i = 0; i < UC3C_USB_ENDPOINTS; i++) 
    {
        ep_job_str[i].call_dsr = false;
    }

    udd_low_speed_disable();

    otg_ack_vbus_transition();
    // Force Vbus interrupt in case of Vbus always with a high level
    // This is possible with a short timing between a Host mode stop/start.
    if (Is_otg_vbus_high()) 
    {
        otg_raise_vbus_transition();
    }
    otg_enable_vbus_interrupt();
    otg_freeze_clock();

    usbs_kinetis_reset_device ();

    cyg_drv_interrupt_create (CYGNUM_HAL_VECTOR_USB,
                            0,  // priority
                            0,  // data
                            &usbs_kinetis_isr,
                            &usbs_kinetis_dsr,
                            &usbs_kinetis_intr_handle, &usbs_kinetis_intr_data);

    cyg_drv_interrupt_attach (usbs_kinetis_intr_handle);
    
#if 0
    cyg_drv_interrupt_create (CYGNUM_HAL_VECTOR_GPIO_4,
                            0,  // priority
                            0,  // data
                            &usbs_plug_isr,
                            &usbs_kinetis_dsr,
                            &usbs_plug_intr_handle, &usbs_plug_intr_data);

    cyg_drv_interrupt_attach (usbs_plug_intr_handle);
    
    gpio_configure_pin(AVR32_PIN_PB05, GPIO_DIR_INPUT | GPIO_INTERRUPT |
                           GPIO_BOTHEDGES);

    gpio_clear_pin_interrupt_flag(AVR32_PIN_PB05);
    gpio_enable_pin_interrupt(AVR32_PIN_PB05,GPIO_PIN_CHANGE);
#endif
#if UC3L3_L4
    usbs_kinetis_attach();
    gpio_enable_module_pin(AVR32_USBC_DM_0_PIN, AVR32_USBC_DM_0_FUNCTION);
    gpio_enable_module_pin(AVR32_USBC_DP_0_PIN, AVR32_USBC_DP_0_FUNCTION);
#endif
    //queue_init();
    HAL_RESTORE_INTERRUPTS(old_intr);

    usbs_kinetis_ep0.state = USBS_STATE_POWERED;
    usbs_state_notify (&usbs_kinetis_ep0);
}

// ----------------------------------------------------------------------------
// Testing support.
usbs_testing_endpoint usbs_testing_endpoints[] = {
    {
        endpoint_type       : USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL,
        endpoint_number     : 0,
        endpoint_direction  : USB_ENDPOINT_DESCRIPTOR_ENDPOINT_IN,
        endpoint            : (void*) &usbs_kinetis_ep0,
#ifdef CYGVAR_DEVS_USB_AT91_EP0_DEVTAB_ENTRY
        devtab_entry        : CYGDAT_DEVS_USB_AT91_DEVTAB_BASENAME "0c",
#else
        devtab_entry        : (const char*) 0,
#endif
        min_size            : 1,            // zero-byte control transfers are meaningless
        max_size            : 0x0FFFF,      // limit imposed by protocol
        max_in_padding      : 0,
        alignment           : 0
    },
    {
        endpoint_type       : USB_ENDPOINT_DESCRIPTOR_ATTR_BULK,
        endpoint_number     : 1,
        endpoint_direction  : USB_ENDPOINT_DESCRIPTOR_ENDPOINT_OUT,
        endpoint            : (void*) &usbs_kinetis_ep1,
#ifdef CYGVAR_DEVS_USB_AT91_EP1_DEVTAB_ENTRY
        devtab_entry        : CYGDAT_DEVS_USB_AT91_DEVTAB_BASENAME "1r",
#else
        devtab_entry        : (const char*) 0,
#endif
        min_size            : 1,
        max_size            : -1,           // No hardware or driver limitation
        max_in_padding      : 0,
        alignment           : 0
    },
    {
        endpoint_type       : USB_ENDPOINT_DESCRIPTOR_ATTR_BULK,
        endpoint_number     : 2,
        endpoint_direction  : USB_ENDPOINT_DESCRIPTOR_ENDPOINT_IN,
        endpoint            : (void*) &usbs_kinetis_ep2,
#ifdef CYGVAR_DEVS_USB_AT91_EP2_DEVTAB_ENTRY
        devtab_entry        : CYGDAT_DEVS_USB_AT91_DEVTAB_BASENAME "2w",
#else
        devtab_entry        : (const char*) 0,
#endif
        min_size            : 1,
        max_size            : -1,           // No hardware or driver limitation
        max_in_padding      : 1,            // hardware limitation
        alignment           : 0
    },
    {
        endpoint_type       : USB_ENDPOINT_DESCRIPTOR_ATTR_BULK,
        endpoint_number     : 3,
        endpoint_direction  : USB_ENDPOINT_DESCRIPTOR_ENDPOINT_IN,
        endpoint            : (void*) &usbs_kinetis_ep3,
#ifdef CYGVAR_DEVS_USB_AT91_EP3_DEVTAB_ENTRY
        devtab_entry        : CYGDAT_DEVS_USB_AT91_DEVTAB_BASENAME "3w",
#else
        devtab_entry        : (const char*) 0,
#endif
        min_size            : 1,
        max_size            : -1,           // No hardware or driver limitation
        max_in_padding      : 1,            // hardware limitation
        alignment           : 0
    },
    USBS_TESTING_ENDPOINTS_TERMINATOR
};



