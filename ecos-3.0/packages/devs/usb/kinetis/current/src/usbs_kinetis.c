//==========================================================================
//
//      usbs_kinetis.c
//
//      Driver for the Freescale Kinetis USB device
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
// Author(s):    Filip,
// Contributors:
// Date:         2017-04-20
//
// This code implements support for the on-chip USB port on the Freescale
// Kinetis family of processors.
//
//####DESCRIPTIONEND####
//==========================================================================

#include <pkgconf/hal.h>
#include <pkgconf/devs_usb_kinetis.h>
#include <pkgconf/system.h>

#include <stdint.h>

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_cache.h>

#include <cyg/io/usb/usb.h>
#include <cyg/io/usb/usbs.h>
#include <cyg/io/usb/usbs_kinetis.h>

#include "usb_hdr.h"

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/cyg_trac.h>
#include <cyg/infra/diag.h>
#include <string.h>

#include "usb_khci.h"
#include "usb_device_config.h"
#include "usb.h"

#include "usb_device.h"
#include "usb_device_dci.h"

#include "usb_spec.h"
#include "usb_device.h"
#include <cyg/io/usb/usbs_kinetis.h>
#include "usb_device_khci.h"
#if 1

/*! @brief Control read and write sequence */
typedef enum _usb_device_control_read_write_sequence
{
    kUSB_DeviceControlPipeSetupStage = 0U, /*!< Setup stage */
    kUSB_DeviceControlPipeDataStage,       /*!< Data stage */
    kUSB_DeviceControlPipeStatusStage,     /*!< status stage */
} usb_device_control_read_write_sequence_t;



//-----------------------------------------------------------------------------
// Maintenance and debug macros.

#define TODO_USB(_msg_) CYG_ASSERT(false, "TODO (USB) : " _msg_)
#define FAIL_USB(_msg_) CYG_ASSERT(false, "FAIL (USB) : " _msg_)
#define ASSERT_USB(_test_, _msg_) CYG_ASSERT(_test_, "FAIL (USB) : " _msg_)

#if defined(CYGBLD_DEVS_USB_CORTEXM_STM32_DEBUG_TRACE)
#define TRACE_USB(_msg_, _args_...) diag_printf ("STM32 USB : " _msg_, ##_args_)
#else
#define TRACE_USB(_msg_, _args_...) while(0){}
#endif


// Driver API functions
static void usbs_kinetis_ep0_start(usbs_control_endpoint *);
static void usbs_kinetis_poll(usbs_control_endpoint *);

void usbs_kinetis_endpoint_start(usbs_rx_endpoint * pep);
void usbs_kinetis_endpoint_set_halted(usbs_rx_endpoint * pep,
                                          cyg_bool new_value);

usb_status_t usbs_state_notify
  (int new_state, usbs_state_change state_change);

extern void USB_DeviceKhciIsrFunction(void *deviceHandle);

static usbs_tx_endpoint* kinetis_usb_get_txep
  (usbs_control_endpoint* control_endpoint, cyg_uint8 ep_id);

static usbs_rx_endpoint* kinetis_usb_get_rxep
  (usbs_control_endpoint* control_endpoint, cyg_uint8 ep_id);

usb_status_t
usbs_kinetis_control_setup(usb_device_struct_t *handle,
		  usb_device_endpoint_callback_message_struct_t *message);


  // Endpoint 0, the control endpoint, structure.
  usbs_control_endpoint ep0 = {
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
                                                       int)) 0,
	  get_rxep_fn            : &kinetis_usb_get_rxep,
	  get_txep_fn            : &kinetis_usb_get_txep,
  };


// jinak
usb_device_struct_t *kinetis_usb_device;

static usb_status_t usbs_kinetis_fill_in_control_buffer(usb_device_struct_t *handle, uint8_t **buffer)
{
	cyg_uint8 *fill_buffer = handle->control_transfer_in_buffer;
	if((handle->controlEndpoint->buffer < ((cyg_uint8*)0x100000)) &&
	   (handle->controlEndpoint->buffer != NULL))
	{
		if(handle->controlEndpoint->buffer_size < sizeof(handle->control_transfer_in_buffer))
		{
			memcpy(fill_buffer,handle->controlEndpoint->buffer,handle->controlEndpoint->buffer_size);
			*buffer = fill_buffer;
		}
		else
		{
			return kStatus_USB_InvalidRequest;
		}
	}
	return kStatus_USB_Success;
}

/*!
 * @brief Handle set address request.
 *
 * This function is used to handle set address request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state.
 */
static usb_status_t USB_DeviceSetAddress(usb_device_struct_t *handle,
                                            usb_setup_struct_t *setup,
                                            uint8_t **buffer,
                                            uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;

    USB_DeviceGetStatus(handle, kUSB_DeviceStatusDeviceState, &state);

    if ((kUSB_DeviceStateAddressing != state) && (kUSB_DeviceStateAddress != state) &&
        (kUSB_DeviceStateDefault != state) && (kUSB_DeviceStateConfigured != state))
    {
        return error;
    }

    if (kUSB_DeviceStateAddressing != state)
    {
        /* If the device address is not setting, pass the address and the device state will change to
         * kUSB_DeviceStateAddressing internally. */
        state = setup->wValue & 0xFFU;
        error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusAddress, &state);
    }
    else
    {
        /* If the device address is setting, set device address and the address will be write into the controller
         * internally. */
        error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusAddress, NULL);
        /* And then change the device state to kUSB_DeviceStateAddress. */
        if (kStatus_USB_Success == error)
        {
            state = kUSB_DeviceStateAddress;
            error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusDeviceState, &state);
        }
    }

    return error;
}

/*!
 * @brief Send the reponse to the host.
 *
 * This function is used to send the reponse to the host.
 *
 * There are two cases this function will be called.
 * Case one when a setup packet is received in control endpoint callback function:
 *        1. If there is not data phase in the setup transfer, the function will prime an IN transfer with the data
 * length is zero for status phase.
 *        2. If there is an IN data phase, the function will prime an OUT transfer with the actual length to need to
 * send for data phase. And then prime an IN transfer with the data length is zero for status phase.
 *        3. If there is an OUT data phase, the function will prime an IN transfer with the actual length to want to
 * receive for data phase.
 *
 * Case two when is not a setup packet received in control endpoint callback function:
 *        1. The function will prime an IN transfer with data length is zero for status phase.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param error           The error code returned from the standard request fucntion.
 * @param stage           The stage of the control transfer.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceControlCallbackFeedback(usb_device_handle handle,
                                                      usb_setup_struct_t *setup,
                                                      usb_status_t error,
                                                      usb_device_control_read_write_sequence_t stage,
                                                      uint8_t **buffer,
                                                      uint32_t *length)
{
    usb_status_t errorCode = kStatus_USB_Error;
    uint8_t direction = USB_IN;

    if(usbs_kinetis_fill_in_control_buffer(handle,buffer) != kStatus_USB_Success)
    	error = kStatus_USB_InvalidRequest;

    if (kStatus_USB_InvalidRequest == error)
    {
    	//diag_printf("stall\n");
        /* Stall the control pipe when the request is unsupported. */
        if ((!((setup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK) == USB_REQUEST_TYPE_TYPE_STANDARD)) &&
            ((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) == USB_REQUEST_TYPE_DIR_OUT) && (setup->wLength) &&
            (kUSB_DeviceControlPipeSetupStage == stage))
        {
            direction = USB_OUT;
        }

        errorCode = USB_DeviceStallEndpoint(
            handle,
            (USB_CONTROL_ENDPOINT) | (uint8_t)((uint32_t)direction << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
    }
    else
    {
        if (*length > setup->wLength)
        {
            *length = setup->wLength;
        }

        errorCode = USB_DeviceSendRequest(handle, (USB_CONTROL_ENDPOINT), *buffer, *length);

        if ((kStatus_USB_Success == errorCode) &&
            (USB_REQUEST_TYPE_DIR_IN == (setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK)))
        {
            errorCode = USB_DeviceRecvRequest(handle, (USB_CONTROL_ENDPOINT), (uint8_t *)NULL, 0U);
        }
    }
    return errorCode;
}


 /******************************************************************************
 *   Device API routines.                                                      *
 ******************************************************************************/
 // Start endpoint reception/trasmitrion
void
usbs_kinetis_endpoint_start (usbs_rx_endpoint * ppep)
{
    usb_rx_tx_endpoint *epstr = (usb_rx_tx_endpoint*)ppep;
    usbs_rx_endpoint * pep    = &epstr->endpoint;
    cyg_uint8 epAddress       =  epstr->epAddress;

    CYG_ASSERT (pep->complete_fn, "No complete_fn()");

    cyg_drv_dsr_lock ();
    if (ep0.state != USBS_STATE_CONFIGURED) {
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

    // Request next transfer
    if((epAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
    		USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT)
    {
		if(USB_DeviceRecvRequest(kinetis_usb_device, epAddress, pep->buffer, pep->buffer_size)
				!= kStatus_USB_Success)
		{
			if (pep->complete_fn)
			{
				(*pep->complete_fn) (pep->complete_data, -EIO);
			}
		}
    }
    else
    {
    	if(USB_DeviceSendRequest(kinetis_usb_device, epAddress, pep->buffer, pep->buffer_size)
				!= kStatus_USB_Success)
		{
			if (pep->complete_fn)
			{
				(*pep->complete_fn) (pep->complete_data, -EIO);
			}
		}
    }

    cyg_drv_dsr_unlock ();
}

// Stall or UnStall specifed endpoint
void
usbs_kinetis_endpoint_set_halted (usbs_rx_endpoint * ppep, cyg_bool new_value)
{
	usb_rx_tx_endpoint *epstr = (usb_rx_tx_endpoint*)ppep;
	usbs_rx_endpoint   *pep   = &epstr->endpoint;
	cyg_uint8 epAddress       = epstr->epAddress;

    cyg_drv_dsr_lock ();

    if (pep->halted != new_value) {
        /* There is something is to do */
        pep->halted = new_value;

        if ( new_value )
        {
            /* Halt endpoint */
        	USB_DeviceStallEndpoint(kinetis_usb_device, epAddress);
        	// Indicate error via the completion callback.

        } else
        {
        	USB_DeviceUnstallEndpoint(kinetis_usb_device, epAddress);
            // Ready to use
            if (pep->complete_fn) {
              (*pep->complete_fn) (pep->complete_data, ENOERR);
             }
        }
    }
    cyg_drv_dsr_unlock ();
}

// Init all used edpoints
void usbs_ep_alloc(void)
{
    int epn;
    const usb_endpoint_descriptor *usb_endpoints;

    // Now walk the endpoints configuring them correctly. This only
    // works if there is one interface.
    usb_endpoints = ep0.enumeration_data->endpoints;

    for (epn = 0;
    epn < ep0.enumeration_data->total_number_endpoints;
    epn++)
    {
        usb_device_endpoint_init_struct_t initStr;

        if((usb_endpoints[epn].endpoint & USB_ENDPOINT_NUMBER_MASK) == 0)
        	++epn;

        initStr.maxPacketSize = ((cyg_uint32)usb_endpoints[epn].max_packet_lo |
                            ((cyg_uint32)usb_endpoints[epn].max_packet_hi << 8));

        initStr.endpointAddress = usb_endpoints[epn].endpoint;
        initStr.transferType    = usb_endpoints[epn].attributes;
        initStr.zlt             = 0U;

        USB_DeviceInitEndpoint(kinetis_usb_device, &initStr, NULL);
    }
}

// Start endpoint 0
void
usbs_kinetis_ep0_start (usbs_control_endpoint * endpoint)
{
    usb_device_endpoint_init_struct_t initStr;

    initStr.zlt             = 1U;
    initStr.transferType    = USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL;
    initStr.endpointAddress = USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL |
                    (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    initStr.maxPacketSize   = USB_CONTROL_MAX_PACKET_SIZE;


    USB_DeviceInitEndpoint(kinetis_usb_device, &initStr, NULL);

    initStr.endpointAddress = USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL |
                    (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);

    USB_DeviceInitEndpoint(kinetis_usb_device, &initStr, NULL);

    // Run
    //USB_DeviceRun(kinetis_usb_device);
}

// There has been a change in state. Update the end point.
usb_status_t usbs_state_notify
  (int new_state, usbs_state_change state_change)
{
	// TODO reset endpoints
	int old_state = ep0.state;
	if(state_change == USBS_STATE_CHANGE_RESET)
		usbs_kinetis_ep0_start(&ep0);

  ep0.state = new_state;
  if (ep0.state_change_fn)
    (*ep0.state_change_fn) (&ep0,
      ep0.state_change_data, state_change, old_state);

  return kStatus_USB_Success;
}

//-----------------------------------------------------------------------------
// Get a handle on the specified transmit (in) endpoint.

static usbs_tx_endpoint* kinetis_usb_get_txep
  (usbs_control_endpoint* control_endpoint, cyg_uint8 ep_id)
{
	usbs_tx_endpoint* txep = NULL;

  // Map from endpoint ID to physical endpoint.
  if (ep_id > 0 && ep_id < 16)
    txep = (usbs_tx_endpoint*)&kinetis_usb_device->endpoints [ep_id - 1];

  // Return endpoint handle or null pointer for invalid endpoint.
  if (txep == NULL) {
	  CYG_FAIL ("Invalid endpoint ID when accessing transmit (in) endpoint.");
    return NULL;
  }
  return (usbs_tx_endpoint*) txep;
}

//-----------------------------------------------------------------------------
// Get a handle on the specified receive (out) endpoint.

static usbs_rx_endpoint* kinetis_usb_get_rxep
  (usbs_control_endpoint* control_endpoint, cyg_uint8 ep_id)
{
    usbs_rx_endpoint* rxep = NULL;

  // Map from endpoint ID to physical endpoint.
  if (ep_id > 0 && ep_id < 16)
     rxep = (usbs_tx_endpoint*)&kinetis_usb_device->endpoints [ep_id - 1];

  // Return endpoint handle or null pointer for invalid endpoint.
  if (rxep == NULL) {
	  CYG_FAIL ("Invalid endpoint ID when accessing receive (out) endpoint.");
    return NULL;
  }
  return (usbs_rx_endpoint*) rxep;
}


/*!
 * @brief Handle get status request.
 *
 * This function is used to handle get status request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
static usb_status_t USB_DeviceChGetStatus(usb_device_struct_t *handle,
                                           usb_setup_struct_t *setup,
                                           uint8_t **buffer,
                                           uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;

    USB_DeviceGetStatus(handle, kUSB_DeviceStatusDeviceState, &state);

    if ((kUSB_DeviceStateAddress != state) && (kUSB_DeviceStateConfigured != state))
    {
        return error;
    }

    if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_DEVICE)
    {
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        if (setup->wIndex == USB_REQUEST_STANDARD_GET_STATUS_OTG_STATUS_SELECTOR)
        {
            error =
                USB_DeviceGetStatus(handle, kUSB_DeviceStatusOtg, &ep0.transaction_buffer[0]);
            classHandle->standardTranscationBuffer = USB_SHORT_TO_LITTLE_ENDIAN(classHandle->standardTranscationBuffer);
            /* The device status length must be USB_DEVICE_STATUS_SIZE. */
            *length = USB_DEVICE_STATUS_SIZE;
        }
        else /* Get the device status */
        {
#endif
        	cyg_uint16 status;
            error = USB_DeviceGetStatus(handle, kUSB_DeviceStatusDevice,
                                        &status);
            status = status & USB_GET_STATUS_DEVICE_MASK;
            handle->controlEndpoint->transaction_buffer[0] = status & 0xff;
            handle->controlEndpoint->transaction_buffer[1] = (status >> 8) & 0xff;

            /* The device status length must be USB_DEVICE_STATUS_SIZE. */
            *length = USB_DEVICE_STATUS_SIZE;
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        }
#endif
    }
    else if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
    {
        /* Get the interface status */
        error = kStatus_USB_Success;
        handle->controlEndpoint->transaction_buffer[0] = 0;
        handle->controlEndpoint->transaction_buffer[1] = 0;
        /* The interface status length must be USB_INTERFACE_STATUS_SIZE. */
        *length = USB_INTERFACE_STATUS_SIZE;
    }
    else if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_ENDPOINT)
    {
        /* Get the endpoint status */
        usb_device_endpoint_status_struct_t endpointStatus;
        endpointStatus.endpointAddress = (uint8_t)setup->wIndex;
        endpointStatus.endpointStatus = kUSB_DeviceEndpointStateIdle;
        error = USB_DeviceGetStatus(handle, kUSB_DeviceStatusEndpoint, &endpointStatus);
        handle->controlEndpoint->transaction_buffer[0] = endpointStatus.endpointStatus & USB_GET_STATUS_ENDPOINT_MASK;
        handle->controlEndpoint->transaction_buffer[1] = 0;
        /* The endpoint status length must be USB_INTERFACE_STATUS_SIZE. */
        *length = USB_ENDPOINT_STATUS_SIZE;
    }
    else
    {
    }

    *buffer = (uint8_t *)&handle->controlEndpoint->transaction_buffer[0];

    return error;
}

/*!
 * @brief Handle set or clear device feature request.
 *
 * This function is used to handle set or clear device feature request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
static usb_status_t USB_DeviceChSetClearFeature(usb_device_struct_t *handle,
                                                 usb_setup_struct_t *setup,
                                                 uint8_t **buffer,
                                                 uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;
    uint8_t isSet = 0U;

    USB_DeviceGetStatus(handle, kUSB_DeviceStatusDeviceState, &state);

    if ((kUSB_DeviceStateAddress != state) && (kUSB_DeviceStateConfigured != state))
    {
        return error;
    }

    /* Identify the request is set or clear the feature. */
    if (USB_REQUEST_STANDARD_SET_FEATURE == setup->bRequest)
    {
        isSet = 1U;
    }

    if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_DEVICE)
    {
        /* Set or Clear the device featrue. */
        if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_DEVICE_REMOTE_WAKEUP == setup->wValue)
        {
#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
            USB_DeviceSetStatus(handle, kUSB_DeviceStatusRemoteWakeup, &isSet);
#endif
            /* Set or Clear the device remote wakeup featrue. */
            //error = USB_DeviceClassCallback(handle, kUSB_DeviceEventSetRemoteWakeup, &isSet);
        }
#if ((defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))) && \
    (defined(USB_DEVICE_CONFIG_USB20_TEST_MODE) && (USB_DEVICE_CONFIG_USB20_TEST_MODE > 0U))
        else if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_DEVICE_TEST_MODE == setup->wValue)
        {
            state = kUSB_DeviceStateTestMode;
            error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusDeviceState, &state);
        }
#endif
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        else if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_B_HNP_ENABLE == setup->wValue)
        {
            error = USB_DeviceClassCallback(handle, kUSB_DeviceEventSetBHNPEnable, &isSet);
        }
#endif
        else
        {
        }
    }
    else if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_ENDPOINT)
    {
        /* Set or Clear the endpoint featrue. */
        if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_ENDPOINT_HALT == setup->wValue)
        {
            usbs_rx_endpoint * pep = &handle->endpoints[((uint8_t)setup->wIndex & USB_ENDPOINT_NUMBER_MASK) - 1].endpoint;

            if (USB_CONTROL_ENDPOINT == (setup->wIndex & USB_ENDPOINT_NUMBER_MASK))
            {
                /* Set or Clear the control endpoint status(halt or not). */
                if (isSet)
                {
                    USB_DeviceStallEndpoint(handle, (uint8_t)setup->wIndex);
                }
                else
                {
                    USB_DeviceUnstallEndpoint(handle, (uint8_t)setup->wIndex);
                }
            }

            /* Set or Clear the endpoint status featrue. */
            if (isSet)
            {
            	if (pep->complete_fn) {
				  (*pep->complete_fn) (pep->complete_data, -EIO);
				 }
				error = kStatus_USB_Success;
            }
            else
            {


                if (pep->complete_fn) {
				  (*pep->complete_fn) (pep->complete_data, -EAGAIN);
				 }
                error = kStatus_USB_Success;
            }
        }
        else
        {
        }
    }
    else
    {
    }

    return error;
}


usb_status_t usbs_kinetis_fill_buffer(usb_device_struct_t *handle)
{
	usb_status_t error = kStatus_USB_Success;
	cyg_uint8 *fill_buffer = handle->control_transfer_in_buffer;

	if(handle->controlEndpoint->fill_buffer_fn)
	{
		cyg_uint32 pos = 0;

		memcpy(fill_buffer,handle->controlEndpoint->buffer,handle->controlEndpoint->buffer_size);
		pos += handle->controlEndpoint->buffer_size;
		while(handle->controlEndpoint->fill_buffer_fn)
		{
			if((handle->controlEndpoint->buffer_size + pos) < sizeof(handle->control_transfer_in_buffer))
			{
			  (*handle->controlEndpoint->fill_buffer_fn) (handle->controlEndpoint);
			  memcpy(fill_buffer + pos,handle->controlEndpoint->buffer,handle->controlEndpoint->buffer_size);
			  pos += handle->controlEndpoint->buffer_size;
			}
			else
			{
				error = kStatus_USB_InvalidRequest;
			    CYG_ASSERT(false,"USB control buffer overflow\n");
			    return error;
			}
		}

		handle->controlEndpoint->buffer      = fill_buffer;
		handle->controlEndpoint->buffer_size = pos;
	}
	return error;
}

// Handle a setup message from the host
usb_status_t
usbs_kinetis_control_setup(usb_device_struct_t *handle,
		usb_device_endpoint_callback_message_struct_t *message)
{
	usbs_control_return status;
    usb_setup_struct_t *deviceSetup;
    //usb_device_common_class_struct_t *classHandle;
    uint8_t *buffer = (uint8_t *)NULL;
    uint32_t length = 0U;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;

    if ((0xFFFFFFFFU == message->length))
    {
        return error;
    }

    deviceSetup = (usb_setup_struct_t *)&handle->controlEndpoint->control_buffer[0];
    USB_DeviceGetStatus(handle, kUSB_DeviceStatusDeviceState, &state);

    //diag_printf("s: %d\n",message->isSetup);

    if (message->isSetup)
    {
        if ((USB_SETUP_PACKET_SIZE != message->length) || (NULL == message->buffer))
        {
            /* If a invalid setup is received, the control pipes should be de-init and init again.
             * Due to the IP can not meet this require, it is revesed for feature.
             */
            /*
            USB_DeviceDeinitEndpoint(handle,
                         USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
            USB_DeviceDeinitEndpoint(handle,
                         USB_CONTROL_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
            USB_DeviceControlPipeInit(handle, callbackParam);
            */
            return error;
        }
        /* Receive a setup request */
        usb_setup_struct_t *setup = (usb_setup_struct_t *)(message->buffer);

        /* Copy the setup packet to the application buffer */
        deviceSetup->wValue = USB_SHORT_FROM_LITTLE_ENDIAN(setup->wValue);
        deviceSetup->wIndex = USB_SHORT_FROM_LITTLE_ENDIAN(setup->wIndex);
        deviceSetup->wLength = USB_SHORT_FROM_LITTLE_ENDIAN(setup->wLength);
        deviceSetup->bRequest = setup->bRequest;
        deviceSetup->bmRequestType = setup->bmRequestType;

        //diag_printf("rt: 0x%x, rq: 0x%x, w: 0x%x\n",deviceSetup->bmRequestType, deviceSetup->bRequest,deviceSetup->wLength);

        if ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK) == USB_REQUEST_TYPE_TYPE_STANDARD)
        {
            /* Handle the standard request */
			switch (deviceSetup->bRequest) {
				case USB_DEVREQ_GET_STATUS:
					error = USB_DeviceChGetStatus(handle, deviceSetup, &buffer,
							&length);
					break;
				case USB_DEVREQ_SET_ADDRESS:
					CYG_TRACE0( true, "Control Setup Set Addr\n" );
					error = USB_DeviceSetAddress(handle, deviceSetup,  &buffer,
							&length);
				  break;
				case USB_DEVREQ_SET_FEATURE:
					error = USB_DeviceChSetClearFeature(handle, deviceSetup, &buffer,
												&length);
					break;
				case USB_DEVREQ_CLEAR_FEATURE:
					error = USB_DeviceChSetClearFeature(handle, deviceSetup, &buffer,
												&length);
					break;
				default:
					if (!handle->controlEndpoint->standard_control_fn) {
						cyg_uint8 tmp[8];
						memcpy(tmp,deviceSetup,8);
						status = usbs_handle_standard_control (handle->controlEndpoint);
						if(status == USBS_CONTROL_RETURN_HANDLED)
							error = kStatus_USB_Success;
						usbs_kinetis_fill_buffer(handle);
						memcpy(deviceSetup,tmp,8);
						buffer = handle->controlEndpoint->buffer;
						length = handle->controlEndpoint->buffer_size;
						break;
					}

					status =
					  (*handle->controlEndpoint->standard_control_fn) (handle->controlEndpoint,
							  handle->controlEndpoint->standard_control_data);

					if (status == USBS_CONTROL_RETURN_UNKNOWN) {
						status = usbs_handle_standard_control (handle->controlEndpoint);
					}

					if(status == USBS_CONTROL_RETURN_HANDLED)
						error = kStatus_USB_Success;

					if(handle->controlEndpoint->fill_buffer_fn)
					{
						CYG_FAIL("Unexpected fill function\n");
					}

					buffer = handle->controlEndpoint->buffer;
					length = handle->controlEndpoint->buffer_size;
					break;
			}
        }
        else
        {
            if ((deviceSetup->wLength) &&
                ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) == USB_REQUEST_TYPE_DIR_OUT))
            {
                /* Class or vendor request with the OUT data phase. */
                if ((deviceSetup->wLength) &&
                    ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_CLASS) == USB_REQUEST_TYPE_TYPE_CLASS))
                {
                    /* Get data buffer to receive the data from the host. */
                	if (handle->controlEndpoint->class_control_fn) {
						status = (*handle->controlEndpoint->class_control_fn)
								 (handle->controlEndpoint,
								  handle->controlEndpoint->class_control_data);
						if(status == USBS_CONTROL_RETURN_HANDLED)
							error = kStatus_USB_Success;
						buffer = handle->controlEndpoint->buffer;
						length = handle->controlEndpoint->buffer_size;
                	}
                }
                else if ((deviceSetup->wLength) &&
                         ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_VENDOR) == USB_REQUEST_TYPE_TYPE_VENDOR))
                {
                    /* Get data buffer to receive the data from the host. */
                	if (handle->controlEndpoint->vendor_control_fn) {
						status = (*handle->controlEndpoint->vendor_control_fn)
								 (handle->controlEndpoint,
								  handle->controlEndpoint->vendor_control_data);
						if(status == USBS_CONTROL_RETURN_HANDLED)
							error = kStatus_USB_Success;
						buffer = handle->controlEndpoint->buffer;
						length = handle->controlEndpoint->buffer_size;
                	}
                }
                else
                {
                }
                if (kStatus_USB_Success == error)
                {
                    /* Prime an OUT transfer */
                    error = USB_DeviceRecvRequest(handle, USB_CONTROL_ENDPOINT, buffer, deviceSetup->wLength);
                    return error;
                }
            }
            else
            {
                /* Class or vendor request with the IN data phase. */
                if (((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_CLASS) == USB_REQUEST_TYPE_TYPE_CLASS))
                {
                    /* Get data buffer to response the host. */
                	if (handle->controlEndpoint->class_control_fn) {
						status = (*handle->controlEndpoint->class_control_fn)
								 (handle->controlEndpoint,
								  handle->controlEndpoint->class_control_data);
						if(status == USBS_CONTROL_RETURN_HANDLED)
							error = kStatus_USB_Success;
						buffer = handle->controlEndpoint->buffer;
						length = handle->controlEndpoint->buffer_size;
                	}
                }
                else if (((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_VENDOR) == USB_REQUEST_TYPE_TYPE_VENDOR))
                {
                    /* Get data buffer to response the host. */
                	if (handle->controlEndpoint->vendor_control_fn) {
						status = (*handle->controlEndpoint->vendor_control_fn)
								 (handle->controlEndpoint,
								  handle->controlEndpoint->vendor_control_data);
						if(status == USBS_CONTROL_RETURN_HANDLED)
							error = kStatus_USB_Success;
						buffer = handle->controlEndpoint->buffer;
						length = handle->controlEndpoint->buffer_size;
                	}
                }
                else
                {
                }
            }
        }
        /* Send the reponse to the host. */
        error = USB_DeviceControlCallbackFeedback(handle, deviceSetup, error, kUSB_DeviceControlPipeSetupStage, &buffer,
                                                  &length);
    }
    else if (kUSB_DeviceStateAddressing == state)
    {
        /* Set the device address to controller. */
    	error = USB_DeviceSetAddress(handle, deviceSetup, &buffer,&length);
    	usbs_ep_alloc();
    }
#if ((defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))) && \
    (defined(USB_DEVICE_CONFIG_USB20_TEST_MODE) && (USB_DEVICE_CONFIG_USB20_TEST_MODE > 0U))
    else if (kUSB_DeviceStateTestMode == state)
    {
        uint8_t portTestControl = (uint8_t)(deviceSetup->wIndex >> 8);
        /* Set the controller.into test mode. */
        error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusTestMode, &portTestControl);
    }
#endif
    else if ((message->length) && (deviceSetup->wLength) &&
             ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) == USB_REQUEST_TYPE_DIR_OUT))
    {
        if (((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_CLASS) == USB_REQUEST_TYPE_TYPE_CLASS))
        {
            /* Data received in OUT phase, and notify the class driver. */
        	if (handle->controlEndpoint->class_control_fn) {
				status = (*handle->controlEndpoint->class_control_fn)
						 (handle->controlEndpoint,
							  handle->controlEndpoint->class_control_data);
				if(status == USBS_CONTROL_RETURN_HANDLED)
					error = kStatus_USB_Success;
        	}
        }
        else if (((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_VENDOR) == USB_REQUEST_TYPE_TYPE_VENDOR))
        {
            /* Data received in OUT phase, and notify the application. */
        	if (handle->controlEndpoint->class_control_fn) {
				status = (*handle->controlEndpoint->class_control_fn)
						 (handle->controlEndpoint,
						  handle->controlEndpoint->vendor_control_data);
				if(status == USBS_CONTROL_RETURN_HANDLED)
					error = kStatus_USB_Success;
        	}
        }
        else
        {
        }
        /* Send the reponse to the host. */
        error = USB_DeviceControlCallbackFeedback(handle, deviceSetup, error, kUSB_DeviceControlPipeDataStage, &buffer,
                                                  &length);
    }
    else
    {
    }
    return error;
}

void
usbs_kinetis_dsr (cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	USB_DeviceKhciIsrFunction((void*)data);
	cyg_drv_interrupt_unmask(vector);
}

cyg_uint32
usbs_kinetis_isr (cyg_vector_t vector, cyg_addrword_t data)
{
    CYG_ASSERT(CYGNUM_HAL_INTERRUPT_USB0 == vector, "USB ISR should only be invoked for USB interrupts");

    cyg_drv_interrupt_mask(vector);

    return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
}

// ----------------------------------------------------------------------------
// Initialization
//
// This routine gets called from a prioritized static constructor during
// eCos startup.
// ----------------------------------------------------------------------------
// Initialization
//
// This routine gets called from a prioritized static constructor during
// eCos startup.
/*!
 * @brief Initialize the USB device KHCI instance.
 *
 * This function initizlizes the USB device KHCI module specified by the controllerId.
 *
 * @param controllerId The controller id of the USB IP. Please refer to enumeration type usb_controller_index_t.
 * @param handle        Pointer of the device handle, used to identify the device object is belonged to.
 * @param khciHandle   It is out parameter, is used to return pointer of the device KHCI handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
void usbs_kinetis_init (void)
{
	usb_device_handle handle;
	usb_device_khci_state_struct_t *khciState;

    CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_USBOTG);

    USB_DeviceInit(0,NULL,&handle);

    kinetis_usb_device = (usb_device_struct_t*)(handle);
    kinetis_usb_device->controlEndpoint = &ep0;

    khciState = (usb_device_khci_state_struct_t *)(kinetis_usb_device->controllerHandle);
#if ((defined FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED) && \
     (FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED > 0U))
    khciState->registerBase->CLK_RECOVER_IRC_EN = 3;
    khciState->registerBase->CLK_RECOVER_CTRL = 0x80;
#endif

    // TODO set interrupt priority properly
    cyg_drv_interrupt_create (CYGNUM_HAL_INTERRUPT_USB0,
                            0x00,
							(cyg_addrword_t)kinetis_usb_device,  // data
                            &usbs_kinetis_isr,
                            &usbs_kinetis_dsr,
                            &kinetis_usb_device->usbs_kinetis_intr_handle,
                            &kinetis_usb_device->usbs_kinetis_intr_data);

    cyg_drv_interrupt_attach (kinetis_usb_device->usbs_kinetis_intr_handle);

    cyg_drv_interrupt_acknowledge(CYGNUM_HAL_INTERRUPT_USB0);
    cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_USB0);

    USB_DeviceRun(kinetis_usb_device);
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
  CYG_ASSERT (endpoint == &ep0, "Wrong endpoint");
  if (CYG_ISR_CALL_DSR == usbs_kinetis_isr (CYGNUM_HAL_INTERRUPT_USB0, 0)) {
     usbs_kinetis_dsr (CYGNUM_HAL_INTERRUPT_USB0, 0, 0);
  }
}

#endif


