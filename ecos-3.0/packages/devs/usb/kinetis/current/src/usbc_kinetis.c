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



#define USB_DEVICE_MAX_EP 16

#define UDC_BSS(x)               __attribute__((__aligned__(x)))

#define USB_DEVICE_EP_CTRL_SIZE    8

#define CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS    4


//Kci configration macros
#define USB_DEVICE_CONFIG_LOW_POWER_MODE            1
#define USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING       1
#define USB_DEVICE_CONFIG_DETACH_ENABLE             1
#define FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED    0
#define USB_DEVICE_CHARGER_DETECT_ENABLE            0
#define FSL_FEATURE_SOC_USBDCD_COUNT                1
#define USB_DEVICE_CONFIG_REMOTE_WAKEUP             1
#define FSL_FEATURE_USB_KHCI_OTG_ENABLED            0
#define USB_DEVICE_CONFIG_OTG                       0
#define FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED     0
#define USB_DEVICE_CONFIG_KEEP_ALIVE_MODE           0
#define FSL_FEATURE_USB_KHCI_USB_RAM                0
#define FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED 1


// ecos Interrupt objects
static cyg_interrupt usbs_kinetis_intr_data;
static cyg_handle_t  usbs_kinetis_intr_handle;

static cyg_interrupt usbs_plug_intr_data;
static cyg_handle_t  usbs_plug_intr_handle;

// Driver API functions
static void usbs_kinetis_ep0_start(usbs_control_endpoint *);
static void usbs_kinetis_poll(usbs_control_endpoint *);

static void usbs_kinetis_endpoint_start(usbs_rx_endpoint * pep);
static void usbs_kinetis_endpoint_set_halted(usbs_rx_endpoint * pep,
                                          cyg_bool new_value);

static void
usbs_state_notify (usbs_control_endpoint * pcep);

//Endpoints control banks buffer
UDC_BSS(512) static uint8_t s_UsbDeviceKhciBdtBuffer[1][512U];

// Driver data sructrue
static usb_device_khci_state_struct_t kinetis_usb_device
{
    bdt:                        &s_UsbDeviceKhciBdtBuffer,
    registerBase:               USB0,
    dmaAlignBuffer:             NULL,
    isDmaAlignBufferInusing:    0,
    isResetting:                0,
    controllerId:               0,
};

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
                                                     int)) 0
};

// Endpoint 1 receive control structure
usbs_rx_endpoint ep1 = {
    start_rx_fn:    usbs_kinetis_endpoint_start,
    set_halted_fn:  usbs_kinetis_endpoint_set_halted,
    complete_fn:    (void (*)(void *, int)) 0,
    complete_data:  (void *) 0,
    buffer:         (unsigned char *) 0,
    buffer_size:    0,
    halted:         0,
};

// Endpoint 2 Receive control structure
usbs_rx_endpoint ep2 = {
    start_rx_fn:    usbs_kinetis_endpoint_start,
    set_halted_fn:  usbs_kinetis_endpoint_set_halted,
    complete_fn:    (void (*)(void *, int)) 0,
    complete_data:  (void *) 0,
    buffer:         (unsigned char *) 0,
    buffer_size:    0,
    halted:         0,
};

// Endpoint 3 Receive control structure
usbs_rx_endpoint ep3 = {
    start_rx_fn:    usbs_kinetis_endpoint_start,
    set_halted_fn:  usbs_kinetis_endpoint_set_halted,
    complete_fn:    (void (*)(void *, int)) 0,
    complete_data:  (void *) 0,
    buffer:         (unsigned char *) 0,
    buffer_size:    0,
    halted:         0,
};

#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 4)
// Endpoint 4 Receive control structure
usbs_rx_endpoint ep4 = {
    start_rx_fn:    usbs_kinetis_endpoint_start,
    set_halted_fn:  usbs_kinetis_endpoint_set_halted,
    complete_fn:    (void (*)(void *, int)) 0,
    complete_data:  (void *) 0,
    buffer:         (unsigned char *) 0,
    buffer_size:    0,
    halted:         0,
};
#endif

#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 5)
// Endpoint 5 Receive control structure
usbs_rx_endpoint ep5 = {
    start_rx_fn:    usbs_kinetis_endpoint_start,
    set_halted_fn:  usbs_kinetis_endpoint_set_halted,
    complete_fn:    (void (*)(void *, int)) 0,
    complete_data:  (void *) 0,
    buffer:         (unsigned char *) 0,
    buffer_size:    0,
    halted:         0,
};
#endif

#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 6)
// Endpoint 6 Receive control structure
usbs_rx_endpoint ep6 = {
    start_rx_fn:    usbs_kinetis_endpoint_start,
    set_halted_fn:  usbs_kinetis_endpoint_set_halted,
    complete_fn:    (void (*)(void *, int)) 0,
    complete_data:  (void *) 0,
    buffer:         (unsigned char *) 0,
    buffer_size:    0,
    halted:         0,
};
#endif

#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 7)
// Endpoint 7 Receive control structure
usbs_rx_endpoint ep7 = {
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
static const void *usbs_kinetis_endpoints[CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS] = {
  (void *) &ep0,
  (void *) &ep1,
  (void *) &ep2,
  (void *) &ep3
#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 4)
  ,(void *) &ep4, (void *) &ep5
#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 6)
  ,(void *) &ep6, (void *) &ep7
#endif
#endif
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

/******************************************************************************
 *   Mofidifed kci USB driver routines from NXP source.                       *
 ******************************************************************************/

/*!
 * @brief Write the BDT to start a transfer.
 *
 * The function is used to start a transfer by writing the BDT.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 * @param endpoint         Endpoint number.
 * @param direction        The direction of the endpoint, 0U - USB_OUT, 1U - USB_IN.
 * @param buffer           The memory address to save the received data, or the memory address to hold the data need to
 * be sent.
 * @param length           The length of the data.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceKhciEndpointTransfer(
        usb_device_khci_state_struct_t *khciState, uint8_t endpoint, 
        uint8_t direction, uint8_t *buffer, uint32_t length)
{
    uint32_t index = ((uint32_t)endpoint << 1U) | (uint32_t)direction;
    cyg_uint32 irq;

    /* Enter critical */
    HAL_DISABLE_INTERRUPTS(irq);

    /* Flag the endpoint is busy. */
    khciState->endpointState[index].stateUnion.stateBitField.transferring = 1U;

    /* Add the data buffer address to the BDT. */
    USB_KHCI_BDT_SET_ADDRESS((uint32_t)khciState->bdt, endpoint, direction,
                khciState->endpointState[index].stateUnion.stateBitField.bdtOdd, 
                (uint32_t)buffer);

    /* Change the BDT control field to start the transfer. */
    USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState->bdt, endpoint, direction,
            khciState->endpointState[index].stateUnion.stateBitField.bdtOdd,
            USB_LONG_TO_LITTLE_ENDIAN(USB_KHCI_BDT_BC(length) | 
                                      USB_KHCI_BDT_OWN        | 
                                      USB_KHCI_BDT_DTS        |
            USB_KHCI_BDT_DATA01(khciState->endpointState[index].stateUnion.stateBitField.data0)));

    /* Exit critical */
    HAL_RESTORE_INTERRUPTS(irq);

    /* Clear the token busy state */
    khciState->registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
    return kStatus_USB_Success;
}

/*!
 * @brief Prime a next setup transfer.
 *
 * The function is used to prime a buffer in control out pipe to wait for receiving the host's setup packet.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciPrimeNextSetup(usb_device_khci_state_struct_t *khciState)
{
/* Update the endpoint state */
/* Save the buffer address used to receive the setup packet. */
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
    /* In case of lowpower mode enabled, it requires to put the setup packet buffer(16 bytes) into the USB RAM so
     * that the setup packet would wake up the USB.
     */
    khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].transferBuffer =
        (uint8_t *)(khciState->bdt + 0x200U - 0x10U) +
        khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].stateUnion.stateBitField.bdtOdd *
            USB_SETUP_PACKET_SIZE;
#else
    khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].transferBuffer =
        (uint8_t *)&khciState->setupPacketBuffer[0] +
        khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].stateUnion.stateBitField.bdtOdd *
            USB_SETUP_PACKET_SIZE;
#endif
    /* Clear the transferred length. */
    khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].transferDone = 0U;
    /* Save the data length expected to get from a host. */
    khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].transferLength = USB_SETUP_PACKET_SIZE;
    /* Save the data buffer DMA align flag. */
    khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].stateUnion.stateBitField.dmaAlign = 1U;
    /* Set the DATA0/1 to DATA0. */
    khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].stateUnion.stateBitField.data0 = 0U;

    USB_DeviceKhciEndpointTransfer(khciState, USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL, USB_OUT,
                                   khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].transferBuffer,
                                   USB_SETUP_PACKET_SIZE);
}

/*!
 * @brief Set device controller state to default state.
 *
 * The function is used to set device controller state to default state.
 * The function will be called when USB_DeviceKhciInit called or the control type kUSB_DeviceControlGetEndpointStatus
 * received in USB_DeviceKhciControl.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciSetDefaultState(usb_device_khci_state_struct_t *khciState)
{
    uint8_t interruptFlag;

    /* Clear the error state register */
    khciState->registerBase->ERRSTAT = 0xFFU;

    /* Setting this bit to 1U resets all the BDT ODD ping/pong fields to 0U, which then specifies the EVEN BDT bank. */
    khciState->registerBase->CTL |= USB_CTL_ODDRST_MASK;

    /* Clear the device address */
    khciState->registerBase->ADDR = 0U;

    /* Clear the endpoint state and disable the endpoint */
    for (uint8_t count = 0U; count < CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS; count++)
    {
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState->bdt, count, USB_OUT, 0U, 0U);
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState->bdt, count, USB_OUT, 1U, 0U);
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState->bdt, count, USB_IN, 0U, 0U);
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState->bdt, count, USB_IN, 1U, 0U);

        khciState->endpointState[((uint32_t)count << 1U) | USB_OUT].stateUnion.state = 0U;
        khciState->endpointState[((uint32_t)count << 1U) | USB_IN].stateUnion.state = 0U;
        khciState->registerBase->ENDPOINT[count].ENDPT = 0x00U;
    }
    khciState->isDmaAlignBufferInusing = 0U;

    /* Clear the BDT odd reset flag */
    khciState->registerBase->CTL &= ~USB_CTL_ODDRST_MASK;

    /* Enable all error */
    khciState->registerBase->ERREN = 0xFFU;

    /* Enable reset, sof, token, stall interrupt */
    interruptFlag = kUSB_KhciInterruptReset
#if 0U
                    | kUSB_KhciInterruptSofToken
#endif
                    | kUSB_KhciInterruptTokenDone | kUSB_KhciInterruptStall;

#if (defined(USB_DEVICE_CONFIG_LOW_POWER_MODE) && (USB_DEVICE_CONFIG_LOW_POWER_MODE > 0U))
    /* Enable suspend interruprt */
    interruptFlag |= kUSB_KhciInterruptSleep;
#endif /* USB_DEVICE_CONFIG_LOW_POWER_MODE */

#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING) && (USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING > 0U)
    /* Enable error interruprt */
    interruptFlag |= kUSB_KhciInterruptError;
#endif /* USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING */
    /* Write the interrupt enable register */
    khciState->registerBase->INTEN = interruptFlag;

    /* Clear reset flag */
    khciState->isResetting = 0U;

    khciState->registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}

/*!
 * @brief Initialize a specified endpoint.
 *
 * The function is used to initialize a specified endpoint.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 * @param epInit          The endpoint initialization structure pointer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceKhciEndpointInit(usb_device_khci_state_struct_t *khciState,
                                               usb_device_endpoint_init_struct_t *epInit)
{
    uint16_t maxPacketSize = epInit->maxPacketSize;
    uint8_t endpoint = (epInit->endpointAddress & USB_ENDPOINT_NUMBER_MASK);
    uint8_t direction = (epInit->endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                        USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT;
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Make the endpoint max packet size align with USB Specification 2.0. */
    if (USB_ENDPOINT_DESCRIPTOR_ATTR_ISOCHRONOUS == epInit->transferType)
    {
        if (maxPacketSize > USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE)
        {
            maxPacketSize = USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE;
        }
    }
    else
    {
        if (maxPacketSize > USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE)
        {
            maxPacketSize = USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE;
        }
        /* Enable an endpoint to perform handshaking during a transaction to this endpoint. */
        khciState->registerBase->ENDPOINT[endpoint].ENDPT |= USB_ENDPT_EPHSHK_MASK;
    }
    /* Set the endpoint idle */
    khciState->endpointState[index].stateUnion.stateBitField.transferring = 0U;
    /* Save the max packet size of the endpoint */
    khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize = maxPacketSize;
    /* Set the data toggle to DATA0 */
    khciState->endpointState[index].stateUnion.stateBitField.data0 = 0U;
    /* Clear the endpoint stalled state */
    khciState->endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Set the ZLT field */
    khciState->endpointState[index].stateUnion.stateBitField.zlt = epInit->zlt;
    /* Enable the endpoint. */
    khciState->registerBase->ENDPOINT[endpoint].ENDPT |=
        (USB_IN == direction) ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;

    /* Prime a transfer to receive next setup packet when the endpoint is control out endpoint. */
    if ((USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == endpoint) && 
        (USB_ENDPOINT_DESCRIPTOR_ENDPOINT_OUT == direction))
    {
        USB_DeviceKhciPrimeNextSetup(khciState);
    }

    return kStatus_USB_Success;
}

/*!
 * @brief De-initialize a specified endpoint.
 *
 * The function is used to de-initialize a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be disabled.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceKhciEndpointDeinit(
        usb_device_khci_state_struct_t *khciState, uint8_t ep)
{
    uint8_t endpoint = (ep & USB_ENDPOINT_NUMBER_MASK);
    uint8_t direction =
        (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT;
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Cancel the transfer of the endpoint */
    USB_DeviceKhciCancel(khciState, ep);

    /* Disable the endpoint */
    khciState->registerBase->ENDPOINT[endpoint].ENDPT = 0x00U;
    /* Clear the max packet size */
    khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize = 0U;

    return kStatus_USB_Success;
}

/*!
 * @brief Stall a specified endpoint.
 *
 * The function is used to stall a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be stalled.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceKhciEndpointStall(
                usb_device_khci_state_struct_t *khciState, uint8_t ep)
{
    uint8_t endpoint = ep & USB_ENDPOINT_NUMBER_MASK;
    uint8_t direction =
        (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT;
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Cancel the transfer of the endpoint */
    USB_DeviceKhciCancel(khciState, ep);

    /* Set endpoint stall flag. */
    khciState->endpointState[index].stateUnion.stateBitField.stalled = 1U;

    /* Set endpoint stall in BDT. And then if the host send a IN/OUT tanscation, the device will response a STALL state.
     */
    USB_KHCI_BDT_SET_CONTROL(
        (uint32_t)khciState->bdt, endpoint, direction, khciState->endpointState[index].stateUnion.stateBitField.bdtOdd,
        USB_LONG_TO_LITTLE_ENDIAN(
            (uint32_t)(USB_KHCI_BDT_BC(khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize) |
                       USB_KHCI_BDT_DTS | USB_KHCI_BDT_STALL | USB_KHCI_BDT_OWN)));

    khciState->registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

    return kStatus_USB_Success;
}

/*!
 * @brief Un-stall a specified endpoint.
 *
 * The function is used to un-stall a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be un-stalled.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceKhciEndpointUnstall(
            usb_device_khci_state_struct_t *khciState, uint8_t ep)
{
    uint32_t control;
    uint8_t endpoint = ep & USB_ENDPOINT_NUMBER_MASK;
    uint8_t direction =
        (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT;
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Clear the endpoint stall state */
    khciState->endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Reset the endpoint data toggle to DATA0 */
    khciState->endpointState[index].stateUnion.stateBitField.data0 = 0U;

    /* Clear stall state in BDT */
    for (uint8_t i = 0U; i < 2U; i++)
    {
        control = USB_KHCI_BDT_GET_CONTROL((uint32_t)khciState->bdt, endpoint, direction, i);
        if (control & USB_KHCI_BDT_STALL)
        {
            USB_KHCI_BDT_SET_CONTROL(
                (uint32_t)khciState->bdt, endpoint, direction, i,
                USB_LONG_TO_LITTLE_ENDIAN(
                    (uint32_t)(USB_KHCI_BDT_BC(khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize) |
                               USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(0U))));
        }
    }

    /* Clear stall state in endpoint control register */
    khciState->registerBase->ENDPOINT[endpoint].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;

    /* Prime a transfer to receive next setup packet when the endpoint is a control out endpoint. */
    if ((USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == endpoint) && 
        (USB_ENDPOINT_DESCRIPTOR_ENDPOINT_OUT == direction))
    {
        USB_DeviceKhciPrimeNextSetup(khciState);
    }

    khciState->registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

    return kStatus_USB_Success;
}

/*!
 * @brief De-initialize the USB device KHCI instance.
 *
 * This function de-initizlizes the USB device KHCI module.
 *
 * @param khciHandle   Pointer of the device KHCI handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceKhciDeinit(usb_device_khci_state_struct_t *khciState)
{
    /* Clear all interrupt flags. */
    khciState->registerBase->ISTAT = 0xFFU;
    /* Disable all interrupts. */
    khciState->registerBase->INTEN &= ~(0xFFU);
    /* Clear device address. */
    khciState->registerBase->ADDR = (0U);

    /* Clear USB_CTL register */
    khciState->registerBase->CTL = 0x00U;
    khciState->registerBase->USBCTRL |= USB_USBCTRL_PDE_MASK | USB_USBCTRL_SUSP_MASK;

    return kStatus_USB_Success;
}

/*!
 * @brief Send data through a specified endpoint.
 *
 * This function sends data through a specified endpoint.
 *
 * @param khciHandle      Pointer of the device KHCI handle.
 * @param endpointAddress Endpoint index.
 * @param buffer           The memory address to hold the data need to be sent.
 * @param length           The data length need to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the sending request is successful or not; the transfer done is notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceKhciSend(usb_device_khci_state_struct_t *khciState,
                                uint8_t endpointAddress,
                                uint8_t *buffer,
                                uint32_t length)
{
    uint32_t index = ((endpointAddress & USB_ENDPOINT_NUMBER_MASK) << 1U) | USB_IN;
    usb_status_t error = kStatus_USB_Error;

    /* Save the tansfer information */
    if (0U == khciState->endpointState[index].stateUnion.stateBitField.transferring)
    {
        khciState->endpointState[index].transferDone = 0U;
        khciState->endpointState[index].transferBuffer = buffer;
        khciState->endpointState[index].transferLength = length;
        khciState->endpointState[index].stateUnion.stateBitField.dmaAlign = 1U;
    }

    /* Data length needs to less than max packet size in each call. */
    if (length > khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize)
    {
        length = khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize;
    }

    /* Send data when the device is not resetting. */
    if (0U == khciState->isResetting)
    {
        error = USB_DeviceKhciEndpointTransfer(khciState, endpointAddress & USB_ENDPOINT_NUMBER_MASK, USB_IN,
                                               (uint8_t *)((uint32_t)khciState->endpointState[index].transferBuffer +
                                                           (uint32_t)khciState->endpointState[index].transferDone),
                                               length);
    }

    /* Prime a transfer to receive next setup packet if the dat length is zero in a control in endpoint. */
    if ((0U == khciState->endpointState[index].transferDone) && (0U == length) &&
        (USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == (endpointAddress & USB_ENDPOINT_NUMBER_MASK)))
    {
        USB_DeviceKhciPrimeNextSetup(khciState);
    }
    return error;
}

/*!
 * @brief Receive data through a specified endpoint.
 *
 * This function Receives data through a specified endpoint.
 *
 * @param khciHandle      Pointer of the device KHCI handle.
 * @param endpointAddress Endpoint index.
 * @param buffer           The memory address to save the received data.
 * @param length           The data length want to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the receiving request is successful or not; the transfer done is notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceKhciRecv(usb_device_khci_state_struct_t *khciState,
                                uint8_t endpointAddress,
                                uint8_t *buffer,
                                uint32_t length)
{
    uint32_t index = ((endpointAddress & USB_ENDPOINT_NUMBER_MASK) << 1U) | USB_OUT;
    usb_status_t error = kStatus_USB_Error;

    if ((0U == length) && 
        (USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == (endpointAddress & USB_ENDPOINT_NUMBER_MASK)))
    {
        khciState->endpointState[index].stateUnion.stateBitField.transferring = 0U;
        USB_DeviceKhciPrimeNextSetup(khciState);
    }
    else
    {
        /* Save the tansfer information */
        if (0U == khciState->endpointState[index].stateUnion.stateBitField.transferring)
        {
            khciState->endpointState[index].transferDone = 0U;
            khciState->endpointState[index].transferBuffer = buffer;
            khciState->endpointState[index].transferLength = length;
        }
        khciState->endpointState[index].stateUnion.stateBitField.dmaAlign = 1U;

        /* Data length needs to less than max packet size in each call. */
        if (length > khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize)
        {
            length = khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize;
        }

        buffer = (uint8_t *)((uint32_t)buffer + (uint32_t)khciState->endpointState[index].transferDone);

        if ((khciState->dmaAlignBuffer) && (0U == khciState->isDmaAlignBufferInusing) &&
            (USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH >= length) &&
            ((length & 0x03U) || (((uint32_t)buffer) & 0x03U)))
        {
            khciState->endpointState[index].stateUnion.stateBitField.dmaAlign = 0U;
            buffer = khciState->dmaAlignBuffer;
            khciState->isDmaAlignBufferInusing = 1U;
        }

        /* Receive data when the device is not resetting. */
        if (0U == khciState->isResetting)
        {
            error = USB_DeviceKhciEndpointTransfer(khciState, endpointAddress & USB_ENDPOINT_NUMBER_MASK, USB_OUT,
                                                   buffer, length);
        }
    }
    return error;
}

/*!
 * @brief Cancel the pending transfer in a specified endpoint.
 *
 * The function is used to cancel the pending transfer in a specified endpoint.
 *
 * @param khciHandle      Pointer of the device KHCI handle.
 * @param ep               Endpoint address, bit7 is the direction of endpoint, 1U - IN, abd 0U - OUT.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceKhciCancel(usb_device_khci_state_struct_t *khciState, uint8_t ep)
{
    usbs_rx_endpoint * pep = (usbs_rx_endpoint*)
            usbs_uc3c_endpoints[(ep & USB_ENDPOINT_NUMBER_MASK)];
    uint8_t index = ((ep & USB_ENDPOINT_NUMBER_MASK) << 1U) | 
                    ((ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);

    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    if (khciState->endpointState[index].stateUnion.stateBitField.transferring)
    {
        if (pep->complete_fn) 
        {
            (*pep->complete_fn) (pep->complete_data, returncode);
        }
    }
    return kStatus_USB_Success;
}

/*!
 * @brief Control the status of the selected item.
 *
 * The function is used to control the status of the selected item.
 *
 * @param khciHandle      Pointer of the device KHCI handle.
 * @param type             The selected item. Please refer to enumeration type usb_device_control_type_t.
 * @param param            The param type is determined by the selected item.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceKhciControl(usb_device_khci_state_struct_t *khciState, usb_device_control_type_t type, void *param)
{
    uint16_t *temp16;
    uint8_t *temp8;
#if (defined(USB_DEVICE_CHARGER_DETECT_ENABLE) && (USB_DEVICE_CHARGER_DETECT_ENABLE > 0U)) && \
    (defined(FSL_FEATURE_SOC_USBDCD_COUNT) && (FSL_FEATURE_SOC_USBDCD_COUNT > 0U))
    usb_device_dcd_state_struct_t *dcdState;
    dcdState = &s_UsbDeviceDcdState[khciState->controllerId - kUSB_ControllerKhci0];
    usb_device_dcd_charging_time_t *deviceDcdTimingConfig = (usb_device_dcd_charging_time_t *)param;
#endif
#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
    usb_device_struct_t *deviceHandle;
    uint64_t startTick;
#endif
    usb_status_t error = kStatus_USB_Error;

#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
    deviceHandle = (usb_device_struct_t *)khciState->deviceHandle;
#endif

    switch (type)
    {
        case kUSB_DeviceControlRun:
            khciState->registerBase->USBCTRL = 0U;
#if defined(FSL_FEATURE_USB_KHCI_OTG_ENABLED) && (FSL_FEATURE_USB_KHCI_OTG_ENABLED > 0U)
            if (khciState->registerBase->OTGCTL & USB_OTGCTL_OTGEN_MASK)
            {
                khciState->registerBase->OTGCTL |= USB_OTGCTL_DPHIGH_MASK;
            }
#endif /* FSL_FEATURE_USB_KHCI_OTG_ENABLED */
            khciState->registerBase->CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
            khciState->registerBase->CTL |= USB_CTL_USBENSOFEN_MASK;

            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceControlStop:
#if defined(FSL_FEATURE_USB_KHCI_OTG_ENABLED) && (FSL_FEATURE_USB_KHCI_OTG_ENABLED > 0U)
            if (khciState->registerBase->OTGCTL & USB_OTGCTL_OTGEN_MASK)
            {
                khciState->registerBase->OTGCTL &= ~USB_OTGCTL_DPHIGH_MASK;
            }
#endif /* FSL_FEATURE_USB_KHCI_OTG_ENABLED */
            khciState->registerBase->CONTROL &= ~USB_CONTROL_DPPULLUPNONOTG_MASK;
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceControlEndpointInit:
            if (param)
            {
                error = USB_DeviceKhciEndpointInit(khciState, (usb_device_endpoint_init_struct_t *)param);
            }
            break;
        case kUSB_DeviceControlEndpointDeinit:
            if (param)
            {
                temp8 = (uint8_t *)param;
                error = USB_DeviceKhciEndpointDeinit(khciState, *temp8);
            }
            break;
        case kUSB_DeviceControlEndpointStall:
            if (param)
            {
                temp8 = (uint8_t *)param;
                error = USB_DeviceKhciEndpointStall(khciState, *temp8);
            }
            break;
        case kUSB_DeviceControlEndpointUnstall:
            if (param)
            {
                temp8 = (uint8_t *)param;
                error = USB_DeviceKhciEndpointUnstall(khciState, *temp8);
            }
            break;
        case kUSB_DeviceControlGetDeviceStatus:
            if (param)
            {
                temp16 = (uint16_t *)param;
                *temp16 = (USB_DEVICE_CONFIG_SELF_POWER << (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT))
#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
                          | ((uint16_t)(((uint32_t)deviceHandle->remotewakeup)
                                        << (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_REMOTE_WARKUP_SHIFT)))
#endif
                    ;
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceControlGetEndpointStatus:
            if (param)
            {
                usb_device_endpoint_status_struct_t *endpointStatus = (usb_device_endpoint_status_struct_t *)param;

                if (((endpointStatus->endpointAddress) & USB_ENDPOINT_NUMBER_MASK) < USB_DEVICE_CONFIG_ENDPOINTS)
                {
                    endpointStatus->endpointStatus =
                        (uint16_t)(
                            khciState
                                ->endpointState[(((endpointStatus->endpointAddress) & USB_ENDPOINT_NUMBER_MASK) << 1U) |
                                                (((endpointStatus->endpointAddress) &
                                                  USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                                                 USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)]
                                .stateUnion.stateBitField.stalled == 1U) ?
                            kUSB_DeviceEndpointStateStalled :
                            kUSB_DeviceEndpointStateIdle;
                    error = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceControlSetDeviceAddress:
            if (param)
            {
                temp8 = (uint8_t *)param;
                khciState->registerBase->ADDR = (*temp8);
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceControlGetSynchFrame:
            break;
#if ((defined(USB_DEVICE_CONFIG_LOW_POWER_MODE)) && (USB_DEVICE_CONFIG_LOW_POWER_MODE > 0U))
#if defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U)
        case kUSB_DeviceControlResume:
            khciState->registerBase->CTL |= USB_CTL_RESUME_MASK;
            startTick = deviceHandle->hwTick;
            while ((deviceHandle->hwTick - startTick) < 10)
            {
                __ASM("nop");
            }
            khciState->registerBase->CTL &= ~USB_CTL_RESUME_MASK;
            error = kStatus_USB_Success;
            break;
#endif /* USB_DEVICE_CONFIG_REMOTE_WAKEUP */
        case kUSB_DeviceControlSuspend:
            error = kStatus_USB_Success;
            break;
#endif /* USB_DEVICE_CONFIG_LOW_POWER_MODE */
        case kUSB_DeviceControlSetDefaultStatus:
            for (uint8_t count = 0U; count < CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS; count++)
            {
                USB_DeviceKhciEndpointDeinit(khciState, (count | (USB_IN << 0x07U)));
                USB_DeviceKhciEndpointDeinit(khciState, (count | (USB_OUT << 0x07U)));
            }
            USB_DeviceKhciSetDefaultState(khciState);
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceControlGetSpeed:
            if (param)
            {
                temp8 = (uint8_t *)param;
                *temp8 = USB_SPEED_FULL;
                error = kStatus_USB_Success;
            }
            break;
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        case kUSB_DeviceControlGetOtgStatus:
            *((uint8_t *)param) = khciState->otgStatus;
            break;
        case kUSB_DeviceControlSetOtgStatus:
            khciState->otgStatus = *((uint8_t *)param);
            break;
#endif
        case kUSB_DeviceControlSetTestMode:
            break;
#if (defined(USB_DEVICE_CHARGER_DETECT_ENABLE) && (USB_DEVICE_CHARGER_DETECT_ENABLE > 0U)) && \
    (defined(FSL_FEATURE_SOC_USBDCD_COUNT) && (FSL_FEATURE_SOC_USBDCD_COUNT > 0U))
        case kUSB_DeviceControlDcdInitModule:
            dcdState->dcdRegisterBase->CONTROL |= USBDCD_CONTROL_SR_MASK;
            dcdState->dcdRegisterBase->TIMER0 = USBDCD_TIMER0_TSEQ_INIT(deviceDcdTimingConfig->dcdSeqInitTime);
            dcdState->dcdRegisterBase->TIMER1 = USBDCD_TIMER1_TDCD_DBNC(deviceDcdTimingConfig->dcdDbncTime);
            dcdState->dcdRegisterBase->TIMER1 |= USBDCD_TIMER1_TVDPSRC_ON(deviceDcdTimingConfig->dcdDpSrcOnTime);
            dcdState->dcdRegisterBase->TIMER2_BC12 =
                USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD(deviceDcdTimingConfig->dcdTimeWaitAfterPrD);
            dcdState->dcdRegisterBase->TIMER2_BC12 |=
                USBDCD_TIMER2_BC12_TVDMSRC_ON(deviceDcdTimingConfig->dcdTimeDMSrcOn);
            dcdState->dcdRegisterBase->CONTROL |= USBDCD_CONTROL_IE_MASK;
            dcdState->dcdRegisterBase->CONTROL |= USBDCD_CONTROL_BC12_MASK;
            dcdState->dcdRegisterBase->CONTROL |= USBDCD_CONTROL_START_MASK;
            break;
        case kUSB_DeviceControlDcdDeinitModule:
            dcdState->dcdRegisterBase->CONTROL |= USBDCD_CONTROL_SR_MASK;
            break;
#endif

        default:
            break;
    }

    return error;
}

/******************************************************************************
 *   Device Inerrupt hangling routines.                                       *
 ******************************************************************************/

/*!
 * @brief Handle the token done interrupt.
 *
 * The function is used to handle the token done interrupt.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptTokenDone(usb_device_khci_state_struct_t *khciState)
{
    uint32_t control;
    uint32_t length;
    uint32_t remainingLength;
    uint8_t *bdtBuffer;
    cyg_uint8 *buffer;
    cyg_uint32 size;
    uint8_t endpoint;
    uint8_t direction;
    uint8_t bdtOdd;
    uint8_t isSetup;
    uint8_t index;
    uint8_t stateRegister = khciState->registerBase->STAT;

    /* Get the endpoint number to identify which one triggers the token done interrupt. */
    endpoint = (stateRegister & USB_STAT_ENDP_MASK) >> USB_STAT_ENDP_SHIFT;

    /* Get the direction of the endpoint number. */
    direction = (stateRegister & USB_STAT_TX_MASK) >> USB_STAT_TX_SHIFT;

    /* Get the finished BDT ODD. */
    bdtOdd = (stateRegister & USB_STAT_ODD_MASK) >> USB_STAT_ODD_SHIFT;

    /* Clear token done interrupt flag. */
    khciState->registerBase->ISTAT = kUSB_KhciInterruptTokenDone;

    /* Get the Control field of the BDT element according to the endpoint number, the direction and finished BDT ODD. */
    control = USB_KHCI_BDT_GET_CONTROL((uint32_t)khciState->bdt, endpoint, direction, bdtOdd);

    /* Get the buffer field of the BDT element according to the endpoint number, the direction and finished BDT ODD. */
    bdtBuffer = (uint8_t *)USB_KHCI_BDT_GET_ADDRESS((uint32_t)khciState->bdt, endpoint, direction, bdtOdd);

    /* Get the transferred length. */
    length = ((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 16U) & 0x3FFU;

    /* Get the transferred length. */
    isSetup = (USB_KHCI_BDT_DEVICE_SETUP_TOKEN == ((uint8_t)(((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 2U) & 0x0FU))) ?
                  1U :
                  0U;

    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    if (0U == khciState->endpointState[index].stateUnion.stateBitField.transferring)
    {
        return;
    }

    if (isSetup)
    {
        khciState->setupBufferIndex = bdtOdd;
    }

    /* USB_IN, Send completed */
    if (direction == USB_IN)
    {
        /* The transferred length */
        khciState->endpointState[index].transferDone += length;

        /* Remaining length */
        remainingLength = khciState->endpointState[index].transferLength - khciState->endpointState[index].transferDone;

        /* Change the data toggle flag */
        khciState->endpointState[index].stateUnion.stateBitField.data0 ^= 1U;
        /* Change the BDT odd toggle flag */
        khciState->endpointState[index].stateUnion.stateBitField.bdtOdd ^= 1U;

        /* Whether the transfer is completed or not. */
        /*
         * The transfer is completed when one of the following conditions meet:
         * 1. The remaining length is zero.
         * 2. The length of current transcation is less than the max packet size of the current pipe.
         */
        if ((0U == remainingLength) ||
            (khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize > length))
        {
            size = khciState->endpointState[index].transferDone;
            buffer = khciState->endpointState[index].transferBuffer;
            khciState->endpointState[index].stateUnion.stateBitField.transferring = 0U;

            /*
             * Whether need to send ZLT when the pipe is control in pipe and the transferred length of current
             * transaction equals to max packet size.
             */
            if ((length) && (!(length % khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize)))
            {
                if (USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == endpoint)
                {
                    usb_setup_struct_t *setup_packet =
                        (usb_setup_struct_t
                             *)(&khciState->setupPacketBuffer[(USB_SETUP_PACKET_SIZE * khciState->setupBufferIndex)]);
                    /*
                     * Send the ZLT and terminate the token done interrupt service when the tranferred length in data
                     * phase
                     * is less than the host request.
                     */
                    if (USB_SHORT_FROM_LITTLE_ENDIAN(setup_packet->wLength) >
                        khciState->endpointState[index].transferLength)
                    {
                        (void)USB_DeviceKhciEndpointTransfer(khciState, endpoint, USB_IN, (uint8_t *)NULL, 0U);
                        return;
                    }
                }
                else if (khciState->endpointState[index].stateUnion.stateBitField.zlt)
                {
                    (void)USB_DeviceKhciEndpointTransfer(khciState, endpoint, USB_IN, (uint8_t *)NULL, 0U);
                    return;
                }
                else
                {
                }
            }
        }
        else
        {
            /* Send remaining data and terminate the token done interrupt service. */
            (void)USB_DeviceKhciSend(khciState, endpoint | (USB_IN << 0x07U),
                                     khciState->endpointState[index].transferBuffer, remainingLength);
            return;
        }
    }
    else
    {
        if ((USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == endpoint) && (0U == length))
        {
            /*message.length = 0U;
            message.buffer = (uint8_t *)NULL;*/
            size = 0;
            buffer = (cyg_uint8*)NULL;
            //Nevím co tady
        }
        else
        {
            if (0U == khciState->endpointState[index].stateUnion.stateBitField.dmaAlign)
            {
                uint8_t *buffer = (uint8_t *)USB_LONG_FROM_LITTLE_ENDIAN(
                    USB_KHCI_BDT_GET_ADDRESS((uint32_t)khciState->bdt, endpoint, USB_OUT,
                                             khciState->endpointState[index].stateUnion.stateBitField.bdtOdd));
                uint8_t *transferBuffer =
                    khciState->endpointState[index].transferBuffer + khciState->endpointState[index].transferDone;
                if (buffer != transferBuffer)
                {
                    for (uint32_t i = 0U; i < length; i++)
                    {
                        transferBuffer[i] = buffer[i];
                    }
                }
                khciState->isDmaAlignBufferInusing = 0U;
            }
            /* The transferred length */
            khciState->endpointState[index].transferDone += length;
            /* Remaining length */
            remainingLength =
                khciState->endpointState[index].transferLength - khciState->endpointState[index].transferDone;

            if ((USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL == endpoint) && isSetup)
            {
                khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].stateUnion.stateBitField.data0 = 1U;
                khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_IN].stateUnion.stateBitField.data0 = 1U;
            }
            else
            {
                khciState->endpointState[index].stateUnion.stateBitField.data0 ^= 1U;
            }
            khciState->endpointState[index].stateUnion.stateBitField.bdtOdd ^= 1U;
            if ((!khciState->endpointState[index].transferLength) || (!remainingLength) ||
                (khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize > length))
            {
                size = khciState->endpointState[index].transferDone;
                if (isSetup)
                {
                    static ep0_low_level_status_t status = UDD_EPCTRL_SETUP;
                    ep0.buffer_size = 0;
                    ep0.fill_buffer_fn = 0;
                    ep0.complete_fn = 0;
                    memcpy(ep0.control_buffer, bdtBuffer, 8);
                    usbs_kinetis_control_setup(status);
                    //message.buffer = bdtBuffer;
                }
                else
                {
                    buffer = khciState->endpointState[index].transferBuffer;
                }
                khciState->endpointState[index].stateUnion.stateBitField.transferring = 0U;
            }
            else
            {
                /* Receive remaining data and terminate the token done interrupt service. */
                USB_DeviceKhciRecv(khciState, (endpoint) | (USB_OUT << 0x07U),
                                   khciState->endpointState[index].transferBuffer, remainingLength);
                return;
            }
        }
    }

    if(endpoint == 0)
    {
        if (direction == USB_OUT)
        {
            // Data was expected and received. Transfer the data to the
            // user's buffer, and perform completion.
            usbs_control_return result;
            
            CYG_ASSERT( (usbs_control_return (*)(usbs_control_endpoint*, int))0 != ep0.common.complete_fn, \
                        "A completion function should be provided for OUT control messages");
            CYG_ASSERT(size == ep0.buffer_size, "Inconsistency between buffer and transfer sizes");
            memcpy(ep0.buffer, buffer, size);
            result = (*ep0.complete_fn)(&ep0, 0);
            ep0.buffer           = (unsigned char*) 0;
            ep0.buffer_size      = 0;
            ep0.complete_fn      = (usbs_control_return (*)(usbs_control_endpoint*, int)) 0;
        }
        else
        {
            CYG_ASSERT("IN Token for ep0 is unexpected.\n");
        }
    }
    else
    {
        usbs_rx_endpoint *pep = (usbs_rx_endpoint *) usbs_kinetis_endpoints[epn];

        CYG_ASSERT (UC3C_USB_ENDPOINTS > epn && epn, "Invalid end point");
        CYG_ASSERT (pep->complete_fn, "No complete_fn()");

        if (pep->complete_fn)
        {
            /* Do not check on pep->buffer_size != 0, user should
            * be allowed to send empty packet */
            // TODO: nevim jestli neni buffer modifikovany
            if (!pep->halted) 
            {
                (*pep->complete_fn) (buffer,size);
            } 
            else 
            {
                (*pep->complete_fn) (buffer, -EAGAIN);
            }
        }
    }
#if 0
    message.isSetup = isSetup;
    message.code = (endpoint) | (uint8_t)(((uint32_t)direction << 0x07U));

    /* Notify the up layer the KHCI status changed. */
    USB_DeviceNotificationTrigger(khciState->deviceHandle, &message);
#endif

    khciState->registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}

/*!
 * @brief Handle the USB bus reset interrupt.
 *
 * The function is used to handle the USB bus reset interrupt.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptReset(usb_device_khci_state_struct_t *khciState)
{
    /* Set KHCI reset flag */
    khciState->isResetting = 1U;

    /* Clear the reset interrupt */
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptReset);
#if ((defined(USB_DEVICE_CONFIG_LOW_POWER_MODE)) && (USB_DEVICE_CONFIG_LOW_POWER_MODE > 0U))
    /* Clear the suspend interrupt */
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptSleep);
    khciState->registerBase->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
#endif

    /* Notify up layer the USB bus reset signal detected. */
    ep0.state = USBS_STATE_DEFAULT;
    usbs_state_notify(&ep0);
}

/* The USB suspend and resume signals need to be detected and handled when the 
 * low power or remote wakeup function
 * enabled. */
#if (defined(USB_DEVICE_CONFIG_LOW_POWER_MODE) && (USB_DEVICE_CONFIG_LOW_POWER_MODE > 0U))

/*!
 * @brief Handle the suspend interrupt.
 *
 * The function is used to handle the suspend interrupt when the suspend signal 
 * detected.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptSleep(usb_device_khci_state_struct_t *khciState)
{
    /* Enable the resume interrupt */
    khciState->registerBase->INTEN |= kUSB_KhciInterruptResume;
    khciState->registerBase->USBTRC0 |= USB_USBTRC0_USBRESMEN_MASK;
    khciState->registerBase->USBCTRL |= USB_USBCTRL_SUSP_MASK;
    /* Disable the suspend interrupt */
    khciState->registerBase->INTEN &= ~((uint32_t)kUSB_KhciInterruptSleep);

    /* Clear the suspend interrupt */
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptSleep);
    /* Clear the resume interrupt */
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptResume);

    /* Notify up layer the USB suspend signal detected. */
    ep0.state = ep0.state | USBS_STATE_SUSPENDED;
    usbs_state_notify (&ep0);
}

/*!
 * @brief Handle the resume interrupt.
 *
 * The function is used to handle the resume interrupt when the resume signal detected.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptResume(usb_device_khci_state_struct_t *khciState)
{
    khciState->registerBase->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
    /* Enable the suspend interrupt */
    khciState->registerBase->INTEN |= kUSB_KhciInterruptSleep;
    /* Disable the resume interrupt */
    khciState->registerBase->INTEN &= ~((uint32_t)kUSB_KhciInterruptResume);
    khciState->registerBase->USBTRC0 &= ~USB_USBTRC0_USBRESMEN_MASK;

    /* Clear the resume interrupt */
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptResume);
    /* Clear the suspend interrupt */
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptSleep);

    /* Notify up layer the USB resume signal detected. */
    ep0.state = USBS_STATE_DEFAULT;
    usbs_state_notify (&ep0);
}
#endif /* USB_DEVICE_CONFIG_LOW_POWER_MODE */

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U)) && \
    (defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED) && (FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED > 0U))
/*!
 * @brief Handle the VBUS rising interrupt.
 *
 * The function is used to handle the VBUS rising interrupt when the VBUS rising signal detected.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptVbusRising(usb_device_khci_state_struct_t *khciState)
{
    /* Disable the VBUS rising interrupt */
    khciState->registerBase->MISCCTRL &= ~USB_MISCCTRL_VREDG_EN_MASK;
    /* Enable the VBUS rising interrupt */
    khciState->registerBase->MISCCTRL |= USB_MISCCTRL_VREDG_EN_MASK;

    /* Notify up layer the USB VBUS rising signal detected. */
    ep0.state = USBS_STATE_ATTACHED;
    usbs_state_notify (&ep0);
}

/*!
 * @brief Handle the VBUS falling interrupt.
 *
 * The function is used to handle the VBUS falling interrupt when the VBUS falling signal detected.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptVbusFalling(usb_device_khci_state_struct_t *khciState)
{
    /* Disable the VBUS rising interrupt */
    khciState->registerBase->MISCCTRL &= ~USB_MISCCTRL_VFEDG_EN_MASK;
    /* Enable the VBUS rising interrupt */
    khciState->registerBase->MISCCTRL |= USB_MISCCTRL_VFEDG_EN_MASK;

    /* Notify up layer the USB VBUS falling signal detected. */
    ep0.state = USBS_STATE_DETACHED;
    usbs_state_notify (&ep0);
}
#endif /* USB_DEVICE_CONFIG_DETACH_ENABLE || FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED */

#if 0U
/*!
 * @brief Handle the sof interrupt.
 *
 * The function is used to handle the sof interrupt.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
void USB_DeviceKhciInterruptSof(usb_device_khci_state_struct_t *khciState)
{
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptSofToken);

    khciState->registerBase->ISTAT = (kUSB_KhciInterruptResume);
}
#endif

/*!
 * @brief Handle endpoint stalled interrupt.
 *
 * The function is used to handle  endpoint stalled interrupt.
 *
 * @param khciState       Pointer of the device KHCI state structure.
 *
 */
static void USB_DeviceKhciInterruptStall(usb_device_khci_state_struct_t *khciState)
{
    /* Clear the endpoint stalled interrupt flag */
    while (khciState->registerBase->ISTAT & (kUSB_KhciInterruptStall))
    {
        khciState->registerBase->ISTAT = (kUSB_KhciInterruptStall);
    }

    /* Un-stall the control in and out pipe when the control in or out pipe stalled. */
    if ((khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_IN].stateUnion.stateBitField.stalled) ||
        (khciState->endpointState[(USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL << 1U) | USB_OUT].stateUnion.stateBitField.stalled))
    {
        USB_DeviceKhciEndpointUnstall(
            khciState, (USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)));
        USB_DeviceKhciEndpointUnstall(
            khciState, (USB_ENDPOINT_DESCRIPTOR_ATTR_CONTROL | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)));
    }
}

#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING) && (USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING > 0U)
static void USB_DeviceKhciInterruptError(usb_device_khci_state_struct_t *khciState)
{
    khciState->registerBase->ISTAT = (kUSB_KhciInterruptError);

    CYG_ASSERT("USB Error Interrupt. Halted!!!\n");
    /* Notify up layer the USB error detected. */
    /* There is no appropriate message to high level driver*/
    ep0.state = USBS_STATE_DETACHED;
    usbs_state_notify (&ep0);
}
#endif /* USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING */
/*!
 * @brief Handle the KHCI device interrupt.
 *
 * The function is used to handle the KHCI device interrupt.
 *
 * @param deviceHandle    The device handle got from USB_DeviceInit.
 *
 */
cyg_uint32
usbs_kinetis_isr (cyg_vector_t vector, cyg_addrword_t data)
{
    usb_device_khci_state_struct_t *khciState;
    CYG_ASSERT(CYGNUM_HAL_INTERRUPT_USB0 == vector, "USB ISR should only be invoked for USB interrupts");
    CYG_ASSERT(0 == data, "The Kinetis ISR needs no global data pointer");
    
    khciState = (usb_device_khci_state_struct_t *)data;
    
    intr_status = khciState->registerBase->ISTAT;
    
    cyg_drv_interrupt_acknowledge(vector);
    return ((0 == intr_status)) ?
        CYG_ISR_HANDLED : CYG_ISR_CALL_DSR;
}

 void
usbs_kinetis_dsr (cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    uint8_t status;
    usb_device_khci_state_struct_t *khciState;
    CYG_ASSERT(CYGNUM_HAL_INTERRUPT_USB0 == vector, "USB ISR should only be invoked for USB interrupts");
    CYG_ASSERT(0 == data, "The Kinetis ISR needs no global data pointer");
    
    khciState = (usb_device_khci_state_struct_t *)data;

    status = intr_status;

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
    /* Clear EEP_ALIVE_CTRL_WAKE_INT interrupt state */
    if (khciState->registerBase->KEEP_ALIVE_CTRL & USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_MASK)
    {
        khciState->registerBase->KEEP_ALIVE_CTRL |= USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_MASK;
    }
    /* Clear SOFTOK interrupt state */
    if (khciState->registerBase->ISTAT & USB_ISTAT_SOFTOK_MASK)
    {
        khciState->registerBase->ISTAT = USB_ISTAT_SOFTOK_MASK;
    }
#endif
#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING) && (USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING > 0U)
    /* Error interrupt */
    if (status & kUSB_KhciInterruptError)
    {
        USB_DeviceKhciInterruptError(khciState);
    }
#endif /* USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING */
    /* Token done interrupt */
    if (status & kUSB_KhciInterruptTokenDone)
    {
        USB_DeviceKhciInterruptTokenDone(khciState);
    }

    /* Reset interrupt */
    if (status & kUSB_KhciInterruptReset)
    {
        USB_DeviceKhciInterruptReset(khciState);
    }

#if (defined(USB_DEVICE_CONFIG_LOW_POWER_MODE) && (USB_DEVICE_CONFIG_LOW_POWER_MODE > 0U))
    /* Suspend interrupt */
    if (status & kUSB_KhciInterruptSleep)
    {
        USB_DeviceKhciInterruptSleep(khciState);
    }

    /* Resume interrupt */
    if (status & kUSB_KhciInterruptResume)
    {
        USB_DeviceKhciInterruptResume(khciState);
    }

    if (khciState->registerBase->USBTRC0 & USB_USBTRC0_USB_RESUME_INT_MASK)
    {
        USB_DeviceKhciInterruptResume(khciState);
    }
#endif /* USB_DEVICE_CONFIG_LOW_POWER_MODE */

    /* Endpoint stalled interrupt */
    if (status & kUSB_KhciInterruptStall)
    {
        USB_DeviceKhciInterruptStall(khciState);
    }

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U)) && \
    (defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED) && (FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED > 0U))
    if (khciState->registerBase->USBTRC0 & USB_USBTRC0_VREDG_DET_MASK)
    {
        USB_DeviceKhciInterruptVbusRising(khciState);
    }

    if (khciState->registerBase->USBTRC0 & USB_USBTRC0_VFEDG_DET_MASK)
    {
        USB_DeviceKhciInterruptVbusFalling(khciState);
    }
#endif /* USB_DEVICE_CONFIG_DETACH_ENABLE && FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED */

#if 0U
    /* Sof token interrupt */
    if (status & kUSB_KhciInterruptSofToken)
    {
        USB_DeviceKhciInterruptSof(khciState);
    }
#endif

#if ((defined FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED) && \
     (FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED > 0U))
    status = khciState->registerBase->CLK_RECOVER_INT_STATUS;
    if (status)
    {
        /* USB RECOVER interrupt is happenned */
        if (USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK & status)
        {
            /* Indicates that the USB clock recovery algorithm has detected that the frequency trim adjustment needed
             * for the IRC48M output clock is outside the available TRIM_FINE adjustment range for the IRC48M
             * module.
             */
            CYG_ASSERT("IRC48M Error.\n");
        }
        khciState->registerBase->CLK_RECOVER_INT_STATUS = status;
    }
#endif
}

 /******************************************************************************
 *   Device API routines.                                                      *
 ******************************************************************************/
 // Start endpoint reception/trasmitrion
void
usbs_kinetis_endpoint_start (usbs_rx_endpoint * pep)
{
    const usb_endpoint_descriptor *usb_endpoints;
    int epn = usbs_kinetis_pep_to_number(pep);
    usb_endpoints = ep0.enumeration_data->endpoints;
    cyg_uint8 epAddress = usb_endpoints[epn].endpoint;

    CYG_ASSERT (pep->complete_fn, "No complete_fn()");

    cyg_drv_dsr_lock ();
    if (ep0.common.state != USBS_STATE_CONFIGURED) {
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
    if((epAddress&0x80) == USB_ENDPOINT_DESCRIPTOR_ENDPOINT_IN)
    {
        USB_DeviceKhciSend(&kinetis_usb_device,epAddress,pep->buffer,pep->buffer_size);
    }
    else if((epAddress&0x80) == USB_ENDPOINT_DESCRIPTOR_ENDPOINT_OUT)
    {
        USB_DeviceKhciRecv(&kinetis_usb_device,epAddress,pep->buffer,pep->buffer_size);
    }
    else
    {
        if (pep->complete_fn) {
            (*pep->complete_fn) (pep->complete_data, -EIO);
        }
    }

    cyg_drv_dsr_unlock ();
}

// Stall or UnStall specifed endpoint
void
usbs_kinetis_endpoint_set_halted (usbs_rx_endpoint * pep, cyg_bool new_value)
{
    const usb_endpoint_descriptor *usb_endpoints;
    int epn = usbs_kinetis_pep_to_number(pep);
    usb_endpoints = ep0.enumeration_data->endpoints;
    cyg_uint8 epAddress = usb_endpoints[epn].endpoint;
    
    usb_endpoints = ep0.enumeration_data->endpoints;

    cyg_drv_dsr_lock ();

    if (pep->halted != new_value) {
        /* There is something is to do */
        pep->halted = new_value;

        if ( new_value ) 
        {
            /* Halt endpoint */
            // TODO: je to blbe protoze funkce pozaduje i informaci o smeru
            USB_DeviceKhciEndpointStall(&kinetis_usb_device,epAddress);

        } else 
        {
            // TODO: je to blbe protoze funkce pozaduje i informaci o smer
            USB_DeviceKhciEndpointUnstall(&kinetis_usb_device,epAddress);
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
        
        initStr.maxPacketSize = ((cyg_uint32)usb_endpoints[epn].max_packet_lo |
                            ((cyg_uint32)usb_endpoints[epn].max_packet_hi << 8));
    
        initStr.endpointAddress = usb_endpoints[epn].endpoint;
        initStr.transferType    = usb_endpoints[epn].attributes;
        
        USB_DeviceKhciControl(&kinetis_usb_device,
                kUSB_DeviceControlEndpointInit,&initStr);
    }
}

// Start endpoint 0
void
usbs_kinetis_ep0_start (usbs_control_endpoint * endpoint)
{

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
    usb_device_khci_state_struct_t *khciState = &kinetis_usb_device;

    /* Clear all interrupt flags. */
    khciState->registerBase->ISTAT = 0xFFU;

#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
    khciState->otgStatus = 0U;
#else
    /* Disable the device functionality. */
    USB_DeviceKhciControl(khciState, kUSB_DeviceControlStop, NULL);
#endif

    /* Set BDT buffer address */
    khciState->registerBase->BDTPAGE1 = (uint8_t)((((uint32_t)khciState->bdt) >> 8U) & 0xFFU);
    khciState->registerBase->BDTPAGE2 = (uint8_t)((((uint32_t)khciState->bdt) >> 16U) & 0xFFU);
    khciState->registerBase->BDTPAGE3 = (uint8_t)((((uint32_t)khciState->bdt) >> 24U) & 0xFFU);

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U)) && \
    (defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED) && (FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED > 0U))
    khciState->registerBase->MISCCTRL |= USB_MISCCTRL_VREDG_EN_MASK | USB_MISCCTRL_VFEDG_EN_MASK;
#endif

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
    khciState->registerBase->CLK_RECOVER_CTRL |= USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK;
    khciState->registerBase->KEEP_ALIVE_CTRL =
        USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN_MASK | USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN_MASK |
        USB_KEEP_ALIVE_CTRL_WAKE_INT_EN_MASK | FSL_FEATURE_USB_KHCI_KEEP_ALIVE_MODE_CONTROL;
    /* wake on out and setup transaction */
    khciState->registerBase->KEEP_ALIVE_WKCTRL = 0x1U;
#if defined(FSL_FEATURE_SOC_MCGLITE_COUNT) && (FSL_FEATURE_SOC_MCGLITE_COUNT > 0U)
    MCG->MC |= MCG_MC_HIRCLPEN_MASK;
#endif
    PMC->REGSC |= PMC_REGSC_BGEN_MASK | PMC_REGSC_VLPO_MASK;
#endif
    /* Set KHCI device state to default value. */
    USB_DeviceKhciSetDefaultState(khciState);
    
    usbs_ep_alloc();

    cyg_drv_interrupt_create (CYGNUM_HAL_INTERRUPT_USB0,
                            CYGNUM_DEVS_USB_ISR_PRIO,  
                            khciState,  // data
                            &usbs_kinetis_isr,
                            &usbs_kinetis_dsr,
                            &usbs_kinetis_intr_handle, 
                            &usbs_kinetis_intr_data);

    cyg_drv_interrupt_attach (usbs_kinetis_intr_handle);
    
    usbs_kinetis_ep0.state = USBS_STATE_POWERED;
    usbs_state_notify (&usbs_kinetis_ep0);
}

// There has been a change in state. Update the end point.
static void
usbs_state_notify (usbs_control_endpoint * pcep)
{
    static int old_state = USBS_STATE_CHANGE_POWERED;
    int state = pcep->state & USBS_STATE_MASK;

    if (pcep->state != old_state) {
        for(int i = 0; i < CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS; i++)
            USB_DeviceKhciCancel(&kinetis_usb_device, i);
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

/******************************************************************************
 *   Functions to process setup packet.                                       *
 ******************************************************************************/

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
            (req->index_lo < CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS)) {

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
            (epn < CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS)) 
        {
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
            (epn < CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS)) {
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
    // Data are in control_buffer
    usb_devreq *req = (usb_devreq *) ep0.control_buffer;
    cyg_uint8   protocol;
    cyg_uint16 length;
    cyg_bool dev_to_host;
    usbs_control_return usbcode;
    cyg_bool handled = false;
    
    length = (req->length_hi << 8) | req->length_lo;
    dev_to_host = req->type & USB_DEVREQ_DIRECTION_IN;

    CYG_TRACE0( true, "Control Setup\n" );

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
                USB_DeviceKhciControl(&kinetis_usb_device,
                        kUSB_DeviceControlSetDeviceAddress,&usb_address);
                ep0.state = USBS_STATE_ADDRESSED;
                usbs_sate_noftify(&ep0);
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
        // TODO: Neni to asi idealni
        usbcode = usbs_parse_host_get_command (&usbs_kinetis_ep0);
        usbs_kinetis_ep0.buffer_size = MIN (ep0.buffer_size, length);
        if (usbcode == USBS_CONTROL_RETURN_HANDLED)
        {
            handled = true;
            if(usbs_kinetis_ep0.fill_buffer_fn)
            {
                cyg_uint32 pos = 0;
                memcpy(kinetis_usb_device.setupPacketBuffer,
                       ep0.buffer,ep0.buffer_size);
                pos += ep0.buffer_size;
                while(ep0.fill_buffer_fn)
                {
                    if((ep0.buffer_size + pos) < USB_SETUP_PACKET_SIZE)
                    {
                        (*ep0.fill_buffer_fn) (&rp0);
                        memcpy(kinetis_usb_device.setupPacketBuffer + pos,
                               ep0.buffer,ep0.buffer_size);
                        pos += ep0.buffer_size;
                    }
                    else
                    {
                        CYG_ASSERT(false,"USB control buffer overflow\n");
                    }					
                }		

                usbs_kinetis_ep0.buffer = kinetis_usb_device.setupPacketBuffer;
                usbs_kinetis_ep0.buffer_size = pos;
                USB_DeviceKhciEndpointTransfer(&kinetis_usb_device,0,USB_IN,
                        usbs_kinetis_ep0.buffer, usbs_kinetis_ep0.buffer_size);
            }			

        }
        else
            handled = false;
    }
    
    if (status == UDD_EPCTRL_STALL_REQ)
    {
        cyg_uint8 epn = 0;
        USB_DeviceKhciControl(&kinetis_usb_device,
                        kUSB_DeviceControlEndpointStall,&epn);
    }
      
    return status;
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
     usbs_kinetis_dsr (CYGNUM_HAL_VECTOR_USB, 0, 0);
  }
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



