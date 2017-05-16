//==========================================================================
//
//      usbs_serial.c
//
//      Support for slave-side USB serial devices.
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2008, 2010 Free Software Foundation, Inc.                        
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
// Author(s):    Frank M. Pagliughi (fmp), SoRo Systems, Inc.
// Contributors: jld
// Date:         2008-06-02
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/cyg_trac.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/drv_api.h>
#include <cyg/kernel/kapi.h>

#include <pkgconf/io_usb_slave_winusb_serial.h>
#include <cyg/io/usb/usbs_serial_winusb.h>
#include <string.h>

#if defined(CYGBLD_IO_USB_WINUSB_SLAVE_SERIAL_DEBUG)
#define DBG diag_printf
#else
#define DBG (1) ? (void)0 : diag_printf
#endif

#define EP0_MAX_PACKET_SIZE     CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_EP0_MAX_PACKET_SIZE

extern usbs_control_endpoint    CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_EP0;

#if defined(CYGPKG_IO_USB_WINUSB_SLAVE_SERIAL_STATIC_EP)
extern usbs_tx_endpoint         CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_TX_EP;
extern usbs_rx_endpoint         CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_RX_EP;
#define TX_EP                   (&CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_TX_EP)
#define RX_EP                   (&CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_RX_EP)
#define INTR_EP                 (&CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_INTR_EP)
#endif

#define TX_EP_NUM               CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_TX_EP_NUM
#define RX_EP_NUM               CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_RX_EP_NUM
#define INTR_EP_NUM             CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_INTR_EP_NUM
#define EP0                     (&CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_EP0)


#define VENDOR_ID               CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_VENDOR_ID
#define PRODUCT_ID              CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_PRODUCT_ID

//#define USB_MAX_STR_LEN         256
#define USB_MAX_STR_LEN         64

#define LO_BYTE_16(word16)      ((cyg_uint8) ((word16) & 0xFF))
#define HI_BYTE_16(word16)      ((cyg_uint8) (((word16) >> 8) & 0xFF))

#define BYTE0_32(word32)        ((cyg_uint8) ((word32) & 0xFF))
#define BYTE1_32(word32)        ((cyg_uint8) (((word32) >>  8) & 0xFF))
#define BYTE2_32(word32)        ((cyg_uint8) (((word32) >> 16) & 0xFF))
#define BYTE3_32(word32)        ((cyg_uint8) (((word32) >> 24) & 0xFF))


#define MFG_STR_INDEX           '\x01'
#define PRODUCT_STR_INDEX       '\x02'
#define SN_STR_INDEX       	'\x03'

#define USB_CONFIGURATION_DESCRIPTOR_TOTAL_LENGTH(interfaces, endpoints) \
            (USB_CONFIGURATION_DESCRIPTOR_LENGTH +            \
            ((interfaces) * USB_INTERFACE_DESCRIPTOR_LENGTH) +  \
            ((endpoints)  * USB_ENDPOINT_DESCRIPTOR_LENGTH))



#define USBS_SERIAL_DEVICE_CLASS        0
#define USBS_SERIAL_NUM_IFACE           1
#define USBS_SERIAL_NUM_ENDP            2
#define USBS_SERIAL_DATA_IFACE_CLASS    0    // Vendor




// ----- Configuration Descriptor -----

static const usb_configuration_descriptor usb_configuration = {
    length:             sizeof(usb_configuration_descriptor),
    type:               USB_CONFIGURATION_DESCRIPTOR_TYPE,
    total_length_lo:    
        USB_CONFIGURATION_DESCRIPTOR_TOTAL_LENGTH_LO(USBS_SERIAL_NUM_IFACE, 
                                                     USBS_SERIAL_NUM_ENDP),
    total_length_hi:    
        USB_CONFIGURATION_DESCRIPTOR_TOTAL_LENGTH_HI(USBS_SERIAL_NUM_IFACE, 
                                                     USBS_SERIAL_NUM_ENDP),
    number_interfaces:  USBS_SERIAL_NUM_IFACE,
    configuration_id:   1,
    configuration_str:  0,
#ifdef CYGOPT_IO_USB_WINUSB_SLAVE_SERIAL_BUSPOWERED
    attributes:         (USB_CONFIGURATION_DESCRIPTOR_ATTR_REQUIRED),
#else
    attributes:         (USB_CONFIGURATION_DESCRIPTOR_ATTR_REQUIRED |
                         USB_CONFIGURATION_DESCRIPTOR_ATTR_SELF_POWERED),
#endif
    max_power:          (CYGNUM_IO_USB_WINUSB_SLAVE_SERIAL_CURRENTDRAW+1)/2	
};

// ----- Interface Descriptor -----

static const usb_interface_descriptor usb_interface[] = {

    {
        length:             sizeof(usb_interface_descriptor),
        type:               USB_INTERFACE_DESCRIPTOR_TYPE,
        interface_id:       0,
        alternate_setting:  0,
        number_endpoints:   2,
        interface_class:    USBS_SERIAL_DATA_IFACE_CLASS,
        interface_subclass: 0x00,
        interface_protocol: 0x00,
        interface_str:      0x00
    }
};

// ----- Endpoint Descriptors -----

static const usb_endpoint_descriptor usb_endpoints[] =
{ 
    // Tx (Bulk IN) Endpoint Descriptor
    {
        sizeof(usb_endpoint_descriptor),
        USB_ENDPOINT_DESCRIPTOR_TYPE,
        USB_ENDPOINT_DESCRIPTOR_ENDPOINT_IN | TX_EP_NUM,
        USB_ENDPOINT_DESCRIPTOR_ATTR_BULK,
        0x40,
        0,
        0
    },

    // Rx (Bulk OUT) Endpoint Descriptor
    {
        sizeof(usb_endpoint_descriptor),
        USB_ENDPOINT_DESCRIPTOR_TYPE,
        USB_ENDPOINT_DESCRIPTOR_ENDPOINT_OUT | RX_EP_NUM,
        USB_ENDPOINT_DESCRIPTOR_ATTR_BULK,
        0x40,
        0,
        0
    }
};

// ----- String Descriptors -----

static char mfg_str_descr[USB_MAX_STR_LEN],
            product_str_descr[USB_MAX_STR_LEN],
	    sn_str_descr[USB_MAX_STR_LEN];

static char os_str_desc[USB_MAX_STR_LEN] = 
{
  0x12,    // Length of the descriptor
  0x03,    // Descriptor type
  // Signature field MSFT100
  0x4D, 0x00, 0x53, 0x00, 0x46, 0x00, 0x54, 
  0x00, 0x31, 0x00, 0x30, 0x00, 0x30, 0x00,
  0x02, // bMS_VendorCode 0x01 is for MSOS Descriptor 2.0
  0x00  // Pad value
};


static const char* usb_strings[] = {
    "\x04\x03\x09\x04",
    mfg_str_descr,
    product_str_descr,
    sn_str_descr,
    os_str_desc
};

// ----- Enumeration Data w/ Device Descriptor -----

static const unsigned char bos_capability [0x21] = 
{
    0x05, // BOS descriptor len
    USB_DEVREQ_DESCRIPTOR_TYPE_BOS, 
    0x21, 0x00, // total lengtht
    0x01,       // One capability descriptor
    //
    // Microsoft OS 2.0 Platform Capability Descriptor Header
    //
    0x1C,                    // bLength - 28 bytes
    0x10,                    // bDescriptorType - 16
    0x05,                    // bDevCapability – 5 for Platform Capability
    0x00,                    // bReserved - 0
    0xDF, 0x60, 0xDD, 0xD8,  // MS_OS_20_Platform_Capability_ID -
    0x89, 0x45, 0xC7, 0x4C,  // {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
    0x9C, 0xD2, 0x65, 0x9D,  // 
    0x9E, 0x64, 0x8A, 0x9F,  //

    //
    // Descriptor Information Set for Windows 8.1 or later
    //
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion – 0x06030000 for Windows Blue
    0x9E, 0x00,              // wLength – size of MS OS 2.0 descriptor set
    0x01,                    // bMS_VendorCode
    0x00,                    // bAltEnumCmd – 0 Does not support alternate enum
};

static const unsigned char bos_platform [0x9E] = 
{
    0x0A, 0x00,					// Descriptor size (10 bytes)
    0x00, 0x00,					// MS OS 2.0 descriptor set header
    0x00, 0x00, 0x03, 0x06,			// Windows version (8.1) (0x06030000)
    0x9E, 0x00,					// Size, MS OS 2.0 descriptor set (158 bytes)

    // Microsoft OS 2.0 compatible ID descriptor

    0x14, 0x00,						// Descriptor size (20 bytes)
    0x03, 0x00,			 		  // MS OS 2.0 compatible ID descriptor
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00,			// WINUSB string
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			// Sub-compatible ID

    // Registry property descriptor

    0x80, 0x00,				// Descriptor size (130 bytes)
    0x04, 0x00,				// Registry Property descriptor
    0x01, 0x00,				// Strings are null-terminated Unicode
    0x28, 0x00,				// Size of Property Name (40 bytes)

    //Property Name ("DeviceInterfaceGUID")

    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00,
    0x49, 0x00, 0x6E, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00,
    0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00, 0x49, 0x00,
    0x44, 0x00, 0x00, 0x00, 

    0x4E, 0x00,				// Size of Property Data (78 bytes)

    // Vendor-defined Property Data: {c22dd994-a578-47dd-ae0e-3fef4d03429a}
            
    0x7B, 0x00, 0x63, 0x00, 0x32, 0x00, 0x32, 0x00, 0x64, 0x00, 0x64, 0x00, 
    0x39, 0x00, 0x39, 0x00, 0x34, 0x00, 0x2D, 0x00, 0x61, 0x00, 0x35, 0x00, 
    0x37, 0x00, 0x38, 0x00, 0x2D, 0x00, 0x34, 0x00, 0x37, 0x00, 0x64, 0x00, 
    0x64, 0x00, 0x2D, 0x00, 0x61, 0x00, 0x65, 0x00, 0x30, 0x00, 0x65, 0x00, 
    0x2D, 0x00, 0x33, 0x00, 0x66, 0x00, 0x65, 0x00, 0x66, 0x00, 0x34, 0x00, 
    0x64, 0x00, 0x30, 0x00, 0x33, 0x00, 0x34, 0x00, 0x32, 0x00, 0x39, 0x00, 
    0x61, 0x00, 0x7D, 0x00, 0x00, 0x00
};

static const unsigned char bos_platform_10[0x28] = 
{
    0x28, 0x00, 0x00, 0x00, // length of this descriptor
    0x00, 0x01, // Version 1.0
    0x04, 0x00, // Extended compat ID desriptor
    0x01,       // Number of function sections
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // RESERVED
    0x00,       // Interface umber
    0x01,       // Reserved
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, // CompatibleID "WINUSB"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Secondary ID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00              // Reserved
};

static const unsigned char bos_platforme_10[0x92] = 
{
    //Reading Extended Properties OS Feature Descriptor (wIndex = 0x0005):
    0x92, 0x00, 0x00, 0x00, // length of this descriptor
    0x00, 0x01,             // Version 1.0
    0x05, 0x00,             // Extended compat ID desriptor
    0x01, 0x00,             // Number of function sections
    0x86, 0x00, 0x00, 0x00, // Size of the property section
    0x07, 0x00, 0x00, 0x00, // Property data type
    0x2a, 0x00,             // Property name length
    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 
    0x49, 0x00, 0x6e, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00,
    0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00, 0x49, 0x00, 
    0x44, 0x00, 0x73, 0x00, 0x00, 0x00,
    0x50, 0x00, 0x00, 0x00, // Property data length
    0x7B, 0x00, 0x63, 0x00, 0x32, 0x00, 0x32, 0x00, 0x64, 0x00, 0x64, 0x00, 
    0x39, 0x00, 0x39, 0x00, 0x34, 0x00, 0x2D, 0x00, 0x61, 0x00, 0x35, 0x00, 
    0x37, 0x00, 0x38, 0x00, 0x2D, 0x00, 0x34, 0x00, 0x37, 0x00, 0x64, 0x00, 
    0x64, 0x00, 0x2D, 0x00, 0x61, 0x00, 0x65, 0x00, 0x30, 0x00, 0x65, 0x00, 
    0x2D, 0x00, 0x33, 0x00, 0x66, 0x00, 0x65, 0x00, 0x66, 0x00, 0x34, 0x00, 
    0x64, 0x00, 0x30, 0x00, 0x33, 0x00, 0x34, 0x00, 0x32, 0x00, 0x39, 0x00, 
    0x61, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00 
};

static usbs_enumeration_data usb_enum_data = {
    {
        length:                 sizeof(usb_device_descriptor),
        type:                   USB_DEVICE_DESCRIPTOR_TYPE,
        usb_spec_lo:            0x10, 
        usb_spec_hi:            0x02,
        device_class:           USBS_SERIAL_DEVICE_CLASS,
        device_subclass:        0,
        device_protocol:        0,
        max_packet_size:        EP0_MAX_PACKET_SIZE,
        vendor_lo:              LO_BYTE_16(VENDOR_ID),
        vendor_hi:              HI_BYTE_16(VENDOR_ID),
        product_lo:             LO_BYTE_16(PRODUCT_ID),
        product_hi:             HI_BYTE_16(PRODUCT_ID),
        device_lo:              0x00,
        device_hi:              0x00,
        manufacturer_str:       MFG_STR_INDEX,
        product_str:            PRODUCT_STR_INDEX,
        serial_number_str:      SN_STR_INDEX,
        number_configurations:  1
    },

    total_number_interfaces:    USBS_SERIAL_NUM_IFACE,
    total_number_endpoints:     USBS_SERIAL_NUM_ENDP,
    total_number_strings:       4,
    configurations:             &usb_configuration,
    interfaces:                 usb_interface,
    endpoints:                  usb_endpoints,
    strings:                    (const unsigned char **) usb_strings,
    bos_capability_descriptor:  bos_capability,
    bos_capability_descriptor_len: 0x21,
    bos_platform_descriptor:    bos_platform,
    bos_platform_descriptor_len: 0x9e
};

// --------------------------------------------------------------------------
// USBS Serial Data
// --------------------------------------------------------------------------

usbs_control_endpoint* usbs_serial_ep0 = EP0;

// Lock for the state.
cyg_mutex_t usbs_serial_lock;   

// Condition variable for state changes
cyg_cond_t  usbs_serial_state_cond;

int usbs_serial_state;

usbs_serial usbs_ser0 = {
    tx_result:  0,    
    rx_result:  0,    
};

static void (*usbs_serial_app_state_change_fn)(struct usbs_control_endpoint*, 
                                               void*, usbs_state_change, int) 
= 0;

// --------------------------------------------------------------------------
// Create a USB String Descriptor from a C string.

void
usbs_serial_create_str_descriptor(char descr[], const char *str)
{
    int i, n = strlen(str);

    if (n > (USB_MAX_STR_LEN/2 - 2))
        n = USB_MAX_STR_LEN/2 - 2;

    descr[0] = (cyg_uint8) (2*n + 2);
    descr[1] = USB_DEVREQ_DESCRIPTOR_TYPE_STRING;

    for (i=0; i<n; i++) {
        descr[i*2+2] = str[i];
        descr[i*2+3] = '\x00';
    }
}

// --------------------------------------------------------------------------
// ACM Class Handler
//
// For a Windows host, the device must, at least, respond to a SetLineCoding
// request (0x20), otherwise Windows will report that it's unable to open the 
// port. This request normally sets the standard serial parameters:
//          baud rate, # stop bits, parity, and # data bits
// If we're just making believe that we're a serial port to communicate with
// the host via USB, then these values don't matter. So we ACK the request,
// but ignore the parameters.
// If we were actually creating a USB-serial converter, then we would need to
// read these values and configure the serial port accordingly.
// 
// Similarly, the host can request the current settings through a 
// GetLineCoding request (0x21). Since we're just faking it, we return some
// arbitrary values: 38400,1,N,8

static usbs_control_return 
usbs_serial_ms_vendor_handler(usbs_control_endpoint* ep0, void* data)
{
  //int length;
  usbs_control_return result = USBS_CONTROL_RETURN_STALL;//USBS_CONTROL_RETURN_UNKNOWN;
  usb_devreq      *req = (usb_devreq *) ep0->control_buffer;
  //length      = (req->length_hi << 8) | req->length_lo;

  DBG("USB Serial ACM Class Handler: ");
          
  switch (req->request) {
    
    case 1 :
      DBG("ACM Request: Set Line Coding\n");
      ep0->buffer = (unsigned char*)bos_platform;
      ep0->buffer_size = 0x9e;
      result = USBS_CONTROL_RETURN_HANDLED;
      break; 
      
    case 2 :
      DBG("ACM Request: Set Line Coding\n");
      //diag_printf("bp req %d, %d\n",req->index_lo, length);
      result = USBS_CONTROL_RETURN_HANDLED;
      if(req->index_lo == 0x04)
      {
        ep0->buffer = (unsigned char*)bos_platform_10;
        ep0->buffer_size = 0x28;
      }
      else if(req->index_lo == 0x05 || req->index_lo == 0x00)
      {
          //diag_printf("ext\n");
        ep0->buffer = (unsigned char*)bos_platforme_10;
        ep0->buffer_size = 0x92;
      }
      break; 
      
    default :
      DBG("*** Unhandled ACM Request: 0x%02X ***\n",
          (unsigned) req->request);
  }
  
  return result;
}

// --------------------------------------------------------------------------
// Callback for a USB state change

void
usbs_serial_state_change_handler(usbs_control_endpoint* ep, void* data,
                                 usbs_state_change change, int prev_state)
{
#if defined(CYGBLD_IO_USB_WINUSB_SLAVE_SERIAL_DEBUG)
  const char *STATE_CHG_STR[] = { "Detached", "Attached", "Powered", "Reset",
                                  "Addressed", "Configured", "Deconfigured",
                                  "Suspended", "Resumed" };
  
  if (change > 0) {
    DBG("### %d:%s ###\n", change, STATE_CHG_STR[(int) change-1]);
  }
#endif
  
  // Called from DSR, cond broadcast should be ok without mutex lock
  usbs_serial_state = usbs_serial_ep0->state;
  cyg_cond_broadcast(&usbs_serial_state_cond);
  
  if (usbs_serial_app_state_change_fn)
    (*usbs_serial_app_state_change_fn)(ep, data, change, prev_state);
}

// --------------------------------------------------------------------------
// Block the calling thread until the USB is configured.
 
void
usbs_serial_wait_until_configured(void)
{
  cyg_mutex_lock(&usbs_serial_lock);
  while (usbs_serial_state != USBS_STATE_CONFIGURED)
    cyg_cond_wait(&usbs_serial_state_cond);

#if !defined(CYGPKG_IO_USB_WINUSB_SLAVE_SERIAL_STATIC_EP)
  usbs_ser0.tx_ep = usbs_get_tx_endpoint(usbs_serial_ep0, TX_EP_NUM);
  usbs_ser0.rx_ep = usbs_get_rx_endpoint(usbs_serial_ep0, RX_EP_NUM);
#endif

  cyg_mutex_unlock(&usbs_serial_lock);
}

// --------------------------------------------------------------------------
// Determine if the device is currently configured.

cyg_bool
usbs_serial_is_configured(void)
{
  return usbs_serial_state == USBS_STATE_CONFIGURED;
}

// --------------------------------------------------------------------------
// Callback for when a transmit is complete

static void 
usbs_serial_tx_complete(void *p, int result)
{
  usbs_serial* ser = (usbs_serial*) p;
  ser->tx_result = result;
  cyg_semaphore_post(&ser->tx_ready);
}

// --------------------------------------------------------------------------
// Callback for when a receive is complete

static void 
usbs_serial_rx_complete(void *p, int result)
{
  usbs_serial* ser = (usbs_serial*) p;
  ser->rx_result = result;
  cyg_semaphore_post(&ser->rx_ready);
}

// --------------------------------------------------------------------------
// Start an asynchronous transmit of a buffer.
// 
 
void
usbs_serial_start_tx(usbs_serial* ser, const void* buf, int n)
{
  usbs_start_tx_buffer(ser->tx_ep, (unsigned char*) buf, n,
                       usbs_serial_tx_complete, ser);
}

// --------------------------------------------------------------------------
// Block the caller until the transmit is complete

int
usbs_serial_wait_for_tx(usbs_serial* ser)
{
  cyg_semaphore_wait(&ser->tx_ready);
  return ser->tx_result;
}

// --------------------------------------------------------------------------
// Perform a synchronous transmit and wait for it to complete.

int
usbs_serial_tx(usbs_serial* ser, const void* buf, int n)
{
  usbs_serial_start_tx(ser, buf, n);
  return usbs_serial_wait_for_tx(ser);
}

// --------------------------------------------------------------------------
// Start an asynchronous receive of a buffer.

void
usbs_serial_start_rx(usbs_serial* ser, void* buf, int n)
{
  usbs_start_rx_buffer(ser->rx_ep, (unsigned char*) buf, n,
                       usbs_serial_rx_complete, ser);
}

// --------------------------------------------------------------------------
// Block the caller until the receive is complete

int
usbs_serial_wait_for_rx(usbs_serial* ser)
{
  cyg_semaphore_wait(&ser->rx_ready);
  return ser->rx_result;
}

// --------------------------------------------------------------------------
// Perform a synchronous receive and wait for it to complete.

int
usbs_serial_rx(usbs_serial* ser, void* buf, int n)
{
  usbs_serial_start_rx(ser, buf, n);
  return usbs_serial_wait_for_rx(ser);
}

// --------------------------------------------------------------------------
// Initialize a serial port structure.

void
usbs_serial_init(usbs_serial* ser, usbs_tx_endpoint* tx_ep, 
                 usbs_rx_endpoint* rx_ep)
{
  ser->tx_ep = tx_ep;
  ser->rx_ep = rx_ep;
  
  cyg_semaphore_init(&ser->tx_ready, 0);
  cyg_semaphore_init(&ser->rx_ready, 0);
}

// --------------------------------------------------------------------------
// Start the USB subsystem

void
usbs_serial_start(void)
{
#if defined(CYGPKG_IO_USB_WINUSB_SLAVE_SERIAL_STATIC_EP)
  usbs_serial_init(&usbs_ser0, TX_EP, RX_EP);
#else  
  usbs_serial_init(&usbs_ser0, NULL, NULL);
#endif
  
  cyg_mutex_init(&usbs_serial_lock);
  cyg_cond_init(&usbs_serial_state_cond, &usbs_serial_lock);
  
  // Make the mfg & product names into USB string descriptors
  
  usbs_serial_create_str_descriptor(mfg_str_descr, 
                                    CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_MFG_STR);
  usbs_serial_create_str_descriptor(product_str_descr, 
                                    /*(char*)0x80800030*/CYGDAT_IO_USB_WINUSB_SLAVE_SERIAL_PRODUCT_STR);
  usbs_serial_create_str_descriptor(sn_str_descr, 
                                    /*(char*)0x80800020*/"0000000000");
  
  // ----- Set up enumeration & USB callbacks -----
  
  usbs_serial_state = usbs_serial_ep0->state;
  
  usbs_serial_ep0->enumeration_data   = &usb_enum_data;
  
  if (usbs_serial_ep0->state_change_fn)
    usbs_serial_app_state_change_fn = usbs_serial_ep0->state_change_fn;
  
  usbs_serial_ep0->state_change_fn = usbs_serial_state_change_handler;
  
  if (!usbs_serial_ep0->class_control_fn)
    usbs_serial_ep0->class_control_fn = usbs_serial_ms_vendor_handler;
  
  // ----- Start USB subsystem -----
  
  usbs_start(usbs_serial_ep0);
}
