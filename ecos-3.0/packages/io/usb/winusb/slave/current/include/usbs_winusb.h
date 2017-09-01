#ifndef CYGONCE_USBS_WINUSB_H
#define CYGONCE_USBS_WINUSB_H
//==========================================================================
//
//      include/usbs_winusb.h
//
//      Description of the USB slave-side winusb device support
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
// Author(s):    Filip
// Contributors: 
// Date:         2008-06-02
// Purpose:
// Description:  USB slave-side winusb support
//
//
//####DESCRIPTIONEND####
//==========================================================================

#ifdef __cplusplus
extern "C" {
#endif
    

#include <cyg/infra/cyg_type.h>
#include <cyg/io/usb/usbs.h>

// ----------------------------------------------------------------------------
// Data structure to configure WinUSB driver.
    
 typedef void (*app_state_change_fn)(struct usbs_control_endpoint*, 
                                               void*, usbs_state_change, int);
 


// ----------------------------------------------------------------------------
// Data structure to manage the pair of USB endpoints that comprise a single
// port connection. Each "port" requires one Bulk IN endpoint and one
// Bulk OUT endpoint.

typedef struct usbs_winusb {
    // The communication endpoints. For the first (default) channel, these
    // are normally set by the configuration, but can be changed by the
    // application, if desired.
    usbs_tx_endpoint*   tx_ep;
    usbs_rx_endpoint*   rx_ep;

    // The signal that a transmit operation is complete, and it's result.
    cyg_sem_t   tx_ready;
    int         tx_result;

    // The signal that a receive operation is complete, and it's result.
    cyg_sem_t   rx_ready;
    int         rx_result;

    // Callback to notyfi application about state change
    app_state_change_fn app_state_change_callback;
} usbs_winusb;

// The package contains one USB winusb device.
extern usbs_winusb usbs_winusb0;

// It's assumed that there's a single USB slave chip in the system, with a
// single control endpoint 0. The actual variable is contained in the device
// driver, but the USB winusb code keeps a pointer to it for driver 
// independence. The application might find it useful for overriding low-level
// code or callbacks.
extern usbs_control_endpoint* usbs_winusb_ep0;

// ----------------------------------------------------------------------------
// A C interface to the winusb USB code.
// The application can use this interface, the standard (low-level) USB slave
// API, the standard Unix-like I/O API, or C stdio API.
    
// Initialize support for a particular USB winusb "port"
// This associates a usbs_winusb structure with specific endpoints and 
// initializes the structure for communications.
void usbs_winusb_init(usbs_winusb*, usbs_tx_endpoint*, usbs_rx_endpoint*);

// Block the calling thread until the host configures the USB device.
void usbs_winusb_wait_until_configured(void);

// Determines if the USB subsystem is configured
cyg_bool usbs_winusb_is_configured(void);

// Start an asynchronous transmit of a single buffer.
void usbs_winusb_start_tx(usbs_winusb*, const void* buf, int n);

// Block the calling thread until the transmit completes.
// Returns the result code for the transfer
int usbs_winusb_wait_for_tx(usbs_winusb*);

// Blocking, synchronous transmit of a single buffer.
int usbs_winusb_tx(usbs_winusb*, const void* buf, int n);

// Start an asynchronous receive of a buffer.
void usbs_winusb_start_rx(usbs_winusb*, void* buf, int n);

// Block the calling thread until the receive completes.
// Returns the result code for the transfer
int usbs_winusb_wait_for_rx(usbs_winusb*);

// Blocking, synchronous receive of a single buffer.
int usbs_winusb_rx(usbs_winusb*, void* buf, int n);

// The default USB-winusb state change handler paces the functions
// usbs_winusb_wait_until_configured() and usbs_winusb_is_configured().
// The application can override the state chain handler, but chain to 
// this function to keep the full USB-winusb system working.
void usbs_winusb_state_change_handler(usbs_control_endpoint*, void*, 
                                      usbs_state_change, int);

// Starts the USB subsystem
void usbs_winusb_start(app_state_change_fn state_change_fn,
                       unsigned char *mfg_str,
                       unsigned char *product_str,
                       unsigned char *sn_str);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // CYGONCE_USBS_SERIAL_H

