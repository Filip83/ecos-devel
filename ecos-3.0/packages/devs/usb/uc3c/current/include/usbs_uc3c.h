#ifndef CYGONCE_USBS_UC3C_H
#define CYGONCE_USBS_UC3C_H
//==========================================================================
//
//      include/usbs_uc3c.h
//
//      The interface exported by the AVR32UC3C USB device driver
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2006, 2007 Free Software Foundation, Inc.                  
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
// Author(s):    Filip Adamec
// Contributors: bartv
// Date:         2012-02-25
// Purpose:
//
//####DESCRIPTIONEND####
//==========================================================================

#include <cyg/io/usb/usbs.h>
#include <pkgconf/devs_usb_uc3c.h>
#include <pkgconf/system.h>



#define UC3C_USB_ENDPOINTS 6

 
extern usbs_control_endpoint    usbs_uc3c_ep0;
extern usbs_rx_endpoint         usbs_uc3c_ep1;
extern usbs_rx_endpoint         usbs_uc3c_ep2;
extern usbs_rx_endpoint         usbs_uc3c_ep3;
#if (UC3C_USB_ENDPOINTS > 4)
extern usbs_rx_endpoint         usbs_uc3c_ep4;
extern usbs_rx_endpoint         usbs_uc3c_ep5;
#if (UC3C_USB_ENDPOINTS > 6)
extern usbs_rx_endpoint         usbs_uc3c_ep6;
extern usbs_rx_endpoint         usbs_uc3c_ep7;
#endif
#endif
/*
extern void usbs_uc3c_endpoint_init(usbs_rx_endpoint * pep, 
                                    cyg_uint8 endpoint_type, cyg_bool enable);*/
#endif /* CYGONCE_USBS_UC3C_H */
