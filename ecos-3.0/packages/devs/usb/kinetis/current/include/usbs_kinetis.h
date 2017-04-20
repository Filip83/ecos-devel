#ifndef CYGONCE_USBS_UC3C_H
#define CYGONCE_USBS_UC3C_H
//==========================================================================
//
//      include/usbs_kinetis.h
//
//      The interface exported by the Freescale Kinetis USB device driver
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
// Author(s):    Filip
// Contributors: 
// Date:         2017-04-20
// Purpose:
//
//####DESCRIPTIONEND####
//==========================================================================
#include <pkgconf/hal.h>
#include <pkgconf/devs_usb_kinetis.h>

#include <cyg/io/usb/usbs.h>

 
extern usbs_control_endpoint    ep0;
extern usbs_rx_endpoint         ep1;
extern usbs_rx_endpoint         ep2;
extern usbs_rx_endpoint         ep3;
#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 4)
extern usbs_rx_endpoint         ep4;
extern usbs_rx_endpoint         ep5;
#if (CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS > 6)
extern usbs_rx_endpoint         ep6;
extern usbs_rx_endpoint         ep7;
#endif
#endif
/*
extern void usbs_uc3c_endpoint_init(usbs_rx_endpoint * pep, 
                                    cyg_uint8 endpoint_type, cyg_bool enable);*/
#endif /* CYGONCE_USBS_UC3C_H */
