#ifndef CYGONCE_BEEPER_H
#define CYGONCE_BEEPER_H
/*==========================================================================
//
//      beeper.h
//
//      Generic beeper driver layer header
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2008 Free Software Foundation, Inc.                        
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
// Date:         2018-12-19
// Description:  Implements generic layer of beeper drivers.
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <pkgconf/system.h>
#include <pkgconf/io_beeper.h>


#ifdef __cplusplus
extern "C" {
#endif


//==========================================================================
// The functional API.

// Value of beep_interval_in_sys_ticks les tan zero menas default beep interval
extern void cyg_beep( int beep_interval_ms );
extern void cyg_beep_nb(int beep_interval_ms);
//extern void cyg_beep_f(unsigned int freq_hz, int beep_interval_ms);
extern int  cyg_beep_is_beeping(void);
extern int  cyg_beep_wait(void);


//==========================================================================
#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif // CYGONCE_BEEPER_H

