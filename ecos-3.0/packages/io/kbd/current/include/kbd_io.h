#ifndef CYGONCE_KBD_IO_H
#define CYGONCE_KBD_IO_H
//==========================================================================
//
//      kbd_io.h
//
//      Kyboard io definitions
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
// Author(s):     Filip Adamec
// Date:          2013-04-05
//
//####DESCRIPTIONEND####
//
//==========================================================================


#ifdef __cplusplus
extern "C" {
#endif
    
typedef void (*kbd_call_back_t)(cyg_uint16, cyg_uint16);

#define CYG_KBD_SET_CONFIG_INTERVAL         0
#define CYG_KBD_SET_CONFIG_ENABLE           1
#define CYG_KBD_SET_CONFIG_DISABLE          2
#define CYG_KBD_SET_CONFIG_CALLBACK         3
#define CYG_KBD_SET_CONFIG_CALLBACK_REMOVE  4
#define CYG_KBD_SET_CONFIG_DEFAULT_INTERVAL 5
    
#define CYG_KBD_GET_CONFIG_INTERVAL         0
#define CYG_KBD_GET_CONFIG_ENABLED          1
    
// Aditional status flags for callback mode    
#define CYG_KBD_KEY_DOWNO                   0
#define CYG_KBD_KEY_HOLD                    1


#ifdef __cplusplus
}
#endif

#endif /* CYGONCE_KBD_IO_H */

