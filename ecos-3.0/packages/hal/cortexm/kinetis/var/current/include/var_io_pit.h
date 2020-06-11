#ifndef CYGONCE_HAL_VAR_IO_PIT_H
#define CYGONCE_HAL_VAR_IO_PIT_H
//===========================================================================
//
//      var_io_pit.h
//
//      Variant specific registers
//
//===========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2011, 2013 Free Software Foundation, Inc.                        
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
//===========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):     Filip <filip.gnu@gmail.com>
// Date:          2011-02-05
// Purpose:       Kinetis variant specific registers
// Description:
// Usage:         #include <cyg/hal/var_io_pit.h>
//
//####DESCRIPTIONEND####
//
//===========================================================================
#ifdef __cplusplus
extern "C" {
#endif

/** PIT - Register Layout Typedef */
typedef volatile struct cyghwr_hal_kinteis_pit_s{
  cyg_uint32 MCR;                               /**< PIT Module Control Register, offset: 0x0 */
  cyg_uint8 RESERVED_0[252];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    cyg_uint32 LDVAL;                             /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
    cyg_uint32 CVAL;                              /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
    cyg_uint32 TCTRL;                             /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
    cyg_uint32 TFLG;                              /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
  } CHANNEL[4];
} cyghwr_hal_kinteis_pit_t;

#define CYGHWR_HAL_KINETIS_PIT_MCR_MDIS             (0x02)
#define CYGHWR_HAL_KINETIS_PIT_MCR_FRZ              (0x01)

#define CYGHWR_HAL_KINETIS_PIT_TCTRL_CHN            (0x04)
#define CYGHWR_HAL_KINETIS_PIT_TCTRL_TIE            (0x02)
#define CYGHWR_HAL_KINETIS_PIT_TCTRL_TEN            (0x01)

#define CYGHWR_HAL_KINETIS_PIT_TFLG_TIF             (0x01)

#define CYGHWR_HAL_KINETIS_PIT_CHANNEL0             (0)
#define CYGHWR_HAL_KINETIS_PIT_CHANNEL1             (1)
#define CYGHWR_HAL_KINETIS_PIT_CHANNEL2             (2)
#define CYGHWR_HAL_KINETIS_PIT_CHANNEL3             (3)

#define CYGHWR_HAL_KINETIS_PIT_BASE                 (0x40037000u)

#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
// end of var_io_pit.h
#endif /* CYGONCE_HAL_VAR_IO_PIT_H */

