#ifndef CYGONCE_DEVS_FB_ST7063_H
#define CYGONCE_DEVS_FB_ST7063_H
//==========================================================================
//
//      fb_st7063.h
//
//      ST7063 LCD driver defines
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
// Date:          2016-11-23
//
//####DESCRIPTIONEND####
//
//==========================================================================

/**
 * @addtogroup Drivers
 *
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif
	
#include <pkgconf/hal.h>
#include <pkgconf/devs_fb_st7063.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
    
#define DISP_OFF                0xae
#define DISP_ON                 0xaf
#define DISP_SET_START_LINE     0x40
#define DISP_SET_REF_VM         0x81
#define DISP_SET_PAGE           0xb0
#define DISP_SET_C_ADR_MSB      0x10
#define DISP_SET_C_ADR_LSB      0x00
#define DISP_SET_ADC_NORMAL     0xa0
#define DISP_SET_ADC_REVERS     0xa1
#define DISP_REVERS_OFF         0xa6
#define DISP_REVERS_ON          0xa7
#define DISP_ENTRIE_OFF         0xa4
#define DISP_ENTRIE_ON          0xa5
#define DISP_BIAS_SELECT_OFF    0xa2
#define DISP_BIAS_SELECT_ON     0xa3
#define DIPS_SET_MODIFI_READ    0xe0
#define DISP_RESET_MODIFI_READ  0xee
#define DISP_RESET              0xe2
#define DISP_SHL_NORMAL         0xc0
#define DISP_SHL_REVERS         0xc8
#define DISP_POWR_CONTROL       0x28
#define DISP_REG_RES_SELECT     0x20
#define DISP_SET_STATIC_IND     0xad
#define DISP_BOOSTER_RATION     0xf8
#define DISP_SET_STATIC_IND     0xac


#define VOLTAGE_CONVERTOR_ON    4 
#define VOLTAGE_CONVERTOR_OFF   0
#define VOLTAGE_REGULATOR_ON    2 
#define VOLTAGE_REGULATOR_OFF   0
#define VOLTAGE_FOLLOWER_ON     1 
#define VOLTAGE_FOLLOWER_OFF    0

#define STATIC_INDICATOR_OFF         0
#define STATIC_INDICATOR_ON_1SB      1
#define STATIC_INDICATOR_ON_05SB     2
#define STATIC_INDICATOR_ON          3

#define PAGE_COUNT                   9


#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif // CYGONCE_DEVS_FB_ST7063_H

/** @} */
//-----------------------------------------------------------------------------
// End of fb_st7063.h
