#ifndef CYGONCE_DEVS_FB_ST7565_H
#define CYGONCE_DEVS_FB_ST7565_H
//==========================================================================
//
//      fb_st7565.h
//
//      ST7565 LCD driver defines
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
// Author(s):     Filip
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
	
    
#define DISP_OFF                            0xae
#define DISP_ON                             0xaf
#define DISP_START_LINE_SET                 0x40
#define DISP_PAGE_ADDRESS_SET               0xb0
#define DISP_COLUMN_ADDRESS_MSB             0x10
#define DISP_COLUMN_ADDRESS_LSB             0x00
#define DISP_ADC_SELECT_NORMAL              0xa0
#define DISP_ADC_SELECT_REVERS              0xa1
#define DISP_NORMAL                         0xa6
#define DISP_REVERS                         0xa7
#define DISP_ALL_POINTS_ON                  0xa5    
#define DISP_ALL_POINTS_NORMAL              0xa4
#define DISP_BIAS_SET_1_9                   0xa2
#define DISP_BIAS_SET_1_7                   0xa3
#define DISP_RESET                          0xe2    
#define DISP_COMMON_OUTPUT_MODE_NORMAL      0xc0
#define DISP_COMMON_OUTPUT_MODE_REVERS      0xc8
#define DISP_POWER_CONTROL_SET              0x28
#define DISP_POWER_CONTROL_BOOSTER_ON       0x04
#define DISP_POWER_CONTROL_V_REG_ON         0x02
#define DISP_POWER_CONTROL_V_FOLLOWER_ON    0x01
#define DISP_VOTAGE_REG_RESISTOR_RATIO_SEL  0x20
#define DISP_ELECTRONIC_VOLUME_MODE_SET     0x81
#define DISP_STATIC_INDICATOR_OFF           0xac
#define DISP_STATIC_INDICATOR_ON            0xad
#define STATIC_INDICATOR_ON_1SB             0x01
#define STATIC_INDICATOR_ON_05SB            0x02
#define STATIC_INDICATOR_ON                 0x03
#define DISP_BOOSTER_RATIO                  0xf8
#define BOOSTER_RATIO_2x_3x_4x              0x00
#define BOOSTER_RATIO_5x                    0x01
#define BOOSTER_RATIO_6x                    0x02


#define DIPS_READ_MODIFY_WRITE              0xe0
#define DISP_READ_MODIFY_WRITE_END          0xee

typedef struct cyg_st7565_fb_driver_s
{
    cyg_bool        blank_on;
    cyg_ucount32    backlight;
    cyg_ucount32    contrast;
}cyg_st7565_fb_driver_t;

#define PAGE_COUNT                  9


#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif // CYGONCE_DEVS_FB_ST7565_H

/** @} */
//-----------------------------------------------------------------------------
// End of fb_st7565.h
