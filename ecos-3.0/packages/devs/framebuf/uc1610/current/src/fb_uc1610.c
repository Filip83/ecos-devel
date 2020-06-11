//==========================================================================
//
//      fv_uc1610.c
//
//      Frame buffer UC1610 driver
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
// Author(s):    Filip <filip.gnu@gmail.com>
// Contributors:
// Date:         2019-09-17
// Purpose:
// Description:  SPI framebuffer driver for UC1610
//
//####DESCRIPTIONEND####
//
//==========================================================================
#include <pkgconf/hal.h>
#include <string.h>
#include <errno.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_cache.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <pkgconf/devs_framebuf_uc1610.h>
#include <cyg/io/fb_uc1610.h>

#include <cyg/io/spi.h>
#include <cyg/io/framebuf.h>
#include <cyg/io/framebuf.inl>

//#include <cyg/io/spi_freescale_dspi.h>

#define CYG_FB_FLAGS1_FILL_COPY     0
#define CYG_FB_FLAGS1_FILL_OR       1
#define CYG_FB_FLAGS1_FILL_XOR      2
#define CYG_FB_FLAGS1_FILL_AND      3
#define CYG_FB_FLAGS1_FILL_COPY_NEG 4


externC cyg_spi_device lcd_spi_device;

cyg_uc1610_fb_driver_t cyg_uc1610_fb_driver = 
{
    .blank_on    = false,
    .backlight   = 0,
    .contrast    = 0x5f, // BIAS POT
};

#ifndef CYGPKG_KERNEL
# define UC1610_POLLED false
#else
# define UC1610_POLLED true
#endif

// A default area of memory for the framebuffer.
static cyg_uint8 cyg_uc1610_fb_default_base[160 *124*2/8];

// Switch on a framebuffer device. This may get called multiple
// times, e.g. when switching between different screen modes.
// It just involves sending a message to the auxiliary.
static int
cyg_uc1610_fb_on(struct cyg_fb* fb)
{
    cyg_uint8 tx_data[20];
    //gpio_set_pin_high(LCD_RESET_PIN);
    //cyg_thread_delay(1);
    
	tx_data[0] = LCD_SET_COM_END;
	tx_data[1] = 0x67;
    tx_data[2] = LCD_SET_MAPPING_CONTROL;
	tx_data[3] = LCD_SET_SCROLL_LINE_LSB;
	tx_data[4] = LCD_SET_SCROLL_LINE_MSB;
	tx_data[5] = LCD_SET_PANEL_LOADING | LCD_PANEL_LOADING_28_39nF;
	tx_data[6] = LCD_SET_BIAS_RATIO | LCD_BIAS_RATIO_12;
	tx_data[7] = LCD_SET_BIAS_POT;
	tx_data[8] = 0x5f;
	tx_data[9] = LCD_SET_RAM_ADDR_CONTROL | 0x01;
	tx_data[10] = LCD_SET_DISPLAY_ENABLE | 0x01;
    tx_data[11] = LCD_SET_INVERS_DISPLAY | 0x01;

    CYGNUM_DEVS_FRAMEBUF_RESETPIN_HIGH;
    CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW;
    cyg_thread_delay(10);
    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, UC1610_POLLED, 12, tx_data, NULL, true);
    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
    return 0;
}

static int
cyg_uc1610_fb_off(struct cyg_fb* fb)
{
    return 0;
}

static int
cyg_uc1610_fb_ioctl(struct cyg_fb* fb, cyg_ucount16 key, void* data, size_t* len)
{
    cyg_uc1610_fb_driver_t *fb_data = 
            (cyg_uc1610_fb_driver_t*)fb->fb_driver0;
    int             result  = ENOSYS;

    switch(key)
    {
        case CYG_FB_IOCTL_BLANK_GET:
          {
              cyg_fb_ioctl_blank* blank = (cyg_fb_ioctl_blank*)data;
              CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_blank), "data argument should be a cyg_fb_ioctl_blank structure");
              blank->fbbl_on  = fb_data->blank_on;
              result = 0;
          }
          break;
        case CYG_FB_IOCTL_BLANK_SET:
        {
            cyg_fb_ioctl_blank* blank = (cyg_fb_ioctl_blank*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_blank), "data argument should be a cyg_fb_ioctl_blank structure");
           /* if (blank->fbbl_on != fb_data->blank_on) 
            {
                fb_data->blank_on = blank->fbbl_on;
                if(fb_data->blank_on)
                {
                    cyg_uint8 tx_data[5];
                    tx_data[0] = DISP_STATIC_INDICATOR_OFF;
                    tx_data[1] = 0x00;
                    tx_data[2] = DISP_OFF;
                    tx_data[3] = LCD_SET_ALL_PX_ON;

                    CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW;

                    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
                    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device,
                            true, 4, tx_data, NULL, true);
                    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
                }
                else
                {
                    cyg_uint8 tx_data[5];
                    tx_data[0] = DISP_ALL_POINTS_NORMAL; 
                    tx_data[1] = DISP_STATIC_INDICATOR_ON;
                    tx_data[2] = 0x03;      // No blinking constantly on

                    CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW;

                    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
                    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device,
                            true, 3, tx_data, NULL, true);
                    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
                }
            }*/
            result = ENOSYS;
        }
          break;
        case CYG_FB_IOCTL_BACKLIGHT_GET:
        {
            cyg_fb_ioctl_backlight* backlight = (cyg_fb_ioctl_backlight*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_backlight), 
                    "data argument should be a cyg_fb_ioctl_backlight structure");
            backlight->fbbl_current = fb_data->backlight;
            backlight->fbbl_max     = 1;
            result = 0;
        }
            break;
        case CYG_FB_IOCTL_BACKLIGHT_SET:
        {
            cyg_fb_ioctl_backlight* backlight = (cyg_fb_ioctl_backlight*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_backlight), 
                    "data argument should be a cyg_fb_ioctl_backlight structure");
            if(backlight->fbbl_current == 0)
            {
                CYGHWR_IO_CLEAR_PIN_LCD_BL;
                fb_data->backlight = 0;
            }
            else
            {
                CYGHWR_IO_SET_PIN_LCD_BL;
                fb_data->backlight = 1;
            }
            result = 0;
        }   
            break;
        case CYG_FB_IOCTL_CONTRAST_GET:
        {
            cyg_fb_ioctl_contrast* contrast = (cyg_fb_ioctl_contrast*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_contrast), 
                    "data argument should be a cyg_fb_ioctl_contrast structure");
            contrast->fbco_current = fb_data->contrast;
            contrast->fbco_max     = 0xff;
            result = 0;
        }
            break;
        case CYG_FB_IOCTL_CONTRAST_SET:
        {
            cyg_fb_ioctl_contrast* contrast = (cyg_fb_ioctl_contrast*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_contrast), 
                    "data argument should be a cyg_fb_ioctl_contrast structure");
            cyg_uint8 tx_data[2];
            fb_data->contrast = contrast->fbco_current;

			tx_data[0] = LCD_SET_BIAS_POT;
            tx_data[1] = fb_data->contrast&0xff;
            CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW;

            cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
            cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, 
                UC1610_POLLED, 2, tx_data, NULL, true);
            cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
            result = 0;
        }
            break;
      default:
        result  = ENOSYS;
        break;
    }
    return result;
}

// synch buffer with display
void
cyg_uc1610_fb_synch(struct cyg_fb* fb, cyg_ucount16 when)
{
    cyg_uint8 tx_data[20];
#if defined(LCD_DIAG_LEVEL) && LCD_DIAG_LEVEL > 0
    diag_printf("fb UC1610 draw\n");
#endif
    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
    CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW;

    tx_data[0] = LCD_SET_COL_ADDR_LSB;
    tx_data[1] = LCD_SET_COL_ADDR_MSB;
    tx_data[2] = LCD_SET_PAGE_ADDR;

    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, UC1610_POLLED,
        3,tx_data, NULL, true);

	CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_HIGH;
	// Write data to display
    
	cyg_spi_transaction_transfer((cyg_spi_device*)& lcd_spi_device, UC1610_POLLED,
		sizeof(cyg_uc1610_fb_default_base),
		((cyg_uint8*)fb->fb_base), NULL, true);

    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
}


void cyg_uc1610_fb_write_pixel_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_fb_colour colour)
{
    if(x < fb->fb_width && y < fb->fb_height)
    {
        cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                x, y, colour);
    }
}

cyg_fb_colour cyg_uc1610_fb_read_pixel_fn(cyg_fb* fb, cyg_ucount16 x,
        cyg_ucount16 y)
{
    return cyg_fb_linear_read_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride, x, y);
}

void cyg_uc1610_fb_write_hline_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 len, cyg_fb_colour colour)
{
    cyg_fb_linear_write_hline_paged_2LE_inl(fb->fb_base, fb->fb_stride,
        x, y, len, colour);
	
}

void cyg_uc1610_fb_write_vline_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 len, cyg_fb_colour colour)
{   
    cyg_ucount16 _y = y;
    for (; _y < (y + len); _y++)
    {
        cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
            x, _y, colour);
    }
}

void cyg_uc1610_fb_fill_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, cyg_fb_colour colour)
{
    int h,_y;

    if (fb->fb_flags1 == CYG_FB_FLAGS1_FILL_COPY)
    {
        for (; width && x < fb->fb_width; x++, width--)
        {
            for (h = height, _y = y; h && y < fb->fb_height; _y++, h--)
            {
                cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                    x, _y, colour);
            }
        }
    }
    else if (fb->fb_flags1 == CYG_FB_FLAGS1_FILL_COPY_NEG)
    {
        for (; width && x < fb->fb_width; x++, width--)
        {
            for (h = height, _y = y; h && y < fb->fb_height; _y++, h--)
            {
                cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                    x, _y, ~colour);
            }
        }
    }
    else if (fb->fb_flags1 == CYG_FB_FLAGS1_FILL_OR)
    {
        for (; width && x < fb->fb_width; x++, width--)
        {
            for (h = height, _y = y; h && y < fb->fb_height; _y++, h--)
            {
                cyg_fb_colour cur_pixel = cyg_fb_linear_read_pixel_paged_2BE_inl
                    (fb->fb_base, fb->fb_stride, x, _y);
                cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                    x, _y, cur_pixel | colour);
            }
        }
    }
    else if (fb->fb_flags1 == CYG_FB_FLAGS1_FILL_XOR)
    {
        for (; width && x < fb->fb_width; x++, width--)
        {
            for (h = height, _y = y; h && y < fb->fb_height; _y++, h--)
            {
                cyg_fb_colour cur_pixel = cyg_fb_linear_read_pixel_paged_2LE_inl
                (fb->fb_base, fb->fb_stride, x, _y);
                cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                    x, _y, cur_pixel ^ colour);
            }
        }
    }
    else
    {
        for (; width && x < fb->fb_width; x++, width--)
        {
            for (h = height, _y = y; h && y < fb->fb_height; _y++, h--)
            {
                cyg_fb_colour cur_pixel = cyg_fb_linear_read_pixel_paged_2BE_inl
                (fb->fb_base, fb->fb_stride, x, _y);
                cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                    x, _y, cur_pixel & colour);
            }
        }
    }
}

void cyg_uc1610_fb_write_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, const void* source, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    cyg_ucount16 bx, by, h;
    for(bx = 0; width && (x + bx) < fb->fb_width; bx++, width--) 
    {
        for(by = 0, h = height; h && (y + by) < fb->fb_height; by++, h--)
        {    
            cyg_fb_colour colour = cyg_fb_linear_read_pixel_1BE_inl((void*)source,stride,bx,by);
            cyg_fb_linear_write_pixel_paged_2LE_inl(fb->fb_base, fb->fb_stride,
                    x + bx, y + by, colour | colour << 1);
        }
    }
}

void  cyg_uc1610_fb_read_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, void* dest, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    cyg_ucount16 bx, by, h;
    for(bx = 0; width && (x + bx) < fb->fb_width; bx++, width--) 
    {
        for(by = 0, h = height; h && (y + by) < fb->fb_height; by++, h--)
        {    
            cyg_fb_colour colour = cyg_fb_linear_read_pixel_paged_2BE_inl
                    (fb->fb_base, fb->fb_stride, x + bx, y + by);
            cyg_fb_linear_write_pixel_paged_2LE_inl(dest, stride, bx, by, colour);
        }
    }
}

void cyg_uc1610_fb_move_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, cyg_ucount16 new_x, 
        cyg_ucount16 new_y)
{
    cyg_ucount16 h;
    for(; width && new_x < fb->fb_width; x++, new_x++, width--) 
    {
        for(h = height; h && new_y < fb->fb_height; y++, new_y++, h--)
        {    
            cyg_fb_colour colour = cyg_fb_linear_read_pixel_paged_2BE_inl
                    (fb->fb_base, fb->fb_stride, x, y);
            cyg_fb_linear_write_pixel_paged_2LE_inl
                    (fb->fb_base, fb->fb_stride, new_x, new_y, colour);
        }
    }
}



// Driver-specific data needed for interacting with the auxiliary.
//static synth_fb_data    cyg_synth_fb0_data;

CYG_FB_FRAMEBUFFER(cyg_fb_fb0,
                   2,
				   CYG_FB_FORMAT_2BPP_GREYSCALE_0_WHITE,
                   CYGNUM_DEVS_FRAMEBUF_UC1610_FB_WIDTH,
                   CYGNUM_DEVS_FRAMEBUF_UC1610_FB_HEIGHT,
                   0,
                   0,
                   cyg_uc1610_fb_default_base,
                   CYGNUM_DEVS_FRAMEBUF_UC1610_FB_WIDTH,
                   CYG_FB_FLAGS0_LE | CYG_FB_FLAGS0_DOUBLE_BUFFER | CYG_FB_FLAGS0_BACKLIGHT,
                   0,
                   0,
                   0,
                   (CYG_ADDRWORD) &cyg_uc1610_fb_driver,  // id, 0 - 3
                   (CYG_ADDRWORD) 0,
                   (CYG_ADDRWORD) 0,
                   (CYG_ADDRWORD) 0,
                   &cyg_uc1610_fb_on,
                   &cyg_uc1610_fb_off,
                   &cyg_uc1610_fb_ioctl,
                   &cyg_uc1610_fb_synch,
                   NULL,
                   NULL,
                   NULL,
                   NULL,
                   &cyg_uc1610_fb_write_pixel_fn,
                   &cyg_uc1610_fb_read_pixel_fn,
                   &cyg_uc1610_fb_write_hline_fn,
                   &cyg_uc1610_fb_write_vline_fn,
                   &cyg_uc1610_fb_fill_block_fn,
                   &cyg_uc1610_fb_write_block_fn,
                   &cyg_uc1610_fb_read_block_fn,
                   &cyg_uc1610_fb_move_block_fn,
                   0, 0, 0, 0
    );