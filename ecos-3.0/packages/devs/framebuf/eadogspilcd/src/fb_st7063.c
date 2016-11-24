//==========================================================================
//
//      fv_st7063.c
//
//      Frame buffer st7063 driver
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
// Date:         2016-11-23
// Purpose:
// Description:  SPI framebuffer driver for st7036
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/io/spi.h>
#include <cyg/io/spi_avr32.h>
#include <cyg/hal/gpio.h>
#include <cyg/kernel/kernel.h>
#include <cyg/hal/board_config.h>

externC cyg_spi_avr32_device_t lcd_spi_device;

#define GRL_ENDIAN_TYPE CYG_FB_FLAGS0_LE

// Switch on a framebuffer device. This may get called multiple
// times, e.g. when switching between different screen modes.
// It just involves sending a message to the auxiliary.
static int
cyg_synth_fb_on(struct cyg_fb* fb)
{
    cyg_uint8 tx_data[15];
    gpio_set_pin_high(LCD_RESET_PIN);
    cyg_thread_delay(1);
    
    tx_data[0]  = DISP_SET_START_LINE | 0x00;
    tx_data[1]  = DISP_SET_ADC_REVERS;
    tx_data[2]  = DISP_SHL_NORMAL;
    tx_data[3]  = DISP_REVERS_OFF;
    tx_data[4]  = DISP_POWR_CONTROL | 0x07;
    tx_data[5]  = DISP_BOOSTER_RATION;
    tx_data[6]  = 0x00;
    tx_data[7]  = DISP_REG_RES_SELECT | 0x03;
    tx_data[8]  = DISP_SET_REF_VM;
    tx_data[9]  = 0x1f;
    tx_data[10] = DISP_SET_STATIC_IND;
    tx_data[11] = 0x00;
    tx_data[12] = DISP_ON;
   
    gpio_set_pin_low(AVR32_PIN_PC15);
    
    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, true, 12, tx_data, NULL, true);
    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
    
    return 0;
}

static int
cyg_synth_fb_off(struct cyg_fb* fb)
{
    tx_data[4];
    tx_data[0] = DISP_OFF;
   
    gpio_set_pin_low(AVR32_PIN_PC15);
    
    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, true, 1, tx_data, NULL, true);
    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
    return 0;
}

static int
cyg_synth_fb_ioctl(struct cyg_fb* fb, cyg_ucount16 key, void* data, size_t* len)
{
    synth_fb_data*  fb_data = (synth_fb_data*) fb->fb_driver2;
    int             result  = ENOSYS;

    switch(key)
    {
        case CYG_FB_IOCTL_BLANK_GET:
          {
              cyg_fb_ioctl_blank* blank = (cyg_fb_ioctl_blank*)data;
              DEBUG(1, "cyg_synth_fb_ioctl: blank_get, current on state %d\n", fb_data->blank_on);
              CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_blank), "data argument should be a cyg_fb_ioctl_blank structure");
              blank->fbbl_on  = fb_data->blank_on;
              result = 0;
          }
          break;
        case CYG_FB_IOCTL_BLANK_SET:
          {
              cyg_fb_ioctl_blank* blank = (cyg_fb_ioctl_blank*)data;
              CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_blank), "data argument should be a cyg_fb_ioctl_blank structure");
              DEBUG(1, "cyg_synth_fb_ioctl: blank_set, on was %d, now %d\n", fb_data->blank_on, blank->fbbl_on);
              if (blank->fbbl_on != fb_data->blank_on) {
                  fb_data->blank_on = blank->fbbl_on;
                  fb_op(fb, SYNTH_FB_BLANK);
              }
              result = 0;
          }
          break;
        case CYG_FB_IOCTL_BACKLIGHT_GET:
            break;
        case CYG_FB_IOCTL_BACKLIGHT_SET:
            brak;
        
      default:
        result  = ENOSYS;
        break;
    }
    return result;
}

// synch buffer with display
void
cyg_synth_fb_synch(struct cyg_fb* fb, cyg_ucount16 when)
{
    int data_to_trasfer = fb->fb_stride*fb->fb_height;
    unsigned char command_buf[3] =
    {
        DISP_SET_C_ADR_LSB,
        DISP_SET_C_ADR_MSB,
        DISP_SET_PAGE
    };
    #if LCD_DIG_LEVEL > 0
    diag_printf("LCD draw\n");
    #endif
    //cyg_thread_delay(1);
    gpio_set_pin_low(AVR32_PIN_PC15);
    //mozna neblokujici ale nevim
    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);

    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, true, 
                                            3,(cyg_uint8*) command_buf, NULL, true);

    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
    //cyg_thread_delay(1);
    gpio_set_pin_high(AVR32_PIN_PC15);
    cyg_spi_transaction_begin((cyg_spi_device*)&lcd_spi_device);
    cyg_spi_transaction_transfer((cyg_spi_device*)&lcd_spi_device, true, 
                                            data_to_trasfer,(cyg_uint8*) buffer, NULL, true);
    cyg_spi_transaction_end((cyg_spi_device*)&lcd_spi_device);
}

static cyg_fb_colour cyg_fb_get_buffer_pixel(cyg_ucount16 x, cyg_ucount16 y, 
         const void* source, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    cyg_uint8 color_mask =  1;
    cyg_uint8 *cbit_map = ((cyg_uint8*)source);

    cyg_uint32 x_bit  = x&0x7;
#if GRL_ENDIAN_TYPE == 0
    return (cbit_map[y*stride + (x >> 3)] >> (7 - x_bit)) & color_mask;
#else
    return (cbit_map[(y*stride + x) >> 3] >> (x_bit)) & color_mask;
#endif
}

static void cyg_fb_set_buffer_pixel(cyg_ucount16 x, cyg_ucount16 y, 
         cyg_fb_colour colour, void* dest, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    cyg_uint8 *cbit_map       = (cyg_uint8*)source;
    cyg_fb_colour color_mask  = 1;
    colour                   &= color_mask;

    cyg_uint32 x_bit = x&0x7;
#if GRL_ENDIAN_TYPE == 0
    colour <<= (7-x_bit);
    color_mask <<= (7-x_bit);
#else
    colour <<= (x_bit);
    color_mask <<= (x_bit);
#endif

    cbit_map[y*stride + (x >> 3)] &= ~color_mask;
    cbit_map[y*stride + (x >> 3)] |= colour;
}


void cyg_st7063_fb_write_pixel_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_fb_colour colour)
{
    if(x < fb->fb_width && y < fb->fb_height)
    {
	cyg_uint8 color_mask = 1;
	colour &= color_mask;
	
        // eni potreba mam jnom jednu moznost jedn bit jeden pixel
	//y *=fb->fb_depth;// bits_per_pixel;

	cyg_ucount16 y_byte = y >> 3;//y/8;
	cyg_ucount16 y_bit  = y&0x07;//y%8;
#if GRL_ENDIAN_TYPE == 0
	colour <<= (7-y_bit);
	color_mask <<= (7-y_bit);
#else
	colour <<= (y_bit);
	color_mask <<= (y_bit);
#endif
        cyg_uint8 *cbit_map = (cyg_uint8*)fb->fb_base;
        cbit_map[y_byte*fb->fb_stride + x] &= ~color_mask;
        cbit_map[y_byte*fb->fb_stride + x] |= colour;
    }
}

cyg_fb_colour cyg_st7063_fb_read_pixel_fn(cyg_fb* fb, cyg_ucount16 x,
        cyg_ucount16 y)
{
    cyg_uint32 color_mask =  1;
    cyg_uint8 *cbit_map   = (cyg_uint8*)fb->fb_base;

    cyg_ucount16 y_byte = y >> 3;//y/8;
    cyg_ucount16 y_bit  = y&0x07;//y%8;

#if GRL_ENDIAN_TYPE == 0
    color_mask <<= (7-y_bit);
    return (cbit_map[y_byte*fb->fb_stride + x] >> (7 - y_bit)) & color_mask;
#else
    return (cbit_map[y_byte*fb->fb_stride + x] >> (y_bit)) & color_mask;
#endif
}

void cyg_st7063_fb_write_hline_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 len, cyg_fb_colour colour)
{
    
}

void cyg_st7063_fb_write_vline_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 len, cyg_fb_colour colour)
{
    
}

void cyg_st7063_fb_fill_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, cyg_fb_colour colour)
{
    for(cyg_ucount16 ix = x; ix < (x + width) && ix < fb->fb_width; ix++) 
    {
        for(cyg_ucount16 iy = y; iy < (y + height) && iy < fb->fb_height; iy++)
        {
            cyg_uint8 color_mask = 1;
            colour &= color_mask;

            // eni potreba mam jnom jednu moznost jedn bit jeden pixel
            //y *=fb->fb_depth;// bits_per_pixel;

            cyg_ucount16 y_byte = y >> 3;//y/8;
            cyg_ucount16 y_bit  = y&0x07;//y%8;
    #if GRL_ENDIAN_TYPE == 0
            colour <<= (7-y_bit);
            color_mask <<= (7-y_bit);
    #else
            colour <<= (y_bit);
            color_mask <<= (y_bit);
    #endif
            cyg_uint8 *cbit_map = (cyg_uint8*)fb->fb_base;
            cbit_map[y_byte*fb->fb_stride + x] &= ~color_mask;
            cbit_map[y_byte*fb->fb_stride + x] |= colour;
        }
    }
}

void cyg_st7063_fb_write_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, const void* source, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    for(cyg_ucount16 ix = x, bx = 0; ix < (x + width) && ix < fb->fb_width; ix++, bx++) 
    {
        for(cyg_ucount16 iy = y, by = 0; iy < (y + height) && iy < fb->fb_height; iy++, by++)
        {    
            cyg_fb_colour colour = cyg_fb_get_buffer_pixel(bx, by, source, offset, stride);
            cyg_st7063_fb_write_pixel_fn(fb, ix, iy,colour);
        }
    }
}

void  cyg_st7063_fb_read_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, void* dest, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    
    for(cyg_ucount16 ix = x, bx = 0; ix < (x + width) && ix < fb->fb_width; ix++, bx++) 
    {
        for(cyg_ucount16 iy = y, by = 0; iy < (y + height) && iy < fb->fb_height; iy++, by++)
        {    
            cyg_fb_colour colour = cyg_st7063_fb_read_pixel_fn(fb, ix, bi);
            cyg_fb_set_buffer_pixel(bx, by,colour,dest, offset,stride);
        }
    }
}

void cyg_st7063_fb_move_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, cyg_ucount16 new_x, 
        cyg_ucount16 new_y)
{
    for(cyg_ucount16 ix = x, bx = new_x; ix < (x + width) && ix < fb->fb_width; ix++, bx++) 
    {
        for(cyg_ucount16 iy = y, by = new_y; iy < (y + height) && iy < fb->fb_height; iy++, by++)
        {    
            cyg_fb_colour colour = cyg_st7063_fb_read_pixel_fn(fb, ix, iy);
            cyg_st7063_fb_write_pixel_fn(fb, bx, by, colour);
        }
    }
}


// A default area of memory for the framebuffer, if the auxiliary is not
// running.
static cyg_uint8 cyg_st7063_fb_default_base[CYGNUM_DEVS_FRAMEBUF_ST7063_FB_WIDTH * 
                                              CYGNUM_DEVS_FRAMEBUF_ST7063_FB_STRIDE];

// Driver-specific data needed for interacting with the auxiliary.
//static synth_fb_data    cyg_synth_fb0_data;

// flags0 - pixel maxk
CYG_FB_FRAMEBUFFER(cyg_fb_fb0,
                   1,
                   CYG_FB_FORMAT_1BPP_MONO_0_WHITE,
                   CYGNUM_DEVS_FRAMEBUF_ST7063_FB_WIDTH,
                   CYGNUM_DEVS_FRAMEBUF_ST7063_FB_HEIGHT,
                   0,
                   0,
                   cyg_st7063_fb_default_base,
                   CYGNUM_DEVS_FRAMEBUF_ST7063_FB_STRIDE,
                   CYG_FB_FORMAT_1BPP_MONO_0_WHITE,
                   CYG_FB_FLAGS0_LE | CYG_FB_FLAGS0_DOUBLE_BUFFER | CYG_FB_FLAGS0_BACKLIGHT,
                   0,
                   0,
                   (CYG_ADDRWORD) 0,  // id, 0 - 3
                   (CYG_ADDRWORD) 0,
                   (CYG_ADDRWORD) 0,
                   (CYG_ADDRWORD) 0,
                   &cyg_st7063_fb_on,
                   &cyg_st7063_fb_off,
                   &cyg_st7063_fb_ioctl,
                   &cyg_st7063_fb_synch,
                   &NULL,
                   &NULL
                   &NULL,
                   &NULL,
                   &cyg_st7063_fb_write_pixel_fn,
                   &cyg_st7063_fb_read_pixel_fn,
                   &cyg_st7063_fb_write_hline_fn,
                   &cyg_st7063_fb_write_vline_fn,
                   &cyg_st7063_fb_fill_block_fn,
                   &cyg_st7063_fb_write_block_fn,
                   &cyg_st7063_fb_read_block_fn,
                   &cyg_st7063_fb_move_block_fn,
                   0, 0, 0, 0       // Spare0 -> spare3
    );
