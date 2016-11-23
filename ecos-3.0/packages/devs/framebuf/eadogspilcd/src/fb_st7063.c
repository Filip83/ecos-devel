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



// Switch on a framebuffer device. This may get called multiple
// times, e.g. when switching between different screen modes.
// It just involves sending a message to the auxiliary.
static int
cyg_synth_fb_on(struct cyg_fb* fb)
{
    synth_fb_data*  fb_data = (synth_fb_data*) fb->fb_driver2;
    if (fb_data->connected) {
        synth_auxiliary_xchgmsg(fb_data->devid, SYNTH_FB_ON, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    return 0;
}

static int
cyg_synth_fb_off(struct cyg_fb* fb)
{
    synth_fb_data*  fb_data = (synth_fb_data*) fb->fb_driver2;
    if (fb_data->connected) {
        synth_auxiliary_xchgmsg(fb_data->devid, SYNTH_FB_OFF, 0, 0, 0, 0, 0, 0, 0, 0);
    }
    return 0;
}

static int
cyg_synth_fb_ioctl(struct cyg_fb* fb, cyg_ucount16 key, void* data, size_t* len)
{
    synth_fb_data*  fb_data = (synth_fb_data*) fb->fb_driver2;
    int             result  = ENOSYS;

    switch(key) {
      case CYG_FB_IOCTL_VIEWPORT_GET_POSITION:
        DEBUG(1, "cyg_synth_fb_ioctl: viewport_get_position\n");
        if (fb->fb_flags0 & CYG_FB_FLAGS0_VIEWPORT) {
            cyg_fb_ioctl_viewport*  viewport = (cyg_fb_ioctl_viewport*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_viewport), "data argument should be a cyg_fb_ioctl_viewport structure");
            viewport->fbvp_x    = fb_data->viewport_x;
            viewport->fbvp_y    = fb_data->viewport_y;
            result  = 0;
            DEBUG(1, "                  : current viewport x %d, y %d\n", fb_data->viewport_x, fb_data->viewport_y);
        } else {
            DEBUG(1, "                  : framebuffer does not support a viewport\n");
        }
        break;
      case CYG_FB_IOCTL_VIEWPORT_SET_POSITION:
        DEBUG(1, "cyg_synth_fb_ioctl: viewport_set_position\n");
        if (fb->fb_flags0 & CYG_FB_FLAGS0_VIEWPORT) {
            cyg_fb_ioctl_viewport*  viewport = (cyg_fb_ioctl_viewport*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_viewport), "data argument should be a cyg_fb_ioctl_viewport structure");
            CYG_ASSERT(((viewport->fbvp_x + fb->fb_viewport_width) <= fb->fb_width) &&
                       ((viewport->fbvp_y + fb->fb_viewport_height) <= fb->fb_height),
                       "viewport should be within framebuffer dimensions");
            DEBUG(1, "                  : setting viewport from x %d, y %d to x %d, y %d\n",
                  fb_data->viewport_x, fb_data->viewport_y, (int) viewport->fbvp_x, (int) viewport->fbvp_y);
            if ((fb_data->viewport_x != (int)viewport->fbvp_x) || (fb_data->viewport_y != (int)viewport->fbvp_y)) {
                fb_data->viewport_x = (int)viewport->fbvp_x;
                fb_data->viewport_y = (int)viewport->fbvp_y;
                fb_op(fb, SYNTH_FB_VIEWPORT);
            }
            result = 0;
        } else {
            DEBUG(1, "                  : framebuffer does not support a viewport\n");
        }
        break;
      case CYG_FB_IOCTL_PAGE_FLIPPING_GET_PAGES:
        DEBUG(1, "cyg_synth_fb_ioctl: page_flipping_get_pages\n");
        if (fb->fb_flags0 & CYG_FB_FLAGS0_PAGE_FLIPPING) {
            cyg_fb_ioctl_page_flip* page_flip = (cyg_fb_ioctl_page_flip*)data;
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_page_flip), "data argument should be a cyg_fb_ioctl_page_flip structure");
            page_flip->fbpf_number_pages    = fb->fb_driver1;
            page_flip->fbpf_visible_page    = fb_data->page_visible;
            page_flip->fbpf_drawable_page   = fb_data->page_drawable;
            result = 0;
            DEBUG(1, "                  : number_pages %d, visible page %d, drawable page %d\n",
                  fb->fb_driver1, fb_data->page_visible, fb_data->page_drawable);
        } else {
            DEBUG(1, "                  : framebuffer does not support page flipping\n");
        }
        break;
      case CYG_FB_IOCTL_PAGE_FLIPPING_SET_PAGES:
        DEBUG(1, "cyg_synth_fb_ioctl: page_flipping_set_pages\n");
        if (fb->fb_flags0 & CYG_FB_FLAGS0_PAGE_FLIPPING) {
            cyg_fb_ioctl_page_flip* page_flip = (cyg_fb_ioctl_page_flip*)data;
            cyg_uint8*  fb_base;
            
            CYG_ASSERT(*len == sizeof(cyg_fb_ioctl_page_flip), "data argument should be a cyg_fb_ioctl_page_flip structure");
            CYG_ASSERT((page_flip->fbpf_visible_page  < fb->driver1) &&
                       (page_flip->fbpf_drawable_page < fb->driver1),
                       "framebuffer does not have that many pages");
            DEBUG(1, "                  : drawable page was %d, now %d, visible page was %d, now %d\n",
                  fb_data->page_drawable, (int)page_flip->fbpf_drawable_page,
                  fb_data->page_visible, (int)page_flip->fbpf_visible_page);
            fb_base  = (cyg_uint8*)fb->fb_base;
            fb_base -= (fb->fb_height * fb->fb_stride * fb_data->page_drawable);
            fb_data->page_drawable          = page_flip->fbpf_drawable_page;
            fb_base += (fb->fb_height * fb->fb_stride * fb_data->page_drawable);
            fb->fb_base = fb_base;
            *(cyg_uint8**)fb->fb_driver3 = fb_base;
            if (fb_data->page_visible != (int)page_flip->fbpf_visible_page) {
                fb_data->page_visible = (int)page_flip->fbpf_visible_page;
                fb_op(fb, SYNTH_FB_PAGE_FLIP);
            }
            result = 0;
        } else {
            DEBUG(1, "                  : framebuffer does not support page flipping\n");
        }
        break;
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
    // FIXME: update synch_x0/y0/x1/y1 once the generic framebuffer
    // code actually maintains a bounding box.
    fb_op(fb, SYNTH_FB_SYNC);
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
    for(cyg_ucount16 iy = y; iy < (y + height); iy++)
    {
        for(cyg_ucount16 ix = x; ix < (x + width); ix++) 
        {
            cyg_st7063_fb_write_pixel_fn(ix,iy,colour);
        }
    }
}

void cyg_st7063_fb_write_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, const void* source, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    
}

void  cyg_st7063_fb_read_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, void* dest, 
        cyg_ucount16 offset, cyg_ucount16 stride)
{
    
}

void cyg_st7063_fb_move_block_fn(cyg_fb* fb, cyg_ucount16 x, cyg_ucount16 y, 
        cyg_ucount16 width, cyg_ucount16 height, cyg_ucount16 new_x, 
        cyg_ucount16 new_y)
{

}


// A default area of memory for the framebuffer, if the auxiliary is not
// running.
static cyg_uint8            cyg_st7063_fb0_default_base[CYG_FB_ST7063_HEIGHT * 
                                              CYG_FB_ST7063_STRIDE];

// Driver-specific data needed for interacting with the auxiliary.
//static synth_fb_data    cyg_synth_fb0_data;

// flags0 - pixel maxk
CYG_FB_FRAMEBUFFER(CYG_FB_fb0_STRUCT,
                   CYG_FB_fb0_DEPTH,
                   CYG_FB_fb0_FORMAT,
                   CYG_FB_fb0_WIDTH,
                   CYG_FB_fb0_HEIGHT,
                   CYG_FB_fb0_VIEWPORT_WIDTH,
                   CYG_FB_fb0_VIEWPORT_HEIGHT,
                   cyg_st7063_fb0_default_base,
                   CYG_FB_fb0_STRIDE,
                   CYG_FB_fb0_FLAGS0,
                   CYG_FB_fb0_FLAGS1,
                   CYG_FB_fb0_FLAGS2,
                   CYG_FB_fb0_FLAGS3,
                   (CYG_ADDRWORD) 0,  // id, 0 - 3
                   (CYG_ADDRWORD) 0,
                   (CYG_ADDRWORD) 0,
                   (CYG_ADDRWORD) 0,
                   &cyg_synth_fb_on,
                   &cyg_synth_fb_off,
                   &cyg_synth_fb_ioctl,
                   &cyg_synth_fb_synch,
                   &CYG_FB_fb0_READ_PALETTE_FN,
                   &CYG_FB_fb0_WRITE_PALETTE_FN,
                   &CYG_FB_fb0_MAKE_COLOUR_FN,
                   &CYG_FB_fb0_BREAK_COLOUR_FN,
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
