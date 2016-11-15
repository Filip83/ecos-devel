/**
 * \file
 *
 * \brief FLASHC driver for AVR32 UC3.
 *
 * Copyright (c) 2009-2011 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#include <cyg/infra/cyg_type.h>

#include <pkgconf/hal.h>
#include <cyg/io/flashcdw.h>
#include <cyg/io/flash_avr32_cdw.h>
#include <pkgconf/devs_flash_avr32_cdw.h>

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_ass.h>

#include <cyg/io/flash.h>
#include <cyg/io/flash_dev.h>


#include <cyg/hal/utils/compiler.h>


// Initialize the hardware. The FLASH controller does not
// nead any special initialization.
// Make shure taht flash controller is properly
// configured in power manager module
int flash_hwr_init(struct cyg_flash_dev *dev)
{
    return FLASH_ERR_OK;
}

int flash_erase_block( struct cyg_flash_dev *dev, cyg_flashaddr_t block_base)//(volatile unsigned long block)
{
    cyg_uint32 flash_offset = block_base - AVR32_FLASH_ADDRESS;
    if(flashcdw_erase_page(flash_offset/AVR32_FLASHCDW_PAGE_SIZE,true) == false)
	return FLASH_ERR_DRV_VERIFY;

    if(flashcdw_is_lock_error())
        return FLASH_ERR_PROTECT;
    if(flashcdw_is_programming_error())
        return FLASH_ERR_PROGRAM;
    
    return FLASH_ERR_OK;
}

int
flash_program_buf (struct cyg_flash_dev *dev, cyg_flashaddr_t base,
            const void* data, size_t length)
{
    /*volatile cyg_uint32 * dest = (cyg_uint32*)(base - base%AVR32_FLASHCDW_PAGE_SIZE);
    volatile cyg_uint32 * na_dest = (cyg_uint32*)(base);
    const unsigned long * ram_data = (unsigned long*)data;*/
   
    flashcdw_memcpy((void*)base,data,length,true);
    
    if(flashcdw_is_lock_error())
        return FLASH_ERR_PROTECT;
    if(flashcdw_is_programming_error())
        return FLASH_ERR_PROGRAM;
    
    return FLASH_ERR_OK;
   
}

// Flash cannot be bussy. Blocing errase and program used
int flash_query(struct cyg_flash_dev *dev, void* data, size_t len)
{
    return 0;
}


static const CYG_FLASH_FUNS(cyg_avr32_flash_funs,
	               flash_hwr_init,
	               flash_query,
	               flash_erase_block,
	               flash_program_buf,
	               NULL,              // read
	               cyg_flash_devfn_lock_nop,
	               cyg_flash_devfn_unlock_nop);

static /*const*/ cyg_flash_block_info_t cyg_flash_avr32_block_info[1] =
{
    {
         AVR32_FLASHCDW_PAGE_SIZE,
        (AVR32_FLASHCDW_FLASH_SIZE /*- 0x73800*/)/AVR32_FLASHCDW_PAGE_SIZE
    }
};

static /*const*/ cyg_flash_block_info_t cyg_flash_user_avr32_block_info[1] =
{
    {
        AVR32_FLASHCDW_PAGE_SIZE,
        1
    }
};

CYG_FLASH_DRIVER(cyg_flash_avr32_flashdev,
                 &cyg_avr32_flash_funs,
                 0,                     // Flags
                 CYGPKG_DEVS_FLASH_AVR32_CDW_START,      // Start
                 CYGPKG_DEVS_FLASH_AVR32_CDW_END - 1,    // End
                 1,                     // Number of block infos
                 cyg_flash_avr32_block_info,
                 NULL                   // priv
    );
    
CYG_FLASH_DRIVER(cyg_flash_avr32_flashdev_user,
                 &cyg_avr32_flash_funs,
                 0,                     // Flags
                 AVR32_FLASHCDW_USER_PAGE_ADDRESS,// Start
                 AVR32_FLASHCDW_USER_PAGE_ADDRESS + 
                 AVR32_FLASHCDW_USER_PAGE_SIZE - 1,    // End
                 1,                     // Number of block infos
                 cyg_flash_user_avr32_block_info,
                 NULL                   // priv
    );

// See if a range of FLASH addresses overlaps currently running code
bool flash_code_overlaps(void *start, void *end){

    extern char _stext[], _etext[];

    return ((((unsigned long)&_stext >= (unsigned long)start) &&
             ((unsigned long)&_stext < (unsigned long)end)) ||
            (((unsigned long)&_etext >= (unsigned long)start) &&
             ((unsigned long)&_etext < (unsigned long)end)));
}

