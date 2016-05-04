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
#include <cyg/hal/avr32/io.h>

#include <pkgconf/hal.h>
#include <pkgconf/devs_flash_avr32_uc3c.h>

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_ass.h>

#include <cyg/io/flash.h>
#include <cyg/io/flash_dev.h>

#include <cyg/hal/utils/compiler.h>

#include "flash_avr32_uc3c.h"



/*! \brief Clears the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to clear bits.
 * \param mask    Bit-mask indicating bits to clear.
 *
 * \return Resulting value with cleared bits.
 */

//! @}

/*! \name FLASHC Writable Bit-Field Registers
 */
//! @{

typedef union {
	unsigned long                 fcr;
	avr32_flashc_fcr_t            FCR;
} u_avr32_flashc_fcr_t;

typedef union {
	unsigned long                 fcmd;
	avr32_flashc_fcmd_t           FCMD;
} u_avr32_flashc_fcmd_t;

//bool flash_code_overlaps(void *start, void *end);

//! Sticky error status of the FLASHC.
//! This variable is updated by functions that issue FLASHC commands. It
//! contains the cumulated FLASHC error status of all the FLASHC commands issued
//! by a function.
static unsigned int flashc_error_status = 0;

//! @}

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
    if(flashc_erase_page(flash_offset/AVR32_FLASHC_PAGE_SIZE,true) == false)
	    return FLASH_ERR_DRV_VERIFY;

    if(flashc_error_status == 0)
	    return FLASH_ERR_OK;
    else if(flashc_error_status & AVR32_FLASHC_FSR_LOCKE_MASK)
	    return FLASH_ERR_PROTECT;
    else
        return FLASH_ERR_PROGRAM;
}

int
flash_program_buf (struct cyg_flash_dev *dev, cyg_flashaddr_t base,
            const void* data, size_t length)
{
    volatile cyg_uint32 * dest = (cyg_uint32*)(base - base%AVR32_FLASHC_PAGE_SIZE);
    volatile cyg_uint32 * na_dest = (cyg_uint32*)(base);
    const unsigned long * ram_data = (unsigned long*)data;
   
    flashc_memcpy((void*)base,data,length,true);
    
    if(flashc_error_status == 0)
	   return FLASH_ERR_OK;
    else if(flashc_error_status & AVR32_FLASHC_FSR_LOCKE_MASK)
	   return FLASH_ERR_PROTECT;
    else
        return FLASH_ERR_PROGRAM;
   
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
         AVR32_FLASHC_PAGE_SIZE,
        (AVR32_FLASHC_FLASH_SIZE - 0x73800)/AVR32_FLASHC_PAGE_SIZE
    }
};

static /*const*/ cyg_flash_block_info_t cyg_flash_user_avr32_block_info[1] =
{
    {
        AVR32_FLASHC_PAGE_SIZE,
        1
    }
};

CYG_FLASH_DRIVER(cyg_flash_avr32_flashdev,
                 &cyg_avr32_flash_funs,
                 0,                     // Flags
                 /*0x80000000,*/   0x80073800,         // Start
                 0x80000000 + (AVR32_FLASHC_FLASH_SIZE) - 1,    // End
                 1,                     // Number of block infos
                 cyg_flash_avr32_block_info,
                 NULL                   // priv
    );
    
CYG_FLASH_DRIVER(cyg_flash_avr32_flashdev_user,
                 &cyg_avr32_flash_funs,
                 0,                     // Flags
                 0x80800000,// Start
                 0x80800000 + 512 - 1,    // End
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

/*! \name Flash Properties
 */
//! @{


unsigned int flashc_get_flash_size(void)
{
#if (AVR32_FLASHC_H_VERSION >= 300)
    const cyg_uint16 FLASH_SIZE[] = {
        4,
        8,
        16,
        32,
        48,
        64,
        96,
        128,
        192,
        256,
        384,
        512,
        768,
        1024,
        2048,
    };
    return ((unsigned int)FLASH_SIZE[(AVR32_FLASHC.pr & AVR32_FLASHC_PR_FSZ_MASK)
			>> AVR32_FLASHC_PR_FSZ_OFFSET])<<10;

#else // in older flashc version, FSZ is located in FSR register
    const cyg_uint16 FLASH_SIZE[] = {
        32,
        64,
        128,
        256,
        384,
        512,
        768,
        1024,
    };
    return ((unsigned int)FLASH_SIZE[(AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_FSZ_MASK)
                    >> AVR32_FLASHC_FSR_FSZ_OFFSET]) << 10;
#endif
}


unsigned int flashc_get_page_count(void)
{
    return flashc_get_flash_size() / AVR32_FLASHC_PAGE_SIZE;
}


unsigned int flashc_get_page_count_per_region(void)
{
    return flashc_get_page_count() / AVR32_FLASHC_REGIONS;
}


unsigned int flashc_get_page_region(int page_number)
{
    return ((page_number >= 0) ? page_number
             : flashc_get_page_number()) / flashc_get_page_count_per_region();
}


unsigned int flashc_get_region_first_page_number(unsigned int region)
{
    return region * flashc_get_page_count_per_region();
}


//! @}


/*! \name FLASHC Control
 */
//! @{


unsigned int flashc_get_wait_state(void)
{
    return (AVR32_FLASHC.fcr & AVR32_FLASHC_FCR_FWS_MASK) >> AVR32_FLASHC_FCR_FWS_OFFSET;
}


void flashc_set_wait_state(unsigned int wait_state)
{
    u_avr32_flashc_fcr_t u_avr32_flashc_fcr = {AVR32_FLASHC.fcr};
    u_avr32_flashc_fcr.FCR.fws = wait_state;
    AVR32_FLASHC.fcr = u_avr32_flashc_fcr.fcr;
}


void flashc_set_bus_freq(unsigned int cpu_f_hz)
{
    if (cpu_f_hz >= AVR32_FLASHC_FWS_0_MAX_FREQ) {
        // Set 1 WS.
        flashc_set_wait_state(1);
    } else {
        // Set 0 WS.
        flashc_set_wait_state(0);
    }
}

cyg_bool flashc_is_ready_int_enabled(void)
{
    return ((AVR32_FLASHC.fcr & AVR32_FLASHC_FCR_FRDY_MASK) != 0);
}


void flashc_enable_ready_int(cyg_bool enable)
{
    u_avr32_flashc_fcr_t u_avr32_flashc_fcr = {AVR32_FLASHC.fcr};
    u_avr32_flashc_fcr.FCR.frdy = (enable != false);
    AVR32_FLASHC.fcr = u_avr32_flashc_fcr.fcr;
}


cyg_bool flashc_is_lock_error_int_enabled(void)
{
    return ((AVR32_FLASHC.fcr & AVR32_FLASHC_FCR_LOCKE_MASK) != 0);
}


void flashc_enable_lock_error_int(cyg_bool enable)
{
    u_avr32_flashc_fcr_t u_avr32_flashc_fcr = {AVR32_FLASHC.fcr};
    u_avr32_flashc_fcr.FCR.locke = (enable != false);
    AVR32_FLASHC.fcr = u_avr32_flashc_fcr.fcr;
}


cyg_bool flashc_is_prog_error_int_enabled(void)
{
    return ((AVR32_FLASHC.fcr & AVR32_FLASHC_FCR_PROGE_MASK) != 0);
}


void flashc_enable_prog_error_int(cyg_bool enable)
{
    u_avr32_flashc_fcr_t u_avr32_flashc_fcr = {AVR32_FLASHC.fcr};
    u_avr32_flashc_fcr.FCR.proge = (enable != false);
    AVR32_FLASHC.fcr = u_avr32_flashc_fcr.fcr;
}


//! @}


/*! \name FLASHC Status
 */
//! @{


cyg_bool flashc_is_ready(void)
{
    return ((AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_FRDY_MASK) != 0);
}


void flashc_default_wait_until_ready(void)
{
    while (!flashc_is_ready());
}


void (*volatile flashc_wait_until_ready)(void) = flashc_default_wait_until_ready;


/*! \brief Gets the error status of the FLASHC.
 *
 * \return The error status of the FLASHC built up from
 *         \c AVR32_FLASHC_FSR_LOCKE_MASK and \c AVR32_FLASHC_FSR_PROGE_MASK.
 *
 * \warning This hardware error status is cleared by all functions reading the
 *          Flash Status Register (FSR). This function is therefore not part of
 *          the driver's API which instead presents \ref flashc_is_lock_error
 *          and \ref flashc_is_programming_error.
 */
static unsigned int flashc_get_error_status(void)
{
    return AVR32_FLASHC.fsr & (AVR32_FLASHC_FSR_LOCKE_MASK |
                    AVR32_FLASHC_FSR_PROGE_MASK);
}

cyg_bool flashc_is_lock_error(void)
{
    return ((flashc_error_status & AVR32_FLASHC_FSR_LOCKE_MASK) != 0);
}


cyg_bool flashc_is_programming_error(void)
{
    return ((flashc_error_status & AVR32_FLASHC_FSR_PROGE_MASK) != 0);
}


//! @}


/*! \name FLASHC Command Control
 */
//! @{


unsigned int flashc_get_command(void)
{
    return (AVR32_FLASHC.fcmd & AVR32_FLASHC_FCMD_CMD_MASK) >> AVR32_FLASHC_FCMD_CMD_OFFSET;
}


unsigned int flashc_get_page_number(void)
{
    return (AVR32_FLASHC.fcmd & AVR32_FLASHC_FCMD_PAGEN_MASK) >> AVR32_FLASHC_FCMD_PAGEN_OFFSET;
}


void flashc_issue_command(unsigned int command, int page_number)
{
    u_avr32_flashc_fcmd_t u_avr32_flashc_fcmd;

    flashc_wait_until_ready();
    u_avr32_flashc_fcmd.fcmd = AVR32_FLASHC.fcmd;
    u_avr32_flashc_fcmd.FCMD.cmd = command;
    if (page_number >= 0) {
            u_avr32_flashc_fcmd.FCMD.pagen = page_number;
    }
    u_avr32_flashc_fcmd.FCMD.key = AVR32_FLASHC_FCMD_KEY_KEY;
    AVR32_FLASHC.fcmd = u_avr32_flashc_fcmd.fcmd;
    flashc_error_status = flashc_get_error_status();
    flashc_wait_until_ready();
}


//! @}


/*! \name FLASHC Global Commands
 */
//! @{


void flashc_no_operation(void)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_NOP, -1);
}


void flashc_erase_all(void)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_EA, -1);
}


//! @}


/*! \name FLASHC Protection Mechanisms
 */
//! @{


cyg_bool flashc_is_security_bit_active(void)
{
    return ((AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_SECURITY_MASK) != 0);
}


void flashc_activate_security_bit(void)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_SSB, -1);
}


unsigned int flashc_get_bootloader_protected_size(void)
{
    unsigned int bootprot = (1 << AVR32_FLASHC_FGPFRLO_BOOTPROT_SIZE) - 1 -
                    flashc_read_gp_fuse_bitfield(AVR32_FLASHC_FGPFRLO_BOOTPROT_OFFSET,
                    AVR32_FLASHC_FGPFRLO_BOOTPROT_SIZE);
    return (bootprot) ? AVR32_FLASHC_PAGE_SIZE << bootprot : 0;
}


unsigned int flashc_set_bootloader_protected_size(unsigned int bootprot_size)
{
    flashc_set_gp_fuse_bitfield(AVR32_FLASHC_FGPFRLO_BOOTPROT_OFFSET,
                    AVR32_FLASHC_FGPFRLO_BOOTPROT_SIZE,
                    (1 << AVR32_FLASHC_FGPFRLO_BOOTPROT_SIZE) - 1 -
                    ((bootprot_size) ?
                    32 - clz((((min(max(bootprot_size, AVR32_FLASHC_PAGE_SIZE << 1),
                    AVR32_FLASHC_PAGE_SIZE <<
                    ((1 << AVR32_FLASHC_FGPFRLO_BOOTPROT_SIZE) - 1)) +
                    AVR32_FLASHC_PAGE_SIZE - 1) /
                    AVR32_FLASHC_PAGE_SIZE) << 1) - 1) - 1 :
                    0));
    return flashc_get_bootloader_protected_size();
}


cyg_bool flashc_is_external_privileged_fetch_locked(void)
{
    return (!flashc_read_gp_fuse_bit(AVR32_FLASHC_FGPFRLO_EPFL_OFFSET));
}


void flashc_lock_external_privileged_fetch(cyg_bool lock)
{
    flashc_set_gp_fuse_bit(AVR32_FLASHC_FGPFRLO_EPFL_OFFSET, !lock);
}


cyg_bool flashc_is_page_region_locked(int page_number)
{
    return flashc_is_region_locked(flashc_get_page_region(page_number));
}


cyg_bool flashc_is_region_locked(unsigned int region)
{
    return ((AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_LOCK0_MASK << 
            (region & (AVR32_FLASHC_REGIONS - 1))) != 0);
}


void flashc_lock_page_region(int page_number, cyg_bool lock)
{
    flashc_issue_command((lock) ? AVR32_FLASHC_FCMD_CMD_LP : 
        AVR32_FLASHC_FCMD_CMD_UP, page_number);
}


void flashc_lock_region(unsigned int region, cyg_bool lock)
{
    flashc_lock_page_region(flashc_get_region_first_page_number(region), lock);
}


void flashc_lock_all_regions(cyg_bool lock)
{
    unsigned int error_status = 0;
    unsigned int region = AVR32_FLASHC_REGIONS;

    while (region) {
            flashc_lock_region(--region, lock);
            error_status |= flashc_error_status;
    }
    flashc_error_status = error_status;
}


//! @}


/*! \name Access to General-Purpose Fuses
 */
//! @{


cyg_bool flashc_read_gp_fuse_bit(unsigned int gp_fuse_bit)
{
    return ((flashc_read_all_gp_fuses() & 1ULL << (gp_fuse_bit & 0x3F)) != 0);
}


cyg_uint64 flashc_read_gp_fuse_bitfield(unsigned int pos, unsigned int width)
{
    return flashc_read_all_gp_fuses() >> (pos & 0x3F) & ((1ULL << min(width, 64)) - 1);
}


cyg_uint8 flashc_read_gp_fuse_byte(unsigned int gp_fuse_byte)
{
    return flashc_read_all_gp_fuses() >> ((gp_fuse_byte & 0x07) << 3);
}


cyg_uint64 flashc_read_all_gp_fuses(void)
{
    return AVR32_FLASHC.fgpfrlo | (cyg_uint64)AVR32_FLASHC.fgpfrhi << 32;
}


cyg_bool flashc_erase_gp_fuse_bit(unsigned int gp_fuse_bit, cyg_bool check)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_EGPB, gp_fuse_bit & 0x3F);
    return (check) ? flashc_read_gp_fuse_bit(gp_fuse_bit) : true;
}


cyg_bool flashc_erase_gp_fuse_bitfield(unsigned int pos, unsigned int width,
        cyg_bool check)
{
    unsigned int error_status = 0;
    unsigned int gp_fuse_bit;

    pos &= 0x3F;
    width = min(width, 64);
    for (gp_fuse_bit = pos; gp_fuse_bit < pos + width; gp_fuse_bit++) {
            flashc_erase_gp_fuse_bit(gp_fuse_bit, false);
            error_status |= flashc_error_status;
    }
    flashc_error_status = error_status;
    return (check) ? (flashc_read_gp_fuse_bitfield(pos, width) == (1ULL << width) - 1) : true;
}


cyg_bool flashc_erase_gp_fuse_byte(unsigned int gp_fuse_byte, cyg_bool check)
{
    unsigned int error_status;
    unsigned int current_gp_fuse_byte;

    cyg_uint64 value = flashc_read_all_gp_fuses();
    flashc_erase_all_gp_fuses(false);
    error_status = flashc_error_status;

    for (current_gp_fuse_byte = 0; current_gp_fuse_byte < 8;
            current_gp_fuse_byte++, value >>= 8) {
            if (current_gp_fuse_byte != gp_fuse_byte) {
                    flashc_write_gp_fuse_byte(current_gp_fuse_byte, value);
                    error_status |= flashc_error_status;
            }
    }
    flashc_error_status = error_status;
    return (check) ? (flashc_read_gp_fuse_byte(gp_fuse_byte) == 0xFF) : true;
}


cyg_bool flashc_erase_all_gp_fuses(cyg_bool check)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_EAGPF, -1);
    return (check) ? (flashc_read_all_gp_fuses() == 0xFFFFFFFFFFFFFFFFULL) : true;
}


void flashc_write_gp_fuse_bit(unsigned int gp_fuse_bit, cyg_bool value)
{
    if (!value) {
        flashc_issue_command(AVR32_FLASHC_FCMD_CMD_WGPB, gp_fuse_bit & 0x3F);
    }
}


void flashc_write_gp_fuse_bitfield(unsigned int pos, unsigned int width, cyg_uint64 value)
{
    unsigned int error_status = 0;
    unsigned int gp_fuse_bit;

    pos &= 0x3F;
    width = min(width, 64);

    for (gp_fuse_bit = pos; gp_fuse_bit < pos + width; gp_fuse_bit++, value >>= 1) {
        flashc_write_gp_fuse_bit(gp_fuse_bit, value & 0x01);
        error_status |= flashc_error_status;
    }

    flashc_error_status = error_status;
}


void flashc_write_gp_fuse_byte(unsigned int gp_fuse_byte, cyg_uint8 value)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_PGPFB, (gp_fuse_byte & 0x07) | value << 3);
}


void flashc_write_all_gp_fuses(cyg_uint64 value)
{
    unsigned int error_status = 0;
    unsigned int gp_fuse_byte;

    for (gp_fuse_byte = 0; gp_fuse_byte < 8; gp_fuse_byte++, value >>= 8) {
        flashc_write_gp_fuse_byte(gp_fuse_byte, value);
        error_status |= flashc_error_status;
    }

    flashc_error_status = error_status;
}


void flashc_set_gp_fuse_bit(unsigned int gp_fuse_bit, cyg_bool value)
{
    if (value) {
        flashc_erase_gp_fuse_bit(gp_fuse_bit, false);
    } else {
        flashc_write_gp_fuse_bit(gp_fuse_bit, false);
    }
}


void flashc_set_gp_fuse_bitfield(unsigned int pos, unsigned int width, cyg_uint64 value)
{
    unsigned int error_status = 0;
    unsigned int gp_fuse_bit;

    pos &= 0x3F;
    width = min(width, 64);

    for (gp_fuse_bit = pos; gp_fuse_bit < pos + width; gp_fuse_bit++, value >>= 1) {
        flashc_set_gp_fuse_bit(gp_fuse_bit, value & 0x01);
        error_status |= flashc_error_status;
    }

    flashc_error_status = error_status;
}


void flashc_set_gp_fuse_byte(unsigned int gp_fuse_byte, cyg_uint8 value)
{
    unsigned int error_status;

    switch (value) {
    case 0xFF:
        flashc_erase_gp_fuse_byte(gp_fuse_byte, false);
        break;

    case 0x00:
        flashc_write_gp_fuse_byte(gp_fuse_byte, 0x00);
        break;

    default:
        flashc_erase_gp_fuse_byte(gp_fuse_byte, false);
        error_status = flashc_error_status;
        flashc_write_gp_fuse_byte(gp_fuse_byte, value);
        flashc_error_status |= error_status;
        break;
    }
}


void flashc_set_all_gp_fuses(cyg_uint64 value)
{
    unsigned int error_status;

    switch (value) {
    case 0xFFFFFFFFFFFFFFFFULL:
        flashc_erase_all_gp_fuses(false);
        break;

    case 0x0000000000000000ULL:
        flashc_write_all_gp_fuses(0x0000000000000000ULL);
        break;

    default:
        flashc_erase_all_gp_fuses(false);
        error_status = flashc_error_status;
        flashc_write_all_gp_fuses(value);
        flashc_error_status |= error_status;
        break;
    }
}


//! @}


/*! \name Access to Flash Pages
 */
//! @{


void flashc_clear_page_buffer(void)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_CPB, -1);
}


cyg_bool flashc_is_page_erased(void)
{
    return ((AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_QPRR_MASK) != 0);
}


cyg_bool flashc_quick_page_read(int page_number)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_QPR, page_number);
    return flashc_is_page_erased();
}


cyg_bool flashc_erase_page(int page_number, cyg_bool check)
{
    cyg_bool page_erased = true;

    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_EP, page_number);
    if (check) {
        unsigned int error_status = flashc_error_status;
        page_erased = flashc_quick_page_read(-1);
        flashc_error_status |= error_status;
    }
    return page_erased;
}


cyg_bool flashc_erase_all_pages(cyg_bool check)
{
    cyg_bool all_pages_erased = true;
    unsigned int error_status = 0;
    unsigned int page_number = flashc_get_page_count();

    while (page_number) {
        all_pages_erased &= flashc_erase_page(--page_number, check);
        error_status |= flashc_error_status;
    }
    flashc_error_status = error_status;
    return all_pages_erased;
}


void flashc_write_page(int page_number)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_WP, page_number);
}


cyg_bool flashc_quick_user_page_read(void)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_QPRUP, -1);
    return flashc_is_page_erased();
}


cyg_bool flashc_erase_user_page(cyg_bool check)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_EUP, -1);
    return (check) ? flashc_quick_user_page_read() : true;
}


void flashc_write_user_page(void)
{
    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_WUP, -1);
}


volatile void *flashc_memset8(volatile void *dst, cyg_uint8 src, size_t nbytes, cyg_bool erase)
{
    return flashc_memset16(dst, src | (cyg_uint16)src << 8, nbytes, erase);
}


volatile void *flashc_memset16(volatile void *dst, cyg_uint16 src, size_t nbytes, cyg_bool erase)
{
    return flashc_memset32(dst, src | (cyg_uint32)src << 16, nbytes, erase);
}


volatile void *flashc_memset32(volatile void *dst, cyg_uint32 src, 
        size_t nbytes, cyg_bool erase)
{
    return flashc_memset64(dst, src | (cyg_uint64)src << 32, nbytes, erase);
}


volatile void *flashc_memset64(volatile void *dst, cyg_uint64 src, 
        size_t nbytes, cyg_bool erase)
{
    // Use aggregated pointers to have several alignments available for a same address.
    UnionCVPtr flash_array_end;
    UnionVPtr dest;
    Union64 source = {0};
    StructCVPtr dest_end;
    UnionCVPtr flash_page_source_end;
    cyg_bool incomplete_flash_page_end;
    Union64 flash_dword;
    UnionVPtr tmp;
    unsigned int error_status = 0;
    unsigned int i;

    // Reformat arguments.
    flash_array_end.u8ptr = AVR32_FLASH + flashc_get_flash_size();
    dest.u8ptr = dst;
    for (i = (Get_align((cyg_uint32)dest.u8ptr, sizeof(cyg_uint64)) - 1) & (sizeof(cyg_uint64) - 1);
                    src; i = (i - 1) & (sizeof(cyg_uint64) - 1)) {
        source.u8[i] = src;
        src >>= 8;
    }
    dest_end.u8ptr = dest.u8ptr + nbytes;

    // If destination is outside flash, go to next flash page if any.
    if (dest.u8ptr < AVR32_FLASH) {
        dest.u8ptr = AVR32_FLASH;
    } else if (flash_array_end.u8ptr <= dest.u8ptr && dest.u8ptr < AVR32_FLASHC_USER_PAGE) {
        dest.u8ptr = AVR32_FLASHC_USER_PAGE;
    }

    // If end of destination is outside flash, move it to the end of the previous flash page if any.
    if (dest_end.u8ptr > AVR32_FLASHC_USER_PAGE + AVR32_FLASHC_USER_PAGE_SIZE) {
        dest_end.u8ptr = AVR32_FLASHC_USER_PAGE + AVR32_FLASHC_USER_PAGE_SIZE;
    } else if (AVR32_FLASHC_USER_PAGE >= dest_end.u8ptr && dest_end.u8ptr > flash_array_end.u8ptr) {
        dest_end.u8ptr = flash_array_end.u8ptr;
    }

    // Align each end of destination pointer with its natural boundary.
    dest_end.u16ptr = (cyg_uint16 *)Align_down((cyg_uint32)dest_end.u8ptr, sizeof(cyg_uint16));
    dest_end.u32ptr = (cyg_uint32 *)Align_down((cyg_uint32)dest_end.u16ptr, sizeof(cyg_uint32));
    dest_end.u64ptr = (cyg_uint64 *)Align_down((cyg_uint32)dest_end.u32ptr, sizeof(cyg_uint64));

    // While end of destination is not reached...
    while (dest.u8ptr < dest_end.u8ptr) {
        // Clear the page buffer in order to prepare data for a flash page write.
        flashc_clear_page_buffer();
        error_status |= flashc_error_status;

        // Determine where the source data will end in the current flash page.
        flash_page_source_end.u64ptr =
            (cyg_uint64 *)min((cyg_uint32)dest_end.u64ptr,
            Align_down((cyg_uint32)dest.u8ptr, AVR32_FLASHC_PAGE_SIZE) + AVR32_FLASHC_PAGE_SIZE);

        // Determine if the current destination page has an incomplete end.
        incomplete_flash_page_end = (Align_down((cyg_uint32)dest.u8ptr, AVR32_FLASHC_PAGE_SIZE) >=
                        Align_down((cyg_uint32)dest_end.u8ptr, AVR32_FLASHC_PAGE_SIZE));

        // Use a flash double-word buffer to manage unaligned accesses.
        flash_dword.u64 = source.u64;

        // If destination does not point to the beginning of the current flash page...
        if (!Test_align((cyg_uint32)dest.u8ptr, AVR32_FLASHC_PAGE_SIZE)) {
            // Fill the beginning of the page buffer with the current flash page data.
            // This is required by the hardware, even if page erase is not requested,
            // in order to be able to write successfully to erased parts of flash
            // pages that have already been written to.
            for (tmp.u8ptr = (cyg_uint8 *)Align_down((cyg_uint32)dest.u8ptr, AVR32_FLASHC_PAGE_SIZE);
                            tmp.u64ptr < (cyg_uint64 *)Align_down((cyg_uint32)dest.u8ptr, sizeof(cyg_uint64));
                            tmp.u64ptr++) {
                *tmp.u64ptr = *tmp.u64ptr;
            }

            // If destination is not 64-bit aligned...
            if (!Test_align((cyg_uint32)dest.u8ptr, sizeof(cyg_uint64))) {
                // Fill the beginning of the flash double-word buffer with the current
                // flash page data.
                // This is required by the hardware, even if page erase is not
                // requested, in order to be able to write successfully to erased parts
                // of flash pages that have already been written to.
                for (i = 0; i < Get_align((cyg_uint32)dest.u8ptr, sizeof(cyg_uint64)); i++) {
                        flash_dword.u8[i] = *tmp.u8ptr++;
                }

                // Align the destination pointer with its 64-bit boundary.
                dest.u64ptr = (cyg_uint64 *)Align_down((cyg_uint32)dest.u8ptr, sizeof(cyg_uint64));

                // If the current destination double-word is not the last one...
                if (dest.u64ptr < dest_end.u64ptr) {
                    // Write the flash double-word buffer to the page buffer and reinitialize it.
                    *dest.u64ptr++ = flash_dword.u64;
                    flash_dword.u64 = source.u64;
                }
            }
        }

        // Write the source data to the page buffer with 64-bit alignment.
        for (i = flash_page_source_end.u64ptr - dest.u64ptr; i; i--) {
                *dest.u64ptr++ = source.u64;
        }

        // If the current destination page has an incomplete end...
        if (incomplete_flash_page_end) {
            // This is required by the hardware, even if page erase is not requested,
            // in order to be able to write successfully to erased parts of flash
            // pages that have already been written to.
            {
                tmp.u8ptr = (volatile cyg_uint8 *)dest_end.u8ptr;

                // If end of destination is not 64-bit aligned...
                if (!Test_align((cyg_uint32)dest_end.u8ptr, sizeof(cyg_uint64))) {
                    // Fill the end of the flash double-word buffer with the current flash page data.
                    for (i = Get_align((cyg_uint32)dest_end.u8ptr, sizeof(cyg_uint64)); i < sizeof(cyg_uint64); i++)
                        flash_dword.u8[i] = *tmp.u8ptr++;

                    // Write the flash double-word buffer to the page buffer.
                    *dest.u64ptr++ = flash_dword.u64;
                }

                // Fill the end of the page buffer with the current flash page data.
                for (; !Test_align((cyg_uint32)tmp.u64ptr, AVR32_FLASHC_PAGE_SIZE); tmp.u64ptr++) {
                    *tmp.u64ptr = *tmp.u64ptr;
                }
            }
        }

        // If the current flash page is in the flash array...
        if (dest.u8ptr <= AVR32_FLASHC_USER_PAGE) {
            // Erase the current page if requested and write it from the page buffer.
            if (erase) {
                flashc_erase_page(-1, false);
                error_status |= flashc_error_status;
            }
            flashc_write_page(-1);
            error_status |= flashc_error_status;

            // If the end of the flash array is reached, go to the User page.
            if (dest.u8ptr >= flash_array_end.u8ptr) {
                dest.u8ptr = AVR32_FLASHC_USER_PAGE;
            }
        } else {
            // Erase the User page if requested and write it from the page buffer.
            if (erase) {
                flashc_erase_user_page(false);
                error_status |= flashc_error_status;
            }
            flashc_write_user_page();
            error_status |= flashc_error_status;
        }
    }

    // Update the FLASHC error status.
    flashc_error_status = error_status;

    // Return the initial destination pointer as the standard memset function does.
    return dst;
}

volatile void *flashc_memcpy(volatile void *dst, const void *src, size_t nbytes, cyg_bool erase)
{
    cyg_uint16 page_pos;
    Union64 flash_dword;
    cyg_uint8 i;
    cyg_bool b_user_page;
    unsigned int error_status = 0;
    cyg_uint8* flash_add;
    cyg_uint8* dest_add=(cyg_uint8*)dst;
    const cyg_uint8* src_buf=(const cyg_uint8*)src;

    // Copy area must be in flash array or flash user page
    CYG_ASSERT( (((cyg_uint8 *)dst >= AVR32_FLASH)
            && (((cyg_uint8 *)dst + nbytes) <= (AVR32_FLASH + flashc_get_flash_size())))
            || (((cyg_uint8 *)dst >= AVR32_FLASHC_USER_PAGE)
            && (((cyg_uint8 *)dst + nbytes) <= (AVR32_FLASHC_USER_PAGE + AVR32_FLASHC_USER_PAGE_SIZE))),
            "Write to internal FLASH fail. Not in FLASH area.\n");

    b_user_page = (volatile cyg_uint8 *)dst >= AVR32_FLASHC_USER_PAGE;

    flash_add = (cyg_uint8*)((cyg_uint32)dest_add - ((cyg_uint32)dest_add % AVR32_FLASHC_PAGE_SIZE));

    while (nbytes) {
        // Clear the page buffer in order to prepare data for a flash page write.
        flashc_clear_page_buffer();
        error_status |= flashc_error_status;

        // Loop in the page
        for (page_pos=0; page_pos<AVR32_FLASHC_PAGE_SIZE; page_pos+=sizeof(cyg_uint64) ) {
            // Read the flash double-word buffer
            flash_dword.u64 = *(volatile cyg_uint64*)flash_add;

            // Update double-word if necessary
            for (i = 0; i < sizeof(cyg_uint64); i++) {
                if (nbytes && (flash_add == dest_add)) {
                    // Update page with data source
                    flash_dword.u8[i] = *src_buf++;
                    dest_add++;
                    nbytes--;
                }
                flash_add++;
            }

            // Write the flash double-word buffer to the page buffer.
            *(volatile cyg_uint64*)((cyg_uint32)flash_add - sizeof(cyg_uint64))= flash_dword.u64;
        }

        // Erase the current page if requested and write it from the page buffer.
        if (erase) {
            (b_user_page)? flashc_erase_user_page(false) : flashc_erase_page(-1, false);
            error_status |= flashc_error_status;
        }

        // Write the page
        (b_user_page)? flashc_write_user_page() : flashc_write_page(-1);
        error_status |= flashc_error_status;
    }
    // Update the FLASHC error status.
    flashc_error_status = error_status;

    // Return the initial destination pointer as the standard memcpy function does.
    return dst;
}

//uc3c specific settings
#if 1
void flashc_set_flash_waitstate_and_readmode(unsigned long cpu_f_hz)
{
    //! Device-specific data
    #undef AVR32_FLASHC_FWS_0_MAX_FREQ
    #undef AVR32_FLASHC_FWS_1_MAX_FREQ
    #undef AVR32_FLASHC_HSEN_FWS_0_MAX_FREQ
    #undef AVR32_FLASHC_HSEN_FWS_1_MAX_FREQ
    #define AVR32_FLASHC_FWS_0_MAX_FREQ           33000000
    #define AVR32_FLASHC_FWS_1_MAX_FREQ           66000000
    #define AVR32_FLASHC_HSEN_FWS_0_MAX_FREQ      33000000
    #define AVR32_FLASHC_HSEN_FWS_1_MAX_FREQ      72000000
    // These defines are missing from or wrong in the toolchain header files uc3cxxx.h
    // Put a Bugzilla

    if (cpu_f_hz > AVR32_FLASHC_HSEN_FWS_0_MAX_FREQ) { // > 33MHz
            // Set a wait-state
            flashc_set_wait_state(1);
            if (cpu_f_hz <= AVR32_FLASHC_FWS_1_MAX_FREQ) {   // <= 66MHz and >33Mhz
                    // Disable the high-speed read mode.
                    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_HSDIS, -1);
            } else { // > 66Mhz
                    // Enable the high-speed read mode.
                    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_HSEN, -1);
            }
    } else { // <= 33 MHz
            // Disable wait-state
            flashc_set_wait_state(0);
            // Disable the high-speed read mode.
            flashc_issue_command(AVR32_FLASHC_FCMD_CMD_HSDIS, -1);
    }
}
#endif // UC3C device-specific implementation

//! @}

