#define SUCCESS (0)
#define FAIL (1)

//==========================================================================
//
//      devs_disk_freescale_usdhc.c
//
//      Provide a disk device driver for SDHC cards over Freescale USDHC
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2004, 2006 Free Software Foundation, Inc.                  
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
// Author:       Mike Jones <mike@proclivis.com>
// Date:         2013-12-04
//
//####DESCRIPTIONEND####
//==========================================================================

#include <pkgconf/system.h>
#include <pkgconf/devs_disk_freescale_usdhc.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_if.h>             // delays
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/diag.h>
#include <string.h>
#include <errno.h>
#include <cyg/io/io.h>
#include <cyg/io/devtab.h>
#include <cyg/io/disk.h>
#include <cyg/io/usdhc_protocol.h>
#include <cyg/io/usdhc.h>
#include <cyg/io/usdhc_host.h>
#include <cyg/io/registers/regsusdhc.h>

extern bool usdhc_card_detected(cyg_uint32 instance);
extern cyg_int32 usdhc_init_interrupt(cyg_uint32 instance);
extern bool usdhc_write_protected(cyg_uint32 instance);
extern cyg_int32 usdhc_software_reset(cyg_uint32 instance);
extern cyg_int32 sd_get_rca(cyg_uint32 instance);
extern cyg_int32 sd_voltage_validation(cyg_uint32 instance, cyg_uint32 *revision);

// Communication parameters. First some debug support
#define DEBUG   0
#if DEBUG > 0
# define DEBUG1(format, ...)    diag_printf(format, ## __VA_ARGS__)
#else
# define DEBUG1(format, ...)
#endif
#if DEBUG > 1
# define DEBUG2(format, ...)    diag_printf(format, ## __VA_ARGS__)
#else
# define DEBUG2(format, ...)
#endif

#if defined(CYGPKG_HAL_ARM_IMX6_WANDBOARD)
#define USDHC_INSTANCE 1
#elif defined(CYGPKG_HAL_ARM_IMX6_SABRE)
#define USDHC_INSTANCE 2
#else
#define USDHC_INSTANCE 3
#endif

#ifdef CYGIMP_DEVS_DISK_USDHC_POLLED
cyg_bool    cyg_usdhc_polled = true;
#else
cyg_bool    cyg_usdhc_polled = false;
#endif

#undef USDHC_BACKGROUND_WRITES

// The size of each disk block
#define USDHC_BLOCK_SIZE      512
// The number of retries during a mount operation when switching to
// IDLE mode.
#define USDHC_GO_IDLE_RETRIES 16
// The number of retries during a mount operation when switching from
// idle to operational
#define USDHC_OP_COND_RETRIES 128
// The number of retries when waiting for a response to any command
#define USDHC_COMMAND_RETRIES 32
// Retries when waiting for a data response token during a read
#define USDHC_READ_DATA_TOKEN_RETRIES 32768
// Retries during a write while waiting for completion
#define USDHC_WRITE_BUSY_RETRIES 32768

// ----------------------------------------------------------------------------
// Structures and statics.
//

typedef enum sd_capacity_e {
  STANDARD_CAPACITY,
  HIGH_CAPACITY,
  EXTENDED_CAPACITY
} sd_capacity_t;

// Details of a specific SD card
typedef struct cyg_usdhc_disk_info_t {
//    cyg_spi_device*     usdhc_dev;
    cyg_uint32          sd_saved_baudrate;
    cyg_uint32          sd_block_count;
#ifdef USDHC_BACKGROUND_WRITES
    cyg_bool            sd_writing;
#endif
    cyg_bool            sd_read_only;
    cyg_bool            sd_connected;
    cyg_uint32          sd_heads_per_cylinder;
    cyg_uint32          sd_sectors_per_head;
    cyg_uint32          sd_read_block_length;
    cyg_uint32          sd_write_block_length;
    sdhc_cid_register   sd_id;
    sd_ocr_register_t   sd_ocr;
    cyg_uint32          sd_version;
    sd_capacity_t       sd_capacity;
    cyg_uint32          sd_csd_version;
} cyg_usdhc_disk_info_t;


// ----------------------------------------------------------------------------
// The low-level SDHC operations

const cyg_uint32 sd_if_cmd_args[SD_IF_CMD_ARG_COUNT] = {
    SD_IF_HV_COND_ARG,
    SD_IF_LV_COND_ARG,
};

static cyg_bool
usdhc_disk_changed(cyg_usdhc_disk_info_t* disk)
{
    return false;
}

static Cyg_ErrNo
usdhc_check_for_disk(cyg_usdhc_disk_info_t* disk)
{
    sdhc_csd_register    csd;
#if DEBUG > 0
    cyg_int32 i;
#endif

	if(usdhc_card_detected(USDHC_INSTANCE) == false) {
		return -ENODEV;
	}

    /* write protect */
	if(usdhc_write_protected(USDHC_INSTANCE) == true) {
		return -EPERM;
	}

    /* Initialize interrupt */
    if (usdhc_init_interrupt(USDHC_INSTANCE) == FAIL) {
        return -EIO;
    }

    /* Enable Identification Frequency */
    usdhc_host_cfg_clock(USDHC_INSTANCE, IDENTIFICATION_FREQ);

    /* Send Init 80 Clock */
    usdhc_host_init_active(USDHC_INSTANCE);

    /* Issue Software Reset to card */
    if (usdhc_software_reset(USDHC_INSTANCE) == FAIL) {
        return -EIO;
    }

    /* SD Voltage Validation */
    if (sd_voltage_validation(USDHC_INSTANCE, &(disk->sd_csd_version)) == SUCCESS) {
        /* SD Initialization */
        sd_init(USDHC_INSTANCE, FOUR, disk->sd_id.cid_data, csd.csd_data);
    }
    else
    	return -EIO;

    // In case of V2 card, determine its capacity class
// FIXME
//    if (2 == disk->sd_version) {
//      reply = mmc_spi_sd_check_v2_capacity_class(disk);
//      if (MMC_REPLY_SUCCESS != reply) {
//          DEBUG1("%s(): can't establish card's capacity class: reply code %02x\n",
//              __FUNCTION__, reply);
//          return -EIO;
//      }
//    }
    DEBUG2("%s capacity card.\n", (STANDARD_CAPACITY == disk->sd_capacity) ?
        "Standard" : (HIGH_CAPACITY == disk->sd_capacity) ? "High" : "Extended" );

    DEBUG2("CID data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",               \
           disk->sd_id.cid_data[ 0], disk->sd_id.cid_data[ 1], disk->sd_id.cid_data[ 2], disk->sd_id.cid_data[ 3],  \
           disk->sd_id.cid_data[ 4], disk->sd_id.cid_data[ 5], disk->sd_id.cid_data[ 6], disk->sd_id.cid_data[ 7],  \
           disk->sd_id.cid_data[ 8], disk->sd_id.cid_data[ 9], disk->sd_id.cid_data[10], disk->sd_id.cid_data[11],  \
           disk->sd_id.cid_data[12], disk->sd_id.cid_data[13], disk->sd_id.cid_data[14], disk->sd_id.cid_data[15]);
#if DEBUG > 0
    DEBUG1("CID data: register\n");
    DEBUG1("        : Manufacturer ID       : MID = 0x%02x\n", SDHC_CID_REGISTER_MID(&(disk->sd_id)) & 0xff);
    DEBUG1("        : OEM/Application ID    : OID = 0x%04x\n", SDHC_CID_REGISTER_OID(&(disk->sd_id)) & 0xffff);
    DEBUG1("        : Product name          : PNM = 0x%02x%02x%02x%02x%02x%02x\n",
                                                               SDHC_CID_REGISTER_PNM(&(disk->sd_id))[0] & 0xff,
                                                               SDHC_CID_REGISTER_PNM(&(disk->sd_id))[1] & 0xff,
                                                               SDHC_CID_REGISTER_PNM(&(disk->sd_id))[2] & 0xff,
                                                               SDHC_CID_REGISTER_PNM(&(disk->sd_id))[3] & 0xff,
                                                               SDHC_CID_REGISTER_PNM(&(disk->sd_id))[4] & 0xff,
                                                               SDHC_CID_REGISTER_PNM(&(disk->sd_id))[5] & 0xff);
    DEBUG1("        : Product revision      : PRV = 0x%02x\n", SDHC_CID_REGISTER_PRV(&(disk->sd_id)) & 0xff);
    DEBUG1("        : Product serial number : PSN = 0x%08x\n", SDHC_CID_REGISTER_PSN(&(disk->sd_id)) & 0xffffffff);
    DEBUG1("        : Manufacturing date    : MDT = 0x%02x\n", SDHC_CID_REGISTER_MDT(&(disk->sd_id)) & 0xff);
    DEBUG1("        : 7-bit CRC checksum    : CRC = 0x%02x\n", SDHC_CID_REGISTER_CRC(&(disk->sd_id)) & 0xff);
#endif


    DEBUG2("CSD data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",   \
           csd.csd_data[ 0], csd.csd_data[ 1], csd.csd_data[ 2], csd.csd_data[3],                           \
           csd.csd_data[ 4], csd.csd_data[ 5], csd.csd_data[ 6], csd.csd_data[7],                           \
           csd.csd_data[ 8], csd.csd_data[ 9], csd.csd_data[10], csd.csd_data[11],                          \
           csd.csd_data[12], csd.csd_data[13], csd.csd_data[14], csd.csd_data[15]);

    disk->sd_csd_version = SDHC_CSD_REGISTER_CSD_STRUCTURE(&csd) + 1 ;

    // Optionally dump the whole CSD register. This takes a lot of
    // code but gives a lot of info about the card. If the info looks
    // correct then we really are interacting properly with an MMC card.
#if DEBUG > 0
    DEBUG1("CSD data: structure 0x%02x, version 0x%02x\n", SDHC_CSD_REGISTER_CSD_STRUCTURE(&csd), SDHC_CSD_REGISTER_SPEC_VERS(&csd));
    if (0 != SDHC_CSD_REGISTER_FILE_FORMAT_GROUP(&csd)) {
        DEBUG1("        : Reserved (unknown), FILE_FORMAT_GROUP %d, FILE_FORMAT %d\n", \
                    SDHC_CSD_REGISTER_FILE_FORMAT_GROUP(&csd), SDHC_CSD_REGISTER_FILE_FORMAT(&csd));
    } else if (0 == SDHC_CSD_REGISTER_FILE_FORMAT(&csd)) {
        DEBUG1("        : Partioned disk, FILE_FORMAT_GROUP 0, FILE_FORMAT 0\n");
    } else if (1 == SDHC_CSD_REGISTER_FILE_FORMAT(&csd)) {
        DEBUG1("        : FAT disk, FILE_FORMAT_GROUP 0, FILE_FORMAT 1\n");
    } else if (2 == SDHC_CSD_REGISTER_FILE_FORMAT(&csd)) {
        DEBUG1("        : Universal File format, FILE_FORMAT_GROUP 0, FILE_FORMAT 2\n");
    } else {
        DEBUG1("        : Others/Unknown disk, FILE_FORMAT_GROUP 0, FILE_FORMAT 3\n");
    }

    {
        static const cyg_uint32 mantissa_speeds_x10[16]   = { 0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80 };
        static const cyg_uint32 exponent_speeds_div10[8]  = { 10000, 100000, 1000000, 10000000, 0, 0, 0, 0 };
        cyg_uint32 speed = mantissa_speeds_x10[SDHC_CSD_REGISTER_TRAN_SPEED_MANTISSA(&csd)] *
            exponent_speeds_div10[SDHC_CSD_REGISTER_TRAN_SPEED_EXPONENT(&csd)];
        speed /= 1000;
        DEBUG1("        : TRAN_SPEED %d %d -> %d kbit/s\n", \
               SDHC_CSD_REGISTER_TRAN_SPEED_MANTISSA(&csd), SDHC_CSD_REGISTER_TRAN_SPEED_EXPONENT(&csd), speed);
    }

    DEBUG1("        : READ_BL_LEN block length 2^%d (%d)\n", SDHC_CSD_REGISTER_READ_BL_LEN(&csd), \
                0x01 << SDHC_CSD_REGISTER_READ_BL_LEN(&csd));

    if (1 == disk->sd_csd_version) {
      DEBUG1("        : C_SIZE %d, C_SIZE_MULT %d\n", \
          SDHC_CSD_REGISTER_C_SIZE(&csd), SDHC_CSD_REGISTER_C_SIZE_MULT(&csd));
      {
        cyg_uint32 block_len = 0x01 << SDHC_CSD_REGISTER_READ_BL_LEN(&csd);
        cyg_uint32 mult      = 0x01 << (SDHC_CSD_REGISTER_C_SIZE_MULT(&csd) + 2);
        cyg_uint32 size      = block_len * mult * (SDHC_CSD_REGISTER_C_SIZE(&csd) + 1);
        cyg_uint32 sizeK     = (cyg_uint32) (size / 1024);
        cyg_uint32 sizeM     =  sizeK / 1024;
        sizeK  -= (sizeM * 1024);
        DEBUG1("        : total card size %dM%dK\n", sizeM, sizeK);
      }
    }
    else {
      DEBUG1("        : C_SIZE %d\n", SD_CSD_V2_REGISTER_C_SIZE(&csd));
      {
        cyg_uint32 sizeK     = 512 * (SD_CSD_V2_REGISTER_C_SIZE(&csd) + 1 ) ;
        cyg_uint32 sizeM     =  sizeK / 1024;
        sizeK  -= (sizeM * 1024);
        DEBUG1("        : total card size %uM%uK\n", sizeM, sizeK);
      }
    }

    DEBUG1("        : WR_BL_LEN block length 2^%d (%d)\n", \
           SDHC_CSD_REGISTER_WRITE_BL_LEN(&csd), 0x01 << SDHC_CSD_REGISTER_WRITE_BL_LEN(&csd));

    if ( 1 == disk->sd_csd_version) {
        static cyg_uint32 taac_mantissa_speeds_x10[16]   = { 0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80 };
        static cyg_uint32 taac_exponent_speeds_div10[8]  = { 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000 };
        cyg_uint32 taac_speed = taac_mantissa_speeds_x10[SDHC_CSD_REGISTER_TAAC_MANTISSA(&csd)] *
            taac_exponent_speeds_div10[SDHC_CSD_REGISTER_TAAC_EXPONENT(&csd)];
        taac_speed /= 100;
        DEBUG1("        : asynchronous read access time TAAC %d %d -> %d ns\n", \
               SDHC_CSD_REGISTER_TAAC_MANTISSA(&csd), SDHC_CSD_REGISTER_TAAC_EXPONENT(&csd), taac_speed);
    }
    if ( 1 == disk->sd_csd_version) {
      DEBUG1("        : synchronous read access time NSAC %d * 100 cycles\n", \
             SDHC_CSD_REGISTER_NSAC(&csd));
    }
    DEBUG1("        : typical write program time %d * read time\n", SDHC_CSD_REGISTER_R2W_FACTOR(&csd));
    DEBUG1("        : CCC command classes 0x%04x\n", SDHC_CSD_REGISTER_CCC(&csd));
    DEBUG1("        : READ_BL_PARTIAL %d, WRITE_BLK_MISALIGN %d, READ_BLK_MISALIGN %d, DSR_IMP %d\n",   \
           SDHC_CSD_REGISTER_READ_BL_PARTIAL(&csd), SDHC_CSD_REGISTER_WRITE_BLK_MISALIGN(&csd),           \
           SDHC_CSD_REGISTER_READ_BLK_MISALIGN(&csd), SDHC_CSD_REGISTER_DSR_IMP(&csd));
    DEBUG1("        : WR_BL_PARTIAL %d\n", SDHC_CSD_REGISTER_WR_BL_PARTIAL(&csd));

    if ( 1 == disk->sd_csd_version) {
        static cyg_uint8    min_currents[8] = { 1, 1, 5, 10, 25, 35, 60, 100 };
        static cyg_uint8    max_currents[8] = { 1, 5, 10, 25, 35, 45, 80, 200 };
        DEBUG1("        : read current min %dmA, max %dmA\n",               \
                    min_currents[SDHC_CSD_REGISTER_VDD_R_CURR_MIN(&csd)],    \
                    max_currents[SDHC_CSD_REGISTER_VDD_R_CURR_MAX(&csd)]);
        DEBUG1("        : write current min %dmA, max %dmA\n",              \
                    min_currents[SDHC_CSD_REGISTER_VDD_W_CURR_MIN(&csd)],    \
                    max_currents[SDHC_CSD_REGISTER_VDD_W_CURR_MAX(&csd)]);
    }
    if ( 1 == disk->sd_csd_version) {
      DEBUG1("        : erase sector size %d, erase group size %d\n", \
             SDHC_CSD_REGISTER_SECTOR_SIZE(&csd) + 1, SDHC_CSD_REGISTER_ERASE_GRP_SIZE(&csd) + 1);
      DEBUG1("        : write group enable %d, write group size %d\n", \
             SDHC_CSD_REGISTER_WR_GRP_ENABLE(&csd), SDHC_CSD_REGISTER_WR_GRP_SIZE(&csd) + 1);
    }
    DEBUG1("        : copy bit %d\n", SDHC_CSD_REGISTER_COPY(&csd));
    DEBUG1("        : permanent write protect %d, temporary write protect %d\n", \
           SDHC_CSD_REGISTER_PERM_WRITE_PROTECT(&csd), SDHC_CSD_REGISTER_TMP_WRITE_PROTECT(&csd));
    if ( 1 == disk->sd_csd_version) {
      DEBUG1("        : ecc %d, default ecc %d\n", SDHC_CSD_REGISTER_ECC(&csd), SDHC_CSD_REGISTER_DEFAULT_ECC(&csd));
    }
    DEBUG1("        : crc 0x%08x\n", SDHC_CSD_REGISTER_CRC(&csd));
#endif

    if ( 1 == disk->sd_csd_version) {
      // There is information available about the file format, e.g.
      // partitioned vs. simple FAT. With the current version of the
      // generic disk code this needs to be known statically, via
      // the mbr field of the disk channel structure. If the card
      // is inappropriately formatted, reject the mount request.
      if ((0 != SDHC_CSD_REGISTER_FILE_FORMAT_GROUP(&csd)) ||
          (0 != SDHC_CSD_REGISTER_FILE_FORMAT(&csd))) {
          return -ENOTDIR;
      }
    } // According to Spec V3.01, host should not use these two fields in CSD V2 cards

    // Look for a write-protect bit (permanent or temporary), and set
    // the disk as read-only or read-write as appropriate. The
    // temporary write-protect could be cleared by rewriting the CSD
    // register (including recalculating the CRC) but the effort
    // involves does not seem worth-while.
    if ((0 != SDHC_CSD_REGISTER_PERM_WRITE_PROTECT(&csd)) || (0 != SDHC_CSD_REGISTER_TMP_WRITE_PROTECT(&csd))) {
        disk->sd_read_only   = true;
    } else {
        disk->sd_read_only   = false;
    }
    DEBUG1("Disk read-only flag %d\n", disk->sd_read_only);

    // Calculate the disk size, primarily for assertion purposes.
    if ( 1 == disk->sd_csd_version) {
      // By design SDHC cards are limited to 4GB, which still doesn't
      // quite fit into 32 bits.
      disk->sd_block_count = (((cyg_uint64)(0x01 << SDHC_CSD_REGISTER_READ_BL_LEN(&csd))) *
             ((cyg_uint64)(0x01 << (SDHC_CSD_REGISTER_C_SIZE_MULT(&csd) + 2))) *
             ((cyg_uint64)(SDHC_CSD_REGISTER_C_SIZE(&csd) + 1))) / (cyg_uint64)USDHC_BLOCK_SIZE;
    }
    else {
      disk->sd_block_count = 1024 * ( SD_CSD_V2_REGISTER_C_SIZE(&csd) + 1 ) ;
    }
    DEBUG1("Disk blockcount %u (0x%08x)\n", disk->sd_block_count, disk->sd_block_count);

    // Assume for now that the block length is 512 bytes. This is
    // probably a safe assumption since we have just got the card
    // initialized out of idle state. If it ever proves to be a problem
    // the SET_BLOCK_LEN command can be used.
    // Nevertheless store the underlying block sizes
    disk->sd_read_block_length  = 0x01 << SDHC_CSD_REGISTER_READ_BL_LEN(&csd);
    disk->sd_write_block_length = 0x01 << SDHC_CSD_REGISTER_WRITE_BL_LEN(&csd);

    // Read the partition table off the card. This is a way of
    // checking that the card is not password-locked. It also
    // provides information about the "disk geometry" which is
    // needed by higher-level code.
    // FIXME: the higher-level code should be made to use LBA
    // addressing instead.
    {
        cyg_uint8   data[USDHC_BLOCK_SIZE];
        cyg_uint8*  partition;
        cyg_uint32  lba_first, lba_size, lba_end, head, cylinder, sector;

        usdhc_data_read(USDHC_INSTANCE, data, USDHC_BLOCK_SIZE, 0);

#if DEBUG > 1
        {
            cyg_uint8 *ptr_data;

            DEBUG2("MBR dump\n");
            for (i = 0; i < USDHC_BLOCK_SIZE; i += 16) {
                ptr_data = &data[i];
                DEBUG2(" %04x: %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x\n",
                    i,
                    ptr_data[ 0], ptr_data[ 1], ptr_data[ 2], ptr_data[ 3],
                    ptr_data[ 4], ptr_data[ 5], ptr_data[ 6], ptr_data[ 7],
                    ptr_data[ 8], ptr_data[ 9], ptr_data[10], ptr_data[11],
                    ptr_data[12], ptr_data[13], ptr_data[14], ptr_data[15]);
            }
        }
#endif
#if DEBUG > 0
        DEBUG1("Read block 0 (partition table)\n");
        DEBUG1("Signature 0x%02x 0x%02x, should be 0x55 0xaa\n", data[0x1fe], data[0x1ff]);
        // There should be four 16-byte partition table entries at offsets
        // 0x1be, 0x1ce, 0x1de and 0x1ee. The numbers are stored little-endian
        for (i = 0; i < 4; i++) {
            partition = &(data[0x1be + (0x10 * i)]);
            DEBUG1("Partition %d: boot %02x, first CHS %02x, last CHS %02x, first sector %02x %02x %02x, file system %02x, last sector %02x %02x %02x\n", i,   \
		   partition[0], \
		   ((partition[2] & 0xC0) << 2) | partition[3], ((partition[6] & 0xC0) << 2) | partition[7], \
                   partition[1], partition[2], partition[3], partition[4], \
                   partition[5], partition[6], partition[7]);
            DEBUG1("           : first sector (linear) %02x %02x %02x %02x, sector count %02x %02x %02x %02x\n", \
                   partition[11], partition[10], partition[9], partition[8], \
                   partition[15], partition[14], partition[13], partition[12]);
        }
#endif
        if ((0x0055 != data[0x1fe]) || (0x00aa != data[0x1ff])) {
            return -ENOTDIR;
        }
        partition   = &(data[0x1be]);
        lba_first   = (partition[11] << 24) | (partition[10] << 16) | (partition[9] << 8) | partition[8];
        lba_size    = (partition[15] << 24) | (partition[14] << 16) | (partition[13] << 8) | partition[12];
        lba_end     = lba_first + lba_size - 1;

        // First sector in c/h/s format
        cylinder    = ((partition[2] & 0xC0) << 2) | partition[3];
        head        = partition[1];
        sector      = partition[2] & 0x3F;

        // lba_start == (((cylinder * Nh) + head) * Ns) + sector - 1, where (Nh == heads/cylinder) and (Ns == sectors/head)
        // Strictly speaking we should be solving some simultaneous
        // equations here for lba_start/lba_end, but that gets messy.
        // The first partition is at the start of the card so cylinder will be 0,
        // and we can ignore Nh.
        CYG_ASSERT(0 == cylinder, "Driver assumption - partition 0 is at start of card\n");
        CYG_ASSERT(0 != head,     "Driver assumption - partition table is sensible\n");
        disk->sd_sectors_per_head = ((lba_first + 1) - sector) / head;

        // Now for lba_end.
        cylinder    = ((partition[6] & 0xC0) << 2) | partition[7];
        head        = partition[5];
        sector      = partition[6] & 0x3F;
        disk->sd_heads_per_cylinder = ((((lba_end + 1) - sector) / disk->sd_sectors_per_head) - head) / cylinder;
    }

    return ENOERR;
}

// The driver-level SDHC operations

static cyg_bool
usdhc_disk_init(struct cyg_devtab_entry* tab)
{
    disk_channel*   chan    = (disk_channel*) tab->priv;

    /* Initialize uSDHC Controller */
    usdhc_host_init(USDHC_INSTANCE);

    /* Software Reset to Interface Controller */
    usdhc_host_reset(USDHC_INSTANCE, ESDHC_ONE_BIT_SUPPORT, ESDHC_LITTLE_ENDIAN_MODE);

    return (*chan->callbacks->disk_init)(tab);
}

static char*
usdhc_disk_lookup_itoa(cyg_uint32 num, char* where)
{
    if (0 == num) {
        *where++ = '0';
    } else {
        char local[10];  // 2^32 just fits into 10 places
        int  index = 9;
        while (num > 0) {
            local[index--] = (num % 10) + '0';
            num /= 10;
        }
        for (index += 1; index < 10; index++) {
            *where++ = local[index];
        }
    }
    return where;
}

static Cyg_ErrNo
usdhc_disk_lookup(struct cyg_devtab_entry** tab, struct cyg_devtab_entry *sub_tab, const char* name)
{
    disk_channel*               chan    = (disk_channel*) (*tab)->priv;
    cyg_usdhc_disk_info_t*    	disk    = (cyg_usdhc_disk_info_t*) chan->dev_priv;
    Cyg_ErrNo                   result;

    DEBUG2("%s(): target name=%s\n", __FUNCTION__, name );
    DEBUG2("                     : device name=%s dep_name=%s\n", tab[0]->name, tab[0]->dep_name );
//    DEBUG2("                     : sub    name=%s dep_name=%s\n", sub_tab->name, sub_tab->dep_name );

    if (disk->sd_connected) {
        // There was a card plugged in last time we looked. Is it still there?
        if (usdhc_disk_changed(disk)) {
            // The old card is gone. Either there is no card plugged in, or
            // it has been replaced with a different one. If the latter the
            // existing mounts must be removed before anything sensible
            // can be done.
            disk->sd_connected = false;
            (*chan->callbacks->disk_disconnected)(chan);
            if (0 != chan->info->mounts) {
                return -ENODEV;
            }
        }
    }

    if ((0 != chan->info->mounts) && !disk->sd_connected) {
        // There are still mount points to an old card. We cannot accept
        // new mount requests until those have been cleaned out.
        return -ENODEV;
    }

    if (!disk->sd_connected) {
        cyg_disk_identify_t ident;
        cyg_uint32          id_data;
        char*               where;
        int                 i;

        // The world is consistent and the higher-level code does not
        // know anything about the current card, if any. Is there a
        // card?
        result = usdhc_check_for_disk(disk);
        if (ENOERR != result) {
            return result;
        }
        // A card has been found. Tell the higher-level code about it.
        // This requires an identify structure, although it is not
        // entirely clear what purpose that serves.
        disk->sd_connected = true;
        // Serial number, up to 20 characters; The CID register contains
        // various fields which can be used for this.
        where   = &(ident.serial[0]);
        id_data = disk->sd_id.cid_data[0];   // 1-byte manufacturer id -> 3 chars, 17 left
        where   = usdhc_disk_lookup_itoa(id_data, where);
        id_data = (disk->sd_id.cid_data[1] << 8) + disk->sd_id.cid_data[2]; // 2-byte OEM ID, 5 chars, 12 left
        where   = usdhc_disk_lookup_itoa(id_data, where);
        id_data = (disk->sd_id.cid_data[10] << 24) + (disk->sd_id.cid_data[11] << 16) +
            (disk->sd_id.cid_data[12] << 8) + disk->sd_id.cid_data[13];
        where   = usdhc_disk_lookup_itoa(id_data, where); // 4-byte OEM ID, 10 chars, 2 left
        // And terminate the string with a couple of places to spare.
        *where = '\0';

        // Firmware revision number. There is a one-byte product
        // revision number in the CID, BCD-encoded
        id_data = disk->sd_id.cid_data[9] >> 4;
        if (id_data <= 9) {
            ident.firmware_rev[0] = id_data + '0';
        } else {
            ident.firmware_rev[0] = id_data - 10 + 'A';
        }
        id_data = disk->sd_id.cid_data[9] & 0x0F;
        if (id_data <= 9) {
            ident.firmware_rev[1] = id_data + '0';
        } else {
            ident.firmware_rev[1] = id_data - 10 + 'A';
        }
        ident.firmware_rev[2]   = '\0';

        // Model number. There is a six-byte product name in the CID.
        for (i = 0; i < 6; i++) {
            if ((disk->sd_id.cid_data[i + 3] >= 0x20) && (disk->sd_id.cid_data[i+3] <= 0x7E)) {
                ident.model_num[i] = disk->sd_id.cid_data[i + 3];
            } else {
                break;
            }
        }
        ident.model_num[i] = '\0';

        // We don't have no cylinders, heads, or sectors, but
        // higher-level code may interpret partition data using C/H/S
        // addressing rather than LBA. Hence values for some of these
        // settings were calculated above.
        ident.cylinders_num     = 1;
        ident.heads_num         = disk->sd_heads_per_cylinder;
        ident.sectors_num       = disk->sd_sectors_per_head;
        ident.lba_sectors_num   = disk->sd_block_count;
        ident.phys_block_size   = disk->sd_write_block_length/512;
        ident.max_transfer      = disk->sd_write_block_length;

        DEBUG1("Calling disk_connected(): serial %s, firmware %s, model %s, heads %d, sectors %d, lba_sectors_num %d, phys_block_size %d\n", \
               ident.serial, ident.firmware_rev, ident.model_num, ident.heads_num, ident.sectors_num,
               ident.lba_sectors_num, ident.phys_block_size);
        (*chan->callbacks->disk_connected)(*tab, &ident);

        // We now have a valid card and higher-level code knows about it. Fall through.
    }

    // And leave it to higher-level code to finish the lookup, taking
    // into accounts partitions etc.
    return (*chan->callbacks->disk_lookup)(tab, sub_tab, name);}

static Cyg_ErrNo
sd_usdhc_disk_read(disk_channel* chan, void* buf_arg, cyg_uint32 blocks, cyg_uint32 first_block)
{
    cyg_usdhc_disk_info_t*      disk    = (cyg_usdhc_disk_info_t*) chan->dev_priv;
    cyg_uint32                  i;
    cyg_uint8*                  buf = (cyg_uint8*) buf_arg;
    Cyg_ErrNo                   code = ENOERR;

    DEBUG1("%s(): first block %d, buf %p, len %lu blocks (%lu bytes)\n",
        __FUNCTION__, first_block, buf, (unsigned long)blocks,
        (unsigned long)blocks*512);

    if (! disk->sd_connected) {
        return -ENODEV;
    }
    if ((first_block + blocks) >= disk->sd_block_count) {
        return -EINVAL;
    }

    for (i = 0; (i < blocks) && (ENOERR == code); i++) {
        code = usdhc_data_read(USDHC_INSTANCE, buf, USDHC_BLOCK_SIZE, (first_block + i) * USDHC_BLOCK_SIZE);
        buf += USDHC_BLOCK_SIZE;
    }
    return code;
}

static Cyg_ErrNo
sd_usdhc_disk_write(disk_channel* chan, const void* buf_arg, cyg_uint32 blocks, cyg_uint32 first_block)
{
    cyg_usdhc_disk_info_t*      disk    = (cyg_usdhc_disk_info_t*) chan->dev_priv;
    cyg_uint32                  i;
    const cyg_uint8*            buf = (cyg_uint8*) buf_arg;
    Cyg_ErrNo                   code = ENOERR;

    DEBUG1("%s(): first block %d, buf %p, len %lu blocks (%lu bytes)\n",
        __FUNCTION__, first_block, buf, (unsigned long)blocks,
        (unsigned long)blocks*512);

    if (! disk->sd_connected) {
        return -ENODEV;
    }
    if (disk->sd_read_only) {
        return -EROFS;
    }
    if ((first_block + blocks) >= disk->sd_block_count) {
        return -EINVAL;
    }

    for (i = 0; (i < blocks) && (ENOERR == code); i++) {
        code = usdhc_data_write(USDHC_INSTANCE, buf, USDHC_BLOCK_SIZE, (first_block + i) * USDHC_BLOCK_SIZE);
        buf += USDHC_BLOCK_SIZE;
    }
    return code;
}

// get_config() and set_config(). There are no supported get_config() operations
// at this time.
static Cyg_ErrNo
sd_usdhc_disk_get_config(disk_channel* chan, cyg_uint32 key, const void* buf, cyg_uint32* len)
{
    return -EINVAL;
}

static Cyg_ErrNo
sd_usdhc_disk_set_config(disk_channel* chan, cyg_uint32 key, const void* buf, cyg_uint32* len)
{
    return 0;
}

DISK_FUNS(cyg_usdhc_disk_funs,
          sd_usdhc_disk_read,
          sd_usdhc_disk_write,
          sd_usdhc_disk_get_config,
          sd_usdhc_disk_set_config
          );

static cyg_usdhc_disk_info_t cyg_usdhc_disk0_hwinfo = {
//    .usdhc_dev        = &cyg_usdhc_dev0,
#ifdef USDHC_BACKGROUND_WRITES
    .sd_writing        = 0,
#endif
    .sd_connected      = 0
};

// No h/w controller structure is needed, but the address of the
// second argument is taken anyway.
DISK_CONTROLLER(cyg_usdhc_disk_controller_0, cyg_usdhc_disk0_hwinfo);

DISK_CHANNEL(cyg_usdhc_disk0_channel,
             cyg_usdhc_disk_funs,
             cyg_usdhc_disk0_hwinfo,
             cyg_usdhc_disk_controller_0,
             true,                            /* MBR support */
             1                                /* Number of partitions supported */
             );

BLOCK_DEVTAB_ENTRY(cyg_usdhc_disk0_devtab_entry,
                   CYGDAT_DEVS_DISK_USDHC_DISK0_NAME,
                   0,
                   &cyg_io_disk_devio,
                   &usdhc_disk_init,
                   &usdhc_disk_lookup,
                   &cyg_usdhc_disk0_channel);

// EOF devs_disk_freescale_usdhc.c
