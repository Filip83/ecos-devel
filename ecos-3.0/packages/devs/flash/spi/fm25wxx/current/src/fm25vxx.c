//=============================================================================
//
//      fm25vxx.c
//
//      SPI fram driver for Ramtron/Cypress FM25Vxx devices
//      and compatibles.
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2008, 2009, 2011 Free Software Foundation, Inc.
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   Filip
// Original(s): Chris Holgate
// Date:        2011-04-25
// Purpose:     Ramtron/Cypress FM25Vxx devices SPI fram driver
//              implementation
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_diag.h>

#include <cyg/io/spi.h>
#include <cyg/io/flash.h>
#include <cyg/io/flash_dev.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>

#include <pkgconf/devs_flash_spi_fm25vxx.h>

#include <string.h>

//-----------------------------------------------------------------------------
// Enable polled SPI operation for non-kernel builds.

#ifdef CYGPKG_KERNEL
# define FM25VXX_POLLED false
#else
# define FM25VXX_POLLED true
#endif

//-----------------------------------------------------------------------------
// Implement delay functions for kernel and non-kernel builds.  The kernel
// build assumes that the API calls are made in the thread context.

#ifdef CYGPKG_KERNEL
#define FM25VXX_DELAY_MS(_msdelay_) hal_delay_us(10)

#else
#define FM25VXX_DELAY_MS(_msdelay_) CYGACC_CALL_IF_DELAY_US (_msdelay_ * 1000)
#endif

// Byte / word programming takes 10 us max
#define FM25VXX_WAIT_CHIP_READY( dev )                  \
{                                                       \
    cyg_uint8 dev_status;                               \
    do {                                                \
        HAL_DELAY_US (10);                              \
        dev_status = fm25vxx_spi_rdsr ( dev );          \
    } while ( dev_status & FM25VXX_STATUS_BSY );        \
}

//-----------------------------------------------------------------------------
// Maintenance and debug macros.

#define ASSERT_FM25VXX(_test_, _msg_) CYG_ASSERT(_test_, "FAIL (FM25VXX) : " _msg_)
#define TRACE_FM25VXX(_msg_, _args_...) if (dev->pf) dev->pf ("FM25VXX : " _msg_, ##_args_)

//=============================================================================
// Define SST25VFxxx SPI protocol.
//=============================================================================

typedef enum fm25vxx_cmd {
    FM25VXX_CMD_WREN   = 0x06,     // Write enable.
    FM25VXX_CMD_WDRI   = 0x04,     // Write disable.
    FM25VXX_CMD_RDSR   = 0x05,     // Read status register.
    FM25VXX_CMD_WRSR   = 0x01,     // Write status register.
    FM25VXX_CMD_READ   = 0x03,     // Read data
    FM25VXX_CMD_FSTRD  = 0x0C,     // Read data fast
    FM25VXX_CMD_WRITE  = 0x02,     // Byte program.
    FM25VXX_CMD_RDJID  = 0x9F,     // Read JEDEC identification.
} fm25vxx_cmd;

// Status register bitfields.
#define FM25VXX_STATUS_WEL  0x02   /* Write enable latch. */
#define FM25VXX_STATUS_BP0  0x04   /* Block Write Protect 0. */
#define FM25VXX_STATUS_BP1  0x08   /* Block Write Protect 1. */
#define FM25VXX_STATUS_BP2  0x10   /* Block Write Protect 2. */
#define FM25VXX_STATUS_BP3  0x20   /* Block Write Protect 3. */
#define FM25VXX_STATUS_BP4  0x40   /* Block Write Protect 4. */
#define FM25VXX_STATUS_WPEN 0x80   /* Sector Protect Register Bit Lock. */

#define FM25VXX_LOCK_BITS   ( FM25VXX_STATUS_BP0 | FM25VXX_STATUS_BP1 | \
                              FM25VXX_STATUS_BP2 | FM25VXX_STATUS_BP3 )

#define FM25VXX_MEMORY_ID   0x25



//=============================================================================
// Array containing a list of supported devices.  This allows the device
// parameters to be dynamically detected on initialization.
//=============================================================================

typedef struct fm25vxx_params {
    cyg_uint32 size;                  // FRAM size(s) in Byte
    cyg_uint32 jedec_id;              // 3 byte JEDEC identifier for this device.
} fm25vxx_params;

static const fm25vxx_params fm25vxx_supported_devices [] = {
    { // Support for 128 KBit devices (FM25V01).
        size                : 16384,
        jedec_id            : 0x007fc221
    },
    { // Support for 256 KBit devices (FM25V02).
        size                : 32768,
        jedec_id            : 0x007fc222
    },
    { // Support for 512 KBit devices (FM25V05).
        size                : 65536,
        jedec_id            : 0x007fc223
    },
    { // Support for 1024 KBit devices (FM25V10).
        size                : 131072,
        jedec_id            : 0x007fc224
    },
    { // Support for 2048 KBit devices (FM25V20).
        size                : 262144,
        jedec_id            : 0x007fc225
    },
    { // Null terminating entry.
        size                : 0,
        jedec_id            : 0
    }
};

//=============================================================================
// Utility functions for address calculations.
//=============================================================================

//-----------------------------------------------------------------------------
// Strips out any device address offset to give address within device.

static cyg_bool
fm25vxx_to_local_addr(struct cyg_flash_dev *dev, cyg_flashaddr_t * addr)
{
    cyg_bool        retval = false;

    // Range check address before modifying it.
    if ((*addr >= dev->start) && (*addr <= dev->end)) 
    {
        *addr -= dev->start;
        retval = true;
    }
    return retval;
}

//=============================================================================
// Wrapper functions for various SPI transactions.
//=============================================================================

//-----------------------------------------------------------------------------
// Read back the 3-byte JEDEC ID, returning it as a 32-bit integer.
// This function is called during flash initialization, which can often be
// called from the startup/idle thread.  This means that we should always use
// SPI polled mode in order to prevent the thread from attempting to sleep.

static  cyg_uint32
fm25vxx_spi_rdid(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[10] = { FM25VXX_CMD_RDID, 0x0, 0x0, 0x0, 0x00,
                                               0x0 ,0x0, 0x0, 0x0, 0x00};
    cyg_uint8       rx_buf[10];
    cyg_uint32      retval = 0;

    // Carry out SPI transfer.
    cyg_spi_transfer(spi_device, FM25VXX_POLLED, 10, tx_buf, rx_buf);

    // Convert 9-byte ID to 32-bit Jedec ID.
    retval |= ((cyg_uint32)rx_buf[6]) << 16;
    retval |= (((cyg_uint32)rx_buf[7]) << 8);
    retval |= ((cyg_uint32)rx_buf[8]);

    return retval;
}

//-----------------------------------------------------------------------------
// Send write enable command.

static  void
fm25vxx_spi_wren(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[1] = { FM25VXX_CMD_WREN };
    cyg_spi_transfer(spi_device, FM25VXX_POLLED, 1, tx_buf, NULL);
}

//-----------------------------------------------------------------------------
// Send write disable command.

static  void
fm25vxx_spi_wrdis(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[1] = { FM25VXX_CMD_WDRI };
    cyg_spi_transfer(spi_device, FM25VXX_POLLED, 1, tx_buf, NULL);
}

//-----------------------------------------------------------------------------
// Send sector erase command.  The address parameter is a device local address
// within the sector to be erased. Thri fram device does not have such
// a command. Used only fro convenience

static  void
fm25vxx_spi_se(struct cyg_flash_dev *dev, cyg_flashaddr_t addr)
{
}

//-----------------------------------------------------------------------------
// Read and return the 8-bit device status register.

static cyg_uint8
fm25vxx_spi_rdsr(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[2] = { FM25VXX_CMD_RDSR, 0 };
    cyg_uint8       rx_buf[2];

    // Carry out SPI transfer and return the status byte.
    cyg_spi_transfer(spi_device, FM25VXX_POLLED, 2, tx_buf, rx_buf);

    return rx_buf[1];
}

//-----------------------------------------------------------------------------
// Program a single page.

static  void fm25vxx_spi_pp
  (struct cyg_flash_dev *dev, cyg_flashaddr_t addr, cyg_uint8* wbuf, cyg_uint32 wbuf_len)
{
    cyg_spi_device* spi_device = (cyg_spi_device*) dev->priv;
    const cyg_uint8 tx_buf [3] =
    { 
        FM25VXX_CMD_WRITE,
        (cyg_uint8) (addr >> 8),
        (cyg_uint8) (addr) 
    };

    // Implement the program operation as a multistage SPI transaction.
    cyg_spi_transaction_begin (spi_device);
    cyg_spi_transaction_transfer (spi_device, FM25VXX_POLLED, 3, tx_buf, NULL, false);
    cyg_spi_transaction_transfer (spi_device, FM25VXX_POLLED, wbuf_len, wbuf, NULL, true);
    cyg_spi_transaction_end (spi_device);
}


//-----------------------------------------------------------------------------
// Implement reads to the specified buffer.

static  void
fm25vxx_spi_read(struct cyg_flash_dev *dev, cyg_flashaddr_t addr,
                 cyg_uint8 *rbuf, cyg_uint32 rbuf_len)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[3] = 
    { 
        FM25VXX_CMD_READ,
        (cyg_uint8)(addr >> 8), 
        (cyg_uint8)(addr)
    };

    // Implement the read operation as a multistage SPI transaction.
    cyg_spi_transaction_begin(spi_device);
    cyg_spi_transaction_transfer(spi_device, FM25VXX_POLLED,
                                 3, tx_buf, NULL, false);
                                 
    cyg_spi_transaction_transfer(spi_device, FM25VXX_POLLED, rbuf_len, NULL,
                                 rbuf, true);
    cyg_spi_transaction_end(spi_device);
}


//=============================================================================
// Standard Flash device API.  All the following functions assume that a valid
// SPI device handle is passed in the 'priv' reference of the flash device
// data structure.
//=============================================================================

//-----------------------------------------------------------------------------
// Initialize the SPI flash, reading back the flash parameters.

static int
fm25vxx_init(struct cyg_flash_dev *dev)
{
    fm25vxx_params *dev_params = (fm25vxx_params *) fm25vxx_supported_devices;
    cyg_uint32      device_id;
    cyg_uint8       i = 0;

    int             retval = FLASH_ERR_INVALID;

#if CYGNUM_DEVS_FLASH_SPI_FM25VXX_SIZE == 0

    dev->end = dev->start + CYGNUM_DEVS_FLASH_SPI_FM25VXX_SIZE - 1;

    // Strictly speaking the block info fields are 'read only'.
    // However, we have a legitimate reason for updating the contents
    // here and can cast away the const.
    ((cyg_flash_block_info_t *) dev->block_info)->block_size =
        CYGNUM_DEVS_FLASH_SPI_FM25VXX_SIZE;
    ((cyg_flash_block_info_t *) dev->block_info)->blocks =
        1;

    retval = FLASH_ERR_OK;
#else
    // Find the device in the supported devices list.
    device_id = fm25vxx_spi_rdid(dev);

    while ((dev_params->jedec_id != 0) && (dev_params->jedec_id != device_id))
    {
        dev_params++;
    }

    // Found supported device - update device parameters.  N25QXX devices have
    // a uniform sector distribution, so only 1 block info record is required.
    if (dev_params->jedec_id != 0) 
    {
        ASSERT_FM25VXX(dev->num_block_infos == 1,
                       "Only 1 block info record required.");
        ASSERT_FM25VXX(dev->block_info != NULL,
                       "Null pointer to block info record.");

        if ((dev->num_block_infos == 1) && (dev->block_info != NULL))
        {
            TRACE_FM25VXX("Init device with JEDEC ID 0x%06X.\n", device_id);
            dev->end = dev->start + dev_params->size - 1;

            // Strictly speaking the block info fields are 'read only'.
            // However, we have a legitimate reason for updating the contents
            // here and can cast away the const.
            ((cyg_flash_block_info_t *) dev->block_info)->block_size =
                (size_t)(dev_params->size);
            ((cyg_flash_block_info_t *) dev->block_info)->blocks = 1;

            retval = FLASH_ERR_OK;
        }
    }
#endif
    return retval;
}

//-----------------------------------------------------------------------------
// Erase a single sector of the flash. Fram device does not support 
// errase. Data are natively rewriten.

static int
fm25vxx_erase_block(struct cyg_flash_dev *dev, cyg_flashaddr_t block_base)
{
    return FLASH_ERR_OK;
}

static int fm25vxx_verifi(struct cyg_flash_dev *dev, cyg_flashaddr_t local_base,
                         const void *data, size_t len)
{
    cyg_uint32 i;
    cyg_uint8  verifi_buf[100];
    cyg_uint32 rx_bytes;
    cyg_uint8  *verifi_data = (cyg_uint8*)data;
    while(len)
    {
        rx_bytes = (len < 100) ? len : 100; 
        fm25vxx_spi_read(dev, local_base, verifi_buf, rx_bytes);
        for(i = 0; i < rx_bytes; i++)
        {
            if(verifi_data[i] != verifi_buf[i])
                return FLASH_ERR_DRV_VERIFY;
        }
        local_base  += rx_bytes;
        verifi_data += rx_bytes;
        len         -= rx_bytes;
    }
    
    return FLASH_ERR_OK;
}
//-----------------------------------------------------------------------------
// Program an arbitrary number of pages into flash and verify written data.

static int
fm25vxx_program(struct cyg_flash_dev *dev, cyg_flashaddr_t base,
                const void *data, size_t len)
{
    cyg_uint8      *tx_ptr = (cyg_uint8 *)data;
    cyg_flashaddr_t local_base = base;

    int             retval = FLASH_ERR_OK;

    // Fix up the block address.
    if (!fm25vxx_to_local_addr(dev, &local_base))
    {
        retval = FLASH_ERR_INVALID;
        goto out;
    }

    fm25vxx_spi_wren(dev);
    fm25vxx_spi_pp(dev,local_base,tx_ptr,len);
    
    retval = fm25vxx_verifi(dev,local_base,tx_ptr,len);
out:
    return retval;
}

//-----------------------------------------------------------------------------
// Read back an arbitrary amount of data from flash.

static int
fm25vxx_read(struct cyg_flash_dev *dev, const cyg_flashaddr_t base,
             void *data, size_t len)
{
    cyg_flashaddr_t local_base = base;
    int             retval = FLASH_ERR_INVALID;
    cyg_uint8      *rx_ptr = (cyg_uint8 *)data;
    cyg_uint32      rx_bytes_left = (cyg_uint32)len;

    // Fix up the block address and fill the read buffer.
    if (fm25vxx_to_local_addr(dev, &local_base)) 
    {
        fm25vxx_spi_read(dev, local_base, rx_ptr, rx_bytes_left);

        retval = FLASH_ERR_OK;
    }
    return retval;
}

//-----------------------------------------------------------------------------
// Lock device

static int
fm25vxx_lock(struct cyg_flash_dev *dev, cyg_flashaddr_t base)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    cyg_flashaddr_t local_base = base;
    int             retval = FLASH_ERR_INVALID;
    const cyg_uint8 tx_buf[2] = { FM25VXX_CMD_WRSR, FM25VXX_LOCK_BITS };

    if (fm25vxx_to_local_addr(dev, &local_base)) 
    {
        fm25vxx_spi_wren(dev);
        cyg_spi_transfer(spi_device, FM25VXX_POLLED, 2, tx_buf, NULL);
        retval = FLASH_ERR_OK;
    }
    return retval;
}

//-----------------------------------------------------------------------------
// Unlock device

static int
fm25vxx_unlock(struct cyg_flash_dev *dev, cyg_flashaddr_t base)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    cyg_flashaddr_t local_base = base;
    int             retval = FLASH_ERR_INVALID;
    const cyg_uint8 tx_buf[2] = { FM25VXX_CMD_WRSR, 0x0 };

    if (fm25vxx_to_local_addr(dev, &local_base)) 
    {
        fm25vxx_spi_wren(dev);
        cyg_spi_transfer(spi_device, FM25VXX_POLLED, 2, tx_buf, NULL);
        retval = FLASH_ERR_OK;
    }
    return retval;
}

//=============================================================================
// Fill in the driver data structures.
//=============================================================================

CYG_FLASH_FUNS (
    cyg_devs_flash_spi_fm25vxx_funs, // Exported name of function pointers.
    fm25vxx_init,                    // Flash initialization.
    cyg_flash_devfn_query_nop,       // Query operations not supported.
    fm25vxx_erase_block,             // Sector erase.
    fm25vxx_program,                 // Program multiple pages.
    fm25vxx_read,                    // Read arbitrary amount of data.
    fm25vxx_lock,                    // Locking (lock the whole device).
    fm25vxx_unlock
);

//-----------------------------------------------------------------------------
// EOF fm25vxx.c
