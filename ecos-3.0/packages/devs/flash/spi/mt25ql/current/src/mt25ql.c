//=============================================================================
//
//      mt25ql.c
//
//      SPI flash driver for Micron MT25QL devices
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
// Date:        2013-04-04
// Purpose:     Micron MT25QL SPI flash driver
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

#include <pkgconf/devs_flash_spi_mt25ql.h>

#include <string.h>

//-----------------------------------------------------------------------------
// Enable polled SPI operation for non-kernel builds.

#ifdef CYGPKG_KERNEL
# define N25QXX_POLLED false
#else
# define N25QXX_POLLED true
#endif

//-----------------------------------------------------------------------------
// Implement delay functions for kernel and non-kernel builds.  The kernel
// build assumes that the API calls are made in the thread context.

#ifdef CYGPKG_KERNEL
#define N25QXX_DELAY_MS(_msdelay_) hal_delay_us(10)
#else
#define N25QXX_DELAY_MS(_msdelay_) CYGACC_CALL_IF_DELAY_US (_msdelay_ * 1000)
#endif

// Byte / word programming takes 15 us max
#define N25QXX_WAIT_CHIP_READY( dev )                   \
{                                                       \
    cyg_uint8 dev_status;                               \
    do {                                                \
        HAL_DELAY_US (15);                              \
        dev_status = mt25ql_spi_rdsr ( dev );           \
    } while ( dev_status & N25QXX_STATUS_BSY );         \
}

//-----------------------------------------------------------------------------
// Maintenance and debug macros.

#define ASSERT_N25QXX(_test_, _msg_) CYG_ASSERT(_test_, "FAIL (N25QXX) : " _msg_)
#define TRACE_N25QXX(_msg_, _args_...) if (dev->pf) dev->pf ("N25QXX : " _msg_, ##_args_)

//=============================================================================
// Define SST25VFxxx SPI protocol.
//=============================================================================

typedef enum mt25ql_cmd {
    N25QXX_CMD_WREN     = 0x06,     // Write enable.
    N25QXX_CMD_WDRI     = 0x04,     // Write disable.
    N25QXX_CMD_RDJID    = 0x9F,     // Read JEDEC identification.
    N25QXX_CMD_RDSR     = 0x05,     // Read status register.
    N25QXX_CMD_WRSR     = 0x01,     // Write status register.
	N25QXX_CMD_RDFSR    = 0x70,     // Read status flag register.
	N25QXX_CMD_WREAR    = 0xC5,     // Write extended address register.
    N25QXX_CMD_READ     = 0x03,     // Read data
    N25QXX_CMD_FREAD    = 0x0B,     // Read data (fast).
    N25QXX_CMD_PP       = 0x02,     // Page program.
    N25QXX_CMD_SE_4K    = 0x20,     // 4K sector erase.
    N25QXX_CMD_SE_32K   = 0x52,     // 32K sector erase.
	N25QXX_CMD_SE_64K   = 0xD8,     // 64K sector erase.
    N25QXX_CMD_DE       = 0xC4,     // Die erase.
	N25QXX_CMD_4BEN     = 0xB7,	    // Enable 4B addressing mode.
	N25QXX_CMD_4BDIS    = 0xE9,	    // Disable 4B addressing mode.
	N25QXX_CMD_4B_READ  = 0x13,     // 4B Read data
	N25QXX_CMD_4B_FREAD = 0x0C,     // 4B Read data (fast).
	N25QXX_CMD_4B_PP    = 0x12,     // 4B Page program.
	N25QXX_CMD_4B_SE_4K = 0x21,     // 4B 4K sector erase.
	N25QXX_CMD_4B_SE_32K = 0x5C,    // 4B 32K sector erase.
	N25QXX_CMD_4B_SE_64K = 0xDC,    // 4B 64K sector erase.

} mt25ql_cmd;

// Status register bitfields.
#define N25QXX_STATUS_BSY  0x01   /* Operation in progress. */
#define N25QXX_STATUS_WEL  0x02   /* Write enable latch. */
#define N25QXX_STATUS_BP0  0x04   /* Block Write Protect 0. */
#define N25QXX_STATUS_BP1  0x08   /* Block Write Protect 1. */
#define N25QXX_STATUS_BP2  0x10   /* Block Write Protect 2. */
#define N25QXX_STATUS_BP3  0x20   /* Block Write Protect 3. */
#define N25QXX_STATUS_TB   0x40   /* Protected memery are from top or bottom */
#define N25QXX_STATUS_SRWE 0x80   /* Status register write enable */

#define N25QXX_LOCK_BITS   ( N25QXX_STATUS_BP0 | N25QXX_STATUS_BP1 | \
                             N25QXX_STATUS_BP2 | N25QXX_STATUS_BP3 )


// A few helper constants
#ifndef SZ_1K
# define SZ_1K           0x00000400
# define SZ_2K           0x00000800
# define SZ_4K           0x00001000
# define SZ_8K           0x00002000
# define SZ_16K          0x00004000
# define SZ_32K          0x00008000
# define SZ_64K          0x00010000
#endif

// Select read OPCODE
# define N25QXX_CMD_READ_LEN    4
# define N25QXX_CMD_READ_OPCODE N25QXX_CMD_READ

// Select sector size
#if CYGPKG_DEVS_FLASH_SPI_N25QXX_BLOCK_SIZE == SZ_64K
# define N25QXX_CMD_SE N25QXX_CMD_SE_64K
#elif CYGPKG_DEVS_FLASH_SPI_N25QXX_BLOCK_SIZE == SZ_32K
# define N25QXX_CMD_SE N25QXX_CMD_SE_32K
#elif CYGPKG_DEVS_FLASH_SPI_N25QXX_BLOCK_SIZE == SZ_4K
# define N25QXX_CMD_SE N25QXX_CMD_SE_4K
#endif

// Device page size
#define N25QXXX_PAGE_SIZE 256

//=============================================================================
// Array containing a list of supported devices.  This allows the device
// parameters to be dynamically detected on initialization.
//=============================================================================

typedef struct mt25ql_params {
    cyg_uint16 sector_count[4];       // Number of sectors on device.
    cyg_uint32 sector_size[4];        // Supported sector size(s) in Byte
    cyg_uint32 jedec_id;              // 3 byte JEDEC identifier for this device.
} mt25ql_params;


static const mt25ql_params mt25ql_supported_devices [] = {
    // Some Micron parts.
    // Note1: 3V variants
    { // Support for SST 2 GBit devices (N25Q00AA13G).
        sector_count        : {65536, 8192,  4096, 0},
        sector_size         : {SZ_4K, SZ_32K, SZ_64K, 0},
        jedec_id            : 0x0020ba22
    },
    { // Support for SST 1 GBit devices (N25Q512A).
        sector_count        : {32768, 4096, 2048, 0},
        sector_size         : {SZ_4K, SZ_32K, SZ_64K, 0},
        jedec_id            : 0x0020ba21
    },
    { // Support for SST 512 MBit devices (N25Q256A).
        sector_count        : {16384, 2048, 1024, 0},
        sector_size         : {SZ_4K, SZ_32K, SZ_64K, 0},
        jedec_id            : 0x0020ba20
    },
    { // Support for SST 256 MBit devices (N25Q128A).
        sector_count        : {8192, 1024, 512, 0},
        sector_size         : {SZ_4K, SZ_32K, SZ_64K, 0},
        jedec_id            : 0x0020ba19
    },
    { // Support for SST 128 MBit devices (N25Q064A).
        sector_count        : {4096, 512, 128, 0},
        sector_size         : {SZ_4K, SZ_32K, SZ_64K, 0},
        jedec_id            : 0x0020ba18
    },
    { // Support for SST 64 MBit devices (N25Q032A).
        sector_count        : {2048, 1024, 64, 0},
        sector_size         : {SZ_4K, SZ_32K, SZ_64K, 0},
        jedec_id            : 0x0020ba17
    },
	// Note1: 1,8V variants
	{ // Support for SST 2 GBit devices (N25Q00AA13G).
		sector_count: {65536, 8192,  4096, 0},
		sector_size : {SZ_4K, SZ_32K, SZ_64K, 0},
		jedec_id : 0x0020bb22
	},
	{ // Support for SST 1 GBit devices (N25Q512A).
		sector_count: {32768, 4096, 2048, 0},
		sector_size : {SZ_4K, SZ_32K, SZ_64K, 0},
		jedec_id : 0x0020bb21
	},
	{ // Support for SST 512 MBit devices (N25Q256A).
		sector_count: {16384, 2048, 1024, 0},
		sector_size : {SZ_4K, SZ_32K, SZ_64K, 0},
		jedec_id : 0x0020bb20
	},
	{ // Support for SST 256 MBit devices (N25Q128A).
		sector_count: {8192, 1024, 512, 0},
		sector_size : {SZ_4K, SZ_32K, SZ_64K, 0},
		jedec_id : 0x0020bb19
	},
	{ // Support for SST 128 MBit devices (N25Q064A).
		sector_count: {4096, 512, 128, 0},
		sector_size : {SZ_4K, SZ_32K, SZ_64K, 0},
		jedec_id : 0x0020bb18
	},
	{ // Support for SST 64 MBit devices (N25Q032A).
		sector_count: {2048, 1024, 64, 0},
		sector_size : {SZ_4K, SZ_32K, SZ_64K, 0},
		jedec_id : 0x0020bb17
	},
    { // Null terminating entry.
        sector_count        : {0},
        sector_size         : {0},
        jedec_id            : 0
    }
};

//=============================================================================
// Utility functions for address calculations.
//=============================================================================

//-----------------------------------------------------------------------------
// Strips out any device address offset to give address within device.

static cyg_bool
mt25ql_to_local_addr(struct cyg_flash_dev *dev, cyg_flashaddr_t * addr)
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
static inline cyg_uint32
mt25ql_spi_rdid(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[4] = { N25QXX_CMD_RDJID, 0x0, 0x0, 0x0 };
    cyg_uint8       rx_buf[4];
    cyg_uint32      retval = 0;

    // Carry out SPI transfer.
    cyg_spi_transfer(spi_device, N25QXX_POLLED , 4, tx_buf, rx_buf);

    // Convert 3-byte ID to 32-bit integer.
    retval |= ((cyg_uint32)rx_buf[1]) << 16;
    retval |= ((cyg_uint32)rx_buf[2]) << 8;
    retval |= ((cyg_uint32)rx_buf[3]);

    return retval;
}

//-----------------------------------------------------------------------------
// Send write enable command.

static inline void
mt25ql_spi_wren(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[1] = { N25QXX_CMD_WREN };
    cyg_spi_transfer(spi_device, N25QXX_POLLED , 1, tx_buf, NULL);
}

//-----------------------------------------------------------------------------
// Send write disable command.

static  void
mt25ql_spi_wrdis(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[1] = { N25QXX_CMD_WDRI };
    cyg_spi_transfer(spi_device, N25QXX_POLLED , 1, tx_buf, NULL);
}

//-----------------------------------------------------------------------------
// Send sector erase command.  The address parameter is a device local address
// within the sector to be erased.

static  void
mt25ql_spi_se(struct cyg_flash_dev *dev, cyg_flashaddr_t addr)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    cyg_uint8       tx_buf[4] = 
    { 
        N25QXX_CMD_SE,
        (cyg_uint8)(addr >> 16), 
        (cyg_uint8)(addr >> 8), 
        (cyg_uint8)(addr)
    };
    cyg_spi_transfer(spi_device, N25QXX_POLLED , 4, tx_buf, NULL);
}

//-----------------------------------------------------------------------------
// Read and return the 8-bit device status register.

static cyg_uint8
mt25ql_spi_rdsr(struct cyg_flash_dev *dev)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[2] = { N25QXX_CMD_RDSR, 0 };
    cyg_uint8       rx_buf[2];

    // Carry out SPI transfer and return the status byte.
    cyg_spi_transfer(spi_device, N25QXX_POLLED , 2, tx_buf, rx_buf);

    return rx_buf[1];
}

//-----------------------------------------------------------------------------
// Read and return the 8-bit device flag status register.

static cyg_uint8
mt25ql_spi_rdfsr(struct cyg_flash_dev* dev)
{
	cyg_spi_device* spi_device = (cyg_spi_device*)dev->priv;
	const cyg_uint8 tx_buf[2] = { N25QXX_CMD_RDFSR, 0 };
	cyg_uint8       rx_buf[2];

	// Carry out SPI transfer and return the status byte.
	cyg_spi_transfer(spi_device, N25QXX_POLLED, 2, tx_buf, rx_buf);

	return rx_buf[1];
}
//-----------------------------------------------------------------------------
// Program a single byte.

static  void
mt25ql_spi_pp
(struct cyg_flash_dev *dev, cyg_flashaddr_t addr, cyg_uint8* wbuf, cyg_uint32 wbuf_len)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[4] =
    { 
        N25QXX_CMD_PP,
        (cyg_uint8)(addr >> 16), 
        (cyg_uint8)(addr >> 8), 
        (cyg_uint8)(addr)
    };

    // Implement the program operation as a multistage SPI transaction.
    cyg_spi_transaction_begin(spi_device);
    cyg_spi_transaction_transfer(spi_device, N25QXX_POLLED , 4, tx_buf, NULL,
                                 false);
    cyg_spi_transaction_transfer(spi_device, N25QXX_POLLED , wbuf_len, wbuf, NULL,
                                 true);
    cyg_spi_transaction_end(spi_device);
}

//-----------------------------------------------------------------------------
// Implement reads to the specified buffer.

static  void
mt25ql_spi_read(struct cyg_flash_dev *dev, cyg_flashaddr_t addr,
                 cyg_uint8 *rbuf, cyg_uint32 rbuf_len)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    const cyg_uint8 tx_buf[5] = 
    { 
        N25QXX_CMD_READ_OPCODE,
        (cyg_uint8)(addr >> 16),
        (cyg_uint8)(addr >> 8), 
        (cyg_uint8)(addr), 
        0
    };

    // Implement the read operation as a multistage SPI transaction.
    cyg_spi_transaction_begin(spi_device);
    cyg_spi_transaction_transfer(spi_device, N25QXX_POLLED ,
                                 N25QXX_CMD_READ_LEN, tx_buf, NULL, false);
    cyg_spi_transaction_transfer(spi_device, N25QXX_POLLED , rbuf_len, NULL,
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
mt25ql_init(struct cyg_flash_dev *dev)
{
    mt25ql_params *dev_params = (mt25ql_params *) mt25ql_supported_devices;
    cyg_uint32      device_id;
    cyg_uint8       i = 0;
    int             retval = FLASH_ERR_INVALID;

    // Find the device in the supported devices list.
    device_id = mt25ql_spi_rdid(dev);

    while ((dev_params->jedec_id != 0) && (dev_params->jedec_id != device_id))
    {
        dev_params++;
    }

    // Found supported device - update device parameters.  N25QXX devices have
    // a uniform sector distribution, so only 1 block info record is required.
    if (dev_params->jedec_id != 0) 
    {
        // Find out is the wanted sector size is supported by the device
        while ((dev_params->sector_size[i] !=
                CYGPKG_DEVS_FLASH_SPI_N25QXX_BLOCK_SIZE)
               && (dev_params->sector_size[i] != 0))
            i++;

        if (dev_params->sector_size[i] !=
            CYGPKG_DEVS_FLASH_SPI_N25QXX_BLOCK_SIZE) 
        {
            TRACE_N25QXX("Init device with JEDEC ID 0x%06X\n",
                          dev_params->jedec_id);
            TRACE_N25QXX("SPI Flash Error, not supporting %dK sector size\n",
                          CYGPKG_DEVS_FLASH_SPI_N25QXX_BLOCK_SIZE);
            return retval;
        }

        ASSERT_N25QXX(dev->num_block_infos == 1,
                       "Only 1 block info record required.");
        ASSERT_N25QXX(dev->block_info != NULL,
                       "Null pointer to block info record.");

        if ((dev->num_block_infos == 1) && (dev->block_info != NULL))
        {
            TRACE_N25QXX("Init device with JEDEC ID 0x%06X.\n", device_id);
            dev->end =
                dev->start +
                ((cyg_flashaddr_t) dev_params->sector_size[i] *
                 (cyg_flashaddr_t) (dev_params->sector_count[i])) - 1;

            // Strictly speaking the block info fields are 'read only'.
            // However, we have a legitimate reason for updating the contents
            // here and can cast away the const.
            ((cyg_flash_block_info_t *) dev->block_info)->block_size =
                (size_t)(dev_params->sector_size[i]);
            ((cyg_flash_block_info_t *) dev->block_info)->blocks =
                (cyg_uint32)(dev_params->sector_count[i]);

            retval = FLASH_ERR_OK;
        }
    }
    return retval;
}

//-----------------------------------------------------------------------------
// Erase a single sector of the flash.

static int
mt25ql_erase_block(struct cyg_flash_dev *dev, cyg_flashaddr_t block_base)
{
    cyg_flashaddr_t local_base = block_base;
    int             retval = FLASH_ERR_INVALID;
    cyg_uint8       dev_status;

    // Fix up the block address and send the sector erase command.
    if (mt25ql_to_local_addr(dev, &local_base)) 
    {
        mt25ql_spi_wren(dev);
        mt25ql_spi_se(dev, local_base);

        // Spin waiting for the erase to complete. Erasing takes around
        // 25 ms
        do
        {
            N25QXX_DELAY_MS(10);
            dev_status = mt25ql_spi_rdsr(dev);
        } while (dev_status & N25QXX_STATUS_BSY);

        retval = FLASH_ERR_OK;
    }
    return retval;
}

static int mt25ql_verifi(struct cyg_flash_dev *dev, cyg_flashaddr_t local_base,
                         const void *data, size_t len)
{
    cyg_uint32 i;
    cyg_uint8  verifi_buf[100];
    cyg_uint32 rx_bytes;
    cyg_uint8  *verifi_data = (cyg_uint8*)data;
    while(len)
    {
        rx_bytes = (len < 100) ? len : 100; 
        mt25ql_spi_read(dev, local_base, verifi_buf, rx_bytes);
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
mt25ql_program(struct cyg_flash_dev *dev, cyg_flashaddr_t base,
                const void *data, size_t len)
{
    cyg_flashaddr_t local_base = base;
    cyg_flashaddr_t vlocal_base;
    int             retval = FLASH_ERR_OK;
    cyg_uint8      *tx_ptr = (cyg_uint8 *)data;
    cyg_uint32      tx_bytes_left = (cyg_uint32)len;
    cyg_uint32      tx_bytes;
    cyg_uint8       dev_status;

    // Fix up the block address.
    if (!mt25ql_to_local_addr(dev, &local_base)) 
    {
        retval = FLASH_ERR_INVALID;
        goto out;
    }
    vlocal_base = local_base;
    // The start of the transaction may not be page aligned, so we need to work
    // out how many bytes to transmit before we hit the first page boundary.
    tx_bytes =
        N25QXXX_PAGE_SIZE -
        (((cyg_uint32)local_base) & (N25QXXX_PAGE_SIZE - 1));
    if (tx_bytes > tx_bytes_left)
        tx_bytes = tx_bytes_left;

    // Perform page program operations.
    while (tx_bytes_left)
    {
        mt25ql_spi_wren(dev);
        mt25ql_spi_pp(dev, local_base, tx_ptr, tx_bytes);

        // Spin waiting for write to complete.  This can take up to 5ms, so
        // we use a polling interval of 1ms - which may get rounded up to the
        // RTC tick granularity.
        do 
        {
            N25QXX_DELAY_MS(1);
            dev_status = mt25ql_spi_rdsr(dev);
        } while (dev_status & N25QXX_STATUS_BSY);

        // Update counters and data pointers for the next page.
        tx_bytes_left -= tx_bytes;
        tx_ptr        += tx_bytes;
        local_base    += tx_bytes;
        tx_bytes       =
            (tx_bytes_left >
             N25QXXX_PAGE_SIZE) ? N25QXXX_PAGE_SIZE : tx_bytes_left;
    }

    retval = mt25ql_verifi(dev,vlocal_base ,data, len);
out:
    return retval;
}

//-----------------------------------------------------------------------------
// Read back an arbitrary amount of data from flash.

static int
mt25ql_read(struct cyg_flash_dev *dev, const cyg_flashaddr_t base,
             void *data, size_t len)
{
    cyg_flashaddr_t local_base = base;
    int             retval = FLASH_ERR_INVALID;
    cyg_uint8      *rx_ptr = (cyg_uint8 *)data;
    cyg_uint32      rx_bytes_left = (cyg_uint32)len;
    cyg_uint32      rx_bytes;

    // Determine the maximum transfer size to use.
    cyg_uint32      rx_block_size =
        (CYGNUM_DEVS_FLASH_SPI_N25QXX_READ_BLOCK_SIZE ==
         0) ? 0xFFFFFFFF : CYGNUM_DEVS_FLASH_SPI_N25QXX_READ_BLOCK_SIZE;

    // Fix up the block address and fill the read buffer.
    if (mt25ql_to_local_addr(dev, &local_base)) 
    {
        while (rx_bytes_left) 
        {
            rx_bytes =
                (rx_bytes_left <
                 rx_block_size) ? rx_bytes_left : rx_block_size;
            mt25ql_spi_read(dev, local_base, rx_ptr, rx_bytes);

            // Update counters and data pointers for next read block.
            rx_bytes_left -= rx_bytes;
            rx_ptr += rx_bytes;
            local_base += rx_bytes;
        }
        retval = FLASH_ERR_OK;
    }
    return retval;
}

//-----------------------------------------------------------------------------
// Lock device

static int
mt25ql_lock(struct cyg_flash_dev *dev, cyg_flashaddr_t base)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    cyg_flashaddr_t local_base = base;
    int             retval = FLASH_ERR_INVALID;
    const cyg_uint8 tx_buf[2] = { N25QXX_CMD_WRSR, N25QXX_LOCK_BITS };

    if (mt25ql_to_local_addr(dev, &local_base)) 
    {
        mt25ql_spi_wren(dev);
        cyg_spi_transfer(spi_device, N25QXX_POLLED, 2, tx_buf, NULL);
        retval = FLASH_ERR_OK;
    }
    return retval;
}

//-----------------------------------------------------------------------------
// Unlock device

static int
mt25ql_unlock(struct cyg_flash_dev *dev, cyg_flashaddr_t base)
{
    cyg_spi_device *spi_device = (cyg_spi_device *) dev->priv;
    cyg_flashaddr_t local_base = base;
    int             retval = FLASH_ERR_INVALID;
    const cyg_uint8 tx_buf[2] = { N25QXX_CMD_WRSR, 0x0 };

    if (mt25ql_to_local_addr(dev, &local_base)) 
    {
        mt25ql_spi_wren(dev);
        cyg_spi_transfer(spi_device, N25QXX_POLLED, 2, tx_buf, NULL);
        retval = FLASH_ERR_OK;
    }
    return retval;
}

//=============================================================================
// Fill in the driver data structures.
//=============================================================================

CYG_FLASH_FUNS (
    cyg_devs_flash_spi_mt25ql_funs, // Exported name of function pointers.
    mt25ql_init,                    // Flash initialization.
    cyg_flash_devfn_query_nop,       // Query operations not supported.
    mt25ql_erase_block,             // Sector erase.
    mt25ql_program,                 // Program multiple pages.
    mt25ql_read,                    // Read arbitrary amount of data.
    mt25ql_lock,                    // Locking (lock the whole device).
    mt25ql_unlock
);

//-----------------------------------------------------------------------------
// EOF mt25ql.c
