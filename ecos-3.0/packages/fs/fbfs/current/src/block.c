//==========================================================================
//
//      block.c
//
//      SIMPLEFS file system block layer
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003, 2004 Free Software Foundation, Inc.
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
// Author(s):           Filip Adamece <filip.adamec.ez2@gmail.com>
// Original data:       
// Date:                2013-10-22
// Description:         
//
//####DESCRIPTIONEND####
//
//==========================================================================
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <cyg/fileio/fileio.h>
/*#include <cyg/io/fm25wxx.h>
#include <cyg/io/n25qxx.h>
#include <cyg/crc/crc.h>*/
#include <cyg/error/errno.h>
#include <cyg/fs/blockfsconfig.h>
#include <cyg/fs/block.h>

//RAM copy of block bitmap
cyg_uint8 block_bitmap[BLOCKFS_NUM_BOLOCKS/8] BLOCK_FS_STATIC_DTATA_ATRIBUTES;

/************************************************************************/
/* Offset vramci flash chybi nutno upravit                              */
/************************************************************************/

static int find_first_one(cyg_uint8 mask, cyg_uint8 start_bit)					
{												
    int pos = -1;
    if(mask)
    {
        pos = start_bit&0x7;
        while (!(mask & (1 << pos)))
        {
            pos++;
        }
    }

    return pos;
}

int block_write(fbfs_super_block *super, block_id_t block, 
		block_address_t offset, block_len_t len, 
                const void *data, cyg_uint32 *writed)
{
    cyg_uint32      ret;
    cyg_flashaddr_t err_addr;

    if(block < super->s_blocks_count)
    {	
        cyg_uint32 addr = block*super->s_log_block_size + offset + 
                                          BLOCKFS_BLOCK_FLASH_OFFSET;
        if((len + offset) >= super->s_log_block_size)
            len = super->s_log_block_size - offset;

        if((ret = cyg_flash_program(addr,data,len,&err_addr))
                != CYG_FLASH_ERR_OK)
        {
            return EIO;
        }

        *writed = len;
        return ENOERR;
    }		

    CYG_FAIL("Block write failed in block boundaries\n");
    return EINVAL;
}

int block_write_super(fbfs_super_block *super)
{
    cyg_uint32 wr_len;
    if(block_write(super,0,0,sizeof(fbfs_super_block),super,&wr_len)
        != ENOERR || wr_len != sizeof(fbfs_super_block))
    {
         return EIO;
    }
    return ENOERR;
}

int block_read(fbfs_super_block *super, block_id_t block, block_address_t offset, 
               block_len_t len, void *data, cyg_uint32 *readed)
{
    cyg_uint32      ret;
    cyg_flashaddr_t err_addr;

    if(block < super->s_blocks_count)
    {
        if((len + offset) >= super->s_log_block_size)
            len = super->s_log_block_size - offset;

        cyg_uint32 addr = block*super->s_log_block_size + offset +
                                            BLOCKFS_BLOCK_FLASH_OFFSET;
        if((ret = cyg_flash_read(addr,data,len,&err_addr))
                != CYG_FLASH_ERR_OK)
        {
            return EIO;
        }

        *readed = len;
        return ENOERR;
    }

    CYG_FAIL("Block read failed in block boundaries\n");
    return EINVAL;
}

int block_errase(fbfs_super_block *super,block_id_t block)
{
    cyg_uint32		ret;
    cyg_flashaddr_t err_addr;

    if(block < BLOCKFS_NUM_FRAM_BLOCKS)
        return ENOERR;

    if(block < super->s_blocks_count)
    {
        cyg_uint32 addr = block*super->s_log_block_size + BLOCKFS_BLOCK_FLASH_OFFSET;
        if((ret = cyg_flash_erase(addr,super->s_log_block_size,&err_addr))
                != CYG_FLASH_ERR_OK)
        {
            return EIO;
        }

        return FBFS_ERROR_OK;
    }
    CYG_FAIL("Block to errase out of range\n");
    return EINVAL;
}

int is_block_free(fbfs_super_block *super, cyg_uint32 block)
{
    if(block < super->s_blocks_count)
    {
        cyg_uint32 start        = block;
        cyg_uint8  start_bit    = start&0x7;
        cyg_uint8  byte         = block_bitmap[start >> 3];
        cyg_uint8  bit          = byte&(1 << start_bit);
        if(bit == 0)
            return 0;
        return 1;
    }
    return -1;
}


int block_alloc(fbfs_super_block *super, block_id_t *new_block)
{
    cyg_uint32 search_start = super->block_goroup_info.bg_start_search;
    
    if(super->block_goroup_info.bg_free_blocks_count == 0)
        return -1;
    
    while(true)
    {
        int isfree;
        
        if(search_start == super->s_blocks_count)
            search_start = 0;
        
        isfree = is_block_free(super,search_start);
        if(isfree == -1)
            return ENOSPC;
        if(isfree == 1)
            break;
    }
    
    *new_block		= search_start;
    err = block_errase(super,*new_block);
    if(err != ENOERR)
        return err;

    block_bitmap[start] &= ~(1 << bit);
    super->s_free_blocks_count--;
    super->block_goroup_info.bg_free_blocks_count--;
    //super->block_goroup_info.bg_free_blocks_count--;

    if(super->block_goroup_info.bg_start_search < super->s_blocks_count)
        super->block_goroup_info.bg_start_search++;
    else
        super->block_goroup_info.bg_start_search = super->s_first_data_block;
    return ENOERR;
	
}

int block_alloc_reserved(fbfs_super_block *super, block_id_t *new_block)
{
    int start,end,bit;

    end   = super->s_first_data_block_reserved >> 3;

    for(start = 0 ;start <= end; start++)
    {
        if(block_bitmap[start] == 0)
            continue;

        bit = find_first_one(block_bitmap[start],0);
        CYG_ASSERT(bit != -1,"Failed to find block to alloc\n");

        *new_block		= start*8 + bit;
        block_bitmap[start] &= ~(1 << bit);
        super->s_free_blocks_count--;
        super->block_goroup_info.bg_free_blocks_count--;
        return ENOERR;
    }

    return ENOSPC;
	
}
int block_free(fbfs_super_block *super,block_id_t block_id)
{
    block_bitmap[block_id >> 3] |= (1 << (block_id&0x07));

    super->s_free_blocks_count++;
    super->block_goroup_info.bg_free_blocks_count++;

    return ENOERR;
}

int block_write_bitmap(fbfs_super_block *super)
{
    int err;
    cyg_uint32 wr_len;
    
    err = block_write_super(super);
    if(err != ENOERR)
        return err;

    err = block_write(super,super->block_goroup_info.bg_block_bitmap,0,
                              sizeof(block_bitmap),&block_bitmap[0],&wr_len);

    if(wr_len != sizeof(block_bitmap))
        return EIO;

    return err;
}

int block_read_bitmap(fbfs_super_block *super)
{
    int			err;
    cyg_uint32	rd_len;
    err =  block_read(super,1,0,sizeof(block_bitmap),&block_bitmap[0],&rd_len);
    if(err != ENOERR)
        return err;
    if(rd_len != sizeof(block_bitmap))
        return EIO;

    return ENOERR;
}

int block_set_used(fbfs_super_block *super, block_id_t block_id, cyg_bool save)
{
    cyg_uint32	wr_len;
    block_bitmap[block_id >> 3] &= ~(1 << (block_id&0x07));
    super->s_free_blocks_count--;

    if(save)
    {
        int err;
        err = block_write_super(super);
        if(err != ENOERR)
            return err;

        err = block_write(super,
                super->block_goroup_info.bg_block_bitmap,block_id >> 3,
                sizeof(cyg_uint8),&block_bitmap[block_id >> 3],&wr_len);

        if(err != ENOERR)
            return err;

        if(wr_len != sizeof(cyg_uint8))
            return EIO;
    }	
    return ENOERR;
}

int block_set_free(fbfs_super_block *super, block_id_t block_id, cyg_bool save)
{
    cyg_uint32	wr_len;
    block_bitmap[block_id >> 3] |= (1 << (block_id&0static int FindFreeBlock(fbfs_super_block *super)
{
    
}x07));
    super->s_free_blocks_count++;
    super->block_goroup_info.bg_free_blocks_count++;
    if(save)
    {
        int err;
        err = block_write_super(super);
        if(err != ENOERR)
            return err;

        err = block_write(super,
                super->block_goroup_info.bg_block_bitmap,block_id >> 3,
                sizeof(cyg_uint8),&block_bitmap[block_id >> 3],&wr_len);

        if(err != ENOERR)
            return err;

        if(wr_len != sizeof(cyg_uint8))
            return EIO;
    }
    return ENOERR;
}
