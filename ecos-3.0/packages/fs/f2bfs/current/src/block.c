/*
 * block.c
 *
 * Created: 15.4.2013 12:38:00
 *  Author: Filip
 */ 
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <cyg/infra/diag.h>
#include <cyg/fileio/fileio.h>
#include <cyg/io/flash.h>
#include <cyg/crc/crc.h>
#include <cyg/error/errno.h>
#ifndef FSDEVEL
#include <cyg/fs/f2bfsconfig.h>
#include <cyg/fs/block.h>
#else
#include <f2bfsconfig.h>
#include <block.h>
#endif

#define BlOCK_DEBUG_LEVEL 0

cyg_uint8 block_bitmap[F2BFS_NUM_BOLOCKS/8] BLOCK_FS_STATIC_DTATA_ATRIBUTES;


int block_find_free(f2bfs_super_block *super, block_id_t *new_block_id);
int block_find_free_reserved(f2bfs_super_block *super, block_id_t *new_block_id);

/** block_write_super
/*
/* This function writes super block to specified position.
/* The super block is firt block in FRAM. This function
/* calculate super block structure crc32 before write.
/* \param super - pointer to super block structure.
/* \return IO error
*/
int block_write_super(f2bfs_super_block *super)
{
	cyg_uint32 wr_len;
	super->s_crc32 = 0;
	cyg_uint32 crc = cyg_posix_crc32((unsigned char*)super,
						sizeof(f2bfs_super_block) - sizeof(cyg_uint32));
	super->s_crc32 = crc;
	if(block_write(super,0,0,sizeof(f2bfs_super_block),super,&wr_len)
			!= ENOERR || wr_len != sizeof(f2bfs_super_block))
	{
		return EIO;
	}
	return ENOERR;
}

/** block_write_bitmap fuction.
/*
/* This function write block bitmap asn super block structrue.
/* The block bitmap is witen to specified block. Must be in
/* FRAM.
/* \param super - pointer to super bock.
/* \return IO error
*/
int block_write_bitmap(f2bfs_super_block *super)
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

/** block_read_bitmap fuction.
/*
/* This function read block bitmap to internal RAM buffer.
/* \param super - pointer to super bock.
/* \return IO error
*/
int block_read_bitmap(f2bfs_super_block *super)
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

/** block_write function.
/*
/* This function write data to specified block and offset in
/* block. If data cannot fit to rest of the data block
/* they are clipped and real amount of write bytes is returned.
/* \param super - pointer to super block structure.
/* \param block - id of block to write to.
/* \param offset - offset in bytes in block to write to.
/* \param len - length of the data buffer.
/* \param data - pointer to data to write.
/* \param writed - pointer to variable where number of bytes actually
/*                 writed is returned.
/* \return IO error
*/
int block_write(f2bfs_super_block *super, block_id_t block, 
			block_address_t offset, block_len_t len, const void *data, 
			cyg_uint32 *writed)
{

	cyg_uint32		ret;
	cyg_flashaddr_t err_addr;
	
	/*if(block == 0)
		diag_printf("Wite to block 0\n");*/
	
	if(block < super->s_blocks_count)
	{	
		cyg_uint32 addr = (block << super->s_log_block_size) + offset + 
												F2BFS_BLOCK_FLASH_OFFSET;
		if((len + offset) >= super->s_block_size)
			len = super->s_block_size - offset;
			
		#if BlOCK_DEBUG_LEVEL > 0	
		diag_printf("Block write %d, offset %d, len %d\n",(int)block, (int)offset,(int)len);	
		#endif
		if((ret = cyg_flash_program(addr,data,len,&err_addr))
			   != CYG_FLASH_ERR_OK)
		{
			#if BlOCK_DEBUG_LEVEL > 0	
			diag_printf("%s\n",flash_errmsg(ret));
			#endif
			return EIO;
		}
		
		*writed = len;
		return ENOERR;
	}		

	CYG_FAIL("Block write failed in block boundaries\n");
	return EINVAL;
}

/** block_read function.
/*
/* This function read data from specified block and offset in
/* block. Number of actually read bytes from block is returned.
/* \param super - pointer to super block structure.
/* \param block - id of block to read from.
/* \param offset - offset in bytes in block to read from.
/* \param len - length of the data buffer.
/* \param data - pointer to data buffer.
/* \param readed - pointer to variable where number of bytes actually
/*                 read is returned.
/* \return IO error
*/
int block_read(f2bfs_super_block *super, block_id_t block, block_address_t offset, 
				block_len_t len, void *data, cyg_uint32 *readed)
{
	cyg_uint32		ret;
	cyg_flashaddr_t err_addr;
	
	if(block < super->s_blocks_count)
	{
		if((len + offset) >= super->s_block_size)
			len = super->s_block_size - offset;
			 
		cyg_uint32 addr = (block << super->s_log_block_size) + offset +
												F2BFS_BLOCK_FLASH_OFFSET;
		#if BlOCK_DEBUG_LEVEL > 0	
		diag_printf("Block read %d, offset %d, len %d\n",(int)block, (int)offset,(int)len);
		#endif
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

/** block_errase fucntion
/*
/* This function done block eraseing.
/* \param super - pinter to block super structure
/* \param block - block id to errase
/* \return IO error
*/
int block_errase(f2bfs_super_block *super,block_id_t block)
{
	cyg_uint32		ret;
	cyg_flashaddr_t err_addr;
	
	if(block < F2BFS_NUM_FRAM_BLOCKS)
		return ENOERR;
		
	if(block < super->s_blocks_count)
	{
		cyg_uint32 addr = (block << super->s_log_block_size) + F2BFS_BLOCK_FLASH_OFFSET;
		if((ret = cyg_flash_erase(addr,super->s_log_block_size,&err_addr))
			!= CYG_FLASH_ERR_OK)
		{
			return EIO;
		}
		
		return ENOERR;
	}
	CYG_FAIL("Block to errase out of range\n");
	return EINVAL;
}

/** block_alloc function
/*
/* Function find free block, make earase operation of selected block
/* and set it as used in block bitmap.
/* \param super - pointer to super structure
/* \param new_block - pointer to variable wher new block id is returned.
/* \return IO error
*/
int block_alloc(f2bfs_super_block *super, block_id_t *new_block)
{
	int ret;
	
	ret = block_find_free(super,new_block);
	if(ret != ENOERR)
		return ret;
	
	ret = block_errase(super,*new_block);
	if(ret != ENOERR)
		return ret;
	
	ret = block_set_used(super,*new_block,false);
	return ret;
}

/** block_alloc_reserved function
/*
/* Function find free block, make errase operation of selected block
/* and set it as used in block bitmap. This function operate on reserved
/* block for special use in FRAM region.
/* \param super - pointer to super structure
/* \param new_block - pointer to variable where new block id is returned.
/* \return IO error
*/
int block_alloc_reserved(f2bfs_super_block *super, block_id_t *new_block)
{
	int ret;
	
	ret = block_find_free_reserved(super,new_block);
	if(ret != ENOERR)
		return ret;
	
	ret = block_set_used(super,*new_block,false);
	return ret;
}

/** block_find_free function
/*
/* This function finds free block to use. Based on start and stop values in
/* block_goroup_info structure of super block structure. Search in block
/* bitmap for unused block. First unused block is then returned. The start
/* and stop search values are used to ensure cyclic block usage.
/* \param super - pointer to super structure
/* \param new_block_id - pointer to variable where new block id is returned.
/* \return IO error
*/
int block_find_free(f2bfs_super_block *super, block_id_t *new_block_id)
{
	cyg_uint32 search_byte;
	cyg_uint32 search_bit;
	cyg_uint32 sarch_start = super->block_goroup_info.bg_start_search;
	cyg_bool   cycle = false;
	
	if(super->s_free_blocks_count == 0)
		return ENOSPC;
	
	while(true)
	{
		if(sarch_start >= super->s_blocks_count)
		{
			if(cycle)
			{
				//CYG_ASSERT(false,"Inconsistend fre block count and bitmap.\n");
				return ENOSPC;
			}
			sarch_start = super->s_first_data_block;
			cycle = true;
		}
		
		search_byte = sarch_start >> 3;
		search_bit  = sarch_start & 0x7;
		
		if((block_bitmap[search_byte] & (1 << search_bit)) != 0)
		{
			*new_block_id = sarch_start;
			super->block_goroup_info.bg_start_search = sarch_start + 1;
			return ENOERR;
		}
		sarch_start++;
	}
	
	return EINVAL;
}

/** block_find_free_reserved function
/*
/* This function finds free block to use. Based on start and stop values in
/* super block structure. Search in block
/* bitmap for unused block in FRAM region. First unused block is then returned. The start
/* and stop search values are used to ensure cyclic block usage.
/* \param super - pointer to super structure
/* \param new_block_id - pointer to variable where new block id is returned.
/* \return IO error
*/
int block_find_free_reserved(f2bfs_super_block *super, block_id_t *new_block_id)
{
	cyg_uint32 search_byte;
	cyg_uint32 search_bit;
	cyg_uint32 sarch_start = super->s_first_data_block_reserved;
	cyg_bool   cycle = false;
	
	while(true)
	{
		if(sarch_start >= super->s_first_data_block)
		{
			if(cycle)
				return ENOSPC;
			sarch_start = super->s_first_data_block_reserved;
			cycle = true;
		}
		
		search_byte = sarch_start >> 3;
		search_bit  = sarch_start & 0x7;
		
		if((block_bitmap[search_byte] & (1 << search_bit)) != 0)
		{
			*new_block_id = sarch_start;
			return ENOERR;
		}
		sarch_start++;
	}
	
	return EINVAL;
}

/** block_set_used function
/*
/* This function set specified block as used. Number of free blocks is actualized 
/* in this function and as well the specified block is set as used in block
/* bitmap and this bitmap can be optionally saved back to disk.
/* \param super - pointer to super structure.
/* \param block_id - the id of the block to set used.
/* \param save - if true the block bitmap is actualized on disk as well.
/* \return IO error
*/
int block_set_used(f2bfs_super_block *super, block_id_t block_id, cyg_bool save)
{
	cyg_uint32	wr_len;
	
	cyg_uint32 block_byte = block_id >> 3;
	cyg_uint32 block_bit  = block_id & 0x7;
	
	if((block_bitmap[block_byte] & (1 << block_bit)) == 0)
		return EINVAL;
	
	block_bitmap[block_byte] &= ~(1 << block_bit);
	super->s_free_blocks_count--;
	if(block_id >= super->s_first_data_block)
		super->s_free_blocks_count_data--;

	if(save)
	{
		int err;
		err = block_write_super(super);
		if(err != ENOERR)
			return err;
	
		err = block_write(super,super->block_goroup_info.bg_block_bitmap, block_byte,
							sizeof(cyg_uint8),&block_bitmap[block_byte],&wr_len);
							
		if(err != ENOERR)
			return err;
		
		if(wr_len != sizeof(cyg_uint8))
			return EIO;
			
	}	
	return ENOERR;
}

/** block_get_status function
/*
/* This function return status of specified block number.
/* \param super - pointer to super block structure.
/* \param block - the id of block.
/* \return 0 - if block is used, 1 - if block is free.
*/
int block_get_status(f2bfs_super_block *super, block_id_t block)
{
	cyg_uint32 block_byte = block >> 3;
	cyg_uint32 block_bit  = block & 0x7;
	
	if((block_bitmap[block_byte] & (1 << block_bit)) != 0)
		return 1;
	return 0;
}

/** block_set_free function
/*
/* This function set specified block as free. Number of free blocks is actualized
/* in this function and as well the specified block is set as used in block
/* bitmap and this bitmap can be optionally saved back to disk.
/* \param super - pointer to super structure.
/* \param block_id - the id of the block to set used.
/* \param save - if true the block bitmap is actualized on disk as well.
/* \return IO error
*/
int block_set_free(f2bfs_super_block *super, block_id_t block_id, cyg_bool save)
{
	cyg_uint32	wr_len;
	cyg_uint32 block_byte = block_id >> 3;
	cyg_uint32 block_bit  = block_id & 0x7;
	
	if((block_bitmap[block_byte] & (1 << block_bit)) != 0)
		return ENOERR;
	
	block_bitmap[block_byte] |= (1 << block_bit);
	super->s_free_blocks_count++;
	if(block_id >= super->s_first_data_block)
		super->s_free_blocks_count_data++;

	if(save)
	{
		int err;
		err = block_write_super(super);
		if(err != ENOERR)
			return err;
		err = block_write(super,super->block_goroup_info.bg_block_bitmap, block_byte,
							sizeof(cyg_uint8),&block_bitmap[block_byte],&wr_len);
							
		if(err != ENOERR)
			return err;
		
		if(wr_len != sizeof(cyg_uint8))
			return EIO;
	}
	return ENOERR;
}
