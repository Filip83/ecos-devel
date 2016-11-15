/*
 * f2bfscheck.c
 *
 * Created: 5.6.2014 9:40:59
 *  Author: Filip
 */ 
#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <cyg/hal/basetype.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <cyg/infra/diag.h>
#include <cyg/fileio/fileio.h>
#include "f2bfsconfig.h"
#include "block.h"
#include "node.h"
#include "fsck.h"
#include <malloc.h>

extern f2bfs_super_block  root_super  BLOCK_FS_STATIC_DTATA_ATRIBUTES;
extern f2bfs_inode	     root_inode   BLOCK_FS_STATIC_DTATA_ATRIBUTES;

extern cyg_uint8 inode_bitmap[F2BFS_NODES_PER_BLOCK/8 + 1];
extern cyg_uint8 block_bitmap[F2BFS_NUM_BOLOCKS/8];

#define F2BFS_CHECK_OK			0
#define F2BFS_CHECK_IO_ERROR		1

#define F2BFS_CHECK_REF_CNT_ZERO 1
#define F2BFS_CHECK_NODE_ID      2
#define F2BFS_CHECK_NODE_CRC     4
#define F2BFS_CHECK_NODE_BLOCK_COUNT 8
#define F2BFS_CHECK_NODE_SIZE   16
#define F2BFS_CHECK_NODE_BLOCK_OUTOF_RANGE 32
#define F2BFS_CHECK_NODE_BLOCK_INDIR_OUTOF_RANGE 64
#define F2BFS_CHECK_NODE_BLOCK_DUPLICATE 128
#define F2BFS_CHECK_NODE_DIR_RCORD_EXISTS 256
#define F2BFS_CHECK_NODE_DELETED          512

#define F2BFS_CHECK_DIR_SIZE				1
#define F2BFS_CHECK_DIR_NAME_SIZE		2
#define F2BFS_CHECK_DIR_NODE_OUTOF_RANGE	4
#define F2BFS_CHECK_DIR_CRC				8
#define F2BFS_CHECK_DIR_VALID_NODE_EXIST				16

static int f2bfs_check_node_blocks(f2bfs_inode_check_t *cur_ck, f2bfs_inode* cur_node);
static int f2bfs_check_node_blocks_dir(f2bfs_inode_check_t *cur_ck, f2bfs_inode* cur_node);
static int f2bfs_check_node_blocks_indir(f2bfs_inode_check_t *cur_ck, f2bfs_inode* cur_node);
static void f2bfs_check_set_block_used(f2bfs_inode_check_t *cur_ck, cyg_uint32 block_id);

static int f2bfs_check_duplicate_blocks(f2bfs_check_t *check_str);
static int f2bfs_check_node_dir_record(f2bfs_check_t *check_str);
static int f2bfs_check_find_dir_node(f2bfs_check_t *check_str, cyg_uint32 node_id);
static void f2bfs_check_show_results(f2bfs_check_t *check_str);
static void f2bfs_check_free(f2bfs_check_t *check_str);
static int f2bfs_check_super(f2bfs_check_t *check_str);
static int f2bfs_check_repair(f2bfs_check_t *check_str);
static int f2bfs_check_block_from_bitmap(f2bfs_check_t *check_str);
static int f2bfs_check_nodes_from_bitmap(f2bfs_check_t *check_str);

extern int f2bfs_read_direntry(f2bfs_super_block *super, f2bfs_inode *dir,
cyg_uint32 pos, f2bfs_dir_entry * dir_entry );

f2bfs_check_t *check_str;

int f2bfs_collect()
{
	int ret;
	cyg_uint32	wr_len;
	check_str = (f2bfs_check_t*)malloc(sizeof(f2bfs_check_t));
	memset(check_str,0,sizeof(f2bfs_check_t));
	
	check_str->node_table = (f2bfs_inode_check_t**)malloc(root_super.s_inodes_count*sizeof(f2bfs_inode_check_t*));
	
	CYG_ASSERT(check_str->node_table,"Cannot alloc fs check table\n");
	//TODOO ocekovat
	check_str->block_count = 4;
	f2bfs_check_super(check_str);
	
	if(!check_str->super_ok)
	{
		diag_printf("Root super error detected\n");
	}
	
	block_read(&root_super,root_super.block_goroup_info.bg_block_bitmap,
				0,sizeof(block_bitmap),block_bitmap,&wr_len);
	
	block_read(&root_super,root_super.block_goroup_info.bg_inode_bitmap,
				0,sizeof(inode_bitmap),inode_bitmap,&wr_len);
	
	//pak asi na\E8\EDst root_inode
	node_read(&root_super,&root_inode,0);
	
	f2bfs_check_block_from_bitmap(check_str);
	f2bfs_check_nodes_from_bitmap(check_str);
	
	for(int i = 0; i < root_super.s_inodes_count; i++)
	{
		f2bfs_inode cur_node;
		
		ret = node_read(&root_super,&cur_node,i);
		if(ret == EIO)
		{
			f2bfs_check_free(check_str);
			return F2BFS_CHECK_IO_ERROR;
		}
		
		if(!node_get_status(&root_super,(node_id_t)i)/* || cur_node.i_links_count != 0*/)	
		{
			f2bfs_inode_check_t *cur_ck = (f2bfs_inode_check_t*)malloc(sizeof(f2bfs_inode_check_t));
			CYG_ASSERT(cur_ck,"Cannot alock fs check itme\n");
			
			memset(cur_ck,0,sizeof(f2bfs_inode_check_t));
			cur_ck->node_id = i;
			check_str->node_table[i] = cur_ck;

			
			if(ret == ECRC32)
				cur_ck->satus |= F2BFS_CHECK_NODE_CRC;
				
			if(cur_node.i_links_count != 0)
			{
				if(cur_node.i_node_id == i)
				{
					if( S_ISDIR(cur_node.i_mode) )
						cur_ck->dir_node = true;
					else
						cur_ck->dir_node = false;
				}
				else
				{
					cur_ck->satus |= F2BFS_CHECK_NODE_ID;
				}	
			}
			else
			{
				cur_ck->satus |= F2BFS_CHECK_REF_CNT_ZERO;
			}
			
			f2bfs_check_node_blocks(cur_ck,&cur_node);
			
			check_str->block_count += cur_ck->blocks_count;
			check_str->inode_count++;
		}

		
	}
	f2bfs_check_duplicate_blocks(check_str);
	f2bfs_check_node_dir_record(check_str);
	f2bfs_check_show_results(check_str);
	f2bfs_check_repair(check_str);
	f2bfs_check_free(check_str);
	return 0;
}

int f2bfs_check_block_from_bitmap(f2bfs_check_t *check_str)
{
	check_str->block_count_bitmap = 0;
	check_str->block_count_reserved_bitmap = 0;
	for(int i = 0; i < 8; i++)
	{
		if(block_get_status(&root_super,i) == 1)
			check_str->block_count_reserved_bitmap++;
	}
	
	for(int i = 8; i < F2BFS_NUM_BOLOCKS; i++)
	{
		if(block_get_status(&root_super,i) == 1)
			check_str->block_count_bitmap++;
	}
	
	if((check_str->block_count_bitmap + check_str->block_count_reserved_bitmap)!= root_super.s_free_blocks_count)
	{
		diag_printf("Free block count in bitmap and root_super differs\n");
		return 1;
	}
	
	if((check_str->block_count_bitmap)!= root_super.s_free_blocks_count_data)
	{
		diag_printf("Free block count data in bitmap and root_super differs\n");
		return 1;
	}
	return 0;
}

int f2bfs_check_nodes_from_bitmap(f2bfs_check_t *check_str)
{
	check_str->inode_count_bitmap = 0;
	for(int i = 0; i < F2BFS_NODES_PER_BLOCK; i++)
	{
		if(node_get_status(&root_super,i) == 1)
			check_str->inode_count_bitmap++;
	}
	
	if(check_str->inode_count_bitmap != root_super.s_free_inodes_count)
	{
		diag_printf("Free node count in bitmap and root_super differs\n");
		return 1;
	}
	return 0;
}

int f2bfs_check_super(f2bfs_check_t *check_str)
{
	check_str->super_ok = true;
	if(root_super.s_blocks_count != F2BFS_NUM_BOLOCKS)
	{
		root_super.s_blocks_count = F2BFS_NUM_BOLOCKS;
		check_str->super_ok = false;
	}
	//root_super.s_blocks_per_group	= F2BFS_NUM_BOLOCKS;
	if(root_super.s_checkinterval != 0)
	{
		check_str->super_ok = false;
		root_super.s_checkinterval = 0;
	}
	if(root_super.s_first_data_block	!= 8)
	{
		check_str->super_ok = false;
		root_super.s_first_data_block	= 8;
	}
	if(root_super.s_first_data_block_reserved	!= 4)
	{
		check_str->super_ok = false;
		root_super.s_first_data_block_reserved	= 4;
	}
	if(root_super.s_free_blocks_count	> F2BFS_NUM_BOLOCKS)
	{
		check_str->super_ok = false;
	}
	if(root_super.s_free_blocks_count_data > F2BFS_NUM_BOLOCKS - root_super.s_first_data_block)
	{
		check_str->super_ok = false;
	}
	if(root_super.s_free_inodes_count  != F2BFS_NODES_PER_BLOCK)
	{
		root_super.s_free_inodes_count  = F2BFS_NODES_PER_BLOCK;
		check_str->super_ok = false;
	}
	if(root_super.s_inodes_count		!= F2BFS_NODES_PER_BLOCK)
	{
		root_super.s_inodes_count		= F2BFS_NODES_PER_BLOCK;
		check_str->super_ok = false;
	}
	//root_super.s_inodes_per_group   = F2BFS_NODES_PER_BLOCK;
	if(root_super.s_r_blocks_count		!= 3)
	{
		root_super.s_r_blocks_count		= 3;
		check_str->super_ok = false;
	}
	//root_super.s_lastcheck
	if(root_super.s_log_block_size    != CYGNUM_F2BFS_LOG_BLOCK_SIZE)
	{
		root_super.s_log_block_size     = CYGNUM_F2BFS_LOG_BLOCK_SIZE;
		check_str->super_ok = false;
	}
	if(root_super.s_block_size			!= CYGNUM_F2BFS_BLOCK_SIZE)
	{
		root_super.s_block_size			= CYGNUM_F2BFS_BLOCK_SIZE;
		check_str->super_ok = false;
	}

	if(root_super.s_state				== 0)
	{
		root_super.s_state				= 0;
		check_str->super_ok = false;
	}

	
	if(root_super.block_goroup_info.bg_block_bitmap != 1)
	{
		root_super.block_goroup_info.bg_block_bitmap = 1;
		check_str->super_ok = false;
	}
	if(root_super.block_goroup_info.bg_inode_bitmap != 2)
	{
		root_super.block_goroup_info.bg_inode_bitmap = 2;
		check_str->super_ok = false;
	}
	if(root_super.block_goroup_info.bg_inode_table  == 3)
	{
		root_super.block_goroup_info.bg_inode_table  = 3;
		check_str->super_ok = false;
	}
	if(root_super.block_goroup_info.bg_start_search > F2BFS_NUM_BOLOCKS)
	{
		root_super.block_goroup_info.bg_start_search = 8;
		check_str->super_ok = false;
	}
	if(root_super.block_goroup_info.bg_used_dirs_count == 0)
	{
		root_super.block_goroup_info.bg_used_dirs_count = 0;
		check_str->super_ok = false;
	}
	return 0;
}

int f2bfs_check_node_blocks_dir(f2bfs_inode_check_t *cur_ck, f2bfs_inode* cur_node)
{
	cyg_uint32 blocks = (cur_node->i_blocks < F2BFS_NDIR_BLOCKS ) ? cur_node->i_blocks : F2BFS_NDIR_BLOCKS; 
	for(int i = 0; i < blocks; i++)
	{
		if(cur_node->i_block[i] > 0 && cur_node->i_block[i] < F2BFS_NUM_BOLOCKS )
			f2bfs_check_set_block_used(cur_ck,cur_node->i_block[i]);
		else
		{
			cur_ck->satus |= F2BFS_CHECK_NODE_BLOCK_OUTOF_RANGE;
		}
	}
	
	return blocks;
}

int f2bfs_check_node_blocks_indir(f2bfs_inode_check_t *cur_ck, f2bfs_inode* cur_node)
{
	cyg_uint32 blocks = cur_node->i_blocks - F2BFS_NDIR_BLOCKS;
	
	for(int i = 0; i < F2BFS_IND_BLOCK; i++)
	{
		if(cur_node->i_indirect_block[i] == 0)
			break;
			
		if(cur_node->i_indirect_block[i] > 0 &&
		   cur_node->i_indirect_block[i] < F2BFS_NUM_BOLOCKS )
		{
			f2bfs_check_set_block_used(cur_ck,cur_node->i_indirect_block[i]);
			cur_ck->indir_blocks++;
		}
		else
		{
			cur_ck->satus |= F2BFS_CHECK_NODE_BLOCK_INDIR_OUTOF_RANGE;
		}
	}
	
	for(int i = 0; i < blocks; i++)
	{
		int err;
		cyg_uint32 rd_len;
		cyg_uint32 indir_block = i/F2BFS_INDIRECT_PER_BLOCK;
		cyg_uint32 block       = i%F2BFS_INDIRECT_PER_BLOCK;
		block_id_t node_block;
		
		if(!(cur_node->i_indirect_block[indir_block] > 0 &&
		   cur_node->i_indirect_block[indir_block] < F2BFS_NUM_BOLOCKS ))
		{
			continue;
		}
		
		err = block_read(&root_super,cur_node->i_indirect_block[indir_block],block*sizeof(block_id_t),
					sizeof(block_id_t),&node_block,&rd_len);
		if(err != ENOERR || rd_len != sizeof(block_id_t))
			return EIO;
		
		if(node_block > 0 && node_block < F2BFS_NUM_BOLOCKS )
		{
			f2bfs_check_set_block_used(cur_ck,node_block);
		}
		else
		{
			cur_ck->satus |= F2BFS_CHECK_NODE_BLOCK_OUTOF_RANGE;
		}
	}
	
	return 0;
}

int f2bfs_check_node_blocks(f2bfs_inode_check_t *cur_ck, f2bfs_inode* cur_node)
{
	cyg_uint32 block_form_size = 0;
	f2bfs_check_node_blocks_dir(cur_ck,cur_node);
	if(cur_node->i_blocks >= CYGNUM_F2BFS_BLOCKS_DIRECT )
		f2bfs_check_node_blocks_indir(cur_ck,cur_node);
	
	if((cur_ck->blocks_count - cur_ck->indir_blocks) != cur_node->i_blocks)
	{
		cur_ck->satus |= F2BFS_CHECK_NODE_BLOCK_COUNT;
		diag_printf("Node blocks %d, total blocks %d, data blocks %d\n",
		 cur_node->i_blocks, cur_ck->blocks_count, (cur_ck->blocks_count - cur_ck->indir_blocks));
	}
	
	block_form_size = cur_node->i_size >> root_super.s_log_block_size;
	if(block_form_size > (cur_ck->blocks_count - cur_ck->indir_blocks))
	{
		cur_ck->satus |= F2BFS_CHECK_NODE_SIZE;
	}
	return 0;
}

void f2bfs_check_set_block_used(f2bfs_inode_check_t *cur_ck, cyg_uint32 block_id)
{
	cyg_uint32 block_byte = block_id >> 3;
	cyg_uint32 block_bit  = block_id & 0x7;
	
	cur_ck->used_block_bitmap[block_byte] &= ~(1 << block_bit);
	cur_ck->blocks_count++;
}

int f2bfs_check_duplicate_blocks(f2bfs_check_t *check_str)
{
	cyg_uint8 block_bitmap_tmp;
	for(int i = 0; i < check_str->inode_count; i++)
	{
		for(int j = 0; j < check_str->inode_count; j++)
		{
			for(int k = 0; k < F2BFS_NUM_BOLOCKS/8; k++)
			{
				block_bitmap_tmp = check_str->node_table[i]->used_block_bitmap[k] &
				                    check_str->node_table[j]->used_block_bitmap[k];
									
				if(block_bitmap_tmp != 0)
				{
					check_str->duplicate_block_count++;
					check_str->duplicate_block_bitmap[k] |= block_bitmap_tmp;
					check_str->node_table[i]->satus |= F2BFS_CHECK_NODE_BLOCK_DUPLICATE;
					check_str->node_table[j]->satus |= F2BFS_CHECK_NODE_BLOCK_DUPLICATE;
				}
			}
		}
	}
	
	return 0;
}

int f2bfs_check_node_dir_record(f2bfs_check_t *check_str)
{
	off_t pos = 0;
	int				err;
	cyg_uint32		len;
	
	err = node_read(&root_super,&root_inode,0);
	if(err != ENOERR)
	{
		return F2BFS_CHECK_IO_ERROR;
	}
	
	check_str->dir_table = (f2bfs_dir_check_t*)malloc(root_inode.i_size/sizeof(f2bfs_dir_entry)*sizeof(f2bfs_dir_check_t));
	
	for(int i = 0; i < root_inode.i_size/sizeof(f2bfs_dir_entry); i++)
	{
		len = sizeof(f2bfs_dir_entry);
		check_str->dir_table[i].status = 0;
		
		err = f2bfs_read_direntry(&root_super,&root_inode,pos,&check_str->dir_table[i].dir);
		if(err == EIO)
		{
			return F2BFS_CHECK_IO_ERROR;
		}
		
		if(check_str->dir_table[i].dir.rec_len != 0)
		{
			if(check_str->dir_table[i].dir.rec_len != sizeof(f2bfs_dir_entry))
			{
				check_str->dir_table[i].status |= F2BFS_CHECK_DIR_SIZE;
			}
			
			if(check_str->dir_table[i].dir.name_len >= CYGNUM_F2BFS_DIRENT_NAME_SIZE)
			{
				check_str->dir_table[i].status |= F2BFS_CHECK_DIR_NAME_SIZE;
			}
			
			if(check_str->dir_table[i].dir.inode >= root_super.s_inodes_count )
			{
				check_str->dir_table[i].status |= F2BFS_CHECK_DIR_NODE_OUTOF_RANGE;
			}
			
			if(err == ECRC32)
			{
				check_str->dir_table[i].status |= F2BFS_CHECK_DIR_CRC;
			}
			
			check_str->dir_entries++;
			
			if(!f2bfs_check_find_dir_node(check_str,check_str->dir_table[i].dir.inode))
			{
				check_str->dir_table[i].status |= F2BFS_CHECK_DIR_VALID_NODE_EXIST;
			}
		}
		
			
		pos += len;
		
	}				
	
	return 0;
}

int f2bfs_check_find_dir_node(f2bfs_check_t *check_str, cyg_uint32 node_id)
{
	for(int i = 0; i < check_str->inode_count; i++)
	{
		if(check_str->node_table[i]->node_id == node_id)
		{
			check_str->node_table[i]->satus |= F2BFS_CHECK_NODE_DIR_RCORD_EXISTS;
			return 0;
		}
	}
	return 1;
}
/*
#define F2BFS_CHECK_REF_CNT_ZERO 1
#define F2BFS_CHECK_NODE_ID      2
#define F2BFS_CHECK_NODE_CRC     4
#define F2BFS_CHECK_NODE_BLOCK_COUNT 8
#define F2BFS_CHECK_NODE_SIZE   16
#define F2BFS_CHECK_NODE_BLOCK_OUTOF_RANGE 32
#define F2BFS_CHECK_NODE_BLOCK_INDIR_OUTOF_RANGE 64
#define F2BFS_CHECK_NODE_BLOCK_DUPLICATE 128
#define F2BFS_CHECK_NODE_DIR_RCORD_EXISTS 256

#define F2BFS_CHECK_DIR_SIZE				1
#define F2BFS_CHECK_DIR_NAME_SIZE		2
#define F2BFS_CHECK_DIR_NODE_OUTOF_RANGE	4
#define F2BFS_CHECK_DIR_CRC				8
#define F2BFS_CHECK_DIR_VALID_NODE_EXIST				16*/

void f2bfs_check_show_results(f2bfs_check_t *check_str)
{
	if(check_str->block_count != (root_super.s_blocks_count - root_super.s_free_blocks_count))
	{
		diag_printf("Free block count error\n");
		diag_printf("Counted block count %d\n", check_str->block_count);
		diag_printf("Block count %d, free block count %d, used block count %d\n",
		root_super.s_blocks_count,root_super.s_free_blocks_count,
		root_super.s_blocks_count - root_super.s_free_blocks_count);
	}
	
	for(int i = 0; i < check_str->dir_entries; i++)
	{
		if(check_str->dir_table[i].status != F2BFS_CHECK_DIR_VALID_NODE_EXIST)
		{
			diag_printf("Dir %s error %X\n",check_str->dir_table[i].dir.name,check_str->dir_table[i].status);
		}
	}
	
	for(int i = 0; i < check_str->inode_count; i++)
	{
		if(check_str->node_table[i]->satus != F2BFS_CHECK_NODE_DIR_RCORD_EXISTS)
		{
			diag_printf("Node %d error %X\n",check_str->node_table[i]->node_id,check_str->node_table[i]->satus);
		}
	}
	
}

int f2bfs_check_repair(f2bfs_check_t *check_str)
{
	//data blocks
	if((check_str->block_count_bitmap + check_str->block_count) == F2BFS_NUM_BOLOCKS)
	{
		if(root_super.s_free_blocks_count != check_str->block_count_bitmap)
		{
			root_super.s_free_blocks_count = check_str->block_count_bitmap + check_str->block_count_reserved_bitmap;
			root_super.s_free_blocks_count_data = check_str->block_count_bitmap;
		}
	}
	else
	{
		root_super.s_free_blocks_count		= (F2BFS_NUM_BOLOCKS - check_str->block_count);
		root_super.s_free_blocks_count_data = root_super.s_free_blocks_count - 8;	
	}
	
	//node block
	if((check_str->inode_count_bitmap + check_str->inode_count) == F2BFS_NODES_PER_BLOCK)
	{
		if(root_super.s_free_inodes_count != check_str->inode_count_bitmap)
			root_super.s_free_inodes_count = check_str->inode_count_bitmap;
	}
	else
	{
		root_super.s_free_inodes_count = (check_str->inode_count - F2BFS_NUM_BOLOCKS);
	}
	return 0;
}

void f2bfs_check_free(f2bfs_check_t *check_str)
{
	for(int i = 0; i < check_str->inode_count; i++)
	{
		free(check_str->node_table[i]);
	}
	
	free(check_str->dir_table);
	free(check_str);
}