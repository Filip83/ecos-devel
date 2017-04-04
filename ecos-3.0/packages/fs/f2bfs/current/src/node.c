/*
 * Node.c
 *
 * Created: 15.4.2013 14:17:01
 *  Author: Filip
 */ 
#include <pkgconf/system.h>
#include <pkgconf/hal.h>

#include <pkgconf/io_fileio.h>

#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <cyg/infra/diag.h>
#include <cyg/fileio/fileio.h>
#include <cyg/crc/crc.h>
#ifndef FSDEVEL
#include <cyg/fs/f2bfsconfig.h>
#include <cyg/fs/block.h>
#include <cyg/fs/node.h>
#else
#include <f2bfsconfig.h>
#include <block.h>
#include <node.h>
#endif
#include <malloc.h>

void print_node(f2bfs_inode *node);

#define NODE_DIAG_LEVEL 0

/** Node bimap static data. */
cyg_uint8 inode_bitmap[F2BFS_NODES_PER_BLOCK/8 + 1] BLOCK_FS_STATIC_DTATA_ATRIBUTES;

/** node_read_bitmap function
/*
/* This function read node bitmap from block specified by super block
/* structure to inode_bitmap arry.
/* \param super - ponter to super structure.
/* \return IO error
*/
int node_read_bitmap(f2bfs_super_block *super)
{
	int			err;
	cyg_uint32	rd_len;
	err = block_read(super,super->block_goroup_info.bg_inode_bitmap,
		0,sizeof(inode_bitmap),&inode_bitmap[0],&rd_len);
	
	if(err != ENOERR)
		return err;
	
	if(rd_len != sizeof(inode_bitmap))
		return EIO;
	
	return ENOERR;
}

/** node_write_bitmap function
/*
/* This function actualize state of node bitmap for specified node to the disk. If
/* node_id is zero whole bitmap is writ to disk.
/* \param super - pointer to super structure.
/* \param node_id - node to witch actualize state of node bitmap.
/* \return IO error
*/
int node_write_bitmap(f2bfs_super_block *super, node_id_t node_id)
{
	cyg_uint32 wr_len;
	cyg_uint32 ret;
	if(node_id == 0)
	{
		ret = block_write(super,super->block_goroup_info.bg_inode_bitmap,0,
							sizeof(inode_bitmap),&inode_bitmap[0],&wr_len);
		
		if(wr_len != sizeof(inode_bitmap))
			return EIO;
	}
	else
	{
		ret = block_write(super,super->block_goroup_info.bg_inode_bitmap,node_id >> 3,
							sizeof(cyg_uint8),&inode_bitmap[node_id >> 3],&wr_len);
		
		if(wr_len != sizeof(cyg_uint8))
			return EIO;
	}
	
	return ret;
}

/** node_find_free function
/*
/* This function finds first free node. The free node is selected
/* from node bitmap. 
/* \param super - pointer to super block structure.
/* \param new_node_id - pointer to variable where id of the new node is stored.
/* \return IO error
*/
int node_find_free(f2bfs_super_block *super, node_id_t *new_node_id)
{
	cyg_uint32 search_byte;
	cyg_uint32 search_bit;
	cyg_uint32 search_start;
	
	if(super->s_free_inodes_count == 0)
		return ENOSPC;

	for(search_start = 0; search_start < super->s_inodes_count; search_start++)
	{
		search_byte = search_start >> 3;
		search_bit  = search_start & 0x7;
		
		if((inode_bitmap[search_byte] & (1 << search_bit)) != 0)
		{
			*new_node_id = search_start;
			return ENOERR;
		}
	}
	
	CYG_ASSERT(false,"Inconsistent free node count and bitmap.\n");
	return ENOSPC;
}

/** node_read function
/*
/* This function read node from disk. The node crc32 checked hear as well.
/* \param super - pointer to super structure.
/* \param node - pointer to node structure where node data will be rad.
/* \param node_id - id of the node to read.
/* \return IO error
*/
int node_read(f2bfs_super_block *super,f2bfs_inode *node,node_id_t node_id)
{
	if(node_id < super->s_inodes_count)
	{
		cyg_uint32			red_len;
		cyg_uint32			ret;
		cyg_uint32          crc;
		
		ret = block_read(super,super->block_goroup_info.bg_inode_table,
		node_id*sizeof(f2bfs_inode),sizeof(f2bfs_inode),node,&red_len);
		
		if(ret != ENOERR || red_len != sizeof(f2bfs_inode))
			return EIO;
			
		crc = cyg_posix_crc32((unsigned char*)node,sizeof(f2bfs_inode) - sizeof(cyg_uint32));
		if(crc != node->i_crc32)
		{
			#if NODE_DIAG_LEVEL > 0
			diag_printf("read node crc error\n");
			#endif
			return ECRC32;
		}
		return ENOERR;
	}
	return EIO;
}

/** node_open function
/*
/* This function alloc space for node on heap and read node data
/* for specified node id.
/* \param super - pointer to super block structure.
/* \param node_id - id of node to open
/* \return pointer to node structure. Null pointer indicate error.
*/
f2bfs_inode* node_open(f2bfs_super_block *super, node_id_t node_id)
{
	if(node_id < super->s_inodes_count)
	{
		f2bfs_inode			*node;
		node = (f2bfs_inode*)malloc(sizeof(f2bfs_inode));
		CYG_ASSERT(node,"Cannot allocated space for node\n");
				
		if((node_read(super,node,node_id)) != ENOERR)
		{
			free(node);
			return NULL;
		}
		node->i_refcnt = 0;
		return node;
	}
	CYG_FAIL("Node boundaries failed\n");
	return NULL;
}

/** node_close function
/*
/* This function free alloc node space by \see node_open function.
/* \param super - pointer to super block structure.
/* \param node -  pointer to node to free.
/* \return IO error
*/
int node_close(f2bfs_super_block *super, f2bfs_inode * node)
{
	if(node != NULL)
		free(node);
	return ENOERR;
}

/** node_create function
/*
/* This function alloc space for new node, find free node on disk 
/* and init alloc node structure with data as node_id and creation
/* time etc.
/* \param super - pointer to super block structure.
/* \param mode - file mode
/* \return pointer to node structure, NULL if error
*/
f2bfs_inode *node_create(f2bfs_super_block *super, mode_t mode )
{
	cyg_uint32 err;
	f2bfs_inode *node = (f2bfs_inode*)malloc(sizeof(f2bfs_inode));
	CYG_ASSERT(node,"Cannot alloc new node\n");
	memset( node , 0, sizeof(f2bfs_inode) );
	
	err = node_find_free(super,&node->i_node_id);
	if(err != ENOERR)
	{
		node_close(super,node);
		return NULL;
	}
		
	node->i_mode	= mode;
	node->i_atime	=
	node->i_mtime	= 
	node->i_ctime	= cyg_timestamp();

	return node;
}

/** node_write function
/*
/* This function write node structure to disk.
/* \param super - pointer to super block structure.
/* \param node - pointer to node structure.
/* \return IO error
*/
int node_write(f2bfs_super_block *super, f2bfs_inode *node)
{
	if(node->i_node_id < super->s_inodes_count)
	{
		int err;
		cyg_uint32	wr_len;
		node->i_crc32 = 0;
		cyg_uint32 crc = cyg_posix_crc32((unsigned char*)node,sizeof(f2bfs_inode) - sizeof(cyg_uint32));
		node->i_crc32 = crc;
		
		err = block_write(super,super->block_goroup_info.bg_inode_table,
				node->i_node_id*sizeof(f2bfs_inode),sizeof(f2bfs_inode),node,&wr_len);
		if( wr_len != sizeof(f2bfs_inode))
			return EIO;
		return err;
	}
	CYG_FAIL("Node write budaries failed\n");
	return EIO;
}

/** node_set_free function
/*
/* This function set specified node as free and optionally save node bitmap to disk.
/* This function actualize free node count as well.
/* \param super - pointer to super block structure.
/* \param node_id - id of node to set free.
/* \param save if true node bitmap is actualized on disk.
/* \return IO error
*/
int node_set_free(f2bfs_super_block *super, node_id_t node_id, cyg_bool save)
{
	cyg_uint32	wr_len;
	cyg_uint32 node_byte = node_id >> 3;
	cyg_uint32 node_bit  = node_id & 0x7;
		
	inode_bitmap[node_byte] |= (1 << node_bit);
	super->s_free_inodes_count++;
	
	if(save)
	{
		int err;
		err = block_write_super(super);
		if(err != ENOERR)
			return err;
		
		err = block_write(super,super->block_goroup_info.bg_inode_bitmap,node_byte,
								sizeof(cyg_uint8),&inode_bitmap[node_byte],&wr_len);
		
		if(err != ENOERR)
			return err;
		
		if(wr_len != sizeof(cyg_uint8))
			return EIO;
	}
	return ENOERR;
}

/** node_set_used function
/*
/* This function set specified node as used and optionally save node bitmap to disk.
/* This function actualize free node count as well.
/* \param super - pointer to super block structure.
/* \param node_id - id of node to set used.
/* \param save if true node bitmap is actualized on disk.
/* \return IO error
*/
int node_set_used(f2bfs_super_block *super, node_id_t node_id,cyg_bool save)
{
	cyg_uint32	wr_len;
	cyg_uint32 node_byte = node_id >> 3;
	cyg_uint32 node_bit  = node_id & 0x7;;
	
	if((inode_bitmap[node_byte] & (1 << node_bit)) == 0)
	{
		return EINVAL;
	}
	
	inode_bitmap[node_byte] &= ~(1 << node_bit);
	super->s_free_inodes_count--;
	
	if(save)
	{
		int err;
		err = block_write_super(super);
		if(err != ENOERR)
			return err;
		
		err = block_write(super,super->block_goroup_info.bg_inode_bitmap,node_byte,
								sizeof(cyg_uint8),&inode_bitmap[node_byte],&wr_len);
		if(err != ENOERR)
			return err;
		
		if(wr_len != sizeof(cyg_uint8))
			return EIO;
		
		return err;
	}
	
	return ENOERR;
}

/** node_get_status function
/*
/* This function return state of specified node used/free.
/* \param super - pointer to super block structure.
/* \param node_id - if of node to check its state.
/* \return 0 - if node is use, 1 - if free.
*/
int node_get_status(f2bfs_super_block *super, node_id_t node_id)
{
	cyg_uint32 node_byte = node_id >> 3;
	cyg_uint32 node_bit  = node_id & 0x7;
	
	if((inode_bitmap[node_byte] & (1 << node_bit)) != 0)
		return 1;
	return 0;
}

/** node_invalidate function
/*
/* This function invalidate specified node.
/* First block bitmap is actualized to actualize free blocks 
/* on disk then node it self is set as free in function \see node_set_free.
/* \param super - pointer to super block structure.
/* \param node  - pointer to node structure to free.
/* \return IO error
*/
int node_invalidate(f2bfs_super_block *super,f2bfs_inode *node)
{
	int err;
	err = block_write_bitmap(super);
	if(err != ENOERR)
		return err;	

	return node_set_free(super,node->i_node_id,true);
}

/** node_validate function.
/*
/* This function validate node data. In this function node data are
/* write to disk, if node is not set as used it is set as used
/* and block bitmap is witen to the disk.
/* \param super - pointer to block super structure.
/* \param node - pointer to node structur to validate
/* \return IO Error
*/ 
int node_validate(f2bfs_super_block *super,f2bfs_inode *node)
{
	int err;
		
	err = node_write(super,node);
	if(err != ENOERR)
		return err;
	if(node_get_status(super,node->i_node_id) == 1)
		err = node_set_used(super,node->i_node_id,true);
		
	err = block_write_bitmap(super);
	if(err != ENOERR)
		return err;
	return err;
}

/** node_read_data function.
/*
/* This function rads node data.
/* \param super - pointer to super block structure.
/* \param node - pointer to node structure.
/* \param pos - start offset to read from.
/* \param data - pointer to array where to read data.
/* \param len  - point to variable containing length of data.
/* \return IO Error.
*/
int node_read_data(f2bfs_super_block *super, f2bfs_inode *node, cyg_uint32 pos, 
							void *data, cyg_uint32 *len)
{
	int err;
	cyg_uint32	rd_len;
	char *c_data = (char*)data;	
	if(*len == 0)
	{
		*len = 0;
		return ENOERR;	
	}
	
	if(pos >= node->i_size)
	{
		*len = 0;
		return EEOF;
	}
		
	cyg_uint32	block = pos >> super->s_log_block_size;
	cyg_uint32	offset = pos &  (super->s_block_size - 1);
	
	cyg_uint32  length = *len;
	if((pos + length) > node->i_size)
		*len = length = node->i_size - pos;
	
	while (length) 
	{
		if(block < F2BFS_NDIR_BLOCKS)
		{
			err = block_read(super,node->i_block[block],offset,length,c_data,&rd_len);
			if(err != ENOERR)
				return err;
		}
		else
		{
			cyg_uint32 indirect_block_id;
			cyg_uint32 indirect_block  = (block - F2BFS_NDIR_BLOCKS)/F2BFS_INDIRECT_PER_BLOCK;
			cyg_uint32 indirect_offset = (block - F2BFS_NDIR_BLOCKS)%F2BFS_INDIRECT_PER_BLOCK;
			
			/*printf("Pos %x block %x\n",(int)pos,(int)block);
			printf("Indirect block %x offset %x\n",(int)indirect_block,(int)indirect_offset);*/
			err = block_read(super,node->i_indirect_block[indirect_block],indirect_offset*sizeof(block_id_t),
					sizeof(block_id_t),&indirect_block_id,&rd_len);
			if(err != ENOERR || rd_len != sizeof(block_id_t))
				return EIO;
			
			//printf("Read indirect block %x offset %x\n",(int)indirect_block_id,(int)offset);	
			err = block_read(super,indirect_block_id,offset,length,c_data,&rd_len);
			if(err != ENOERR )
				return EIO;	
		}		
		
		length   -= rd_len;
		c_data   += rd_len;
		block++;
		offset = 0;			
	}
	
	//*len = length;
	return ENOERR;
}

/** node_write_data function.
/*
/* This function write node data.
/* \param super - pointer to super block structure.
/* \param node - pointer to node structure.
/* \param pos - start offset to read from.
/* \param data - pointer to array where to read data.
/* \param len  - point to variable containing length of data.
/* \return IO Error.
*/
int node_write_data(f2bfs_super_block *super, f2bfs_inode *node, cyg_uint32 pos, 
                               const void *data, cyg_uint32 *len)
{
	cyg_uint32 rd_len;
	cyg_uint32 wr_len;
	cyg_uint32 add_to_node_length = 0;
	int err;
	char *c_data = (char*)data;	
	
	cyg_uint32	block = pos >> super->s_log_block_size;
	cyg_uint32	offset = pos &  (super->s_block_size - 1);
	cyg_uint32  length = *len;
	
	if((pos + length) > node->i_size)
	{
		if(pos <= node->i_size)
		{
			err = node_alloc_space(super,node, length);
			if(err != ENOERR)
				return err;
				
			add_to_node_length = length;
		}
		else
		{
			add_to_node_length = length + pos - node->i_size;
			err = node_alloc_space(super,node, add_to_node_length);
			if(err != ENOERR)
				return err;
		} 
	}
	
	while(length) 
	{
		if(block < F2BFS_NDIR_BLOCKS)
		{
			#if NODE_DIAG_LEVEL > 0
			diag_printf("Write to block %d\n",node->i_block[block]);
			#endif
			if(node->i_block[block] > super->s_blocks_count)
				print_node(node);
			err = block_write(super,node->i_block[block],offset,length,c_data,&wr_len);
			if(err != ENOERR)
				return err;
		}
		else
		{
			block_id_t indirect_block_id;
			cyg_uint32 indirect_block  = (block - F2BFS_NDIR_BLOCKS)/F2BFS_INDIRECT_PER_BLOCK;
			cyg_uint32 indirect_offset = (block - F2BFS_NDIR_BLOCKS)%F2BFS_INDIRECT_PER_BLOCK;
			
			/*printf("Pos %x block %x\n",(int)pos,(int)block);
			printf("Indirect block %x offset %x\n",(int)indirect_block,(int)indirect_offset);*/
			
			err = block_read(super,node->i_indirect_block[indirect_block],indirect_offset*sizeof(block_id_t),
				sizeof(block_id_t),&indirect_block_id,&rd_len);
			if(err != ENOERR)
				return err;
			if(rd_len != sizeof(block_id_t) )
				return EIO;
			#if NODE_DIAG_LEVEL > 0
			diag_printf("Write indirect block %x offset %x\n",(int)indirect_block_id,(int)offset);
			#endif
			err = block_write(super,indirect_block_id,offset,length,c_data,&wr_len);
			if(err != ENOERR)
				return err;
		}
		
		length   -= wr_len;
		c_data   += wr_len;
		block++;
		offset = 0;
	}
		
	node->i_size += add_to_node_length;	
	
	return err;
}

/** node_rewrite_data function.
/*
/* This function rewrite node data.
/* \param super - pointer to super block structure.
/* \param node - pointer to node structure.
/* \param pos - start offset to read from.
/* \param data - pointer to array where to read data.
/* \param len  - point to variable containing length of data.
/* \return IO Error.
*/
int node_rewrite_data(f2bfs_super_block *super, f2bfs_inode *node, cyg_uint32 pos, 
                               const void *data, cyg_uint32 *len)
{
	cyg_uint32 rd_len;
	cyg_uint32 wr_len;
	cyg_uint32 add_to_node_length = 0;
	int err;
	char *c_data = (char*)data;	
	
	cyg_uint32	block = pos >> super->s_log_block_size;
	cyg_uint32	offset = pos &  (super->s_block_size - 1);
	cyg_uint32  length = *len;
	
	if((pos + length) > node->i_size)
	{
		if(pos <= node->i_size)
		{
			err = node_alloc_space(super,node, length);
			if(err != ENOERR)
				return err;
				
			add_to_node_length = length;
		}
		else
		{
			add_to_node_length = length + pos - node->i_size;
			err = node_alloc_space(super,node, add_to_node_length);
			if(err != ENOERR)
				return err;
		} 
	}
	
	while(length) 
	{
		if(block < F2BFS_NDIR_BLOCKS)
		{
			#if NODE_DIAG_LEVEL > 0
			diag_printf("Write to block %d\n",node->i_block[block]);
			#endif
			if(node->i_block[block] > super->s_blocks_count)
				print_node(node);
			err = block_errase(super,node->i_block[block]);
			if(err != ENOERR)
				return err;
			err = block_write(super,node->i_block[block],offset,length,c_data,&wr_len);
			if(err != ENOERR)
				return err;
		}
		else 
		{
			block_id_t indirect_block_id;
			cyg_uint32 indirect_block  = (block - F2BFS_NDIR_BLOCKS)/F2BFS_INDIRECT_PER_BLOCK;
			cyg_uint32 indirect_offset = (block - F2BFS_NDIR_BLOCKS)%F2BFS_INDIRECT_PER_BLOCK;
			
			/*printf("Pos %x block %x\n",(int)pos,(int)block);
			printf("Indirect block %x offset %x\n",(int)indirect_block,(int)indirect_offset);*/
			
			err = block_read(super,node->i_indirect_block[indirect_block],indirect_offset*sizeof(block_id_t),
				sizeof(block_id_t),&indirect_block_id,&rd_len);
			if(err != ENOERR)
				return err;
			if(rd_len != sizeof(block_id_t) )
				return EIO;
			#if NODE_DIAG_LEVEL > 0
			diag_printf("Write indirect block %x offset %x\n",(int)indirect_block_id,(int)offset);
			#endif
			err = block_errase(super,indirect_block_id);
			if(err != ENOERR)
				return err;
			err = block_write(super,indirect_block_id,offset,length,c_data,&wr_len);
			if(err != ENOERR)
				return err;
		}
		
		length   -= wr_len;
		c_data   += wr_len;
		block++;
		offset = 0;
	}
		
	node->i_size += add_to_node_length;	
	return err;
}

/** node_alloc_space function
/*
/* This function alloc specified space for specified size. If specified
/* space can not be alloc error is returned. If file flag is set 
/* FILE_FLAG_SYSTEM_FILE space is alloc in reserved FRAM space.
/* \param super - pinter to super block structure.
/* \param node - pointer to node structure. 
/* \param size - number of bytes to allock.
/* \return IO Error.
*/
int node_alloc_space(f2bfs_super_block *super,f2bfs_inode *node, size_t size)
{
	int err;
	cyg_uint32	wr_len;
	cyg_uint32	blocks_to_alloc = 0;
	block_id_t	new_blk;
	cyg_uint32  node_space_left = ((node->i_blocks << super->s_log_block_size) - node->i_size);
	
	// zbyva jeste dost mista
	if( size <= node_space_left)
	{
		#if NODE_DIAG_LEVEL > 0
		diag_printf("Space left: %d\n",node_space_left);
		#endif
		return ENOERR;
	}
		
	if(size < (super->s_block_size + node_space_left))
		blocks_to_alloc = 1;
	else
		blocks_to_alloc = (size >> super->s_log_block_size) + 1;
		
	do 
	{
		if(node->i_flags == FILE_FLAG_SYSTEM_FILE )
			err = block_alloc_reserved(super,&new_blk);
		else
		{
			err = block_alloc(super,&new_blk);
			#if NODE_DIAG_LEVEL > 0
			diag_printf("New block allocked: %d\n",(int)new_blk);
			diag_printf("size: %d, Node len: %d\n",size,node->i_size);
			#endif
		}
		
		if(err != ENOERR)
			return err;
	
		if(node->i_blocks < F2BFS_NDIR_BLOCKS)
		{
			node->i_block[node->i_blocks++] = new_blk;
		}
		else 
		{
			cyg_uint32 indirect_block  = (node->i_blocks - F2BFS_NDIR_BLOCKS)/F2BFS_INDIRECT_PER_BLOCK;
			cyg_uint32 indirect_offset = (node->i_blocks - F2BFS_NDIR_BLOCKS)%F2BFS_INDIRECT_PER_BLOCK;
			
			if(indirect_offset == 0)
			{
				if(indirect_block > F2BFS_IND_BLOCK)
					return EFBIG;
				
				node->i_indirect_block[indirect_block] = new_blk;
				
				err = block_alloc(super,&new_blk);
			
				if(err != ENOERR)
					return err;
				#if NODE_DIAG_LEVEL > 0
				diag_printf("New address block alloceked: %d\n",(int)new_blk);
				#endif
			}
				
			err = block_write(super,node->i_indirect_block[indirect_block],
						indirect_offset*sizeof(block_id_t),sizeof(block_id_t),
							&new_blk,&wr_len);	
						
			if(err != ENOERR)
				return err;
			if(wr_len != sizeof(block_id_t))
				return EIO;
			node->i_blocks++;
		}
		
		blocks_to_alloc--;
	} while (blocks_to_alloc);
	
	return err;
}

/** node_remove function
/*
/* This function remove specified node from disk.
/* \param super - pointer to super block structure.
/* \param node - pointer to node structure to remove from disk.
/* \return IO Error.
*/
int node_remove(f2bfs_super_block *super,f2bfs_inode *node)
{
	int err = node_free_space(super,node);
	if(err != ENOERR)
		return err;
	return node_set_free(super,node->i_node_id,true);
}

/** node_free_space function
/*
/* This function free all block node has alloc.
/* \param super - pointer to super block structure.
/* \param node - pointer to node structure to reomve its data.
/* \return IO Error.
*/
int node_free_space(f2bfs_super_block *super,f2bfs_inode *node)
{
	int err,i;
	cyg_uint32	rd_len;
	block_id_t	free_blk = 0;
		
	do 
	{
		if(free_blk < F2BFS_NDIR_BLOCKS)
		{
			err = block_set_free(super,node->i_block[free_blk],false);
			if(err != ENOERR)
				return err;
		}
		else
		{
			block_id_t indirect_bblock_rd;
			cyg_uint32 indirect_block  = (free_blk - F2BFS_NDIR_BLOCKS)/F2BFS_INDIRECT_PER_BLOCK;
			cyg_uint32 indirect_offset = (free_blk - F2BFS_NDIR_BLOCKS)%F2BFS_INDIRECT_PER_BLOCK;
	
			err = block_read(super,node->i_indirect_block[indirect_block],
							indirect_offset*sizeof(block_id_t),sizeof(block_id_t),
								&indirect_bblock_rd,&rd_len);
			
			if(err != ENOERR)
				return err;
				
			if(rd_len != sizeof(block_id_t))
				return EIO;
			
			err = block_set_free(super,indirect_bblock_rd,false);
			
			if(err != ENOERR)
				return err;
			
		}
		
		free_blk++;
	} while (free_blk < node->i_blocks);
	
	for(i = 0; i < CYGNUM_F2BFS_BLOCKS_INDIRECT1; i++)
	{
		if(node->i_indirect_block[i] == 0)
			break;
			
		err = block_set_free(super,node->i_indirect_block[i],false);
		if(err != ENOERR)
			return err;
	}
		
	node->i_size	 = 0;
	node->i_blocks   = 0;	
	memset(node->i_indirect_block,0,sizeof(node->i_indirect_block));
	memset(node->i_block,0,sizeof(node->i_block));
	// zase kriticky moment
	err = block_write_bitmap(super);
	if(err != ENOERR)
		return err;	
	err = node_write(super,node);
	return err;	
}

/** print_node function
/*
/* Service function to print node values.
/* \param node - pinter to node to print.
*/
void print_node(f2bfs_inode *node)
{
	#if NODE_DIAG_LEVEL > 0
    int i;
	diag_printf("Node at %p\n",node);
	diag_printf("Node id %d\n",node->i_node_id);
	
	for(i = 0; i < F2BFS_NDIR_BLOCKS; i++)
	{
		diag_printf("Direct %d, block %d\n",i,node->i_block[i]);
	}
	
	for(i = 0; i < F2BFS_IND_BLOCK; i++)
	{
		diag_printf("Indirect %d, block %d\n",i,node->i_indirect_block[i]);
	}
	#endif
}