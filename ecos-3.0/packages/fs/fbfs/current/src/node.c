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
#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <pkgconf/io_fileio.h>
#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <cyg/fileio/fileio.h>
#include <cyg/io/fm25wxx.h>
#include <cyg/io/n25qxx.h>
#include <cyg/crc/crc.h>
#include <cyg/fs/blockfsconfig.h>
#include <cyg/fs/block.h>
#include <cyg/fs/node.h>
#include <malloc.h>

//node bitmap RAM copy
cyg_uint8 inode_bitmap[BLOCKFS_NODES_PER_BLOCK/8] BLOCK_FS_STATIC_DTATA_ATRIBUTES;


static int find_first_one(cyg_uint8 mask)
{
    int pos = -1;
    if(mask)
    {
        pos = 0;
        while (!(mask & (1 << pos)))
        {
            pos++;;
        }
    }

    return pos;
}

cyg_uint32 node_find_free(fbfs_super_block *super,node_id_t *new_node_id)
{
    int start,bit;

    for(start = 0; start < super->s_inodes_count >> 3; start++)
    {
        if(inode_bitmap[start] == 0)
            continue;

        bit = find_first_one(inode_bitmap[start]);
        CYG_ASSERT(bit != -1,"Failed to find block to alloc\n");

        *new_node_id = start*8 + bit;
        return ENOERR;
    }

    return ENOSPC;
}

fbfs_inode *node_alloc(fbfs_super_block *super, mode_t mode )
{
    cyg_uint32 err;
    fbfs_inode *node = (fbfs_inode*)malloc(sizeof(fbfs_inode));
    CYG_ASSERT(node,"Cannot alloc new node\n");
    memset( node , 0, sizeof(fbfs_inode) );

    err = node_find_free(super,&node->i_node_id);
    if(err != ENOERR)
    {
        node_free(super,node);
        return NULL;
    }

    node->i_mode	= mode;
    node->i_atime	=
    node->i_mtime	= 
    node->i_ctime	= cyg_timestamp();

    return node;
}

int node_free(fbfs_super_block *super, fbfs_inode * node)
{
    if(node != NULL)
        free(node);
    return ENOERR;
}

fbfs_inode *node_alloc_read(fbfs_super_block *super,node_id_t node_id)
{
    if(node_id < super->s_inodes_count)
    {
        cyg_uint32			red_len;
        fbfs_inode			*node;
        cyg_uint32			ret;

        node = (fbfs_inode*)malloc(sizeof(fbfs_inode));
        CYG_ASSERT(node,"Cannot allocated space for node\n");

        ret = block_read(super,super->block_goroup_info.bg_inode_table,
                   node_id*sizeof(fbfs_inode),sizeof(fbfs_inode),node,&red_len);

        if(ret != ENOERR || red_len != sizeof(fbfs_inode))
        {
            free(node);
            return NULL;
        }
        return node;
    }
    CYG_FAIL("Node boundaries failed\n");
    return NULL;		
}

int node_read(fbfs_super_block *super,fbfs_inode *node,node_id_t node_id)
{
    if(node_id < super->s_inodes_count)
    {
        cyg_uint32			red_len;
        cyg_uint32			ret;

        ret = block_read(super,super->block_goroup_info.bg_inode_table,
        node_id*sizeof(fbfs_inode),sizeof(fbfs_inode),node,&red_len);

        if(ret != ENOERR || red_len != sizeof(fbfs_inode))
            return EIO;
        return ENOERR;
    }
    return EIO;
}

int node_write(fbfs_super_block *super, fbfs_inode *node)
{
    if(node->i_node_id < super->s_inodes_count)
    {
        int err;
        cyg_uint32	wr_len;
        err = block_write(super,super->block_goroup_info.bg_inode_table,
            node->i_node_id*sizeof(fbfs_inode),sizeof(fbfs_inode),node,&wr_len);
        if( wr_len != sizeof(fbfs_inode))
            return EIO;
        return err;
    }
    CYG_FAIL("Node write budaries failed\n");
    return EIO;
}

int node_read_data(fbfs_super_block *super, fbfs_inode *node, cyg_uint32 pos, 
        void *data, cyg_uint32 *len)
{
    int err;
    cyg_uint32	rd_len;
    char *c_data = (char*)data;	
    if(pos >= node->i_size || *len == 0 )
    {
        *len = 0;
        return ENOERR;
    }

    cyg_uint32	block = pos/super->s_log_block_size;
    cyg_uint32	offset = pos%super->s_log_block_size;

    cyg_uint32  length = *len;
    if((pos + *len) > node->i_size)
        *len = length = node->i_size - pos;

    do 
    {
        if(block < FBS_NDIR_BLOCKS)
        {
            err = block_read(super,node->i_block[block],offset,length,c_data,&rd_len);
            if(err != ENOERR)
                return err;
        }
        else //if(block >= FBS_NDIR_BLOCKS)
        {
            cyg_uint32 indirect_block_id;
            cyg_uint32 indirect_block  = (block - FBS_NDIR_BLOCKS)/BLOCKFS_INDIRECT_PER_BLOCK;
            cyg_uint32 indirect_offset = (block - FBS_NDIR_BLOCKS)%BLOCKFS_INDIRECT_PER_BLOCK;

            /*printf("Pos %x block %x\n",(int)pos,(int)block);
            printf("Indirect block %x offset %x\n",(int)indirect_block,(int)indirect_offset);*/
            err = block_read(super,node->i_indirect_block[indirect_block],
                            indirect_offset*sizeof(block_id_t),
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
    } while (length);

    return ENOERR;
}

int node_write_data(fbfs_super_block *super, fbfs_inode *node, cyg_uint32 pos, 
        const void *data, cyg_uint32 *len)
{
    cyg_uint32 wr_len;
    cyg_uint32 rd_len;
    int err;
    char *c_data = (char*)data;	
    if(len == 0)
        return ENOSPC;
	
    cyg_uint32	block = pos/super->s_log_block_size;
    cyg_uint32	offset = pos%super->s_log_block_size;

    cyg_uint32  length = *len;
    do 
    {
        if(pos >= node->i_size)
        {
            err = node_alloc_space(super,node,*len);
            if(err != ENOERR)
                return err;
        }

        if(block < FBS_NDIR_BLOCKS)
        {
            err = block_write(super,node->i_block[block],offset,length,c_data,&wr_len);
            if(err != ENOERR)
                return err;

        }
        else //if(block >= FBS_NDIR_BLOCKS)
        {
            cyg_uint32 indirect_block_id;
            cyg_uint32 indirect_block  = (block - FBS_NDIR_BLOCKS)/BLOCKFS_INDIRECT_PER_BLOCK;
            cyg_uint32 indirect_offset = (block - FBS_NDIR_BLOCKS)%BLOCKFS_INDIRECT_PER_BLOCK;

            /*printf("Pos %x block %x\n",(int)pos,(int)block);
            printf("Indirect block %x offset %x\n",(int)indirect_block,(int)indirect_offset);*/

            err = block_read(super,node->i_indirect_block[indirect_block],
                    indirect_offset*sizeof(block_id_t),
                    sizeof(block_id_t),&indirect_block_id,&rd_len);
            
            if(err != ENOERR)
                return err;
            if(rd_len != sizeof(block_id_t) )
                return EIO;

            //printf("Write indirect block %x offset %x\n",(int)indirect_block_id,(int)offset);
            err = block_write(super,indirect_block_id,offset,length,c_data,&wr_len);
            if(err != ENOERR)
                return err;
        }

        length   -= wr_len;
        c_data   += wr_len;
        block++;
        offset = 0;
    }while(length);

    return ENOERR;
}

int node_write_bitmap(fbfs_super_block *super, node_id_t node_id)
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
        ret = block_write(super,super->block_goroup_info.bg_inode_bitmap,
                node_id >> 3,
                sizeof(cyg_uint8),&inode_bitmap[node_id >> 3],&wr_len);

        if(wr_len != sizeof(cyg_uint8))
            return EIO;
    }

    return ret;
}

int node_read_bitmap(fbfs_super_block *super)
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

int node_set_free(fbfs_super_block *super, node_id_t node_id,cyg_bool save)
{
    cyg_uint32	wr_len;
    inode_bitmap[node_id >> 3] |= (1 << (node_id&0x07));
    super->s_free_inodes_count++;
    super->block_goroup_info.bg_free_inodes_count++;
    if(save)
    {
        int err;
        err = block_write_super(super);
        if(err != ENOERR)
            return err;

        err = block_write(super,super->block_goroup_info.bg_inode_bitmap,
                node_id >> 3,
                sizeof(cyg_uint8),&inode_bitmap[node_id >> 3],&wr_len);

        if(err != ENOERR)
            return err;

        if(wr_len != sizeof(cyg_uint8))
            return EIO;
    }
    return ENOERR;	
}

int node_set_used(fbfs_super_block *super, node_id_t node_id,cyg_bool save)
{
    cyg_uint32	wr_len;

    if((inode_bitmap[node_id >> 3] & (1 << (node_id&0x07))))
    {
        inode_bitmap[node_id >> 3] &= ~(1 << (node_id&0x07));
        super->s_free_inodes_count--;
        super->block_goroup_info.bg_free_inodes_count--;
    }
    if(save)
    {
        int err;
        err = block_write_super(super);
        if(err != ENOERR)
                return err;

        err = block_write(super,super->block_goroup_info.bg_inode_bitmap,
                node_id >> 3,
                sizeof(cyg_uint8),&inode_bitmap[node_id >> 3],&wr_len);
        
        if(err != ENOERR)
            return err;

        if(wr_len != sizeof(cyg_uint8))
            return EIO;

        return block_write_super(super);
    }	

    return ENOERR;	
}

int node_alloc_space(fbfs_super_block *super,fbfs_inode *node, size_t size)
{
    int err;
    cyg_uint32	wr_len;
    cyg_uint32	blocks_to_alloc = 0;
    block_id_t	new_blk;

    if((node->i_blocks*super->s_log_block_size - node->i_size) >= size)
        return ENOERR;


    if(size < (super->s_log_block_size + node->i_blocks*super->s_log_block_size
            - node->i_size))
        blocks_to_alloc = 1;
    else
        blocks_to_alloc = size/super->s_log_block_size + 1;

    do 
    {
        if(node->i_flags == FILE_FLAG_SYSTEM_FILE )
            err = block_alloc_reserved(super,&new_blk);
        else
            err = block_alloc(super,&new_blk);

        if(err != ENOERR)
            return err;

        if(node->i_blocks < FBS_NDIR_BLOCKS)
        {
            node->i_block[node->i_blocks++] = new_blk;
        }
        else //if(node->i_blocks >= FBS_NDIR_BLOCKS)
        {
            cyg_uint32 indirect_block  = 
                (node->i_blocks - FBS_NDIR_BLOCKS)/BLOCKFS_INDIRECT_PER_BLOCK;
            cyg_uint32 indirect_offset = 
                (node->i_blocks - FBS_NDIR_BLOCKS)%BLOCKFS_INDIRECT_PER_BLOCK;
                
            if(indirect_offset == 0)
            {
                if(indirect_block >= FBS_IND_BLOCK)
                    return EFBIG;

                node->i_indirect_block[indirect_block] = new_blk;

                err = block_alloc(super,&new_blk);

                if(err != ENOERR)
                    return err;
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

    err = block_write_bitmap(super);
    if(err != ENOERR)
            return err;	
    err = node_write(super,node);
    return err;
}

int node_free_space(fbfs_super_block *super,fbfs_inode *node)
{
    int err;
    cyg_uint32	rd_len;
    block_id_t	free_blk = 0;
		
    do 
    {
        if(free_blk < FBS_NDIR_BLOCKS)
        {
            err = block_free(super,node->i_block[free_blk++]);
            if(err != ENOERR)
                return err;
        }
        else //if(node->i_blocks >= FBS_NDIR_BLOCKS)
        {
            cyg_uint32 indirect_block  =
                    (free_blk - FBS_NDIR_BLOCKS)/BLOCKFS_INDIRECT_PER_BLOCK;
            cyg_uint32 indirect_offset = 
                    (free_blk - FBS_NDIR_BLOCKS)%BLOCKFS_INDIRECT_PER_BLOCK;

            err = block_read(super,node->i_indirect_block[indirect_block],
                    indirect_offset*sizeof(block_id_t),sizeof(block_id_t),
                     &free_blk,&rd_len);

            if(err != ENOERR)
                return err;

            if(rd_len != sizeof(block_id_t))
                return EIO;

            err = block_free(super,free_blk);

            if(err != ENOERR)
                return err;

            if(indirect_offset == (super->s_log_block_size - sizeof(block_id_t)))
            {
                err = block_free(super,node->i_indirect_block[indirect_block]);

                if(err != ENOERR)
                    return err;
            }

            free_blk++;
        }

    } while (free_blk < node->i_blocks);

    node->i_size	 = 0;
    node->i_blocks   = 0;	
    err = block_write_bitmap(super);
    if(err != ENOERR)
            return err;	
    err = node_write(super,node);
    return err;	
}
