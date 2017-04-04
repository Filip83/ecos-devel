#ifndef BLOCKFSCONFIG_H_
#define BLOCKFSCONFIG_H_
//=============================================================================
//
//      blockfsconfig.h
//
//      SIMPLEFS configuration file
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2004 Free Software Foundation, Inc.                        
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
// Author(s):     Filip Adamec <filip.adamec.ez2@gmail.com>
// Contributors:  
// Date:          2013-10-22
// Purpose:       
// Description:   
//              
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <pkgconf/fs_fbfs.h>
/*#define CYGNUM_BLOCKFS_FIRST_BLOCK_ADDRESS  
#define CYGNUM_BLOCKFS_BLOCKS_DIRECT		4
#define CYGNUM_BLOCKFS_BLOCKS_INDIRECT1		2
#define CYGNUM_BLOCKFS_BLOCKS_INDIRECT2		0
#define CYGNUM_BLOCKFS_BLOCK_SIZE			4096
#define CYGNUM_BLOCKFS_DIRENT_NAME_SIZE     20*/
#define CYGNUM_BLOCKFS_DISK_SIZE (CYGNUM_BLOCKFS_FRAM_SIZE + CYGNUM_BLOCKFS_FLASH_SIZE)/8
#define BLOCKFS_NUM_BOLOCKS	 CYGNUM_BLOCKFS_DISK_SIZE/CYGNUM_BLOCKFS_BLOCK_SIZE
#define BLOCKFS_NUM_FRAM_BLOCKS	 (CYGNUM_BLOCKFS_FRAM_SIZE/8)/CYGNUM_BLOCKFS_BLOCK_SIZE
#define BLOCKFS_NUM_BLOCK_GROUPS			1


/*
 * Constants relative to the data blocks
 */
#define	FBS_NDIR_BLOCKS			CYGNUM_BLOCKFS_BLOCKS_DIRECT /* num of direct blocks*/
#define	FBS_IND_BLOCK			CYGNUM_BLOCKFS_BLOCKS_INDIRECT1 /* num of first indirect blocks */
#define	FBS_DIND_BLOCK			CYGNUM_BLOCKFS_BLOCKS_INDIRECT1 /* num of second indirect blocks */
#define	FBS_TIND_BLOCK			CYGNUM_BLOCKFS_BLOCKS_INDIRECT2 /* num of third indirect blocks */
#define	FBS_N_BLOCKS			(FBS_NDIR_BLOCKS + FBS_IND_BLOCK + FBS_DIND_BLOCK + FBS_TIND_BLOCK)

//#define BLOCKFS_BLOCK_INFO_ADDR				0x00000000
#define BLOCKFS_BLOCK_FLASH_OFFSET			0x20000000
//#define BLOCKFS_NODE_INFO_ADDR				0x00000000

#define BLOCKFS_MAGIC						0xfbf0

#define FBFS_ERROR_OK				        0
#define FBFS_ERROR					        1
#define FBFS_ERROR_BLOCK_OUTOF_RANGE		2
#define FBFS_ERR_SIZE		                3

#define FILE_FLAG_FILE					    0
#define FILE_FLAG_SYSTEM_FILE			    1

#define BLOCK_FS_STATIC_DTATA_ATRIBUTES //__attribute__ ((section (".my_section")))
/*
 * Structure of a blocks group descriptor
 */
typedef struct fbfs_group_desc_s
{
    cyg_uint32	bg_block_bitmap;        /* Blocks bitmap block */
    cyg_uint32	bg_inode_bitmap;        /* Inodes bitmap block */
    cyg_uint32	bg_inode_table;		/* Inodes table block */
    cyg_uint16	bg_free_blocks_count;	/* Free blocks count */
    cyg_uint16	bg_free_inodes_count;	/* Free inodes count */
    cyg_uint16	bg_used_dirs_count;	/* Directories count */
    cyg_uint16	bg_start_search;        /* From wich block in group search starts */
    //cyg_uint32	bg_reserved[3];
}fbfs_group_desc;

/*
 * Structure of an inode on the disk
 */
typedef struct fbfs_inode_s {
    cyg_uint32  i_refcnt;
    cyg_uint32  i_node_id;      /* Node id */
    cyg_uint16	i_mode;		/* File mode */
    cyg_uint32	i_size;		/* Size in bytes */
    cyg_uint32	i_atime;	/* Access time */
    cyg_uint32	i_ctime;	/* Creation time */
    cyg_uint32	i_mtime;	/* Modification time */
    cyg_uint32	i_dtime;	/* Deletion Time */
    cyg_uint16	i_links_count;	/* Links count */
    cyg_uint32	i_blocks;	/* Blocks count */
    cyg_uint32	i_flags;	/* File flags */

    cyg_uint32	i_block[FBS_NDIR_BLOCKS];/* Pointers to blocks */
    cyg_uint32  i_indirect_block[FBS_IND_BLOCK];
}fbfs_inode;


/*
 * Structure of a directory entry
 */

typedef struct fbfs_dir_entry_s {
    cyg_uint16	inode;			/* Inode number 0xffff not used*/
    cyg_uint16	rec_len;		/* Directory entry length nemusi byt bo budou stejne dluhe*/
    cyg_uint16	name_len;		/* Name length */
    cyg_uint16	offset;         /* node offset freom the start of direcotry file*/
    char	name[CYGNUM_BLOCKFS_DIRENT_NAME_SIZE];	/* File name, up to CYGNUM_BLOCKFS_DIRENT_NAME_SIZE */
}fbfs_dir_entry;


/*
 * Structure of the super block
 */
typedef struct fbfs_super_block_s {
    cyg_uint32	s_inodes_count;		/* Inodes count */
    cyg_uint32	s_blocks_count;		/* Blocks count */
    cyg_uint32	s_r_blocks_count;	/* Reserved blocks count */
    cyg_uint32	s_free_blocks_count;	/* Free blocks count */
    cyg_uint32	s_free_inodes_count;	/* Free inodes count */
    cyg_uint32	s_first_data_block_reserved;	/* First Data Block in reserved space */
    cyg_uint32	s_first_data_block;	/* First Data Block */
    cyg_uint32	s_log_block_size;	/* Block size */
    cyg_uint32	s_blocks_per_group;	/* # Blocks per group */
    cyg_uint32	s_inodes_per_group;	/* # Inodes per group */
    cyg_uint32	s_mtime;		/* Mount time */
    cyg_uint32	s_wtime;		/* Write time */
    cyg_uint16	s_mnt_count;		/* Mount count */
    cyg_uint16	s_max_mnt_count;	/* Maximal mount count */
    cyg_uint16	s_magic;		/* Magic signature */
    cyg_uint16	s_state;		/* File system state */
    cyg_uint16	s_minor_rev_level; 	/* minor revision level */
    cyg_uint32	s_lastcheck;		/* time of last check */
    cyg_uint32	s_checkinterval;	/* max. time between checks */
    cyg_uint32	s_rev_level;		/* Revision level */

    fbfs_group_desc	block_goroup_info[BLOCKFS_NUM_BLOCK_GROUPS]; /* info of block group only one in for naw */
    //fbfs_inode	    first_inode;	/* first inode root folder not delteabe */
}fbfs_super_block;

//==========================================================================
// Directory search data
// Parameters for a directory search. The fields of this structure are
// updated as we follow a pathname through the directory tree.

typedef struct blockfs_dirsearch_s
{
    fbfs_inode		*dir;           // directory to search
    const char          *path;          // path to follow
    fbfs_inode		*node;          // Node found
    const char          *name;          // last name fragment used
    int                 namelen;        // name fragment length
    cyg_bool            last;           // last name in path?
}blockfs_dirsearch;


// -------------------------------------------------------------------------
// Block allocator parameters

// The number of nodes per block
#define BLOCKFS_NODES_PER_BLOCK (CYGNUM_BLOCKFS_BLOCK_SIZE/sizeof(fbfs_inode))

// The number of indirect pointers that can be stored in a single data block
#define BLOCKFS_INDIRECT_PER_BLOCK (CYGNUM_BLOCKFS_BLOCK_SIZE/sizeof(block_id_t))

// The number of directory entries that can be stored in a single data block
#define BLOCKFS_DIRENT_PER_BLOCK  (CYGNUM_BLOCKFS_BLOCK_SIZE/sizeof(fbfs_dir_entry))

/*
// Number of bytes contained in a one level indirect block
#define BLOCKFS_INDIRECT1_BLOCK_EXTENT (BLOCKFS_INDIRECT_PER_BLOCK* \
CYGNUM_BLOCKFS_BLOCK_SIZE)

// number of bytes contained in a two level indirect block
#define BLOCKFS_INDIRECT2_BLOCK_EXTENT (BLOCKFS_INDIRECT_PER_BLOCK* \
BLOCKFS_INDIRECT_PER_BLOCK*             \
CYGNUM_BLOCKFS_BLOCK_SIZE)

// The maximum data offset for data directly accessed from the node
#define BLOCKFS_DIRECT_MAX        (CYGNUM_BLOCKFS_BLOCKS_DIRECT*CYGNUM_BLOCKFS_BLOCK_SIZE)

// The maximum data offset for data accessed from the single level indirect blocks
#define BLOCKFS_INDIRECT1_MAX     (BLOCKFS_DIRECT_MAX+                      \
(CYGNUM_BLOCKFS_BLOCKS_INDIRECT1*               \
BLOCKFS_INDIRECT1_BLOCK_EXTENT))

// The maximum data offset for data accessed from the two level indirect blocks
#define BLOCKFS_INDIRECT2_MAX     (BLOCKFS_INDIRECT1_MAX+                   \
(CYGNUM_BLOCKFS_BLOCKS_INDIRECT2*		        \
BLOCKFS_INDIRECT2_BLOCK_EXTENT))

// The maximum size of a file
#define BLOCKFS_FILESIZE_MAX      BLOCKFS_INDIRECT2_MAX

#define BLOCKFS_INDIRECT1_BLOCK_MASK		BLOCKFS_INDIRECT_PER_BLOCK - 1*/

#endif /* BLOCKFSCONFIG_H_ */
