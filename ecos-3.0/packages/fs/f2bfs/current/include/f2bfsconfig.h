/*
 * BlockFSConfig.h
 *
 * Created: 17.4.2013 9:44:02
 *  Author: Filip
 */ 


#ifndef BLOCKFSCONFIG_H_
#define BLOCKFSCONFIG_H_

#ifndef FSDEVEL
#include <pkgconf/fs_fbfs.h>
#else

#define CYGNUM_F2BFS_FIRST_BLOCK_ADDRESS  
#define CYGNUM_F2BFS_BLOCKS_DIRECT			4
#define CYGNUM_F2BFS_BLOCKS_INDIRECT1		4
#define CYGNUM_F2BFS_BLOCK_SIZE			    4096
#define CYGNUM_F2BFS_LOG_BLOCK_SIZE			12//4096
#define CYGNUM_F2BFS_DIRENT_NAME_SIZE       20
#define CYGNUM_F2BFS_FRAM_SIZE			    262144
#define CYGNUM_F2BFS_FLASH_SIZE             134217728
// eCos falsh io flash fram start address configured in board config file
#define CYGNUM_F2BFS_FIRST_BLOCK_ADDRESS   0x20000000

#define   CYGNUM_F2BFS_FRAM_DEV_NAME        fm25wxx_spi_fram
#define   CYGNUM_F2BFS_FLASH_DEV_NAME       n25qxx_spi_flash
#endif

#define F2BFS_BLOCK_FLASH_OFFSET                CYGNUM_F2BFS_FIRST_BLOCK_ADDRESS
#define CYGNUM_F2BFS_DISK_SIZE			(CYGNUM_F2BFS_FRAM_SIZE + CYGNUM_F2BFS_FLASH_SIZE)/8
#define F2BFS_NUM_BOLOCKS				(CYGNUM_F2BFS_DISK_SIZE/CYGNUM_F2BFS_BLOCK_SIZE + 1)
#define F2BFS_NUM_FRAM_BLOCKS			(CYGNUM_F2BFS_FRAM_SIZE/8)/CYGNUM_F2BFS_BLOCK_SIZE
#define F2BFS_NUM_BLOCK_GROUPS			1


/*
 * Constants relative to the data blocks
 */
#define	F2BFS_NDIR_BLOCKS		CYGNUM_F2BFS_BLOCKS_DIRECT		/* num of direct blocks*/
#define	F2BFS_IND_BLOCK			CYGNUM_F2BFS_BLOCKS_INDIRECT1   /* num of first indirect blocks */
#define	F2BFS_N_BLOCKS			(F2BFS_NDIR_BLOCKS + F2BFS_IND_BLOCK)



#define F2BFS_MAGIC						0xfbf0


#define FILE_FLAG_FILE					    0
#define FILE_FLAG_SYSTEM_FILE			    1

// crc error not present in standard error values
#define ECRC32								400

#define BLOCK_FS_STATIC_DTATA_ATRIBUTES //__attribute__ ((section (".my_section")))


// -------------------------------------------------------------------------
// Block allocator parameters

// The number of nodes per block
#define F2BFS_NODES_PER_BLOCK (CYGNUM_F2BFS_BLOCK_SIZE/sizeof(f2bfs_inode))

// The number of indirect pointers that can be stored in a single data block
#define F2BFS_INDIRECT_PER_BLOCK (CYGNUM_F2BFS_BLOCK_SIZE/sizeof(block_id_t))

// The number of directory entries that can be stored in a single data block
#define F2BFS_DIRENT_PER_BLOCK  (CYGNUM_F2BFS_BLOCK_SIZE/sizeof(f2bfs_dir_entry))

/*
 * Structure of a blocks group descriptor
 */
typedef struct f2bfs_group_desc_s
{
	cyg_uint32	bg_block_bitmap;		/* Blocks bitmap block */
	cyg_uint32	bg_inode_bitmap;		/* Inodes bitmap block */
	cyg_uint32	bg_inode_table;		/* Inodes table block */
	cyg_uint16	bg_used_dirs_count;	/* Directories count */
	cyg_uint16	bg_start_search;    /* From wich block in group search starts */
}f2bfs_group_desc;

/*
 * Structure of an inode on the disk
 */
typedef struct f2bfs_inode_s {
	cyg_uint32  i_refcnt;
	cyg_uint32  i_node_id;  /* Node id */
	cyg_uint16	i_mode;		/* File mode */
	cyg_uint32	i_size;		/* Size in bytes */
	cyg_uint32	i_atime;	/* Access time */
	cyg_uint32	i_ctime;	/* Creation time */
	cyg_uint32	i_mtime;	/* Modification time */
	cyg_uint32	i_dtime;	/* Deletion Time */
	cyg_uint16	i_links_count;	/* Links count */
	cyg_uint32	i_blocks;	/* Blocks count */
	cyg_uint32	i_flags;	/* File flags */

	cyg_uint32	i_block[F2BFS_NDIR_BLOCKS];/* Pointers to blocks */
	cyg_uint32  i_indirect_block[F2BFS_IND_BLOCK];
	cyg_uint32  i_crc32;
}f2bfs_inode;


/*
 * Structure of a directory entry
 */

typedef struct f2bfs_dir_entry_s {
	cyg_uint16	inode;			/* Inode number 0xffff not used*/
	cyg_uint16	rec_len;		/* Directory entry length nemusi byt bo budou stejne dluhe*/
	cyg_uint16	name_len;		/* Name length */
	cyg_uint16	offset;         /* node offset freom the start of direcotry file*/
	char	    name[CYGNUM_F2BFS_DIRENT_NAME_SIZE];	/* File name, up to CYGNUM_F2BFS_DIRENT_NAME_SIZE */
	cyg_uint32  crc32;
}f2bfs_dir_entry;


/*
 * Structure of the super block
 */
typedef struct f2bfs_super_block_s {
	cyg_uint32	s_inodes_count;		/* Inodes count */
	cyg_uint32	s_blocks_count;		/* Blocks count */
	cyg_uint32	s_r_blocks_count;	/* Reserved blocks count */
	cyg_uint32	s_free_blocks_count;	/* Free blocks count */
	cyg_uint32	s_free_blocks_count_data;	/* Free blocks count */
	cyg_uint32	s_free_inodes_count;	/* Free inodes count */
	cyg_uint32	s_first_data_block_reserved;	/* First Data Block in reserved space */
	cyg_uint32	s_first_data_block;	/* First Data Block */
	cyg_uint32	s_log_block_size;	/* Block size */
	cyg_uint32	s_block_size;	/* Block size */
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
	
	f2bfs_group_desc	block_goroup_info;//[F2BFS_NUM_BLOCK_GROUPS]; /* info of block group only one in for naw */
	cyg_uint32  s_crc32;
}f2bfs_super_block;

//==========================================================================
// Directory search data
// Parameters for a directory search. The fields of this structure are
// updated as we follow a pathname through the directory tree.

typedef struct f2bfs_dirsearch_s
{
	f2bfs_inode			*dir;           // directory to search
	const char          *path;          // path to follow
	f2bfs_inode			*node;          // Node found
	const char          *name;          // last name fragment used
	int                 namelen;        // name fragment length
	cyg_bool            last;           // last name in path?
}f2bfs_dirsearch;

#endif /* BLOCKFSCONFIG_H_ */
