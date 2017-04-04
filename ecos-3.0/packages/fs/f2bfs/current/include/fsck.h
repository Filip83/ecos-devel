/*
 * fsck.h
 *
 * Created: 3.7.2014 9:19:52
 *  Author: Filip
 */ 


#ifndef FSCK_H_
#define FSCK_H_

typedef struct f2bfs_inode_check_s
{
	node_id_t			node_id;
	cyg_bool            dir_node;
	cyg_uint32			blocks_count;
	cyg_uint32          indir_blocks;
	cyg_uint8			used_block_bitmap[F2BFS_NUM_BOLOCKS/8];
	cyg_uint32			satus;
}f2bfs_inode_check_t;

typedef struct f2bfs_dir_check_s
{
	f2bfs_dir_entry      dir;
	cyg_uint32			status;
}f2bfs_dir_check_t;


typedef struct f2bfs_check_s
{
	cyg_bool         super_ok;
	cyg_uint32       inode_count_bitmap;
	cyg_uint32       block_count_reserved_bitmap;
	cyg_uint32       block_count_bitmap;
	cyg_uint32		 inode_count;
	cyg_uint32       free_inodes;
	cyg_uint32       used_inodes;
	cyg_uint32       block_count;
	cyg_uint32       free_bock_count;
	cyg_uint32       used_bock_count;
	
	cyg_uint32       duplicate_block_count;
	cyg_uint32       dir_entries;
	
	cyg_uint8		 duplicate_block_bitmap[F2BFS_NUM_BOLOCKS/8];
	
	f2bfs_inode_check_t	**node_table;
	f2bfs_dir_check_t       *dir_table;
}f2bfs_check_t;

int f2bfs_collect();

#endif /* FSCK_H_ */