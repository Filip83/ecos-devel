/*
 * Node.h
 *
 * Created: 15.4.2013 14:16:53
 *  Author: Filip
 */ 


#ifndef __NODE__
#define __NODE__

typedef cyg_uint32 node_id_t;

f2bfs_inode *node_create(f2bfs_super_block *super, mode_t mode );
int node_close(f2bfs_super_block *super, f2bfs_inode * node);
f2bfs_inode* node_open(f2bfs_super_block *super, node_id_t node_id);

int node_read(f2bfs_super_block *super,f2bfs_inode *node,node_id_t node_id);
f2bfs_inode *node_alloc_read(f2bfs_super_block *super,node_id_t node_id);
int node_write(f2bfs_super_block *super, f2bfs_inode *node);

int node_read_data(f2bfs_super_block *super, f2bfs_inode *node, cyg_uint32 pos,
								 void *data, cyg_uint32 *len);
int node_write_data(f2bfs_super_block *super, f2bfs_inode *node, cyg_uint32 pos,  
								 const void *data, cyg_uint32 *len);
								 
int node_rewrite_data(f2bfs_super_block *super, f2bfs_inode *node, cyg_uint32 pos,
								const void *data, cyg_uint32 *len);

int node_alloc_space(f2bfs_super_block *super,f2bfs_inode *node, size_t size);
int node_free_space(f2bfs_super_block *super,f2bfs_inode *node);
int node_write_bitmap(f2bfs_super_block *super, node_id_t node_id);
int node_read_bitmap(f2bfs_super_block *super);
int node_invalidate(f2bfs_super_block *super,f2bfs_inode *node);
int node_validate(f2bfs_super_block *super,f2bfs_inode *node);
int node_remove(f2bfs_super_block *super,f2bfs_inode *node);

int node_set_free(f2bfs_super_block *super, node_id_t node_id,cyg_bool save);
int node_set_used(f2bfs_super_block *super, node_id_t node_id,cyg_bool save);
int node_get_status(f2bfs_super_block *super, node_id_t node_id);
#endif /* __NODE__ */
