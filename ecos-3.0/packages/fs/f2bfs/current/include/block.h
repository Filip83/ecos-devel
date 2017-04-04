#ifndef __BLOCK__
#define __BLOCK__

typedef unsigned int   block_id_t;
typedef unsigned short block_address_t;
typedef unsigned int   block_len_t;



int block_write_super(f2bfs_super_block *super);

int block_write(f2bfs_super_block *super, block_id_t block, block_address_t offset,
					 block_len_t len, const void *data, cyg_uint32 *writed);
int block_rewrite(block_id_t block, block_address_t offset, block_len_t len, 
					 const void *data);
int block_read(f2bfs_super_block *super, block_id_t block, block_address_t offset, 
					 block_len_t len, void *data, cyg_uint32 *readed);
int block_errase(f2bfs_super_block *super,block_id_t block);


int block_alloc(f2bfs_super_block *super, block_id_t *new_block);
int block_alloc_reserved(f2bfs_super_block *super, block_id_t *new_block);
int block_free(f2bfs_super_block *super, block_id_t block_id);
int block_write_bitmap(f2bfs_super_block *super);
int block_read_bitmap(f2bfs_super_block *super);

int block_set_used(f2bfs_super_block *super, block_id_t block, cyg_bool save);
int block_set_free(f2bfs_super_block *super, block_id_t block, cyg_bool save);

int block_get_status(f2bfs_super_block *super, block_id_t block);

#endif