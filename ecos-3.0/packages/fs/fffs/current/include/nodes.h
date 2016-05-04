/*
 * nodes.h
 *
 * Created: 25.9.2012 15:47:25
 *  Author: Filip
 */ 


#ifndef NODES_H_
#define NODES_H_

//#include "cyg_type.h"
#include <cyg/io/flash.h>
#include <pkgconf/fs_fffs.h>

#define BUF_SIZE           CYGNUM_FS_FF_TEST_BUFFER_SIZE 

#define NODE_TYPE_START		0x46464653
#define NODE_TYPE_RECORD	0x5245434e
#define NODE_TYPE_SYSTEM    0x5359534e
#define NODE_TYPE_NEW       0x4e45574e
#define NODE_TYPE_DELTED    0
#define NODE_TYPE_END		-1
#define NODE_TYPE_FREE		-1

#define FFFS_NAMED_NODE_MAX  CYGNUM_FS_FF_NODE_MAX_NAMED_FILES
#define FFFS_MAX_NAME_SIZE   CYGNUM_FS_FF_NODE_MAX_FILE_NAME_SIZE


#define FFFS_ERR_OK 		 0
#define FFFS_ERR_DETECT		-1
#define FFFS_ERR_ADDR		-2
#define FFFS_ERR_START_NODE -3
#define FFFS_ERR_END_NODE   -4
#define FFFS_ERR_WRITE      -5
#define FFFS_ERR_NAMED_NOT_FOUND      -6
#define FFFS_ERR_NAMED_LIMIT      -7
#define FFFS_ERR_RAM_LIMIT      -8
#define FFFS_ERR_NEED_CHECK      -9
#define FFFS_ERR_CANT_WRITE     -10
#define FFFS_ERR_FULL           -11
#define FFFS_ERR_BAD_NODE_ID    -12
#define FFFS_ERR_NO_NAMED_NODE  -13
#define FFFS_ERR_CORUPTED       -14
#define FFFS_ERR_CRC            -15
#define FFFS_ERR_NEAD_FORMAT    -16

#define FFFS_MODE_APPEDN        0x20
#define FFFS_MODE_WRITE         0x10
#define FFFS_MODE_SYSTEM        4
#define FFFS_MODE_NAMED         2
#define FFFS_MODE_HIDEN         1
#define FFFS_MODE_RECORD        0

#define FFFS_ENABLE_REWRITE		1
#define FFFS_ENABLE_APPEND      1


#define FFFS_CPY_BUFF_LIMIT    CYGNUM_FS_FF_CPY_BUFFER_SIZE
#define FFFS_FLASH_CHIPS       1


typedef struct s_node
{
	cyg_uint32 node_type;
	cyg_uint32 next_node;
	cyg_uint32 prev_node;
	cyg_uint32 data_addr;
	cyg_uint32 node_length;
	cyg_uint32 node_id;
	time_t     create_time;
	cyg_uint16 mode;
	cyg_uint16 crc16;
}t_node;

typedef struct s_nmed_node
{
	t_node node;
	char   name[FFFS_MAX_NAME_SIZE];
}t_named_node;

typedef enum
{
	errase_doen = 0, errase_moving_files, errase_create_start, errase_errasing
}t_fffs_fsm;

typedef struct s_fsm
{
	t_fffs_fsm       flash_fsm;
	cyg_uint32       fsm_data1;
	cyg_uint32       fsm_data2;
}t_fsm;

#define SEARCH_FORWARD	true
#define SEARCH_BACKWARD false

typedef struct s_flash_info
{
	cyg_uint32    start_node_addr;
	cyg_uint32    end_node_addr;
	cyg_uint32    num_files;
	cyg_uint32    num_numbered;
	cyg_uint32    used_space;	
	cyg_uint32    system_space;
	cyg_uint32    progres;
	cyg_uint32    progres_tasks;
	t_node        cur_node;
	cyg_uint32    num_named_nodes;
	t_named_node  named_nodes[FFFS_NAMED_NODE_MAX];
	cyg_flash_info_t flash;
	cyg_uint8         n_flash_chips;
	cyg_uint32        flash_size;
	t_fsm             fsm;
	cyg_uint32        last_error;
#ifdef CYGPKG_KERNEL
	cyg_mutex_t       write_mutex;
#endif
	cyg_bool          serch_dir;
}t_flash_info;

//typedef bool cyg_bool;

typedef struct s_node_info
{
	t_node     node;
	cyg_bool   name_node;
	cyg_uint32 named_id;
	cyg_bool   modified;
	cyg_uint32 file_cur_pos;	
}t_node_info;

void mexit(int c);


cyg_uint32 fffs_init  (t_flash_info *info);
cyg_uint32 fffs_format(t_flash_info *info, cyg_bool errase);
cyg_uint32 fffs_erase (t_flash_info *info, cyg_bool complete);

cyg_uint32 fffs_open_named_node  (const char *file_name, t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_create_named_node(const char *file_name, t_flash_info *info, t_node_info *nfile, cyg_uint16 mode);

cyg_uint32 fffs_get_name(t_flash_info *info,t_node_info *nfile, char *name);

cyg_uint32 fffs_set_current_node(t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_get_current_node(t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_get_next_node   (t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_get_prev_node   (t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_get_first_node  (t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_get_last_node   (t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_find_node       (t_flash_info *info, t_node_info *nfile, cyg_uint32 node_id);

cyg_uint32 fffs_open_node_append(t_flash_info *info, t_node_info *nfile);
cyg_uint32 fffs_open_named_node_apped(const char *file_name, t_flash_info *info, t_node_info *nfile);

cyg_uint32 fffs_create_node(t_flash_info *info, t_node_info *nfile, cyg_uint16 mode);
cyg_uint32 fffs_open_node  (t_flash_info *info, t_node_info *nfile, cyg_uint32 node_id);
cyg_uint32 fffs_read_node  (t_flash_info *info, t_node_info *nfile, void *data, cyg_uint32 length);
cyg_uint32 fffs_write_node (t_flash_info *info, t_node_info *nfile, const void *data, cyg_uint32 length);
cyg_uint32 fffs_close_node (t_flash_info *info, t_node_info *nfile);

cyg_uint32 fffs_delete(t_flash_info *info, t_node_info *nfile);

cyg_uint32 fffs_test();
cyg_uint32 fffs_check(t_flash_info *info);
cyg_uint32 fffs_erasex();

const char * fffs_print_error(cyg_uint32 error);

cyg_uint32 fffs_test(t_flash_info *info,cyg_uint32 test_id);

#endif /* NODES_H_ */
