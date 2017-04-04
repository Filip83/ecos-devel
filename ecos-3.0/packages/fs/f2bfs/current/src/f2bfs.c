#include <pkgconf/system.h>
#include <pkgconf/hal.h>

#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>

#include <stdlib.h>
#include <string.h>

#include <cyg/fileio/fileio.h>
#include <cyg/infra/diag.h>
#include <cyg/io/flash.h>

#ifndef FSDEVEL
#include <cyg/fs/f2bfsconfig.h>
#include <cyg/fs/block.h>
#include <cyg/fs/node.h>
#else
#include <f2bfsconfig.h>
#include <block.h>
#include <node.h>
#endif

//#include "fsck.h"

/** F2BFS (FRAM FLASH Block File System)
/* The file system support one level of directory structure e.g. 
/* only root directory. The files stored on disk can be listed.
/* The write operation allows seek operation but it has meaning to
/* only if data are stored in reserved FRAM region.
/* \todo Skontrolovat alokace a dealokace.
/* \todo Skontrolovat dir search. Je to udelano i pro vice urovni
*/

extern struct cyg_flash_dev CYGNUM_F2BFS_FRAM_DEV_NAME;
extern struct cyg_flash_dev CYGNUM_F2BFS_FLASH_DEV_NAME;


#define F2BFS_DIAG_LEVEL 0


//==========================================================================
// Forward definitions

// Filesystem operations
static int f2bfs_mount    ( cyg_fstab_entry *fste, cyg_mtab_entry *mte );
static int f2bfs_umount   ( cyg_mtab_entry *mte );
static int f2bfs_open     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int mode,  cyg_file *fte );
static int f2bfs_unlink   ( cyg_mtab_entry *mte, cyg_dir dir, const char *name );
static int f2bfs_mkdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name );
static int f2bfs_rmdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name );
static int f2bfs_rename   ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2 );
static int f2bfs_link     ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2, int type );
static int f2bfs_opendir  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_file *fte );
static int f2bfs_chdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_dir *dir_out );
static int f2bfs_stat     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
struct stat *buf);
static int f2bfs_getinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len );
static int f2bfs_setinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len );

// File operations
static int f2bfs_fo_read      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int f2bfs_fo_write     (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int f2bfs_fo_lseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence );
static int f2bfs_fo_ioctl     (struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
CYG_ADDRWORD data);
static int f2bfs_fo_fsync     (struct CYG_FILE_TAG *fp, int mode );
static int f2bfs_fo_close     (struct CYG_FILE_TAG *fp);
static int f2bfs_fo_fstat     (struct CYG_FILE_TAG *fp, struct stat *buf );
static int f2bfs_fo_getinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len );
static int f2bfs_fo_setinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len );

// Directory operations
static int f2bfs_fo_dirread      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int f2bfs_fo_dirlseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence );

static int f2bfs_check();
static int f2bfs_fo_check_repair_size(const char *name, size_t block_len);


//==========================================================================
// Filesystem table entries


// Fstab entry.
// This defines the entry in the filesystem table.
// For simplicity we use _FILESYSTEM synchronization for all accesses since
// we should never block in any filesystem operations.

FSTAB_ENTRY( f2bfs_fste, "f2bfs", 0,
CYG_SYNCMODE_FILE_FILESYSTEM|CYG_SYNCMODE_IO_FILESYSTEM,
	f2bfs_mount,
	f2bfs_umount,
	f2bfs_open,
	f2bfs_unlink,
	f2bfs_mkdir,
	f2bfs_rmdir,
	f2bfs_rename,
	f2bfs_link,
	f2bfs_opendir,
	f2bfs_chdir,
	f2bfs_stat,
	f2bfs_getinfo,
	f2bfs_setinfo);


// File operations.
// This set of file operations are used for normal open files.

static cyg_fileops f2bfs_fileops =
{
	f2bfs_fo_read,
	f2bfs_fo_write,
	f2bfs_fo_lseek,
	f2bfs_fo_ioctl,
	cyg_fileio_seltrue,
	f2bfs_fo_fsync,
	f2bfs_fo_close,
	f2bfs_fo_fstat,
	f2bfs_fo_getinfo,
	f2bfs_fo_setinfo
};


// Directory file operations.
// This set of operations are used for open directories. Most entries
// point to error-returning stub functions. Only the read, lseek and
// close entries are functional.

static cyg_fileops f2bfs_dirops =
{
	f2bfs_fo_dirread,
	(cyg_fileop_write *)cyg_fileio_enosys,
	f2bfs_fo_dirlseek,
	(cyg_fileop_ioctl *)cyg_fileio_enosys,
	cyg_fileio_seltrue,
	(cyg_fileop_fsync *)cyg_fileio_enosys,
	f2bfs_fo_close,
	(cyg_fileop_fstat *)cyg_fileio_enosys,
	(cyg_fileop_getinfo *)cyg_fileio_enosys,
	(cyg_fileop_setinfo *)cyg_fileio_enosys
};

//==========================================================================
// Data typedefs
// Some forward typedefs for the main data structures.

/** Static structure to hold disk super block structure. */
f2bfs_super_block root_super  BLOCK_FS_STATIC_DTATA_ATRIBUTES;
/** Static structure to hold root inode structure data. */
f2bfs_inode	      root_inode  BLOCK_FS_STATIC_DTATA_ATRIBUTES;

#if 0
extern cyg_uint8 inode_bitmap[F2BFS_NODES_PER_BLOCK/8 + 1];
extern cyg_uint8 block_bitmap[F2BFS_NUM_BOLOCKS/8];
#endif


//==========================================================================
// Forward defs

static int del_direntry( f2bfs_inode *dir, const char *name, int namelen );


//==========================================================================
// This seems to be the only string function referenced. Define as static
// here to avoid having to load the string library

static bool match( const char *a, const char *b, int len )
{
	for ( ; len > 0 && *a && *b && *a == *b ; a++, b++, len-- )
	;
	return ( len == 0 && *a == 0 );
}

//==========================================================================
// Ref count and nlink management

/** dec_refcnt function
/*
/* Decrement the reference count on a node. If this makes the ref count
/* zero, and the number of links is either zero for a file or one for
/* a node, then this node is detached from the directory and its used
/* space if freed.
/* \param node - point to node.
/* \return IO Error
*/
static int dec_refcnt( f2bfs_inode *node )
{
	int err = ENOERR;
	node->i_links_count--;

	if( node->i_links_count == 0 &&
	   (S_ISREG(node->i_mode)) )
	{
		// free node space
		err = node_free_space(&root_super, node );
		if(err != ENOERR)
			return err;
		
		// invalidate node
		err = node_invalidate(&root_super,node);
	}

	return err;
}


// read_direntry()
/** read_direntry function
/*
/* This function read directory entry from specified position form
/* specified node \see dir, defined by \see pos.
/* Directory entry crc is checked in this function as well.
/* \param super - pointer to super block structure.
/* \param dir - pointer to directory node.
/* \param pos - directory record position.
/* \param dir_entry - pointer to directory entry structure to read to.
/* \return IO Error.
*/
int f2bfs_read_direntry(f2bfs_super_block *super, f2bfs_inode *dir,
								cyg_uint32 pos, f2bfs_dir_entry * dir_entry )
{
	int err;
	cyg_uint32 rw_len;
	cyg_uint32 crc;
	rw_len = sizeof(f2bfs_dir_entry) ;
	
	err    = node_read_data(super,dir,pos,dir_entry, &rw_len);
	if(err != ENOERR)
		return err;
	
	if( rw_len != sizeof(f2bfs_dir_entry))
		return EIO;
	
	crc = cyg_posix_crc32((unsigned char*)dir_entry,
			sizeof(f2bfs_dir_entry) - sizeof(cyg_uint32));
	
	if(dir_entry->crc32 != crc)
	{
		#if F2BFS_DIAG_LEVEL > 0	
		diag_printf("read dir entry crc error\n");
		#endif
		return ECRC32;
	}
	return err;
}

/** f2bfs_write_direntry function
/*
/* This function write directory entry data to specified node \see dir and
/* specified position \see pos. Ne crc is calculated for directory entry.
/* \param super - super block structure pointer.
/* \param dir - pointer to directory node.
/* \param pos - position in directory node to write to.
/* \param dir_entry - pointer to directory entry structure to write.
/* \return IO Error.
*/
static int f2bfs_write_direntry(f2bfs_super_block *super, f2bfs_inode *dir,
				cyg_uint32 pos, f2bfs_dir_entry * dir_entry )
{
	int err;
	cyg_uint32 len;
	
	len  = sizeof(f2bfs_dir_entry);
	dir_entry->crc32 = 0;
	cyg_uint32 crc = cyg_posix_crc32((unsigned char*)dir_entry,sizeof(f2bfs_dir_entry) - sizeof(cyg_uint32));
	dir_entry->crc32 = crc;
	
	err = node_write_data(super,dir,pos,dir_entry,&len);
	if( err != ENOERR )
		return err;
	
	if(len != sizeof(f2bfs_dir_entry))
		return EIO;
	return err;
}

// Najde prazdnou dir entry. U prazdne dir entry
// je vynulovana položka rec_len
/** find_free_dirrec function
/*
/* This function find free directory entry in directory node.
/* Free directory entry has rec_len item set to zero.
/* \note Directory entries are store in FRAM region and
/* data override is no concern.
/* \param dir - pointer to directory node.
/* \param offset - variable to return position of directory entry in byte offset.
/* \return IO Error, if no free directory entry was found ENOENT is returned.
*/
static int find_free_dirrec(f2bfs_inode *dir, int *offset)
{
	off_t pos = 0;
	f2bfs_dir_entry d;
	int err;
	cyg_uint32	len;
	
	while(pos < dir->i_size)
	{
		len = sizeof(f2bfs_dir_entry);
		err = f2bfs_read_direntry(&root_super,dir,pos,&d);
		if(err != ENOERR)
			return err;		

		if(d.rec_len == 0)
		{
			*offset = d.offset;
			return ENOERR;
		}
		
		pos += len;
	}
	return ENOENT;
}


//==========================================================================
// Directory operations


// add_direntry()
// Add an entry to a directory. This is added as a chain of entry
// fragments until the name is exhausted.

/** add_direntry function
/*
/* Add an entry to a directory. \note Only root directory is suported.
/* \param dir - diectory node.
/* \param name - name to add.
/* \param namelen - length of name.
/* \param node - node to refernce.
/* \return IO Error.
*/
static int add_direntry( f2bfs_inode *dir,       // dir to add to
	const char *name,			// name to add
	int namelen,				// length of name
	f2bfs_inode *node        // node to reference
)
{
	f2bfs_dir_entry	new_dir_entry;
	int				err;
	int				new_pos;
		
	if(namelen >= CYGNUM_F2BFS_DIRENT_NAME_SIZE)
		return ENAMETOOLONG;

	new_dir_entry.inode		   = node->i_node_id;
	new_dir_entry.rec_len      = sizeof(f2bfs_dir_entry);
	new_dir_entry.name_len     = namelen;
	new_dir_entry.offset       = dir->i_size;
	
	memcpy( new_dir_entry.name, name, namelen ); 
	new_dir_entry.name[namelen] = 0;
	// Check if we can use deleted directory entry
	err = find_free_dirrec(dir,&new_pos);
	#if F2BFS_DIAG_LEVEL > 0
	diag_printf("New dir entrty size: %d, dir offset: %d\n",dir->i_size, new_pos);
	#endif
	if(err == ENOENT)
	{
		// If no free directory entry was found add new one to the end of list
		// \TODO: maximum number of entries is not limited
		err = f2bfs_write_direntry(&root_super,dir,dir->i_size,&new_dir_entry);
		if( err != ENOERR )
			return err;
	}
	else if(err == ENOERR)
	{
		// If unused directory entry is found use it
		new_dir_entry.offset       = new_pos;
		err = f2bfs_write_direntry(&root_super,dir,new_pos,&new_dir_entry);
		if( err != ENOERR )
			return err;
	}
	else
		return err;	
	
	// Update directory times
	dir->i_mtime =
	dir->i_ctime = cyg_timestamp();
	
	// Update directory node links count
	dir->i_links_count++;
	
	err = node_validate(&root_super,dir);
	if(err != ENOERR)
		return err;
	
	// Count the new link
	node->i_links_count++;	
	return node_validate(&root_super,node);
}


/** find_direntry function
/*
/* Find a directory entry for the name and return a pointer to the first
/* entry fragment.
/* \param super - pointer to super block structure.
/* \param dir - pointer to directory inode structure.
/* \param name - name of the file to search.
/* \param namelen - length of the file name.
/* \param d - pointer to directory entry structure. This structure is used to read
/* directory entries from disk.
/* \return - pointer to directory entry structure if file sucesfully found,
/* NULL otherwise.
*/
static f2bfs_dir_entry * find_direntry(f2bfs_super_block *super, f2bfs_inode *dir,
							const char *name, int namelen, f2bfs_dir_entry *d)
{
	cyg_uint32 rw_len;
    int err;
	int pos = 0;
    // Loop over all the entries until a match is found or we run out
    // of data.
	#if F2BFS_DIAG_LEVEL > 0
	diag_printf("Dir size: %d\n",dir->i_size);
	#endif
    while( pos < dir->i_size )
    {
		rw_len = sizeof(f2bfs_dir_entry) ;
			
		err = f2bfs_read_direntry(super,dir,pos,d);
		if(err != ENOERR)
			return err;
		
		#if F2BFS_DIAG_LEVEL > 0	
		diag_printf("Dir: %d, %d, %d %s\n",(int)d->inode,(int)d->name_len,d->rec_len,d->name);
		diag_printf("Read pos: %d, %d\n",pos,d->offset);
		#endif

		// Otherwise move on to next entry in chain
		pos += rw_len;
		
		// Free unused dir entry
		if(d->rec_len == 0)
			continue;
	    // Is this the directory entry we're looking for?
	    if ( match( d->name, name, namelen )/* && namelen == d->name_len*/ )
			return d;
    }

    return (f2bfs_dir_entry *)NULL;	
}


/** del_direntry()
/*
/* Delete a named directory entry. \note Only root directory entry is supported.
/* No while directory chain need to be deleted.
/* \param dir - pointer to directory node.
/* \param name - name of the directory entry to delete.
/* \param namelen - length of the name.
/* \return - IO Error.
*/

static int del_direntry( f2bfs_inode *dir, const char *name, int namelen )
{
	f2bfs_dir_entry dd;
	cyg_uint32	len;
	f2bfs_dir_entry *d;
	int err;
	
	d = find_direntry(&root_super, dir, name, namelen, &dd);
	
	if( d == NULL )
		return ENOENT;

	d->rec_len = 0 ;//deleted
	
	err = f2bfs_write_direntry(&root_super,dir,d->offset,d);
  
    // \todo Tento kod je zbytecny otestovat \see del_direntry
	/*len = sizeof(f2bfs_dir_entry);
	err = node_write_data(&root_super,dir,d->offset,d,&len);
	if( err != ENOERR )
		return ENOENT;*/
	
	// decrement directory node number of links
	// and update its content on disk
	dir->i_links_count--;	
	err = node_write(&root_super,dir);
	return err;
}

//==========================================================================
// Directory search


/** init_dirsearch function
/*
/* Initialize a dirsearch object to start a search.
/* \param ds - pointer to \see f2bfs_dirsearch structure to
/* initialize.
/* \param dir - directory node in to search.
/* \param name - name of file to search.
*/

static void init_dirsearch( f2bfs_dirsearch *ds,
	f2bfs_inode *dir,
	const char *name)
{
	ds->dir      = dir;
	ds->path     = name;
	ds->node     = NULL;
	ds->name     = name;
	ds->namelen  = 0;
	ds->last     = false;
}


/** find_entry function
/*
/* Search a single directory for the next name in a path and update the
/* dirsearch object appropriately.
/* \param ds - pointer to directory search structure.
/* \return IO Error.
*/
static int find_entry( f2bfs_dirsearch *ds )
{
	f2bfs_inode *dir   = ds->dir;
	char       *name   = ds->path;
	char       *n      = name;
	int        namelen = 0;
	//off_t pos = 0;
	f2bfs_dir_entry dd;
	f2bfs_dir_entry *d;
	
	// check that we really have a directory
	if( !S_ISDIR(dir->i_mode) )
		return ENOTDIR;

	// Isolate the next element of the path name.
	while( *n != '\0' && *n != '/' )
		n++, namelen++;

	// Check if this is the last path element.
	while( *n == '/') n++;
	if( *n == '\0' )
		ds->last = true;

	// update name in dirsearch object
	ds->name    = name;
	ds->namelen = namelen;
	
	// Here we have the name and its length set up.
	// Search the directory for a matching entry

	d = find_direntry(&root_super, dir, name, namelen,  &dd);

	if( d == NULL )
		return ENOENT;

	// if space for inode is not aloced alloc space
	// for node and read it
	if(ds->node == NULL)
	{
		ds->node = node_open(&root_super,d->inode);
		if(ds->node == NULL)
			return EIO;
		return ENOERR;
	}
	
	// pass back the node we have found
	return node_read(&root_super,ds->node,d->inode);
}


/** f2bfs_find function
/*
/* Main interface to directory search code. This is used in all file
/* level operations to locate the object named by the pathname.
/* \param d - pointer to directory search structure. 
/* \return IO Error.
*/
static int f2bfs_find( f2bfs_dirsearch *d )
{
	int err;
	// Short circuit empty paths
	if( *(d->path) == '\0' )
	{
		d->node = (f2bfs_inode*)malloc(sizeof(f2bfs_inode));
		CYG_ASSERT(d->node,"Cannot alloc new node\n");
		*d->node = root_inode;
		return ENOERR;
	}

	// iterate down directory tree until we find the object
	// we want. We do not really iterate because single level 
	// directory supported.
	for(;;)
	{
		err = find_entry( d );

		if( err != ENOERR )
			return err;

		if( d->last )
			return ENOERR;

		// Update dirsearch object to search next directory.
		d->dir   = d->node;
		d->path += d->namelen;
		while( *(d->path) == '/' ) d->path++; // skip dirname separators
	}
}

//==========================================================================
/** f2bfs_pathconf function
/* Pathconf support
/* This function provides support for pathconf() and fpathconf().
/* \param node - inode structure pointer.
/* \param info - path conf info structure.
/* \return IO Error.
*/
static int f2bfs_pathconf( f2bfs_inode *node, struct cyg_pathconf_info *info )
{
	int err = ENOERR;
	
	switch( info->name )
	{
		case _PC_LINK_MAX:
			info->value = LINK_MAX;
			break;
		
		case _PC_MAX_CANON:
			info->value = -1;       // not supported
			err = EINVAL;
			break;
		
		case _PC_MAX_INPUT:
			info->value = -1;       // not supported
			err = EINVAL;
			break;
		
		case _PC_NAME_MAX:
			info->value = NAME_MAX;
			break;
		
		case _PC_PATH_MAX:
			info->value = PATH_MAX;
			break;

		case _PC_PIPE_BUF:
			info->value = -1;       // not supported
			err = EINVAL;
			break;

		case _PC_ASYNC_IO:
			info->value = -1;       // not supported
			err = EINVAL;
			break;
		
		case _PC_CHOWN_RESTRICTED:
			info->value = -1;       // not supported
			err = EINVAL;
			break;
		
		case _PC_NO_TRUNC:
			info->value = 0;
			break;
		
		case _PC_PRIO_IO:
			info->value = 0;
			break;
		
		case _PC_SYNC_IO:
			info->value = 0;
			break;
		
		case _PC_VDISABLE:
			info->value = -1;       // not supported
			err = EINVAL;
			break;
		
		default:
			err = EINVAL;
			break;
	}

	return err;
}

/** f2bfs_format functin
/*
/* This function format disk. It create F2BFS file system
/* structures on disk. The file system configuration is 
/* manual and all file system parameter must be set as parameter.
/* \return IO Error.
*/
int f2bfs_format(void)
{
	int i;
	cyg_uint32		wr_len;
	
	root_super.s_blocks_count		= F2BFS_NUM_BOLOCKS;
	root_super.s_checkinterval		= 0;
	root_super.s_first_data_block	= 8;
	root_super.s_first_data_block_reserved	= 4;
	root_super.s_free_blocks_count	= F2BFS_NUM_BOLOCKS;
	root_super.s_free_blocks_count_data = F2BFS_NUM_BOLOCKS - root_super.s_first_data_block;
	root_super.s_free_inodes_count  = F2BFS_NODES_PER_BLOCK;
	root_super.s_inodes_count		= F2BFS_NODES_PER_BLOCK;

	root_super.s_r_blocks_count		= 3;//F2BFS_NUM_FRAM_BLOCKS; 3 bloky pro root dir 1 a 2 pro sys soubory zbytek FRAM FS data
	
	root_super.s_log_block_size     = CYGNUM_F2BFS_LOG_BLOCK_SIZE;
	root_super.s_block_size			= CYGNUM_F2BFS_BLOCK_SIZE;
	root_super.s_magic				= F2BFS_MAGIC;
	root_super.s_max_mnt_count		= -1;
	root_super.s_minor_rev_level	= 1;
	root_super.s_mnt_count			= 0;
	root_super.s_state				= 0;
	root_super.s_wtime				= 0;
	
	root_super.block_goroup_info.bg_block_bitmap = 1;
	root_super.block_goroup_info.bg_inode_bitmap = 2;
	root_super.block_goroup_info.bg_inode_table  = 3;
	root_super.block_goroup_info.bg_start_search = 8;
	root_super.block_goroup_info.bg_used_dirs_count = 0;
	

	memset(inode_bitmap,0xff,sizeof(inode_bitmap));
	memset(block_bitmap,0xff,sizeof(block_bitmap));
	block_set_used(&root_super,0,false); // super block
	block_set_used(&root_super,1,false); // block bitmap
	block_set_used(&root_super,2,false); // node bitmap
	block_set_used(&root_super,3,false); // node table

	if(block_write(&root_super,root_super.block_goroup_info.bg_block_bitmap,
		0,sizeof(block_bitmap),block_bitmap,&wr_len) != ENOERR ||
		wr_len != sizeof(block_bitmap))
	{
		return EIO;
	}
	
	if(block_write(&root_super,root_super.block_goroup_info.bg_inode_bitmap,
		0,sizeof(inode_bitmap),inode_bitmap,&wr_len) != ENOERR ||
		wr_len != sizeof(inode_bitmap))
	{
		return EIO;
	}
	
	// Allocate a node to be the root of this filesystem and initialize it.
	if(block_write_super(&root_super) != ENOERR)
	{
		return EIO;
	}
	
	memset(&root_inode,0,sizeof(f2bfs_inode));
	
	for(i = 0; i < root_super.s_inodes_count; i++)
	{
		root_inode.i_node_id = i;
		node_write(&root_super,&root_inode);
	}
	
	root_inode.i_node_id = 0;
	root_inode.i_atime = 
	root_inode.i_ctime = 
	root_inode.i_mtime = cyg_timestamp();
	root_inode.i_mode  = __stat_mode_DIR|S_IRWXU|S_IRWXG|S_IRWXO;
	root_inode.i_links_count = 1;
	root_inode.i_flags = 0;
	root_inode.i_size  = 0;
	root_inode.i_flags = FILE_FLAG_SYSTEM_FILE;
	
	return node_validate(&root_super,&root_inode);
}

//==========================================================================
// Filesystem operations


/** f2bfs_mount function
/* Process a mount request. This mainly creates a root for the
/* filesystem. In this function eCos FLASH sub system is initialized.
/* If no F2BFS is present on disk it is automatically formated.
*/
static int f2bfs_mount    ( cyg_fstab_entry *fste, cyg_mtab_entry *mte )
{
	//f2bfs_inode *root;
	cyg_uint32	wr_len;
	int err;
	cyg_flashaddr_t err_addr;
	cyg_uint32      crc;
	
	cyg_flash_init(NULL);
	// Allocate a node to be the root of this filesystem and initialize it.
	if(cyg_flash_read(F2BFS_BLOCK_FLASH_OFFSET,&root_super,sizeof(f2bfs_super_block),&err_addr) != CYG_FLASH_ERR_OK)
	{
		return EIO;
	}

	if(root_super.s_magic != F2BFS_MAGIC)
	{
		if((err = f2bfs_format()) != ENOERR)
			return err;
	}
	
	crc = cyg_posix_crc32((unsigned char*)&root_super,sizeof(f2bfs_super_block) - sizeof(cyg_uint32));
	if(crc != root_super.s_crc32)
	{
		err = EIO;
		goto mount_io_error;
	}
	
	root_super.s_mtime = cyg_timestamp();
	root_super.s_mnt_count++;
	
	err = block_write_super(&root_super);
	if(err != ENOERR)
		goto mount_io_error;
		
	if(block_read(&root_super,root_super.block_goroup_info.bg_block_bitmap,
		0,sizeof(block_bitmap),block_bitmap,&wr_len) != ENOERR ||
		wr_len != sizeof(block_bitmap))
	{
		err = EIO;
		goto mount_io_error;
	}
	
	if(block_read(&root_super,root_super.block_goroup_info.bg_inode_bitmap,
		0,sizeof(inode_bitmap),inode_bitmap,&wr_len) != ENOERR ||
		wr_len != sizeof(inode_bitmap))
	{
		err = EIO;
		goto mount_io_error;
	}
	
	//pak asi na\E8\EDst root_inode
	err = node_read(&root_super,&root_inode,0);
	mte->root = (cyg_dir)&root_inode;
mount_io_error:
	//f2bfs_collect();
	
	return err;
}


/** f2bfs_umount function
/* Unmount the filesystem. This will currently only succeed if the
/* filesystem is empty.
*/
static int f2bfs_umount   ( cyg_mtab_entry *mte )
{
    f2bfs_inode *root = (f2bfs_inode *)mte->root;

    // Check for open/inuse root
    if( root->i_refcnt != 0 )
		return EBUSY;

    // Clear root pointer
    mte->root = CYG_DIR_NULL;
    
    // That's all folks.
    return ENOERR;
}


/** f2bfs_open function
/*
/* Open a file for reading or writing.
*/
static int f2bfs_open     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
                              int mode,  cyg_file *file )
{
	f2bfs_dirsearch ds;
	int err              = ENOERR;
	cyg_bool system_file = false;

	init_dirsearch( &ds, (f2bfs_inode *)&root_inode, name );
	
	err = f2bfs_find( &ds );
	
	if(ds.name[0] == '.')
		system_file = true;
		
	if( err == ENOENT )
	{
		if( ds.last && (mode & O_CREAT) )
		{
			// No node there, if the O_CREAT bit is set then we must
			// create a new one. The dir and name fields of the dirsearch
			// object will have been updated so we know where to put it.

			ds.node = node_create(&root_super, __stat_mode_REG|S_IRWXU|S_IRWXG|S_IRWXO);
			if( ds.node == NULL )
				return ENOSPC;
		
			ds.node->i_flags = (system_file) ? FILE_FLAG_SYSTEM_FILE : FILE_FLAG_FILE;
			err = add_direntry( ds.dir, ds.name, ds.namelen, ds.node );

			if( err != ENOERR )
			{
				node_close(&root_super, ds.node );
				return err;
			}
			
			err = ENOERR;
		}
		else
			return EINVAL;
	}
	else if(mode & O_EXCL)
		return EINVAL;
	else if( err == ENOERR && (mode & O_TRUNC ))
	{
		err = node_free_space(&root_super,ds.node);
		if(err != ENOERR)
		{
			node_close(&root_super, ds.node );
			return err;
		}
		
		err = node_validate(&root_super,ds.node);
		if(err != ENOERR)
			return err;
	}
	else if(err != ENOERR)
		return err;

	// Check that we actually have a file here
	if( S_ISDIR(ds.node->i_mode) ) 
		return EISDIR;

	ds.node->i_refcnt++; // Count successful open   
	
	// Initialize the file object
	file->f_flag        |= mode & CYG_FILE_MODE_MASK;
	file->f_type        = CYG_FILE_TYPE_FILE;
	file->f_ops         = &f2bfs_fileops;
	file->f_offset      = (mode&O_APPEND) ? ds.node->i_size : 0;
	file->f_data        = (CYG_ADDRWORD)ds.node;
	file->f_xops        = 0;

	return ENOERR;
}


/** f2bfs_unlink function
/*
/* Remove a file link from its directory.
*/
static int f2bfs_unlink   ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
	f2bfs_dirsearch ds;
	int err;

	init_dirsearch( &ds, (f2bfs_inode *)&root_inode, name );
	
	err = f2bfs_find( &ds );

	if( err != ENOERR )
	{
		node_close(&root_super,ds.node);
		return err;
	}

	// Cannot unlink directories, use rmdir() instead
	if( S_ISDIR(ds.node->i_mode) )
	{
		node_close(&root_super,ds.node);
		return EPERM;
	}
	
	#if F2BFS_DIAG_LEVEL > 0
	diag_printf("Ref cnt: %d\n",ds.node->i_refcnt);
	#endif
	// File is opened in another thread
	if(ds.node->i_refcnt != 0)
	{
		node_close(&root_super,ds.node);
		return EBUSY;
	}

	// Delete it from its directory
	err = del_direntry( ds.dir, ds.name, ds.namelen );
	if(err == ENOERR)
		err = dec_refcnt( ds.node );
	node_close(&root_super,ds.node);
	return err;
}


/** f2bfs_mkdir function
/*
/* Create a new directory. Not suported only single level directory is suppored.
*/
static int f2bfs_mkdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
	return EINVAL;
}


/** f2bfs_rmdir function
/*
/* Remove a directory. Not supporeted. 
*/
static int f2bfs_rmdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
	return EINVAL;
}


/** f2bfs_rename function
/*
/* Rename a file/dir. If destination file name exist EEXIST is returned.
/* \todo skontrolovat s puvodni verzi.
*/
static int f2bfs_rename   ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
                              cyg_dir dir2, const char *name2 )
{
	f2bfs_dirsearch ds1, ds2;
	int err;

	init_dirsearch( &ds1, (f2bfs_inode *)dir1, name1 );
	
	err = f2bfs_find( &ds1 );

	if( err != ENOERR ) 
		return err;

	init_dirsearch( &ds2, (f2bfs_inode *)dir2, name2 );
	
	err = f2bfs_find( &ds2 );

	// Allow through renames to non-existent objects.
	if( ds2.last && err == ENOENT )
	{
		node_close(&root_super,ds2.node);
		ds2.node = NULL, err = ENOERR;
	}
	
	if( err != ENOERR ) 
	{
		node_close(&root_super,ds1.node);
		node_close(&root_super,ds2.node);
		return err;
	}

	// Null rename, just return
	if( ds1.node == ds2.node )
	{
		node_close(&root_super,ds1.node);
		node_close(&root_super,ds2.node);
		return ENOERR;
	}

	// First deal with any entry that is at the destination
	if( ds2.node )
	{
		node_close(&root_super,ds1.node);
		node_close(&root_super,ds2.node);
		return EEXIST;
	}

	// Now we know that there is no clashing node at the destination,
	// make a new direntry at the destination and delete the old entry
	// at the source.

	err = add_direntry( ds2.dir, ds2.name, ds2.namelen, ds1.node );

	if( err == ENOERR )
		err = del_direntry( ds1.dir, ds1.name, ds1.namelen );

	// Update directory times
	/*if( err == ENOERR )
		ds1.dir->i_ctime =
		ds1.dir->i_mtime =
		ds2.dir->i_ctime =
		ds2.dir->i_mtime = cyg_timestamp();*/
	
	node_close(&root_super,ds1.node);
	node_close(&root_super,ds2.node);
	return err;
}


/** f2bfs_link function
/*
/* Make a new directory entry for a file.
/* \todo Divne skontrolovat s puvodni verzi
*/
static int f2bfs_link     ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
                          cyg_dir dir2, const char *name2, int type )
{
#if 1
	return ENOSUPP;
#else
	f2bfs_dirsearch ds1, ds2;
	int err;

	// Only do hard links for now in this filesystem
	if( type != CYG_FSLINK_HARD )
		return EINVAL;
	
	init_dirsearch( &ds1, (f2bfs_inode *)dir1, name1 );
	
	err = f2bfs_find( &ds1 );

	if( err != ENOERR ) 
		return err;

	init_dirsearch( &ds2, (f2bfs_inode *)dir2, name2 );
	
	err = f2bfs_find( &ds2 );

	// Don't allow links to existing objects
	if( err == ENOERR ) 
		return EEXIST;
	
	// Allow through links to non-existing terminal objects
	if( ds2.last && err == ENOENT )
		/*ds2.node = NULL,*/ err = ENOERR;

	if( err != ENOERR ) 
		return err;
	
	// Now we know that there is no existing node at the destination,
	// make a new direntry at the destination.

	err = add_direntry( ds2.dir, ds2.name, ds2.namelen, ds1.node );

	if( err == ENOERR )
		ds1.node->i_ctime =
		ds2.node->i_ctime =
		ds2.node->i_mtime = cyg_timestamp();
	
	return err;
#endif
}


/** f2bfs_opendir function
/*
/* Open a directory for reading.
*/
static int f2bfs_opendir  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
								cyg_file *file )
{
	f2bfs_dirsearch ds;
	int err;

	init_dirsearch( &ds, (f2bfs_inode *)dir, name );
	
	err = f2bfs_find( &ds );

	if( err != ENOERR ) 
	{
		node_close(&root_super,ds.node);
		return err;
	}

	// check it is really a directory.
	if( !S_ISDIR(ds.node->i_mode) ) 
	{
		node_close(&root_super,ds.node);
		return ENOTDIR;
	}
	
	ds.node->i_refcnt++;       // Count successful open
	
	// Initialize the file object, setting the f_ops field to a
	// special set of file ops.
	
	file->f_type        = CYG_FILE_TYPE_FILE;
	file->f_ops         = &f2bfs_dirops;
	file->f_offset      = 0;
	file->f_data        = (CYG_ADDRWORD)ds.node;
	file->f_xops        = 0;
	
	return ENOERR;
}


/** f2bfs_chdir function
/*
/* Change directory support.
/* \todo Only singngle level directory supported. This function need to be
/* updated if this support will be added.
*/
static int f2bfs_chdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
                              cyg_dir *dir_out )
{
	if( dir_out != NULL )
	{
		// This is a request to get a new directory pointer in
		f2bfs_dirsearch ds;
		int err;
		
		init_dirsearch( &ds, (f2bfs_inode *)dir, name );
		
		err = f2bfs_find( &ds );

		if( err != ENOERR ) 
		{
			node_close(&root_super,ds.node);
			return err;
		}

		// check it is a directory
		if( !S_ISDIR(ds.node->i_mode) )
		{
			return ENOTDIR;
			node_close(&root_super,ds.node);
		}
		
		// Increment ref count to keep this directory in existent
		// while it is the current cdir.
		ds.node->i_refcnt++;

		// Pass it out
		*dir_out = (cyg_dir)ds.node;
	}
	else
	{
		// If no output dir is required, this means that the mte and
		// dir arguments are the current cdir setting and we should
		// forget this fact.

		f2bfs_inode *node = (f2bfs_inode *)dir;
		
		node->i_refcnt--;
		if(node->i_refcnt > 0)
			return ENOERR;
		// Just decrement directory reference count.
		//dec_refcnt se snizuje a nechceme odstarnit
		node_close(&root_super,node);
	}
	
	return ENOERR;
}


/** f2bfs_stat function
/*
/* Get struct stat info for named object.
*/
static int f2bfs_stat     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
struct stat *buf)
{
	f2bfs_dirsearch ds;
	int err;

	init_dirsearch( &ds,&root_inode, name );
	
	err = f2bfs_find( &ds );

	if( err != ENOERR ) 
		return err;

	// Fill in the status
	buf->st_mode        = ds.node->i_mode;
	buf->st_ino         = (ino_t)ds.node->i_node_id;
	buf->st_dev         = 0;
	buf->st_nlink       = ds.node->i_links_count;//ds.dir->i_links_count;
	buf->st_uid         = 0;
	buf->st_gid         = 0;
	buf->st_size        = ds.node->i_size;
	buf->st_atime       = ds.node->i_atime;
	buf->st_mtime       = ds.node->i_mtime;
	buf->st_ctime       = ds.node->i_ctime;
	
	return node_close(&root_super,ds.node);
}


/** f2bfs_getinfo function
/*
/* Getinfo. Currently only support pathconf() and filesystem usage.
*/
static int f2bfs_getinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
                              int key, void *buf, int len )
{
	f2bfs_dirsearch ds;
	int err;

	init_dirsearch( &ds, (f2bfs_inode *)dir, name );
	
	err = f2bfs_find( &ds );

	if( err != ENOERR ) 
	{
		node_close(&root_super,ds.node);
		return err;
	}

	switch( key )
	{
		case FS_INFO_CONF:
			err = f2bfs_pathconf( ds.node, (struct cyg_pathconf_info *)buf );
			break;
#ifdef CYGSEM_FILEIO_INFO_DISK_USAGE
		case FS_INFO_DISK_USAGE: {
			struct cyg_fs_disk_usage *usage = (struct cyg_fs_disk_usage *) buf;
			usage->block_size	= root_super.s_block_size;
			usage->free_blocks	= root_super.s_free_blocks_count;
			usage->total_blocks = root_super.s_blocks_count;
			break;
		}
		case 8: {
			struct cyg_fs_disk_usage *usage = (struct cyg_fs_disk_usage *) buf;
			usage->block_size	= root_super.s_block_size;
			usage->free_blocks	= root_super.s_free_blocks_count_data - 1;
			usage->total_blocks = root_super.s_blocks_count - root_super.s_first_data_block;
			break;
		}
#endif
		default:
			//node_free(&root_super,ds.node);
			err = EINVAL;
	}
	node_close(&root_super,ds.node);
	return err;
}


/** f2bfs_setinfo function
/*
/* Setinfo. Format disk and disk check is supported. 
/* \note Disk check is not fully tested.
*/
static int f2bfs_setinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len )
{
	// No setinfo keys supported at present
	if(key == FS_INFO_DISK_USAGE)
	{
		return f2bfs_format();
	}
	
    if(key == 10)
	{
		int err;
		cyg_uint32 *output = (cyg_uint32*)buf;
		err = f2bfs_fo_check_repair_size(buf,len);
		*output = err;

		return ENOERR;
	}
	
	return EINVAL;
}


//==========================================================================
// File operations


/** f2bfs_fo_read function
/*
/* Read data from the file.
*/
static int f2bfs_fo_read      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	f2bfs_inode *node = (f2bfs_inode *)fp->f_data;
	int i;
	int err = ENOERR;
	off_t pos = fp->f_offset;
	ssize_t resid = uio->uio_resid;

	// Loop over the io vectors until there are none left
	for( i = 0; i < uio->uio_iovcnt; i++ )
	{
		cyg_iovec *iov  = &uio->uio_iov[i];
		char *buf       = (char *)iov->iov_base;
		cyg_uint32 len  = iov->iov_len;
		
		
		err = node_read_data(&root_super,node,pos,buf,&len);
		if(err == EEOF)
			break;
			
		if( err != ENOERR )
			return EIO;
		#if F2BFS_DIAG_LEVEL > 0
		diag_printf("Readed %d bytes, pos %d, buffer %p\n",len,pos,buf);	
		#endif
		resid       	-= len;
		pos             += len;
	}

// We successfully read some data, update the node's access time
// and update the file offset and transfer residue.

	node->i_atime = cyg_timestamp();

	uio->uio_resid = resid;
	fp->f_offset = pos;

	return ENOERR;
}


/** f2bfs_fo_write function
/*
/* Write data to file.
*/
static int f2bfs_fo_write     (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
    f2bfs_inode *node = (f2bfs_inode *)fp->f_data;
    off_t pos = fp->f_offset;
    ssize_t resid = uio->uio_resid;    
    int err = ENOERR;
    int i;

    // If the APPEND mode bit was supplied, force all writes to
    // the end of the file.
    if( fp->f_flag & CYG_FAPPEND )
        pos = fp->f_offset = node->i_size;
    
    // Now loop over the iovecs until they are all done, or
    // we get an error.
    for( i = 0; i < uio->uio_iovcnt; i++ )
    {
        cyg_iovec *iov  = &uio->uio_iov[i];
        char *buf       = (char *)iov->iov_base;
        cyg_uint32 len  = iov->iov_len;

        err = node_write_data(&root_super,node,pos,buf,&len);
		if(err == ENOSPC)
			break;
		
		if( err != ENOERR)
			return err;
			
		#if F2BFS_DIAG_LEVEL > 0
		diag_printf("Writed %d bytes, pos %d, buffer %p\n",len,pos,buf);	
		#endif
		resid       	-= len;
		pos             += len;
    }

    // We wrote some data successfully, update the modified and access
    // times of the node, increase its size appropriately, and update
    // the file offset and transfer residue.
    node->i_mtime =
    node->i_ctime = cyg_timestamp();
 
	if(err == ENOERR)
	{
		if((err = node_validate(&root_super,node) != ENOERR))
			return err;
	}
   
    fp->f_offset	= pos;
    uio->uio_resid	= resid;
    return err;
}


/** f2bfs_fo_lseek function
/*
/* Seek to a new file position.
*/
static int f2bfs_fo_lseek     (struct CYG_FILE_TAG *fp, off_t *apos, int whence )
{
    f2bfs_inode *node = (f2bfs_inode *)fp->f_data;
    off_t pos = *apos;

    switch( whence )
    {
    case SEEK_SET:
        // Pos is already where we want to be.
        break;

    case SEEK_CUR:
        // Add pos to current offset.
        pos += fp->f_offset;
        break;

    case SEEK_END:
        // Add pos to file size.
        pos += node->i_size;
        break;

    default:
        return EINVAL;
    }
    
	// Check that pos is still within current file size, 
    // or at the very end
    if (pos < 0 || pos > node->i_size)
        return EINVAL;
    // All OK, set fp offset and return new position.
    *apos = fp->f_offset = pos;
    
    return ENOERR;
}


/** f2bfs_fo_ioctl function
/*
/* Handle ioctls. Currently none are defined.
*/
static int f2bfs_fo_ioctl     (struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
                                CYG_ADDRWORD data)
{
    // No Ioctls currenly defined.
    return EINVAL;
}


/** f2bfs_fo_fsync function
/*
/* Force the file out to data storage.
*/
static int f2bfs_fo_fsync     (struct CYG_FILE_TAG *fp, int mode )
{
    // Data is always permanently where it belongs, nothing to do
    // here.
    return ENOERR;
}


/** f2bfs_fo_close function
/*
/* Close a file. We just decrement the refcnt and let it go away if
/* that is all that is keeping it here.
*/
static int f2bfs_fo_close     (struct CYG_FILE_TAG *fp)
{
	int err = ENOERR;
    f2bfs_inode *node = (f2bfs_inode *)fp->f_data;

	node->i_refcnt--;
	if(node->i_refcnt > 1)
	{
		return EBUSY;
	}
	err = node_write(&root_super,node);
	node_close(&root_super,node);
	
    fp->f_data = 0;     // zero data pointer
    
    return err;
}


/** f2bfs_fo_fstat function
/*
/* Get file status.
*/
static int f2bfs_fo_fstat     (struct CYG_FILE_TAG *fp, struct stat *buf )
{
    f2bfs_inode *node = (f2bfs_inode *)fp->f_data;

    // Fill in the status
    buf->st_mode        = node->i_mode;
    buf->st_ino         = (ino_t)node->i_node_id;
    buf->st_dev         = 0;
    buf->st_nlink       = node->i_links_count;
    buf->st_uid         = 0;
    buf->st_gid         = 0;
    buf->st_size        = node->i_size;
    buf->st_atime       = node->i_atime;
    buf->st_mtime       = node->i_mtime;
    buf->st_ctime       = node->i_ctime;
    
    return ENOERR;
}


/** f2bfs_fo_getinfo function
/*
/* Get info. Currently only supports fpathconf().
*/
static int f2bfs_fo_getinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len )
{
    f2bfs_inode *node = (f2bfs_inode *)fp->f_data;    
    int err = EINVAL;

    switch( key )
    {
    case FS_INFO_CONF:
        err = f2bfs_pathconf( node, (struct cyg_pathconf_info *)buf );
        break;
#ifdef CYGSEM_FILEIO_INFO_DISK_USAGE
     case FS_INFO_DISK_USAGE: {
	      struct cyg_fs_disk_usage *usage = (struct cyg_fs_disk_usage *) buf;

	      int free_block_count = root_super.s_free_blocks_count - root_super.s_r_blocks_count - 1;
	      if(free_block_count < 0)
			free_block_count = 0;
			
	      usage->block_size	= root_super.s_block_size;
	      usage->free_blocks	= free_block_count;
	      usage->total_blocks = root_super.s_blocks_count;
	      break;   
	 }
#endif
    default:
        err = EINVAL;
    }
    return err;
}


/** f2bfs_fo_setinfo function
/*
/* Set info. Nothing supported here.
*/
static int f2bfs_fo_setinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len )
{
    // No setinfo key supported at present
    return EINVAL;
}


//==========================================================================
// Directory operations

/** f2bfs_fo_dirread function
/*
/* Read a single directory entry from a file.
*/
static int f2bfs_fo_dirread      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	f2bfs_inode *dir = &root_inode;//(f2bfs_inode *)fp->f_data;
	off_t pos = fp->f_offset;
	int err = ENOERR;
	struct dirent *ent = (struct dirent *)uio->uio_iov[0].iov_base;
	char *nbuf = ent->d_name;
	int nlen = sizeof(ent->d_name)-1;
	off_t len = uio->uio_iov[0].iov_len;
	f2bfs_dir_entry d;
	
	if( len < sizeof(struct dirent) )
		return EINVAL;
	
	// Here we  have the first fragment of a directory entry.

	if( pos == dir->i_size)
		return EIO;
	
	while(pos < dir->i_size)
	{
		err = f2bfs_read_direntry(&root_super,dir,pos,&d);
		if(err != ENOERR)
			return err;
			
		pos += sizeof(f2bfs_dir_entry);
		if(d.rec_len == 0)
			continue;
			
		memcpy( nbuf, d.name,d.name_len);
		nbuf += d.name_len;
		nlen -= d.name_len;
		#ifdef CYGPKG_FS_RAM_RET_DIRENT_DTYPE
		ent->d_type = 0;
		#endif
		break;
	}

	if( pos > dir->i_size || d.rec_len == 0)
		return EIO;
		
	// A successful read. Terminate the entry name with a NUL, set the
	// residue and set the file offset to restart at the next
	// directory entry.
	
	*nbuf = '\0';
	uio->uio_resid -= sizeof(struct dirent);
	fp->f_offset = pos; 
	
	return ENOERR;
}


/** f2bfs_fo_dirlseek function
/*
/* Seek directory to start.
*/
static int f2bfs_fo_dirlseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence )
{
    // Only allow SEEK_SET to zero
    
    if( whence != SEEK_SET || *pos != 0)
        return EINVAL;

    *pos = fp->f_offset = 0;
    
    return ENOERR;
}


int f2bfs_fo_check_repair_size(const char *name, size_t block_len)
{
	f2bfs_dirsearch ds;
	int err = ENOERR;

	init_dirsearch( &ds, (f2bfs_inode *)&root_inode, name );
	
	err = f2bfs_find( &ds );
	
	if( err == ENOERR )
	{
		cyg_uint32 blocks      = ds.node->i_size/block_len;
		cyg_uint32 blocks_rem  = ds.node->i_size%block_len;
		cyg_uint32 block_count = (blocks*block_len) >> root_super.s_log_block_size;
		cyg_uint32 read_pos    = block_count << root_super.s_log_block_size;
		cyg_uint32 read_len    = ds.node->i_size - read_pos - blocks_rem;
		cyg_uint32 len         = read_len;
		char *data_block;
		
		if(blocks_rem == 0)
		{
			node_close(&root_super, ds.node );
			return ENOERR;
		}
		#if F2BFS_DIAG_LEVEL > 0
		diag_printf("Data blocks %d, data remained %d\n",blocks,blocks_rem);
		diag_printf("Flash block count %d, read pos %d, read len %d\n",block_count,read_pos,
		read_len);
		#endif
		
		if(read_len == 0)
		{
			node_free_space(&root_super,ds.node);
			node_validate(&root_super,ds.node);
			return -1;
		}
		
		data_block = malloc(read_len);
		CYG_ASSERT(data_block != NULL, "Cannot alloc free space to correct file\n");
		
		err = node_read_data(&root_super,ds.node,read_pos,data_block,&len);
		if(err != ENOERR || len != read_len)
		{
			free(data_block);
			return err;
		}
		
		err = node_rewrite_data(&root_super,ds.node,read_pos,data_block,&len);
		
		free(data_block);
		if(err != ENOERR || len != read_len)
		{
			return err;
		}
		
		ds.node->i_size = read_pos + read_len;
		err = node_validate(&root_super,ds.node);
		if(err != ENOERR)
			return err;
		
		node_close(&root_super, ds.node );
		
		return -1;
	}
	
	node_close(&root_super, ds.node );
	
	return err;
}

// EOF blockfs.c
