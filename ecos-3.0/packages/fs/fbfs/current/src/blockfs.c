//==========================================================================
//
//      blockfs.c
//
//      SIMPLEFS file system IO
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

#include <cyg/fs/blockfsconfig.h>
#include <cyg/fs/block.h>
#include <cyg/fs/node.h>




//==========================================================================
// Forward definitions

// Filesystem operations
static int blockfs_mount    ( cyg_fstab_entry *fste, cyg_mtab_entry *mte );
static int blockfs_umount   ( cyg_mtab_entry *mte );
static int blockfs_open     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int mode,  cyg_file *fte );
static int blockfs_unlink   ( cyg_mtab_entry *mte, cyg_dir dir, const char *name );
static int blockfs_mkdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name );
static int blockfs_rmdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name );
static int blockfs_rename   ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2 );
static int blockfs_link     ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2, int type );
static int blockfs_opendir  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_file *fte );
static int blockfs_chdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_dir *dir_out );
static int blockfs_stat     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
struct stat *buf);
static int blockfs_getinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len );
static int blockfs_setinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len );

// File operations
static int blockfs_fo_read      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int blockfs_fo_write     (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int blockfs_fo_lseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence );
static int blockfs_fo_ioctl     (struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
CYG_ADDRWORD data);
static int blockfs_fo_fsync     (struct CYG_FILE_TAG *fp, int mode );
static int blockfs_fo_close     (struct CYG_FILE_TAG *fp);
static int blockfs_fo_fstat     (struct CYG_FILE_TAG *fp, struct stat *buf );
static int blockfs_fo_getinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len );
static int blockfs_fo_setinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len );

// Directory operations
static int blockfs_fo_dirread      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int blockfs_fo_dirlseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence );


//==========================================================================
// Filesystem table entries

// -------------------------------------------------------------------------
// Fstab entry.
// This defines the entry in the filesystem table.
// For simplicity we use _FILESYSTEM synchronization for all accesses since
// we should never block in any filesystem operations.

FSTAB_ENTRY( blockfs_fste, "blockfs", 0,
CYG_SYNCMODE_FILE_FILESYSTEM|CYG_SYNCMODE_IO_FILESYSTEM,
blockfs_mount,
blockfs_umount,
blockfs_open,
blockfs_unlink,
blockfs_mkdir,
blockfs_rmdir,
blockfs_rename,
blockfs_link,
blockfs_opendir,
blockfs_chdir,
blockfs_stat,
blockfs_getinfo,
blockfs_setinfo);

// -------------------------------------------------------------------------
// File operations.
// This set of file operations are used for normal open files.

static cyg_fileops blockfs_fileops =
{
    blockfs_fo_read,
    blockfs_fo_write,
    blockfs_fo_lseek,
    blockfs_fo_ioctl,
    cyg_fileio_seltrue,
    blockfs_fo_fsync,
    blockfs_fo_close,
    blockfs_fo_fstat,
    blockfs_fo_getinfo,
    blockfs_fo_setinfo
};

// -------------------------------------------------------------------------
// Directory file operations.
// This set of operations are used for open directories. Most entries
// point to error-returning stub functions. Only the read, lseek and
// close entries are functional.

static cyg_fileops blockfs_dirops =
{
    blockfs_fo_dirread,
    (cyg_fileop_write *)cyg_fileio_enosys,
    blockfs_fo_dirlseek,
    (cyg_fileop_ioctl *)cyg_fileio_enosys,
    cyg_fileio_seltrue,
    (cyg_fileop_fsync *)cyg_fileio_enosys,
    blockfs_fo_close,
    (cyg_fileop_fstat *)cyg_fileio_enosys,
    (cyg_fileop_getinfo *)cyg_fileio_enosys,
    (cyg_fileop_setinfo *)cyg_fileio_enosys
};

//==========================================================================
// Data typedefs
// Some forward typedefs for the main data structures.


fbfs_super_block root_super  BLOCK_FS_STATIC_DTATA_ATRIBUTES;
fbfs_inode	 root_inode  BLOCK_FS_STATIC_DTATA_ATRIBUTES;

extern cyg_uint8 inode_bitmap[BLOCKFS_NODES_PER_BLOCK/8];
extern cyg_uint8 block_bitmap[BLOCKFS_NUM_BOLOCKS/8];


//==========================================================================
// Forward defs

static int del_direntry( fbfs_inode *dir, const char *name, int namelen );


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

// -------------------------------------------------------------------------
// dec_refcnt()
// Decrment the reference count on a node. If this makes the ref count
// zero, and the number of links is either zero for a file or one for
// a node, then this node is detached from the directory tree and can
// be freed.

static int dec_refcnt( fbfs_inode *node )
{
    int err = ENOERR;
    node->i_links_count--;

    if( node->i_links_count == 0 &&
    ((S_ISREG(node->i_mode) && node->i_links_count == 0 ) /*||
    (S_ISDIR(node->i_mode) && node->i_links_count == 1)*/ )
    )
    {
        // This node it now totally detached from the directory tree,
        // so delete it.

        /*if( S_ISDIR(node->i_mode) )
        {
                del_direntry( node, ".", 1 );
                del_direntry( node, "..", 2 );
        }*/

        err = node_free_space(&root_super, node );
        if(err != ENOERR)
            return err;

        err = node_set_free(&root_super,node->i_node_id,true);
        if(err != ENOERR)
            return err;
    }

    return err;
}

static int find_free_dirrec(fbfs_inode *dir, int *offset)
{
    off_t pos = 0;
    fbfs_dir_entry d;
    int err;
    cyg_uint32	len;

    while(pos < dir->i_size)
    {
        len = sizeof(fbfs_dir_entry);
        err = node_read_data(&root_super,dir,pos,&d,&len);
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

// -------------------------------------------------------------------------
// dec_nlink()
// Decrement a node's link count. Since this has to do all the same
// work as dec_refcnt() we implement this using that function by
// essentially transferring the count to refcnt and then decrement
// that.

static int dec_nlink( fbfs_inode *node )
{
    node->i_refcnt++;

    node->i_links_count--;

    return dec_refcnt( node );
}

//==========================================================================
// Directory operations

// -------------------------------------------------------------------------
// add_direntry()
// Add an entry to a directory. This is added as a chain of entry
// fragments until the name is exhausted.

static int add_direntry( fbfs_inode *dir,       // dir to add to
	const char *name,			// name to add
	int namelen,				// length of name
	fbfs_inode *node        // node to reference
)
{
    //off_t pos = 0;
    fbfs_dir_entry d;
    int err;
    int new_pos;
    cyg_uint32	len;

    //if(dir->)
    // Loop inserting fragments of the name into the directory until we
    // have found a home for them all.

    if(namelen >= CYGNUM_BLOCKFS_DIRENT_NAME_SIZE)
            namelen = CYGNUM_BLOCKFS_DIRENT_NAME_SIZE - 1;	

    d.inode		   = node->i_node_id;
    d.rec_len      = sizeof(fbfs_dir_entry);
    d.name_len     = namelen;
    d.offset       = dir->i_size;

    memcpy( d.name, name, namelen ); 
    d.name[namelen] = 0;

    /*err = node_alloc_space(&root_super,dir,sizeof(fbfs_dir_entry));
    if( err != ENOERR )
            return err;*/
    err = find_free_dirrec(dir,&new_pos);
    if(err == ENOENT)
    {
        len  = sizeof(fbfs_dir_entry);
        err = node_write_data(&root_super,dir,dir->i_size,&d,&len);
        if( err != ENOERR )
            return err;

        dir->i_size += sizeof(fbfs_dir_entry);
    }
    else if(err == ENOERR)
    {
        len  = sizeof(fbfs_dir_entry);
        d.offset = new_pos;
        err = node_write_data(&root_super,dir,new_pos,&d,&len);
        if( err != ENOERR )
            return err;
    }
    else
        return err;	
    //node_read_data(&root_super,dir,dir->i_size,&d,&len);	
    //d->last = 1;        // Mark last fragment

    // Update directory times
    dir->i_mtime =
    dir->i_ctime = cyg_timestamp();

    // Count the new link
    node->i_links_count++;
    err = node_write(&root_super,dir);
    if(err != ENOERR)
        return err;

    err = node_set_used(&root_super,node->i_node_id,true);
    if(err != ENOERR)
        return err;
    return node_write(&root_super,node);
}

// -------------------------------------------------------------------------
// find_direntry()
// Find a directory entry for the name and return a pointer to the first
// entry fragment.

static fbfs_dir_entry * find_direntry(fbfs_super_block *super, fbfs_inode *dir, const char *name, int namelen,
		fbfs_dir_entry *d)
{
    cyg_uint32 rw_len;
    int err;
    int pos = 0;
    // Loop over all the entries until a match is found or we run out
    // of data.
    while( pos < dir->i_size )
    {
        rw_len = sizeof(fbfs_dir_entry) ;
        err = node_read_data(super,dir,pos,d, &rw_len);
        if(err != ENOERR)
            return (fbfs_dir_entry *)NULL;

        //diag_printf("Dir: %d, %d, %s\n",(int)d->inode,(int)d->name_len,d->name);	
        if( rw_len != sizeof(fbfs_dir_entry))
            return (fbfs_dir_entry *)NULL;

        // Otherwise move on to next entry in chain
        pos += rw_len;

        //unused directory entry
        /*if(d->inode == 0)
                continue;*/

        if(d->rec_len == 0)
            continue;
        // Is this the directory entry we're looking for?
        if ( match( d->name, name, namelen )/* && namelen == d->name_len*/ )
            return d;
    }

    return (fbfs_dir_entry *)NULL;	
}

// -------------------------------------------------------------------------
// del_direntry()
// Delete a named directory entry. Find it and then follow the chain
// deleting the fragments as we go.

static int del_direntry( fbfs_inode *dir, const char *name, int namelen )
{
    fbfs_dir_entry dd;
    //fbfs_inode     *node;
    //off_t pos = 0;
    cyg_uint32	len;
    fbfs_dir_entry *d = find_direntry(&root_super, dir, name, namelen, &dd);
    int err;

    if( d == NULL )
        return ENOENT;

    d->rec_len = 0 ;//deleted

    len = sizeof(fbfs_dir_entry);
    err = node_write_data(&root_super,dir,d->offset,d,&len);
    if( err != ENOERR )
        return ENOENT;

    /*node = (fbfs_inode*)malloc(sizeof(fbfs_inode));
    CYG_ASSERT(node,"Cannot allocated space for node\n");
    err = node_read(&root_super,node,dd.inode);
    if(err != ENOERR)
    {
            free(node);
            return err;
    }

    dec_nlink( node );*/

    return ENOERR;
}

//==========================================================================
// Directory search

// -------------------------------------------------------------------------
// init_dirsearch()
// Initialize a dirsearch object to start a search

static void init_dirsearch( blockfs_dirsearch *ds,
	fbfs_inode *dir,
	const char *name)
{
    ds->dir      = dir;
    ds->path     = name;
    ds->node     = NULL;
    ds->name     = name;
    ds->namelen  = 0;
    ds->last     = false;
}

// -------------------------------------------------------------------------
// find_entry()
// Search a single directory for the next name in a path and update the
// dirsearch object appropriately.

static int find_entry( blockfs_dirsearch *ds )
{
    fbfs_inode *dir = ds->dir;
    char *name = ds->path;
    char *n = name;
    int namelen = 0;
    //off_t pos = 0;
    fbfs_dir_entry dd;
    fbfs_dir_entry *d;

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
    ds->name = name;
    ds->namelen = namelen;

    // Here we have the name and its length set up.
    // Search the directory for a matching entry

    d = find_direntry(&root_super, dir, name, namelen,  &dd);

    if( d == NULL )
        return ENOENT;

    if(ds->node == NULL)
    {
        ds->node = node_alloc_read(&root_super,d->inode);
        if(ds->node == NULL)
            return EIO;
        return ENOERR;
    }

    // pass back the node we have found
    return node_read(&root_super,ds->node,d->inode);
    /*ds->node = d->inode;

    return ENOERR;*/

}

// -------------------------------------------------------------------------
// blockfs_find()
// Main interface to directory search code. This is used in all file
// level operations to locate the object named by the pathname.

static int blockfs_find( blockfs_dirsearch *d )
{
    int err;

    // Short circuit empty paths
    if( *(d->path) == '\0' )
    {
        d->node = (fbfs_inode*)malloc(sizeof(fbfs_inode));
        CYG_ASSERT(d->node,"Cannot alloc new node\n");
        *d->node = root_inode;
        return ENOERR;
    }


    // iterate down directory tree until we find the object
    // we want.
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
// Pathconf support
// This function provides support for pathconf() and fpathconf().

static int blockfs_pathconf( fbfs_inode *node, struct cyg_pathconf_info *info )
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

int blockfs_format(void)
{
    int err;
    cyg_uint32		wr_len;

    root_super.s_blocks_count               = BLOCKFS_NUM_BOLOCKS;
    root_super.s_blocks_per_group           = BLOCKFS_NUM_BOLOCKS;
    root_super.s_checkinterval              = 0;
    root_super.s_first_data_block           = 8;
    root_super.s_first_data_block_reserved  = 4;
    root_super.s_free_blocks_count          = BLOCKFS_NUM_BOLOCKS;
    root_super.s_free_inodes_count          = BLOCKFS_NODES_PER_BLOCK;
    root_super.s_inodes_count               = BLOCKFS_NODES_PER_BLOCK;
    root_super.s_inodes_per_group           = BLOCKFS_NODES_PER_BLOCK;
    root_super.s_r_blocks_count             = 3;//BLOCKFS_NUM_FRAM_BLOCKS; 3 bloky pro root dir 1 a 2 pro sys soubory zbytek FRAM FS data
    //root_super.s_lastcheck
    root_super.s_log_block_size             = CYGNUM_BLOCKFS_BLOCK_SIZE;
    root_super.s_magic                      = BLOCKFS_MAGIC;
    root_super.s_max_mnt_count              = -1;
    root_super.s_minor_rev_level            = 1;
    root_super.s_mnt_count                  = 0;
    root_super.s_state                      = 0;
    root_super.s_wtime                      = 0;

    root_super.block_goroup_info.bg_block_bitmap        = 1;
    root_super.block_goroup_info.bg_free_blocks_count   = BLOCKFS_NUM_BOLOCKS; //5 bloku 1xsuper 2xbmp 2xnode tab
    root_super.block_goroup_info.bg_free_inodes_count   = BLOCKFS_NODES_PER_BLOCK;
    root_super.block_goroup_info.bg_inode_bitmap        = 2;
    root_super.block_goroup_info.bg_inode_table         = 3;
    root_super.block_goroup_info.bg_start_search        = 8;
    root_super.block_goroup_info.bg_used_dirs_count     = 0;


    memset(inode_bitmap,0xff,sizeof(inode_bitmap));
    memset(block_bitmap,0xff,sizeof(block_bitmap));
    block_set_used(&root_super,0,false);
    block_set_used(&root_super,1,false);
    block_set_used(&root_super,2,false);
    block_set_used(&root_super,3,false);

    //node_set_used(&root_super,0,false);

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
    if(block_write(&root_super,0,0,sizeof(fbfs_super_block),&root_super,&wr_len) 
        != ENOERR || wr_len != sizeof(fbfs_super_block))
    {
        return EIO;
    }

    memset(&root_inode,0,sizeof(fbfs_inode));

    root_inode.i_atime = 
    root_inode.i_ctime = 
    root_inode.i_mtime = cyg_timestamp();
    root_inode.i_mode  = __stat_mode_DIR|S_IRWXU|S_IRWXG|S_IRWXO;
    root_inode.i_links_count = 1;
    root_inode.i_flags = 0;
    root_inode.i_size  = 0;
    root_inode.i_flags = FILE_FLAG_SYSTEM_FILE;

    //ulozi se v add_direntry
    /*err = node_write(&root_super,&root_inode);
    if(err != ENOERR)
            return err;*/

    err = add_direntry( &root_inode, ".", 1, &root_inode );
    if( err == ENOERR )
        err = add_direntry( &root_inode, "..", 2, &root_inode );
    return err;
}

//==========================================================================
// Filesystem operations

// -------------------------------------------------------------------------
// blockfs_mount()
// Process a mount request. This mainly creates a root for the
// filesystem.

static int blockfs_mount    ( cyg_fstab_entry *fste, cyg_mtab_entry *mte )
{
    //fbfs_inode *root;
    cyg_uint32	wr_len;
    int err;
    cyg_flashaddr_t err_addr;

    cyg_flash_init(NULL);
    // Allocate a node to be the root of this filesystem and initialize it.
    if(cyg_flash_read(BLOCKFS_BLOCK_FLASH_OFFSET,&root_super,sizeof(fbfs_super_block),&err_addr) != CYG_FLASH_ERR_OK)
    {
        return EIO;
    }

    if(root_super.s_magic != BLOCKFS_MAGIC)
    {
        if((err = blockfs_format()) != ENOERR)
            return err;
    }

    root_super.s_mtime = cyg_timestamp();
    root_super.s_mnt_count++;

    err = block_write(&root_super,0,0,sizeof(fbfs_super_block),&root_super,&wr_len);
    if(err != ENOERR || wr_len != sizeof(fbfs_super_block))
        return err;

    if(block_read(&root_super,root_super.block_goroup_info.bg_block_bitmap,
            0,sizeof(block_bitmap),block_bitmap,&wr_len) != ENOERR ||
            wr_len != sizeof(block_bitmap))
    {
        return EIO;
    }

    if(block_read(&root_super,root_super.block_goroup_info.bg_inode_bitmap,
            0,sizeof(inode_bitmap),inode_bitmap,&wr_len) != ENOERR ||
            wr_len != sizeof(inode_bitmap))
    {
        return EIO;
    }

    //pak asi na\E8\EDst root_inode
    err = node_read(&root_super,&root_inode,0);
    mte->root = (cyg_dir)&root_inode;

    return ENOERR;
}

// -------------------------------------------------------------------------
// blockfs_umount()
// Unmount the filesystem. This will currently only succeed if the
// filesystem is empty.

static int blockfs_umount   ( cyg_mtab_entry *mte )
{
    return EBUSY;
}

// -------------------------------------------------------------------------
// blockfs_open()
// Open a file for reading or writing.

static int blockfs_open     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int mode,  cyg_file *file )
{

    blockfs_dirsearch ds;
    int err = ENOERR;
    cyg_bool system_file = false;

    init_dirsearch( &ds, (fbfs_inode *)&root_inode, name );

    err = blockfs_find( &ds );

    if(ds.name[0] == '.')
        system_file = true;

    diag_printf("Mode %X\n",mode);	
    if( err == ENOENT )
    {
        if( ds.last && (mode & O_CREAT) )
        {
            // No node there, if the O_CREAT bit is set then we must
            // create a new one. The dir and name fields of the dirsearch
            // object will have been updated so we know where to put it.

            ds.node = node_alloc(&root_super, __stat_mode_REG|S_IRWXU|S_IRWXG|S_IRWXO);
            if( ds.node == NULL )
                return ENOSPC;

            //ds.node->i_refcnt++;
            ds.node->i_flags = (system_file) ? FILE_FLAG_SYSTEM_FILE : FILE_FLAG_FILE;
            err = add_direntry( ds.dir, ds.name, ds.namelen, ds.node );

            if( err != ENOERR )
            {
                node_free(&root_super, ds.node );
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
        //if(ds.node->i_flags == FILE_FLAG_SYSTEM_FILE)
        {
            err = node_free_space(&root_super,ds.node);
            if(err != ENOERR)
            {
                node_free(&root_super, ds.node );
                return err;
            }
            /*err = node_write(&root_super,ds.node);
            if(err != ENOERR)
            {
                    node_free(&root_super, ds.node );
                    return err;
            }*/
        }
        /*else
                return EINVAL;*/

        // The node exists. If the O_CREAT and O_EXCL bits are set, we
        // must fail the open.
        /*if( (mode & O_CREAT) || (mode & O_EXCL))
        {
                node_free(&root_super, ds.node );
                return EEXIST;
        }*/
    }
    /*else //if( err == ENOERR && (mode & O_TRUNC ) )
    {
        // If the O_TRUNC bit is set we must clean out the file data.

    }*/

    /*if( err != ENOERR ) 
        return err;*/

    // Check that we actually have a file here
    if( S_ISDIR(ds.node->i_mode) ) 
        return EISDIR;

    ds.node->i_refcnt++;
    //node->refcnt++;       // Count successful open

    // Initialize the file object
    file->f_flag        |= mode & CYG_FILE_MODE_MASK;
    file->f_type        = CYG_FILE_TYPE_FILE;
    file->f_ops         = &blockfs_fileops;
    file->f_offset      = (mode&O_APPEND) ? ds.node->i_size : 0;
    file->f_data        = (CYG_ADDRWORD)ds.node;
    file->f_xops        = 0;

    return ENOERR;
}

// -------------------------------------------------------------------------
// blockfs_unlink()
// Remove a file link from its directory.

static int blockfs_unlink   ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
    blockfs_dirsearch ds;
    int err;

    init_dirsearch( &ds, (fbfs_inode *)&root_inode, name );

    err = blockfs_find( &ds );

    if( err != ENOERR )
    {
        node_free(&root_super,ds.node);
        return err;
    }

    // Cannot unlink directories, use rmdir() instead
    if( S_ISDIR(ds.node->i_mode) )
    {
        node_free(&root_super,ds.node);
        return EPERM;
    }

    if(ds.node->i_refcnt > 0)
    {
        //node_free(&root_super,ds->node);
        return EBUSY;
    }

    // Delete it from its directory
    err = del_direntry( ds.dir, ds.name, ds.namelen );
    if(err != ENOERR)
    {
        node_free(&root_super,ds.node);
        return err;
    }
    err = dec_refcnt( ds.node );
    node_free(&root_super,ds.node);
    return err;
}

// -------------------------------------------------------------------------
// blockfs_mkdir()
// Create a new directory.

static int blockfs_mkdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
#ifdef FBFS_MKDIR
    blockfs_dirsearch ds;
    fbfs_inode *node = NULL;
    int err;

    init_dirsearch( &ds, (fbfs_inode *)dir, name );

    err = blockfs_find( &ds );

    if( err == ENOENT )
    {
        if( ds.last )
        {
            // The entry does not exist, and it is the last element in
            // the pathname, so we can create it here.
            int doterr, dotdoterr, direrr;

            node = node_alloc( __stat_mode_DIR | S_IRWXU|S_IRWXG|S_IRWXO);

            if( node == NULL )
                return ENOSPC;

            // Add "." and ".." entries.
            doterr = add_direntry( node, ".", 1, node );
            dotdoterr = add_direntry( node, "..", 2, ds.dir );

            // And add to parent directory.
            direrr = add_direntry( ds.dir, ds.name, ds.namelen, node );

            // check for any errors in that...
            if( doterr+dotdoterr+direrr != ENOERR )
            {
                // For each of the add_direntry() calls that succeeded,
                // we must now undo it.

                if( doterr == ENOERR )
                    del_direntry( node, ".", 1 );
                else 
                    err = doterr;

                if( dotdoterr == ENOERR )
                    del_direntry( node, "..", 2 );
                else 
                    err = dotdoterr;

                if( direrr == ENOERR )
                    del_direntry( ds.dir, ds.name, ds.namelen );
                else 
                    err = direrr;

                // Free the data and the node itself.
                node_free( node );
                //free_node( node );
            }
            else err = ENOERR;
        }
        // If this was not the last element, then and intermediate
        // directory does not exist.
    }
    else
    {
        // If there we no error, something already exists with that
        // name, so we cannot create another one.

        if( err == ENOERR )
            err = EEXIST;
    }

    return err;
#else
    return EINVAL;
#endif
}

// -------------------------------------------------------------------------
// blockfs_rmdir()
// Remove a directory.

static int blockfs_rmdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
#ifdef FBFS_MKDIR
    blockfs_dirsearch ds;
    int err;

    init_dirsearch( &ds, (blockfs_node *)dir, name );

    err = blockfs_find( &ds );

    if( err != ENOERR )
        return err;

    // Check that this is actually a directory.
    if( !S_ISDIR(ds.node->mode) )
                return EPERM;

    // Delete the entry. This will adjust the link values
    // accordingly and if the directory is now unreferenced,
    // will cause it to be deleted.

    err = del_direntry( ds.dir, ds.name, ds.namelen );

    return err;
#else
	return EINVAL;
#endif
}

// -------------------------------------------------------------------------
// blockfs_rename()
// Rename a file/dir.

static int blockfs_rename   ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2 )
{
#ifdef FBFS_RENAME
    blockfs_dirsearch ds1, ds2;
    int err;

    init_dirsearch( &ds1, (fbfs_inode *)dir1, name1 );

    err = blockfs_find( &ds1 );

    if( err != ENOERR ) 
        return err;

    init_dirsearch( &ds2, (fbfs_inode *)dir2, name2 );

    err = blockfs_find( &ds2 );

    // Allow through renames to non-existent objects.
    if( ds2.last && err == ENOENT )
    ds2.node = NULL, err = ENOERR;

    if( err != ENOERR ) 
        return err;

    // Null rename, just return
    if( ds1.node == ds2.node )
        return ENOERR;

    // First deal with any entry that is at the destination
    if( ds2.node )
    {
        // Check that we are renaming like-for-like

        if( !S_ISDIR(ds1.node->mode) && S_ISDIR(ds2.node->mode) )
            return EISDIR;

        if( S_ISDIR(ds1.node->mode) && !S_ISDIR(ds2.node->mode) )
            return ENOTDIR;

        // Now delete the destination directory entry

        err = del_direntry( ds2.dir, ds2.name, ds2.namelen );

        if( err != ENOERR ) 
            return err;

    }

    // Now we know that there is no clashing node at the destination,
    // make a new direntry at the destination and delete the old entry
    // at the source.

    err = add_direntry( ds2.dir, ds2.name, ds2.namelen, ds1.node );

    if( err == ENOERR )
        err = del_direntry( ds1.dir, ds1.name, ds1.namelen );

    // Update directory times
    if( err == ENOERR )
        ds1.dir->i_ctime =
        ds1.dir->i_mtime =
        ds2.dir->i_ctime =
        ds2.dir->i_mtime = cyg_timestamp();

    return err;
#else	
	return EINVAL;
#endif
}

// -------------------------------------------------------------------------
// blockfs_link()
// Make a new directory entry for a file.

static int blockfs_link     ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2, int type )
{
    blockfs_dirsearch ds1, ds2;
    int err;

    // Only do hard links for now in this filesystem
    if( type != CYG_FSLINK_HARD )
        return EINVAL;

    init_dirsearch( &ds1, (fbfs_inode *)dir1, name1 );

    err = blockfs_find( &ds1 );

    if( err != ENOERR ) 
        return err;

    init_dirsearch( &ds2, (fbfs_inode *)dir2, name2 );

    err = blockfs_find( &ds2 );

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
}

// -------------------------------------------------------------------------
// blockfs_opendir()
// Open a directory for reading.

static int blockfs_opendir  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_file *file )
{
    blockfs_dirsearch ds;
    int err;

    init_dirsearch( &ds, (fbfs_inode *)dir, name );

    err = blockfs_find( &ds );

    if( err != ENOERR ) 
    {
        node_free(&root_super,ds.node);
        return err;
    }

    // check it is really a directory.
    if( !S_ISDIR(ds.node->i_mode) ) 
    {
        node_free(&root_super,ds.node);
        return ENOTDIR;
    }

    //node_free(&root_super,ds.node);
    ds.node->i_refcnt++;       // Count successful open

    // Initialize the file object, setting the f_ops field to a
    // special set of file ops.

    file->f_type        = CYG_FILE_TYPE_FILE;
    file->f_ops         = &blockfs_dirops;
    file->f_offset      = 0;
    file->f_data        = (CYG_ADDRWORD)ds.node;
    file->f_xops        = 0;

    return ENOERR;

}

// -------------------------------------------------------------------------
// blockfs_chdir()
// Change directory support.

static int blockfs_chdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_dir *dir_out )
{
    if( dir_out != NULL )
    {
        // This is a request to get a new directory pointer in
        // *dir_out.

        blockfs_dirsearch ds;
        int err;

        init_dirsearch( &ds, (fbfs_inode *)dir, name );

        err = blockfs_find( &ds );

        if( err != ENOERR ) 
        {
            node_free(&root_super,ds.node);
            return err;
        }

        // check it is a directory
        if( !S_ISDIR(ds.node->i_mode) )
        {
            return ENOTDIR;
            node_free(&root_super,ds.node);
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

        fbfs_inode *node = (fbfs_inode *)dir;


        node->i_refcnt--;
        if(node->i_refcnt > 0)
            return ENOERR;
        // Just decrement directory reference count.
        //dec_refcnt se snizuje a nechceme odstarnit
        node_free(&root_super,node);
    }

    return ENOERR;
}

// -------------------------------------------------------------------------
// blockfs_stat()
// Get struct stat info for named object.

static int blockfs_stat     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
struct stat *buf)
{
    blockfs_dirsearch ds;
    int err;

    init_dirsearch( &ds,&root_inode, name );

    err = blockfs_find( &ds );

    if( err != ENOERR ) 
        return err;

    // Fill in the status
    buf->st_mode        = ds.node->i_mode;
    buf->st_ino         = (ino_t)ds.node->i_node_id;
    buf->st_dev         = 0;
    buf->st_nlink       = ds.dir->i_links_count;
    buf->st_uid         = 0;
    buf->st_gid         = 0;
    buf->st_size        = ds.node->i_size;
    buf->st_atime       = ds.node->i_atime;
    buf->st_mtime       = ds.node->i_mtime;
    buf->st_ctime       = ds.node->i_ctime;

    return node_free(&root_super,ds.node);
}

// -------------------------------------------------------------------------
// blockfs_getinfo()
// Getinfo. Currently only support pathconf() and filesystem usage.

static int blockfs_getinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len )
{
    blockfs_dirsearch ds;
    int err;

    init_dirsearch( &ds, (fbfs_inode *)dir, name );

    err = blockfs_find( &ds );

    if( err != ENOERR ) 
    {
        node_free(&root_super,ds.node);
        return err;
    }

    switch( key )
    {
    case FS_INFO_CONF:
        err = blockfs_pathconf( ds.node, (struct cyg_pathconf_info *)buf );
        break;
    case FS_INFO_DISK_USAGE: {
        /*cyg_uint32 total_clusters;
        cyg_uint32 free_clusters;*/
        struct cyg_fs_disk_usage *usage = (struct cyg_fs_disk_usage *) buf;

        usage->block_size	= root_super.s_log_block_size;
        usage->free_blocks	= root_super.s_free_blocks_count;
        usage->total_blocks = root_super.s_blocks_count;
        break;
    }

    default:
        //node_free(&root_super,ds.node);
        err = EINVAL;
    }
    node_free(&root_super,ds.node);
    return err;
}

// -------------------------------------------------------------------------
// blockfs_setinfo()
// Setinfo. Nothing to support here at present.

static int blockfs_setinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len )
{
    // No setinfo keys supported at present
    return EINVAL;
}

//==========================================================================
// File operations

// -------------------------------------------------------------------------
// blockfs_fo_read()
// Read data from the file.

static int blockfs_fo_read      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
    fbfs_inode *node = (fbfs_inode *)fp->f_data;
    int i;
    int err;
    off_t pos = fp->f_offset;
    ssize_t resid = uio->uio_resid;

    // Loop over the io vectors until there are none left
    for( i = 0; i < uio->uio_iovcnt; i++ )
    {
        cyg_iovec *iov  = &uio->uio_iov[i];
        char *buf       = (char *)iov->iov_base;
        cyg_uint32 len  = iov->iov_len;

        err = node_read_data(&root_super,node,pos,buf,&len);
        if( err < 0 )
            return EIO;

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

// -------------------------------------------------------------------------
// blockfs_fo_write()
// Write data to file.

static int blockfs_fo_write     (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
    fbfs_inode *node = (fbfs_inode *)fp->f_data;
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
        if( err != ENOERR)
            return err;
        node->i_size	+= len;	
        resid       	-= len;
        pos             += len;
    }

    // We wrote some data successfully, update the modified and access
    // times of the node, increase its size appropriately, and update
    // the file offset and transfer residue.
    node->i_mtime =
    node->i_ctime = cyg_timestamp();
    /*if( pos > node->i_size )
        node->i_size = pos; */   
    if((err = node_write(&root_super,node) != ENOERR))
        return err;
    //asi ulozit nodu
    fp->f_offset	= pos;
    uio->uio_resid	= resid;
    return ENOERR;
}

// -------------------------------------------------------------------------
// blockfs_fo_lseek()
// Seek to a new file position.

static int blockfs_fo_lseek     (struct CYG_FILE_TAG *fp, off_t *apos, int whence )
{
    fbfs_inode *node = (fbfs_inode *)fp->f_data;
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

// -------------------------------------------------------------------------
// blockfs_fo_ioctl()
// Handle ioctls. Currently none are defined.

static int blockfs_fo_ioctl     (struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
                                CYG_ADDRWORD data)
{
    // No Ioctls currenly defined.
    return EINVAL;
}

// -------------------------------------------------------------------------
// blockfs_fo_fsync().
// Force the file out to data storage.

static int blockfs_fo_fsync     (struct CYG_FILE_TAG *fp, int mode )
{
    // Data is always permanently where it belongs, nothing to do
    // here.
    return ENOERR;
}

// -------------------------------------------------------------------------
// blockfs_fo_close()
// Close a file. We just decrement the refcnt and let it go away if
// that is all that is keeping it here.

static int blockfs_fo_close     (struct CYG_FILE_TAG *fp)
{
    int err = ENOERR;
    fbfs_inode *node = (fbfs_inode *)fp->f_data;

    node->i_refcnt--;
    err = node_write(&root_super,node);
    if(node->i_refcnt == 0)
        node_free(&root_super,node);

    fp->f_data = 0;     // zero data pointer
    
    return err;
}

// -------------------------------------------------------------------------
//blockfs_fo_fstat()
// Get file status.

static int blockfs_fo_fstat     (struct CYG_FILE_TAG *fp, struct stat *buf )
{
    fbfs_inode *node = (fbfs_inode *)fp->f_data;

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

// -------------------------------------------------------------------------
// blockfs_fo_getinfo()
// Get info. Currently only supports fpathconf().

static int blockfs_fo_getinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len )
{
    fbfs_inode *node = (fbfs_inode *)fp->f_data;    
    int err = EINVAL;

    switch( key )
    {
    case FS_INFO_CONF:
        err = blockfs_pathconf( node, (struct cyg_pathconf_info *)buf );
        break;
     case FS_INFO_DISK_USAGE: {
        /*cyg_uint32 total_clusters;
        cyg_uint32 free_clusters;*/
        struct cyg_fs_disk_usage *usage = (struct cyg_fs_disk_usage *) buf;

        usage->block_size	  = root_super.s_log_block_size;
        usage->free_blocks  = root_super.s_log_block_size;
        usage->total_blocks = root_super.s_free_blocks_count;
        break;   
	 }
    default:
        err = EINVAL;
    }
    return err;
}

// -------------------------------------------------------------------------
// blockfs_fo_setinfo()
// Set info. Nothing supported here.

static int blockfs_fo_setinfo   (struct CYG_FILE_TAG *fp, int key, void *buf, int len )
{
    // No setinfo key supported at present
    
    if(key == FS_INFO_DISK_USAGE)
    {
        return blockfs_format();
    }

    return EINVAL;
}


//==========================================================================
// Directory operations

// -------------------------------------------------------------------------
// blockfs_fo_dirread()
// Read a single directory entry from a file.

static int blockfs_fo_dirread      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
    cyg_uint32	rw_len;
    fbfs_inode *dir = &root_inode;//(fbfs_inode *)fp->f_data;
    off_t pos = fp->f_offset;
    int err = ENOERR;
    struct dirent *ent = (struct dirent *)uio->uio_iov[0].iov_base;
    char *nbuf = ent->d_name;
    int nlen = sizeof(ent->d_name)-1;
    off_t len = uio->uio_iov[0].iov_len;
    fbfs_dir_entry d;
    //cyg_uint8 *buf;
    //size_t size;

    if( len < sizeof(struct dirent) )
        return EINVAL;

    // look for a first name fragment

    /*while( pos < dir->size )
    {
        err = findbuffer_node( dir, pos, &buf, &size, false );
        if( err != ENOERR || size == 0)
            break;

        d = (fbfs_dir_entry *)buf;

        if( size < sizeof(fbfs_dir_entry) || !d->inuse || !d->first )
        {
            pos += sizeof(fbfs_dir_entry);
            continue;
        }

        break;
    }

    // Check we have not exceeded the size of the directory.
    if( pos == dir->size )
        return err;*/

    // Here we  have the first fragment of a directory entry.

    if( pos == dir->i_size)
        return EIO;

    while(pos < dir->i_size)
    {
        rw_len = sizeof(fbfs_dir_entry);
        err = node_read_data(&root_super,dir,pos,&d,&rw_len);
        if(err != ENOERR)
            return err;

        pos += sizeof(fbfs_dir_entry);
        if(d.rec_len == 0)
            continue;

        memcpy( nbuf, d.name,d.name_len);
        nbuf += d.name_len;
        nlen -= d.name_len;
        #ifdef CYGPKG_FS_RAM_RET_DIRENT_DTYPE
        ent->d_type = 0;
        #endif
        break;
        // if we hit the last entry, we have a successful transfer
        /*if( pos >= dir->i_size)
                break;*/
    }

    if( pos > dir->i_size || d.rec_len == 0)
        return EIO;

    // A successful read. Terminate the entry name with a NUL, set the
    // residue and set the file offset to restart at the next
    // directory entry.

    *nbuf = '\0';
    uio->uio_resid -= sizeof(struct dirent);
    //fp->f_offset = pos+sizeof(fbfs_dir_entry);
    fp->f_offset = pos; //sizeof(fbfs_dir_entry);

    return ENOERR;
}


// -------------------------------------------------------------------------
// blockfs_fo_dirlseek()
// Seek directory to start.

static int blockfs_fo_dirlseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence )
{
    // Only allow SEEK_SET to zero
    
    if( whence != SEEK_SET || *pos != 0)
        return EINVAL;

    *pos = fp->f_offset = 0;
    
    return ENOERR;
}

// -------------------------------------------------------------------------
// EOF blockfs.c
