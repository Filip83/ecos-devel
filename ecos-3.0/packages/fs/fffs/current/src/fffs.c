/*
 * fffs.c
 *
 * Created: 26.9.2012 14:58:11
 *  Author: Filip
 */

#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <pkgconf/io_fileio.h>

#include <cyg/infra/cyg_type.h>
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

#include <cyg/infra/diag.h>
#include <cyg/fileio/fileio.h>
#include <cyg/io/io.h>
#include <cyg/fs/nodes.h>
#include <cyg/fs/fffs.h>

#define TFO true

//==========================================================================
// Forward definitions

// Filesystem operations
static int fffs_mount  (cyg_fstab_entry *fste, cyg_mtab_entry *mte);
static int fffs_umount (cyg_mtab_entry *mte);
static int fffs_open   (cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int mode, cyg_file *fte);
static int fffs_unlink (cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int fffs_mkdir  (cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int fffs_rmdir  (cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int fffs_rename (cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2 );
static int fffs_link   (cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
cyg_dir dir2, const char *name2, int type );
static int fffs_opendir(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_file *fte );
static int fffs_chdir  (cyg_mtab_entry *mte, cyg_dir dir, const char *name,
cyg_dir *dir_out );
static int fffs_stat   (cyg_mtab_entry *mte, cyg_dir dir, const char *name,
struct stat *buf);
static int fffs_getinfo(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len );
static int fffs_setinfo(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
int key, void *buf, int len );

// File operations
static int fffs_fo_read   (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int fffs_fo_write  (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int fffs_fo_lseek  (struct CYG_FILE_TAG *fp, off_t *pos, int whence );
static int fffs_fo_ioctl  (struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
CYG_ADDRWORD data);
static int fffs_fo_fsync  (struct CYG_FILE_TAG *fp, int mode );
static int fffs_fo_close  (struct CYG_FILE_TAG *fp);
static int fffs_fo_fstat  (struct CYG_FILE_TAG *fp, struct stat *buf );
static int fffs_fo_getinfo(struct CYG_FILE_TAG *fp, int key,
void *buf, int len );
static int fffs_fo_setinfo(struct CYG_FILE_TAG *fp, int key,
void *buf, int len );

// Directory operations
static int fffs_fo_dirread (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int fffs_fo_dirlseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence);

//==========================================================================
// Filesystem table entries

// -------------------------------------------------------------------------
// Fstab entry.

FSTAB_ENTRY(fffs_fste, "fffs", 0,
	CYG_SYNCMODE_FILE_FILESYSTEM|CYG_SYNCMODE_IO_FILESYSTEM,
	fffs_mount,
	fffs_umount,
	fffs_open,
	fffs_unlink,
	fffs_mkdir,
	fffs_rmdir,
	fffs_rename,
	fffs_link,
	fffs_opendir,
	fffs_chdir,
	fffs_stat,
	fffs_getinfo,
	fffs_getinfo
);



// -------------------------------------------------------------------------
// File operations.

static cyg_fileops fffs_fileops =
{
	fffs_fo_read,
	fffs_fo_write,
	fffs_fo_lseek,
	fffs_fo_ioctl,
	cyg_fileio_seltrue,
	fffs_fo_fsync,
	fffs_fo_close,
	fffs_fo_fstat,
	fffs_fo_getinfo,
	fffs_fo_setinfo
};

// -------------------------------------------------------------------------
// Directory file operations.
/*
static cyg_fileops fffs_dirops =
{
	fffs_fo_dirread,
	(cyg_fileop_write *)cyg_fileio_enosys,
	fffs_fo_dirlseek,
	(cyg_fileop_ioctl *)cyg_fileio_enosys,
	cyg_fileio_seltrue,
	(cyg_fileop_fsync *)cyg_fileio_enosys,
	fffs_fo_close,
	(cyg_fileop_fstat *)cyg_fileio_enosys,
	(cyg_fileop_getinfo *)cyg_fileio_enosys,
	(cyg_fileop_setinfo *)cyg_fileio_enosys
};*/

t_flash_info fffs_flash_info
/*{
	.start_node_addr = 0,
	.end_node_addr   = 0,
	.num_files       = 0,

	.cur_node.node_type   = NODE_TYPE_START,
	.cur_node.next_node   = sizeof(t_node),
	.cur_node.prev_node   = 0,
	.cur_node.data_addr   = 0,
	.cur_node.node_length = 0,
	.cur_node.node_id     = 0,
	.cur_node.file_id     = 0,
	.cur_node.create_time = 0
}*/;

static int
fffs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte)
{
	cyg_uint32 ret = fffs_init(&fffs_flash_info);
	if(ret != FFFS_ERR_OK)
		errno = ret;

	mte->root = (cyg_dir)"spiflash";
	mte->data = (CYG_ADDRWORD)&fffs_flash_info;
	return ENOERR;
}

//soubor zacinajici teckou je skryty a soubor s
//a priponou sys je systemovy
static int
fffs_open(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             mode,
	cyg_file       *file)
{
	t_flash_info * info = (t_flash_info*)mte->data;
	cyg_uint32     ret;
	t_node_info   *node_info;
	t_node_info   tmp_node;
	cyg_uint16    file_mode = FFFS_MODE_RECORD;

	if(name == NULL)
	{
		file->f_flag   |= mode & CYG_FILE_MODE_MASK;
		file->f_type    = CYG_FILE_TYPE_FILE;
		file->f_ops     = &fffs_fileops;
		file->f_offset  = /*(mode & O_APPEND) ? node->dentry.size :*/ 0;
		file->f_data    = (CYG_ADDRWORD) NULL;
		file->f_xops    = 0;

		return ENOERR;
	}

	if(name[0] == '.')
	{
		file_mode |= FFFS_MODE_HIDEN;
		//name++;
	}

	if(!strcmp(name + (strlen(name) - 3),"sys"))
	{
		file_mode |= FFFS_MODE_SYSTEM;

	}

	if(mode & O_CREAT)
	{

		if(!strcmp(name,"new"))
		{
			ret = fffs_create_node(info,&tmp_node, file_mode);
			if(ret != FFFS_ERR_OK)
				return ret;
		}
		else
		{
			//zjistit jestli stejne jmeno neexistuje
			if(fffs_open_named_node(name,info,&tmp_node) == FFFS_ERR_OK)
			{
				return EMFILE;
			}
			else
			{

				ret = fffs_create_named_node(name,info,&tmp_node, file_mode);
				if(ret != FFFS_ERR_OK)
					return ret;
			}

		}
	}
	/*else if(mode & O_TRUNC)
	{
		if(fffs_open_named_node(name,info,&tmp_node) != FFFS_ERR_OK)
		{
			return EMFILE;
		}

		if(fffs_move_node(info,&tmp_node) != FFFS_ERR_OK)
		{
			return EMFILE;
		}
	}*/
	else if((mode & O_RDONLY) || (mode & O_WRONLY ) || (mode & O_RDWR))
	{
		if(!strcmp(name,"first"))
		{
			ret = fffs_get_first_node(info,&tmp_node);
			if(ret != FFFS_ERR_OK)
				return ret;
		}
		else if(!strcmp(name,"next"))
		{
			ret = fffs_get_next_node(info,NULL);
			if(ret != FFFS_ERR_OK)
				return ret;

			fffs_get_current_node(info,&tmp_node);
		}
		else if(!strcmp(name,"last"))
		{
			ret = fffs_get_last_node(info,&tmp_node);
			if(ret != FFFS_ERR_OK)
				return ret;
		}
		else if(!strcmp(name,"prev"))
		{
			ret = fffs_get_prev_node(info,NULL);
			if(ret != FFFS_ERR_OK)
				return ret;

			fffs_get_current_node(info,&tmp_node);
		}
		else if(name[0] == 'n')
		{
			ret = fffs_open_node(info,&tmp_node,atoi(name + 1));
			if(ret != FFFS_ERR_OK)
			return ret;
		}
		else
		{
			ret = fffs_open_named_node(name,info,&tmp_node);
			if(ret != FFFS_ERR_OK)
			return ret;
		}
	}


	node_info = (t_node_info *)malloc(sizeof(t_node_info));
	if(node_info == NULL)
		return EMFILE;
	*node_info = tmp_node;

	file->f_flag   |= mode & CYG_FILE_MODE_MASK;
	file->f_type    = CYG_FILE_TYPE_FILE;
	file->f_ops     = &fffs_fileops;
	file->f_offset  = /*(mode & O_APPEND) ? node->dentry.size :*/ 0;
	file->f_data    = (CYG_ADDRWORD) node_info;
	file->f_xops    = 0;

	return ENOERR;
}

static int fffs_unlink (cyg_mtab_entry *mte, cyg_dir dir, const char *name)
{
	cyg_uint32 ret;
	t_flash_info * disk = (t_flash_info*)mte->data;
	t_node_info tmp_node;
	if(!strcmp(name,"current"))
	{
		fffs_get_current_node(disk,&tmp_node);
		return fffs_delete(disk,&tmp_node);
	}
	else if(!strcmp(name,"last"))
	{
		if((ret = fffs_get_last_node(disk,&tmp_node)) != FFFS_ERR_OK)
			return ret;
		return fffs_delete(disk,&tmp_node);
	}
	else
	{
		ret = fffs_open_named_node(name,disk,&tmp_node);
		if(ret == FFFS_ERR_OK)
			return ret;

		return fffs_delete(disk,&tmp_node);
	}

	//return ENOERR;
}


static int
fffs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	t_flash_info  *disk   = (t_flash_info *) fp->f_mte->data;
	t_node_info    *fd     = (t_node_info *)   fp->f_data;
	cyg_uint32     pos    = fp->f_offset;
	ssize_t        resid  = uio->uio_resid;
	int            i;
	 // Loop over the io vectors until there are none left

	 if(fd == NULL)
		return EINVAL;

	 for (i = 0; i < uio->uio_iovcnt; i++)
	 {
		 cyg_iovec  *iov  = &uio->uio_iov[i];
		 char       *buf  = (char *) iov->iov_base;
		 off_t       len  = iov->iov_len;

		 // Loop over each vector filling it with data from the file

		 while (len > 0 && pos < fd->node.node_length)
		 {
			 cyg_uint32 l = len;
			 int        err;

			 // Adjust size to end of file if necessary
			 if (l > fd->node.node_length-pos)
			 l = fd->node.node_length-pos;

			 err = fffs_read_node(disk,fd,buf,l);
			// err = fatfs_read_data(disk, &node->dentry, &fd->pos, buf, &l);
			 if (err < ENOERR)
			 {
				 errno = err;
				 return err;
			 }

			 // Update working vars

			 len   -= l;
			 buf   += l;
			 pos   += l;
			 resid -= l;
		 }
	 }

	 // We successfully read some data, update the access time,
	 // file offset and transfer residue

	 uio->uio_resid     = resid;
	 fp->f_offset       = (off_t) pos;

	 return ENOERR;
}

static int
fffs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	t_flash_info  *disk   = (t_flash_info *) fp->f_mte->data;
	t_node_info    *fd     = (t_node_info *)   fp->f_data;

	cyg_uint32     pos    = fp->f_offset;
	ssize_t        resid  = uio->uio_resid;
	int            err    = ENOERR;
	int            i;

	CYG_TRACE3(TFO, "write fp=%p uio=%p pos=%d", fp, uio, pos);

	if(fd == NULL)
		return EINVAL;
	// If the APPEND mode bit was supplied, force all writes to
	// the end of the file
	if (fp->f_flag & CYG_FAPPEND)
	{
		return EINVAL;
	}

	// Check that pos is within current file size, or at the very end
	if (pos < 0 || pos > fd->node.node_length)
		return EINVAL;

	// Now loop over the iovecs until they are all done, or we get an error

	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec  *iov  = &uio->uio_iov[i];
		char       *buf  = (char *) iov->iov_base;
		off_t       len  = iov->iov_len;

		// Loop over the vector writing it to the file
		// until it has all been done

		while (len > 0)
		{
			cyg_uint32 l = len;

			err = fffs_write_node(disk,fd,buf,l);
			//err = fatfs_write_data(disk, &node->dentry, &fd->pos, buf, &l);

			// Update working vars

			len   -= l;
			buf   += l;
			pos   += l;
			resid -= l;

			// Stop writing if there is no more space in the file
			if (err == ENOSPC)
			break;

			if (err  < ENOERR)
			{
				errno = err;
				return err;
			}

		}
	}

	// We wrote some data successfully, update the modified and access
	// times of the node, increase its size appropriately, and update
	// the file offset and transfer residue.

	/*node->dentry.mtime =
	node->dentry.atime = cyg_timestamp();*/

	if (pos > fd->node.node_length)
		fd->node.node_length = pos;

	uio->uio_resid = resid;
	fp->f_offset   = (off_t) pos;

	return ENOERR;
}

static int
fffs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *apos, int whence)
{
	//t_flash_info  *disk   = (t_flash_info *) fp->f_mte->data;
	t_node_info    *fd     = (t_node_info *)   fp->f_data;
	off_t          pos   = *apos;
	//int            err;

	CYG_TRACE3(TFO, "lseek fp=%p pos=%d whence=%d", fp, fp->f_offset, whence);

	if(fd == NULL)
		return EINVAL;


	switch (whence)
	{
		case SEEK_SET:
		// Pos is already where we want to be
		break;
		case SEEK_CUR:
		// Add pos to current offset
		pos += fp->f_offset;
		break;
		case SEEK_END:
		// Add pos to file size
		pos += fd->node.node_length;
		break;
		default:
		return EINVAL;
	}

	// Check that pos is still within current file size,
	// or at the very end
	if (pos < 0 || pos > fd->node.node_length)
	return EINVAL;

	// All OK, set fp offset and return new position

	fd->file_cur_pos = pos;
	//err = fatfs_setpos(disk, &fd->node->dentry, &fd->pos, pos);

	/*if (ENOERR == err)
		*apos = fp->f_offset = pos;*/

	CYG_TRACE2(TFO, "lseek fp=%p new pos=%d", fp, *apos);

	return ENOERR;
}


static int
fffs_fo_ioctl(struct CYG_FILE_TAG *fp, CYG_ADDRWORD com, CYG_ADDRWORD data)
{
	t_flash_info  *info   = (t_flash_info *) fp->f_mte->data;
	//t_node_info    *fd     = (t_node_info *)   fp->f_data;
	CYG_TRACE3(TFO, "ioctl fp=%p com=%x data=%x", fp, com, data);

	if(com == FFFS_IO_CTL_COMM_ERRASE)
		return fffs_erase(info,false);

	if(com == FFFS_IO_CTL_COMM_ERRASE_ALL)
		return fffs_erase(info,true);

	if(com == FFFS_IO_CTL_COMM_FORMAT)
		return fffs_format(info,true);

	if(com == FFFS_IO_CTL_COMM_CHECK)
		return fffs_check(info);

	if(com == FFFS_IO_CTL_COMM_TEST_1)
	{
		errno = fffs_test(info,0);
		return errno;
	}

	if(com == FFFS_IO_CTL_COMM_TEST_2)
	{
		errno = fffs_test(info,1);
		return errno;
	}
	if(com == FFFS_IO_CTL_COMM_TEST_3)
	{
		errno = fffs_test(info,2);
		return errno;
	}

	return EINVAL;
}

static int
fffs_fo_fsync(struct CYG_FILE_TAG *fp, int mode)
{

	return ENOERR;
}


static int
fffs_fo_close(struct CYG_FILE_TAG *fp)
{
	t_flash_info  *disk   = (t_flash_info *) fp->f_mte->data;
	t_node_info    *fd     = (t_node_info *)   fp->f_data;

	int            err   = ENOERR;

	CYG_TRACE1(TFO, "close fp=%p", fp);

	if(fd == NULL)
		return err;

	// Write file attributes to disk, unreference
	// the file node and free its private data
    err = fffs_close_node(disk,fd);

	free(fd);
	fp->f_data = NULL;
	errno = err;
	return err;
}

static int
fffs_fo_fstat(struct CYG_FILE_TAG *fp, struct stat *buf)
{
	t_flash_info  *disk   = (t_flash_info *) fp->f_mte->data;
	t_node_info    *fd     = (t_node_info *)   fp->f_data;

	CYG_TRACE2(TFO, "fstat fp=%p buf=%p", fp, buf);

	// Fill in the status
    if(fd != NULL)
	{
		buf->st_mode   = fd->node.mode;
		buf->st_ino    = (ino_t) fd->node.node_id;
		buf->st_dev    = 0;
		buf->st_nlink  = 1;
		buf->st_uid    = 0;
		buf->st_gid    = 0;
		buf->st_size   = fd->node.node_length;
		buf->st_atime  = fd->node.create_time;
		buf->st_mtime  = fd->node.create_time;
		buf->st_ctime  = fd->node.create_time;
	}
	else
	{
		cyg_uint32 ret;
		t_node_info    tmp_node;
		if((ret = fffs_get_first_node(disk,&tmp_node)) != FFFS_ERR_OK)
		{
			errno = ret;
			return ret;
		}

		buf->st_mode   = tmp_node.node.mode;
		buf->st_ino    = (ino_t) fd->node.node_id;
		buf->st_dev    = disk->flash_size;
		buf->st_nlink  = disk->num_files;
		buf->st_uid    = 0;
		buf->st_gid    = 0;
		buf->st_size   = disk->used_space;
		buf->st_atime  = tmp_node.node.create_time;
		buf->st_mtime  = tmp_node.node.create_time;
		buf->st_ctime  = tmp_node.node.create_time;
	}

	return ENOERR;
}

// -------------------------------------------------------------------------
// fatfs_fo_getinfo()
// Get info.

static int
fffs_fo_getinfo(struct CYG_FILE_TAG *fp, int key, void *buf, int len)
{
	CYG_TRACE4(TFO, "getinfo fp=%p key=%d buf=%p len=%d", fp, key, buf, len);
	return EINVAL;
}

// -------------------------------------------------------------------------
// fatfs_fo_setinfo()
// Set info.

static int
fffs_fo_setinfo(struct CYG_FILE_TAG *fp, int key, void *buf, int len)
{
	CYG_TRACE4(TFO, "setinfo fp=%p key=%d buf=%p len=%d", fp, key, buf, len);
	return EINVAL;
}

// -------------------------------------------------------------------------
// fatfs_opendir()
// Open a directory for reading.

static int
fffs_opendir(cyg_mtab_entry *mte,
cyg_dir         dir,
const char     *name,
cyg_file       *file)
{

	return ENOERR;
}



// -------------------------------------------------------------------------
// fatfs_getinfo()
// Getinfo. Support for attrib

static int
fffs_getinfo(cyg_mtab_entry *mte,
cyg_dir         dir,
const char     *name,
int             key,
void           *buf,
int             len)
{
	int err = EINVAL;

	CYG_TRACE6(TFO, "getinfo mte=%p dir=%p name='%s' key=%d buf=%p len=%d",
	mte, dir, name, key, buf, len);

	return err;
}

static int fffs_umount   ( cyg_mtab_entry *mte )
{
    CYG_FAIL("umount function is unimplemented\n");
    return EINVAL;
}

static int fffs_mkdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
    CYG_FAIL("mkdir function is unimplemented\n");
    return EINVAL;
}

static int fffs_rmdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name )
{
    CYG_FAIL("rmdir function is unimplemented\n");
    return EINVAL;
}

static int fffs_rename   ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
                             cyg_dir dir2, const char *name2 )
{
    CYG_FAIL("rename function is unimplemented\n");
    return EINVAL;
}

static int fffs_link     ( cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
                             cyg_dir dir2, const char *name2, int type )
{
    CYG_FAIL("link function is unimplemented\n");
    return EINVAL;
}

static int fffs_chdir    ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
                             cyg_dir *dir_out )
{
    CYG_FAIL("chdir function is unimplemented\n");
    return EINVAL;
}

static int fffs_stat     ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
                             struct stat *buf)
{
    CYG_FAIL("stat function is unimplemented\n");
    return EINVAL;
}

static int fffs_setinfo  ( cyg_mtab_entry *mte, cyg_dir dir, const char *name,
                             int key, void *buf, int len )
{
    // No setinfo keys supported at present
    CYG_FAIL("setinfo function is unimplemented\n");
    return EINVAL;
}

static int fffs_fo_dirread      (struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
    CYG_FAIL("fo_dirread function is unimplemented\n");
    return EINVAL;
}

static int fffs_fo_dirlseek     (struct CYG_FILE_TAG *fp, off_t *pos, int whence )
{
    CYG_FAIL("fo_dirlseek function is unimplemented\n");
    return EINVAL;
}



