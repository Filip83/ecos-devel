// on this file might be covered by the GNU General Public License.         
// -------------------------------------------                              
// ####ECOSGPLCOPYRIGHTEND####                                              
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):           Savin Zlobec <savin@elatec.si> (based on ramfs.c)
// Original data:       nickg
// Date:                2003-06-29
// Purpose:             FAT file system
// Description:         This is a FAT filesystem for eCos. 
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <pkgconf/io_fileio.h>
#include <pkgconf/fs_litte.h>

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

#include <cyg/io/flash.h>
//#include <blib/blib.h>
//#include <cyg/fs/lfs.h>

#include <cyg/fs/lfs.h>
#include <cyg/fs/lfs_util.h>

typedef struct lfs_data_s
{
	lfs_t _lfs; // _the actual filesystem
	struct lfs_config _config;

	// default parameters
	lfs_size_t _read_size;
	lfs_size_t _prog_size;
	lfs_size_t _block_size;
	lfs_size_t _lookahead;
	lfs_size_t _block_count;

	cyg_flashaddr_t  _lfs_flash_base;
}lfs_data_t;

lfs_data_t lfs_data =
{
	._read_size      = CYGNUM_FS_LITTE_READ_SIZE,
	._prog_size      = CYGNUM_FS_LITTE_PROG_SIZE,
	._block_size     = CYGNUM_FS_LITTE_BLOCK_SIZE,
	._lookahead      = CYGNUM_FS_LITTE_LOOKAHEAD,
	._block_count    = CYGNUM_FS_LITTE_BLOCK_COUNT,
	._lfs_flash_base = CYGNUM_FS_LITTE_FLASH_BASE_ADDRESS
};


//==========================================================================
// Forward definitions

int
format(
	lfs_size_t read_size, lfs_size_t prog_size,
	lfs_size_t block_size, lfs_size_t lookahead);

// Filesystem operations
static int _lfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte);
static int _lfs_umount(cyg_mtab_entry *mte);
static int _lfs_open(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	int mode, cyg_file *fte);
static int _lfs_unlink(cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int _lfs_mkdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int _lfs_rmdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int _lfs_rename(cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
	cyg_dir dir2, const char *name2);
static int _lfs_link(cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
	cyg_dir dir2, const char *name2, int type);
static int _lfs_opendir(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	cyg_file *fte);
static int _lfs_chdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	cyg_dir *dir_out);
static int _lfs_stat(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	struct stat *buf);
static int _lfs_getinfo(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	int key, void *buf, int len);
static int _lfs_setinfo(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	int key, void *buf, int len);

// File operations
static int _lfs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int _lfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int _lfs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence);
static int _lfs_fo_ioctl(struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
	CYG_ADDRWORD data);
static int _lfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode);
static int _lfs_fo_close(struct CYG_FILE_TAG *fp);
static int _lfs_fo_fstat(struct CYG_FILE_TAG *fp, struct stat *buf);
static int _lfs_fo_getinfo(struct CYG_FILE_TAG *fp, int key,
	void *buf, int len);
static int _lfs_fo_setinfo(struct CYG_FILE_TAG *fp, int key,
	void *buf, int len);

// Directory operations
static int _lfs_fo_dirread(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int _lfs_fo_dirlseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence);
static int _lfs_fo_dirclose(struct CYG_FILE_TAG *fp);

//==========================================================================
// Filesystem table entries

// -------------------------------------------------------------------------
// Fstab entry.

FSTAB_ENTRY(lfs_fste, "lfs", 0,
	CYG_SYNCMODE_FILE_FILESYSTEM | CYG_SYNCMODE_IO_FILESYSTEM,
	_lfs_mount,
	_lfs_umount,
	_lfs_open,
	_lfs_unlink,
	_lfs_mkdir,
	_lfs_rmdir,
	_lfs_rename,
	_lfs_link,
	_lfs_opendir,
	_lfs_chdir,
	_lfs_stat,
	_lfs_getinfo,
	_lfs_setinfo);

// -------------------------------------------------------------------------
// File operations.

static cyg_fileops lfs_fileops =
{
	_lfs_fo_read,
	_lfs_fo_write,
	_lfs_fo_lseek,
	_lfs_fo_ioctl,
	cyg_fileio_seltrue,
	_lfs_fo_fsync,
	_lfs_fo_close,
	_lfs_fo_fstat,
	_lfs_fo_getinfo,
	_lfs_fo_setinfo
};

// -------------------------------------------------------------------------
// Directory file operations.

static cyg_fileops lfs_dirops =
{
	_lfs_fo_dirread,
	(cyg_fileop_write *)cyg_fileio_enosys,
	_lfs_fo_dirlseek,
	(cyg_fileop_ioctl *)cyg_fileio_enosys,
	cyg_fileio_seltrue,
	(cyg_fileop_fsync *)cyg_fileio_enosys,
	_lfs_fo_dirclose,
	(cyg_fileop_fstat *)cyg_fileio_enosys,
	(cyg_fileop_getinfo *)cyg_fileio_enosys,
	(cyg_fileop_setinfo *)cyg_fileio_enosys
};

// -------------------------------------------------------------------------
////// Conversion functions //////
static int lfs_toerror(int err)
{
	switch (err) {
	case LFS_ERR_OK:
		return 0;
	case LFS_ERR_IO:
		return EIO;
	case LFS_ERR_NOENT:
		return ENOENT;
	case LFS_ERR_EXIST:
		return EEXIST;
	case LFS_ERR_NOTDIR:
		return ENOTDIR;
	case LFS_ERR_ISDIR:
		return EISDIR;
	case LFS_ERR_INVAL:
		return EINVAL;
	case LFS_ERR_NOSPC:
		return ENOSPC;
	case LFS_ERR_NOMEM:
		return ENOMEM;
	case LFS_ERR_CORRUPT:
		return EILSEQ;
	default:
		return err;
	}
}

static int lfs_fromflags(int flags)
{
	return (
		(((flags & 3) == O_RDONLY) ? LFS_O_RDONLY : 0) |
		(((flags & 3) == O_WRONLY) ? LFS_O_WRONLY : 0) |
		(((flags & 3) == O_RDWR) ? LFS_O_RDWR : 0) |
		((flags & O_CREAT) ? LFS_O_CREAT : 0) |
		((flags & O_EXCL) ? LFS_O_EXCL : 0) |
		((flags & O_TRUNC) ? LFS_O_TRUNC : 0) |
		((flags & O_APPEND) ? LFS_O_APPEND : 0));
}

static int lfs_fromwhence(int whence)
{
	switch (whence) {
	case SEEK_SET:
		return LFS_SEEK_SET;
	case SEEK_CUR:
		return LFS_SEEK_CUR;
	case SEEK_END:
		return LFS_SEEK_END;
	default:
		return whence;
	}
}

static int lfs_tomode(int type)
{
	int mode = S_IRWXU | S_IRWXG | S_IRWXO;
	switch (type) {
	case LFS_TYPE_DIR:
		return mode | S_IFDIR;
	case LFS_TYPE_REG:
		return mode | S_IFREG;
	default:
		return 0;
	}
}


////// Block device operations //////
static int lfs_bd_read(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, void *buffer, lfs_size_t size)
{
	cyg_flashaddr_t err_address;
	cyg_flashaddr_t base_addr = *((cyg_flashaddr_t *)c->context);
	return -cyg_flash_read((cyg_flashaddr_t)base_addr + block * c->block_size + off, buffer, size, &err_address);
}

static int lfs_bd_prog(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, const void *buffer, lfs_size_t size)
{
	cyg_flashaddr_t err_address;
	cyg_flashaddr_t base_addr = *((cyg_flashaddr_t *)c->context);
	return -cyg_flash_program((cyg_flashaddr_t)base_addr + block * c->block_size + off, buffer, size, &err_address);
}

static int lfs_bd_erase(const struct lfs_config *c, lfs_block_t block)
{
	cyg_flashaddr_t err_address;
	cyg_flashaddr_t base_addr = *((cyg_flashaddr_t *)c->context);
	return -cyg_flash_erase((cyg_flashaddr_t)base_addr + block * c->block_size,
			c->block_size, &err_address);
}

static int lfs_bd_sync(const struct lfs_config *c)
{
	return 0;
}

// -------------------------------------------------------------------------
// lfs_getinfo()
// Count used blocks

#if defined(CYGSEM_FILEIO_BLOCK_USAGE)
static int lfs_statvfs_count(void *p, lfs_block_t b)
{
    *(lfs_size_t *)p += 1;
    return 0;
}
#endif

// Getinfo. Support for attrib

static int
_lfs_getinfo(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             key,
	void           *buf,
	int             len)
{
	int err = EINVAL;

	CYG_TRACE6(TFS, "getinfo mte=%p dir=%p name='%s' key=%d buf=%p len=%d",
		mte, dir, name, key, buf, len);

    switch( key )
    {
#if defined(CYGSEM_FILEIO_BLOCK_USAGE)
      case FS_INFO_BLOCK_USAGE: {
      lfs_size_t in_use = 0
	  struct cyg_fs_block_usage *usage = (struct cyg_fs_block_usage *) buf;
      lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;

	  err = lfs_traverse(&_lfs_data->_lfs, lfs_statvfs_count, &in_use);
	  if (err)
	    return err;
	  usage->total_blocks = _lfs_data->_config.block_count;
	  usage->free_blocks  = _lfs_data->_config.block_count - in_use;
	  usage->block_size   = _lfs_data->_config.block_size;
	  break;
	}
#endif
        default:
            err = EINVAL;
            break;
    }
    return err;
}

// -------------------------------------------------------------------------
// lfs_setinfo()
// Setinfo. Support for fssync and attrib

static int
_lfs_setinfo(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             key,
	void           *buf,
	int             len)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	int err = EINVAL;

	CYG_TRACE6(TFS, "setinfo mte=%p dir=%p name='%s' key=%d buf=%p len=%d",
		mte, dir, name, key, buf, len);

	switch (key)
	{
	case 120:
		err = format(_lfs_data->_read_size, _lfs_data->_prog_size,
			_lfs_data->_block_size, _lfs_data->_lookahead);
		break;
	default:
		err = EINVAL;
		break;
	}
	return err;
}

// -------------------------------------------------------------------------
// fatfs_rmdir()
// Remove a directory.

static int
_lfs_rmdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name)
{
	return _lfs_unlink(mte, dir, name);
}

//==========================================================================
// Filesystem operations

// -------------------------------------------------------------------------
// lfs_mount()
// Process a mount request. This mainly creates a root for the
// filesystem.

static int
_lfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte)
{
	cyg_flash_info_t info;
	int err;
	
	cyg_flash_get_info_addr(lfs_data._lfs_flash_base, &info);

	memset(&lfs_data._config, 0, sizeof(lfs_data._config));
	lfs_data._config.context = &lfs_data._lfs_flash_base;
	lfs_data._config.read = lfs_bd_read;
	lfs_data._config.prog = lfs_bd_prog;
	lfs_data._config.erase = lfs_bd_erase;
	lfs_data._config.sync = lfs_bd_sync;

	lfs_data._config.read_size = lfs_data._read_size;
	lfs_data._config.prog_size = lfs_data._prog_size;

	if(lfs_data._block_size == 0)
	{
		lfs_data._config.block_size = info.block_info[0].block_size;
		lfs_data._config.block_count = info.block_info[0].blocks;
	}
	else
	{
		lfs_data._config.block_size =  lfs_data._block_size;
		lfs_data._config.block_count = lfs_data._block_count;
	}

	if(lfs_data._lookahead == 0)
		lfs_data._config.lookahead = 32 * ((lfs_data._config.block_count + 31) / 32);
	else
		lfs_data._config.lookahead = lfs_data._lookahead;

	err = lfs_mount(&lfs_data._lfs, &lfs_data._config);
	mte->data = (CYG_ADDRWORD)&lfs_data;
	if (err) {
		LFS_INFO("mount -> %d", lfs_toerror(err));
		return lfs_toerror(err);
	}
	LFS_INFO("mount -> %d", 0);
	return 0;
}

// -------------------------------------------------------------------------
// lfs_umount()
// Unmount the filesystem. This will currently only succeed if the
// filesystem is empty.

static int
_lfs_umount(cyg_mtab_entry *mte)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	LFS_INFO("unmount(%s)", "");
	int res = 0;
	if (_lfs_data) {
		int err = lfs_unmount(&_lfs_data->_lfs);
		if (err && !res) {
			res = lfs_toerror(err);
		}
	}

	LFS_INFO("unmount -> %d", res);

	mte->root = CYG_DIR_NULL;
	mte->data = (CYG_ADDRWORD)NULL;

	CYG_TRACE0(TFS, "disk umounted");

	return res;
}

int
format(
	lfs_size_t read_size, lfs_size_t prog_size,
	lfs_size_t block_size, lfs_size_t lookahead)
{
	//lfs_t _lfs;
	int   err;
	//struct lfs_config _config;
	cyg_flash_info_t info;

	cyg_flash_get_info_addr(lfs_data._lfs_flash_base, &info)

	LFS_INFO("format(%ld, %ld, %ld, %ld)",
		read_size, prog_size, block_size, lookahead);

	memset(&lfs_data._config, 0, sizeof(lfs_data._config));
	lfs_data._config.context = &lfs_data._lfs_flash_base;
	lfs_data._config.read = lfs_bd_read;
	lfs_data._config.prog = lfs_bd_prog;
	lfs_data._config.erase = lfs_bd_erase;
	lfs_data._config.sync = lfs_bd_sync;

	lfs_data._config.read_size = lfs_data._read_size;
	lfs_data._config.prog_size = lfs_data._prog_size;

	if(lfs_data._block_size == 0)
	{
		lfs_data._config.block_size = info.block_info[0].block_size;
		lfs_data._config.block_count = info.block_info[0].blocks;
	}
	else
	{
		lfs_data._config.block_size =  lfs_data._block_size;
		lfs_data._config.block_count = lfs_data._block_count;
	}

	if(lfs_data._lookahead == 0)
		lfs_data._config.lookahead = 32 * ((lfs_data._config.block_count + 31) / 32);
	else
		lfs_data._config.lookahead = lfs_data._lookahead;


	err = lfs_format(&lfs_data._lfs, &lfs_data._config);
	if (err) {
		LFS_INFO("format -> %d", lfs_toerror(err));
		return lfs_toerror(err);
	}

	LFS_INFO("format -> %d", 0);
	return 0;
}

// -------------------------------------------------------------------------
// lfs_unlink()
// Remove a file link from its directory.

static int
_lfs_unlink(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	LFS_INFO("remove(\"%s\")", name);
	int err = lfs_remove(&_lfs_data->_lfs, name);
	LFS_INFO("remove -> %d", lfs_toerror(err));
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_rename()
// Rename a file/dir.

static int
_lfs_rename(cyg_mtab_entry *mte,
	cyg_dir         dir1,
	const char     *name1,
	cyg_dir         dir2,
	const char     *name2)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	LFS_INFO("rename(\"%s\", \"%s\")", name1, name2);
	int err = lfs_rename(&_lfs_data->_lfs, name1, name2);
	LFS_INFO("rename -> %d", lfs_toerror(err));
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_mkdir()
// Create a new directory.

static int
_lfs_mkdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	LFS_INFO("mkdir(\"%s\", 0x%lx)", name, mode);
	int err = lfs_mkdir(&_lfs_data->_lfs, name);
	LFS_INFO("mkdir -> %d", lfs_toerror(err));
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_stat()
// Get struct stat info for named object.

static int
_lfs_stat(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	struct stat    *buf)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	struct lfs_info info;
	LFS_INFO("stat(\"%s\", %p)", name, st);
	int err = lfs_stat(&_lfs_data->_lfs, name, &info);
	LFS_INFO("stat -> %d", lfs_toerror(err));

	buf->st_mode = lfs_tomode(info.type);
	buf->st_dev = 0;
	buf->st_nlink = 1;
	buf->st_uid = 0;
	buf->st_gid = 0;
	buf->st_size = info.size;
	buf->st_atime = 0;
	buf->st_mtime = 0;
	buf->st_ctime = 0;

	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_chdir()
// Change directory support.

//static  char dir_path[PATH_MAX];

static int
_lfs_chdir(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	cyg_dir        *dir_out)
{
	//lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;

    CYG_TRACE4(TFS, "chdir mte=%p dir=%p dir_out=%p name=%d",
                    mte, dir, dir_out, name);

    return EINVAL;
}

// -------------------------------------------------------------------------
// lfs_open()
// Open a file for reading or writing.

static int
_lfs_open(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             mode,
	cyg_file       *file)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;

	lfs_file_t *f = (lfs_file_t*)malloc(sizeof(lfs_file_t));
	if(f == NULL)
		return -ENOMEM;

	LFS_INFO("file_open(%p, \"%s\", 0x%x)", file, name, mode);
	int err = lfs_file_open(&_lfs_data->_lfs, f, name, lfs_fromflags(mode));
	LFS_INFO("file_open -> %d", lfs_toerror(err));
	if (!err) {
		file->f_flag |= mode & CYG_FILE_MODE_MASK;
		file->f_type = CYG_FILE_TYPE_FILE;
		file->f_ops = &lfs_fileops;
		file->f_offset = (mode & O_APPEND) ? f->pos : 0;
		file->f_data = (CYG_ADDRWORD)f;
		file->f_xops = 0;
	}
	else {
		free(f);
	}
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_fo_close()
// Close a file.

static int
_lfs_fo_close(struct CYG_FILE_TAG *fp)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;;
	LFS_INFO("file_close(%p)", f);
	int err = lfs_file_close(&_lfs_data->_lfs, f);
	LFS_INFO("file_close -> %d", lfs_toerror(err));
	free(f);
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_fo_read()
// Read data from the file.

static int
_lfs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	//cyg_uint32     pos = fp->f_offset;
	ssize_t        resid = uio->uio_resid;
	int            i;
	lfs_ssize_t   res = 0;

	CYG_TRACE3(TFO, "read fp=%p uio=%p pos=%d", fp, uio, fp->f_offset);

	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec  *iov = &uio->uio_iov[i];
		char       *buf = (char *)iov->iov_base;
		off_t       len = iov->iov_len;

		res = lfs_file_read(&_lfs_data->_lfs, f, buf, len);
		LFS_INFO("file_read -> %d", lfs_toerror(res));
		if (res < 0)
		{
			return lfs_toerror(res);
		}

		resid -= res;
	}

	LFS_INFO("file_read -> %d", lfs_toerror(res));
	uio->uio_resid = resid;
	fp->f_offset = (off_t)f->pos;
	return ENOERR;
}

// -------------------------------------------------------------------------
// lfs_fo_write()
// Write data to file.

static int
_lfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	//cyg_uint32     pos = fp->f_offset;
	ssize_t        resid = uio->uio_resid;
	lfs_ssize_t    res = 0;
	int            i;

	CYG_TRACE3(TFO, "write fp=%p uio=%p pos=%d", fp, uio, fp->f_offset);

	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec  *iov = &uio->uio_iov[i];
		char       *buf = (char *)iov->iov_base;
		off_t       len = iov->iov_len;

		res = lfs_file_write(&_lfs_data->_lfs, f, buf, len);
		LFS_INFO("file_write -> %d", lfs_toerror(res));
		if (res == LFS_ERR_NOSPC)
			break;

		if (res < 0)
		{
			return lfs_toerror(res);
		}

		resid -= res;
	}

	uio->uio_resid = resid;
	fp->f_offset = (off_t)f->pos;

	return ENOERR;
}

// -------------------------------------------------------------------------
// lfs_fo_fsync().
// Force the file out to data storage.

static int
_lfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	int            err;

	CYG_TRACE2(TFO, "fsync fp=%p mode=%d", fp, mode);

	err = lfs_file_sync(&_lfs_data->_lfs, f);
	LFS_INFO("file_sync -> %d", lfs_toerror(err));

	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// lfs_fo_lseek()
// Seek to a new file position.

static int
_lfs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *apos, int whence)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	off_t          pos = *apos;
	off_t          res;

	CYG_TRACE3(TFO, "lseek fp=%p pos=%d whence=%d", fp, fp->f_offset, whence);

	res = lfs_file_seek(&_lfs_data->_lfs, f, pos, lfs_fromwhence(whence));
	LFS_INFO("file_seek -> %d", lfs_toerror(res));
	if(res < 0)
		return lfs_toerror(res);

	*apos = fp->f_offset = f->pos; // new position
	return ENOERR;
}

// -------------------------------------------------------------------------
// lfs_fo_fstat()
// Get file status.

static int
_lfs_fo_fstat(struct CYG_FILE_TAG *fp, struct stat *buf)
{
	//lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;

	CYG_TRACE2(TFO, "fstat fp=%p buf=%p", fp, buf);

	// Fill in the status

	buf->st_mode = f->flags;
	buf->st_ino = (ino_t)0;
	buf->st_dev = 0;
	buf->st_nlink = 1;
	buf->st_uid = 0;
	buf->st_gid = 0;
	buf->st_size = f->size;
	buf->st_atime = 0;
	buf->st_mtime = 0;
	buf->st_ctime = 0;

	return ENOERR;
}


// -------------------------------------------------------------------------
// lfs_fo_ioctl()
// Handle ioctls. Currently none are defined.

static int
_lfs_fo_ioctl(struct CYG_FILE_TAG *fp, CYG_ADDRWORD com, CYG_ADDRWORD data)
{
	CYG_TRACE3(TFO, "ioctl fp=%p com=%x data=%x", fp, com, data);
	return EINVAL;
}

// -------------------------------------------------------------------------
// lfs_fo_getinfo()
// Get info.

static int
_lfs_fo_getinfo(struct CYG_FILE_TAG *fp, int key, void *buf, int len)
{
	CYG_TRACE4(TFO, "getinfo fp=%p key=%d buf=%p len=%d", fp, key, buf, len);
	return EINVAL;
}

// -------------------------------------------------------------------------
// lfs_fo_setinfo()
// Set info.

static int
_lfs_fo_setinfo(struct CYG_FILE_TAG *fp, int key, void *buf, int len)
{
	CYG_TRACE4(TFO, "setinfo fp=%p key=%d buf=%p len=%d", fp, key, buf, len);
	return EINVAL;
}

//==========================================================================
// Directory operations

// -------------------------------------------------------------------------
// lfs_link()
// Make a new directory entry for a file.

static int
_lfs_link(cyg_mtab_entry *mte,
	cyg_dir         dir1,
	const char     *name1,
	cyg_dir         dir2,
	const char     *name2,
	int             type)
{
	CYG_TRACE6(TFS, "link mte=%p dir1=%p name1='%s' dir2=%p name2='%s' type=%d",
		mte, dir1, name1, dir2, name2, type);

	// Linking not supported
	return EINVAL;
}

// -------------------------------------------------------------------------
// lfs_opendir()
// Open a directory for reading.

static int
_lfs_opendir(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	cyg_file       *file)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	int                 err;

	lfs_dir_t *d = (lfs_dir_t*)malloc(sizeof(lfs_dir_t));
	if(d == NULL)
		return -ENOMEM;

	CYG_TRACE4(TFS, "opendir mte=%p dir=%p name='%s' file=%p",
		mte, dir, name, file);

	LFS_INFO("dir_open(%p, \"%s\")", d, name);
	err = lfs_dir_open(&_lfs_data->_lfs, d, name);
	LFS_INFO("dir_open -> %d", lfs_toerror(err));
	if (!err) {
		file->f_type = CYG_FILE_TYPE_FILE;
		file->f_ops = &lfs_dirops;
		file->f_data = (CYG_ADDRWORD)d;
		file->f_xops = 0;
		file->f_offset = 0;
	}
	else {
		free(d);
	}
	return lfs_toerror(err);

}
// -------------------------------------------------------------------------
// lfs_dir_close()
// Close a directorz.

static int
_lfs_fo_dirclose(struct CYG_FILE_TAG *fp)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_dir_t *d = (lfs_dir_t *)fp->f_data;

	int            err = ENOERR;

	CYG_TRACE1(TFO, "close fp=%p", fp);

	err = lfs_dir_close(&_lfs_data->_lfs, d);
	LFS_INFO("dir_close -> %d", lfs_toerror(err));

	free(d);
	fp->f_data = NULL;

	return lfs_toerror(err);;
}

// -------------------------------------------------------------------------
// lfs_fo_dirread()
// Read a single directory entry from a file.

static int
_lfs_fo_dirread(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_dir_t *d = (lfs_dir_t *)fp->f_data;
	struct dirent     *ent = (struct dirent *) uio->uio_iov[0].iov_base;
	char              *nbuf = ent->d_name;
	off_t              len = uio->uio_iov[0].iov_len;

	struct lfs_info info;
	
	CYG_TRACE3(TFO, "dirread fp=%p uio=%p pos=%d", fp, uio, fp->f_offset);

	if (len < sizeof(struct dirent))
		return EINVAL;

	int res = lfs_dir_read(&_lfs_data->_lfs, d, &info);
	LFS_INFO("dir_read -> %d", lfs_toerror(res));
	
	if (res == 1) {
		strcpy(nbuf, info.name);

		uio->uio_resid         -= sizeof(struct dirent);
		fp->f_offset++;

		return ENOERR;
	}

	return lfs_toerror(res);
}

// -------------------------------------------------------------------------
// lfs_fo_dirlseek()
// Seek directory to start.

static int
_lfs_fo_dirlseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence)
{
	lfs_data_t  *_lfs_data = (lfs_data_t *)fp->f_mte->data;
	lfs_dir_t *d = (lfs_dir_t *)fp->f_data;
	//int            err;

	CYG_TRACE2(TFO, "dirlseek fp=%p whence=%d", fp, whence);

	// Only allow SEEK_SET to zero

	if (whence != SEEK_SET || *pos != 0)
		return EINVAL;

	LFS_INFO("dir_rewind(%p)", dir);
	lfs_dir_rewind(&_lfs_data->_lfs, d);
	LFS_INFO("dir_rewind -> %s", "void");

	return ENOERR;
}
