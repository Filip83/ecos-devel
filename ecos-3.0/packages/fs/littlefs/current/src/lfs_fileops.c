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
#include <pkgconf/fs_lfs.h>

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
#include <blib/blib.h>
#include <cyg/fs/lfs.h>

#include "lfs.h"
#include "lfs_util.h"

typedef struct lfs_data_s
{
	lfs_t _lfs; // _the actual filesystem
	struct lfs_config _config;
	//mbed::BlockDevice *_bd; // the block device

	// default parameters
	lfs_size_t _read_size;
	lfs_size_t _prog_size;
	lfs_size_t _block_size;
	lfs_size_t _lookahead;

	cyg_mutex _mutex;
}lfs_data_t;

lfs_data_t lfs_data;

//==========================================================================
// Tracing support defines 

#ifdef LFS_TRACE_FS_OP
# define TFS 1
#else
# define TFS 0
#endif

#ifdef LFS_TRACE_FILE_OP
# define TFO 1
#else
# define TFO 0
#endif

//==========================================================================
// Forward definitions

// Filesystem operations
static int lfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte);
static int lfs_umount(cyg_mtab_entry *mte);
static int lfs_open(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	int mode, cyg_file *fte);
static int lfs_unlink(cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int lfs_mkdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int lfs_rmdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name);
static int lfs_rename(cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
	cyg_dir dir2, const char *name2);
static int lfs_link(cyg_mtab_entry *mte, cyg_dir dir1, const char *name1,
	cyg_dir dir2, const char *name2, int type);
static int lfs_opendir(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	cyg_file *fte);
static int lfs_chdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	cyg_dir *dir_out);
static int lfs_stat(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	struct stat *buf);
static int lfs_getinfo(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	int key, void *buf, int len);
static int lfs_setinfo(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
	int key, void *buf, int len);

// File operations
static int lfs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int lfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int lfs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence);
static int lfs_fo_ioctl(struct CYG_FILE_TAG *fp, CYG_ADDRWORD com,
	CYG_ADDRWORD data);
static int lfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode);
static int lfs_fo_close(struct CYG_FILE_TAG *fp);
static int lfs_fo_fstat(struct CYG_FILE_TAG *fp, struct stat *buf);
static int lfs_fo_getinfo(struct CYG_FILE_TAG *fp, int key,
	void *buf, int len);
static int lfs_fo_setinfo(struct CYG_FILE_TAG *fp, int key,
	void *buf, int len);

// Directory operations
static int lfs_fo_dirread(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int lfs_fo_dirlseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence);

//==========================================================================
// Filesystem table entries

// -------------------------------------------------------------------------
// Fstab entry.

FSTAB_ENTRY(lfs_fste, "lfs", 0,
	CYG_SYNCMODE_FILE_FILESYSTEM | CYG_SYNCMODE_IO_FILESYSTEM,
	lfs_mount,
	lfs_umount,
	lfs_open,
	lfs_unlink,
	lfs_mkdir,
	lfs_rmdir,
	lfs_rename,
	lfs_link,
	lfs_opendir,
	lfs_chdir,
	lfs_stat,
	lfs_getinfo,
	lfs_setinfo);

// -------------------------------------------------------------------------
// File operations.

static cyg_fileops lfs_fileops =
{
	lfs_fo_read,
	lfs_fo_write,
	lfs_fo_lseek,
	lfs_fo_ioctl,
	cyg_fileio_seltrue,
	lfs_fo_fsync,
	lfs_fo_close,
	lfs_fo_fstat,
	lfs_fo_getinfo,
	lfs_fo_setinfo
};

// -------------------------------------------------------------------------
// Directory file operations.

static cyg_fileops lfs_dirops =
{
	lfs_fo_dirread,
	(cyg_fileop_write *)cyg_fileio_enosys,
	lfs_fo_dirlseek,
	(cyg_fileop_ioctl *)cyg_fileio_enosys,
	cyg_fileio_seltrue,
	(cyg_fileop_fsync *)cyg_fileio_enosys,
	lfs_fo_close,
	(cyg_fileop_fstat *)cyg_fileio_enosys,
	(cyg_fileop_getinfo *)cyg_fileio_enosys,
	(cyg_fileop_setinfo *)cyg_fileio_enosys
};

// -------------------------------------------------------------------------

extern "C" void lfs_crc(uint32_t *crc, const void *buffer, size_t size)
{
	uint32_t initial_xor = *crc;
	MbedCRC<POLY_32BIT_REV_ANSI, 32> ct(initial_xor, 0x0, true, false);
	ct.compute((void *)buffer, size, (uint32_t *)crc);
}

////// Conversion functions //////
static int lfs_toerror(int err)
{
	switch (err) {
	case LFS_ERR_OK:
		return 0;
	case LFS_ERR_IO:
		return -EIO;
	case LFS_ERR_NOENT:
		return -ENOENT;
	case LFS_ERR_EXIST:
		return -EEXIST;
	case LFS_ERR_NOTDIR:
		return -ENOTDIR;
	case LFS_ERR_ISDIR:
		return -EISDIR;
	case LFS_ERR_INVAL:
		return -EINVAL;
	case LFS_ERR_NOSPC:
		return -ENOSPC;
	case LFS_ERR_NOMEM:
		return -ENOMEM;
	case LFS_ERR_CORRUPT:
		return -EILSEQ;
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

static int lfs_totype(int type)
{
	switch (type) {
	case LFS_TYPE_DIR:
		return DT_DIR;
	case LFS_TYPE_REG:
		return DT_REG;
	default:
		return DT_UNKNOWN;
	}
}


////// Block device operations //////
static int lfs_bd_read(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, void *buffer, lfs_size_t size)
{
	BlockDevice *bd = (BlockDevice *)c->context;
	return bd->read(buffer, (bd_addr_t)block * c->block_size + off, size);
}

static int lfs_bd_prog(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, const void *buffer, lfs_size_t size)
{
	BlockDevice *bd = (BlockDevice *)c->context;
	return bd->program(buffer, (bd_addr_t)block * c->block_size + off, size);
}

static int lfs_bd_erase(const struct lfs_config *c, lfs_block_t block)
{
	BlockDevice *bd = (BlockDevice *)c->context;
	return bd->erase((bd_addr_t)block * c->block_size, c->block_size);
}

static int lfs_bd_sync(const struct lfs_config *c)
{
	BlockDevice *bd = (BlockDevice *)c->context;
	return bd->sync();
}

// -------------------------------------------------------------------------
// fatfs_getinfo()
// Getinfo. Support for attrib

static int
fatfs_getinfo(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             key,
	void           *buf,
	int             len)
{
	int err = EINVAL;

	CYG_TRACE6(TFS, "getinfo mte=%p dir=%p name='%s' key=%d buf=%p len=%d",
		mte, dir, name, key, buf, len);
	switch (key)
	{
#ifdef CYGCFG_FS_FAT_USE_ATTRIBUTES
	case FS_INFO_ATTRIB:
		err = fatfs_get_attrib(mte, dir, name, (cyg_fs_attrib_t*)buf);
		break;
#endif // CYGCFG_FS_FAT_USE_ATTRIBUTES
#if defined(CYGSEM_FILEIO_BLOCK_USAGE)
	case FS_INFO_BLOCK_USAGE: {
		cyg_uint32 total_clusters;
		cyg_uint32 free_clusters;
		struct cyg_fs_block_usage *usage = (struct cyg_fs_block_usage *) buf;
		fatfs_disk_t  *disk = (fatfs_disk_t *)mte->data;

		err = fatfs_get_disk_usage(disk, &total_clusters, &free_clusters);
		if (err)
			return err;
		usage->total_blocks = total_clusters;
		usage->free_blocks = free_clusters;
		usage->block_size = disk->cluster_size;
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
// fatfs_setinfo()
// Setinfo. Support for fssync and attrib

static int
fatfs_setinfo(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             key,
	void           *buf,
	int             len)
{
	int err = EINVAL;

	CYG_TRACE6(TFS, "setinfo mte=%p dir=%p name='%s' key=%d buf=%p len=%d",
		mte, dir, name, key, buf, len);

	switch (key)
	{
	case FS_INFO_SYNC:
		err = cyg_blib_sync(&(((fatfs_disk_t *)mte->data)->blib));
		break;
#ifdef CYGCFG_FS_FAT_USE_ATTRIBUTES
	case FS_INFO_ATTRIB:
		err = fatfs_set_attrib(mte, dir, name, *(cyg_fs_attrib_t *)buf);
		break;
#endif // CYGCFG_FS_FAT_USE_ATTRIBUTES
	default:
		err = EINVAL;
		break;
	}
	return err;
}

//==========================================================================
// Filesystem operations

// -------------------------------------------------------------------------
// fatfs_mount()
// Process a mount request. This mainly creates a root for the
// filesystem.

static int
fatfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte)
{
	//_mutex.lock();
	/*LFS_INFO("mount(%p)", bd);
	_bd = bd;
	int err = lfs_data._bd->init();
	if (err) {
		lfs_data._bd = NULL;
		LFS_INFO("mount -> %d", err);
		//_mutex.unlock();
		return err;
	}*/

	cyg_flash_info_t info;
	cyg_flashaddr_t lfs_flash_base;
	cyg_flash_get_info_addr(lfs_flash_base, &info)

	memset(&lfs_data._config, 0, sizeof(lfs_data._config));
	lfs_data._config.context = NULL;
	lfs_data._config.read = lfs_bd_read;
	lfs_data._config.prog = lfs_bd_prog;
	lfs_data._config.erase = lfs_bd_erase;
	lfs_data._config.sync = lfs_bd_sync;
	lfs_data._config.read_size = sizeof(char); //TODO: Neco lepsiho mozna
	if (lfs_data._config.read_size < _read_size) {
		lfs_data._config.read_size = _read_size;
	}
	lfs_data._config.prog_size = sizeof(char); //TODO: Neco lepsiho mozna
	if (lfs_data._config.prog_size < _prog_size) {
		lfs_data._config.prog_size = _prog_size;
	}
	lfs_data._config.block_size = info.block_info[0].block_size;
	if (lfs_data._config.block_size < _block_size) {
		lfs_data._config.block_size = _block_size;
	}
	lfs_data._config.block_count = info.block_info[0].blocks;
	lfs_data._config.lookahead = 32 * ((lfs_data._config.block_count + 31) / 32);
	if (lfs_data._config.lookahead > _lookahead) {
		lfs_data._config.lookahead = _lookahead;
	}

	err = lfs_mount(&lfs_data._lfs, &lfs_data._config);
	mte->data = (CYG_ADDRWORD)lfs_data;
	if (err) {
		//lfs_data._bd = NULL;
		LFS_INFO("mount -> %d", lfs_toerror(err));
		//_mutex.unlock();
		return lfs_toerror(err);
	}

	//_mutex.unlock();
	LFS_INFO("mount -> %d", 0);
	return 0;
}

// -------------------------------------------------------------------------
// fatfs_umount()
// Unmount the filesystem. This will currently only succeed if the
// filesystem is empty.

static int
fatfs_umount(cyg_mtab_entry *mte)
{
	//_mutex.lock();
	lfs_data_t  *_lfs_data = (lfs_data_t *)mte->data;
	LFS_INFO("unmount(%s)", "");
	int res = 0;
	//if (_bd) {
		int err = lfs_unmount(lfs_data->_lfs);
		if (err && !res) {
			res = lfs_toerror(err);
		}

		/*err = lfs_data._bd->deinit();
		if (err && !res) {
			res = err;
		}
		
		lfs_data._bd = NULL;*/
	}

	LFS_INFO("unmount -> %d", res);
	//_mutex.unlock();
	return res;
}

static int 
format(BlockDevice *bd,
	lfs_size_t read_size, lfs_size_t prog_size,
	lfs_size_t block_size, lfs_size_t lookahead)
{
	LFS_INFO("format(%p, %ld, %ld, %ld, %ld)",
		bd, read_size, prog_size, block_size, lookahead);
	int err = lfs_data.bd->init();
	if (err) {
		LFS_INFO("format -> %d", err);
		return err;
	}

	lfs_t _lfs;
	struct lfs_config _config;

	memset(&lfs_data._config, 0, sizeof(lfs_data._config));
	lfs_data._config.context = bd;
	lfs_data._config.read = lfs_bd_read;
	lfs_data._config.prog = lfs_bd_prog;
	lfs_data._config.erase = lfs_bd_erase;
	lfs_data._config.sync = lfs_bd_sync;
	lfs_data._config.read_size = lfs_data.bd->get_read_size();
	if (lfs_data._config.read_size < read_size) {
		lfs_data._config.read_size = read_size;
	}
	lfs_data._config.prog_size = bd->get_program_size();
	if (lfs_data._config.prog_size < prog_size) {
		lfs_data._config.prog_size = prog_size;
	}
	lfs_data._config.block_size = lfs_data.bd->get_erase_size();
	if (lfs_data._config.block_size < block_size) {
		lfs_data._config.block_size = block_size;
	}
	lfs_data._config.block_count = bd->size() / lfs_data._config.block_size;
	lfs_data._config.lookahead = 32 * ((lfs_data._config.block_count + 31) / 32);
	if (lfs_data._config.lookahead > lookahead) {
		lfs_data._config.lookahead = lookahead;
	}

	err = lfs_format(&lfs_data._lfs, &lfs_data._config);
	if (err) {
		LFS_INFO("format -> %d", lfs_toerror(err));
		return lfs_toerror(err);
	}

	err = lfs_data.bd->deinit();
	if (err) {
		LFS_INFO("format -> %d", err);
		return err;
	}

	LFS_INFO("format -> %d", 0);
	return 0;
}

static int
reformat(BlockDevice *bd)
{
	//_mutex.lock();
	///LFS_INFO("reformat(%p)", bd);
	//if (_bd) {
		/*if (!bd) {
			bd = lfs_data._bd;
		}
		*/
		int err = unmount();
		if (err) {
			LFS_INFO("reformat -> %d", err);
			//_mutex.unlock();
			return err;
		}
	}

	/*if (!bd) {
		LFS_INFO("reformat -> %d", -ENODEV);
		//_mutex.unlock();
		return -ENODEV;
	}*/

	int err = format(bd,
		lfs_data._read_size, lfs_data._prog_size, lfs_data._block_size, lfs_data._lookahead);
	if (err) {
		LFS_INFO("reformat -> %d", err);
		//_mutex.unlock();
		return err;
	}

	err = mount(bd);
	if (err) {
		LFS_INFO("reformat -> %d", err);
		//_mutex.unlock();
		return err;
	}

	LFS_INFO("reformat -> %d", 0);
	//_mutex.unlock();
	return 0;
}

// -------------------------------------------------------------------------
// fatfs_unlink()
// Remove a file link from its directory.

static int
fatfs_unlink(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name)
{
	//_mutex.lock();
	LFS_INFO("remove(\"%s\")", name);
	int err = lfs_remove(&lfs_data._lfs, name);
	LFS_INFO("remove -> %d", lfs_toerror(err));
	//_mutex.unlock();
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// fatfs_rename()
// Rename a file/dir.

static int
fatfs_rename(cyg_mtab_entry *mte,
	cyg_dir         dir1,
	const char     *name1,
	cyg_dir         dir2,
	const char     *name2)
{
	//_mutex.lock();
	LFS_INFO("rename(\"%s\", \"%s\")", name1, name2);
	int err = lfs_rename(&lfs_data._lfs, name1, name2);
	LFS_INFO("rename -> %d", lfs_toerror(err));
	//_mutex.unlock();
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// fatfs_mkdir()
// Create a new directory.

static int
fatfs_mkdir(cyg_mtab_entry *mte, cyg_dir dir, const char *name)
{
	//_mutex.lock();
	LFS_INFO("mkdir(\"%s\", 0x%lx)", name, mode);
	int err = lfs_mkdir(&lfs_data._lfs, name);
	LFS_INFO("mkdir -> %d", lfs_toerror(err));
	//_mutex.unlock();
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// fatfs_stat()
// Get struct stat info for named object.

static int
fatfs_stat(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	struct stat    *buf)
{
	struct lfs_info info;
	//_mutex.lock();
	LFS_INFO("stat(\"%s\", %p)", name, st);
	int err = lfs_stat(&lfs_data.__lfs, name, &info);
	LFS_INFO("stat -> %d", lfs_toerror(err));
	//_mutex.unlock();

	buf->st_mode = lfs_tomode(info.type);
	//buf->st_ino = (ino_t)ds.node->dentry.cluster;
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
// fatfs_open()
// Open a file for reading or writing.

static int
fatfs_open(cyg_mtab_entry *mte,
	cyg_dir         dir,
	const char     *name,
	int             mode,
	cyg_file       *file)
{
	lfs_file_t *f = new lfs_file_t;
	//_mutex.lock();
	LFS_INFO("file_open(%p, \"%s\", 0x%x)", *file, name, mode);
	int err = lfs_file_open(&lfs_data._lfs, f, name, lfs_fromflags(mode));
	LFS_INFO("file_open -> %d", lfs_toerror(err));
	//_mutex.unlock();
	if (!err) {
		//*file = f;
		file->f_flag |= mode & CYG_FILE_MODE_MASK;
		file->f_type = CYG_FILE_TYPE_FILE;
		file->f_ops = &fatfs_fileops;
		file->f_offset = (mode & O_APPEND) ? f->pos : 0;
		file->f_data = (CYG_ADDRWORD)f;
		file->f_xops = 0;
	}
	else {
		delete f;
	}
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// fatfs_fo_close()
// Close a file.

static int
fatfs_fo_close(struct CYG_FILE_TAG *fp)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;;
	//_mutex.lock();
	LFS_INFO("file_close(%p)", f);
	int err = lfs_file_close(&lfs_data._lfs, f);
	LFS_INFO("file_close -> %d", lfs_toerror(err));
	//_mutex.unlock();
	delete f;
	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// fatfs_fo_read()
// Read data from the file.

static int
fatfs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	cyg_uint32     pos = fp->f_offset;
	ssize_t        resid = uio->uio_resid;
	int            i;

	//_mutex.lock();
	CYG_TRACE3(TFO, "read fp=%p uio=%p pos=%d", fp, uio, pos);

	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec  *iov = &uio->uio_iov[i];
		char       *buf = (char *)iov->iov_base;
		off_t       len = iov->iov_len;

		lfs_ssize_t res = lfs_file_read(&lfs_data._lfs, f, buf, len);

		// Update working vars

		len -= l;
		buf += l;
		pos += l;
		resid -= l;
	}

	LFS_INFO("file_read -> %d", lfs_toerror(res));
	uio->uio_resid = resid;
	fp->f_offset = (off_t)pos;
	//_mutex.unlock();
	return lfs_toerror(res);
}

// -------------------------------------------------------------------------
// fatfs_fo_write()
// Write data to file.

static int
fatfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	cyg_uint32     pos = fp->f_offset;
	ssize_t        resid = uio->uio_resid;
	int            err = ENOERR;
	int            i;

	CYG_TRACE3(TFO, "write fp=%p uio=%p pos=%d", fp, uio, pos);

	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec  *iov = &uio->uio_iov[i];
		char       *buf = (char *)iov->iov_base;
		off_t       len = iov->iov_len;

		// Loop over the vector writing it to the file 
		// until it has all been done

		lfs_ssize_t res = lfs_file_write(&lfs_data._lfs, f, buf, len);
		LFS_INFO("file_write -> %d", lfs_toerror(res));

	}

	// We wrote some data successfully, update the modified and access
	// times of the node, increase its size appropriately, and update
	// the file offset and transfer residue.



	if (pos > node->dentry.size)
		node->dentry.size = pos;

	uio->uio_resid = resid;
	fp->f_offset = (off_t)pos;

	return lfs_toerror(res);
}

// -------------------------------------------------------------------------
// fatfs_fo_fsync().
// Force the file out to data storage.

static int
fatfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	int            err;

	CYG_TRACE2(TFO, "fsync fp=%p mode=%d", fp, mode);

	int err = lfs_file_sync(&lfs_data._lfs, f);
	LFS_INFO("file_sync -> %d", lfs_toerror(err));

	return lfs_toerror(err);
}

// -------------------------------------------------------------------------
// fatfs_fo_lseek()
// Seek to a new file position.

static int
fatfs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *apos, int whence)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	lfs_file_t *f = (lfs_file_t *)fp->f_data;
	int            err;

	CYG_TRACE3(TFO, "lseek fp=%p pos=%d whence=%d", fp, fp->f_offset, whence);

	off_t res = lfs_file_seek(&lfs_data._lfs, f, *apos, lfs_fromwhence(whence));
	LFS_INFO("file_seek -> %d", lfs_toerror(res));

	return lfs_toerror(res);
}

//==========================================================================
// Directory operations


// -------------------------------------------------------------------------
// fatfs_dir_close()
// Close a directorz.

static int
fatfs_dir_close(struct CYG_FILE_TAG *fp)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	fs_dir_t *d = (fs_dir_t *)fp->f_data;

	int            err = ENOERR;

	CYG_TRACE1(TFO, "close fp=%p", fp);

	err = lfs_dir_close(&lfs_data._lfs, d);
	LFS_INFO("dir_close -> %d", lfs_toerror(err));

	delete d;

	return lfs_toerror(err);;
}

// -------------------------------------------------------------------------
// fatfs_fo_dirread()
// Read a single directory entry from a file.

static int
fatfs_fo_dirread(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	fatfs_disk_t      *disk = (fatfs_disk_t *)fp->f_mte->data;
	fs_dir_t *d = (fs_dir_t *)fp->f_data;
	struct dirent     *ent = (struct dirent *) uio->uio_iov[0].iov_base;
	char              *nbuf = ent->d_name;
	off_t              len = uio->uio_iov[0].iov_len;

	struct lfs_info info;
	
	CYG_TRACE3(TFO, "dirread fp=%p uio=%p pos=%d", fp, uio, fp->f_offset);

	if (len < sizeof(struct dirent))
		return EINVAL;

	int res = lfs_dir_read(&lfs_data._lfs, d, &info);
	LFS_INFO("dir_read -> %d", lfs_toerror(res));
	
	if (res == 1) {
		ent->d_type = lfs_totype(info.type);
		strcpy(ent->d_name, info.name);
	}
	return lfs_toerror(res);
}

// -------------------------------------------------------------------------
// fatfs_fo_dirlseek()
// Seek directory to start.

static int
fatfs_fo_dirlseek(struct CYG_FILE_TAG *fp, off_t *pos, int whence)
{
	fatfs_disk_t  *disk = (fatfs_disk_t *)fp->f_mte->data;
	fs_dir_t *d = (fs_dir_t *)fp->f_data;
	int            err;

	CYG_TRACE2(TFO, "dirlseek fp=%p whence=%d", fp, whence);

	// Only allow SEEK_SET to zero

	if (whence != SEEK_SET || *pos != 0)
		return EINVAL;

	lfs_dir_seek(&lfs_data._lfs, d, *pos);



	return 0;
}