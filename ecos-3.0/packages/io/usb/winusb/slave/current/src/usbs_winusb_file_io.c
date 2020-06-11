/*
 * usbs_winusb_io.c
 *
 * Created: 25.10.2012 9:04:34
 *  Author: Filip
 */ 

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/cyg_trac.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/drv_api.h>
#include <cyg/kernel/kapi.h>

#include <cyg/io/usb/usbs_winusb.h>
#include <string.h>
#include <cyg/io/io.h>
#include <cyg/io/devtab.h>
#include <pkgconf/io_usb_slave_winusb.h>
//#include <usbio/usbs_winusb_io.h>

/**
 * \file usbs_winusb_io.c
 * \brief File containing usb io mapping to stdio functions.
 *
 * In this file usb io mapping to stdio function is implemented.
 * The usb stdio support read and write operations only with not
 * timeout.
 * \todo In future it will be nice to have usb io with timeouts.
 */
//#define CYGOPT_IO_USB_WINUSB_SLAVE_FILE_IO 1
#if CYGOPT_IO_USB_WINUSB_SLAVE_FILE_IO == 1
Cyg_ErrNo cyg_usbs_write(cyg_io_handle_t handle, const void *buf, cyg_uint32 *len);
Cyg_ErrNo cyg_usbs_read(cyg_io_handle_t handle, void *buf, cyg_uint32 *len);
cyg_bool  cyg_usbs_select(cyg_io_handle_t handle, cyg_uint32 which, CYG_ADDRWORD info);
Cyg_ErrNo cyg_usbs_get_config(cyg_io_handle_t handle, cyg_uint32 key, void *buf, cyg_uint32 *len);
Cyg_ErrNo cyg_usbs_set_config(cyg_io_handle_t handle, cyg_uint32 key, const void *buf, cyg_uint32 *len);

cyg_bool  cyg_usbs_inti(struct cyg_devtab_entry *tab);
Cyg_ErrNo cyg_usbs_lookup(struct cyg_devtab_entry **tab,
struct cyg_devtab_entry *sub_tab,
const char *name);

DEVIO_TABLE(usbs_io_funs,&cyg_usbs_write,&cyg_usbs_read,&cyg_usbs_select,
			&cyg_usbs_get_config,&cyg_usbs_set_config);
			
DEVTAB_ENTRY(usb_devtab,CYGOPT_IO_USB_WINUSB_SLAVE_FILE_IO_NAME,NULL,&usbs_io_funs,&cyg_usbs_inti,&cyg_usbs_lookup,&usbs_winusb0);

static cyg_bool Initialized = false;
static cyg_bool DriverInitialized = false;

cyg_bool  cyg_usbs_inti(struct cyg_devtab_entry *tab)
{
    if(!DriverInitialized)
    {
        //usbs_kinetis_init(); 
        DriverInitialized = true;
    }
    return true;
}

Cyg_ErrNo cyg_usbs_lookup(struct cyg_devtab_entry **tab,
struct cyg_devtab_entry *sub_tab,
const char *name)
{
    if(!Initialized)
    {
        Initialized = true;
        //usbs_winusb_start(NULL, NULL, NULL, NULL);
    }

    return ENOERR;
}

Cyg_ErrNo cyg_usbs_write(cyg_io_handle_t handle, const void *buf, cyg_uint32 *len)
{
    int ret;
    ret = usbs_winusb_tx(&usbs_winusb0, buf, *len);
    if(ret < 0)
    {
        *len = 0;
        return ret;
    }
    *len = ret;
    return ENOERR;
}

Cyg_ErrNo cyg_usbs_read(cyg_io_handle_t handle, void *buf, cyg_uint32 *len)
{
    int ret;
    ret = usbs_winusb_rx(&usbs_winusb0,buf,*len);
    if(ret < 0)
    {
        *len = 0;
        return ret;
    }
    *len = ret;
    return ENOERR;
}

cyg_bool  cyg_usbs_select(cyg_io_handle_t handle, cyg_uint32 which, CYG_ADDRWORD info)
{
    return false;
}

Cyg_ErrNo cyg_usbs_get_config(cyg_io_handle_t handle, cyg_uint32 key, void *buf, cyg_uint32 *len)
{
    cyg_uint32 *configured = (cyg_uint32*)buf;
    *configured = usbs_winusb_is_configured();
    return ENOERR;
}

Cyg_ErrNo cyg_usbs_set_config(cyg_io_handle_t handle, cyg_uint32 key, const void *buf, cyg_uint32 *len)
{
    return EINVAL;	
}
#endif