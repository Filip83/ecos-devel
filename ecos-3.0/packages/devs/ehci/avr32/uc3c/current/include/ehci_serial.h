/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   hal_uart_dma.h
 * Author: filip
 *
 * Created on 31. října 2016, 13:49
 */

#ifndef CYGONCE_EHCI_SERIAL_H
#define CYGONCE_EHCI_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <pkgconf/system.h>
#include <pkgconf/io_serial.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/io/io.h>
#include <cyg/io/serialio.h>
#include <cyg/io/devtab.h>
#include <cyg/hal/drv_api.h>
    
typedef struct ehci_serial_channel ehci_serial_channel;
typedef struct ehci_serial_funs ehci_serial_funs;

    
// Pointers into upper-level driver which interrupt handlers need
typedef struct
{
    void (*ehci_init)(ehci_serial_channel *chan);
    void (*block_received_handler)(void);
    void (*block_sent_handler)(void);
    
#if defined(CYGOPT_IO_EHCI_SERIAL_SUPPORT_LINE_STATUS)
    void (*csr_handler)(void);
#endif
} ehci_serial_callbacks_t;

#if defined(CYGOPT_IO_EHCI_SERIAL_SUPPORT_LINE_STATUS)
#  define EHCI_SERIAL_CALLBACKS(_l,_receive,_send,_csr_status)  \
ehci_serial_callbacks_t _l = {                                  \
    _receive,                                                   \
    _send                                                       \  
};
#else
#define EHCI_SERIAL_CALLBACKS(_l,_init,_receive,_send,_csr_status)   \
ehci_serial_callbacks_t _l = {                                 \
    _init,                                                     \
    _receive,                                                  \
    _send,                                                     \
    _csr_status                                                \
};
#endif
    
extern ehci_serial_callbacks_t cyg_io_ehci_serial_callbacks;
   
// Private data which describes this channel
struct ehci_serial_channel 
{
    ehci_serial_funs        *funs;
    ehci_serial_callbacks_t *callbacks;
    void                    *dev_priv;  // Whatever is needed by actual device routines
    cyg_serial_info_t       config;    // Current configuration
    bool                    init;
};

// Initialization macro for serial channel
#define EHCI_SERIAL_CHANNEL(_l,                                              \
                       _funs,                                           \
                       _dev_priv,                                       \
                       _baud, _stop, _parity, _word_length, _flags)     \
ehci_serial_channel _l = {                                                   \
    &_funs,                                                             \
    &cyg_io_ehci_serial_callbacks,                                           \
    &(_dev_priv),                                                       \
    CYG_SERIAL_INFO_INIT(_baud, _stop, _parity, _word_length, _flags),  \
};

// Low level interface functions
struct ehci_serial_funs 
{
    void (*block_receive)(ehci_serial_channel *priv, const uint8_t *buffer, uint16_t length);
    void (*block_send)(ehci_serial_channel *priv,const uint8_t *buffer, uint16_t length);
    // Change hardware configuration (baud rate, etc)
    Cyg_ErrNo (*set_config)(ehci_serial_channel *priv, cyg_uint32 key, const void *xbuf,
                            cyg_uint32 *len);
    void (*block_colse)(ehci_serial_channel *priv);
};

#define EHCI_SERIAL_FUNS(_l,_receive,_send,_set_config,_close)  \
ehc_serial_funs _l = {                                   \
  _receive,                                              \
  _send,                                                 \
  _set_config,                                           \
  _close                                                 \
};

extern cyg_devio_table_t cyg_io_ehci_serial_devio;



#ifdef __cplusplus
}
#endif

#endif /* CYGONCE_EHCI_SERIAL_H */

