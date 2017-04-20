#ifndef CYGONCE_DAC_H
#define CYGONCE_DAC_H
/*==========================================================================
//
//      dac.h
//
//      Generic DAC driver layer header
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2008 Free Software Foundation, Inc.                        
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
// Author(s):    Mike Jones
// Date:         2013-06-25
// Description:  Implements generic layer of DAC drivers.
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <pkgconf/system.h>
#include <pkgconf/io_dac.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/io/io.h>
#include <cyg/io/devtab.h>
#include <cyg/io/config_keys.h>
#include <cyg/hal/drv_api.h>


#ifdef CYGPKG_IO_DAC_SELECT_SUPPORT
#include <cyg/fileio/fileio.h>
#endif

//==========================================================================
// Configuration information structure

typedef enum dac_polarity {
    DAC_POL_BIPOLAR   = 0x00,
    DAC_POL_UNIPOLAR  = 0x01
} dac_polarity;

typedef enum dac_gain {
    DAC_GAIN1         = 0x00,
    DAC_GAIN2         = 0x01,
    DAC_GAIN3         = 0x02,
    DAC_GAIN4         = 0x03,
    DAC_GAIN5         = 0x04,
    DAC_GAIN6         = 0x05
} dac_gain;

typedef struct
{
    cyg_uint32          rate;           // Output rate         
    dac_gain            gain;           // Part specific gain
    dac_polarity        polarity;       // Part specific polarity
} cyg_dac_info_t;

//==========================================================================
// Output size type.
//
// Define output size type depending on hardware capability.

#if CYGNUM_IO_DAC_OUTPUT_SIZE > 16
typedef cyg_int32 cyg_dac_output_t;
#elif CYGNUM_IO_DAC_OUTPUT_SIZE > 8
typedef cyg_int16 cyg_dac_output_t;
#else
typedef cyg_int8 cyg_dac_output_t;
#endif

//==========================================================================
// Forward type definitions.

typedef struct cyg_dac_device cyg_dac_device;
typedef struct cyg_dac_channel cyg_dac_channel;
typedef struct cyg_dac_functions cyg_dac_functions;

//==========================================================================
// Callbacks from hardware drivers to generic driver.

__externC void cyg_dac_device_init( cyg_dac_device *device );

__externC void cyg_dac_channel_init(cyg_dac_channel *chan);

__externC cyg_uint32 cyg_dac_take_output(cyg_dac_channel *chan, cyg_dac_output_t *output);

__externC void cyg_dac_wakeup(cyg_dac_channel *chan );

//==========================================================================
// Device table functions

__externC cyg_devio_table_t cyg_io_dac_devio;

//==========================================================================
// DAC device
//
// A single device may support several channels which share interrupt
// vectors and output rate settings.

struct cyg_dac_device
{
    cyg_dac_functions   *funs;          // Hardware device functions
    void                *dev_priv;      // Hardware device private data
    cyg_dac_info_t      config;         // Current configuration

    cyg_bool            init;           // Initialized ?
    cyg_drv_mutex_t     lock;           // Device lock
};

#define CYG_DAC_DEVICE(__name, __funs, __dev_priv, __rate )     \
cyg_dac_device __name =                                         \
{                                                               \
    .funs               = __funs,                               \
    .dev_priv           = __dev_priv,                           \
    .config.rate        = __rate,                               \
    .init               = false                                 \
};


//==========================================================================
// DAC channel
//
// Each device may support several channels, each providing a separate
// stream of outputs.

struct  cyg_dac_channel
{
    int                 channel;        // Channel number
    
    cyg_dac_output_t    *buf;           // Output data buffer
    int                 len;            // Buffer length
    volatile int        put;            // Output insert index
    volatile int        get;            // Output extract index

    cyg_dac_device      *device;        // Controlling device
    
    cyg_bool            init;           // Initialized ?
    cyg_drv_cond_t      wait;           // Readers wait here for data
    cyg_bool            waiting;        // True if any threads waiting
    cyg_bool            wakeup;         // True if wakeup needed
    
    cyg_bool            enabled;        // Channel enabled?
    cyg_bool            blocking;       // Blocking IO
    int                 overflow;       // Overflow counter
    
#ifdef CYGPKG_IO_DAC_SELECT_SUPPORT    
    struct CYG_SELINFO_TAG   selinfo;   // Select info
#endif

};

#define CYG_DAC_CHANNEL( __name, __channel, __bufsize, __device )       \
static cyg_dac_output_t __name##_buf[__bufsize];                        \
cyg_dac_channel __name =                                                \
{                                                                       \
    .channel            = __channel,                                    \
    .buf                = __name##_buf,                                 \
    .len                = __bufsize,                                    \
    .put                = 0,                                            \
    .get                = 0,                                            \
    .device             = __device,                                     \
    .init               = false                                         \
};
              
//==========================================================================
// Device functions
//
// These are the functions exported by the hardware device to the
// generic layer.

struct cyg_dac_functions
{
    void (*enable)( cyg_dac_channel *chan );
    void (*disable)( cyg_dac_channel *chan );

    void (*set_rate)( cyg_dac_channel *chan, cyg_uint32 rate );
    void (*set_gain)( cyg_dac_channel *chan, cyg_uint32 gain );
    void (*set_polarity)( cyg_dac_channel *chan, cyg_uint32 polarity );
};

#define CYG_DAC_FUNCTIONS( __name, __enable, __disable, __set_rate, __set_gain, __set_polarity )    \
cyg_dac_functions __name =                                              \
{                                                                       \
    .enable             = __enable,                                     \
    .disable            = __disable,                                    \
    .set_rate           = __set_rate,                                   \
    .set_gain           = __set_gain,                                   \
    .set_polarity       = __set_polarity,                               \
};

//==========================================================================
#endif // CYGONCE_DAC_H
