#ifndef CYGONCE_NET_TELNETD_TELNETD_H
#define CYGONCE_NET_TELNETD_TELNETD_H
/* =================================================================
 *
 *      telnetd.h
 *
 *      A simple embedded TELNET server
 *
 * ================================================================= 
 * ####ECOSGPLCOPYRIGHTBEGIN####                                     
 * -------------------------------------------                       
 * This file is part of eCos, the Embedded Configurable Operating System.
 * Copyright (C) 2002 Free Software Foundation, Inc.                 
 *
 * eCos is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 or (at your option) any later
 * version.                                                          
 *
 * eCos is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.                                                 
 *
 * You should have received a copy of the GNU General Public License 
 * along with eCos; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.     
 *
 * As a special exception, if other files instantiate templates or use
 * macros or inline functions from this file, or you compile this file
 * and link it with other works to produce a work based on this file,
 * this file does not by itself cause the resulting work to be covered by
 * the GNU General Public License. However the source code for this file
 * must still be made available in accordance with section (3) of the GNU
 * General Public License v2.                                        
 *
 * This exception does not invalidate any other reasons why a work based
 * on this file might be covered by the GNU General Public License.  
 * -------------------------------------------                       
 * ####ECOSGPLCOPYRIGHTEND####                                       
 * =================================================================
 * #####DESCRIPTIONBEGIN####
 * 
 *  Author(s):    Mike Jones <mike@proclivis.com>
 *  Data:         nickg@calivar.com
 *  Contributors: 
 *  Date:         2014-01-02
 *  Purpose:      
 *  Description:  
 *               
 * ####DESCRIPTIONEND####
 * 
 * =================================================================
 */

#include <pkgconf/system.h>
#include <pkgconf/isoinfra.h>
#include <pkgconf/telnetd.h>

#include <cyg/hal/hal_tables.h>

#include <stdio.h>

/* ================================================================= */
/* Start daemon explicitly
 */

#ifndef CYGNUM_TELNETD_SERVER_AUTO_START

__externC void cyg_telnetd_startup(void);

#endif

/* ================================================================= */
/* Lookup Table
 *
 * 
 */

typedef cyg_bool cyg_telnetd_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data);

struct cyg_telnetd_table_entry
{
    char                  *pattern;
    cyg_telnetd_handler   *handler;
    void                  *arg;
} CYG_HAL_TABLE_TYPE;

typedef struct cyg_telnetd_table_entry cyg_telnetd_table_entry;

#define CYG_TELNETD_TABLE_ENTRY( __name, __pattern, __handler, __arg ) \
cyg_telnetd_table_entry __name CYG_HAL_TABLE_ENTRY( telnetd_table ) = { __pattern, __handler, __arg } 

/* ================================================================= */
/* Useful handler functions
 */

/* ----------------------------------------------------------------- */
/*
 */

__externC cyg_bool cyg_telnetd_send_response( FILE *fp, char *response );

/* ----------------------------------------------------------------- */
/*
 */

typedef struct
{
    char        *content_type;
    cyg_uint32  content_length;
    cyg_uint8   *data;
} cyg_telnetd_data;

__externC cyg_bool cyg_telnetd_send_data( int fd, char command, char value );
__externC cyg_bool cyg_telnetd_send_data_array( int fd, char *data );

#define CYG_TELNETD_DATA( __name, __type, __length, __data ) \
cyg_telnetd_data __name = { __type, __length, __data }

/* ================================================================= */
/* TELNET helper macros and functions
 */

/* ----------------------------------------------------------------- */
/* TELNET header support
 *
 * cyg_telnet_start() sends an <CR>. cyg_telnet_finish() sends a prompt.
 */

__externC void cyg_telnet_start( int fd );
__externC void cyg_telnet_finish( int fd );

#define telnet_begin(__client)                    \
        cyg_telnet_start( __client );

#define telnet_end( __client )                    \
        cyg_telnet_finish( __client )


/* ----------------------------------------------------------------- */
#endif /* CYGONCE_NET_TELNETD_TELNETD_H                              */
/* end of telnetd.c                                                  */
