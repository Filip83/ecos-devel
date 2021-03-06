//===========================================================================
//
//      fputc.cxx
//
//      ISO Standard I/O character output functions
//
//===========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
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
//===========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    jlarmour
// Contributors: 
// Date:         2000-04-20
// Purpose:      Provide the fputc() function. Also provides the function
//               versions of putc() and putchar()
// Description: 
// Usage:       
//
//####DESCRIPTIONEND####
//
//===========================================================================

// CONFIGURATION

#include <pkgconf/libc_stdio.h>   // Configuration header

// INCLUDES

#include <cyg/infra/cyg_type.h>     // Common project-wide type definitions
#include <cyg/infra/cyg_ass.h>      // Standard eCos assertion support
#include <cyg/infra/cyg_trac.h>     // Standard eCos tracing support
#include <stddef.h>                 // NULL and size_t from compiler
#include <stdio.h>                  // header for this file
#include <errno.h>                  // error codes
#include <cyg/libc/stdio/stream.hxx>// Cyg_StdioStream

// FUNCTIONS

externC int
fputc( int c, FILE *stream ) __THROW
{
    Cyg_StdioStream *real_stream = (Cyg_StdioStream *)stream;
    Cyg_ErrNo err;
    cyg_uint8 real_c = (cyg_uint8) c;
    
    CYG_REPORT_FUNCNAMETYPE("fputc", "wrote char %d");
    CYG_REPORT_FUNCARG2( "c = %d, stream=%08x", c, stream );
    
    CYG_CHECK_DATA_PTR( stream, "stream is not a valid pointer" );

    err = real_stream->write_byte( real_c );

    if (err)
    {
        real_stream->set_error( err );
        errno = err;
        CYG_REPORT_RETVAL(EOF);
        return EOF;
    } // if
    
    CYG_REPORT_RETVAL((int)real_c);
    return (int)real_c;

} // fputc()


// Also define putchar() even though it can be a macro.
// Undefine the macro first though
#undef putchar

externC int
putchar( int c ) __THROW
{
    return fputc( c, stdout );
} // putchar()

// Also define putc() even though it can be a macro.
// Undefine the macro first though
#undef putc

externC int
putc( int, FILE * ) __THROW CYGBLD_ATTRIB_WEAK_ALIAS(fputc);

// EOF fputc.cxx
