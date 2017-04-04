//==========================================================================
//
//      spr_defs.h
//
//      Defines AVR32 architecture specific special-purpose registers (SPRs)
//
//==========================================================================
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Filip Adamec
// Contributors:
// Date:         2012-11-15
// Purpose:      Define AVR32 architecture special-purpose registers
// Usage:        #include <cyg/hal/hal_arch.h>
//
//####DESCRIPTIONEND####
//
//==========================================================================

/* Definition of special-purpose registers (SPRs) */

#ifndef _ASM_SPR_DEFS_H
#define _ASM_SPR_DEFS_H

#define MAX_GRPS (16)
#define MAX_SPRS (0x03fc)


/* System group */
#define SPR_SR          0x0000
#define SPR_EVBA        0x0004
#define SPR_ACBA        0x0008
#define SPR_CPUCR       0x000c
#define SPR_ECDR        0x0010
#define SPR_BEAR        0x013c

/* Debug group */
#define SPR_RSR_DBG     0x0030
#define SPR_RAR_DBG     0x0050

/* Configuartion CPU group */
#define SPR_CONFIG0	0x0100
#define SPR_CONFIG1	0x0104

/* Tick Timer group */
#define SPR_COUNT       0x0108
#define SPR_COMPARE     0x010c

#define SPR_SR_GIE      0x00010000
#define SPR_SR_I0M      0x00020000
#define SPR_SR_I1M      0x00040000
#define SPR_SR_I2M      0x00080000
#define SPR_SR_I3M      0x00100000
#define SPR_SR_EM       0x00200000
#define SPR_SR_M0       0x00400000
#define SPR_SR_M1       0x00800000
#define SPR_SR_M2       0x01000000

#define SPR_SR_OFFSET_GIE      16
#define SPR_SR_OFFSET_I0M      17
#define SPR_SR_OFFSET_I1M      18
#define SPR_SR_OFFSET_I2M      19
#define SPR_SR_OFFSET_I3M      20
#define SPR_SR_OFFSET_EM       21
#define SPR_SR_OFFSET_M0       22
#define SPR_SR_OFFSET_M1       23
#define SPR_SR_OFFSET_M2       24


#endif

// EOF spr_defs.h
