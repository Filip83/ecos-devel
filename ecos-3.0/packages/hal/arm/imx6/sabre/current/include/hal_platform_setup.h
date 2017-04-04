#ifndef CYGONCE_HAL_PLATFORM_SETUP_H
#define CYGONCE_HAL_PLATFORM_SETUP_H

/*=============================================================================
//
//      hal_platform_setup.h
//
//      Platform specific support for HAL
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2006 Free Software Foundation, Inc.
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   Mike Jones
// Contributors:Ant Micro, ITR-GmbH
// Date:        2013-08-08
// Purpose:     Freescale iMXQ platform specific support routines
// Description:
// Usage:       #include <cyg/hal/hal_platform_setup.h>
//
//####DESCRIPTIONEND####
//
//===========================================================================*/

#include <cyg/hal/var_io.h>
#include <cyg/hal/plf_io.h>
#include <cyg/hal/hal_mmu.h>
#include <pkgconf/hal_arm.h>

        // This will startup the main core and intentionally does not enter
        // SMP so that it can be reused on a single core application. If
        // the main core participates in SMP, then code elsewhere must
        // enable snoop control and put this core in SMP before running
        // other cores.
        .macro _setup
        nop
        
        // Disable interuptss and set to SVC mode
        mov     r0, #(CPSR_IRQ_DISABLE|CPSR_FIQ_DISABLE|CPSR_SUPERVISOR_MODE)
        msr     cpsr, r0

        // Set SPSR
        mov     r0, #(CPSR_IRQ_DISABLE|CPSR_FIQ_DISABLE|CPSR_SUPERVISOR_MODE)
        msr     spsr_cxsf, r0
        mrs     r0, spsr

        // Disable strict align check
    	mrc     p15, 0, r0, c1, c0, 0
    	bic     r0, r0, #(0x1<<1)                     @ Clear A bit of SCTLR
    	mcr     p15, 0, r0, c1, c0, 0

    	// Disable branch prediction
        mrc     p15, 0, r0, c1, c0, 0                 @ Read SCTLR
    	orr     r0, r0, #(1 << 11)                    @ Set the Z bit (bit 11)
    	mcr     p15, 0,r0, c1, c0, 0                  @ Write SCTLR

    	// Invalidate L1 I-cache
    	mov	r1,	#0x0
    	mcr p15, 0, r1, c7, c5, 0       @ Invalidate I-Cache
        mcr p15, 0, r1, c7, c5, 6       @ Invalidate Branch Predictor
        mov  r1, #0x1800
        mcr p15, 0, r1, c1, c0, 0       @ Enable I-Cache and Branch Predictor
        isb

    	// Invalidate L1 D-cache
        mov     r0, #0
        mcr     p15, 2, r0, c0, c0, 0
        mrc     p15, 1, r0, c0, c0, 0

        ldr     r1, =0x7fff
        and     r2, r1, r0, lsr #13

        ldr     r1, =0x3ff

        and     r3, r1, r0, lsr #3  @ NumWays - 1
        add     r2, r2, #1          @ NumSets

        and     r0, r0, #0x7
        add     r0, r0, #4          @ SetShift

        clz     r1, r3              @ WayShift
        add     r4, r3, #1          @ NumWays
10:     sub     r2, r2, #1          @ NumSets--
        mov     r3, r4              @ Temp = NumWays
20:     subs    r3, r3, #1          @ Temp--
        mov     r5, r3, lsl r1
        mov     r6, r2, lsl r0
        orr     r5, r5, r6          @ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
        mcr     p15, 0, r5, c7, c6, 2
        bgt     20b
        cmp     r2, #0
        bgt     10b
        dsb

        // Enable I Cache
        mrc     p15, 0, r0, c1, c0, 0                 @ Read SCTLR
    	orr     r0, r0, #(1 << 12)                    @ Set the I bit (bit 11)
    	mcr     p15, 0,r0, c1, c0, 0                  @ Write SCTLR
        isb

        // Set MMU table address
        ldr     r0, =__mmu_tables_start
        mcr     p15, 0, r0, c2, c0, 0 @ TTBR0

        // Set Client mode for all Domains
        ldr     r0, =0x55555555
        mcr     p15, 0, r0, c3, c0, 0 @ DACR

        // Clear the MMU table
        ldr     r1, =__mmu_tables_start
        ldr     r2, =__mmu_tables_start
        mov     r3, #16
        lsl     r3, r3, #10
        add     r2, r2, r3
        sub     r2, #1                               @ 16 * 1024
        mov     r0,#0
        cmp     r1,r2
        beq     2f
1:      str     r0,[r1],#4
        cmp     r2,r1
        bhi     1b
2:

	// Initialize MMU table
        // These sections are strongly ordered and non-buffered and non-cached all in OCRAM
        ldr     r0, =0x00010C02          @ Entry attributes
        ldr     r1, =__mmu_tables_start  @ Start of the MMU table
        ldr     r2, =0x00000000          @ Virtual address
        add     r1, r2, lsl #2           @ Set base of data
        mov     r2, r2, lsl #2           @ Translate to 4 byte address
        ldr     r3, =0x00000000          @ Physical address
        ldr     r4, =0x00000009          @ Size of memory
1:
        ldr     r5, =0xfffff             @ Mask for address
        and     r0, r5                   @ Remove address from entry
        orr     r0, r3, lsl #20          @ Add base address
        str     r0, [r1]                 @ Store entry
        add     r1, r1, #4               @ Increment virtual address
        add     r3, r3, #1               @ Increment physical address
        sub     r4, r4, #1               @ Decrease number of entries
        cmp     r4, #0                   @ Check if zero
        bne     1b                       @ Repeat if not zero

        ldr     r0, =0x00010C02          @ Entry attributes
        ldr     r1, =__mmu_tables_start  @ Start of the MMU table
        ldr     r2, =0x00000009          @ Virtual address
        add     r1, r2, lsl #2           @ Set base of data
        ldr     r3, =0x00000009          @ Physical address
        ldr     r4, =0x00000001          @ Size of memory
1:
        ldr     r5, =0xfffff             @ Mask for address
        and     r0, r5                   @ Remove address from entry
        orr     r0, r3, lsl #20          @ Add base address
        str     r0, [r1]                 @ Store entry
        add     r1, r1, #4               @ Increment virtual address
        add     r3, r3, #1               @ Increment physical address
        sub     r4, r4, #1               @ Decrease number of entries
        cmp     r4, #0                   @ Check if zero
        bne     1b                       @ Repeat if not zero

        ldr     r0, =0x00010C02          @ Entry attributes
        ldr     r1, =__mmu_tables_start  @ Start of the MMU table
        ldr     r2, =0x0000000a          @ Virtual address
        add     r1, r2, lsl #2           @ Set base of data
        ldr     r3, =0x0000000a          @ Physical address
        ldr     r4, =0x000000f6          @ Size of memory
1:
        ldr     r5, =0xfffff             @ Mask for address
        and     r0, r5                   @ Remove address from entry
        orr     r0, r3, lsl #20          @ Add base address
        str     r0, [r1]                 @ Store entry
        add     r1, r1, #4               @ Increment virtual address
        add     r3, r3, #1               @ Increment physical address
        sub     r4, r4, #1               @ Decrease number of entries
        cmp     r4, #0                   @ Check if zero
        bne     1b                       @ Repeat if not zero

	// Main DDR memory that is cachable
        ldr     r0, =0x00011C0e          @ Entry attributes
        ldr     r1, =__mmu_tables_start  @ Start of the MMU table
        ldr     r2, =0x00000100          @ Virtual address
        add     r1, r2, lsl #2           @ Set base of data
        ldr     r3, =0x00000100          @ Physical address
        ldr     r4, =0x00000800          @ Size of memory
1:
        ldr     r5, =0xfffff             @ Mask for address
        and     r0, r5                   @ Remove address from entry
        orr     r0, r3, lsl #20          @ Add base address
        str     r0, [r1]                 @ Store entry
        add     r1, r1, #4               @ Increment virtual address
        add     r3, r3, #1               @ Increment physical address
        sub     r4, r4, #1               @ Decrease number of entries
        cmp     r4, #0                   @ Check if zero
        bne     1b                       @ Repeat if not zero

        mov     r0, #1
        mcr     p15, 0, r0, c8, c7, 0    @ TLBIALL - Invalidate entire unified TLB
        dsb

        // Setup L2 cache
        ldr     r0, =0x00A02108          @ Tag ram control
        ldr     r1, =0x00000132
        str     r1, [r0]
        ldr     r0, =0x00A0210c          @ Data ram control
        ldr     r1, =0x00000132
        str     r1, [r0]
        ldr     r0, =0x00A02f60          @ Prefetch control
        ldr     r1, =0x40800000
        str     r1, [r0]

        // Enable MMU
        mrc     p15, 0, r0, c1, c0, 0
        orr     r0, r0, #1
        mcr     p15, 0, r0, c1, c0, 0
        isb
        dsb

    	// Invalidate L1 D-cache
        mov     r0, #0
        mcr     p15, 2, r0, c0, c0, 0
        mrc     p15, 1, r0, c0, c0, 0

        ldr     r1, =0x7fff
        and     r2, r1, r0, lsr #13

        ldr     r1, =0x3ff

        and     r3, r1, r0, lsr #3  @ NumWays - 1
        add     r2, r2, #1          @ NumSets

        and     r0, r0, #0x7
        add     r0, r0, #4          @ SetShift

        clz     r1, r3              @ WayShift
        add     r4, r3, #1          @ NumWays
10:     sub     r2, r2, #1          @ NumSets--
        mov     r3, r4              @ Temp = NumWays
20:     subs    r3, r3, #1          @ Temp--
        mov     r5, r3, lsl r1
        mov     r6, r2, lsl r0
        orr     r5, r5, r6          @ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
        mcr     p15, 0, r5, c7, c6, 2
        bgt     20b
        cmp     r2, #0
        bgt     10b
        dsb

        // Enable I and D Caches
        mrc     p15, 0, r0, c1, c0, 0 @ SCTLR
        orr     r0, r0, #0x1000
        orr     r0, r0, #0x0004
        mcr     p15, 0, r0, c1, c0, 0 @ SCTLR
        dsb
        isb

        // Invalidate L2 cache
        ldr     r0, =0x00A0277c          @ Inv way
        ldr     r1, [r0]
        ldr     r2, =0xFFFF
        orr     r1, r1, r2
        str     r1, [r0]
        ldr     r0, =0x00A02730          @ Cache sync
1:
        ldr     r1, [r0]
        and     r1, r1, #1
        cmp     r1, #1
        beq     1b                       @ Loop until 0

        // Enable L2 cache
        ldr     r0, =0x00A02100          @ Control register
        ldr     r1, [r0]
        ldr     r2, =0xFFFFFFFE
        and     r1, r1, r2
        str     r1, [r0]
        ldr     r0, =0x00A02730          @ Cache sync
1:
        ldr     r1, [r0]
        and     r1, r1, #1
        cmp     r1, #1
        beq     1b                       @ Loop until 0

        .endm

#define PLATFORM_SETUP1 _setup

        // This code adds a core to SMP. It assumes the primary core already
        // has enabled snoop control and has entered SMP.
        .macro _setup2
        nop

        // Disable interupts and set to SVC mode
        mov     r0, #(CPSR_IRQ_DISABLE|CPSR_FIQ_DISABLE|CPSR_SUPERVISOR_MODE)
        msr     cpsr, r0

        // Set SPSR
        mov     r0, #(CPSR_IRQ_DISABLE|CPSR_FIQ_DISABLE|CPSR_SUPERVISOR_MODE)
        msr     spsr_cxsf, r0
        mrs     r0, spsr

        cpsie A                         @ Enable non-precise data aborts.

        // Join SMP, Enable SCU broadcast.
        mrc     p15, 0, r0, c1, c0, 1   @ Read ACTLR
        orr     r0, r0, #0x042          @ Set bit 6, 2 and 1
        mcr     p15, 0, r0, c1, c0, 1   @ Write ACTLR

        // Disable strict align check
    	mrc     p15, 0, r0, c1, c0, 0
    	bic     r0, r0, #(0x1<<1)       @ Clear A bit of SCTLR
    	mcr     p15, 0, r0, c1, c0, 0

    	// Disable branch prediction
        mrc     p15, 0, r0, c1, c0, 0                 @ Read SCTLR
    	orr     r0, r0, #(1 << 11)                    @ Set the Z bit (bit 11)
    	mcr     p15, 0,r0, c1, c0, 0                  @ Write SCTLR

    	// Invalidate L1 I-cache
    	mov	r1,	#0x0
    	mcr p15, 0, r1, c7, c5, 0       @ Invalidate I-Cache
        mcr p15, 0, r1, c7, c5, 6       @ Invalidate Branch Predictor
        mov  r1, #0x1800
        mcr p15, 0, r1, c1, c0, 0       @ Enable I-Cache and Branch Predictor
        isb

    	// Invalidate L1 D-cache
        mov     r0, #0
        mcr     p15, 2, r0, c0, c0, 0
        mrc     p15, 1, r0, c0, c0, 0

        ldr     r1, =0x7fff
        and     r2, r1, r0, lsr #13

        ldr     r1, =0x3ff

        and     r3, r1, r0, lsr #3  @ NumWays - 1
        add     r2, r2, #1          @ NumSets

        and     r0, r0, #0x7
        add     r0, r0, #4          @ SetShift

        clz     r1, r3              @ WayShift
        add     r4, r3, #1          @ NumWays
10:     sub     r2, r2, #1          @ NumSets--
        mov     r3, r4              @ Temp = NumWays
20:     subs    r3, r3, #1          @ Temp--
        mov     r5, r3, lsl r1
        mov     r6, r2, lsl r0
        orr     r5, r5, r6          @ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
        mcr     p15, 0, r5, c7, c6, 2
        bgt     20b
        cmp     r2, #0
        bgt     10b
        dsb

        // Set MMU table address
        ldr     r0, =__mmu_tables_start
        mcr     p15, 0, r0, c2, c0, 0 @ TTBR0

        // Set Client mode for all Domains
        ldr     r0, =0x55555555
        mcr     p15, 0, r0, c3, c0, 0 @ DACR

        mov     r0, #1
        mcr     p15, 0, r0, c8, c7, 0                 @ TLBIALL - Invalidate entire unified TLB
        dsb

        // Enable MMU
        mrc     p15, 0, r0, c1, c0, 0
        orr     r0, r0, #1
        mcr     p15, 0, r0, c1, c0, 0
        isb
        dsb

        // Enable I and D Caches
        mrc     p15, 0, r0, c1, c0, 0 @ SCTLR
        orr     r0, r0, #0x1000
        orr     r0, r0, #0x0004
        mcr     p15, 0, r0, c1, c0, 0 @ SCTLR
        dsb
        isb

        .endm

#define PLATFORM_SETUP2 _setup2



//-----------------------------------------------------------------------------
// end of hal_platform_setup.h
#endif // CYGONCE_HAL_PLATFORM_SETUP_H
