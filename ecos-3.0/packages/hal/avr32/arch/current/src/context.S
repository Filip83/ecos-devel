##=============================================================================##
##      context.S
##
##      AVR32 context switch code
##
##=============================================================================
## ####ECOSGPLCOPYRIGHTBEGIN####
## -------------------------------------------
## This file is part of eCos, the Embedded Configurable Operating System.
## Copyright (C) 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
##
## eCos is free software; you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free
## Software Foundation; either version 2 or (at your option) any later
## version.
##
## eCos is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License
## along with eCos; if not, write to the Free Software Foundation, Inc.,
## 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
##
## As a special exception, if other files instantiate templates or use
## macros or inline functions from this file, or you compile this file
## and link it with other works to produce a work based on this file,
## this file does not by itself cause the resulting work to be covered by
## the GNU General Public License. However the source code for this file
## must still be made available in accordance with section (3) of the GNU
## General Public License v2.
##
## This exception does not invalidate any other reasons why a work based
## on this file might be covered by the GNU General Public License.
## -------------------------------------------
## ####ECOSGPLCOPYRIGHTEND####
##=============================================================================
#######DESCRIPTIONBEGIN####
##
## Author(s):   Filip
## Contributors:
## Date:        2012-1-15
## Purpose:     AVR32 context switch code
## Description: This file contains implementations of the thread context
##              switch routines. It also contains the longjmp() and setjmp()
##              routines.
##
######DESCRIPTIONEND####
##
##=============================================================================

#include <pkgconf/hal.h>

#include <cyg/hal/arch.inc>
#include <cyg/hal/avr32.inc>


/////////////////////////
##-----------------------------------------------------------------------------
#//Execute pending DSRs on the interrupt stack with interrupts enabled.
#//Note: this can only be called from code running on a thread stack

#ifdef CYGIMP_HAL_COMMON_INTERRUPTS_USE_INTERRUPT_STACK
        .extern cyg_interrupt_call_pending_DSRs

FUNC_START(hal_interrupt_stack_call_pending_DSRs)
    //Switch to interrupt stack
    mov     r12, sp
    lda.w   sp, __interrupt_stack
    mfsr    r11, SPR_SR
    pushm   r11,r12,lr
    //Enable interrupts before calling DSRs
    csrf    SPR_SR_OFFSET_GIE #GM enable
    csrf    SPR_SR_OFFSET_EM #EM enable

    call    cyg_interrupt_call_pending_DSRs
    ldm     sp++,r11,r12,lr

    mtsr    SPR_SR, r11
    mov	    sp,r12
    retal   r12
FUNC_END(hal_interrupt_stack_call_pending_DSRs)
#endif

#------------------------------------------------------------------------------
# hal_thread_switch_context()
# Switch thread contexts
# R12 = address of sp of next thread to execute
# R11 = address of sp save location of current thread

FUNC_START(hal_thread_switch_context)
    #Push  registers automaticaly saved
    #on interrupt request at the stack
    pushm   r8-r9,r10,r11,r12,lr,pc
    mfsr    r10,SPR_SR
    pushm   r10
    #Push rest of the register on the stack
    pushm   r0-r3,r4-r7
    #save current stack pointer
    st.w    r11[0],sp
    # Now load the destination thread by dropping through
    # to hal_thread_load_context...
FUNC_END(hal_thread_switch_context)


#------------------------------------------------------------------------------
# hal_thread_load_context()
# Load thread context
# R12 = address of sp of next thread to execute
# Note that this function is also the second half of hal_thread_switch_context()
# and is simply dropped into from it.

FUNC_START(hal_thread_load_context)
    # Copy SP to SP
    ld.w    sp,r12[0]
    #Load first set of registers
    ldm     sp++,r0-r7
    #Load and restore status register
    ldm     sp++,r8, r9
    mtsr    SPR_SR, r9
    #Load rest of the registers
    ldm     sp++, r8-r12,pc
FUNC_END(hal_thread_load_context)

#------------------------------------------------------------------------------
# HAL longjmp, setjmp implementations
# hal_setjmp saves only callee-saved registers into buffer supplied in r12:
# Note: These definitions are repeated in hal_arch.h. If changes are required
# remember to update both sets.

FUNC_START(hal_setjmp)
    # Store General Purpose Registers (GPRs).
    # Caller-saved registers are not stored
    stm	    r12, r0, r1, r2, r3, r4, r5, r6, r7, sp, lr
    retal   0
FUNC_END(hal_setjmp)


FUNC_START(hal_longjmp)
    ldm     r12, r0, r1, r2, r3, r4, r5, r6, r7, sp, lr
    # Put the return value into the integer result register.
    # But if it is zero then return 1 instead.	
    cp      r11, 0
    reteq   1
    retal   r11
FUNC_END(hal_longjmp)

#------------------------------------------------------------------------------
# end of context.S
