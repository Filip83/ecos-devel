##=============================================================================
##
##	vectors.S
##
##	AVR32 exception vectors
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
## Author(s): 	Filip
## Contributors:
## Date:	2012-1-15
## Purpose:	AVR32 exception vectors
## Description:	This file defines the code placed into the exception
##              vectors. It also contains the first level default VSRs
##		that save and restore state for both exceptions and
##		interrupts.
##
######DESCRIPTIONEND####
##
##=============================================================================


#include <cyg/hal/avr32/io.h>
#include <pkgconf/hal.h>

#ifdef CYGPKG_KERNEL
#include <pkgconf/kernel.h>
#endif

#include <cyg/hal/hal_intr.h>

    // This must be linked @ 0x80000000 if it is to be run upon reset.
    .section  .reset, "ax", @progbits

    .global _start
    .type _start, @function
_start:
  //  movh    r0,0xffff
  //  orl     r0,0x2c00

  //  movh    r1,0x0008
  //  orl     r1,0x0000
  //  st.w    r0[88],r1

  //  st.w    r0[68],r1

  //  st.w    r0[0],r1
    
#ifdef BOOT_LOADER
    // If boot loader startup is enabled
    // this code load first 4B from user FLASH
    // page and if its value is 0xa5a55a5a its
    // start execution at 0x80010000 address
    // otherwise it continue to execute code
    // fromn _stext
    movh    r0,0x8080
    orl     r0,0x0004
    ld.w    r1,r0
    movh    r0,0xa5a5
    orl     r0,0x5a5a
    cp      r0,r1
    brne    _stext
    movh    r0,0x8001
    icall   r0
#else
    // Jump to the C runtime startup routine.
    rjmp _stext
#endif
    // _stext is placed outside the .reset section so that the program 
    // entry point can be changed without affecting the C runtime startup.
    .section  .text._stext, "ax", @progbits


    .global _stext
    .type _stext, @function
_stext:
    // Set initial stack pointer.
    lda.w   sp, __interrupt_stack

    // Set up EVBA so interrupts can be enabled.
    lda.w   r0, _evba
    mtsr    AVR32_EVBA, r0

    // Load initialized data having a global lifetime from the data LMA.
    lda.w   r0, _data
    lda.w   r1, _edata
    cp      r0, r1
    brhs    idata_load_loop_end
    lda.w   r2, _data_lma
idata_load_loop:
    ld.d    r4, r2++
    st.d    r0++, r4
    cp      r0, r1
    brlo    idata_load_loop
idata_load_loop_end:
    // Clear uninitialized data having a global lifetime 
    // in the blank static storage section.
    lda.w   r0, __bss_start
    lda.w   r1, _end
    cp      r0, r1
    brhs    udata_clear_loop_end
    mov     r2, 0
    mov     r3, 0
udata_clear_loop:
    st.d    r0++, r2
    cp      r0, r1
    brlo    udata_clear_loop
udata_clear_loop_end:

#ifdef CONFIG_FRAME_POINTER
    // Safety: Set the default "return" @ to the exit routine address.
    lda.w   lr, exit
#endif
    call    hal_platform_init
    // Enable the exception processing.
    csrf    AVR32_SR_EM_OFFSET
    call    cyg_hal_invoke_constructors

    // Start the show.
    call    cyg_start
    // If kernel package is not defined call main function
    #ifndef CYGPKG_KERNEL
    call    main
    #endif
    movh    r0,0x8001
    mov     pc,r0
    rjmp    _start

#---------------------------------------------------------------------------
# Define the exception vectors.
# Every exception save register contex on stack and set its
# number to R11 for further processing.
_Unrecoverable_Exception:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_UNRECOVERABLE
    rjmp    _exception_handler
    // in unrecoverable exception nothing can be done
    rjmp    $

_TLB_Multiple_Hit:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_MPU_TLB_MULT_MISS
    rjmp    _exception_handler
        
_Bus_Error_Data_Fetch:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_BUS_ERROR_DATA
    rjmp    _exception_handler
      
_Bus_Error_Instruction_Fetch:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_BUS_ERROR_INSTR
    rjmp    _exception_handler
       
_NMI:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_NMI
    rjmp    _exception_handler
      
_Instruction_Address:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_INSTR_ADDRESS
    rjmp    _exception_handler

_ITLB_Protection:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_MPU_ITLB_PROTECTION
    rjmp    _exception_handler
      
_Breakpoint:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_BREAKEPOINT 
    rjmp    _exception_handler
  
_Illegal_Opcode:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_ILEGAL_OPC
    rjmp    _exception_handler

_Unimplemented_Instruction:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_UNIMPLEMENTED_INSTR
    rjmp    _exception_handler

_Privilege_Violation:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_PRIVILEGE_VIOLATION
    rjmp    _exception_handler


    // Floating-Point: UNUSED IN AVR32UC and AVR32AP.
_Floating_Point:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_FPU
    rjmp    _exception_handler

    // Coprocessor Absent: UNUSED IN AVR32UC.
_Coprocessor_Absent:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_COPROCESSOR_ABSETN
    rjmp    _exception_handler

_Data_Address_Read:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_DATA_ADDRESS_READ
    rjmp    _exception_handler

_Data_Address_Write:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_DATA_ADDRRESS_WRITE
    rjmp    _exception_handler

_DTLB_Protection_Read:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_MPU_DTLB_PROTECTION_READ
    rjmp    _exception_handler

_DTLB_Protection_Write:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_MPU_DTLB_PROTECTION_WRITE
    rjmp    _exception_handler

    // DTLB Modified: UNUSED IN AVR32UC.
_DTLB_Modified:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_DTLB_MODIFIED
    rjmp    _exception_handler

_ITLB_Miss:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_UNIMPLEMENTED
    rjmp    _exception_handler

_DTLB_Miss_Read:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_MPU_DTLB_MISS_READ
    rjmp    _exception_handler

_DTLB_Miss_Write:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r11, CYGNUM_HAL_VECTOR_MPU_DTLB_MISS_WRITE
    rjmp    _exception_handler

    // Supervisor Call.
    // Supervisor call must be handled separately from otehr exception
    // beacaose rets instruction to return must be used
_Supervisor_Call:
    pushm   r0-r7,r8-r9,r10,r11,r12,lr
    mov     r12,sp
    mov     r11, CYGNUM_HAL_VECTOR_CUPERVISOR_CALL
    call    cyg_hal_exception_handler
    popm    r0-r7,r8-r9,r10,r11,r12,lr
    rets
        
_exception_handler:
    mov     r12,sp
    call    cyg_hal_exception_handler
    ldm	    sp++, r0, r1, r2, r3, r4, r5, r6, r7,r8,r9,r10,r11,r12, lr
    rete
	

.section  .exception, "ax", @progbits

// Start of Exception Vector Table.

/*
 * EVBA must be aligned with a power of two strictly greater than the
 * EVBA-relative offset of the last vector.
 */
    .balign 0x200

    // Export symbol.
    .global _evba
    .type _evba, @function
_evba:

    .org  0x000
    // Unrecoverable Exception.
_handle_Unrecoverable_Exception:
    rjmp _Unrecoverable_Exception

    .org  0x004
    // TLB Multiple Hit.
_handle_TLB_Multiple_Hit:
    rjmp _TLB_Multiple_Hit

    .org  0x008
    // Bus Error Data Fetch.
_handle_Bus_Error_Data_Fetch:
    rjmp _Bus_Error_Data_Fetch

    .org  0x00C
    // Bus Error Instruction Fetch.
_handle_Bus_Error_Instruction_Fetch:
    rjmp _Bus_Error_Instruction_Fetch

    .org  0x010
    // NMI.
_handle_NMI:
    rjmp _NMI

    .org  0x014
    // Instruction Address.
_handle_Instruction_Address:
    rjmp _Instruction_Address

    .org  0x018
    // ITLB Protection.
_handle_ITLB_Protection:
    rjmp _ITLB_Protection

    .org  0x01C
    // Breakpoint.
_handle_Breakpoint:
    rjmp _Breakpoint

    .org  0x020
    // Illegal Opcode.
_handle_Illegal_Opcode:
    rjmp _Illegal_Opcode

    .org  0x024
    // Unimplemented Instruction.
_handle_Unimplemented_Instruction:
    rjmp _Unimplemented_Instruction

    .org  0x028
    // Privilege Violation.
_handle_Privilege_Violation:
    rjmp _Privilege_Violation

    .org  0x02C
    // Floating-Point: UNUSED IN AVR32UC and AVR32AP.
_handle_Floating_Point:
    rjmp _Floating_Point

    .org  0x030
    // Coprocessor Absent: UNUSED IN AVR32UC.
_handle_Coprocessor_Absent:
    rjmp _Coprocessor_Absent

    .org  0x034
    // Data Address (Read).
_handle_Data_Address_Read:
    rjmp _Data_Address_Read

    .org  0x038
    // Data Address (Write).
_handle_Data_Address_Write:
    rjmp _Data_Address_Write

    .org  0x03C
    // DTLB Protection (Read).
_handle_DTLB_Protection_Read:
    rjmp _DTLB_Protection_Read

    .org  0x040
    // DTLB Protection (Write).
_handle_DTLB_Protection_Write:
    rjmp _DTLB_Protection_Write

    .org  0x044
    // DTLB Modified: UNUSED IN AVR32UC.
_handle_DTLB_Modified:
    rjmp _DTLB_Modified

    .org  0x050
    // ITLB Miss.
_handle_ITLB_Miss:
    rjmp _ITLB_Miss

    .org  0x060
    // DTLB Miss (Read).
_handle_DTLB_Miss_Read:
    rjmp _DTLB_Miss_Read

    .org  0x070
    // DTLB Miss (Write).
_handle_DTLB_Miss_Write:
    rjmp _DTLB_Miss_Write

    .org  0x100
    // Supervisor Call.
_handle_Supervisor_Call:
    rjmp _Supervisor_Call

/*
 * Interrupt support.
 * The interrupt controller must provide the offset address relative to EVBA.
 * Important note:
 * All interrupts call a C function named _get_interrupt_handler.
 * This function will read group and interrupt line number to then return in
 * R12 a pointer to a user-provided interrupt handler. Nested interrupt
 * are not implemented.
 */

    .balign 4

    .irp    priority, 0, 1, 2, 3
    .global _int\priority
    .type   _int\priority, @function
_int\priority:
#   breakpoint
    pushm   r0-r3,r4-r7

    // Pass the int_level parameter to the _get_interrupt_handler function.
    // Set scheduler lock to prevent thread rescheduling while the ISR runs
#ifdef CYGFUN_HAL_COMMON_KERNEL_SUPPORT
    .extern cyg_scheduler_sched_lock
    lda.w   r12, cyg_scheduler_sched_lock
    ld.w    r11,r12
    sub     r11,-1
    st.w    r12,r11
#endif

#ifdef CYGIMP_HAL_COMMON_INTERRUPTS_USE_INTERRUPT_STACK
    //Interrupts execute on their own dedicated stack.
    //If we're on a thread stack, switch to the interrupt stack.
    //If we're called inside a nested interrupt, do nothing.
    mov     r12,sp                          //Stash SP for later
    lda.w   r11,__interrupt_stack           //stack top (highest addr + 1)
    lda.w   r10,__interrupt_stack_base      //stack base (lowest addr)
    cp.w    sp,r10                          //if (sp < __interrupt_stack_base)
    brlt    1f                              //   switch to interrupt stack
    cp.w    sp,r11                          //if (sp < __interrupt_stack_top)
    brlt    2f                              //   already on interrupt stack
    nop                                     //delay slot
1:  mov     sp,r11                          //Switch to interrupt stack
2:  pushm   r12                             //Make space to save old SP...
                                            //...and save it on the stack
#endif

#if defined(CYGPKG_KERNEL_INSTRUMENT) && defined(CYGDBG_KERNEL_INSTRUMENT_INTR)
    // Log the interrupt if kernel tracing is enabled
    // This code do net set interrupt number corectly!!!
    mov     r12,0x0301                    //arg1 = type = INTR,RAISE
    mov     r11,0                         //arg2 = vector number
    mov     r10,0                         //arg3 = 0
    call   _cyg_instrument                //call instrument function
#endif
    // Nested interrupts are not enabled appropriate code
    // would be wise to add heare
    mov     r12, \priority
    call    hal_cal_interrupt_handler

#ifdef CYGIMP_HAL_COMMON_INTERRUPTS_USE_INTERRUPT_STACK

    //If we are returning from the last nested interrupt, move back
    //to the thread stack. interrupt_end() must be called on the
    //thread stack since it potentially causes a context switch.
    //Since we have arranged for the top of stack location to
    //contain the sp we need to go back to here, just pop it off
    //and put it in SP.

    ld.w    sp,sp[0]
#endif

#ifdef CYGFUN_HAL_COMMON_KERNEL_SUPPORT

    //We only need to call _interrupt_end() when there is a kernel
    //present to do any tidying up.

    //Call interrupt_end() to execute any pending DSRs
    //  Arg 0 = return value from ISR
    //  Arg 1 = object associated with interrupt
    //  Arg 2 = HAL_SavedRegisters struct
    mov     r12, r11
    mov     r11, r10
    mov     r10, sp

    .extern interrupt_end
    call    interrupt_end                   //call into C to finish off
                                            // DSRs are called hear
#endif
    //Fall through to restore_state...
    //Return from an interrupt
    ldm  sp++,r0-r7
    rete
.endr

##-----------------------------------------------------------------------------
## end of vectors.S
