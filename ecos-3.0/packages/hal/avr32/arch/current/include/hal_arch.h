//==========================================================================
//
//      hal_arch.h
//
//      Architecture specific abstractions
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
// Author(s):    Filip
// Contributors:
// Date:         2012-06-17
// Purpose:      Define architecture abstractions
// Usage:        #include <cyg/hal/hal_arch.h>
//
//####DESCRIPTIONEND####
//
//==========================================================================

#ifndef CYGONCE_HAL_HAL_ARCH_H
#define CYGONCE_HAL_HAL_ARCH_H

// Include macros to access special-purpose registers (SPRs)
#include <cyg/hal/spr_defs.h>
#include <cyg/hal/avr32/io.h>


#define CYG_HAL_AVR32_REG_SIZE 4

#ifndef __ASSEMBLER__
#include <pkgconf/hal.h>
#include <cyg/infra/cyg_type.h>

//-------------------------------------------------------------------------
// Processor selap macro
#include <cyg/hal/utils/compiler.h>

#define SLEEP(sleep_mode)  {__asm__ __volatile__ ("sleep "STRINGZ(sleep_mode));}

//--------------------------------------------------------------------------
// Processor saved states:
// The layout of this structure is also defined in "arch.inc", for assembly
// code. Do not change this without changing that (or vice versa).

#define CYG_HAL_AVR32_REG CYG_WORD32

typedef struct
{
    // These are common to all saved states exceptions ans interrupts
    // Most of stores are handled by software
    CYG_WORD32		r7;
    CYG_WORD32		r6;
    CYG_WORD32		r5;
    CYG_WORD32		r4;
    CYG_WORD32		r3;
    CYG_WORD32		r2;
    CYG_WORD32		r1;
    CYG_WORD32		r0;
    CYG_WORD32          sr;             /* Status Reg             */
    CYG_WORD32          pc;             /* Program Counter        */
    CYG_WORD32          lr;             /* Link Reg               */
    CYG_WORD32		r12;
    CYG_WORD32		r11;
    CYG_WORD32		r10;
    CYG_WORD32		r9;
    CYG_WORD32		r8;
} HAL_SavedRegisters;

//--------------------------------------------------------------------------
//  Utilities


// Move from architecture special register (SPR)
#define MFSPR(_spr_)  __builtin_mfsr(_spr_)

// Move data to architecture special registers (SPR)
#define MTSPR(_spr_, _val_)  __builtin_mtsr(_spr_, _val_)

// Set flag in status register
#define SSRF(_srf_)			    \
	({ asm volatile ("ssrf %0"          \
	:				    \
	: "i"(_srf_) );                     \
	})

// Celear flag in status register
#define CSRF(_srf_)			    \
        ({ asm volatile ("csrf %0"          \
        :				    \
        : "i"(_srf_)			    \
        );				    \
	})

#define clz(u)              __builtin_clz(u)
//--------------------------------------------------------------------------
// Exception handling function.
// This function is defined by the kernel according to this prototype. It is
// invoked from the HAL to deal with any CPU exceptions that the HAL does
// not want to deal with itself. It usually invokes the kernel's exception
// delivery mechanism.

externC void cyg_hal_deliver_exception( CYG_WORD code, CYG_ADDRWORD data );

//--------------------------------------------------------------------------
// Bit manipulation macros

externC cyg_uint32 hal_lsbit_index(cyg_uint32 mask);
externC cyg_uint32 hal_msbit_index(cyg_uint32 mask);

#define HAL_LSBIT_INDEX(index, mask) index = hal_lsbit_index(mask);

// NOTE - Below can be optimized with l.ff1 instruction if that optional
//        instruction is implemented in HW.  OR12k does not implement
//        it at this time, however.
//#define HAL_MSBIT_INDEX(index, mask) index = hal_msbit_index(mask);

//--------------------------------------------------------------------------
// Context Initialization


// Initialize the context of a thread.
// Arguments:
// _sparg_ name of variable containing current sp, will be written with new sp
// _thread_ thread object address, passed as argument to entry point
// _entry_ entry point address.
// _id_ bit pattern used in initializing registers, for debugging.

#define HAL_THREAD_INIT_CONTEXT( _sparg_, _thread_, _entry_, _id_ )         \
{                                                                           \
    int _i_;                                                                \
    register CYG_WORD _sp_ = ((CYG_ADDRESS)_sparg_);                        \
    register HAL_SavedRegisters *_regs_;                                    \
    _regs_ = (HAL_SavedRegisters *)(((_sp_) -                               \
                sizeof(HAL_SavedRegisters)) & ~(CYGARC_ALIGNMENT - 1));     \
    _sp_ &= ~(CYGARC_ALIGNMENT - 1);                                        \
    (_regs_)->r12 = (CYG_HAL_AVR32_REG)(_thread_); /* arg1 = thread ptr */  \
    (_regs_)->r11 = (CYG_HAL_AVR32_REG)(_id_);     /* arg2 = thread ID */   \
    (_regs_)->sr = (0x00400000);                   /* Interrupts enabled*/  \
    (_regs_)->pc = (CYG_WORD32)(_entry_);          /* PC = entry point*/    \
    (_regs_)->lr = (CYG_WORD32)(_entry_);          /* PC = entry point*/    \
    (_regs_)->r0 = 0;                                                       \
    (_regs_)->r1 = 0;                                                       \
    (_regs_)->r2 = 0;                                                       \
    (_regs_)->r3 = 0;                                                       \
    (_regs_)->r4 = 0;                                                       \
    _sparg_ = (CYG_ADDRESS)_regs_;                                          \
}

//--------------------------------------------------------------------------
// Context switch macros.

// The arguments to these macros are *pointers* to locations where the
// stack pointer of the thread is to be stored/retrieved, i.e. *not*
// the value of the stack pointer itself.

externC void hal_thread_switch_context( CYG_ADDRESS to, CYG_ADDRESS from );
externC void hal_thread_load_context( CYG_ADDRESS to )
    __attribute__ ((noreturn));

#define HAL_THREAD_SWITCH_CONTEXT(_fspptr_,_tspptr_)                    \
        hal_thread_switch_context( (CYG_ADDRESS)_tspptr_,               \
                                   (CYG_ADDRESS)_fspptr_);

#define HAL_THREAD_LOAD_CONTEXT(_tspptr_)                               \
        hal_thread_load_context( (CYG_ADDRESS)_tspptr_ );

// Translate a stack pointer as saved by the thread context macros above into
// a pointer to a HAL_SavedRegisters structure.
#define HAL_THREAD_GET_SAVED_REGISTERS( _sp_, _regs_ )  \
        (_regs_) = (HAL_SavedRegisters *)(_sp_)

//--------------------------------------------------------------------------
// Execution reorder barrier.
// When optimizing the compiler can reorder code. In multithreaded systems
// where the order of actions is vital, this can sometimes cause problems.
// This macro may be inserted into places where reordering should not happen.
// The "memory" keyword is potentially unnecessary, but it is harmless to
// keep it.

#define HAL_REORDER_BARRIER() asm volatile ( "" : : : "memory" )

//--------------------------------------------------------------------------
// Breakpoint support
// HAL_BREAKPOINT() is a code sequence that will cause a breakpoint to
//    occur if executed.
// HAL_BREAKINST is the value of the breakpoint instruction and...
// HAL_BREAKINST_SIZE is its size in bytes and...
// HAL_BREAKINST_TYPE is its type.

#define HAL_BREAKPOINT(_label_)                 \
    asm volatile (" .globl  " #_label_ ";"      \
                  #_label_ ":"                  \
                 " breakpoint;"                 \
                );

#define HAL_BREAKINST           (0xd673)    // breakepoint instruction

#define HAL_BREAKINST_SIZE      2

#define HAL_BREAKINST_TYPE      cyg_uint16

//--------------------------------------------------------------------------
// Thread register state manipulation for GDB support.
// GDB support is not fully implemented JTAG debugger
// is used instead.
// Default to a 32 bit register size for GDB register dumps.
// Not realy neaded but by default GDB debug is enabled in 
// ecos kernel. This is not working code!!!
#ifndef CYG_HAL_GDB_REG
#define CYG_HAL_GDB_REG CYG_WORD32
#endif

// Register layout expected by GDB
typedef struct
{
    // These are common to all saved states exceptions
    // Most of stores are handled by software
    CYG_WORD32              r0[5];
    CYG_WORD32              lr;             /* Link Reg               */
    CYG_WORD32              pc;             /* Program Counter        */
    CYG_WORD32              sr;             /* Status Reg             */
    CYG_WORD32              r1[8];
} GDB_Registers;

// Copy a set of registers from a HAL_SavedRegisters structure into a
// GDB_Registers structure.
#define HAL_GET_GDB_REGISTERS( _aregval_, _regs_ )               \
    CYG_MACRO_START                                              \
    GDB_Registers *_gdb_ = (GDB_Registers *)(_aregval_);         \
    int _i_;                                                     \
                                                                 \
    for( _i_ = 0; _i_ <  15; _i_++ ) {                           \
        ((CYG_WORD32*)_gdb_)[_i_] = ((CYG_WORD32*)(_regs_))[_i_];\
    }                                                            \
                                                                 \
    _gdb_->pc = (_regs_)->pc;                                    \
    _gdb_->sr = (_regs_)->sr;                                    \
    CYG_MACRO_END

// Copy a set of registers from a GDB_Registers structure into a
// HAL_SavedRegisters structure.
#define HAL_SET_GDB_REGISTERS( _regs_ , _aregval_ )              \
    CYG_MACRO_START                                              \
    GDB_Registers *_gdb_ = (GDB_Registers *)(_aregval_);         \
    int _i_;                                                     \
                                                                 \
    for( _i_ = 0; _i_ <  15; _i_++ )                             \
        ((CYG_WORD32*)(_regs_))[_i_] = ((CYG_WORD32*)_gdb_)[_i_];\
                                                                 \
    (_regs_)->pc = _gdb_->pc;                                    \
    (_regs_)->sr = _gdb_->sr;                                    \
    CYG_MACRO_END

//--------------------------------------------------------------------------
// HAL setjmp
// Note: These definitions are repeated in context.S. If changes are
// required remember to update both sets.

#define CYGARC_JMP_BUF_R0       0
#define CYGARC_JMP_BUF_R1       1
#define CYGARC_JMP_BUF_R2       2
#define CYGARC_JMP_BUF_R3       3
#define CYGARC_JMP_BUF_R4       4
#define CYGARC_JMP_BUF_R5       5
#define CYGARC_JMP_BUF_R6       6
#define CYGARC_JMP_BUF_R7       7
#define CYGARC_JMP_BUF_SP       8
#define CYGARC_JMP_BUF_LR       9

#define CYGARC_JMP_BUF_SIZE     10

#define jmpbuf_regsize          4

typedef CYG_HAL_AVR32_REG hal_jmp_buf[CYGARC_JMP_BUF_SIZE];

externC int hal_setjmp(hal_jmp_buf env);
externC void hal_longjmp(hal_jmp_buf env, int val);

//-------------------------------------------------------------------------
// Idle thread code.
// This macro is called in the idle thread loop, and gives the HAL the
// chance to run code when no threads are runnable. Typical idle
// thread behaviour might be to halt the processor.

externC void hal_idle_thread_action(cyg_uint32 loop_count);

#define HAL_IDLE_THREAD_ACTION(_count_) hal_idle_thread_action(_count_)

//--------------------------------------------------------------------------
// Minimal and sensible stack sizes: the intention is that applications
// will use these to provide a stack size in the first instance prior to
// proper analysis.  Idle thread stack should be this big.

// *** THESE ARE NOT INTENDED TO BE GUARANTEED SUFFICIENT STACK SIZES ***
// They are, however, enough to start programming.
// You might, for example, need to make your stacks larger if you have
// large "auto" variables.

// This is not a config option because it should not be adjusted except
// under "enough rope to hang yourself" sort of disclaimers.

// Typical case stack frame size: 
#define CYGNUM_HAL_STACK_FRAME_SIZE (7*4)

// Stack needed for a context switch:
#define CYGNUM_HAL_STACK_CONTEXT_SIZE (17 * 4) 

// Interrupt + call to ISR, interrupt_end() and the DSR
#define CYGNUM_HAL_STACK_INTERRUPT_SIZE  \
            (CYGNUM_HAL_STACK_CONTEXT_SIZE + 2*CYGNUM_HAL_STACK_FRAME_SIZE)

// We define a minimum stack size as the minimum any thread could ever
// legitimately get away with. We can throw asserts if users ask for less
// than this. Allow enough for three interrupt sources - clock, serial and
// one other

// If interrupts are segregated onto their own stack...
#ifdef CYGIMP_HAL_COMMON_INTERRUPTS_USE_INTERRUPT_STACK

// An interrupt stack which is large enough for all possible interrupt
// conditions (and only used for that purpose) exists.  "User" stacks
// can therefore be much smaller
// NOTE - interrupt stack sizes can be smaller if we don't allow interrupts
//         to nest.

# define CYGNUM_HAL_STACK_SIZE_MINIMUM                      \
                    ((3 * 5)*CYGNUM_HAL_STACK_FRAME_SIZE +  \
                    2*CYGNUM_HAL_STACK_INTERRUPT_SIZE)

#else

// No separate interrupt stack exists.  Make sure all threads contain
// a stack sufficiently large
# define CYGNUM_HAL_STACK_SIZE_MINIMUM                  \
        (( 3*CYGNUM_HAL_STACK_INTERRUPT_SIZE) +         \
         (25*CYGNUM_HAL_STACK_FRAME_SIZE))
#endif

// Now make a reasonable choice for a typical thread size. Pluck figures
// from thin air and say 40 call frames
#define CYGNUM_HAL_STACK_SIZE_TYPICAL                \
        (CYGNUM_HAL_STACK_SIZE_MINIMUM +             \
         40 * (CYGNUM_HAL_STACK_FRAME_SIZE))

#endif /* __ASSEMBLER__ */

//--------------------------------------------------------------------------
// Macros for switching context between two eCos instances (jump from
// code in ROM to code in RAM or vice versa).
// This is not implemented
#define CYGARC_HAL_SAVE_GP()        __asm("nop")
#define CYGARC_HAL_RESTORE_GP()     __asm("nop")

#define HAL_PLATFORM_RESET_ENTRY     0x80000000

//--------------------------------------------------------------------------
// Macro for finding return address of current function
#define CYGARC_HAL_GET_RETURN_ADDRESS(_x_, _dummy_) \
  asm volatile ( "mov%0,lr;" : "=r" (_x_) )

#define CYGARC_HAL_GET_RETURN_ADDRESS_BACKUP(_dummy_)

//--------------------------------------------------------------------------
#endif // CYGONCE_HAL_HAL_ARCH_H
// End of hal_arch.h
