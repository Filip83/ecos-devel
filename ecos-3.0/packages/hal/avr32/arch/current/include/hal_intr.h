//==========================================================================
//
//      hal_intr.h
//
//      HAL Interrupt and clock support
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
// Date:         2012-11-17
// Purpose:      Define Interrupt support
// Description:  The macros defined here provide the HAL APIs for handling
//               both external interrupts and clock interrupts.
//
// Usage:
//              #include <cyg/hal/hal_intr.h>
//              ...
//
//
//####DESCRIPTIONEND####
//
//==========================================================================

#ifndef CYGONCE_HAL_HAL_INTR_H
#define CYGONCE_HAL_HAL_INTR_H


#include <cyg/hal/hal_arch.h>


//--------------------------------------------------------------------------
// AVR32 vectors.

// These are the exception/interrupt causes defined by the hardware.
// These values are the ones to use for HAL_VSR_GET/SET

// Reset
#define CYGNUM_HAL_VECTOR_RESET                      0x00
#define CYGNUM_HAL_VECTOR_UNRECOVERABLE              0x01
#define CYGNUM_HAL_VECTOR_MPU_TLB_MULT_MISS          0x02
#define CYGNUM_HAL_VECTOR_BUS_ERROR_DATA             0x03
#define CYGNUM_HAL_VECTOR_BUS_ERROR_INSTR            0x04
#define CYGNUM_HAL_VECTOR_NMI                        0x05
#define CYGNUM_HAL_VECTOR_INSTR_ADDRESS              0x06
#define CYGNUM_HAL_VECTOR_MPU_ITLB_MISS              0x07
#define CYGNUM_HAL_VECTOR_MPU_ITLB_PROTECTION        0x08
#define CYGNUM_HAL_VECTOR_BREAKEPOINT                0x09
#define CYGNUM_HAL_VECTOR_ILEGAL_OPC                 0x0A
#define CYGNUM_HAL_VECTOR_UNIMPLEMENTED_INSTR        0x0B
#define CYGNUM_HAL_VECTOR_PRIVILEGE_VIOLATION        0x0C
#define CYGNUM_HAL_VECTOR_FPU                        0x0D
#define CYGNUM_HAL_VECTOR_COPROCESSOR_ABSETN         0x0E
#define CYGNUM_HAL_VECTOR_CUPERVISOR_CALL            0x0F
#define CYGNUM_HAL_VECTOR_DATA_ADDRESS_READ          0x10
#define CYGNUM_HAL_VECTOR_DATA_ADDRRESS_WRITE        0x11
#define CYGNUM_HAL_VECTOR_MPU_DTLB_MISS_READ         0x12
#define CYGNUM_HAL_VECTOR_MPU_DTLB_MISS_WRITE        0x13
#define CYGNUM_HAL_VECTOR_MPU_DTLB_PROTECTION_READ   0x14
#define CYGNUM_HAL_VECTOR_MPU_DTLB_PROTECTION_WRITE  0x15
#define CYGNUM_HAL_VECTOR_DTLB_MODIFIED              0x16
#define CYGNUM_HAL_VECTOR_UNIMPLEMENTED              0x17


#define CYGNUM_HAL_VSR_MIN          CYGNUM_HAL_VECTOR_RESET
#define CYGNUM_HAL_VSR_MAX          CYGNUM_HAL_VECTOR_UNIMPLEMENTED
#define CYGNUM_HAL_VSR_COUNT       (CYGNUM_HAL_VSR_MAX - CYGNUM_HAL_VSR_MIN + 1)

// Exception vectors. These are the values used when passed out to an
// external exception handler using cyg_hal_deliver_exception()

#define CYGNUM_HAL_EXCEPTION_SYSTEM_CALL    CYGNUM_HAL_VECTOR_CUPERVISOR_CALL
#define CYGNUM_HAL_EXCEPTION_ILLEGAL_INSTRUCTION \
          CYGNUM_HAL_VECTOR_UNIMPLEMENTED_INSTR

// Min/Max exception numbers and how many there are
#define CYGNUM_HAL_EXCEPTION_MIN                CYGNUM_HAL_VSR_MIN
#define CYGNUM_HAL_EXCEPTION_MAX                CYGNUM_HAL_VSR_MAX
#define CYGNUM_HAL_EXCEPTION_COUNT           \
                 ( CYGNUM_HAL_EXCEPTION_MAX - CYGNUM_HAL_EXCEPTION_MIN + 1 )

#ifndef CYGHWR_HAL_INTERRUPT_VECTORS_DEFINED
    #if  (defined(__AVR32_UC256L3U__) || \
          defined(__AVR32_UC128L3U__) || \
          defined(__AVR32_UC64L3U__) || \
          defined(__AVR32_UC256L4U__) || \
          defined(__AVR32_UC128L4U__) || \
          defined(__AVR32_UC64L4U__))    
    #include <cyg/hal/avr32_L3U_L4U_ISR_Vectors.h>
    #elif (defined(__AVR32_UC3C0512C__)  || \
           defined(__AVR32_UC3C0256C__)  || \
           defined(__AVR32_UC3C0128C__)  || \
           defined(__AVR32_UC3C064C__ )  || \
           defined(__AVR32_UC3C1512C__)  || \
           defined(__AVR32_UC3C1256C__)  || \
           defined(__AVR32_UC3C1128C__)  || \
           defined(__AVR32_UC3C164C__)   || \
           defined(__AVR32_UC3C2512C__)  || \
           defined(__AVR32_UC3C2256C__)  || \
           defined(__AVR32_UC3C2128C__)  || \
           defined(__AVR32_UC3C264C__ ))    
    #include <cyg/hal/avr32_uc3c_ISR_Vectors.h>
    #else
    #error "Unsupported arhitecture. Missing ISR vectors."
    #endif
              
#endif

#ifndef __ASSEMBLER__
#include <pkgconf/hal.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_io.h>

#include <cyg/hal/plf_intr.h>

#define PROGRAM_START_ADDRESS   (AVR32_FLASH_ADDRESS + PROGRAM_START_OFFSET)
#define PROGRAM_START_OFFSET    0x00002000

//! Maximal number of interrupt request lines per group.
#define AVR32_INTC_MAX_NUM_IRQS_PER_GRP      32

//! Number of interrupt priority levels.
#define AVR32_INTC_NUM_INT_LEVELS   \
        (1 << AVR32_INTC_IPR_INTLEVEL_SIZE)

/**
 * \internal
 * \brief Import the _evba symbol from exception.S
 */
extern void *_evba;

/**
 * \internal
 * \brief Import the symbols _int0, _int1, _int2, _int3 defined in exception.S
 */
extern void *_int0, *_int1, *_int2, *_int3;

/**
 * \internal
 * \brief Values to store in the interrupt priority registers for the various
 *  interrupt priority levels.
 */
#define IPR_INT0   ((AVR32_INTC_INT0 << AVR32_INTC_IPR_INTLEVEL_OFFSET) \
			| ((int)&_int0 - (int)&_evba))
#define IPR_INT1   ((AVR32_INTC_INT1 << AVR32_INTC_IPR_INTLEVEL_OFFSET) \
			| ((int)&_int1 - (int)&_evba))
#define IPR_INT2   ((AVR32_INTC_INT2 << AVR32_INTC_IPR_INTLEVEL_OFFSET) \
			| ((int)&_int2 - (int)&_evba))
#define IPR_INT3   ((AVR32_INTC_INT3 << AVR32_INTC_IPR_INTLEVEL_OFFSET) \
			| ((int)&_int3 - (int)&_evba))


//--------------------------------------------------------------------------
// Static data used by HALIPR_INT0

/**
 * \internal
 * \brief Table containing for each interrupt group the number of interrupt
 *  request lines and a pointer to the table of interrupt line handlers.
 */

typedef struct
{
    unsigned int           num_irqs;
    volatile CYG_ADDRWORD *_int_line_handler_table;
} hal_interrupt_handlers_t;

typedef struct
{
    unsigned int           num_irqs;
    volatile CYG_ADDRWORD *_int_line_handler_data;
} hal_interrupt_data_t;

typedef struct
{
    unsigned int           num_irqs;
    volatile CYG_ADDRWORD *_int_line_handler_objects;
} hal_interrupt_objects_t;

externC volatile hal_interrupt_handlers_t 
                hal_interrupt_handlers[AVR32_INTC_NUM_INT_GRPS];

externC volatile hal_interrupt_data_t 
                hal_interrupt_data[AVR32_INTC_NUM_INT_GRPS];

externC volatile hal_interrupt_objects_t 
                hal_interrupt_objects[AVR32_INTC_NUM_INT_GRPS];


// VSR table
externC volatile CYG_ADDRESS    hal_vsr_table[CYGNUM_HAL_VSR_MAX+1];

//--------------------------------------------------------------------------
// Default ISR
// The #define is used to test whether this routine exists, and to allow
// us to call it.

externC cyg_uint32 hal_default_isr(CYG_ADDRWORD vector, CYG_ADDRWORD data);

#define HAL_DEFAULT_ISR hal_default_isr

externC void HAL_init_interrupts(void);
externC void hal_vsr_table_init(void);
externC cyg_uint32 hal_get_interrupt_num(cyg_uint32 int_level);
externC CYG_ADDRESS hal_get_interrupt_object(cyg_uint32 int_level);
externC void hal_interrupt_handler(int pri);
externC cyg_uint64 hal_cal_interrupt_handler(cyg_uint32 int_level);
//--------------------------------------------------------------------------
// Interrupt state storage

//typedef cyg_uint32 CYG_INTERRUPT_STATE;

//--------------------------------------------------------------------------
// Interrupt control macros
#ifndef CYGHWR_HAL_INTERRUPT_ENABLE_DISABLE_RESTORE_DEFINED

// Clear both tick timer and external interrupts in the Supervisor Register
#define HAL_DISABLE_INTERRUPTS(_old_)                     \
    CYG_MACRO_START                                       \
    _old_ = MFSPR(SPR_SR);                                \
    MTSPR(SPR_SR, _old_ | SPR_SR_GIE);                    \
    CYG_MACRO_END

// Enable both tick timer and external interrupts in the Supervisor Register
#define HAL_ENABLE_INTERRUPTS()                           \
    CSRF(SPR_SR_OFFSET_GIE)

// Copy interrupt flags from argument into Supervisor Register
#define HAL_RESTORE_INTERRUPTS(_old_)                     \
    CYG_MACRO_START                                       \
    cyg_uint32 t1,t2;                                     \
    t1 = MFSPR(SPR_SR) & ~(SPR_SR_GIE);                   \
    t2 = (_old_) & (SPR_SR_GIE);                          \
    MTSPR(SPR_SR, t1 | t2);                               \
    CYG_MACRO_END

#define HAL_QUERY_INTERRUPTS( _state_ )                   \
    CYG_MACRO_START                                       \
    _state = MFSPR(SPR_SR);                               \
    CYG_MACRO_END

#endif // CYGHWR_HAL_INTERRUPT_ENABLE_DISABLE_RESTORE_DEFINED

//--------------------------------------------------------------------------
// Routine to execute DSRs using separate interrupt stack

#ifdef  CYGIMP_HAL_COMMON_INTERRUPTS_USE_INTERRUPT_STACK
externC void hal_interrupt_stack_call_pending_DSRs(void);
#define HAL_INTERRUPT_STACK_CALL_PENDING_DSRS() \
    hal_interrupt_stack_call_pending_DSRs()

// these are offered solely for stack usage testing
// if they are not defined, then there is no interrupt stack.
#define HAL_INTERRUPT_STACK_BASE cyg_interrupt_stack_base
#define HAL_INTERRUPT_STACK_TOP  cyg_interrupt_stack
// use them to declare these extern however you want:
//       extern char HAL_INTERRUPT_STACK_BASE[];
//       extern char HAL_INTERRUPT_STACK_TOP[];
// is recommended
#endif

//--------------------------------------------------------------------------
// Vector translation.
// For chained interrupts we only have a single vector though which all
// are passed. For unchained interrupts we have a vector per interrupt.

#ifndef HAL_TRANSLATE_VECTOR

#if defined(CYGIMP_HAL_COMMON_INTERRUPTS_CHAIN)

#define HAL_TRANSLATE_VECTOR(_vector_,_index_) (_index_) = 0

#else

#define HAL_TRANSLATE_VECTOR(_vector_,_index_) (_index_) = (_vector_)

#endif

#endif

//--------------------------------------------------------------------------
// Interrupt and VSR attachment macros

#define HAL_INTERRUPT_IN_USE( _vector_, _state_)                           \
    CYG_MACRO_START                                                        \
    cyg_uint32 _index_ = _vector_ % AVR32_INTC_MAX_NUM_IRQS_PER_GRP;       \
	cyg_uint32 _int_grp_ = _vector_ / AVR32_INTC_MAX_NUM_IRQS_PER_GRP; \
    if( hal_interrupt_handlers[_int_grp_]._int_line_handler_table[_index_] \
	      == (CYG_ADDRESS)HAL_DEFAULT_ISR )                            \
        (_state_) = 0;                                                     \
    else                                                                   \
        (_state_) = 1;                                                     \
    CYG_MACRO_END

#define HAL_INTERRUPT_ATTACH( _vector_, _isr_, _data_, _object_ )          \
{                                                                          \
    cyg_uint32 _index_ = _vector_ % AVR32_INTC_MAX_NUM_IRQS_PER_GRP;       \
	cyg_uint32 _int_grp_ = _vector_ / AVR32_INTC_MAX_NUM_IRQS_PER_GRP; \
    if( hal_interrupt_handlers[_int_grp_]._int_line_handler_table[_index_] \
    == (CYG_ADDRESS)HAL_DEFAULT_ISR )                                      \
    {                                                                      \
        hal_interrupt_handlers[_int_grp_]._int_line_handler_table[_index_] \
		 = (CYG_ADDRESS)_isr_;                                     \
        hal_interrupt_data[_int_grp_]._int_line_handler_data[_index_]      \
		 = (CYG_ADDRWORD)_data_;                                   \
        hal_interrupt_objects[_int_grp_]._int_line_handler_objects[_index_]\
		 = (CYG_ADDRESS)_object_;                                  \
    }                                                                      \
}

#define HAL_INTERRUPT_DETACH( _vector_, _isr_ )                            \
{                                                                          \
    cyg_uint32 _index_ = _vector_ % AVR32_INTC_MAX_NUM_IRQS_PER_GRP;       \
    cyg_uint32 _int_grp_ = _vector_ / AVR32_INTC_MAX_NUM_IRQS_PER_GRP;	   \
                                                                           \
    if( hal_interrupt_handlers[_int_grp_]._int_line_handler_table[_index_] \
	== (CYG_ADDRESS)_isr_ )                                            \
    {                                                                      \
        hal_interrupt_handlers[_int_grp_]._int_line_handler_table[_index_] \
        = (CYG_ADDRESS)HAL_DEFAULT_ISR;;                                   \
        hal_interrupt_data[_int_grp_]._int_line_handler_data[_index_]      \
        = 0;                                                               \
        hal_interrupt_objects[_int_grp_]._int_line_handler_objects[_index_]\
        = 0;                                                               \
    }                                                                      \
}

#define HAL_VSR_GET( _vector_, _pvsr_ )                 \
    *(_pvsr_) = (void (*)())hal_vsr_table[_vector_];


#define HAL_VSR_SET( _vector_, _vsr_, _poldvsr_ ) CYG_MACRO_START         \
    if( (void*)_poldvsr_ != NULL)                                         \
        *(CYG_ADDRESS *)_poldvsr_ = (CYG_ADDRESS)hal_vsr_table[_vector_]; \
    hal_vsr_table[_vector_] = (CYG_ADDRESS)_vsr_;                         \
CYG_MACRO_END

// This is an ugly name, but what it means is: grab the VSR back to eCos
// internal handling, or if you like, the default handler.  But if
// cooperating with GDB and CygMon, the default behaviour is to pass most
// exceptions to CygMon.  This macro undoes that so that eCos handles the
// exception.  So use it with care.

externC void cyg_hal_default_exception_vsr(void);
externC void cyg_hal_default_interrupt_vsr(void);

#define HAL_VSR_SET_TO_ECOS_HANDLER( _vector_, _poldvsr_ ) CYG_MACRO_START  \
    HAL_VSR_SET( _vector_, _vector_ == CYGNUM_HAL_VECTOR_INTERRUPT          \
                              ? (CYG_ADDRESS)cyg_hal_default_interrupt_vsr  \
                              : (CYG_ADDRESS)cyg_hal_default_exception_vsr, \
                 _poldvsr_ );                                               \
CYG_MACRO_END

//--------------------------------------------------------------------------
// Interrupt controller access

#ifndef CYGHWR_HAL_INTERRUPT_CONTROLLER_ACCESS_DEFINED
#define CYGHWR_HAL_INTERRUPT_CONTROLLER_ACCESS_DEFINED

// AVR32 do not support masking separate interrupts
// Interrupts from peripherals can be masked, unmasked
// and acknowledged from theier registers
// Mask (disable) interrupts from specified source
// is not supported
#define HAL_INTERRUPT_MASK( _vector_ )            CYG_EMPTY_STATEMENT

// Allow interrupts from specified source
#define HAL_INTERRUPT_UNMASK( _vector_ )          CYG_EMPTY_STATEMENT

// Reset interrupt request in the PIC for specified device
#define HAL_INTERRUPT_ACKNOWLEDGE( _vector_ )     CYG_EMPTY_STATEMENT

#define HAL_INTERRUPT_CONFIGURE( _vector_, _level_, _up_ ) CYG_EMPTY_STATEMENT

//#define HAL_INTERRUPT_SET_LEVEL( _vector_, _level_ )  CYG_EMPTY_STATEMENT

#define HAL_INTERRUPT_SET_LEVEL( _vector_, _level_ )                 \
    cyg_uint32 int_grp = _vector_ / AVR32_INTC_MAX_NUM_IRQS_PER_GRP; \
        if (_level_ == AVR32_INTC_INT0) {                            \
                AVR32_INTC.ipr[int_grp] = IPR_INT0;                  \
        } else if (_level_ == AVR32_INTC_INT1) {                     \
                AVR32_INTC.ipr[int_grp] = IPR_INT1;                  \
        } else if (_level_ == AVR32_INTC_INT2) {                     \
                AVR32_INTC.ipr[int_grp] = IPR_INT2;                  \
        } else {                                                     \
                AVR32_INTC.ipr[int_grp] = IPR_INT3;                  \
        }                                                            \

#endif

//--------------------------------------------------------------------------
// Clock control.

externC CYG_WORD32 cyg_hal_clock_period;
#define CYGHWR_HAL_CLOCK_PERIOD_DEFINED

// Start tick timer interrupts
#define HAL_CLOCK_INITIALIZE( _period_ )        \
CYG_MACRO_START                                 \
{                                               \
    int ttmr_new = _period_;                    \
    MTSPR(SPR_COMPARE, ttmr_new);               \
    cyg_hal_clock_period = _period_;            \
}                                               \
CYG_MACRO_END

// Acknowledge clock timer interrupt
#define HAL_CLOCK_RESET( _vector_, _period_ )   \
CYG_MACRO_START                                 \
    int ttmr_new = _period_;                    \
    MTSPR(SPR_COMPARE, ttmr_new);               \
CYG_MACRO_END

// Read the current value of the tick timer
#define HAL_CLOCK_READ( _pvalue_ )              \
CYG_MACRO_START                                 \
    *(_pvalue_) = MFSPR(SPR_COUNT);             \
CYG_MACRO_END

#if defined(CYGVAR_KERNEL_COUNTERS_CLOCK_LATENCY) && \
    !defined(HAL_CLOCK_LATENCY)
#define HAL_CLOCK_LATENCY( _pvalue_ )                   \
CYG_MACRO_START                                         \
    register CYG_WORD32 _cval_;                         \
    HAL_CLOCK_READ(&_cval_);                            \
    *(_pvalue_) = _cval_ - cyg_hal_clock_period;        \
CYG_MACRO_END
#endif


//--------------------------------------------------------------------------
// Microsecond delay function provided in hal_misc.c
externC void hal_delay_us(int us);

#define HAL_DELAY_US(n)          hal_delay_us(n)

#endif /* #ifndef __ASSEMBLER__ */

//--------------------------------------------------------------------------
#endif // ifndef CYGONCE_HAL_HAL_INTR_H
// End of hal_intr.h
