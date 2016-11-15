//==========================================================================
//
//      hal_misc.c
//
//      HAL miscellaneous functions
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
// Date:         2012-11-15
// Purpose:      HAL miscellaneous functions
// Description:  This file contains miscellaneous functions provided by the
//               HAL.
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <pkgconf/hal.h>

#include <cyg/hal/avr32/io.h>
#include <cyg/infra/cyg_type.h>         // Base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#define CYGARC_HAL_COMMON_EXPORT_CPU_MACROS
#include <cyg/hal/hal_arch.h>           // architectural definitions
#include <cyg/hal/hal_intr.h>           // Interrupt handling
#include <cyg/hal/hal_if.h>             // hal_ctrlc_isr()

#include <cyg/hal/utils/compiler.h>
#include <cyg/hal/utils/preprocessor/mrepeat.h>

#include CYGHWR_MEMORY_LAYOUT_H



/*------------------------------------------------------------------------*/
/* If required, define a variable to store the clock period.              */

#ifdef CYGHWR_HAL_CLOCK_PERIOD_DEFINED

CYG_WORD32 cyg_hal_clock_period;

#endif

/*------------------------------------------------------------------------*/
/* Macros to generate approprite interrupt table and associted            */
/* object tables. Code is taken from avr afs library                      */
#  define DECL_INT_LINE_HANDLER_TABLE(GRP, unused) \
volatile static CYG_ADDRWORD \
	_int_line_handler_table_##GRP[Max(AVR32_INTC_NUM_IRQS_PER_GRP##GRP, 1)];
MREPEAT(AVR32_INTC_NUM_INT_GRPS, DECL_INT_LINE_HANDLER_TABLE, ~);
#undef DECL_INT_LINE_HANDLER_TABLE

#  define DECL_INT_LINE_DATA_TABLE(GRP, unused) \
volatile static CYG_ADDRWORD \
_int_line_data_table_##GRP[Max(AVR32_INTC_NUM_IRQS_PER_GRP##GRP, 1)];
MREPEAT(AVR32_INTC_NUM_INT_GRPS, DECL_INT_LINE_DATA_TABLE, ~);
#undef DECL_INT_LINE_DATA_TABLE

#  define DECL_INT_LINE_OBJECTS_TABLE(GRP, unused) \
volatile static CYG_ADDRWORD \
_int_line_objects_table_##GRP[Max(AVR32_INTC_NUM_IRQS_PER_GRP##GRP, 1)];
MREPEAT(AVR32_INTC_NUM_INT_GRPS, DECL_INT_LINE_OBJECTS_TABLE, ~);
#undef DECL_INT_LINE_OBJECTS_TABLE

/**
 * \internal
 * \brief Table containing for each interrupt group the number of interrupt
 *  request lines and a pointer to the table of interrupt line handlers.
 */

volatile hal_interrupt_handlers_t 
    hal_interrupt_handlers[AVR32_INTC_NUM_INT_GRPS] =
{
#define INSERT_INT_LINE_HANDLER_TABLE(GRP, unused) \
	{AVR32_INTC_NUM_IRQS_PER_GRP##GRP, _int_line_handler_table_##GRP},
	MREPEAT(AVR32_INTC_NUM_INT_GRPS, INSERT_INT_LINE_HANDLER_TABLE, ~)
#undef INSERT_INT_LINE_HANDLER_TABLE
};

volatile hal_interrupt_data_t hal_interrupt_data[AVR32_INTC_NUM_INT_GRPS] =
{
#define INSERT_INT_LINE_DATA_TABLE(GRP, unused) \
{AVR32_INTC_NUM_IRQS_PER_GRP##GRP, _int_line_data_table_##GRP},
	MREPEAT(AVR32_INTC_NUM_INT_GRPS, INSERT_INT_LINE_DATA_TABLE, ~)
	#undef INSERT_INT_LINE_DATA_TABLE
};

volatile hal_interrupt_objects_t 
    hal_interrupt_objects[AVR32_INTC_NUM_INT_GRPS] =
{
#define INSERT_INT_LINE_OBJECTS_TABLE(GRP, unused) \
{AVR32_INTC_NUM_IRQS_PER_GRP##GRP, _int_line_objects_table_##GRP},
	MREPEAT(AVR32_INTC_NUM_INT_GRPS, INSERT_INT_LINE_OBJECTS_TABLE, ~)
	#undef INSERT_INT_LINE_OBJECTS_TABLE
};

cyg_uint32 
cyg_hal_exception_handler(HAL_SavedRegisters *regs, cyg_uint32 vector);

volatile CYG_ADDRESS    hal_vsr_table[CYGNUM_HAL_EXCEPTION_COUNT] =
{ 
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL,
    (CYG_ADDRESS)NULL
};

volatile CYG_ADDRWORD hal_virtual_vector_table[CYGNUM_CALL_IF_TABLE_SIZE];

typedef cyg_uint32 (*hal_isr)(CYG_ADDRWORD, CYG_ADDRWORD);


/*------------------------------------------------------------------------*/
/* Function find function to call for interrupt vector and call it.       */
/* The ISR function return value and associated ISR object are returned   */
/* as 64bit value                                                         */
cyg_uint64 hal_cal_interrupt_handler(cyg_uint32 int_level)
{
    cyg_uint32 isr_ret = 0;
    CYG_ADDRESS object = 0;
    cyg_uint32 int_grp = AVR32_INTC.icr[AVR32_INTC_INT3 - int_level];
    cyg_uint32 int_req = AVR32_INTC.irr[int_grp];

    if(int_req)
    {
        hal_isr fnc = 
            (hal_isr)hal_interrupt_handlers[int_grp]._int_line_handler_table[32
            - clz(int_req) - 1];

        isr_ret = fnc((int_grp << 5) | (int_req - 1),
            hal_interrupt_data[int_grp]._int_line_handler_data[32
            - clz(int_req) - 1]
        );

        object = hal_interrupt_objects[int_grp]._int_line_handler_objects[32
            - clz(int_req) - 1];

    }

    return ((cyg_uint64)isr_ret << 32) | object;
}

cyg_uint32 hal_get_interrupt_num(cyg_uint32 int_level)
{
    cyg_uint32 int_grp = AVR32_INTC.icr[AVR32_INTC_INT3 - int_level];
    cyg_uint32 int_req = AVR32_INTC.irr[int_grp];
    return (int_grp << 8) | int_req;
}

CYG_ADDRESS hal_get_interrupt_object(cyg_uint32 int_level)
{
    cyg_uint32 int_grp = 0;
    cyg_uint32 int_req = 1;
    return hal_interrupt_objects[int_grp]._int_line_handler_objects[32
            - clz(int_req) - 1];
}

void HAL_init_interrupts(void)
{
    cyg_uint32 int_grp, int_req;

    MTSPR(SPR_EVBA, (cyg_int32)&_evba );

    // For all interrupt groups,
    for (int_grp = 0; int_grp < AVR32_INTC_NUM_INT_GRPS; int_grp++)
    {
        // For all interrupt request lines of each group,
        for (int_req = 0;
                int_req < hal_interrupt_handlers[int_grp].num_irqs;
                int_req++)
        {
            /* Assign _unhandled_interrupt as the default interrupt
            handler. */
            hal_interrupt_handlers[int_grp]
                    ._int_line_handler_table[int_req]
                            = (CYG_ADDRESS)&hal_default_isr;

            hal_interrupt_data[int_grp]._int_line_handler_data[int_req] = 0;

            hal_interrupt_objects[int_grp]._int_line_handler_objects[int_req]
                = (CYG_ADDRESS)NULL;
        }

        /* Set the interrupt group priority register to its default
        value.
        By default, all interrupt groups are linked to the interrupt
        priority level 0 and to the interrupt vector _int0. */
        AVR32_INTC.ipr[int_grp] = IPR_INT0;
    }
}

/*------------------------------------------------------------------------*/
/* First level C exception handler.                                       */

externC void __handle_exception (void);

externC HAL_SavedRegisters *_hal_registers;

#if defined(CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS)
externC void* volatile __mem_fault_handler;
#endif

/*------------------------------------------------------------------------*/
/* default Exception                                                      */
cyg_uint32 cyg_hal_exception_handler(HAL_SavedRegisters *regs, cyg_uint32 vector)
{
#if defined(CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS)

    // If we caught an exception inside the stubs, see if we were expecting it
    // and if so jump to the saved address
    if (__mem_fault_handler) {
        regs->pc = (CYG_HAL_AVR32_REG)(signed long)__mem_fault_handler;
        return 0; // Caught an exception inside stubs
    }

    // Set the pointer to the registers of the current exception
    // context. At entry the GDB stub will expand the
    // HAL_SavedRegisters structure into a (bigger) register array.
    _hal_registers = regs;
    __handle_exception();

#elif defined(CYGFUN_HAL_COMMON_KERNEL_SUPPORT) && defined(CYGPKG_HAL_EXCEPTIONS)

    // We should decode the vector and pass a more appropriate
    // value as the second argument. For now we simply pass a
    // pointer to the saved registers. We should also divert
    // breakpoint and other debug vectors into the debug stubs.
    // Only NMI exception is delived to the krenel.
    if(vector == CYGNUM_HAL_VECTOR_NMI)
        cyg_hal_deliver_exception(vector, (CYG_ADDRWORD)regs );
    else
    {
        switch(vector)
        {
        case CYGNUM_HAL_VECTOR_UNRECOVERABLE:
            CYG_FAIL("Unrecoverable Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_TLB_MULT_MISS:
            CYG_FAIL("MPU_TLB_MULT_MISS Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_BUS_ERROR_DATA:
            CYG_FAIL("Bus error data Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_BUS_ERROR_INSTR:
            CYG_FAIL("Bus error Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_NMI:
            CYG_FAIL("NMI Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_INSTR_ADDRESS:
            CYG_FAIL("Instruction address Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_ITLB_MISS:
            CYG_FAIL("MPU ITLB MISS Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_ITLB_PROTECTION:
            CYG_FAIL("MPU ITLB PROTECTION Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_BREAKEPOINT:
            CYG_FAIL("Breakepoint Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_ILEGAL_OPC:
            CYG_FAIL("Ilegal opc Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_UNIMPLEMENTED_INSTR:
            CYG_FAIL("Unimplemented instruction Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_PRIVILEGE_VIOLATION:
            CYG_FAIL("Privilege violation Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_FPU:
            CYG_FAIL("FPU Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_COPROCESSOR_ABSETN:
            CYG_FAIL("Coprocessor absent Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_CUPERVISOR_CALL:
            CYG_FAIL("Supervisor call Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_DATA_ADDRESS_READ:
            CYG_FAIL("Data address read Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_DATA_ADDRRESS_WRITE:
            CYG_FAIL("Data address write Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_DTLB_MISS_READ:
            CYG_FAIL("MPU DTLB MISS READ Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_DTLB_MISS_WRITE:
            CYG_FAIL("MPU DTLB MISS WRITE Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_DTLB_PROTECTION_READ:
            CYG_FAIL("MPU DTLB PROTECTION READ Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_MPU_DTLB_PROTECTION_WRITE:
            CYG_FAIL("MPU DTLB PROTECTION WRITE Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_DTLB_MODIFIED :
            CYG_FAIL("MPU DTLB MODIFIED Exception!!!\n");
            break;
        case CYGNUM_HAL_VECTOR_UNIMPLEMENTED:
            CYG_FAIL("Unimplemented vector Exception!!!\n");
            break;
        default:
            CYG_FAIL("Unknown Exception!!!\n");
            break;
        }
    }
#else

    CYG_FAIL("Exception!!!");

#endif
    return 0;
}

/*------------------------------------------------------------------------*/
/* default ISR                                                            */

externC cyg_uint32
hal_arch_default_isr(CYG_ADDRWORD vector, CYG_ADDRWORD data)
{
    CYG_FAIL("Exception/ISR!!!");
    return 0;
}

/*------------------------------------------------------------------------*/
// Come here if interrupt triggered, but no apparent cause
void hal_spurious_IRQ(HAL_SavedRegisters *regs) CYGBLD_ATTRIB_WEAK;
void
hal_spurious_IRQ(HAL_SavedRegisters *regs)
{
#if defined(CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS)
    cyg_hal_exception_handler(regs);
#else
    CYG_FAIL("Spurious interrupt!!");
#endif
}
/*------------------------------------------------------------------------*/

#ifdef CYGSEM_HAL_STOP_CONSTRUCTORS_ON_FLAG
cyg_bool cyg_hal_stop_constructors;
#endif

typedef void (*pfunc) (void);
extern pfunc __CTOR_LIST__[];
extern pfunc __CTOR_END__[];

void
cyg_hal_invoke_constructors(void)
{
#ifdef CYGSEM_HAL_STOP_CONSTRUCTORS_ON_FLAG
    static pfunc *p = &__CTOR_END__[-1];

    cyg_hal_stop_constructors = 0;
    for (; p >= __CTOR_LIST__; p--) {
        (*p) ();
        if (cyg_hal_stop_constructors) {
            p--;
            break;
        }
    }
#else
    pfunc *p;

    for (p = &__CTOR_END__[-1]; p >= __CTOR_LIST__; p--)
        (*p) ();
#endif

} // cyg_hal_invoke_constructors()

/*------------------------------------------------------------------------*/
/* Determine the index of the ls bit of the supplied mask.                */
//TODO can we use l.ff1 ?
cyg_uint32 hal_lsbit_index(cyg_uint32 mask)
{
    cyg_uint32 n = mask;

    static const signed char tab[64] =
    { -1, 0, 1, 12, 2, 6, 0, 13, 3, 0, 7, 0, 0, 0, 0, 14, 10,
      4, 0, 0, 8, 0, 0, 25, 0, 0, 0, 0, 0, 21, 27 , 15, 31, 11,
      5, 0, 0, 0, 0, 0, 9, 0, 0, 24, 0, 0 , 20, 26, 30, 0, 0, 0,
      0, 23, 0, 19, 29, 0, 22, 18, 28, 17, 16, 0
    };

    n &= ~(n-1UL);
    n = (n<<16)-n;
    n = (n<<6)+n;
    n = (n<<4)+n;

    return tab[n>>26];
}

/*------------------------------------------------------------------------*/
/* Determine the index of the ms bit of the supplied mask.                */
//TODO can we use l.fl1 ?
cyg_uint32 hal_msbit_index(cyg_uint32 mask)
{
    cyg_uint32 x = mask;
    cyg_uint32 w;

    /* Phase 1: make word with all ones from that one to the right */
    x |= x >> 16;
    x |= x >> 8;
    x |= x >> 4;
    x |= x >> 2;
    x |= x >> 1;

    /* Phase 2: calculate number of "1" bits in the word        */
    w = (x & 0x55555555) + ((x >> 1) & 0x55555555);
    w = (w & 0x33333333) + ((w >> 2) & 0x33333333);
    w = w + (w >> 4);
    w = (w & 0x000F000F) + ((w >> 8) & 0x000F000F);
    return (cyg_uint32)((w + (w >> 16)) & 0xFF) - 1;

}

/*------------------------------------------------------------------------*/
/* Delay for some number of useconds.                                     */
void
hal_delay_us(int us)
{
    cyg_uint32 val1, val2;
    int diff;
    long usticks;
    long ticks;

    // Calculate the number of counter register ticks per microsecond.

    usticks = (CYGNUM_HAL_RTC_PERIOD * CYGNUM_HAL_RTC_DENOMINATOR) / 1000000;

    // Make sure that the value is not zero.
    if( usticks == 0 ) usticks = 1;

    while( us > 0 )
    {
        int us1 = us;

        // Wait in bursts of less than 10000us to avoid any overflow
        // problems in the multiply.
        if( us1 > 10000 )
            us1 = 10000;

        us -= us1;

        ticks = us1 * usticks;

        HAL_CLOCK_READ(&val1);
        while (ticks > 0) {
            do {
                HAL_CLOCK_READ(&val2);
            } while (val1 == val2);
            diff = val2 - val1;
            if (diff < 0) diff += CYGNUM_HAL_RTC_PERIOD;
            ticks -= diff;
            val1 = val2;
        }
    }
}

/*------------------------------------------------------------------------*/
void hal_arch_program_new_stack(void *_func)
{
    // Used in GDB stubs whose are not implemented
   /* externC void hal_program_new_stack( void *func, CYG_ADDRESS addr);
    hal_program_new_stack( (void *)_func,
                   (CYGMEM_REGION_ram + CYGMEM_REGION_ram_SIZE - sizeof(CYG_ADDRESS)) & ~7 );*/
}

/*------------------------------------------------------------------------*/
/* Idle thread action                                                     */

#include <cyg/infra/diag.h>

void hal_idle_thread_action( cyg_uint32 count )
{
    SLEEP(AVR32_PM_SMODE_IDLE);
}

/*------------------------------------------------------------------------*/

//==========================================================================
// When compiling C++ code with static objects the compiler
// inserts a call to __cxa_atexit() with __dso_handle as one of the
// arguments. __cxa_atexit() would normally be provided by glibc, and
// __dso_handle is part of crtstuff.c. eCos applications
// are linked rather differently, so either a differently-configured
// compiler is needed or dummy versions of these symbols should be
// provided. If these symbols are not actually used then providing
// them is still harmless, linker garbage collection will remove them.
void __cxa_atexit(void (*arg1)(void*), void* arg2, void* arg3)
{
}

void *__dso_handle = (void*) &__dso_handle;

#include <sys/reent.h>


/* End of hal_misc.c                                                      */
