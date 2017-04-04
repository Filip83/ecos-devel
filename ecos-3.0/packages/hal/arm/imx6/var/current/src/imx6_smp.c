//==========================================================================
//
//      imx6_smp.c
//
//      HAL SMP implementation
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
// Author(s):    nickg
// Contributors: nickg
// Date:         2001-08-03
// Purpose:      HAL SMP implementation
// Description:  This file contains SMP support functions.
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <pkgconf/hal.h>

#ifdef CYGPKG_HAL_SMP_SUPPORT

#ifdef CYGPKG_KERNEL
#include <pkgconf/kernel.h>
#endif

#include <cyg/infra/cyg_type.h>         // Base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros
#include <cyg/infra/diag.h>

#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_smp.h>
#include <cyg/hal/drv_api.h>

#include <cyg/hal/cortex_a9.h>
#include <cyg/hal/var_intr.h>
#include <cyg/hal/var_spinlock.h>
#include <cyg/hal/var_utility.h>
#include <cyg/hal/var_timer.h>

/*------------------------------------------------------------------------*/

// If a wait for ram initialization is added, set this to 1 in cyg_hal_cpu_start.
//static volatile CYG_WORD32 init_deasserted = 1;

// The i386 defines these in the .ld file.
//static CYG_WORD32 cyg_hal_smp_cpu_sync_flag[HAL_SMP_CPU_MAX];
//static CYG_WORD32 cyg_hal_smp_cpu_sync[HAL_SMP_CPU_MAX];
//static void (*cyg_hal_smp_cpu_entry[HAL_SMP_CPU_MAX])(void*);
static CYG_WORD32 cyg_hal_smp_cpu_running[HAL_SMP_CPU_MAX];

/*------------------------------------------------------------------------*/

__externC HAL_SMP_CPU_TYPE cyg_hal_smp_cpu(void)
{
    return hal_cpu_get_current();
}

__externC void cyg_kernel_smp_startup(void);
//__externC void cyg_hal_smp_startup(void);

// This starts the kernel for CPUs 1-3. It is called after setting the
// registers for a CPU and then enabling it. It is called by the CPU
// it self, so function calls with the implied CPU are ok. Therefore,
// much of the CPU setup is done in this function.
__externC void cyg_hal_smp_start(void *arg)
{
    HAL_SMP_CPU_TYPE cpu = HAL_SMP_CPU_THIS();

    cyg_hal_smp_cpu_running[cpu] = 1;

    // Allow interrupts from the CPU (set priority mask, disable preemption, enable gic interface)
    hal_interrupt_init_cpu();

    // Enable interrupts via the CPSR register.
    hal_set_interrupt_state(true);

    cyg_kernel_smp_startup();
}

// This is called by the kernel for CPU 1-3 to get them started.
// Do not use function calls that operate on the current CPU,
// if the intent is to carry out the action on the CPU passed 
// to this call.
__externC void cyg_hal_cpu_start( HAL_SMP_CPU_TYPE cpu )
{
    if (cpu == 0)
    {
        cyg_hal_smp_cpu_running[cpu] = 1;

		// Join SMP
		hal_scu_join_smp();

		// Enable interrupts via the CPSR register.
        hal_set_interrupt_state(true);
    }
    else
    {
        hal_delay_us( 100 );

        hal_cpu_start_secondary((cyg_uint8)cpu, &cyg_hal_smp_start, 0);

        // Some time for the CPU to get through initialization
        cyg_uint32 i;
        for (i = 0; i < 10; i++)
            hal_delay_us( 100 );
    }

}

// This will be called from platform_int.c. It just marks the CPU started.
__externC void cyg_hal_smp_cpu_start_first(void)
{
    CYG_REPORT_FUNCTION();

    HAL_SMP_CPU_TYPE cpu;

    cpu  = HAL_SMP_CPU_THIS();
    cyg_hal_cpu_start(cpu);

    CYG_REPORT_RETURN();
}


/*------------------------------------------------------------------------*/
// SMP message buffers.
// SMP CPUs pass messages to eachother via a small circular buffer
// protected by a spinlock. Each message is a single 32 bit word with
// a type code in the top 4 bits and any argument in the remaining
// 28 bits.

#define SMP_MSGBUF_SIZE 4

static struct smp_msg_t
{
    spinlock_t                  lock;           // protecting spinlock
    volatile CYG_WORD32         msgs[SMP_MSGBUF_SIZE]; // message buffer
    volatile CYG_WORD32         head;           // head of list
    volatile CYG_WORD32         tail;           // tail of list
    volatile CYG_WORD32         reschedule;     // reschedule request
    volatile CYG_WORD32         timeslice;      // timeslice request
    volatile CYG_WORD32         reschedule_count;  // number of reschedules
    volatile CYG_WORD32         timeslice_count;   // number of timeslices
} smp_msg[HAL_SMP_CPU_MAX];

/*------------------------------------------------------------------------*/


__externC void cyg_hal_smp_init(void)
{
    cyg_int32 i;

    // Enable snoop control
    hal_scu_enable();

    for (i=0; i<HAL_SMP_CPU_MAX; i++)
        hal_spinlock_init(&(smp_msg[i].lock));
}

/*------------------------------------------------------------------------*/
// Pass a message to another CPU.

__externC void cyg_hal_cpu_message( HAL_SMP_CPU_TYPE cpu,
                                    CYG_WORD32 msg,
                                    CYG_WORD32 arg,
                                    CYG_WORD32 wait)
{
    CYG_REPORT_FUNCTION();

#if 1
    // return;
    struct smp_msg_t *m = &smp_msg[cpu];
//    struct smp_msg_t *m1 = &smp_msg[1];
//    struct smp_msg_t *m = &smp_msg[0];
    int i;
    CYG_INTERRUPT_STATE old_ints;
 
    // This only works because we are assigning the vector by cpu number.
    HAL_DISABLE_INTERRUPTS( old_ints );
    
    // Get access to the message buffer for the selected CPU
    HAL_SPINLOCK_SPIN( m->lock );
 
    if( msg == HAL_SMP_MESSAGE_RESCHEDULE )
        m->reschedule = true;
    else if( msg == HAL_SMP_MESSAGE_TIMESLICE )
        m->timeslice = true;
    else
    {
        CYG_WORD32 next = (m->tail + 1) & (SMP_MSGBUF_SIZE-1);

        // If the buffer is full, wait for space to appear in it.
        // This should only need to be done very rarely.
    
        while( next == m->head )
        {
            HAL_SPINLOCK_CLEAR( m->lock );
            for( i = 0; i < 1000; i++ );
            HAL_SPINLOCK_SPIN( m->lock );        
        }

        m->msgs[m->tail] = msg | arg;

        m->tail = next;
    }
    
    // Now send an interrupt to the CPU.
        
    if( cyg_hal_smp_cpu_running[cpu] )
    {
        //int cpuCount = HAL_SMP_CPU_COUNT();
        //int cpuCount = hal_cpu_get_cores();
        //if (cpu < (cpuCount - 1))
        //{
            //hal_gic_send_sgi(CYGNUM_HAL_INTERRUPT_SW_0, 0xF & (1 << (cpu + 1)), UseTargetList);
//        hal_gic_send_sgi(CYGNUM_HAL_INTERRUPT_SW_0, 1 << cpu, UseTargetList);
        hal_interrupt_send_sgi(CYGNUM_HAL_SMP_CPU_INTERRUPT_VECTOR(cpu), 1 << cpu, UseTargetList);

//        hal_gic_send_sgi(CYGNUM_HAL_INTERRUPT_SW_0, 1, kGicSgiFilter_UseTargetList);
        //}
    }

    HAL_SPINLOCK_CLEAR( m->lock );

    // If we are expected to wait for the command to complete, then
    // spin here until it does. We actually wait for the destination
    // CPU to empty its input buffer. So we might wait for messages
    // from other CPUs as well. But this is benign.
    
    while(wait)
    {
        for( i = 0; i < 1000; i++ );
        
        HAL_SPINLOCK_SPIN( m->lock );

        if( m->head == m->tail )
            wait = false;
        
        HAL_SPINLOCK_CLEAR( m->lock );

    } 

    HAL_RESTORE_INTERRUPTS( old_ints );
#endif    

    CYG_REPORT_RETURN();
}

/*------------------------------------------------------------------------*/

externC void hal_debug_gic(void);
externC cyg_uint32 hal_gic_pending(cyg_uint32 irqID);
externC cyg_uint32 hal_gic_active(cyg_uint32 irqID);
externC cyg_uint32 hal_gic_enabled(cyg_uint32 irqID);
externC cyg_uint8 hal_gic_priority(cyg_uint32 irqID);
externC cyg_uint32 hal_gic_priority_mask (void);
externC cyg_uint32 hal_gic_running_priority (void);


__externC CYG_WORD32 cyg_hal_cpu_message_isr( CYG_WORD32 vector, CYG_ADDRWORD data )
{
    HAL_SMP_CPU_TYPE me = HAL_SMP_CPU_THIS();
    struct smp_msg_t *m = &smp_msg[me];
    CYG_WORD32 ret = 1;
//    CYG_INTERRUPT_STATE old_ints;

    cyg_drv_interrupt_mask(vector);
//    HAL_DISABLE_INTERRUPTS( old_ints );

    HAL_SPINLOCK_SPIN( m->lock );

    // First, acknowledge the interrupt.
    
    cyg_drv_interrupt_acknowledge(vector);
    //HAL_INTERRUPT_ACKNOWLEDGE( (me << 10) | vector );

    if (m->reschedule)
    	m->reschedule_count++;
    if (m->timeslice)
    	m->timeslice_count++;
    if( m->reschedule || m->timeslice )
        ret |= 2;               // Ask for the DSR to be called.
    
    // Now pick messages out of the buffer and handle them
    
    while( m->head != m->tail )
    {
        CYG_WORD32 msg = m->msgs[m->head];

        switch( msg & HAL_SMP_MESSAGE_TYPE )
        {
        case HAL_SMP_MESSAGE_RESCHEDULE:
            ret |= 2;           // Ask for the DSR to be called.
            break;
        case HAL_SMP_MESSAGE_MASK:
            // Mask the supplied vector
//            cyg_hal_interrupt_set_mask( msg&HAL_SMP_MESSAGE_ARG, false );
            break;
        case HAL_SMP_MESSAGE_UNMASK:
            // Unmask the supplied vector
//            cyg_hal_interrupt_set_mask( msg&HAL_SMP_MESSAGE_ARG, true );
            break;
        case HAL_SMP_MESSAGE_REVECTOR:
            // Deal with a change of CPU assignment for a vector. We
            // only actually worry about what happens when the vector
            // is changed to some other CPU. We just mask the
            // interrupt locally.
//            if( hal_interrupt_cpu[msg&HAL_SMP_MESSAGE_ARG] != me )
//                cyg_hal_interrupt_set_mask( msg&HAL_SMP_MESSAGE_ARG, false );
            break;
        }

        // Update the head pointer after handling the message, so that
        // the wait in cyg_hal_cpu_message() completes after the action
        // requested.
        
        m->head = (m->head + 1) & (SMP_MSGBUF_SIZE-1);
    }
    HAL_SPINLOCK_CLEAR( m->lock );
    cyg_drv_interrupt_unmask(vector);
//    HAL_RESTORE_INTERRUPTS( old_ints );

    return ret;
}

/*------------------------------------------------------------------------*/
// CPU message DSR.
// This is only executed if the message was
// HAL_SMP_MESSAGE_RESCHEDULE. It calls up into the kernel to effect a
// reschedule.

__externC void cyg_scheduler_set_need_reschedule(void);
__externC void cyg_scheduler_timeslice_cpu(void);

__externC CYG_WORD32 cyg_hal_cpu_message_dsr( CYG_WORD32 vector, CYG_ADDRWORD data )
{
    HAL_SMP_CPU_TYPE me = HAL_SMP_CPU_THIS();
    struct smp_msg_t *m = &smp_msg[me];
    CYG_WORD32 reschedule;
#ifdef CYGSEM_KERNEL_SCHED_TIMESLICE
    CYG_WORD32 timeslice;
#endif
//    CYG_INTERRUPT_STATE old_ints;

    cyg_drv_interrupt_mask(vector);
//    HAL_DISABLE_INTERRUPTS( old_ints );
    HAL_SPINLOCK_SPIN( m->lock );
    
    reschedule = m->reschedule;
#ifdef CYGSEM_KERNEL_SCHED_TIMESLICE
    timeslice = m->timeslice;
    m->reschedule = m->timeslice = false;
#else
    m->reschedule = false;
#endif

    HAL_SPINLOCK_CLEAR( m->lock );
    cyg_drv_interrupt_unmask(vector);
//    HAL_RESTORE_INTERRUPTS( old_ints );
        
    if( reschedule )
    {     
        cyg_scheduler_set_need_reschedule();
    }
#ifdef CYGSEM_KERNEL_SCHED_TIMESLICE
    if( timeslice )
    {    
        cyg_scheduler_timeslice_cpu();
    }
#endif
    return 0;
}

#endif // CYGPKG_HAL_SMP_SUPPORT

/*------------------------------------------------------------------------*/
/* End of imx6_smp.c                                                      */
