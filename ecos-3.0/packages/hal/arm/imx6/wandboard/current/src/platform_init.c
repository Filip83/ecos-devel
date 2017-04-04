/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pkgconf/hal.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_smp.h>
#include <cyg/hal/cortex_a9.h>
#include <cyg/hal/var_mmu.h>
#include <cyg/hal/var_ccm.h>
#include <cyg/hal/var_timer.h>
#include <cyg/hal/var_intr.h>

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

extern void hal_enet_init(void);

// This init is designed to be called by CPU 0 once at startup. Other CPUs should
// not call this.

void platform_init(void)
{
    // Enable interrupts. Until this point, the startup code has left interrupts disabled.
    // Once this init is called, CPU 0 has its interrupt interface enabled (because CPU 0 called this)
    // and the distributor is enabled. This is independent of the CPSR IRQ/FIQ enable, which
    // operates independently for each CPU. This does not setup any other CPU interfaces.
    // The CPU priority will be at the lowest priority so any interrupt will get through.

    // Enable GIC (disable distributor, clear pending interrupts, allow interrupts from CPU, enable distributor)
    // Allow interrupts from the CPU (set priority mask, disable preemption, enable gic interface)
    hal_interrupt_init();
    
    // Initialize clock sources, dividers, ... 
    hal_ccm_init();
    
    // Configure the EPIT timer used for system delay function. 
    hal_system_time_init();

    // Enable the vector table so that the debug console works.
    hal_if_init();

#ifdef CYGPKG_DEVS_ETH_FREESCALE_ENET
	hal_enet_init();
#endif

#ifdef CYGPKG_HAL_SMP_SUPPORT
    // Setup basic SMP support.
    cyg_hal_smp_init();

    // Mark first CPU started.
    cyg_hal_smp_cpu_start_first();
    
#endif
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
