/*
 * Copyright (c) 2011-2012, Freescale Semiconductor, Inc.
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

#ifndef __EPIT_H__
#define __EPIT_H__

//#include <cyg/hal/var_timer.h>

#define FREE_RUNNING    0
#define SET_AND_FORGET  1
#define IRQ_MODE 1
#define EPIT_IRQS(x) ( (x) == HW_EPIT1 ? CYGNUM_HAL_INTERRUPT_EPIT1 : (x) == HW_EPIT2 ? CYGNUM_HAL_INTERRUPT_EPIT2 : 0xFFFFFFFF)


#if defined(__cplusplus)
extern "C" {
#endif

void hal_epit_init(cyg_uint32 instance, cyg_uint32 clock_src, cyg_uint32 prescaler,
               cyg_uint32 reload_mode, cyg_uint32 load_val, cyg_uint32 low_power_mode);
void hal_epit_setup_interrupt(cyg_uint32 instance, void (*irq_subroutine)(void), bool enableIt);
void hal_epit_counter_enable(cyg_uint32 instance, cyg_uint32 load_val, cyg_uint32 irq_mode);
void hal_epit_counter_disable(cyg_uint32 instance);
cyg_uint32 hal_epit_get_compare_event(cyg_uint32 instance);
void hal_epit_set_compare_event(cyg_uint32 instance, cyg_uint32 compare_val);
cyg_uint32 hal_epit_get_counter_value(cyg_uint32 instance);
void hal_epit_reload_counter(cyg_uint32 instance, cyg_uint32 load_val);

#if defined(__cplusplus)
}
#endif

//! @}

#endif //__EPIT_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
