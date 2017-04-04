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

#ifndef __TIMER_H__
#define __TIMER_H__

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/hal/var_epit.h>

#define HAL_TIMER_ENABLE(__period) hal_epit_counter_enable(2, __period, IRQ_MODE)
#define HAL_TIMER_DISABLE() hal_epit_counter_disable(2)
#define HAL_TIMER_WAIT_COMPARE() while (!hal_epit_get_compare_event(2))
#define HAL_TIMER_VALUE() hal_epit_get_counter_value(2)

//! @brief Options for low power mode support for the timers.
//!
//! These constants are bit masks that are or'd together to select in which low
//! power modes the timer will continue counting.
enum _timer_low_power_modes
{
    WAIT_MODE_EN = 1,   //!< Timer is enabled in wait mode.
    STOP_MODE_EN = 2    //!< Timer is enabled in stop mode.
};

//! @brief Available clock sources for the timers.
enum _timer_clock_sources
{
    CLKSRC_OFF = 0,     //!< clock source is OFF
    CLKSRC_IPG_CLK = 1, //!< clock source is peripheral clock
    CLKSRC_PER_CLK = 2, //!< clock source is high-freq reference clock
    CLKSRC_CLKIN = 3,   //!< clock source is external from a CLKIN input
    CLKSRC_CKIL = 3     //!< clock source is low-freq reference clock
};

//! @brief Do not enable interrupts.
#define POLLING_MODE 0

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////
extern cyg_uint32 g_system_timer_port;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @brief Delay for a given number of microseconds.
//!
//! system_time_init() must have been called before using this function.
//!
//! @param usecs Delay in microseconds.
void hal_delay_us(cyg_int32 usecs);

//! @brief Init system timer facilities.
//!
//! Inits the EPIT timer used for delay, and inits the microsecond counter.
void hal_system_time_init(void);

#if defined(__cplusplus)
}
#endif

#endif // __TIMER_H__
