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

#ifndef _CCM_PLL_H_
#define _CCM_PLL_H_

#include <cyg/infra/cyg_type.h>         // base types

#define CLK_SRC_32K         32768

//! @brief Create a clock gate bit mask value.
//! @param x 0..15, for CG0 to CG15
#define CG(x) (3 << (x*2))

//! @brief Constants for CCM CCGR register fields.
enum _clock_gate_constants
{
    CLOCK_ON = 0x3, //!< Clock always on in both run and stop modes.
    CLOCK_ON_RUN = 0x1, //!< Clock on only in run mode.
    CLOCK_OFF = 0x0 //!< Clocked gated off.
};

//! @brief Low power mdoes.
typedef enum _lp_modes {
    RUN_MODE,
    WAIT_MODE,
    STOP_MODE,
} lp_modes_t;

//! @brief Main clock sources.
typedef enum _main_clocks {
    CPU_CLK,
    AXI_CLK,
    MMDC_CH0_AXI_CLK,
    AHB_CLK,
    IPG_CLK,
    IPG_PER_CLK,
    MMDC_CH1_AXI_CLK,
} main_clocks_t;

//! @brief Peripheral clocks.
typedef enum _peri_clocks {
    UART1_MODULE_CLK,
    UART2_MODULE_CLK,
    UART3_MODULE_CLK,
    UART4_MODULE_CLK,
    SSI1_BAUD,
    SSI2_BAUD,
    CSI_BAUD,
    MSTICK1_CLK,
    MSTICK2_CLK,
    RAWNAND_CLK,
    USB_CLK,
    VPU_CLK,
    SPI_CLK,
    CAN_CLK
} peri_clocks_t;

//! @brief Available PLLs.
typedef enum plls {
    PLL1,
    PLL2,
    PLL3,
    PLL4,
    PLL5,
} plls_t;

extern const cyg_uint32 PLL1_OUTPUT;
extern const cyg_uint32 PLL2_OUTPUT[];
extern const cyg_uint32 PLL3_OUTPUT[];
extern const cyg_uint32 PLL4_OUTPUT;
extern const cyg_uint32 PLL5_OUTPUT;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

void hal_clock_gating_config(cyg_uint32 base_address, cyg_uint32 gating_mode);
cyg_uint32 hal_get_main_clock(main_clocks_t clk);
cyg_uint32 hal_get_peri_clock(peri_clocks_t clk);
void hal_ccm_init(void);

#if defined(__cplusplus)
}
#endif

#endif
