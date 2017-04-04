/*==========================================================================
//
//      wandboard_misc.c
//
//      HAL misc board support for Freescale Wandboard
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 2006 Free Software Foundation, Inc.
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
// Author(s):    Mike Jones
// Date:         2013-08-08
// Description:  Platform Hal for Freescale Wandboard
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <string.h>

#include <sys/reent.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_diag.h>

#include <cyg/hal/cortex_a9.h> // From SDK
#include <cyg/hal/arm_cp_registers.h> // From SDK

#include <cyg/hal/hal_mmu.h>
#include <cyg/hal/var_mmu.h>
#include <cyg/hal/var_ccm.h>
#include <cyg/hal/var_timer.h>

#include <cyg/hal/var_enet.h>
#include <cyg/hal/registers/regsenet.h>

#include <cyg/io/gpio.h>

#define errno (*__errno())

//-------------------------------------------------------------------------
//reset platfrom
void hal_reset(void)
{
    /* Unlock SLCR regs */
//    HAL_WRITE_UINT32(IMX6_SYS_CTRL_BASEADDR + XSLCR_UNLOCK_OFFSET, XSLCR_UNLOCK_KEY);
    
    /* Tickle soft reset bit */
    HAL_WRITE_UINT32(0xF8000200, 1);

    while (1);
}

//==========================================================================
// EPNET RGMII Ethernet Controller
//

// For Saber SD board
//void hal_reset_phy(cyg_uint32 delay)
//{
//    /* Select ALT5 mode of ENET_CRS-DV for GPIO1_25 - PGMII_NRST */
//    /* active low output */
//    gpio_set_direction(GPIO_PORT1, 25, GPIO_GDIR_OUTPUT);
//    gpio_set_level(GPIO_PORT1, 25, GPIO_LOW_LEVEL);
//    hal_delay_us(500);
//    gpio_set_level(GPIO_PORT1, 25, GPIO_HIGH_LEVEL);
//
//}

// Remains here.
void hal_reset_phy()
{
    // This is a hard reset.
    gpio_set_direction(GPIO_PORT3, 29, GPIO_GDIR_OUTPUT);
    gpio_set_level(GPIO_PORT3, 29, GPIO_LOW_LEVEL);
    hal_delay_us(500);
    gpio_set_level(GPIO_PORT3, 29, GPIO_HIGH_LEVEL);
}

// -------------------------------------------------------------------------
// Hardware init

void hal_plf_hardware_init(void) {
#ifndef CYGPKG_IO_WATCHDOG
  /* Disable the watchdog. The eCos philosophy is that the watchdog is
     disabled unless the watchdog driver is used to enable it.
     Whoever if we disable it here we cannot re-enable it in the
     watchdog driver, hence the conditional compilation. */
//  HAL_WRITE_UINT32(IMX6_SCU_WDT_BASEADDR + XSCUWDTIMER_DISABLE_OFFSET, XSCUWDTIMER_WD_DISABLE_SEQ1); 
//  HAL_WRITE_UINT32(IMX6_SCU_WDT_BASEADDR + XSCUWDTIMER_DISABLE_OFFSET, XSCUWDTIMER_WD_DISABLE_SEQ2); 
#endif
}

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

//static cyg_uint8 enet_read_buffer[2048] __attribute__((section (".noncache"), aligned(16)));
//static cyg_uint8 enet_write_buffer[2048] __attribute__((section (".noncache"), aligned(16)));

void hal_enet_init(void)
{
	// This is 4 byte less than the data sheet because there is a
	// reserved 32 bit integer at the beginning of the struct.
    volatile hw_enet_t *enet_reg = (hw_enet_t *) 0x02188000;
	unsigned int ipg_clk;

//    enet_reg->ECR.U = ENET_RESET;

//    while (enet_reg->ECR.U & ENET_RESET) {
//        hal_delay_us(ENET_COMMON_TICK);
//    }

//    enet_reg->EIMR.U = 0x00000000;
//    enet_reg->EIR.U = 0xFFFFFFFF;

    /*
     * setup the MII gasket for RMII mode
     */
//    enet_reg->RCR.U = (enet_reg->RCR.U & ~(0x0000003F)) | ENET_RCR_RGMII_EN | ENET_RCR_FCE | ENET_RCR_PROM;
//    enet_reg->TCR.U |= ENET_TCR_FDEN;
//    enet_reg->MIBC.U |= ENET_MIB_DISABLE;

//    enet_reg->IAUR.U = 0;
//    enet_reg->IALR.U = 0;
//    enet_reg->GAUR.U = 0;
//    enet_reg->GALR.U = 0;

    /*TODO:: Use MII_SPEED(IPG_CLK) to get the value */
//	ipg_clk = hal_get_main_clock(IPG_CLK);
//    enet_reg->MSCR.U = (enet_reg->MSCR.U & (~0x7e)) | (((ipg_clk + 499999) / 5000000) << 1); 

    /*Enable ETHER_EN */
//    enet_reg->MRBR.U = 2048 - 16;
//    enet_reg->RDSR.U = (unsigned long)enet_read_buffer;
//    enet_reg->TDSR.U = (unsigned long)enet_write_buffer;

#if 0//defined(CHIP_MX6DQ) || defined(CHIP_MX6SDL)
	/* Enable Swap to support little-endian device */
	enet_reg->ECR.U |= (0x1 << 8); //BM_ENET_ECR_DBSWP;
	/* set ENET tx at store and forward mode */
	enet_reg->TFWR.U |= BM_ENET_TFWR_STRFWD; //(0x1 << 8);
#endif	
}

//--------------------------------------------------------------------------
// EOF wandboard_misc.c
