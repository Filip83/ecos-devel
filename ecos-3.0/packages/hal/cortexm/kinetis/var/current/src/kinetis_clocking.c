//==========================================================================
//
//      kinetis_clocking.c
//
//      Cortex-M Kinetis HAL functions
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2010 Free Software Foundation, Inc.                        
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
// Author(s):    Ilija kocho <ilijak@siva.com.mk>
// Date:         2011-10-19
// Description:
//
//####DESCRIPTIONEND####
//
//========================================================================

#include <pkgconf/hal.h>
#include <pkgconf/hal_cortexm.h>
#include <pkgconf/hal_cortexm_kinetis.h>
#ifdef CYGPKG_KERNEL
#include <pkgconf/kernel.h>
#endif

#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/cortexm_endian.h>
#include <cyg/hal/hal_arch.h>           // HAL header
#include <cyg/hal/hal_intr.h>           // HAL header
#include <cyg/hal/hal_if.h>             // HAL header

#include <cyg/io/ser_freescale_uart.h>

/*
#ifndef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC1
#define CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC1 0
#endif*/
//===========================================================================
// Forward declarations
//===========================================================================

cyg_uint32 hal_cortexm_systick_clock;
cyg_uint32 hal_kinetis_sysclk;
cyg_uint32 hal_kinetis_busclk;

typedef enum 
{
    mcg_fei, mcg_fee, mcg_fbe, mcg_fbi, 
    mcg_blpi, mcg_blpe, mcg_pbe, mcg_pee
} t_hall_kinetis_mcg_mode;

t_hall_kinetis_mcg_mode hal_kinetis_mcg_current_mode;
t_hall_kinetis_mcg_mode hal_kinetis_mcg_new_mode;

cyg_uint32 hal_get_cpu_clock(void);

static void hal_start_main_clock(void);
void hal_start_main_clock2(void);
void hal_set_clock_dividers(void);
#ifdef CYGHWR_HAL_CORTEXM_KINETIS_RTC
void hal_start_rtc_clock(void);
#endif

//#define CYGOPT_HAL_CORTEXM_KINETIS_MCG_REF_EXT
//==========================================================================
// Setup up system clocks
//
// Set up clocks from configuration. In the future this should be extended so
// that clock rates can be changed at runtime.

void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_start_clocks( void )
{
    cyghwr_hal_kinetis_sim_t *sim_p = CYGHWR_HAL_KINETIS_SIM_P;
#ifdef CYGHWR_HAL_CORTEXM_KINETIS_TRACE_CLKOUT
    cyghwr_hal_kinetis_port_t *port_p = CYGHWR_HAL_KINETIS_PORTA_P;
#endif
#if !defined(CYG_HAL_STARTUP_RAM)
# ifdef CYGHWR_HAL_CORTEXM_KINETIS_RTC
    // Real Time Clock
    hal_start_rtc_clock();
# endif
    hal_set_clock_dividers();
    // Main clock - MCG
    hal_start_main_clock();
	hal_set_clock_dividers();
#endif
    // Trace clock
#ifdef CYGHWR_HAL_CORTEXM_KINETIS_TRACECLK_CORE
    sim_p->sopt2 |= CYGHWR_HAL_KINETIS_SIM_SOPT2_TRACECLKSEL_M;
#else
    sim_p->sopt2 &= ~CYGHWR_HAL_KINETIS_SIM_SOPT2_TRACECLKSEL_M;
#endif
#ifdef CYGHWR_HAL_CORTEXM_KINETIS_TRACE_CLKOUT
    port_p->pcr[6] = CYGHWR_HAL_KINETIS_PORT_PCR_MUX(0x7);
#endif
}

#define MCG_WAIT_WHILE(_condition_) do{}while(_condition_)

// Setup MCG
// Note: Currently only PBE mode is supported and tested.

#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL

// MCG can have 1 or 2 PLL oscillators.
// PLL0 aka PLL is always present.
static inline void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_start_pll0(cyghwr_hal_kinetis_mcg_t *mcg_p)
{
    mcg_p->c5 = CYGHWR_HAL_KINETIS_MCG_C5_PRDIV(
          CYGOPT_HAL_CORTEXM_KINETIS_100_MCGOUT_PLL_PRDIV-1) |
          CYGHWR_HAL_KINETIS_MCG_C5_PLLSTEN_M
# ifdef CYGHWR_HAL_CORTEXM_KINETIS_PLLREFSEL_1
          | CYGHWR_HAL_KINETIS_MCG_C5_PLLREFSEL0_M
# endif //CYGHWR_HAL_CORTEXM_KINETIS_PLLREFSEL_1
          ;
# if CYGINT_HAL_CORTEXM_KINETIS_150
    mcg_p->c6 = CYGHWR_HAL_KINETIS_MCG_C6_VDIV(
          CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL_VDIV-16);
# else
    mcg_p->c6 = CYGHWR_HAL_KINETIS_MCG_C6_VDIV(
          CYGOPT_HAL_CORTEXM_KINETIS_100_MCGOUT_PLL_VDIV-24);
# endif
    mcg_p->c5 |= CYGHWR_HAL_KINETIS_MCG_C5_PLLCLKEN_M;
    MCG_WAIT_WHILE(!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_LOCK_M));
}
#endif //CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL

#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL1

// PLL1 Oscillator is present on some devices.
static inline void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_start_pll1(cyghwr_hal_kinetis_mcg_t *mcg_p)
{
    mcg_p->c11 = CYGHWR_HAL_KINETIS_MCG_C11_PLLCS_M;
    mcg_p->c11 = CYGHWR_HAL_KINETIS_MCG_C11_PRDIV1(
          CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL1_PRDIV-1) |
          CYGHWR_HAL_KINETIS_MCG_C11_PLLSTEN1_M
# ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL1
          | CYGHWR_HAL_KINETIS_MCG_C11_PLLCS_M
# endif
# ifdef CYGHWR_HAL_CORTEXM_KINETIS_PLL1REFSEL_1
          | CYGHWR_HAL_KINETIS_MCG_C11_PLLREFSEL1_M
# endif
          ;
    mcg_p->c12 = CYGHWR_HAL_KINETIS_MCG_C12_VDIV1(
          CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL1_VDIV-16);
    mcg_p->c11 |= CYGHWR_HAL_KINETIS_MCG_C11_PLLCLKEN1_M;
    MCG_WAIT_WHILE(!(mcg_p->s2 & CYGHWR_HAL_KINETIS_MCG_S2_LOCK1_M));
}
#endif // CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_PLL1


// There are 1 or 2 external oscillators
void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_start_ext_ref(void)
{
    volatile cyg_uint8 *osc_cr_p = CYGHWR_HAL_KINETIS_OSC_CR_P;
	hal_set_pin_function(CYGHWR_HAL_KINETIS_PIN(A, 18, 0 , 0));
	hal_set_pin_function(CYGHWR_HAL_KINETIS_PIN(A, 19, 1, 0));
# if CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0 == 1
#  ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_XTAL
    // Set the oscillator 0
    *osc_cr_p = CYGHWR_HAL_CORTEXM_KINETIS_OSC_CAP / 2;
#  elif defined CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_OSC
    // Select external oscillator
    // In this mode OSCERCLK is enabled and is astive in 
    // Stop mode. Can be used as error clock source
    *osc_cr_p = CYGHWR_HAL_KINETIS_OSC_CR_ERCLKEN_M |
          CYGHWR_HAL_KINETIS_OSC_CR_EREFSTEN_M;
#  endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_OSC
# endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0 

# if CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC1 == 1
#  ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC1_IS_XTAL
    // Set the oscillator 1
    osc_cr_p = CYGHWR_HAL_KINETIS_OSC1_CR_P;
    *osc_cr_p = CYGHWR_HAL_CORTEXM_KINETIS_OSC1_CAP / 2;
#  elif defined CYGOPT_HAL_CORTEXM_KINETIS_MCG_REF_EXT1_IS_OSC
    // Select external oscillator
    *osc_cr_p = CYGHWR_HAL_KINETIS_OSC1_CR_ERCLKEN_M |
          CYGHWR_HAL_KINETIS_OSC1_CR_EREFSTEN_M;
#  endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC1_IS_XTAL
# endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC1 
}


void/* CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR*/
hal_start_main_clock(void)
{
	cyg_uint32 i;
	cyghwr_hal_kinetis_mcg_t* mcg_p = CYGHWR_HAL_KINETIS_MCG_P;
	cyghwr_hal_kinetis_sim_t* sim_p = CYGHWR_HAL_KINETIS_SIM_P;

#if CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0 == 1
	hal_start_ext_ref();
# endif

#if defined(CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_FLL) && \
    defined(CYGNUM_HAL_CORTEXM_KINETIS_MCG_REF_FREQ_SOURCE_EXT_32K_RTC)
	// Select RTC clock source for MCG reference
# if CYGHWR_HAL_CORTEXM_KINETIS_REV == 1
	sim_p->sopt2 |= CYGHWR_HAL_KINETIS_SIM_SOPT2_MCGCLKSEL_M;
# elif CYGHWR_HAL_CORTEXM_KINETIS_REV == 2
	mcg_p->c7 |= CYGHWR_HAL_KINETIS_MCG_C7_OSCSEL_M;
# endif // CYGHWR_HAL_CORTEXM_KINETIS_REV == 2
#endif // CYGNUM_HAL_CORTEXM_KINETIS_MCG_REF_FREQ_SOURCE_EXT_32_RTC

#  ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_EXTAL
	mcg_p->c2 |= CYGHWR_HAL_KINETIS_MCG_C2_EREFS_M | CYGHWR_HAL_KINETIS_MCG_C2_HGO_M
#  endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_REF_EXT_IS_XTAL
		;

#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL   
	// Wait for oscillator start up
	MCG_WAIT_WHILE(!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_OSCINIT_M));
	// Wait for reference clock to switch to external reference
	MCG_WAIT_WHILE(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_IREFST_M);
	// Wait for status flags update
	MCG_WAIT_WHILE((mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_CLKST_M) !=
		CYGHWR_HAL_KINETIS_MCG_S_CLKST_EXT);

	hal_start_pll0(mcg_p);

	// Switch to PBE mode
	mcg_p->c6 |= CYGHWR_HAL_KINETIS_MCG_C6_PLLS_M;

	MCG_WAIT_WHILE((mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_CLKST_M) !=
		CYGHWR_HAL_KINETIS_MCG_S_CLKST_EXT);
	MCG_WAIT_WHILE(!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_PLLST_M));
	MCG_WAIT_WHILE(!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_LOCK_M));

	// Enter PEE mode
	mcg_p->c1 &= ~CYGHWR_HAL_KINETIS_MCG_C1_CLKS_M;
	MCG_WAIT_WHILE((mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_CLKST_M) !=
		CYGHWR_HAL_KINETIS_MCG_S_CLKST_PLL);

	sim_p->sopt2 |= CYGHWR_HAL_KINETIS_SIM_SOPT2_PLLFLLSEL_M;
# endif

#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_FLL 
	mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_RANGE(
		CYGNUM_HAL_CORTEXM_KINETIS_MCG_REF_FREQ_RANGE)
#  ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_EXTAL
		| CYGHWR_HAL_KINETIS_MCG_C2_EREFS_M | CYGHWR_HAL_KINETIS_MCG_C2_HGO_M
#  endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_REF_EXT_IS_XTAL
		;
	mcg_p->c1 = CYGHWR_HAL_KINETIS_MCG_C1_FRDIV(
		CYGNUM_HAL_CORTEXM_KINETIS_MCG_REF_FRDIV_REG)
# if defined(CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL) || \
     defined(CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL1) || \
     defined(CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_EXT_REFCLK)
		| CYGHWR_HAL_KINETIS_MCG_C1_CLKS(CYGHWR_HAL_KINETIS_MCG_C1_CLKS_EXT_REF)
# endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL*
		;

	// Configure FLL
	mcg_p->c4 = (mcg_p->c4 & 0x1f) |
		(CYGNUM_HAL_CORTEXM_MCG_DCO_DMX32 |
			CYGHWR_HAL_KINETIS_MCG_C4_DRST_DRS(
				CYGNUM_HAL_CORTEXM_MCG_DCO_DRST_DRS));
#endif

#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_EXT_REFCLK
	mcg_p->c5 = 12;

#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_OSC
	mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_RANGE(2)/* | CYGHWR_HAL_KINETIS_MCG_C2_HGO_M*/;
#endif

	// Wait for oscillator start up
#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_OSC0_IS_XTAL
#if CYGHWR_HAL_CORTEXM_KINETIS_MCG_OSC0_FREQ < 40000
	mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_RANGE(0) | CYGHWR_HAL_KINETIS_MCG_C2_HGO_M | CYGHWR_HAL_KINETIS_MCG_C2_EREFS_M;
#elif CYGHWR_HAL_CORTEXM_KINETIS_MCG_OSC0_FREQ < 8000000
	mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_RANGE(1) | CYGHWR_HAL_KINETIS_MCG_C2_HGO_M | CYGHWR_HAL_KINETIS_MCG_C2_EREFS_M;
#elif CYGHWR_HAL_CORTEXM_KINETIS_MCG_OSC0_FREQ < 32000000
	mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_RANGE(2) | CYGHWR_HAL_KINETIS_MCG_C2_HGO_M | CYGHWR_HAL_KINETIS_MCG_C2_EREFS_M;
#else
#error "Cystall OSC0 invalid freq"
#endif
	// TODO check if it is all right for crystal oscillaor
	MCG_WAIT_WHILE(!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_OSCINIT_M));
#endif

	mcg_p->c1 = 0;
	mcg_p->c7 = 0;

	i = 1500U;
	while (i--)
	{
		asm("nop");
	}

	mcg_p->c1 = CYGHWR_HAL_KINETIS_MCG_C1_CLKS(2);

	// Wait for reference clock to switch to external reference
	MCG_WAIT_WHILE(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_IREFST_M);
	// Wait for status flags update
	MCG_WAIT_WHILE((mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_CLKST_M) !=
		CYGHWR_HAL_KINETIS_MCG_S_CLKST_EXT);

	/* In FBE now, start to enter BLPE. */
	mcg_p->c2 |= CYGHWR_HAL_KINETIS_MCG_C2_LP_M;

#endif
    
#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_INT_RC
    // Select internal reference
    mcg_p->c1 = CYGHWR_HAL_KINETIS_MCG_C1_CLKS(1) |
                CYGHWR_HAL_KINETIS_MCG_C1_IREFS_M;
    
    mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_LP_M
#if CYGOPT_HAL_CORTEXM_KINETIS_MCGOUT_INT_RC_HI == 1
            | CYGHWR_HAL_KINETIS_MCG_C1_IREFSTEN_M
#endif
            ;

    // Wait for reference clock to switch to internal reference
    MCG_WAIT_WHILE(!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_IREFST_M));
    // Wait for status flags update
    MCG_WAIT_WHILE((mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_CLKST_M) !=
            CYGHWR_HAL_KINETIS_MCG_S_CLKST_INT);
#endif
#if 1
    
    sim_p->clk_div2 = 0;
    sim_p->sopt2 |=  
                    CYGHWR_HAL_KINETIS_SIM_SOPT2_USBSRC_M  | (3l << 16);         
#endif   
}

void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_start_main_clock2(void)
{
    #if 1
    cyg_uint32 i;

    cyghwr_hal_kinetis_mcg_t *mcg_p = CYGHWR_HAL_KINETIS_MCG_P;
    cyghwr_hal_kinetis_sim_t *sim_p = CYGHWR_HAL_KINETIS_SIM_P;
    volatile cyg_uint8 *osc_cr_p = CYGHWR_HAL_KINETIS_OSC_CR_P;

    sim_p->clk_div1 = 0x55550000U;
    

    //CLOCK_InitOsc0
    *osc_cr_p = 0x80 | 0 / 2;
   
    mcg_p->c2 = CYGHWR_HAL_KINETIS_MCG_C2_RANGE(2) | 4;
    
    //CLOCK_CONFIG_SetFllExtRefDiv
    mcg_p->c1 = 0;
                
    //CLOCK_BootToBlpeMode
    //CLOCK_SetExternalRefClkConfig(oscsel);
    

    mcg_p->c7 =  0;
    /* ERR009878 Delay at least 50 micro-seconds for external clock change valid. */
    i = 1500U;
    while (i--)
    {
        asm ("nop");
    }

    mcg_p->c1 = CYGHWR_HAL_KINETIS_MCG_C1_CLKS(2);


    while (!(mcg_p->status & CYGHWR_HAL_KINETIS_MCG_S_OSCINIT_M))
    {
    }

    
    /* Wait for MCG_S[CLKST] and MCG_S[IREFST]. */
    while ((mcg_p->status & (CYGHWR_HAL_KINETIS_MCG_S_IREFST_M | CYGHWR_HAL_KINETIS_MCG_S_CLKST_M)) !=
           (CYGHWR_HAL_KINETIS_MCG_S_CLKST(2)))
    {
    }


    /* In FBE now, start to enter BLPE. */
    mcg_p->c2 |= CYGHWR_HAL_KINETIS_MCG_C2_LP_M;
    
    

    //CLOCK_SetSimConfig
   /* sim_p->clk_div1 = 0x200000U;
    sim_p->clk_div2 = 0;*/
    sim_p->sopt2 =  (2 << CYGHWR_HAL_KINETIS_SIM_SOPT2_PLLFLLSEL_S) |
                    CYGHWR_HAL_KINETIS_SIM_SOPT2_USBSRC_M           |
                    (3 << 16) | (7 << 5);
    //sim_p->sopt1 =  (2 << CYGHWR_HAL_KINETIS_SIM_SOPT2_PLLFLLSEL_S);
    hal_set_clock_dividers();
#endif

}

cyg_uint32 CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_get_cpu_clock(void)
{

    cyg_uint32 freq;
#if 0
#ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL
    cyghwr_hal_kinetis_mcg_t *mcg_p = CYGHWR_HAL_KINETIS_MCG_P;

# if CYGINT_HAL_CORTEXM_KINETIS_150
    freq = CYGNUM_HAL_CORTEXM_KINETIS_MCG_FLL_PLL_REF_FREQ /
          ((mcg_p->c5 & CYGHWR_HAL_KINETIS_MCG_C5_PRDIV_M)+1) *
          ((mcg_p->c6 & CYGHWR_HAL_KINETIS_MCG_C6_VDIV_M)+16) / 2;
# else
    freq = CYGNUM_HAL_CORTEXM_KINETIS_MCG_FLL_PLL_REF_FREQ /
          ((mcg_p->c5 & CYGHWR_HAL_KINETIS_MCG_C5_PRDIV_M)+1) *
          ((mcg_p->c6 & CYGHWR_HAL_KINETIS_MCG_C6_VDIV_M)+24);
# endif
#elif defined CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_PLL1
    cyghwr_hal_kinetis_mcg_t *mcg_p = CYGHWR_HAL_KINETIS_MCG_P;

    freq = CYGNUM_HAL_CORTEXM_KINETIS_MCG_FLL_PLL_REF_FREQ /
          ((mcg_p->c11 & CYGHWR_HAL_KINETIS_MCG_C11_PRDIV1_M)+1) *
          ((mcg_p->c12 & CYGHWR_HAL_KINETIS_MCG_C12_VDIV1_M)+16) / 2;
#else 
    freq = CYGNUM_HAL_CORTEXM_KINETIS_MCG_MCGOUT_FREQ_SP;
#endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_MCGOUTCLK_end
#endif

	freq = CYGNUM_HAL_CORTEXM_KINETIS_MCG_MCGOUT_FREQ_SP;

    return freq;
}


// Clock dividers provide clock sources for various peripherals.

void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_set_clock_dividers(void)
{
    cyghwr_hal_kinetis_sim_t *sim_p = CYGHWR_HAL_KINETIS_SIM_P;

    sim_p->clk_div1 = CYGHWR_HAL_KINETIS_SIM_CLKDIV1_OUTDIV1(0) |
          CYGHWR_HAL_KINETIS_SIM_CLKDIV1_OUTDIV2(
                CYGHWR_HAL_CORTEXM_KINETIS_CLKDIV_PER_BUS-1)  |
          CYGHWR_HAL_KINETIS_SIM_CLKDIV1_OUTDIV3(
                CYGHWR_HAL_CORTEXM_KINETIS_CLKDIV_FLEX_BUS-1) |
          CYGHWR_HAL_KINETIS_SIM_CLKDIV1_OUTDIV4(
                CYGHWR_HAL_CORTEXM_KINETIS_CLKDIV_FLASH-1);

    /*sim_p->clk_div2 = CYGHWR_HAL_KINETIS_SIM_CLKDIV2_USBDIV(
                          CYGHWR_HAL_CORTEXM_KINETIS_USBCLK_DIV-1) |
                          (CYGHWR_HAL_CORTEXM_KINETIS_USBCLK_FRAC==2 ?
                               CYGHWR_HAL_KINETIS_SIM_CLKDIV2_USBFRAC_M : 0);*/
}

#ifdef CYGHWR_HAL_CORTEXM_KINETIS_RTC
void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_start_rtc_clock(void)
{
    cyghwr_hal_kinetis_rtc_t *rtc_p = CYGHWR_HAL_KINETIS_RTC_P;

    rtc_p->ier=0; // Disable RTC interrupts

    //Start RTC clock if not already started
    if(!(rtc_p->cr & CYGHWR_HAL_KINETIS_RTC_CR_OSCE)){
        rtc_p->cr = CYGHWR_HAL_KINETIS_RTC_CR_OSCE |
              CYGHWR_HAL_CORTEXM_KINETIS_RTC_OSC_CAP;
# ifdef CYGOPT_HAL_CORTEXM_KINETIS_MCG_REF_EXT_IS_RTC
        {
            volatile cyg_uint32 busycnt;
            for(busycnt=1000000; busycnt; busycnt--)
                __asm__ volatile ("nop\n");
        }
# endif // CYGOPT_HAL_CORTEXM_KINETIS_MCG_REF_EXT_IS_RTC
    }
}
#endif


//==========================================================================
// UART baud rate
//
// Set the baud rate divider of a UART based on the requested rate and
// the current clock settings.


void  CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_freescale_uart_setbaud(cyg_uint32 uart_p, cyg_uint32 baud)
{
    cyg_uint32 sbr, brfa;
    cyg_uint32 regval;

    switch(uart_p) {
    case CYGADDR_IO_SERIAL_FREESCALE_UART0_BASE:
    case CYGADDR_IO_SERIAL_FREESCALE_UART1_BASE:
        sbr = hal_kinetis_sysclk/(16*baud);
        break;
    case CYGADDR_IO_SERIAL_FREESCALE_UART2_BASE:
    case CYGADDR_IO_SERIAL_FREESCALE_UART3_BASE:
    case CYGADDR_IO_SERIAL_FREESCALE_UART4_BASE:
    case CYGADDR_IO_SERIAL_FREESCALE_UART5_BASE:
        sbr = hal_kinetis_busclk/(16*baud);
        break;
    default:
        sbr=0;
        break;
    }
    if(sbr) {
        HAL_READ_UINT8(uart_p + CYGHWR_DEV_FREESCALE_UART_BDH, regval);
        regval &= 0xE0;
        regval |= sbr >> 8;
        HAL_WRITE_UINT8(uart_p + CYGHWR_DEV_FREESCALE_UART_BDH, regval);
        HAL_WRITE_UINT8(uart_p + CYGHWR_DEV_FREESCALE_UART_BDL, (sbr & 0xFF));
        brfa = (((32*hal_kinetis_busclk)/(16*baud))-(32*sbr));
        HAL_READ_UINT8(uart_p + CYGHWR_DEV_FREESCALE_UART_C4, regval);
        regval &= 0xE0;
        regval |= brfa & 0x1f;
        HAL_WRITE_UINT8(uart_p + CYGHWR_DEV_FREESCALE_UART_C4, regval);
    }
}


void hal_update_clock_var(void)
{
#if 0
    hal_kinetis_sysclk=hal_get_cpu_clock();
    /*hal_kinetis_busclk=hal_kinetis_sysclk /
          CYGHWR_HAL_CORTEXM_KINETIS_CLKDIV_PER_BUS;*/
    hal_kinetis_busclk = hal_kinetis_sysclk;
    hal_cortexm_systick_clock=hal_kinetis_sysclk;
#else
    hal_cortexm_systick_clock=hal_get_cpu_clock();
    hal_kinetis_sysclk=hal_cortexm_systick_clock;
    hal_kinetis_busclk=hal_cortexm_systick_clock /
          CYGHWR_HAL_CORTEXM_KINETIS_CLKDIV_PER_BUS;
#endif
}


cyg_uint32 hal_get_peripheral_clock(void)
{
    return hal_kinetis_busclk;
}

//==========================================================================
// EOF kinetis_clocking.c
