//=============================================================================
//
//      hal_aux.c
//
//      HAL auxiliary objects and code; per platform
//
//=============================================================================
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   Filip 
// Contributors:
// Original data: hmt
// Date:        2014-04-28
// Purpose:     HAL aux objects: startup tables.
// Description: Tables for per-platform initialization
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <cyg/hal/hal_if.h>
#include <cyg/hal/avr32/io.h>

#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_plf.h>
#include <pkgconf/system.h>
#include <cyg/io/flashcdw.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
#include <pkgconf/hal.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>


#include <cyg/hal/board_config.h>

// Nutno opravit
#define CYGNUM_HAL_OSCILATORS_OSC0_EXTERNAL_STARTUP_TIME 10

//! Oscilator startup error flags
#define HW_ERROR_OSC0_NOT_RUNNING       0x0000000000000001
#define HW_ERROR_OSC1_NOT_RUNNING       0x0000000000000002
#define HW_ERROR_OSC32_NOT_RUNNING      0x0000000000000004
#define HW_ERROR_RC8M_NOT_RUNNING       0x0000000000000008
#define HW_ERROR_MAIN_CLOCK_NOT_RUNNIGN 0x0000000000000010
#define HW_ERROR_DFLL_NO_LOCK           0x0000000000000400

//! The timeguard used for polling in ticks.
#define SCIF_POLL_TIMEOUT             100000

#include CYGBLD_HAL_PLATFORM_H

//! Unlock SCIF register macro
#define SCIF_UNLOCK(reg)  (AVR32_SCIF.unlock =  \
        (AVR32_SCIF_UNLOCK_KEY_VALUE << AVR32_SCIF_UNLOCK_KEY_OFFSET) | (reg))

//! Unlock PM register macro
#define PM_UNLOCK(reg)  (AVR32_PM.unlock =      \
        (AVR32_PM_UNLOCK_KEY_VALUE << AVR32_PM_UNLOCK_KEY_OFFSET) | (reg))


#define PLL_SOURCE(_source_)  (_source_ == OSC0) ? 0 : 1  


//! The min DFLL output frequency
#if (UC3L0128 || UC3L0256)
#define SCIF_DFLL_MINFREQ_KHZ         20000UL
#define SCIF_DFLL_MINFREQ_HZ          20000000UL
#else
#define SCIF_DFLL_MINFREQ_KHZ         40000UL
#define SCIF_DFLL_MINFREQ_HZ          40000000UL
#endif

//! The max DFLL output frequency
#define SCIF_DFLL_MAXFREQ_KHZ         150000UL
#define SCIF_DFLL_MAXFREQ_HZ          150000000UL

#define SCIF_DFLL_COARSE_MAX  (AVR32_SCIF_COARSE_MASK >> AVR32_SCIF_COARSE_OFFSET)
#define SCIF_DFLL_FINE_MAX    (AVR32_SCIF_FINE_MASK >> AVR32_SCIF_FINE_OFFSET)
#define SCIF_DFLL_FINE_HALF   (1 << (AVR32_SCIF_DFLL0CONF_FINE_SIZE-1))

#define SCIF_DFLL0_MODE_OPENLOOP     0
#define SCIF_DFLL0_MODE_CLOSEDLOOP   2


#define RCSYS       0
#define OSC0        1
#define OSC32K      2
#define GCLK9       3
#define DFLL        4
#define RC120M      5
#define CLKCPU      6
#define RC32K       7
#define CLK1K       8
#define PLL0        9
#define GCLKIN0     10
#define GCLKIN1     11
#define GCLKIN2     12



void hal_clocks_init(void);
void hal_oscilator_init(void);
void hal_bpd_init(void);
void hal_init_error_interrupts(void);
void hal_init_dfll_openloop(void);
void hal_init_dfll_closedloop(void);
int scif_pclksr_statushigh_wait(unsigned long statusMask);

extern void hal_board_init(void );

externC cyg_uint64	_hw_error;

//--------------------------------------------------------------------------
// Platform init code.
void
hal_platform_init(void)
{
    // Basic hardware initialization
    hal_bpd_init();
    hal_board_init();
    hal_oscilator_init();
    hal_clocks_init();
    
    HAL_init_interrupts();
    hal_if_init();   // Initialize logical I/O layer (virtual vector support)
    
    hal_init_error_interrupts();
}

//-----------------------------------------------------------------------------
// Initialize CPU DFLL in openloop modde
void hal_init_dfll_openloop(void)
{
    unsigned int  configValue = 0;
    unsigned int  timeout = SCIF_POLL_TIMEOUT;
    unsigned long             Coarse;
    unsigned long             Fine;
    unsigned long             CoarseFreq;
    unsigned long             DeltaFreq;

    //**
    //** Dynamically compute the COARSE and FINE values.
    //**
    // Fdfll = (Fmin+(Fmax-Fmin)*(COARSE/0xFF))*(1+X*(FINE-0x100)/0x1FF)=CoarseFreq*(1+X*(FINE-0x100)/0x1FF)

    // Compute the COARSE value.
    Coarse = ((CYGNUM_HAL_OSCILATORS_DFLL0_FREQ - SCIF_DFLL_MINFREQ_KHZ)*
          SCIF_DFLL_COARSE_MAX)/(SCIF_DFLL_MAXFREQ_KHZ - SCIF_DFLL_MINFREQ_KHZ);
    // Compute the COARSE DFLL frequency.
    CoarseFreq = SCIF_DFLL_MINFREQ_KHZ + 
            (((SCIF_DFLL_MAXFREQ_KHZ - SCIF_DFLL_MINFREQ_KHZ)/
            SCIF_DFLL_COARSE_MAX)*Coarse);
    // Compute the coarse error.
    DeltaFreq = CYGNUM_HAL_OSCILATORS_DFLL0_FREQ - CoarseFreq;
    // Compute the FINE value.
    // Fine = ((DeltaFreq*SCIF_DFLL_FINE_MAX)*10/CoarseFreq) + SCIF_DFLL_FINE_HALF;
    // Theoretical equation don't work on silicon: the best was to use X=5/2 to
    // find FINE, then do FINE/4.
    Fine = ((DeltaFreq*SCIF_DFLL_FINE_MAX)*2/CoarseFreq*5) + SCIF_DFLL_FINE_HALF;
    Fine >>=2;
   
#ifdef CYGNUM_HAL_OSCILATORS_DFLL0_CCEN
    configValue = AVR32_SCIF_CCEN_MASK | AVR32_SCIF_DFLL0CONF_EN_MASK;
#else
    configValue = AVR32_SCIF_DFLL0CONF_EN_MASK;
#endif
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = configValue;
   
    timeout = SCIF_POLL_TIMEOUT;

    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
    
    configValue |= SCIF_DFLL0_MODE_OPENLOOP;
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = configValue;
    
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }

    // Write DFLL0CONF.COARSE & DFLL0CONF.FINE
    configValue |= (Coarse << AVR32_SCIF_COARSE_OFFSET)&AVR32_SCIF_COARSE_MASK;
    configValue |= (Fine << AVR32_SCIF_FINE_OFFSET)&AVR32_SCIF_FINE_MASK;;
    
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = configValue;
    
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
}

void hal_init_dfll_closedloop(void)
{
    unsigned int  configValue = 0;
    unsigned long             Coarse;
    unsigned long             Imul;
    unsigned long             Fmul;
    unsigned int gc_source_clock_freq_hz;
    unsigned long long target_freq_hz = 1e3*CYGNUM_HAL_OSCILATORS_DFLL0_FREQ;

    // This function only supports the following source clocks for the CLK_DFLLIF_REF generic clock:
    // SCIF_GCCTRL_SLOWCLOCK (aka RCSYS), SCIF_GCCTRL_OSC32K, SCIF_GCCTRL_RC32K,
    // SCIF_GCCTRL_OSC0, SCIF_GCCTRL_RC120M, SCIF_GCCTRL_CLK1K.
#if CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == RCSYS
    AVR32_SCIF.gcctrl[0] = (0 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = 115000;
#elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == OSC32K
    AVR32_SCIF.gcctrl[0] = (1 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = 32768;
#elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == OSC0
    AVR32_SCIF.gcctrl[0] = (3 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = CYGNUM_HAL_OSCILATORS_OSC0_FREQV*1e6;
#elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == CLKCPU
    AVR32_SCIF.gcctrl[0] = (5 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = CYGHWR_HAL_AVR32_CPU_FREQ*1e6;
#elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == RC32K
    AVR32_SCIF.gcctrl[0] = (9 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = SCIF_RC32K_FREQ_HZ;
#elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == RC120M
    AVR32_SCIF.gcctrl[0] = (4 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = SCIF_RC32K_FREQ_HZ;
#elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == PLL0
    AVR32_SCIF.gcctrl[0] = (12 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = SCIF_RC32K_FREQ_HZ;
#else 
    AVR32_SCIF.gcctrl[0] = (11 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    gc_source_clock_freq_hz = 1000;
#endif
    
    // Configure the DFLL.
    // The coarse value (= (dfll_f - SCIF_DFLL_MINFREQ_KHZ)*255/(SCIF_DFLL_MAXFREQ_KHZ - SCIF_DFLL_MINFREQ_KHZ))
    Coarse = ((target_freq_hz - SCIF_DFLL_MINFREQ_HZ)*255)/
            (SCIF_DFLL_MAXFREQ_HZ - SCIF_DFLL_MINFREQ_HZ);

    // imul = (fDFLL)/fref,
    // fmul = (fDFLL*2^16)/fref - imul*2^16,
    // with fref being the frequency of the DFLL main reference generic clock
    // and fDFLL being the target frequency of the DFLL
    Imul = target_freq_hz/gc_source_clock_freq_hz;

    Fmul = (target_freq_hz<<16)/gc_source_clock_freq_hz 
            - ((unsigned long long)(Imul)<<16);

    
#ifdef CYGNUM_HAL_OSCILATORS_DFLL0_CCEN
    configValue = AVR32_SCIF_CCEN_MASK | AVR32_SCIF_DFLL0CONF_EN_MASK;
#else
    configValue = AVR32_SCIF_DFLL0CONF_EN_MASK;
#endif
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = configValue;
   
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
    
    SCIF_UNLOCK(AVR32_SCIF_DFLL0STEP);
    AVR32_SCIF.dfll0step = ((CYGNUM_HAL_OSCILATORS_DFLL0_COARSE_MAX_STEPP << 
                            AVR32_SCIF_DFLL0STEP_CSTEP_OFFSET)
                          &AVR32_SCIF_DFLL0STEP_CSTEP_MASK) |
                           ((CYGNUM_HAL_OSCILATORS_DFLL0_FINE_MAX_STEPP << 
                            AVR32_SCIF_DFLL0STEP_FSTEP_OFFSET)
                          &AVR32_SCIF_DFLL0STEP_FSTEP_MASK);
    
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
    

    SCIF_UNLOCK(AVR32_SCIF_DFLL0MUL);
    AVR32_SCIF.dfll0mul = ((Fmul << AVR32_SCIF_DFLL0MUL_FMUL_OFFSET)
                            &AVR32_SCIF_DFLL0MUL_FMUL_MASK)             |
                          ((Imul << AVR32_SCIF_DFLL0MUL_IMUL_OFFSET)
                            &AVR32_SCIF_DFLL0MUL_IMUL_MASK);
    
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
    
    configValue |= SCIF_DFLL0_MODE_CLOSEDLOOP;
    configValue |= (Coarse << AVR32_SCIF_COARSE_OFFSET)&AVR32_SCIF_COARSE_MASK;
#if CYGNUM_HAL_OSCILATORS_DFLL0_QLEN == 1
    configValue |= AVR32_SCIF_DFLL0CONF_QLEN_MASK;
#endif
            
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = configValue;
    
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
    
#if CYGNUM_HAL_OSCILATORS_DFLL0_DITHERING == 1
    #if CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == RCSYS
        AVR32_SCIF.gcctrl[1] = (0 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == OSC32K
        AVR32_SCIF.gcctrl[0] = (1 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == OSC0
        AVR32_SCIF.gcctrl[1] = (3 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == CLKCPU
        AVR32_SCIF.gcctrl[1] = (5 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == RC32K
        AVR32_SCIF.gcctrl[1] = (9 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == RC120M
        AVR32_SCIF.gcctrl[1] = (4 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #elif CYGNUM_HAL_OSCILATORS_DFLL0_SOURCE == PLL0
        AVR32_SCIF.gcctrl[1] = (12 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #else 
        AVR32_SCIF.gcctrl[1] = (11 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    #endif

    configValue |= AVR32_SCIF_DITHER_MASK;
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = configValue;
    
    // Wait for PCLKSR.DFLL0RDY is high
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
#endif

    if(scif_pclksr_statushigh_wait(AVR32_SCIF_PCLKSR_DFLL0LOCKF_MASK) != 1)
    {
        _hw_error |= HW_ERROR_DFLL_NO_LOCK;
        return;
    }
    
   /* AVR32_SCIF.gcctrl[5] = (2 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET) | 
                                       AVR32_SCIF_GCCTRL_CEN_MASK;
    
    gpio_enable_module_pin(AVR32_SCIF_GCLK_5_0_PIN , AVR32_SCIF_GCLK_5_0_FUNCTION);*/

}

int scif_pclksr_statushigh_wait(unsigned long statusMask)
{
  unsigned int  timeout = SCIF_POLL_TIMEOUT;

  while(!(AVR32_SCIF.pclksr & statusMask))
  {
    if(--timeout == 0)
      return 0;
  }
  return 1;
}

//-----------------------------------------------------------------------------
// Initialize CPU clock sources
void hal_oscilator_init(void)
{
    /*SCIF_UNLOCK(AVR32_SCIF_VREGCR);
    AVR32_SCIF.vregcr = (0x3 << 2);*/
    //oscilator 0 settings
#ifdef CYGNUM_HAL_OSCILATORS_OSC0_ENABLED
    cyg_uint32 gain = 
            (CYGNUM_HAL_OSCILATORS_OSC0_FREQV < 12) ? AVR32_SCIF_OSCCTRL0_GAIN_G0 :
            (CYGNUM_HAL_OSCILATORS_OSC0_FREQV < 16) ? AVR32_SCIF_OSCCTRL0_GAIN_G1 :
            AVR32_SCIF_OSCCTRL0_GAIN_G2;

    SCIF_UNLOCK(AVR32_SCIF_OSCCTRL0);				  
    #ifdef CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL	
        #ifdef CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE				  
    AVR32_SCIF.oscctrl0 = 
      (CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE << AVR32_SCIF_OSCCTRL0_AGC_OFFSET)
    | (gain << AVR32_SCIF_OSCCTRL0_GAIN_OFFSET )
    | (CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL << AVR32_SCIF_OSCCTRL0_MODE_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC0_ENABLED << AVR32_SCIF_OSCCTRL0_OSCEN_OFFSET);
        #else
    AVR32_SCIF.oscctrl0 = 
      (gain << AVR32_SCIF_OSCCTRL0_GAIN_OFFSET )
    | (CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL << AVR32_SCIF_OSCCTRL0_MODE_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC0_ENABLED << AVR32_SCIF_OSCCTRL0_OSCEN_OFFSET);
        #endif //CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE

    if(scif_pclksr_statushigh_wait(AVR32_SCIF_ISR_OSC0RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_OSC0_NOT_RUNNING;
        return;
    }
    #else
    AVR32_SCIF.oscctrl0 = 
     /* (CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE << AVR32_SCIF_OSCCTRL0_AGC_OFFSET)
    |*/ (gain << AVR32_SCIF_OSCCTRL0_GAIN_OFFSET )
    | (CYGNUM_HAL_OSCILATORS_OSC0_EXTERNAL_STARTUP_TIME 
            << AVR32_SCIF_OSCCTRL0_STARTUP_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC0_ENABLED << AVR32_SCIF_OSCCTRL0_OSCEN_OFFSET);
    #endif //CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL	
#endif //CYGNUM_HAL_OSCILATORS_OSC0_ENABLED

        // RC32K settings
#ifdef CYGNUM_HAL_OSCILATORS_RC32K_ENABLED
    SCIF_UNLOCK(AVR32_SCIF_RC32KCR);
    AVR32_SCIF.rc32kcr = AVR32_SCIF_RC32KCR_EN_MASK;
#endif //CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED
    
    //32k oscilator settings
#ifdef CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED
    SCIF_UNLOCK(AVR32_SCIF_RC32KCR);
    AVR32_SCIF.rc32kcr = 0;
    SCIF_UNLOCK(AVR32_SCIF_OSCCTRL32);
    #ifdef CYGNUM_HAL_OSCILATORS_OSC32K_EXTERNAL
    AVR32_SCIF.oscctrl32 = 
      (AVR32_SCIF_OSCCTRL32_MODE_EXT_CLOCK << AVR32_SCIF_OSCCTRL32_MODE_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC32K_EXTERNAL_STARTUP_TIME 
            << AVR32_SCIF_OSCCTRL32_STARTUP_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED 
            << AVR32_SCIF_OSCCTRL32_OSC32EN_OFFSET);
    #else
	#ifdef CYGNUM_HAL_OSCILATORS_OSC32K_MODE
    AVR32_SCIF.oscctrl32 = 
      (AVR32_SCIF_OSCCTRL32_MODE_CRYSTAL_HIC 
            << AVR32_SCIF_OSCCTRL32_MODE_OFFSET)
    | (2 << AVR32_SCIF_OSCCTRL32_STARTUP_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED 
            << AVR32_SCIF_OSCCTRL32_OSC32EN_OFFSET);
	#else
    AVR32_SCIF.oscctrl32 = 
      (AVR32_SCIF_OSCCTRL32_MODE_CRYSTAL << AVR32_SCIF_OSCCTRL32_MODE_OFFSET)
    | (2 << AVR32_SCIF_OSCCTRL32_STARTUP_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED << AVR32_SCIF_OSCCTRL32_OSC32EN_OFFSET);
	#endif //CYGNUM_HAL_OSCILATORS_OSC32K_MODE
    #endif //CYGNUM_HAL_OSCILATORS_OSC32K_EXTERNAL
    
    if(scif_pclksr_statushigh_wait(AVR32_SCIF_ISR_OSC32RDY_MASK) != 1)
    {
        _hw_error |= HW_ERROR_OSC32_NOT_RUNNING;
        return;
    }
#endif //CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED

    // RC120M settings
#ifdef CYGNUM_HAL_OSCILATORS_RC120M_ENABLED
    SCIF_UNLOCK(AVR32_SCIF_RC120MCR);
    AVR32_SCIF.rc120mcr = AVR32_SCIF_RC120MCR_EN_MASK;
#endif //CYGNUM_HAL_OSCILATORS_RC120M_ENABLED

    // PLL0 configuration
#ifdef CYGNUM_HAL_OSCILATORS_PLL0_ENABLED

    cyg_uint32 pllopt0 = 0;
    #ifdef CYGNUM_HAL_OSCILATORS_PLL0_VCO_FREQ
    pllopt0 |= 1;
    #endif

    #ifdef CYGNUM_HAL_OSCILATORS_PLL0_OUTPUT_DIVIDER
    pllopt0 |= 2;
    #endif

    #ifdef CYGNUM_HAL_OSCILATORS_PLL0_BANDWIDTH_MODE
    pllopt0 |= 4;
    #endif
    SCIF_UNLOCK(AVR32_SCIF_PLL0);
    AVR32_SCIF.pll0 = 
    ((PLL_SOURCE(CYGNUM_HAL_OSCILATORS_PLL0_SOURCE)) << AVR32_SCIF_PLLOSC_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_PLL0_DIVIDER << AVR32_SCIF_PLL0_PLLDIV_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_PLL0_MULTIPLIER << AVR32_SCIF_PLL0_PLLMUL_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_PLL0_START_COUNT << AVR32_SCIF_PLL0_PLLCOUNT_OFFSET)
    | (pllopt0 << AVR32_SCIF_PLLOPT_OFFSET)
    | (CYGNUM_HAL_OSCILATORS_PLL0_ENABLED << AVR32_SCIF_PLL0_PLLEN_OFFSET);					   

#endif //CYGNUM_HAL_OSCILATORS_PLL0_ENABLED

    // DFLL0 configuration
#ifdef CYGNUM_HAL_OSCILATORS_DFLL0_ENABLED
#ifdef CYGNUM_HAL_OSCILATORS_DFLL0_MODE
    hal_init_dfll_closedloop();
#else
    hal_init_dfll_openloop();
#endif
#endif
    
    // RC32K settings
#ifndef CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED
    SCIF_UNLOCK(AVR32_SCIF_RC32KCR);
    AVR32_SCIF.rc32kcr = 0;
#endif //CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED
}

//-----------------------------------------------------------------------------
// Initialize clock sources for clock domain cpu, pba, pbb and pbc
// domains
void hal_clocks_init(void)
{
    cyg_uint32 wait_loop = 500;
    //register dummy read
    AVR32_PM.sr;

    //wait for main clock startup
    while(!(AVR32_PM.sr & AVR32_PM_SR_CKRDY_MASK))
    {
        wait_loop--;
        if(wait_loop == 0)
        {
            _hw_error |= HW_ERROR_OSC0_NOT_RUNNING;
            return;
        }
    }
    //Configure flash speek mode
    flashcdw_set_flash_waitstate_and_readmode(CYGHWR_HAL_AVR32_CPU_FREQ*1000000);
	
#ifdef CYGNUM_HAL_CPU_CLOCK_DIVIDER
    PM_UNLOCK(AVR32_PM_CPUSEL);
    AVR32_PM.cpusel = AVR32_PM_CPUSEL_CPUDIV_MASK | CYGNUM_HAL_CPU_CLOCK_DIVIDER;
#else
    AVR32_PM.cpusel = 0;
#endif
    
#ifdef CYGNUM_HAL_PBA_CLOCK_DIVIDER
    PM_UNLOCK(AVR32_PM_PBASEL);
    AVR32_PM.pbasel = AVR32_PM_PBASEL_PBADIV_MASK | CYGNUM_HAL_PBA_CLOCK_DIVIDER;
#else
    AVR32_PM.pbasel = 0;
#endif

#ifdef CYGNUM_HAL_PBB_CLOCK_DIVIDER
    PM_UNLOCK(AVR32_PM_PBBSEL);
    AVR32_PM.pbbsel = AVR32_PM_PBBSEL_PBBDIV_MASK | CYGNUM_HAL_PBB_CLOCK_DIVIDER;
#else
    AVR32_PM.pbbsel = 0;
#endif


    //main clock source settings
    PM_UNLOCK(AVR32_PM_MCCTRL);
#if CYGNUM_HAL_MAIN_CLOCK_SOURCE == OSC0
    AVR32_PM.mcctrl = 1;
#elif CYGNUM_HAL_MAIN_CLOCK_SOURCE == DFLL
    AVR32_PM.mcctrl = 2;
#elif CYGNUM_HAL_MAIN_CLOCK_SOURCE == RC120M
    AVR32_PM.mcctrl = 3;
#else
    AVR32_PM.mcctrl = 0;
#endif
}

//-----------------------------------------------------------------------------
// Initalice Brown out detection 5V desabled
void hal_bpd_init(void)
{
    /*SCIF_UNLOCK(AVR32_SCIF_BOD50);
    AVR32_SCIF.bod50 = 0x80000100;*/
  while((AVR32_SCIF.pclksr & AVR32_SCIF_PCLKSR_BODDET_MASK))
  {

  }
}

//-----------------------------------------------------------------------------
// Interrupt initialisation for PM and SCIF modules

static cyg_interrupt     pm_interrupt;        // PM interrupt object
static cyg_handle_t      pm_interrupt_handle; // PM interrupt handle
static cyg_interrupt     scif_interrupt;        // SCIF interrupt object
static cyg_handle_t      scif_interrupt_handle; // SCIF interrupt handle

static cyg_uint32 pm_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void pm_avr32_DSR(cyg_vector_t   vector,
                         cyg_ucount32   count,
                         cyg_addrword_t data);
                         
static cyg_uint32 scif_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void scif_avr32_DSR(cyg_vector_t   vector,
                           cyg_ucount32   count,
                           cyg_addrword_t data);

void hal_init_error_interrupts()
{
    // Create and attach PMC interrupt object
    cyg_drv_interrupt_create(CYGNUM_HAL_VECTOR_PMC,
                             0,
                             (cyg_addrword_t)NULL,
                             &pm_avr32_ISR,
                             &pm_avr32_DSR,
                             &pm_interrupt_handle,
                             &pm_interrupt);

    cyg_drv_interrupt_attach(pm_interrupt_handle);

    // Create and attach SCIF interrupt object
    cyg_drv_interrupt_create(CYGNUM_HAL_VECTOR_SCIF,
                             0,
                             (cyg_addrword_t)NULL,
                             &scif_avr32_ISR,
                             &scif_avr32_DSR,
                             &scif_interrupt_handle,
                             &scif_interrupt);

    cyg_drv_interrupt_attach(scif_interrupt_handle);

    // enable clock failure detection
    PM_UNLOCK(AVR32_PM_CFDCTRL);
    AVR32_PM.icr = 1;
    
    // enable clock failure interrrupt
    PM_UNLOCK(AVR32_PM_ICR);
    AVR32_PM.icr = 1;
    PM_UNLOCK(AVR32_PM_IER);
    AVR32_PM.ier = 1;

   /* SCIF_UNLOCK(AVR32_SCIF_ICR);
    AVR32_SCIF.icr = 0xffffffff;
    SCIF_UNLOCK(AVR32_SCIF_IER);
    AVR32_SCIF.ier = 0xffffffff & ~(0x10 | 0x20);*/
}

//-----------------------------------------------------------------------------
// PM module interrupt clock loose.
static cyg_uint32 pm_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    //CYG_ASSERT(false,"Clock losed\n");
    diag_printf("Clock loose\n");
    Reset_CPU();
    return CYG_ISR_HANDLED; 
}
//-----------------------------------------------------------------------------
// PM module DSR. Not realy used.
static void pm_avr32_DSR(cyg_vector_t   vector,
                         cyg_ucount32   count,
                         cyg_addrword_t data)
{
}
  
//-----------------------------------------------------------------------------
// SCIF module interrupt. Prints error message only
static cyg_uint32 scif_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_uint32 status;
    //PM_UNLOCK(AVR32_PM_ISR);
    status = AVR32_SCIF.isr;
    
    diag_printf("Status: %d\n",status);
    while(status)
    {
        if(status&0x0001)
        {
            diag_printf("OSC0 Redy\n");
            status &= ~0x0001;
        }
        if(status&0x0002)
        {
            diag_printf("OSC1 Redy\n");
            status &= ~0x0002;
        }
        if(status&0x0004)
        {
            diag_printf("32k Redy\n");
            status &= ~0x0004;
        }
        if(status&0x0008)
        {
            diag_printf("8M Redy\n");
            status &= ~0x0008;
        }
        if(status&0x0010)
        {
            diag_printf("PLL0 locked\n");
            status &= ~0x0010;
        }
        if(status&0x0020)
        {
            diag_printf("PLL1 locked\n");
            status &= ~0x0020;
        }
        if(status&0x0040)
        {
            diag_printf("1.8 Brown out detection\n");
            status &= ~0x0040;
        }
        if(status&0x0080)
        {
            diag_printf("3.3 Brown out detection\n");
            status &= ~0x0080;
        }
        if(status&0x0100)
        {
            diag_printf("5 Brown out detection\n");
            status &= ~0x0100;
        }
        if(status&0x0200)
        {
            diag_printf("PLL0 lock lost\n");
            status &= ~0x0200;
        }
        if(status&0x0400)
        {
            diag_printf("PLL1 lock lost\n");
            status &= ~0x0400;
        }
        if(status&0x80000000)
        {
            diag_printf("SCIF acces error\n");
            status &= ~0x0400;
        }
    }
    
    AVR32_SCIF.icr = 0xffffffff;
    return CYG_ISR_HANDLED; 
}
//-----------------------------------------------------------------------------
// SCIF module DSR. Not realy used.
static void scif_avr32_DSR(cyg_vector_t   vector,
                         cyg_ucount32   count,
                         cyg_addrword_t data)
{
}                    

// EOF hal_aux.c