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
// Author(s):   hmt
// Contributors:hmt
// Date:        2003-02-28
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
#include <cyg/hal/sdramc.h>
#include <pkgconf/system.h>
#include <flash_avr32_uc3c.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
#include <pkgconf/hal.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>


#define HW_ERROR_OSC0_NOT_RUNNING       0x0000000000000001
#define HW_ERROR_OSC1_NOT_RUNNING       0x0000000000000002
#define HW_ERROR_OSC32_NOT_RUNNING      0x0000000000000004
#define HW_ERROR_RC8M_NOT_RUNNING       0x0000000000000008
#define HW_ERROR_MAIN_CLOCK_NOT_RUNNIGN     0x0000000000000010

#include CYGBLD_HAL_PLATFORM_H

//! Unlock SCIF register macro
#define SCIF_UNLOCK(reg)  (AVR32_SCIF.unlock = (AVR32_SCIF_UNLOCK_KEY_VALUE << AVR32_SCIF_UNLOCK_KEY_OFFSET) | (reg))

//! Unlock PM register macro
#define PM_UNLOCK(reg)  (AVR32_PM.unlock = (AVR32_PM_UNLOCK_KEY_VALUE << AVR32_PM_UNLOCK_KEY_OFFSET) | (reg))

#define OSC0 	AVR32_PM_MCCTRL_MCSEL_OSC0
#define OSC1 	AVR32_PM_MCCTRL_MCSEL_OSC1
#define PLL0 	AVR32_PM_MCCTRL_MCSEL_PLL0
#define PLL1 	AVR32_PM_MCCTRL_MCSEL_PLL1
#define RCOSC8  AVR32_PM_MCCTRL_MCSEL_RCOSC8
#define RC120M  AVR32_PM_MCCTRL_MCSEL_RC120M
//Prijde upravit v ucf
#define RC8M  AVR32_PM_MCCTRL_MCSEL_RCOSC8

#define PLL_SOURCE(_source_)  (_source_ == AVR32_PM_MCCTRL_MCSEL_OSC0) ? 0 : \
                              (_source_ == AVR32_PM_MCCTRL_MCSEL_OSC1) ? 1 : 2  

//#define PM_SOURCE(_source_)  AVR32_PM_MCCTRL_MCSEL_##_source_

void hal_clocks_init(void);
void hal_oscilator_init(void);
void hal_bpd_init(void);
void hal_init_error_interrupts(void);

extern void hal_board_init(void);

externC cyg_uint64	_hw_error;

//--------------------------------------------------------------------------
// Platform init code.
void
hal_platform_init(void)
{
    // Basic hardware initialization has already taken place
    //hal_vsr_table_init();
    hal_bpd_init();
    hal_board_init();
    HAL_init_interrupts();
    
    hal_if_init();   // Initialize logical I/O layer (virtual vector support)
    hal_oscilator_init();
    hal_clocks_init();
    hal_init_error_interrupts();
      
    //sdramc_init(CYGHWR_HAL_AVR32_CPU_FREQ*1000000);
}


void hal_oscilator_init(void)
{
    cyg_uint32 wait_loop = 500;
	SCIF_UNLOCK(AVR32_SCIF_VREGCTRL);
	AVR32_SCIF.vregctrl = (0x3 << 2);
//oscilator 0 settings
#ifdef CYGNUM_HAL_OSCILATORS_OSC0_ENABLED
	cyg_uint32 gain = ((CYGNUM_HAL_OSCILATORS_OSC0_FREQV) <  2 ) ? AVR32_SCIF_OSCCTRL0_GAIN_G0 :
			  ((CYGNUM_HAL_OSCILATORS_OSC0_FREQV) <  10) ? AVR32_SCIF_OSCCTRL0_GAIN_G1 :
			  ((CYGNUM_HAL_OSCILATORS_OSC0_FREQV) <  16) ? AVR32_SCIF_OSCCTRL0_GAIN_G2 :
			                                               AVR32_SCIF_OSCCTRL0_GAIN_G3;
	SCIF_UNLOCK(AVR32_SCIF_OSCCTRL);				  
#ifdef CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL	
#ifdef CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE				  
	AVR32_SCIF.oscctrl[0] = (CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE << AVR32_SCIF_OSCCTRL_AGC_OFFSET)
	                      | (gain << AVR32_SCIF_OSCCTRL_GAIN_OFFSET )
						  | (CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL << AVR32_SCIF_OSCCTRL_MODE_OFFSET)
						  | (CYGNUM_HAL_OSCILATORS_OSC0_ENABLED << AVR32_SCIF_OSCCTRL_OSCEN_OFFSET);
#else
		AVR32_SCIF.oscctrl[0] = (gain << AVR32_SCIF_OSCCTRL_GAIN_OFFSET )
						  | (CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL << AVR32_SCIF_OSCCTRL_MODE_OFFSET)
						  | (CYGNUM_HAL_OSCILATORS_OSC0_ENABLED << AVR32_SCIF_OSCCTRL_OSCEN_OFFSET);
#endif //CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE
	while(!(AVR32_SCIF.pclksr & AVR32_SCIF_ISR_OSC0RDY_MASK))
	{
	    wait_loop--;
	    if(wait_loop == 0)
	    {
	        _hw_error |= HW_ERROR_OSC0_NOT_RUNNING;
		gpio_set_pin_high(AVR32_PIN_PB22);
	        //wait_loop = 500;
	        break;
	    }
	} 
#else
	AVR32_SCIF.oscctrl[0] = (CYGNUM_HAL_OSCILATORS_OSC0_AGC_ENABLE << AVR32_SCIF_OSCCTRL_AGC_OFFSET)
	| (gain << AVR32_SCIF_OSCCTRL_GAIN_OFFSET )
	| (CYGNUM_HAL_OSCILATORS_OSC0_EXTERNAL_STARTUP_TIME << AVR32_SCIF_OSCCTRL_STARTUP_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_OSC0_ENABLED << AVR32_SCIF_OSCCTRL_OSCEN_OFFSET);
#endif //CYGNUM_HAL_OSCILATORS_OSC0_CRYSTAL	
#endif //CYGNUM_HAL_OSCILATORS_OSC0_ENABLED

//oscilator 1 settings
#ifdef CYGNUM_HAL_OSCILATORS_OSC1_ENABLED
	cyg_uint32 gain = (CYGNUM_HAL_OSCILATORS_OSC1_FREQV <  900000) ? AVR32_SCIF_OSCCTRL1_GAIN_G0 :
					  (CYGNUM_HAL_OSCILATORS_OSC1_FREQV < 3000000) ? AVR32_SCIF_OSCCTRL1_GAIN_G1 :
					  (CYGNUM_HAL_OSCILATORS_OSC1_FREQV < 8000000) ? AVR32_SCIF_OSCCTRL1_GAIN_G2 :
					  AVR32_SCIF_OSCCTRL1_GAIN_G3;
    SCIF_UNLOCK(AVR32_SCIF_OSCCTRL);
#ifdef CYGNUM_HAL_OSCILATORS_OSC1_CRYSTAL
	AVR32_SCIF.OSCCTRL[1] = (CYGNUM_HAL_OSCILATORS_OSC1_AGC_ENABLE << AVR32_SCIF_OSCCTRL_AGC_OFFSET)
						| (gain << AVR32_SCIF_OSCCTRL_GAIN_OFFSET )
						| (CYGNUM_HAL_OSCILATORS_OSC1_CRYSTAL << AVR32_SCIF_OSCCTRL_MODE_OFFSET)
						| (CYGNUM_HAL_OSCILATORS_OSC1_ENABLED << AVR32_SCIF_OSCCTRL_OSCEN_OFFSET);
#else
	AVR32_SCIF.OSCCTRL[1] = (CYGNUM_HAL_OSCILATORS_OSC1_AGC_ENABLE << AVR32_SCIF_OSCCTRL_AGC_OFFSET)
						| (gain << AVR32_SCIF_OSCCTRL_GAIN_OFFSET )
						| (CYGNUM_HAL_OSCILATORS_OSC1_EXTERNAL_STARTUP_TIME << AVR32_SCIF_OSCCTRL_STARTUP_OFFSET)
						| (CYGNUM_HAL_OSCILATORS_OSC1_ENABLED << AVR32_SCIF_OSCCTRL_OSCEN_OFFSET);

    wait_loop = 500;
	while(!(AVR32_SCIF.pclksr & AVR32_SCIF_ISR_OSC1RDY_MASK))
	{
	    wait_loop--;
	    if(wait_loop == 0)
	    {
	        _hw_error |= HW_ERROR_OSC1_NOT_RUNNING;
		gpio_set_pin_high(AVR32_PIN_PB22);
	        //wait_loop = 500;
	        break;
	    }
	} 
#endif //CYGNUM_HAL_OSCILATORS_OSC1_CRYSTAL
#endif //CYGNUM_HAL_OSCILATORS_OSC1_ENABLED


//32k oscilator settings
#ifdef CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED
	SCIF_UNLOCK(AVR32_SCIF_OSCCTRL32);
#ifdef CYGNUM_HAL_OSCILATORS_OSC32K_EXTERNAL
	AVR32_SCIF.oscctrl32 = (AVR32_SCIF_OSCCTRL32_MODE_EXT_CLOCK << AVR32_SCIF_OSCCTRL32_MODE_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_OSC32K_EXTERNAL_STARTUP_TIME << AVR32_SCIF_OSCCTRL32_STARTUP_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED << AVR32_SCIF_OSCCTRL32_OSC32EN_OFFSET);
#else
	#ifdef CYGNUM_HAL_OSCILATORS_OSC32K_MODE
	AVR32_SCIF.oscctrl32 = (AVR32_SCIF_OSCCTRL32_MODE_CRYSTAL_NO_ACG << AVR32_SCIF_OSCCTRL32_MODE_OFFSET)
		| (2 << AVR32_SCIF_OSCCTRL32_STARTUP_OFFSET)
		| (CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED << AVR32_SCIF_OSCCTRL32_OSC32EN_OFFSET);
	#else
	AVR32_SCIF.oscctrl32 = (AVR32_SCIF_OSCCTRL32_MODE_CRYSTAL_ACG << AVR32_SCIF_OSCCTRL32_MODE_OFFSET)
		| (2 << AVR32_SCIF_OSCCTRL32_STARTUP_OFFSET)
		| (CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED << AVR32_SCIF_OSCCTRL32_OSC32EN_OFFSET);
	#endif //CYGNUM_HAL_OSCILATORS_OSC32K_MODE
#endif //CYGNUM_HAL_OSCILATORS_OSC32K_EXTERNAL
    wait_loop = 50000;
	while(!(AVR32_SCIF.pclksr & AVR32_SCIF_ISR_OSC32RDY_MASK))
	{
	    wait_loop--;
	    if(wait_loop == 0)
	    {
	        _hw_error |= HW_ERROR_OSC32_NOT_RUNNING;
		gpio_set_pin_high(AVR32_PIN_PB22);
	        //wait_loop = 500;
	        break;
	    }
	} 
#endif //CYGNUM_HAL_OSCILATORS_OSC32K_ENABLED


// RC120M settings
#ifdef CYGNUM_HAL_OSCILATORS_RC120M_ENABLED
	SCIF_UNLOCK(AVR32_SCIF_RC120MCR);
	AVR32_SCIF.rc120mcr = AVR32_SCIF_RC120MCR_EN_MASK;
#endif //CYGNUM_HAL_OSCILATORS_RC120M_ENABLED

// RC8M settings
#ifdef CYGNUM_HAL_OSCILATORS_RC8M_ENABLED
	unsigned int* calibration_bits = (unsigned int*)0x80800200;
	wait_loop = 500;
	SCIF_UNLOCK(AVR32_SCIF_RCCR8);
	AVR32_SCIF.rccr8 = AVR32_SCIF_RCCR8_RCOSC8_EN_MASK | ((*calibration_bits)&AVR32_SCIF_RCCR8_CALIB_MASK);
	while(!(AVR32_SCIF.pclksr & AVR32_SCIF_ISR_RCOSC8MRDY_MASK))
	{
	    wait_loop--;
	    if(wait_loop == 0)
	    {
	        _hw_error |= HW_ERROR_RC8M_NOT_RUNNING;
		gpio_set_pin_high(AVR32_PIN_PB22);
	        wait_loop = 500;
	        break;
	    }
	} 
#endif //CYGNUM_HAL_OSCILATORS_RC8M_ENABLED

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
    SCIF_UNLOCK(AVR32_SCIF_PLL + 4*0);
	AVR32_SCIF.pll[0] = ((PLL_SOURCE(CYGNUM_HAL_OSCILATORS_PLL0_SOURCE)) << AVR32_SCIF_PLLOSC_OFFSET)
					   | (CYGNUM_HAL_OSCILATORS_PLL0_DIVIDER << AVR32_SCIF_PLL_PLLDIV_OFFSET)
					   | (CYGNUM_HAL_OSCILATORS_PLL0_MULTIPLIER << AVR32_SCIF_PLL_PLLMUL_OFFSET)
					   | (CYGNUM_HAL_OSCILATORS_PLL0_START_COUNT << AVR32_SCIF_PLL_PLLCOUNT_OFFSET)
					   | (pllopt0 << AVR32_SCIF_PLLOPT_OFFSET)
					   | (CYGNUM_HAL_OSCILATORS_PLL0_ENABLED << AVR32_SCIF_PLL_PLLEN_OFFSET);					   

#endif //CYGNUM_HAL_OSCILATORS_PLL0_ENABLED

// PLL1 configuration
#ifdef CYGNUM_HAL_OSCILATORS_PLL1_ENABLED

	cyg_uint32 pllopt1 = 0;
#ifdef CYGNUM_HAL_OSCILATORS_PLL1_VCO_FREQ
	pllopt1 |= 1;
#endif

#ifdef CYGNUM_HAL_OSCILATORS_PLL1_OUTPUT_DIVIDER
	pllopt1 |= 2;
#endif

#ifdef CYGNUM_HAL_OSCILATORS_PLL1_BANDWIDTH_MODE
	pllopt1 |= 4;
#endif
	SCIF_UNLOCK(AVR32_SCIF_PLL + 4*1);
	AVR32_SCIF.pll[1] = ((PLL_SOURCE(CYGNUM_HAL_OSCILATORS_PLL1_SOURCE)) << AVR32_SCIF_PLLOSC_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_PLL1_DIVIDER << AVR32_SCIF_PLL_PLLDIV_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_PLL1_MULTIPLIER << AVR32_SCIF_PLL_PLLMUL_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_PLL1_START_COUNT << AVR32_SCIF_PLL_PLLCOUNT_OFFSET)
	| (pllopt1 << AVR32_SCIF_PLLOPT_OFFSET)
	| (CYGNUM_HAL_OSCILATORS_PLL1_ENABLED << AVR32_SCIF_PLL_PLLEN_OFFSET);

#endif //CYGNUM_HAL_OSCILATORS_PLL1_ENABLED

}

void hal_clocks_init(void)
{
    cyg_uint32 wait_loop = 500;
	AVR32_PM.sr;

    
		
	//wait for main clock startup
	while(!(AVR32_PM.sr & AVR32_PM_SR_CKRDY_MASK))
	{
	    wait_loop--;
	    if(wait_loop == 0)
	    {
	        _hw_error |= HW_ERROR_OSC0_NOT_RUNNING;
		gpio_set_pin_high(AVR32_PIN_PB22);
	        return;
	    }
	}
	
	flashc_set_flash_waitstate_and_readmode(CYGHWR_HAL_AVR32_CPU_FREQ*1000000);
	
	PM_UNLOCK(AVR32_PM_CPUSEL);
#ifdef CYGNUM_HAL_CPU_CLOCK_DIVIDER
	AVR32_PM.cpusel = AVR32_PM_CPUSEL_CPUDIV_MASK | CYGNUM_HAL_CPU_CLOCK_DIVIDER;
#else
	AVR32_PM.cpusel = 0;
#endif

	PM_UNLOCK(AVR32_PM_PBASEL);
#ifdef CYGNUM_HAL_PBA_CLOCK_DIVIDER
	AVR32_PM.pbasel = AVR32_PM_PBASEL_PBADIV_MASK | CYGNUM_HAL_PBA_CLOCK_DIVIDER;
#else
	AVR32_PM.pbasel = 0;
#endif

	PM_UNLOCK(AVR32_PM_PBBSEL);
#ifdef CYGNUM_HAL_PBB_CLOCK_DIVIDER
	AVR32_PM.pbbsel = AVR32_PM_PBBSEL_PBBDIV_MASK | CYGNUM_HAL_PBB_CLOCK_DIVIDER;
#else
	AVR32_PM.pbbsel = 0;
#endif

	PM_UNLOCK(AVR32_PM_PBCSEL);
#ifdef CYGNUM_HAL_PBC_CLOCK_DIVIDER
	AVR32_PM.pbcsel = AVR32_PM_PBCSEL_PBCDIV_MASK | CYGNUM_HAL_PBC_CLOCK_DIVIDER;
#else
	AVR32_PM.pbcsel = 0;
#endif

	//main clock source settings
	PM_UNLOCK(AVR32_PM_MCCTRL);
	AVR32_PM.mcctrl = (CYGNUM_HAL_MAIN_CLOCK_SOURCE);
	
	//AVR32_PM.mcctrl;
}

void hal_bpd_init(void)
{
    SCIF_UNLOCK(AVR32_SCIF_BOD50);
	AVR32_SCIF.bod50 = 0x80000100;
}

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

void hal_init_error_interrupts(void)
{
// Create and attach SPI interrupt object
    cyg_drv_interrupt_create(CYGNUM_HAL_VECTOR_PMC,
                             0,
                             (cyg_addrword_t)NULL,
                             &pm_avr32_ISR,
                             &pm_avr32_DSR,
                             &pm_interrupt_handle,
                             &pm_interrupt);

    cyg_drv_interrupt_attach(pm_interrupt_handle);

    // Create and attach SPI interrupt object
    cyg_drv_interrupt_create(CYGNUM_HAL_VECTOR_SCIF,
                             0,
                             (cyg_addrword_t)NULL,
                             &scif_avr32_ISR,
                             &scif_avr32_DSR,
                             &scif_interrupt_handle,
                             &scif_interrupt);

    cyg_drv_interrupt_attach(scif_interrupt_handle);



    PM_UNLOCK(AVR32_PM_ICR);
    AVR32_PM.icr = 1;
    PM_UNLOCK(AVR32_PM_IER);
    AVR32_PM.ier = 1;

    SCIF_UNLOCK(AVR32_SCIF_ICR);
    AVR32_SCIF.icr = 0xffffffff;
    SCIF_UNLOCK(AVR32_SCIF_IER);
    AVR32_SCIF.ier = 0xffffffff & ~(0x10 | 0x20);
}

static cyg_uint32 pm_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    CYG_ASSERT(false,"Clock losed\n");
    return CYG_ISR_HANDLED; 
}

static void pm_avr32_DSR(cyg_vector_t   vector,
                         cyg_ucount32   count,
                         cyg_addrword_t data)
{
}
                         
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
    //CYG_ASSERT(false,"HW failure\n");
    return CYG_ISR_HANDLED; 
}

static void scif_avr32_DSR(cyg_vector_t   vector,
                         cyg_ucount32   count,
                         cyg_addrword_t data)
{
    
}                    

// EOF hal_aux.c

