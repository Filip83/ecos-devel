/**************************************************************************/
/**
*
* @file     imx6_misc.c
*
* @brief    iMX6 Cortex-A9 core functions
*
***************************************************************************/
/*==========================================================================
//
//      imx6_misc.c
//
//      HAL misc board support code for Freescale iMX6
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003 Free Software Foundation, Inc.
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
// Contributors: ITR-GmbH
// Date:         2013-08-08
// Purpose:      HAL board support
// Description:  Implementations of HAL board interfaces
//
//####DESCRIPTIONEND####
//
//========================================================================*/


#include <pkgconf/hal.h>

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_arch.h>           // Register state info
#include <cyg/hal/hal_diag.h>
#include <cyg/hal/hal_intr.h>           // necessary?
#include <cyg/hal/hal_if.h>             // calling interface
#include <cyg/hal/hal_misc.h>           // helper functions
#ifdef CYGDBG_HAL_DEBUG_GDB_BREAK_SUPPORT
#include <cyg/hal/drv_api.h>            // HAL ISR support
#endif
#include <cyg/hal/var_io.h>
#include <cyg/hal/var_io_devs.h>
#include <cyg/hal/platform_init.h>
#include <cyg/hal/var_ccm.h>
#include <cyg/hal/var_epit.h>
#include <cyg/hal/var_timer.h>
#include <cyg/hal/var_utility.h>
#include <cyg/hal/var_spinlock.h>
#include <cyg/hal/io.h>

#include <cyg/hal/gic_registers.h>
#include <cyg/hal/iomux_define.h>
#include <cyg/hal/cortex_a9.h>
#include <cyg/hal/arm_cp_registers.h>
#include <cyg/hal/registers/regsccm.h>
#include <cyg/hal/registers/regsccmanalog.h>
#include <cyg/hal/registers/regsgpc.h>
#include <cyg/hal/registers/regsiomuxc.h>
#include <cyg/hal/registers/regsuart.h>
#include <cyg/hal/registers/regsssi.h>
#include <cyg/hal/registers/regsepit.h>
#include <cyg/hal/registers/regsgpt.h>
#include <cyg/hal/registers/regsi2c.h>
#include <cyg/hal/registers/regsspdif.h>
#include <cyg/hal/registers/regsspba.h>
#include <cyg/hal/registers/regssdmaarm.h>
#include <cyg/hal/registers/regsecspi.h>
#include <cyg/hal/registers/regssata.h>
#include <cyg/hal/registers/regsarmglobaltimer.h>
#include <cyg/hal/registers/regsocotp.h>
#include <cyg/hal/registers/regssrc.h>

//void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
void
hal_set_pin_function(cyg_uint32 pin)
{
	hal_imx6_config_pin(pin);
}

void hal_spinlock_spin(spinlock_t *lock)
{
    hal_spinlock_lock(lock, SpinlockWaitForever);
}

cyg_bool hal_spinlock_tryspin(spinlock_t *lock)
{
    return hal_spinlock_trylock(lock, SpinlockWaitForever);
}

void hal_spinlock_clear(spinlock_t *lock)
{
	cyg_bool failed = hal_spinlock_unlock(lock);
	CYG_ASSERT (!failed, "Correct spinlock spinlock");
}

void hal_spinlock_init_clear(spinlock_t *lock)
{
    lock->owner = Unlocked;
}

void hal_spinlock_init_set(spinlock_t *lock)
{
    lock->owner = Locked;
}

cyg_bool hal_spinlock_try(spinlock_t *lock)
{
    cyg_bool retval = hal_spinlock_trylock(lock, SpinlockNoWait);
    return retval;
}

void hal_spinlock_init(spinlock_t *lock)
{
    lock->owner = Unlocked;
}

cyg_bool hal_spinlock_is_locked(spinlock_t *lock)
{
    return (lock->owner != Unlocked);
}

/****************************************************************************/
/**
*
* Hardware initialization.
*
* @return   none
*
*****************************************************************************/
#include <cyg/hal/var_hab.h>

void hal_hardware_init(void)
{
    platform_init();
    touch_hab();
}

/****************************************************************************/
/**
*
* IRQ handler.
* This routine is called to respond to a hardware interrupt (IRQ).  It
* should interrogate the hardware and return the IRQ vector number.
*
* @return   none
*
*****************************************************************************/
int hal_IRQ_handler(void)
{
    cyg_uint32 vector;
    cyg_uint32 intr;

    // This will send an ACK back to the GIC, but will not yet allow more
    // interrupts which is proper eCos behavior.
    gicc_t * gicc = HAL_GICC;
    vector = gicc->IAR;

    // Strip off CPU
    intr = vector & 0x3FF;

    if (intr == 1023)
    {
        // Spurious interrupt.
        vector = CYGNUM_HAL_INTERRUPT_NONE;
    }
    else
    {
        // An invalid interrupt source is treated as a spurious interrupt    
        if (intr < CYGNUM_HAL_ISR_MIN || intr > CYGNUM_HAL_ISR_REAL_MAX)
            vector = CYGNUM_HAL_INTERRUPT_NONE;

    }
    return vector;
}

void hal_disable_interrupt(cyg_uint32 vector)
{
    gicd_t * gicd = HAL_GICD;

    cyg_uint32 reg = HAL_IRQ_REG_OFFSET(vector);
    cyg_uint32 mask = HAL_IRQ_GET_BIT_MASK(vector);
    // For edge triggered.
//    cyg_uint32 reg2 = HAL_IRQ_REG_OFFSET2(irqID);
//    cyg_uint32 mask2 = HAL_IRQ_GET_BIT_MASK2(irqID);

    // Select set-enable or clear-enable register based on enable flag.
    gicd->ICENABLERn[reg] = mask;
//        gicd->ICFGRn[reg2] |= mask2;
}

void hal_enable_interrupt(cyg_uint32 vector)
{
    gicd_t * gicd = HAL_GICD;

    cyg_uint32 reg = HAL_IRQ_REG_OFFSET(vector);
    cyg_uint32 mask = HAL_IRQ_GET_BIT_MASK(vector);
    // For edge triggered.
//    cyg_uint32 reg2 = HAL_IRQ_REG_OFFSET2(irqID);
//    cyg_uint32 mask2 = HAL_IRQ_GET_BIT_MASK2(irqID);

    gicd->ISENABLERn[reg] = mask;
//        gicd->ICFGRn[reg2] |= mask2;

}

/****************************************************************************/
/**
*
* Interrupt control: mask interrupt.
*
* @param    vector
*
* @return   none
*
*****************************************************************************/
void hal_interrupt_mask(int vector)
{
    hal_disable_interrupt(vector);
}

/****************************************************************************/
/**
*
* Interrupt control: unmask interrupt.
*
* @param    vector
*
* @return   none
*
*****************************************************************************/
void hal_interrupt_unmask(int vector)
{
    hal_enable_interrupt(vector);
}

/****************************************************************************/
/**
*
* Interrupt acknowlage.
*
* @param    vector
*
* @return   none
*
*****************************************************************************/
void hal_interrupt_acknowledge(int vector)
{
    cyg_uint32 irq_num;
//    cyg_uint32 cpu_num;

//    cpu_num = (vector >> 10) & 0x7;
    irq_num = vector & 0x1FF;

    // Even though from a hal perspective this is an ACK, it really allows
    // the GIC to deal with a new interrupt. However, the eCos documentation
    // says that the purpose of the ACK is to allow further interrupts, so
    // this does the correct thing.
    if (irq_num != CYGNUM_HAL_INTERRUPT_NONE)
    {
        gicc_t * gicc = HAL_GICC;
        gicc->EOIR = vector;
    }
}

/****************************************************************************/
/**
*
* Interrupt control: Set interrupt configuration.
*
* @param    vector - interrupt number [0..94].
* @param    level  - priority is the new priority for the IRQ source. 0x00 is highest, 0xFF lowest.
* @param    up     - trigger type for the IRQ source.
*
* @return   none
*
*****************************************************************************/
void hal_interrupt_configure(int vector, int level, int up)
{
	hal_interrupt_set_level(vector, level);
}

/****************************************************************************/
/**
*
* Interrupt control: Set reduced interrupt configuration.
*
* @param    vector -
* @param    level  - priority is the new priority for the IRQ source. 0x00 is highest, 0xFF lowest.
*
* @return   none
*
*****************************************************************************/
void hal_interrupt_set_level(int vector, int level)
{
	// Set priority
    // This needs to be called after enable or the level will be 0.
    gicd_t * gicd = HAL_GICD;
    gicd->IPRIORITYRn[vector] = level & 0xff;

    // Set security
    cyg_uint32 reg = HAL_IRQ_REG_OFFSET(vector);
    cyg_uint32 mask = HAL_IRQ_GET_BIT_MASK(vector);

    cyg_uint32 value = gicd->IGROUPRn[reg];
    value &= ~mask;
    gicd->IGROUPRn[reg] = value;
}

void hal_interrupt_set_cpu_target(cyg_uint32 vector, unsigned cpu, cyg_bool enable)
{
    // Make sure the CPU number is valid.
//    assert(cpu <= 7);

    gicd_t * gicd = HAL_GICD;
    uint8_t mask = 1 << cpu;

    // Like the priority registers, the target registers are byte accessible, and the register
    // struct has the them as a byte array, so we can just index directly by the
    // interrupt ID.
    if (enable)
    {
        gicd->ITARGETSRn[vector] |= (mask & 0xff);
    }
    else
    {
        gicd->ITARGETSRn[vector] &= ~(mask & 0xff);
    }
}

void hal_interrupt_distributor_enable(cyg_bool enable)
{
    gicd_t * gicd = HAL_GICD;

    if (enable)
    {
        // Enable both secure and non-secure.
        gicd->CTLR |= kBM_GICD_CTLR_EnableGrp0 | kBM_GICD_CTLR_EnableGrp1;
    }
    else
    {
        // Clear the enable bits.
        gicd->CTLR &= ~(kBM_GICD_CTLR_EnableGrp0 | kBM_GICD_CTLR_EnableGrp1);
    }
}

void hal_interrupt_cpu_enable(cyg_bool enable)
{
    gicc_t * gicc = HAL_GICC;

    if (enable)
    {
        gicc->CTLR |= kBM_GICC_CTLR_EnableS | kBM_GICC_CTLR_EnableNS;
    }
    else
    {
        gicc->CTLR &= ~(kBM_GICC_CTLR_EnableS | kBM_GICC_CTLR_EnableNS);
    }
}

void hal_interrupt_init(void)
{
    gicd_t * gicd = HAL_GICD;

    // First disable the distributor.
    hal_interrupt_distributor_enable(false);

    // Clear all pending interrupts.
    int i;
    for (i = 0; i < 32; ++i)
    {
        gicd->ICPENDRn[i] = 0xffffffff;
    }

    // Set all interrupts to secure.
    for (i = 0; i < 8; ++i)
    {
        gicd->IGROUPRn[i] = 0;
    }

    // Init the GIC CPU interface.
    hal_interrupt_init_cpu();

    // Now enable the distributor.
    hal_interrupt_distributor_enable(true);
}

void hal_interrupt_init_cpu(void)
{
    gicc_t * gicc = HAL_GICC;

    // Init the GIC CPU interface.
	gicc->PMR = 0xff;

    // Disable preemption.
    gicc->BPR = 7;

    // Enable signaling the CPU.
    hal_interrupt_cpu_enable(true);
}

void hal_interrupt_send_sgi(cyg_uint32 vector, cyg_uint32 target_list, cyg_uint32 filter_list)
{
    gicd_t * gicd = HAL_GICD;

    gicd->SGIR = (filter_list << kBP_GICD_SGIR_TargetListFilter)
                    | (target_list << kBP_GICD_SGIR_CPUTargetList)
                    | (vector & 0xf);
}


/****************************************************************************/
/**
*
* NOP function.
*
* @param    vector   - interrupt number [0..94].
* @param    data     - Unknown.
* @param    handler  - Unknown.
*
* @return   none
*
*****************************************************************************/
void hal_show_IRQ(int vector, int data, int handler)
{
//    UNDEFINED(__FUNCTION__);  // FIXME
}

//==========================================================================
// Pin configuration functions
//

// Very ugly. Driven by inability to convert values into macro symbols. The final solution might
// require redefining the SDK macros so this problem does not exist.
//
// Furthermore, it only supports UART at this point.

void hal_imx6_config_pin(cyg_uint32 pin)
{
    if (pin != CYGHWR_HAL_IMX6_PIN_NONE)
    {
    	switch (CYGHWR_HAL_IMX6_PIN_TYPE(pin))
    	{
    	case CYGHWR_HAL_FREESCALE_PIN_CSI0_DATA10 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(CSI0_DATA10)(
				BF_IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA10_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA10_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(CSI0_DATA10)(
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA10_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			// This might be incorrect.
            HW_IOMUXC_UART_UART_SELECT_INPUT_WR(2, RX_DATA)(
                BF_IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT_DAISY(CYGHWR_HAL_IMX6_PIN_DAISY(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_CSI0_DATA11 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(CSI0_DATA11)(
				BF_IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA11_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA11_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(CSI0_DATA11)(
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_CSI0_DATA11_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
            HW_IOMUXC_UART_UART_SELECT_INPUT_WR(1, RX_DATA)(
                BF_IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT_DAISY(CYGHWR_HAL_IMX6_PIN_DAISY(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_MDIO :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(ENET_MDIO)(
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_MDIO_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_MDIO_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(ENET_MDIO)(
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
		    HW_IOMUXC_ENET_MAC0_MDIO_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_MDC :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(ENET_MDC)(
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_MDC_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_MDC_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(ENET_MDC)(
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII_RD1 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_RD1)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD1_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD1_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_RD1)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    HW_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM_WR(CYGHWR_HAL_IMX6_PIN_ODT(pin));
		    HW_IOMUXC_ENET_MAC0_RX_DATA1_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII_RD0 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_RD0)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD0_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD0_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_RD0)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    HW_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM_WR(CYGHWR_HAL_IMX6_PIN_ODT(pin));
		    HW_IOMUXC_ENET_MAC0_RX_DATA0_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII_TD0 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_TD0)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD0_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD0_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_TD0)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII_TD1 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_TD1)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD1_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD1_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_TD1)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII0_RD3 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_RD3)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD3_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD3_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_RD3)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    HW_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM_WR(CYGHWR_HAL_IMX6_PIN_ODT(pin));
		    HW_IOMUXC_ENET_MAC0_RX_DATA3_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII0_RD2 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_RD2)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD2_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD2_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_RD2)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    HW_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM_WR(CYGHWR_HAL_IMX6_PIN_ODT(pin));
		    HW_IOMUXC_ENET_MAC0_RX_DATA2_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII0_RXC :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_RXC)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RXC_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RXC_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_RXC)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    HW_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM_WR(CYGHWR_HAL_IMX6_PIN_ODT(pin));
		    HW_IOMUXC_ENET_MAC0_RX_CLK_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII0_TD2 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_TD2)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD2_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD2_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_TD2)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII0_TXC :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_TXC)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TXC_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TXC_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_TXC)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TXC_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TXC_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TXC_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TXC_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TXC_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_RGMII0_TD3 :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_TD3)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD3_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD3_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_TD3)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
		    break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_CRS_DV :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(ENET_CRS_DV)(
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_CRS_DV_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_CRS_DV_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(ENET_CRS_DV)(
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_CRS_DV_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_REF_CLK :
			HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(ENET_REF_CLK)(
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_REF_CLK_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_REF_CLK_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
			HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(ENET_REF_CLK)(
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_RX_CTL :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_RX_CTL)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RX_CTL_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_RX_CTL_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_RX_CTL)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    	HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
			    HW_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM_WR(CYGHWR_HAL_IMX6_PIN_ODT(pin));
			    HW_IOMUXC_ENET_MAC0_RX_EN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_TX_CTL :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(RGMII_TX_CTL)(
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TX_CTL_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_RGMII_TX_CTL_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(RGMII_TX_CTL)(
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)));
		    	HW_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII_WR(CYGHWR_HAL_IMX6_PIN_IOV(pin));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_RX_DATA1 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(ENET_RX_DATA1)(
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_RX_DATA1_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_RX_DATA1_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(ENET_RX_DATA1)(
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_RX_DATA1_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_ENET_TX_EN :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(ENET_TX_EN)(
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_TX_EN_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_ENET_TX_EN_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(ENET_TX_EN)(
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_ENET_TX_EN_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_EIM_D29 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(EIM_DATA29)(
				BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA29_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA29_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(EIM_DATA29)(
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA29_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
			break;

    	case CYGHWR_HAL_FREESCALE_PIN_I2C_SDA0 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(EIM_DATA28)(
				BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(EIM_DATA28)(
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
    		HW_IOMUXC_I2C1_SDA_IN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;

    	case CYGHWR_HAL_FREESCALE_PIN_I2C_SDA1 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(KEY_ROW3)(
				BF_IOMUXC_SW_MUX_CTL_PAD_KEY_ROW3_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_KEY_ROW3_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(KEY_ROW3)(
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_ROW3_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
    		HW_IOMUXC_I2C2_SDA_IN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;

    	case CYGHWR_HAL_FREESCALE_PIN_I2C_SDA2 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(GPIO16)(
				BF_IOMUXC_SW_MUX_CTL_PAD_GPIO16_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_GPIO16_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(GPIO16)(
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO16_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
    		HW_IOMUXC_I2C3_SDA_IN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;

    	case CYGHWR_HAL_FREESCALE_PIN_I2C_SCL0 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(EIM_DATA21)(
				BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(EIM_DATA21)(
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
    		HW_IOMUXC_I2C1_SCL_IN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;

    	case CYGHWR_HAL_FREESCALE_PIN_I2C_SCL1 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(KEY_COL3)(
				BF_IOMUXC_SW_MUX_CTL_PAD_KEY_COL3_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_KEY_COL3_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(KEY_COL3)(
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_KEY_COL3_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
    		HW_IOMUXC_I2C2_SCL_IN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;

    	case CYGHWR_HAL_FREESCALE_PIN_I2C_SCL2 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(GPIO05)(
				BF_IOMUXC_SW_MUX_CTL_PAD_GPIO05_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_GPIO05_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(GPIO05)(
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_GPIO05_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));
    		HW_IOMUXC_I2C3_SCL_IN_SELECT_INPUT_WR(CYGHWR_HAL_IMX6_PIN_DAISY(pin));
			break;
    	case CYGHWR_HAL_FREESCALE_PIN_DISP0_DAT20 :
    		HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(DISP0_DATA20)(
				BF_IOMUXC_SW_MUX_CTL_PAD_DISP0_DATA20_SION(CYGHWR_HAL_IMX6_PIN_SION(pin)) |
				BF_IOMUXC_SW_MUX_CTL_PAD_DISP0_DATA20_MUX_MODE(CYGHWR_HAL_IMX6_PIN_ALT(pin)));
    		HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(DISP0_DATA20)(
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_HYS(CYGHWR_HAL_IMX6_PIN_HYS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_PUS(CYGHWR_HAL_IMX6_PIN_PUS(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_PUE(CYGHWR_HAL_IMX6_PIN_PUE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_PKE(CYGHWR_HAL_IMX6_PIN_PKE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_ODE(CYGHWR_HAL_IMX6_PIN_ODE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_SPEED(CYGHWR_HAL_IMX6_PIN_SPD(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_DSE(CYGHWR_HAL_IMX6_PIN_DSE(pin)) |
				BF_IOMUXC_SW_PAD_CTL_PAD_DISP0_DATA20_SRE(CYGHWR_HAL_IMX6_PIN_SRE(pin)));

			break;
    	default:
    		break;
    	}
    }
}

void hal_imx6_config_pin_daisy(cyg_uint32 periph, cyg_uint32 type, cyg_uint32 port, cyg_uint32 pin)
{
    if (pin != CYGHWR_HAL_IMX6_PIN_NONE)
    {
        // Only RX and RTS have select input registers
        if (periph == CYGHWR_HAL_FREESCALE_PERIPH_UART) {
            if (type == CYGHWR_HAL_FREESCALE_PIN_FUN_RX) {
                if (port == 1)
                    HW_IOMUXC_UART_UART_SELECT_INPUT_WR(1, RX_DATA)(
                        BF_IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT_DAISY(CYGHWR_HAL_IMX6_PIN_DAISY(pin)));
                else
                    HW_IOMUXC_UART_UART_SELECT_INPUT_WR(2, RX_DATA)(
                        BF_IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT_DAISY(CYGHWR_HAL_IMX6_PIN_DAISY(pin)));
            }
            else if (type == CYGHWR_HAL_FREESCALE_PIN_FUN_RTS) {
                if (port == 1)
                    HW_IOMUXC_UART_UART_SELECT_INPUT_WR(1, RTS_B)(
                        BF_IOMUXC_UART1_UART_RTS_B_SELECT_INPUT_DAISY(CYGHWR_HAL_IMX6_PIN_DAISY(pin)));
                else
                    HW_IOMUXC_UART_UART_SELECT_INPUT_WR(2, RTS_B)(
                        BF_IOMUXC_UART1_UART_RTS_B_SELECT_INPUT_DAISY(CYGHWR_HAL_IMX6_PIN_DAISY(pin)));
            }
        }
    }
}

/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL FREESCALE BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

//==========================================================================
// Clock distribution
//

void hal_clock_enable(cyg_uint32 desc)
{
    volatile cyg_uint32 *ccgr_p;

    if(desc != CYGHWR_HAL_CCGR_NONE) {
        ccgr_p = ((cyg_uint32*) HW_CCM_CCGR0_ADDR) +
                 CYGHWR_HAL_IMX6_SIM_CCGR_REG(desc);
        *ccgr_p |= CYGHWR_HAL_IMX6_SIM_CCGR_VAL0(desc) << (CYGHWR_HAL_IMX6_SIM_CCGR_BIT0(desc) * 2);
        *ccgr_p |= CYGHWR_HAL_IMX6_SIM_CCGR_VAL1(desc) << (CYGHWR_HAL_IMX6_SIM_CCGR_BIT1(desc) * 2);
    }
}

void hal_clock_disable(cyg_uint32 desc)
{
    volatile cyg_uint32 *ccgr_p;

    if(desc != CYGHWR_HAL_CCGR_NONE) {
        ccgr_p = ((cyg_uint32*) HW_CCM_CCGR0_ADDR) +
                 CYGHWR_HAL_IMX6_SIM_CCGR_REG(desc);
        *ccgr_p &= ~(0x11 << (CYGHWR_HAL_IMX6_SIM_CCGR_BIT0(desc) * 2));
        *ccgr_p &= ~(0x11 << (CYGHWR_HAL_IMX6_SIM_CCGR_BIT1(desc) * 2));
    }
}

//==========================================================================
// CCM Clock Control Module
//


const cyg_uint32 PLL1_OUTPUT = 792000000;
const cyg_uint32 PLL2_OUTPUT[] = { 528000000, 396000000, 352000000, 198000000, 594000000 };
const cyg_uint32 PLL3_OUTPUT[] = { 480000000, 720000000, 540000000, 508235294, 454736842 };
const cyg_uint32 PLL4_OUTPUT = 650000000;
const cyg_uint32 PLL5_OUTPUT = 650000000;

void hal_ccm_init(void)
{
    // ETHNET
    HW_CCM_ANALOG_PLL_ENET_CLR(BM_CCM_ANALOG_PLL_ENET_POWERDOWN);
    HW_CCM_ANALOG_PLL_ENET_SET(BM_CCM_ANALOG_PLL_ENET_ENABLE);
    HW_CCM_ANALOG_PLL_ENET_CLR(BM_CCM_ANALOG_PLL_ENET_BYPASS);
    HW_CCM_ANALOG_PLL_ENET.B.DIV_SELECT = 0x3;

    // Ungate clocks that are not enabled in a driver - need to be updated 
    HW_CCM_CCGR0_WR(0xffffffff);
    HW_CCM_CCGR1_WR(0xFFCF0FFF);    // EPIT, ESAI, GPT enabled by driver
    HW_CCM_CCGR2_WR(0xFFFFF03F);    // I2C enabled by driver
    HW_CCM_CCGR3_WR(0xffffffff);
    HW_CCM_CCGR4_WR(0x00FFFF03);    // GPMI, Perfmon enabled by driver
    HW_CCM_CCGR5_WR(0xF0FFFFCF);    // UART, SATA enabled by driver
    HW_CCM_CCGR6_WR(0xffffffff);

    /*
     * Keep default settings at reset.
     * pre_periph_clk_sel is by default at 0, so the selected output
     * of PLL2 is the main output at 528MHz.
     * => by default, ahb_podf divides by 4 => AHB_CLK@132MHz.
     * => by default, ipg_podf divides by 2 => IPG_CLK@66MHz.
     */
    HW_CCM_CBCDR.U = BF_CCM_CBCDR_AHB_PODF(3)
        | BF_CCM_CBCDR_AXI_PODF(1)
        | BF_CCM_CBCDR_IPG_PODF(1);

    /*
     * UART clock tree: PLL3 (480MHz) div-by-6: 80MHz
     * 80MHz uart_clk_podf (div-by-1) = 80MHz (UART module clock input)
     */
//    writel(readl(CCM_CSCDR1) & 0x0000003F, CCM_CSCDR1);
//     HW_CCM_CSCDR1.U = 

    /* Mask all interrupt sources that could wake up the processor when in
       a low power mode. A source is individually masked/unmasked when the 
       interrupt is enabled/disabled by the GIC/interrupt driver. */
    HW_GPC_IMR1_WR(0xFFFFFFFF);
    HW_GPC_IMR2_WR(0xFFFFFFFF);
    HW_GPC_IMR3_WR(0xFFFFFFFF);
    HW_GPC_IMR4_WR(0xFFFFFFFF);
}

cyg_uint32 hal_get_main_clock(main_clocks_t clock)
{
    cyg_uint32 ret_val = 0;
    cyg_uint32 pre_periph_clk_sel = HW_CCM_CBCMR.B.PRE_PERIPH_CLK_SEL;

    switch (clock) {
    case CPU_CLK:
        ret_val = PLL1_OUTPUT;
        break;
    case AXI_CLK:
        ret_val = PLL2_OUTPUT[pre_periph_clk_sel] / (HW_CCM_CBCDR.B.AXI_PODF + 1);
        break;
    case MMDC_CH0_AXI_CLK:
        ret_val = PLL2_OUTPUT[pre_periph_clk_sel] / (HW_CCM_CBCDR.B.MMDC_CH0_AXI_PODF + 1);
        break;
    case AHB_CLK:
        ret_val = PLL2_OUTPUT[pre_periph_clk_sel] / (HW_CCM_CBCDR.B.AHB_PODF + 1);
        break;
    case IPG_CLK:
        ret_val =
            PLL2_OUTPUT[pre_periph_clk_sel] / (HW_CCM_CBCDR.B.AHB_PODF +
                                               1) / (HW_CCM_CBCDR.B.IPG_PODF + 1);
        break;
    case IPG_PER_CLK:
        ret_val =
            PLL2_OUTPUT[pre_periph_clk_sel] / (HW_CCM_CBCDR.B.AHB_PODF +
                                               1) / (HW_CCM_CBCDR.B.IPG_PODF +
                                                     1) / (HW_CCM_CSCMR1.B.PERCLK_PODF + 1);
        break;
    case MMDC_CH1_AXI_CLK:
        ret_val = PLL2_OUTPUT[pre_periph_clk_sel] / (HW_CCM_CBCDR.B.MMDC_CH1_AXI_PODF + 1);
        break;
    default:
        break;
    }

    return ret_val;
}

cyg_uint32 hal_get_peri_clock(peri_clocks_t clock)
{
    cyg_uint32 ret_val = 0;

    switch (clock)
    {
        case UART1_MODULE_CLK:
        case UART2_MODULE_CLK:
        case UART3_MODULE_CLK:
        case UART4_MODULE_CLK:
            // UART source clock is a fixed PLL3 / 6
            ret_val = PLL3_OUTPUT[0] / 6 / (HW_CCM_CSCDR1.B.UART_CLK_PODF + 1);
            break;

        // eCSPI clock:
        //     PLL3(480) -> /8 -> CSCDR2[ECSPI_CLK_PODF]
        case SPI_CLK:
            ret_val = PLL3_OUTPUT[0] / 8 / (HW_CCM_CSCDR2.B.ECSPI_CLK_PODF + 1);
            break;

        case RAWNAND_CLK:
            ret_val =
                PLL3_OUTPUT[0] / (HW_CCM_CS2CDR.B.ENFC_CLK_PRED + 1) / (HW_CCM_CS2CDR.B.ENFC_CLK_PODF +
                                                                        1);
            break;

        case CAN_CLK:
            // For i.mx6dq/sdl CAN source clock is a fixed PLL3 / 8
        ret_val = PLL3_OUTPUT[0] / 8 / (HW_CCM_CSCMR2.B.CAN_CLK_PODF + 1);
        break;

        default:
            break;
    }

    return ret_val;
}

/*!
 * Set/unset clock gating for a peripheral.
 * @param   ccm_ccgrx Address of the clock gating register: CCM_CCGR1,...
 * @param   cgx_offset Offset of the clock gating field: CG(x).
 * @param   gating_mode Clock gating mode: CLOCK_ON or CLOCK_OFF.
 */
void hal_ccm_ccgr_config(cyg_uint32 ccm_ccgrx, cyg_uint32 cgx_offset, cyg_uint32 gating_mode)
{
    if (gating_mode == CLOCK_ON)
    {
        *(volatile cyg_uint32 *)(ccm_ccgrx) |= cgx_offset;
    }
    else
    {
        *(volatile cyg_uint32 *)(ccm_ccgrx) &= ~cgx_offset;
    }
}

void hal_clock_gating_config(cyg_uint32 base_address, cyg_uint32 gating_mode)
{
    cyg_uint32 ccm_ccgrx = 0;
    cyg_uint32 cgx_offset = 0;

    switch (base_address)
    {
        case REGS_UART1_BASE:
        case REGS_UART2_BASE:
        case REGS_UART3_BASE:
        case REGS_UART4_BASE:
        case REGS_UART5_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(13) | CG(12);
            break;
        case REGS_SSI3_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(11);
            break;
        case REGS_SSI2_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(10);
            break;
        case REGS_SSI1_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(9);
            break;
        case REGS_SPDIF_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(7);
            break;
        case REGS_SPBA_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(6);
            break;
        case REGS_SDMAARM_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(3);
            break;
#if ( CYGPKG_HAL_SMP_CPU_MAX == 4)
        case REGS_SATA_BASE:
            ccm_ccgrx = HW_CCM_CCGR5_ADDR;
            cgx_offset = CG(2);
            break;
#endif
        case REGS_EPIT1_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(6);
            break;
        case REGS_EPIT2_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(7);
            break;
        case REGS_GPT_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(10);
            break;
        case REGS_I2C1_BASE:
            ccm_ccgrx = HW_CCM_CCGR2_ADDR;
            cgx_offset = CG(3);
            break;
        case REGS_I2C2_BASE:
            ccm_ccgrx = HW_CCM_CCGR2_ADDR;
            cgx_offset = CG(4);
            break;
        case REGS_I2C3_BASE:
            ccm_ccgrx = HW_CCM_CCGR2_ADDR;
            cgx_offset = CG(5);
            break;
        case REGS_ECSPI1_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(0);
            break;
        case REGS_ECSPI2_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(1);
            break;
        case REGS_ECSPI3_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(2);
            break;
        case REGS_ECSPI4_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(3);
            break;
#if (CYGPKG_HAL_SMP_CPU_MAX == 4)
        case REGS_ECSPI5_BASE:
            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
            cgx_offset = CG(4);
            break;
#endif
            // Add Freescale includes to use these.
//        case REGS_GPMI_BASE:
//            ccm_ccgrx = HW_CCM_CCGR4_ADDR;
//            cgx_offset = CG(15) | CG(14) | CG(13) | CG(12);
//            break;
//        case REGS_ESAI_BASE:
//            ccm_ccgrx = HW_CCM_CCGR1_ADDR;
//            cgx_offset = CG(8);
//            break;
//        case CAAM_BASE_ADDR:
//            ccm_ccgrx = HW_CCM_CCGR0_ADDR;
//            cgx_offset = CG(6) | CG(5) | CG(4);
//            break;
        default:
            break;
    }

    // apply changes only if a valid address was found
    if (ccm_ccgrx != 0)
    {
        hal_ccm_ccgr_config(ccm_ccgrx, cgx_offset, gating_mode);
    }
}

//==========================================================================
// EPIT Enhanced Periodic Interrupt Timer
//

//void hal_epit_reload_counter(cyg_uint32 instance, cyg_uint32 load_val)
//{
//    // set the load register especially if RLD=reload_mode=SET_AND_FORGET=1
//    HW_EPIT_LR_WR(instance, load_val);
//}
//
cyg_uint32 hal_epit_get_counter_value(cyg_uint32 instance)
{
    return HW_EPIT_CNR_RD(instance);
}

//void hal_epit_set_compare_event(cyg_uint32 instance, cyg_uint32 compare_val)
//{
//    HW_EPIT_CMPR_WR(instance, compare_val);
//}
//
cyg_uint32 hal_epit_get_compare_event(cyg_uint32 instance)
{
    cyg_uint32 status_register;

    // get the status
    status_register = HW_EPIT_SR_RD(instance);

    // clear it if found set
    if (status_register & BM_EPIT_SR_OCIF)
    {
        HW_EPIT_SR_SET(instance, BM_EPIT_SR_OCIF);
    }

    // return the read value before the bit was cleared
    return status_register & BM_EPIT_SR_OCIF;
}

void hal_epit_counter_disable(cyg_uint32 instance)
{
    /* temporary workaround for the discovered issue when disabling the
     * counter during end of count/reload/set compare flag ??.
     * Set to the max value so that it ensures that the counter couldn't
     * reach 0 when it is disabled.
     */
    HW_EPIT_LR_WR(instance, 0xFFFFFFFF);

    // disable the counter
    HW_EPIT_CR_CLR(instance, BM_EPIT_CR_EN);

    // ensure to leave the counter in a proper state
    // by disabling the output compare interrupt
    HW_EPIT_CR_CLR(instance, BM_EPIT_CR_OCIEN);

    // and clearing possible remaining compare event
    HW_EPIT_SR_SET(instance, BM_EPIT_SR_OCIF);
}

void hal_epit_counter_enable(cyg_uint32 instance, cyg_uint32 load_val, cyg_uint32 irq_mode)
{
    // set the load register especially if RLD=reload_mode=SET_AND_FORGET=1
    // and if the value is different from 0 which is the lowest counter value
    if (load_val != 0)
    {
        HW_EPIT_LR_WR(instance, load_val);
    }

    // ensure to start the counter in a proper state
    // by clearing possible remaining compare event
    HW_EPIT_SR_SET(instance, BM_EPIT_SR_OCIF);

    // set the mode when the output compare event occur: IRQ or polling
    if (irq_mode == IRQ_MODE)
    {
        HW_EPIT_CR_SET(instance, BM_EPIT_CR_OCIEN);
    }
    else
    {
        // polling
        HW_EPIT_CR_CLR(instance, BM_EPIT_CR_OCIEN);
    }

    // finally, enable the counter
    HW_EPIT_CR_SET(instance, BM_EPIT_CR_EN);
}

void hal_epit_init(cyg_uint32 instance, cyg_uint32 clock_src, cyg_uint32 prescaler,
               cyg_uint32 reload_mode, cyg_uint32 load_val, cyg_uint32 low_power_mode)
{
    cyg_uint32 control_reg_tmp = 0;
    cyg_uint32 base = REGS_EPIT_BASE(instance);

    // enable the source clocks to the EPIT port
    hal_clock_gating_config(base, CLOCK_ON);

    // start with a known state by disabling and reseting the module
    HW_EPIT_CR_WR(instance, BM_EPIT_CR_SWR);

    // wait for the reset to complete
    while ((HW_EPIT_CR(instance).B.SWR) != 0) ;

    // set the reference source clock for the counter
    control_reg_tmp |= BF_EPIT_CR_CLKSRC(clock_src);

    // set the counter clock prescaler value - 0 to 4095
    control_reg_tmp |= BF_EPIT_CR_PRESCALAR(prescaler-1);

    // set the reload mode
    if (reload_mode == SET_AND_FORGET)
    {
        control_reg_tmp |= BM_EPIT_CR_RLD;
    }

    // set behavior for low power mode
    if (low_power_mode & WAIT_MODE_EN)
    {
        control_reg_tmp |= BM_EPIT_CR_WAITEN;
    }
    if (low_power_mode & STOP_MODE_EN)
    {
        control_reg_tmp |= BM_EPIT_CR_STOPEN;
    }

    // make the counter start from a known value when enabled, this is loaded from
    // EPITLR register if RLD=reload_mode=1 or 0xFFFFFFFF if RLD=reload_mode=0
    control_reg_tmp |= BM_EPIT_CR_IOVW | BM_EPIT_CR_ENMOD;

    // finally write the control register
    HW_EPIT_CR_WR(instance, control_reg_tmp);

    // initialize the load register especially if RLD=reload_mode=SET_AND_FORGET=1
    // and if the value is different from 0 which is the lowest counter value
    if (load_val != 0)
    {
        HW_EPIT_LR_WR(instance, load_val);
    }
}

//==========================================================================
// GPT General Purpise Timer
//


static void hal_time_init_global_timer(void);

cyg_uint32 g_microsecondTimerMultiple;

void hal_system_time_init(void)
{
    cyg_uint32 freq;
    // Init microsecond tick counter.
    hal_time_init_global_timer();

    /* EPIT1 is used for the delay function */
    /* Initialize the EPIT timer used for system time functions */
    /* typical IPG_CLK is in MHz, so divide it to get a reference
       clock of 1MHz => 1us per count */
    freq = hal_get_main_clock(IPG_CLK);
    hal_epit_init(1, CLKSRC_IPG_CLK, freq / 1000000,
              SET_AND_FORGET, 1000, WAIT_MODE_EN | STOP_MODE_EN);
    /* EPIT2 is used for the kernel timer. */
    /* This assumes the HAL will hook the interrupt rather than
       the call in this file. */
    hal_epit_init(2, CLKSRC_IPG_CLK, freq / 1000000,
              SET_AND_FORGET, 1000, WAIT_MODE_EN | STOP_MODE_EN);
//     hal_epit_counter_enable(2, 1000, IRQ_MODE);
}

//! Init the ARM global timer to a microsecond-frequency clock.
void hal_time_init_global_timer(void)
{
    // The ARM private peripheral clock is half the CPU clock.
    cyg_uint32 periphClock = hal_get_main_clock(CPU_CLK) / 2;
    cyg_uint32 prescaler = (periphClock / 1000000) - 1;

    // Divide down the prescaler until it fits into 8 bits. We add up the number of ticks
    // it takes to equal a microsecond interval.
    g_microsecondTimerMultiple = 1;
    while (prescaler > 0xff)
    {
        prescaler /= 2;
        ++g_microsecondTimerMultiple;
    }

    // Make sure the timer is off.
    HW_ARMGLOBALTIMER_CONTROL.B.TIMER_ENABLE = 0;

    // Clear counter.
    HW_ARMGLOBALTIMER_COUNTERn_WR(0, 0);
    HW_ARMGLOBALTIMER_COUNTERn_WR(1, 0);

    // Set prescaler and clear other flags.
    HW_ARMGLOBALTIMER_CONTROL_WR(BF_ARMGLOBALTIMER_CONTROL_PRESCALER(prescaler));

    // Now turn on the timer.
    HW_ARMGLOBALTIMER_CONTROL.B.TIMER_ENABLE = 1;
}

//==========================================================================
// Cache
//

//! @brief Check if dcache is enabled or disabled
int hal_dcache_state_query()
{
    uint32_t sctlr; // System Control Register 
    
    // read sctlr 
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);
        
    if (sctlr & BM_SCTLR_C)
    {
        return 1;
    } else {
        return 0;
    }
}

void hal_dcache_enable()
{
    uint32_t sctlr; // System Control Register 
    
    // read sctlr 
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);
        
    if (!(sctlr & BM_SCTLR_C))
    {
        // set  C bit (data caching enable) 
        sctlr |= BM_SCTLR_C;
        
        // write modified sctlr
        _ARM_MCR(15, 0, sctlr, 1, 0, 0);
        
        // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
        _ARM_DSB();
    }
}

void hal_dcache_disable()
{
    uint32_t sctlr; // System Control Register 
    
    // read sctlr 
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);
    
    // set  C bit (data caching enable) 
    sctlr &= ~BM_SCTLR_C;

    // write modified sctlr
    _ARM_MCR(15, 0, sctlr, 1, 0, 0);
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void hal_dcache_invalidate()
{
    uint32_t csid;    // Cache Size ID
    uint32_t wayset;  // wayset parameter 
    int num_sets; // number of sets  
    int num_ways; // number of ways

    _ARM_MRC(15, 1, csid, 0, 0, 0);    // Read Cache Size ID 
    
    // Fill number of sets  and number of ways from csid register  This walues are decremented by 1
    num_ways = (csid >> 0x03) & 0x3FFu; //((csid& csid_ASSOCIATIVITY_MASK) >> csid_ASSOCIATIVITY_SHIFT)
    
    // Invalidation all lines (all Sets in all ways) 
    while (num_ways >= 0)
    {
        num_sets = (csid >> 0x0D) & 0x7FFFu; //((csid & csid_NUMSETS_MASK) >> csid_NUMSETS_SHIFT)
        while (num_sets >= 0 )
        {
            wayset = (num_sets << 5u) | (num_ways << 30u); //(num_sets << SETWAY_SET_SHIFT) | (num_sets << 3SETWAY_WAY_SHIFT)
            // invalidate line if we know set and way 
            _ARM_MCR(15, 0, wayset, 7, 6, 2);
            num_sets--;
        }
        num_ways--;
    }
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void hal_dcache_invalidate_line(const void * addr)
{
    uint32_t csidr = 0, line_size = 0;
    uint32_t va;
    
    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);    
    va = (uint32_t) addr & (~(line_size - 1)); //addr & va_VIRTUAL_ADDRESS_MASK

    // Invalidate data cache line by va to PoC (Point of Coherency). 
    _ARM_MCR(15, 0, va, 7, 6, 1);
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void hal_dcache_invalidate_mlines(const void * addr, size_t length)
{
    uint32_t va;
    uint32_t csidr = 0, line_size = 0;
    
    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);

    // align the address with line
    const void * end_addr = (const void *)((uint32_t)addr + length);
            
    do
    {
        // Clean data cache line to PoC (Point of Coherence) by va. 
        va = (uint32_t) ((uint32_t)addr & (~(line_size - 1))); //addr & va_VIRTUAL_ADDRESS_MASK
        _ARM_MCR(15, 0, va, 7, 6, 1);
        // increment addres to next line and decrement lenght 
        addr = (const void *) ((uint32_t)addr + line_size);
    } while (addr < end_addr);
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void hal_dcache_flush()
{
    uint32_t csid;    // Cache Size ID
    uint32_t wayset;  // wayset parameter 
    int num_sets; // number of sets  
    int num_ways; // number of ways

    _ARM_MRC(15, 1, csid, 0, 0, 0);    // Read Cache Size ID 
    
    // Fill number of sets  and number of ways from csid register  This walues are decremented by 1
    num_ways = (csid >> 0x03) & 0x3FFu; //((csid& csid_ASSOCIATIVITY_MASK) >> csid_ASSOCIATIVITY_SHIFT`)
    while (num_ways >= 0)
    {
        num_sets = (csid >> 0x0D) & 0x7FFFu; //((csid & csid_NUMSETS_MASK)      >> csid_NUMSETS_SHIFT       )
        while (num_sets >= 0 )
        {
            wayset = (num_sets << 5u) | (num_ways << 30u); //(num_sets << SETWAY_SET_SHIFT) | (num_ways << 3SETWAY_WAY_SHIFT)
            // FLUSH (clean) line if we know set and way 
            _ARM_MCR(15, 0, wayset, 7, 10, 2);
            num_sets--;
        }
        num_ways--;
    }
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void hal_dcache_flush_line(const void * addr)
{
    uint32_t csidr = 0, line_size = 0;
    uint32_t va;
    
    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);    
    va = (uint32_t) addr & (~(line_size - 1)); //addr & va_VIRTUAL_ADDRESS_MASK
    
    // Clean data cache line to PoC (Point of Coherence) by va. 
    _ARM_MCR(15, 0, va, 7, 10, 1);
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void hal_dcache_flush_mlines(const void * addr, size_t length)
{
    uint32_t va;
    uint32_t csidr = 0, line_size = 0;
    const void * end_addr = (const void *)((uint32_t)addr + length);

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);
    
    do
    {
        // Clean data cache line to PoC (Point of Coherence) by va. 
        va = (uint32_t) ((uint32_t)addr & (~(line_size  - 1))); //addr & va_VIRTUAL_ADDRESS_MASK
        _ARM_MCR(15, 0, va, 7, 10, 1);
        
        // increment addres to next line and decrement lenght 
        addr = (const void *) ((uint32_t)addr + line_size);
    } while (addr < end_addr);
    
    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

int hal_icache_state_query()
{
    uint32_t sctlr; // System Control Register 
    
    // read sctlr 
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);
        
    if (sctlr & BM_SCTLR_I)
    {
        return 1;
    } else {
        return 0;
    }
}

void hal_icache_enable()
{
    uint32_t sctlr  ;// System Control Register 
    
    // read sctlr 
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);
    
    // ignore the operation if I is enabled already
    if(!(sctlr & BM_SCTLR_I))
    {	
        // set  I bit (instruction caching enable)
        sctlr |= BM_SCTLR_I;
        
        // write modified sctlr
        _ARM_MCR(15, 0, sctlr, 1, 0, 0);
        
        // synchronize context on this processor 
        _ARM_ISB();
    }
}

void hal_icache_disable()
{
    uint32_t sctlr  ;// System Control Register 
    
    // read sctlr 
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);
    
    // Clear  I bit (instruction caching enable) 
    sctlr &= ~BM_SCTLR_I;

    // write modified sctlr
    _ARM_MCR(15, 0, sctlr, 1, 0, 0);
    
    // synchronize context on this processor 
    _ARM_ISB();
}

void hal_icache_invalidate()
{
    uint32_t SBZ = 0x0u;
    
    _ARM_MCR(15, 0, SBZ, 7, 5, 0);
    
    // synchronize context on this processor 
    _ARM_ISB();
}

void hal_icache_invalidate_is()
{
    uint32_t SBZ = 0x0u;
    
    _ARM_MCR(15, 0, SBZ, 7, 1, 0);
    
    // synchronize context on this processor 
    _ARM_ISB();
}

void arm_icache_invalidate_line(const void * addr)
{
    uint32_t csidr = 0, line_size = 0;
    uint32_t va;
    
    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);    
    va = (uint32_t) addr & (~(line_size - 1)); //addr & va_VIRTUAL_ADDRESS_MASK
    
    // Invalidate instruction cache by va to PoU (Point of unification). 
    _ARM_MCR(15, 0, va, 7, 5, 1);
    
    // synchronize context on this processor 
    _ARM_ISB();
}

void hal_icache_invalidate_mlines(const void * addr, size_t length)
{
    uint32_t va;
    uint32_t csidr = 0, line_size = 0;
    const void * end_addr = (const void *)((uint32_t)addr + length);

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);    
    
    do
    {
        // Clean data cache line to PoC (Point of Coherence) by va. 
        va = (uint32_t) ((uint32_t)addr & (~(line_size - 1))); //addr & va_VIRTUAL_ADDRESS_MASK
        _ARM_MCR(15, 0, va, 7, 5, 1);
        // increment addres to next line and decrement lenght 
        addr = (const void *) ((uint32_t)addr + line_size);
    } while (addr < end_addr);
    
    // synchronize context on this processor 
    _ARM_ISB();
}

//==========================================================================
// Cpu support
//

#define CORE_NUM_MASK (0x0300000) //!< Number of cores available bit mask.
#define CORE_NUM_SHIFT (20)       //!< Number of cores available bit shift.
#define FOUR_CORES (0)            //!< Four cores available register value.

typedef struct _core_startup_info {
    cpu_entry_point_t entry;    //!< Function to call after starting a core.
    void * arg;                 //!< Argument to pass core entry point.
} core_startup_info_t;

//! @brief Startup routine information for each CPU core.
#ifdef CYGPKG_HAL_SMP_SUPPORT
static core_startup_info_t s_core_info[CYGPKG_HAL_SMP_CPU_MAX] = {{0}};

cyg_uint32 hal_cpu_get_cores(void)
{
    int core_count = GET_CORES_ERROR;             

    // Mask and shift the contents of the control register so bit 21 and 20, which
    // are responsible for tracking cpu accessbility, are isolated.
    cyg_uint32 raw_data = (HW_OCOTP_CFG2_RD() & CORE_NUM_MASK) >> CORE_NUM_SHIFT;
    
    // Determine whether there are 2 or 4 cores active and set the return
    // value accordingly. If no core is active, return error.
    switch(raw_data)
    {
    case FOUR_CORES:
        core_count = FOUR_CORES_ACTIVE;
        break;
//    case TWO_CORES:
//        core_count = TWO_CORES_ACTIVE;
//        break;
//    case ONE_CORES:
//        core_count = ONE_CORE_ACTIVE;
//        break;
        default:
            core_count = GET_CORES_ERROR;
    }
    return core_count;
}

static void common_cpu_entry(void)
{
    uint32_t myCoreNumber = hal_cpu_get_current();
    core_startup_info_t * info = &s_core_info[myCoreNumber];
    
    // Call the requested entry point for this CPU number.
    if (info->entry)
    {
        info->entry(info->arg);
    }
}

extern void start(void);    // entry function, startup routine, defined in startup.s

void hal_cpu_start_secondary(cyg_uint8 coreNumber, cpu_entry_point_t entryPoint, void * arg)
{
    int actualCores = hal_cpu_get_cores();
    
    // Exit if the requested core is not available.
    if (coreNumber == 0 || coreNumber >= actualCores)
    {
        return;
    }
    
    // Save entry point and arg.
//    assert(coreNumber < CYGPKG_HAL_SMP_CPU_MAX);
    s_core_info[coreNumber].entry = entryPoint;
    s_core_info[coreNumber].arg = arg;
    
    // This makes sure the entry point is in physical memory.
    // There is a barrier instruction in the dcache flush that matters.
    hal_dcache_flush();
    hal_l2_cache_flush();
//    l2c310_cache_flush();

    // Prepare pointers for ROM code. The entry point is always _start, which does some
    // basic preparatory work and then calls the common_cpu_entry function, which itself
    // calls the entry point saved in s_core_info.
    switch (coreNumber)
    {
        case 1:
            HW_SRC_GPR3_WR((uint32_t) & start);
            HW_SRC_GPR4_WR((uint32_t) common_cpu_entry);

            if (HW_SRC_SCR.B.CORE1_ENABLE == 1)
            	HW_SRC_SCR.B.CORE1_RST = 1;
            else
                HW_SRC_SCR.B.CORE1_ENABLE = 1;
            break;
#if (CYGPKG_HAL_SMP_CPU_MAX == 4)
        case 2:
            HW_SRC_GPR5_WR((uint32_t) & start);
            HW_SRC_GPR6_WR((uint32_t) common_cpu_entry);

            if (HW_SRC_SCR.B.CORE2_ENABLE == 1)
             	HW_SRC_SCR.B.CORE2_RST = 1;
            else
                HW_SRC_SCR.B.CORE2_ENABLE = 1;
            break;
        case 3:
            HW_SRC_GPR7_WR((uint32_t) & start);
            HW_SRC_GPR8_WR((uint32_t) common_cpu_entry);

            if (HW_SRC_SCR.B.CORE3_ENABLE == 1)
             	HW_SRC_SCR.B.CORE3_RST = 1;
            else
                HW_SRC_SCR.B.CORE3_ENABLE = 1;
            break;
#endif
    }
}

void hal_cpu_disable(cyg_uint8 coreNumber)
{
    int actualCores = hal_cpu_get_cores();
    
    // Exit if the requested core is not available.
    if (coreNumber == 0 || coreNumber >= actualCores)
    {
        return;
    }
    
    switch (coreNumber)
    {
        case 1:
            HW_SRC_SCR.B.CORE1_ENABLE = 0;
            break;
#if (CYGPKG_HAL_SMP_CPU_MAX == 4)
        case 2:
            HW_SRC_SCR.B.CORE2_ENABLE = 0;
            break;
    
        case 3:
            HW_SRC_SCR.B.CORE3_ENABLE = 0;
            break;
#endif
    }
}

#endif

//--------------------------------------------------------------------------
// EOF imx6_misc.c
