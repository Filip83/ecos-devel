/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

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

int SMC_SetPowerModeWait()
{
    cyg_uint32 *SCB_SCR = (cyg_uint32*)SCB_SCR_BASE;
    /* configure Normal Wait mode */
    *SCB_SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    /*__DSB();
    __WFI();
    __ISB();*/

    return 0;
}


int SMC_SetPowerModeStop( int option)
{
    cyg_uint8 reg;
    cyg_uint32 *SCB_SCR = (cyg_uint32*)SCB_SCR_BASE;

    /* configure the Partial Stop mode in Noraml Stop mode */
    reg = SMC->STOPCTRL;
    reg &= ~SMC_STOPCTRL_PSTOPO_MASK;
    reg |= ((cyg_uint32)option << SMC_STOPCTRL_PSTOPO_SHIFT);
    SMC->STOPCTRL = reg;


    /* configure Normal Stop mode */
    reg = SMC->PMCTRL;
    reg &= ~SMC_PMCTRL_STOPM_MASK;
    reg |= (0 << SMC_PMCTRL_STOPM_SHIFT);
    SMC->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode (stop mode) */
    *SCB_SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before enter stop mode */
    (void)SMC->PMCTRL;
#if 0
    __DSB();
    __WFI();
    __ISB();

    /* check whether the power mode enter Stop mode succeed */
    if (base->PMCTRL & SMC_PMCTRL_STOPA_MASK)
    {
        return kStatus_SMC_StopAbort;
    }
    else
    {
        return kStatus_Success;
    }
#endif
    return 0;
}