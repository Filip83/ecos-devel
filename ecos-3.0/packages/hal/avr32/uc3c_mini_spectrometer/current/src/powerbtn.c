/*
 * powerbtn.c
 *
 * Created: 25.4.2013 16:21:45
 *  Author: Filip
 */ 
#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>
#include <cyg/kernel/kapi.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <st/window.h>

#include <cyg/hal/powerbtn.h>
#include <DrvMsgQueue.h>


static void nmi_exception_handler(cyg_addrword_t data, cyg_code_t exception, cyg_addrword_t info);

void PwrbtnInit(void)
{
	cyg_exception_handler_t *old;
	cyg_uint32				*old_data;
	gpio_enable_module_pin(AVR32_EIC_EXTINT_0_1_PIN , AVR32_EIC_EXTINT_0_1_FUNCTION);
	gpio_enable_pin_pull_up(AVR32_EIC_EXTINT_0_1_PIN);
	cyg_exception_set_handler(CYGNUM_HAL_VECTOR_NMI,&nmi_exception_handler,NULL,&old,old_data);
	
	//na hranu
	AVR32_EIC.mode &= ~AVR32_EIC_MODE_NMI_MASK;
	//spadovou
	AVR32_EIC.edge &= ~AVR32_EIC_EDGE_NMI_MASK;
	AVR32_EIC.en   = AVR32_EIC_EN_NMI_MASK;
	
	AVR32_EIC.icr = AVR32_EIC_ICR_NMI_MASK;
	AVR32_EIC.ier = AVR32_EIC_IER_NMI_MASK;
}

void nmi_exception_handler(cyg_addrword_t data, cyg_code_t exception, cyg_addrword_t info)
{
	AVR32_EIC.icr = AVR32_EIC_ICR_NMI_MASK;
	queue_put(WM_ON_POWER_DOWN_BUTTON);
}
