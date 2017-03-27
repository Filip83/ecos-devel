//==========================================================================
//
//      adc0.inl
//
//      Parameters for ADC device 0
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2009 Free Software Foundation, Inc.                        
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
// Author(s):    Mike Jones <mike@proclivis.com>
// Contributors:
// Date:         2013-06-13
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

static ltc1859_settings ltc1859_adc_channel_settings0[] = {
    { LTC1859_MODE_SGL, LTC1859_ODD_NORM, LTC1859_SELECT_0, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_ALT, LTC1859_SELECT_0, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_NORM, LTC1859_SELECT_1, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_ALT, LTC1859_SELECT_1, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_NORM, LTC1859_SELECT_2, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_ALT, LTC1859_SELECT_2, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_NORM, LTC1859_SELECT_3, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON },
    { LTC1859_MODE_SGL, LTC1859_ODD_ALT, LTC1859_SELECT_3, LTC1859_POL_BI, LTC1859_GAIN_LOW, LTC1859_ACT_ON }
};

// ADC setup
#if (CYGNUM_DEVS_ADC_LTC1859_ADC0_TMR == 2)
static const ltc1859_adc_setup ltc1859_adc_setup0 = {
    .adc_base          = CYGADDR_IO_PDB_FREESCALE_PDB0_BASE,
    .tmr_int_vector    = CYGNUM_HAL_INTERRUPT_PDB0,
    .tmr_int_pri       = CYGNUM_DEVS_ADC_LTC1859_ADC0_TMR_INT_PRI,
    .channel_settings  = ltc1859_adc_channel_settings0
};
#else
static const ltc1859_adc_setup ltc1859_adc_setup0 = {
    .adc_base          = CYGADDR_IO_FTM_FREESCALE_FTM_BASE(CYGNUM_DEVS_ADC_LTC1859_ADC0_TMR),
    .tmr_int_vector    = CYGNUM_HAL_INTERRUPT_FTM_VECTOR(CYGNUM_DEVS_ADC_LTC1859_ADC0_TMR), 
    .tmr_int_pri       = CYGNUM_DEVS_ADC_LTC1859_ADC0_TMR_INT_PRI,
    .channel_settings  = ltc1859_adc_channel_settings0
};
#endif

// ADC device info
static ltc1859_adc_info ltc1859_adc_info0 = {
    .setup          = &ltc1859_adc_setup0,
    .spi_dev        = &ltc1859_spi_dev0
};

// ADC device instance
CYG_ADC_DEVICE(ltc1859_adc_device0,
               &ltc1859_adc_funs,
               &ltc1859_adc_info0,
               CYGNUM_DEVS_ADC_LTC1859_ADC0_DEFAULT_RATE);

// ADC channels
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL0
LTC1859_ADC_CHANNEL(0, 0)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL1
LTC1859_ADC_CHANNEL(0, 1)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL2
LTC1859_ADC_CHANNEL(0, 2)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL3
LTC1859_ADC_CHANNEL(0, 3)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL4
LTC1859_ADC_CHANNEL(0, 4)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL5
LTC1859_ADC_CHANNEL(0, 5)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL6
LTC1859_ADC_CHANNEL(0, 6)
#endif
#ifdef CYGHWR_DEVS_ADC_LTC1859_ADC0_CHANNEL7
LTC1859_ADC_CHANNEL(0, 7)
#endif

//-----------------------------------------------------------------------------
// End of adc0.inl
