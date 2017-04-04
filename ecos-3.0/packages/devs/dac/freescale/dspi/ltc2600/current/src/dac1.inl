//==========================================================================
//
//      dac1.inl
//
//      Parameters for DAC device 1
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

static ltc2600_settings ltc2600_dac_channel_settings0[] = {
    { LTC2600_SELECT_0 },
    { LTC2600_SELECT_1 },
    { LTC2600_SELECT_2 },
    { LTC2600_SELECT_3 },
    { LTC2600_SELECT_4 },
    { LTC2600_SELECT_5 },
    { LTC2600_SELECT_6 },
    { LTC2600_SELECT_7 }
};

// DAC setup
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
static const ltc2600_dac_setup ltc2600_dac_setup0 = {
    .dac_base          = CYGADDR_IO_PDB_FREESCALE_PDB0_BASE,
    .tmr_int_vector    = CYGNUM_HAL_INTERRUPT_PDB0,
    .tmr_int_pri       = CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR_INT_PRI,
    .channel_settings  = ltc2600_dac_channel_settings0
};
#else
static const ltc2600_dac_setup ltc2600_dac_setup0 = {
    .dac_base          = CYGADDR_IO_FTM_FREESCALE_FTM_BASE(CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR),
    .tmr_int_vector    = CYGNUM_HAL_INTERRUPT_FTM_VECTOR(CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR), 
    .tmr_int_pri       = CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR_INT_PRI,
    .channel_settings  = ltc2600_dac_channel_settings0
};
#endif

// DAC device info
static ltc2600_dac_info ltc2600_dac_info0 = {
    .setup          = &ltc2600_dac_setup0,
    .spi_dev        = &ltc2600_spi_dev0
};

// DAC device instance
CYG_DAC_DEVICE(ltc2600_dac_device1,
               &ltc2600_dac_funs,
               &ltc2600_dac_info0,
               CYGNUM_DEVS_DAC_LTC2600_DAC1_DEFAULT_RATE);

// DAC channels
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL0
LTC2600_DAC_CHANNEL(1, 0)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL1
LTC2600_DAC_CHANNEL(1, 1)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL2
LTC2600_DAC_CHANNEL(1, 2)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL3
LTC2600_DAC_CHANNEL(1, 3)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL4
LTC2600_DAC_CHANNEL(1, 4)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL5
LTC2600_DAC_CHANNEL(1, 5)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL6
LTC2600_DAC_CHANNEL(1, 6)
#endif
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1_CHANNEL7
LTC2600_DAC_CHANNEL(1, 7)
#endif

//-----------------------------------------------------------------------------
// End of DAC1.inl
