#ifndef CYGONCE_HAL_VAR_IO_FTM_H
#define CYGONCE_HAL_VAR_IO_FTM_H
//==========================================================================
//
//      var_io_ftm.h
//
//      Freescale PWD definitions.
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2011, 2013 Free Software Foundation, Inc.                        
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
// Author(s):   Mike Jones <mike@proclivis.com>
// Contributors:
// Date:        2013-06-22
// Purpose:     Freescale FTM definitions.
// Description:
//
//
//####DESCRIPTIONEND####
//==========================================================================

#define CYGADDR_IO_FTM_FREESCALE_FTM0_BASE (0x40038000)
#define CYGADDR_IO_FTM_FREESCALE_FTM1_BASE (0x40039000)
#define CYGADDR_IO_FTM_FREESCALE_FTM2_BASE (0x400B8000)
#define CYGADDR_IO_FTM_FREESCALE_FTM3_BASE (0x400B9000)

#define    CYGHWR_DEV_FREESCALE_FTM_SC       0       // FTM Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_CNT      4       // FTM Counter
#define    CYGHWR_DEV_FREESCALE_FTM_MOD      8       // FTM Modulo

#define    CYGHWR_DEV_FREESCALE_FTM_C0SC     12      // FTM Channel 0 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_COV      16      // FTM Channel 0 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C1SC     20      // FTM Channel 1 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C1V      24      // FTM Channel 1 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C2SC     28      // FTM Channel 2 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C2V      32      // FTM Channel 2 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C3SC     36      // FTM Channel 3 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C3V      40      // FTM Channel 3 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C4SC     44      // FTM Channel 4 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C4V      48      // FTM Channel 4 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C5SC     52      // FTM Channel 5 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C5V      56      // FTM Channel 5 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C6SC     60      // FTM Channel 6 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C6V      64      // FTM Channel 6 Value
#define    CYGHWR_DEV_FREESCALE_FTM_C7SC     68      // FTM Channel 7 Status and Control
#define    CYGHWR_DEV_FREESCALE_FTM_C7V      72      // FTM Channel 7 Value

#define    CYGHWR_DEV_FREESCALE_FTM_CNTIN    76      // FTM Counter Initial Value
#define    CYGHWR_DEV_FREESCALE_FTM_STATUS   80      // FTM Capture and Compare Status
#define    CYGHWR_DEV_FREESCALE_FTM_MODE     84      // FTM Features Mode Selection
#define    CYGHWR_DEV_FREESCALE_FTM_SYNC     88      // FTM Synchronization
#define    CYGHWR_DEV_FREESCALE_FTM_OUTINIT  92      // FTM Initial State for Channels Output
#define    CYGHWR_DEV_FREESCALE_FTM_OUTMASK  96      // FTM Output Mask
#define    CYGHWR_DEV_FREESCALE_FTM_COMBINE  100     // FTM Function for Linked Channels
#define    CYGHWR_DEV_FREESCALE_FTM_DEADTIME 104     // FTM Deadtime Insertion Control
#define    CYGHWR_DEV_FREESCALE_FTM_EXTTRIG  108     // FTM External Trigger
#define    CYGHWR_DEV_FREESCALE_FTM_POL      112     // FTM Channels Polarity
#define    CYGHWR_DEV_FREESCALE_FTM_FMS      116     // FTM Fault Mode Status
#define    CYGHWR_DEV_FREESCALE_FTM_FILTER   120     // FTM Input Capture Filter Control
#define    CYGHWR_DEV_FREESCALE_FTM_FLTCTRL  124     // FTM Fault Control
#define    CYGHWR_DEV_FREESCALE_FTM_QDCTRL   128     // FTM Quadrature Decoder Control and Status
#define    CYGHWR_DEV_FREESCALE_FTM_CONF     132     // FTM Configuration
#define    CYGHWR_DEV_FREESCALE_FTM_FLTPOL   136     // FTM Fault Input Polarity
#define    CYGHWR_DEV_FREESCALE_FTM_SYNCONF  140     // FTM Synchronization Configuration
#define    CYGHWR_DEV_FREESCALE_FTM_INVCTRL  144     // FTM Inverting Control
#define    CYGHWR_DEV_FREESCALE_FTM_SWOCTRL  148     // FTM Software Output Control
#define    CYGHWR_DEV_FREESCALE_FTM_PWMLOAD  152     // FTM PWM Load


#define CYGHWR_DEV_FREESCALE_FTM_SC_TOF        (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_SC_TOIE       (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_SC_CPWMS      (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_SC_CLKS       (0x00000018)
#define CYGHWR_DEV_FREESCALE_FTM_SC_PS         (0x00000007)

#define CYGHWR_DEV_FREESCALE_FTM_CNT_COUNT     (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_FTM_MOD_MOD       (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_FTM_CNSC_CHF      (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_CNSC_CHIE     (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_CNSC_MSB      (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_CNSC_MSA      (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_CNSC_ELSB     (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_CNSC_ELSA     (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_CNSC_DMA      (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_CV_VAL        (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_FTM_CNTIN_INIT    (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH7F   (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH6F   (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH5F   (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH4F   (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH3F   (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH2F   (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH1F   (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_STATUS_CH0F   (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_MODE_FAULTIE  (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_MODE_FAULTM   (0x00000060)
#define CYGHWR_DEV_FREESCALE_FTM_MODE_CAPTEST  (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_MODE_PWMSYNC  (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_MODE_WPDIS    (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_MODE_INIT     (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_MODE_FTMEN    (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_SYNC_SWSYNC   (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_TRIG2    (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_TRIG1    (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_TRIG0    (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_SYNCHOM  (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_REINIT   (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_CNTMAX   (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_SYNC_CNTMIN   (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH7OI  (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH6OI  (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH5OI  (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH4OI  (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH3OI  (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH2OI  (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH1OI  (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_OUTINIT_CH0OI  (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH7OM  (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH6OM  (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH5OM  (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH4OM  (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH3OM  (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH2OM  (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH1OM  (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_OUTMASK_CH0OM  (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_FAULTEN3  (0x40000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_SYNCEN3   (0x20000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DTEN3     (0x10000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAP3    (0x08000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAPEN3  (0x04000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMP3     (0x02000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMBINE3  (0x01000000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_FAULTEN2  (0x00400000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_SYNCEN2   (0x00200000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DTEN2     (0x00100000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAP2    (0x00080000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAPEN2  (0x00040000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMP2     (0x00020000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMBINE2  (0x00010000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_FAULTEN1  (0x00004000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_SYNCEN1   (0x00002000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DTEN1     (0x00001000)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAP1    (0x00000800)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAPEN1  (0x00000400)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMP1     (0x00000200)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMBINE1  (0x00000100)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_FAULTEN0  (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_SYNCEN0   (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DTEN0     (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAP0    (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_DECAPEN0  (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMP0     (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_COMBINE_COMBINE0  (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_DEADTIME_DTPS     (0x000000C0)
#define CYGHWR_DEV_FREESCALE_FTM_DEADTIME_DTVAL    (0x0000003F)

#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_TRIGF      (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_INITTRIGEN (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_CH1TRIG    (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_CH0TRIG    (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_CH5TRIG    (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_CH4TRIG    (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_CH3TRIG    (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_EXTRIG_CH2TRIG    (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_POL_POL7          (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL6          (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL5          (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL4          (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL3          (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL2          (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL1          (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_POL_POL0          (0x00000001)


#define CYGHWR_DEV_FREESCALE_FTM_FMS_FAULTF        (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_FMS_WPEN          (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_FMS_FAULTIN       (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_FMS_FAULTF3       (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_FMS_FAULTF2       (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_FMS_FAULTF1       (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_FMS_FAULTF0       (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_FILTER_CH3FVAL    (0x0000F000)
#define CYGHWR_DEV_FREESCALE_FTM_FILTER_CH2FVAL    (0x00000F00)
#define CYGHWR_DEV_FREESCALE_FTM_FILTER_CH1FVAL    (0x000000F0)
#define CYGHWR_DEV_FREESCALE_FTM_FILTER_CH0FVAL    (0x0000000F)

#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FFVAL     (0x00000F00)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FFLTR3EN  (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FFLTR2EN  (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FFLTR1EN  (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FFLTR0EN  (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FAULT3EN  (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FAULT2EN  (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FAULT1EN  (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_FLTCTRL_FAULT0EN  (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_PHAFLTREN  (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_PHBFLTREN  (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_PHAPOL     (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_PHBPOL     (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_QUADMODE   (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_QUADIR     (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_TOFDIR     (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_QDCTRL_QUADEN     (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_CONF_GTBEOUT      (0x00000400)
#define CYGHWR_DEV_FREESCALE_FTM_CONF_GTBEEN       (0x00000200)
#define CYGHWR_DEV_FREESCALE_FTM_CONF_BDMMODE      (0x000000C0)
#define CYGHWR_DEV_FREESCALE_FTM_CONF_NUMTOF       (0x0000001F)

#define CYGHWR_DEV_FREESCALE_FTM_FLTPOL_FLT3POL    (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_FLTPOL_FLT2POL    (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_FLTPOL_FLT1POL    (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_FLTPOL_FLT0POL    (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_HWSOC     (0x00100000)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_HWINVC    (0x00080000)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_HWOM      (0x00040000)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_HWWRBUF   (0x00020000)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_HWRSTCNT  (0x00010000)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWSOC     (0x00001000)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWINVC    (0x00000800)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWOM      (0x00000400)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWWRBUF   (0x00000200)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWRSTCNT  (0x00000100)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SYNCMODE  (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWOC      (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_INVC      (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_CNTINC    (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_SYNCONF_HWTRIGMODE (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_INVCTRL_INV3EN    (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_INVCTRL_INV2EN    (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_INVCTRL_INV1EN    (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_INVCTRL_INV0EN    (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH7OCV    (0x00008000)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH6OCV    (0x00004000)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH5OCV    (0x00002000)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH4OCV    (0x00001000)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH3OCV    (0x00000800)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH2OCV    (0x00000400)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH1OCV    (0x00000200)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH0OCV    (0x00000100)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH7OC     (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH6OC     (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH5OC     (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH4OC     (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH3OC     (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH2OC     (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH1OC     (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_SWOCTRL_CH0OC     (0x00000001)

#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_LDOK      (0x00000200)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH7SEL    (0x00000080)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH6SEL    (0x00000040)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH5SEL    (0x00000020)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH4SEL    (0x00000010)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH3SEL    (0x00000008)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH2SEL    (0x00000004)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH1SEL    (0x00000002)
#define CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_CH0SEL    (0x00000001)

/* ----------------------------------------------------------------------------
   -- FTM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral_Access_Layer FTM Peripheral Access Layer
 * @{
 */

/** FTM - Register Layout Typedef */
typedef volatile struct cyghwr_hal_kinteis_ftm_s{
  cyg_uint32 SC;                                /**< Status And Control, offset: 0x0 */
  cyg_uint32 CNT;                               /**< Counter, offset: 0x4 */
  cyg_uint32 MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    cyg_uint32 CnSC;                              /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    cyg_uint32 CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[8];
  cyg_uint32 CNTIN;                             /**< Counter Initial Value, offset: 0x4C */
  cyg_uint32 STATUS;                            /**< Capture And Compare Status, offset: 0x50 */
  cyg_uint32 MODE;                              /**< Features Mode Selection, offset: 0x54 */
  cyg_uint32 SYNC;                              /**< Synchronization, offset: 0x58 */
  cyg_uint32 OUTINIT;                           /**< Initial State For Channels Output, offset: 0x5C */
  cyg_uint32 OUTMASK;                           /**< Output Mask, offset: 0x60 */
  cyg_uint32 COMBINE;                           /**< Function For Linked Channels, offset: 0x64 */
  cyg_uint32 DEADTIME;                          /**< Deadtime Insertion Control, offset: 0x68 */
  cyg_uint32 EXTTRIG;                           /**< FTM External Trigger, offset: 0x6C */
  cyg_uint32 POL;                               /**< Channels Polarity, offset: 0x70 */
  cyg_uint32 FMS;                               /**< Fault Mode Status, offset: 0x74 */
  cyg_uint32 FILTER;                            /**< Input Capture Filter Control, offset: 0x78 */
  cyg_uint32 FLTCTRL;                           /**< Fault Control, offset: 0x7C */
  cyg_uint32 QDCTRL;                            /**< Quadrature Decoder Control And Status, offset: 0x80 */
  cyg_uint32 CONF;                              /**< Configuration, offset: 0x84 */
  cyg_uint32 FLTPOL;                            /**< FTM Fault Input Polarity, offset: 0x88 */
  cyg_uint32 SYNCONF;                           /**< Synchronization Configuration, offset: 0x8C */
  cyg_uint32 INVCTRL;                           /**< FTM Inverting Control, offset: 0x90 */
  cyg_uint32 SWOCTRL;                           /**< FTM Software Output Control, offset: 0x94 */
  cyg_uint32 PWMLOAD;                           /**< FTM PWM Load, offset: 0x98 */
} cyghwr_hal_kinteis_ftm_t;

/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/*! @name SC - Status And Control */
#define FTM_SC_PS_MASK                           (0x7U)
#define FTM_SC_PS_SHIFT                          (0U)
#define FTM_SC_PS(x)                             (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SC_PS_SHIFT)) & FTM_SC_PS_MASK)
#define FTM_SC_CLKS_MASK                         (0x18U)
#define FTM_SC_CLKS_SHIFT                        (3U)
#define FTM_SC_CLKS(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SC_CLKS_SHIFT)) & FTM_SC_CLKS_MASK)
#define FTM_SC_CPWMS_MASK                        (0x20U)
#define FTM_SC_CPWMS_SHIFT                       (5U)
#define FTM_SC_CPWMS(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SC_CPWMS_SHIFT)) & FTM_SC_CPWMS_MASK)
#define FTM_SC_TOIE_MASK                         (0x40U)
#define FTM_SC_TOIE_SHIFT                        (6U)
#define FTM_SC_TOIE(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SC_TOIE_SHIFT)) & FTM_SC_TOIE_MASK)
#define FTM_SC_TOF_MASK                          (0x80U)
#define FTM_SC_TOF_SHIFT                         (7U)
#define FTM_SC_TOF(x)                            (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SC_TOF_SHIFT)) & FTM_SC_TOF_MASK)

/*! @name CNT - Counter */
#define FTM_CNT_COUNT_MASK                       (0xFFFFU)
#define FTM_CNT_COUNT_SHIFT                      (0U)
#define FTM_CNT_COUNT(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CNT_COUNT_SHIFT)) & FTM_CNT_COUNT_MASK)

/*! @name MOD - Modulo */
#define FTM_MOD_MOD_MASK                         (0xFFFFU)
#define FTM_MOD_MOD_SHIFT                        (0U)
#define FTM_MOD_MOD(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MOD_MOD_SHIFT)) & FTM_MOD_MOD_MASK)

/*! @name CnSC - Channel (n) Status And Control */
#define FTM_CnSC_DMA_MASK                        (0x1U)
#define FTM_CnSC_DMA_SHIFT                       (0U)
#define FTM_CnSC_DMA(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_DMA_SHIFT)) & FTM_CnSC_DMA_MASK)
#define FTM_CnSC_ICRST_MASK                      (0x2U)
#define FTM_CnSC_ICRST_SHIFT                     (1U)
#define FTM_CnSC_ICRST(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_ICRST_SHIFT)) & FTM_CnSC_ICRST_MASK)
#define FTM_CnSC_ELSA_MASK                       (0x4U)
#define FTM_CnSC_ELSA_SHIFT                      (2U)
#define FTM_CnSC_ELSA(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_ELSA_SHIFT)) & FTM_CnSC_ELSA_MASK)
#define FTM_CnSC_ELSB_MASK                       (0x8U)
#define FTM_CnSC_ELSB_SHIFT                      (3U)
#define FTM_CnSC_ELSB(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_ELSB_SHIFT)) & FTM_CnSC_ELSB_MASK)
#define FTM_CnSC_MSA_MASK                        (0x10U)
#define FTM_CnSC_MSA_SHIFT                       (4U)
#define FTM_CnSC_MSA(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_MSA_SHIFT)) & FTM_CnSC_MSA_MASK)
#define FTM_CnSC_MSB_MASK                        (0x20U)
#define FTM_CnSC_MSB_SHIFT                       (5U)
#define FTM_CnSC_MSB(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_MSB_SHIFT)) & FTM_CnSC_MSB_MASK)
#define FTM_CnSC_CHIE_MASK                       (0x40U)
#define FTM_CnSC_CHIE_SHIFT                      (6U)
#define FTM_CnSC_CHIE(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_CHIE_SHIFT)) & FTM_CnSC_CHIE_MASK)
#define FTM_CnSC_CHF_MASK                        (0x80U)
#define FTM_CnSC_CHF_SHIFT                       (7U)
#define FTM_CnSC_CHF(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnSC_CHF_SHIFT)) & FTM_CnSC_CHF_MASK)

/* The count of FTM_CnSC */
#define FTM_CnSC_COUNT                           (8U)

/*! @name CnV - Channel (n) Value */
#define FTM_CnV_VAL_MASK                         (0xFFFFU)
#define FTM_CnV_VAL_SHIFT                        (0U)
#define FTM_CnV_VAL(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CnV_VAL_SHIFT)) & FTM_CnV_VAL_MASK)

/* The count of FTM_CnV */
#define FTM_CnV_COUNT                            (8U)

/*! @name CNTIN - Counter Initial Value */
#define FTM_CNTIN_INIT_MASK                      (0xFFFFU)
#define FTM_CNTIN_INIT_SHIFT                     (0U)
#define FTM_CNTIN_INIT(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CNTIN_INIT_SHIFT)) & FTM_CNTIN_INIT_MASK)

/*! @name STATUS - Capture And Compare Status */
#define FTM_STATUS_CH0F_MASK                     (0x1U)
#define FTM_STATUS_CH0F_SHIFT                    (0U)
#define FTM_STATUS_CH0F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH0F_SHIFT)) & FTM_STATUS_CH0F_MASK)
#define FTM_STATUS_CH1F_MASK                     (0x2U)
#define FTM_STATUS_CH1F_SHIFT                    (1U)
#define FTM_STATUS_CH1F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH1F_SHIFT)) & FTM_STATUS_CH1F_MASK)
#define FTM_STATUS_CH2F_MASK                     (0x4U)
#define FTM_STATUS_CH2F_SHIFT                    (2U)
#define FTM_STATUS_CH2F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH2F_SHIFT)) & FTM_STATUS_CH2F_MASK)
#define FTM_STATUS_CH3F_MASK                     (0x8U)
#define FTM_STATUS_CH3F_SHIFT                    (3U)
#define FTM_STATUS_CH3F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH3F_SHIFT)) & FTM_STATUS_CH3F_MASK)
#define FTM_STATUS_CH4F_MASK                     (0x10U)
#define FTM_STATUS_CH4F_SHIFT                    (4U)
#define FTM_STATUS_CH4F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH4F_SHIFT)) & FTM_STATUS_CH4F_MASK)
#define FTM_STATUS_CH5F_MASK                     (0x20U)
#define FTM_STATUS_CH5F_SHIFT                    (5U)
#define FTM_STATUS_CH5F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH5F_SHIFT)) & FTM_STATUS_CH5F_MASK)
#define FTM_STATUS_CH6F_MASK                     (0x40U)
#define FTM_STATUS_CH6F_SHIFT                    (6U)
#define FTM_STATUS_CH6F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH6F_SHIFT)) & FTM_STATUS_CH6F_MASK)
#define FTM_STATUS_CH7F_MASK                     (0x80U)
#define FTM_STATUS_CH7F_SHIFT                    (7U)
#define FTM_STATUS_CH7F(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_STATUS_CH7F_SHIFT)) & FTM_STATUS_CH7F_MASK)

/*! @name MODE - Features Mode Selection */
#define FTM_MODE_FTMEN_MASK                      (0x1U)
#define FTM_MODE_FTMEN_SHIFT                     (0U)
#define FTM_MODE_FTMEN(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_FTMEN_SHIFT)) & FTM_MODE_FTMEN_MASK)
#define FTM_MODE_INIT_MASK                       (0x2U)
#define FTM_MODE_INIT_SHIFT                      (1U)
#define FTM_MODE_INIT(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_INIT_SHIFT)) & FTM_MODE_INIT_MASK)
#define FTM_MODE_WPDIS_MASK                      (0x4U)
#define FTM_MODE_WPDIS_SHIFT                     (2U)
#define FTM_MODE_WPDIS(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_WPDIS_SHIFT)) & FTM_MODE_WPDIS_MASK)
#define FTM_MODE_PWMSYNC_MASK                    (0x8U)
#define FTM_MODE_PWMSYNC_SHIFT                   (3U)
#define FTM_MODE_PWMSYNC(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_PWMSYNC_SHIFT)) & FTM_MODE_PWMSYNC_MASK)
#define FTM_MODE_CAPTEST_MASK                    (0x10U)
#define FTM_MODE_CAPTEST_SHIFT                   (4U)
#define FTM_MODE_CAPTEST(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_CAPTEST_SHIFT)) & FTM_MODE_CAPTEST_MASK)
#define FTM_MODE_FAULTM_MASK                     (0x60U)
#define FTM_MODE_FAULTM_SHIFT                    (5U)
#define FTM_MODE_FAULTM(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_FAULTM_SHIFT)) & FTM_MODE_FAULTM_MASK)
#define FTM_MODE_FAULTIE_MASK                    (0x80U)
#define FTM_MODE_FAULTIE_SHIFT                   (7U)
#define FTM_MODE_FAULTIE(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_MODE_FAULTIE_SHIFT)) & FTM_MODE_FAULTIE_MASK)

/*! @name SYNC - Synchronization */
#define FTM_SYNC_CNTMIN_MASK                     (0x1U)
#define FTM_SYNC_CNTMIN_SHIFT                    (0U)
#define FTM_SYNC_CNTMIN(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_CNTMIN_SHIFT)) & FTM_SYNC_CNTMIN_MASK)
#define FTM_SYNC_CNTMAX_MASK                     (0x2U)
#define FTM_SYNC_CNTMAX_SHIFT                    (1U)
#define FTM_SYNC_CNTMAX(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_CNTMAX_SHIFT)) & FTM_SYNC_CNTMAX_MASK)
#define FTM_SYNC_REINIT_MASK                     (0x4U)
#define FTM_SYNC_REINIT_SHIFT                    (2U)
#define FTM_SYNC_REINIT(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_REINIT_SHIFT)) & FTM_SYNC_REINIT_MASK)
#define FTM_SYNC_SYNCHOM_MASK                    (0x8U)
#define FTM_SYNC_SYNCHOM_SHIFT                   (3U)
#define FTM_SYNC_SYNCHOM(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_SYNCHOM_SHIFT)) & FTM_SYNC_SYNCHOM_MASK)
#define FTM_SYNC_TRIG0_MASK                      (0x10U)
#define FTM_SYNC_TRIG0_SHIFT                     (4U)
#define FTM_SYNC_TRIG0(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_TRIG0_SHIFT)) & FTM_SYNC_TRIG0_MASK)
#define FTM_SYNC_TRIG1_MASK                      (0x20U)
#define FTM_SYNC_TRIG1_SHIFT                     (5U)
#define FTM_SYNC_TRIG1(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_TRIG1_SHIFT)) & FTM_SYNC_TRIG1_MASK)
#define FTM_SYNC_TRIG2_MASK                      (0x40U)
#define FTM_SYNC_TRIG2_SHIFT                     (6U)
#define FTM_SYNC_TRIG2(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_TRIG2_SHIFT)) & FTM_SYNC_TRIG2_MASK)
#define FTM_SYNC_SWSYNC_MASK                     (0x80U)
#define FTM_SYNC_SWSYNC_SHIFT                    (7U)
#define FTM_SYNC_SWSYNC(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNC_SWSYNC_SHIFT)) & FTM_SYNC_SWSYNC_MASK)

/*! @name OUTINIT - Initial State For Channels Output */
#define FTM_OUTINIT_CH0OI_MASK                   (0x1U)
#define FTM_OUTINIT_CH0OI_SHIFT                  (0U)
#define FTM_OUTINIT_CH0OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH0OI_SHIFT)) & FTM_OUTINIT_CH0OI_MASK)
#define FTM_OUTINIT_CH1OI_MASK                   (0x2U)
#define FTM_OUTINIT_CH1OI_SHIFT                  (1U)
#define FTM_OUTINIT_CH1OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH1OI_SHIFT)) & FTM_OUTINIT_CH1OI_MASK)
#define FTM_OUTINIT_CH2OI_MASK                   (0x4U)
#define FTM_OUTINIT_CH2OI_SHIFT                  (2U)
#define FTM_OUTINIT_CH2OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH2OI_SHIFT)) & FTM_OUTINIT_CH2OI_MASK)
#define FTM_OUTINIT_CH3OI_MASK                   (0x8U)
#define FTM_OUTINIT_CH3OI_SHIFT                  (3U)
#define FTM_OUTINIT_CH3OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH3OI_SHIFT)) & FTM_OUTINIT_CH3OI_MASK)
#define FTM_OUTINIT_CH4OI_MASK                   (0x10U)
#define FTM_OUTINIT_CH4OI_SHIFT                  (4U)
#define FTM_OUTINIT_CH4OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH4OI_SHIFT)) & FTM_OUTINIT_CH4OI_MASK)
#define FTM_OUTINIT_CH5OI_MASK                   (0x20U)
#define FTM_OUTINIT_CH5OI_SHIFT                  (5U)
#define FTM_OUTINIT_CH5OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH5OI_SHIFT)) & FTM_OUTINIT_CH5OI_MASK)
#define FTM_OUTINIT_CH6OI_MASK                   (0x40U)
#define FTM_OUTINIT_CH6OI_SHIFT                  (6U)
#define FTM_OUTINIT_CH6OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH6OI_SHIFT)) & FTM_OUTINIT_CH6OI_MASK)
#define FTM_OUTINIT_CH7OI_MASK                   (0x80U)
#define FTM_OUTINIT_CH7OI_SHIFT                  (7U)
#define FTM_OUTINIT_CH7OI(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTINIT_CH7OI_SHIFT)) & FTM_OUTINIT_CH7OI_MASK)

/*! @name OUTMASK - Output Mask */
#define FTM_OUTMASK_CH0OM_MASK                   (0x1U)
#define FTM_OUTMASK_CH0OM_SHIFT                  (0U)
#define FTM_OUTMASK_CH0OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH0OM_SHIFT)) & FTM_OUTMASK_CH0OM_MASK)
#define FTM_OUTMASK_CH1OM_MASK                   (0x2U)
#define FTM_OUTMASK_CH1OM_SHIFT                  (1U)
#define FTM_OUTMASK_CH1OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH1OM_SHIFT)) & FTM_OUTMASK_CH1OM_MASK)
#define FTM_OUTMASK_CH2OM_MASK                   (0x4U)
#define FTM_OUTMASK_CH2OM_SHIFT                  (2U)
#define FTM_OUTMASK_CH2OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH2OM_SHIFT)) & FTM_OUTMASK_CH2OM_MASK)
#define FTM_OUTMASK_CH3OM_MASK                   (0x8U)
#define FTM_OUTMASK_CH3OM_SHIFT                  (3U)
#define FTM_OUTMASK_CH3OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH3OM_SHIFT)) & FTM_OUTMASK_CH3OM_MASK)
#define FTM_OUTMASK_CH4OM_MASK                   (0x10U)
#define FTM_OUTMASK_CH4OM_SHIFT                  (4U)
#define FTM_OUTMASK_CH4OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH4OM_SHIFT)) & FTM_OUTMASK_CH4OM_MASK)
#define FTM_OUTMASK_CH5OM_MASK                   (0x20U)
#define FTM_OUTMASK_CH5OM_SHIFT                  (5U)
#define FTM_OUTMASK_CH5OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH5OM_SHIFT)) & FTM_OUTMASK_CH5OM_MASK)
#define FTM_OUTMASK_CH6OM_MASK                   (0x40U)
#define FTM_OUTMASK_CH6OM_SHIFT                  (6U)
#define FTM_OUTMASK_CH6OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH6OM_SHIFT)) & FTM_OUTMASK_CH6OM_MASK)
#define FTM_OUTMASK_CH7OM_MASK                   (0x80U)
#define FTM_OUTMASK_CH7OM_SHIFT                  (7U)
#define FTM_OUTMASK_CH7OM(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_OUTMASK_CH7OM_SHIFT)) & FTM_OUTMASK_CH7OM_MASK)

/*! @name COMBINE - Function For Linked Channels */
#define FTM_COMBINE_COMBINE0_MASK                (0x1U)
#define FTM_COMBINE_COMBINE0_SHIFT               (0U)
#define FTM_COMBINE_COMBINE0(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMBINE0_SHIFT)) & FTM_COMBINE_COMBINE0_MASK)
#define FTM_COMBINE_COMP0_MASK                   (0x2U)
#define FTM_COMBINE_COMP0_SHIFT                  (1U)
#define FTM_COMBINE_COMP0(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMP0_SHIFT)) & FTM_COMBINE_COMP0_MASK)
#define FTM_COMBINE_DECAPEN0_MASK                (0x4U)
#define FTM_COMBINE_DECAPEN0_SHIFT               (2U)
#define FTM_COMBINE_DECAPEN0(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAPEN0_SHIFT)) & FTM_COMBINE_DECAPEN0_MASK)
#define FTM_COMBINE_DECAP0_MASK                  (0x8U)
#define FTM_COMBINE_DECAP0_SHIFT                 (3U)
#define FTM_COMBINE_DECAP0(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAP0_SHIFT)) & FTM_COMBINE_DECAP0_MASK)
#define FTM_COMBINE_DTEN0_MASK                   (0x10U)
#define FTM_COMBINE_DTEN0_SHIFT                  (4U)
#define FTM_COMBINE_DTEN0(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DTEN0_SHIFT)) & FTM_COMBINE_DTEN0_MASK)
#define FTM_COMBINE_SYNCEN0_MASK                 (0x20U)
#define FTM_COMBINE_SYNCEN0_SHIFT                (5U)
#define FTM_COMBINE_SYNCEN0(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_SYNCEN0_SHIFT)) & FTM_COMBINE_SYNCEN0_MASK)
#define FTM_COMBINE_FAULTEN0_MASK                (0x40U)
#define FTM_COMBINE_FAULTEN0_SHIFT               (6U)
#define FTM_COMBINE_FAULTEN0(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_FAULTEN0_SHIFT)) & FTM_COMBINE_FAULTEN0_MASK)
#define FTM_COMBINE_COMBINE1_MASK                (0x100U)
#define FTM_COMBINE_COMBINE1_SHIFT               (8U)
#define FTM_COMBINE_COMBINE1(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMBINE1_SHIFT)) & FTM_COMBINE_COMBINE1_MASK)
#define FTM_COMBINE_COMP1_MASK                   (0x200U)
#define FTM_COMBINE_COMP1_SHIFT                  (9U)
#define FTM_COMBINE_COMP1(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMP1_SHIFT)) & FTM_COMBINE_COMP1_MASK)
#define FTM_COMBINE_DECAPEN1_MASK                (0x400U)
#define FTM_COMBINE_DECAPEN1_SHIFT               (10U)
#define FTM_COMBINE_DECAPEN1(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAPEN1_SHIFT)) & FTM_COMBINE_DECAPEN1_MASK)
#define FTM_COMBINE_DECAP1_MASK                  (0x800U)
#define FTM_COMBINE_DECAP1_SHIFT                 (11U)
#define FTM_COMBINE_DECAP1(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAP1_SHIFT)) & FTM_COMBINE_DECAP1_MASK)
#define FTM_COMBINE_DTEN1_MASK                   (0x1000U)
#define FTM_COMBINE_DTEN1_SHIFT                  (12U)
#define FTM_COMBINE_DTEN1(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DTEN1_SHIFT)) & FTM_COMBINE_DTEN1_MASK)
#define FTM_COMBINE_SYNCEN1_MASK                 (0x2000U)
#define FTM_COMBINE_SYNCEN1_SHIFT                (13U)
#define FTM_COMBINE_SYNCEN1(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_SYNCEN1_SHIFT)) & FTM_COMBINE_SYNCEN1_MASK)
#define FTM_COMBINE_FAULTEN1_MASK                (0x4000U)
#define FTM_COMBINE_FAULTEN1_SHIFT               (14U)
#define FTM_COMBINE_FAULTEN1(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_FAULTEN1_SHIFT)) & FTM_COMBINE_FAULTEN1_MASK)
#define FTM_COMBINE_COMBINE2_MASK                (0x10000U)
#define FTM_COMBINE_COMBINE2_SHIFT               (16U)
#define FTM_COMBINE_COMBINE2(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMBINE2_SHIFT)) & FTM_COMBINE_COMBINE2_MASK)
#define FTM_COMBINE_COMP2_MASK                   (0x20000U)
#define FTM_COMBINE_COMP2_SHIFT                  (17U)
#define FTM_COMBINE_COMP2(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMP2_SHIFT)) & FTM_COMBINE_COMP2_MASK)
#define FTM_COMBINE_DECAPEN2_MASK                (0x40000U)
#define FTM_COMBINE_DECAPEN2_SHIFT               (18U)
#define FTM_COMBINE_DECAPEN2(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAPEN2_SHIFT)) & FTM_COMBINE_DECAPEN2_MASK)
#define FTM_COMBINE_DECAP2_MASK                  (0x80000U)
#define FTM_COMBINE_DECAP2_SHIFT                 (19U)
#define FTM_COMBINE_DECAP2(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAP2_SHIFT)) & FTM_COMBINE_DECAP2_MASK)
#define FTM_COMBINE_DTEN2_MASK                   (0x100000U)
#define FTM_COMBINE_DTEN2_SHIFT                  (20U)
#define FTM_COMBINE_DTEN2(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DTEN2_SHIFT)) & FTM_COMBINE_DTEN2_MASK)
#define FTM_COMBINE_SYNCEN2_MASK                 (0x200000U)
#define FTM_COMBINE_SYNCEN2_SHIFT                (21U)
#define FTM_COMBINE_SYNCEN2(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_SYNCEN2_SHIFT)) & FTM_COMBINE_SYNCEN2_MASK)
#define FTM_COMBINE_FAULTEN2_MASK                (0x400000U)
#define FTM_COMBINE_FAULTEN2_SHIFT               (22U)
#define FTM_COMBINE_FAULTEN2(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_FAULTEN2_SHIFT)) & FTM_COMBINE_FAULTEN2_MASK)
#define FTM_COMBINE_COMBINE3_MASK                (0x1000000U)
#define FTM_COMBINE_COMBINE3_SHIFT               (24U)
#define FTM_COMBINE_COMBINE3(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMBINE3_SHIFT)) & FTM_COMBINE_COMBINE3_MASK)
#define FTM_COMBINE_COMP3_MASK                   (0x2000000U)
#define FTM_COMBINE_COMP3_SHIFT                  (25U)
#define FTM_COMBINE_COMP3(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_COMP3_SHIFT)) & FTM_COMBINE_COMP3_MASK)
#define FTM_COMBINE_DECAPEN3_MASK                (0x4000000U)
#define FTM_COMBINE_DECAPEN3_SHIFT               (26U)
#define FTM_COMBINE_DECAPEN3(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAPEN3_SHIFT)) & FTM_COMBINE_DECAPEN3_MASK)
#define FTM_COMBINE_DECAP3_MASK                  (0x8000000U)
#define FTM_COMBINE_DECAP3_SHIFT                 (27U)
#define FTM_COMBINE_DECAP3(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DECAP3_SHIFT)) & FTM_COMBINE_DECAP3_MASK)
#define FTM_COMBINE_DTEN3_MASK                   (0x10000000U)
#define FTM_COMBINE_DTEN3_SHIFT                  (28U)
#define FTM_COMBINE_DTEN3(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_DTEN3_SHIFT)) & FTM_COMBINE_DTEN3_MASK)
#define FTM_COMBINE_SYNCEN3_MASK                 (0x20000000U)
#define FTM_COMBINE_SYNCEN3_SHIFT                (29U)
#define FTM_COMBINE_SYNCEN3(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_SYNCEN3_SHIFT)) & FTM_COMBINE_SYNCEN3_MASK)
#define FTM_COMBINE_FAULTEN3_MASK                (0x40000000U)
#define FTM_COMBINE_FAULTEN3_SHIFT               (30U)
#define FTM_COMBINE_FAULTEN3(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_COMBINE_FAULTEN3_SHIFT)) & FTM_COMBINE_FAULTEN3_MASK)

/*! @name DEADTIME - Deadtime Insertion Control */
#define FTM_DEADTIME_DTVAL_MASK                  (0x3FU)
#define FTM_DEADTIME_DTVAL_SHIFT                 (0U)
#define FTM_DEADTIME_DTVAL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_DEADTIME_DTVAL_SHIFT)) & FTM_DEADTIME_DTVAL_MASK)
#define FTM_DEADTIME_DTPS_MASK                   (0xC0U)
#define FTM_DEADTIME_DTPS_SHIFT                  (6U)
#define FTM_DEADTIME_DTPS(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_DEADTIME_DTPS_SHIFT)) & FTM_DEADTIME_DTPS_MASK)

/*! @name EXTTRIG - FTM External Trigger */
#define FTM_EXTTRIG_CH2TRIG_MASK                 (0x1U)
#define FTM_EXTTRIG_CH2TRIG_SHIFT                (0U)
#define FTM_EXTTRIG_CH2TRIG(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_CH2TRIG_SHIFT)) & FTM_EXTTRIG_CH2TRIG_MASK)
#define FTM_EXTTRIG_CH3TRIG_MASK                 (0x2U)
#define FTM_EXTTRIG_CH3TRIG_SHIFT                (1U)
#define FTM_EXTTRIG_CH3TRIG(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_CH3TRIG_SHIFT)) & FTM_EXTTRIG_CH3TRIG_MASK)
#define FTM_EXTTRIG_CH4TRIG_MASK                 (0x4U)
#define FTM_EXTTRIG_CH4TRIG_SHIFT                (2U)
#define FTM_EXTTRIG_CH4TRIG(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_CH4TRIG_SHIFT)) & FTM_EXTTRIG_CH4TRIG_MASK)
#define FTM_EXTTRIG_CH5TRIG_MASK                 (0x8U)
#define FTM_EXTTRIG_CH5TRIG_SHIFT                (3U)
#define FTM_EXTTRIG_CH5TRIG(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_CH5TRIG_SHIFT)) & FTM_EXTTRIG_CH5TRIG_MASK)
#define FTM_EXTTRIG_CH0TRIG_MASK                 (0x10U)
#define FTM_EXTTRIG_CH0TRIG_SHIFT                (4U)
#define FTM_EXTTRIG_CH0TRIG(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_CH0TRIG_SHIFT)) & FTM_EXTTRIG_CH0TRIG_MASK)
#define FTM_EXTTRIG_CH1TRIG_MASK                 (0x20U)
#define FTM_EXTTRIG_CH1TRIG_SHIFT                (5U)
#define FTM_EXTTRIG_CH1TRIG(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_CH1TRIG_SHIFT)) & FTM_EXTTRIG_CH1TRIG_MASK)
#define FTM_EXTTRIG_INITTRIGEN_MASK              (0x40U)
#define FTM_EXTTRIG_INITTRIGEN_SHIFT             (6U)
#define FTM_EXTTRIG_INITTRIGEN(x)                (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_INITTRIGEN_SHIFT)) & FTM_EXTTRIG_INITTRIGEN_MASK)
#define FTM_EXTTRIG_TRIGF_MASK                   (0x80U)
#define FTM_EXTTRIG_TRIGF_SHIFT                  (7U)
#define FTM_EXTTRIG_TRIGF(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_EXTTRIG_TRIGF_SHIFT)) & FTM_EXTTRIG_TRIGF_MASK)

/*! @name POL - Channels Polarity */
#define FTM_POL_POL0_MASK                        (0x1U)
#define FTM_POL_POL0_SHIFT                       (0U)
#define FTM_POL_POL0(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL0_SHIFT)) & FTM_POL_POL0_MASK)
#define FTM_POL_POL1_MASK                        (0x2U)
#define FTM_POL_POL1_SHIFT                       (1U)
#define FTM_POL_POL1(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL1_SHIFT)) & FTM_POL_POL1_MASK)
#define FTM_POL_POL2_MASK                        (0x4U)
#define FTM_POL_POL2_SHIFT                       (2U)
#define FTM_POL_POL2(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL2_SHIFT)) & FTM_POL_POL2_MASK)
#define FTM_POL_POL3_MASK                        (0x8U)
#define FTM_POL_POL3_SHIFT                       (3U)
#define FTM_POL_POL3(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL3_SHIFT)) & FTM_POL_POL3_MASK)
#define FTM_POL_POL4_MASK                        (0x10U)
#define FTM_POL_POL4_SHIFT                       (4U)
#define FTM_POL_POL4(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL4_SHIFT)) & FTM_POL_POL4_MASK)
#define FTM_POL_POL5_MASK                        (0x20U)
#define FTM_POL_POL5_SHIFT                       (5U)
#define FTM_POL_POL5(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL5_SHIFT)) & FTM_POL_POL5_MASK)
#define FTM_POL_POL6_MASK                        (0x40U)
#define FTM_POL_POL6_SHIFT                       (6U)
#define FTM_POL_POL6(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL6_SHIFT)) & FTM_POL_POL6_MASK)
#define FTM_POL_POL7_MASK                        (0x80U)
#define FTM_POL_POL7_SHIFT                       (7U)
#define FTM_POL_POL7(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_POL_POL7_SHIFT)) & FTM_POL_POL7_MASK)

/*! @name FMS - Fault Mode Status */
#define FTM_FMS_FAULTF0_MASK                     (0x1U)
#define FTM_FMS_FAULTF0_SHIFT                    (0U)
#define FTM_FMS_FAULTF0(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_FAULTF0_SHIFT)) & FTM_FMS_FAULTF0_MASK)
#define FTM_FMS_FAULTF1_MASK                     (0x2U)
#define FTM_FMS_FAULTF1_SHIFT                    (1U)
#define FTM_FMS_FAULTF1(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_FAULTF1_SHIFT)) & FTM_FMS_FAULTF1_MASK)
#define FTM_FMS_FAULTF2_MASK                     (0x4U)
#define FTM_FMS_FAULTF2_SHIFT                    (2U)
#define FTM_FMS_FAULTF2(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_FAULTF2_SHIFT)) & FTM_FMS_FAULTF2_MASK)
#define FTM_FMS_FAULTF3_MASK                     (0x8U)
#define FTM_FMS_FAULTF3_SHIFT                    (3U)
#define FTM_FMS_FAULTF3(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_FAULTF3_SHIFT)) & FTM_FMS_FAULTF3_MASK)
#define FTM_FMS_FAULTIN_MASK                     (0x20U)
#define FTM_FMS_FAULTIN_SHIFT                    (5U)
#define FTM_FMS_FAULTIN(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_FAULTIN_SHIFT)) & FTM_FMS_FAULTIN_MASK)
#define FTM_FMS_WPEN_MASK                        (0x40U)
#define FTM_FMS_WPEN_SHIFT                       (6U)
#define FTM_FMS_WPEN(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_WPEN_SHIFT)) & FTM_FMS_WPEN_MASK)
#define FTM_FMS_FAULTF_MASK                      (0x80U)
#define FTM_FMS_FAULTF_SHIFT                     (7U)
#define FTM_FMS_FAULTF(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FMS_FAULTF_SHIFT)) & FTM_FMS_FAULTF_MASK)

/*! @name FILTER - Input Capture Filter Control */
#define FTM_FILTER_CH0FVAL_MASK                  (0xFU)
#define FTM_FILTER_CH0FVAL_SHIFT                 (0U)
#define FTM_FILTER_CH0FVAL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FILTER_CH0FVAL_SHIFT)) & FTM_FILTER_CH0FVAL_MASK)
#define FTM_FILTER_CH1FVAL_MASK                  (0xF0U)
#define FTM_FILTER_CH1FVAL_SHIFT                 (4U)
#define FTM_FILTER_CH1FVAL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FILTER_CH1FVAL_SHIFT)) & FTM_FILTER_CH1FVAL_MASK)
#define FTM_FILTER_CH2FVAL_MASK                  (0xF00U)
#define FTM_FILTER_CH2FVAL_SHIFT                 (8U)
#define FTM_FILTER_CH2FVAL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FILTER_CH2FVAL_SHIFT)) & FTM_FILTER_CH2FVAL_MASK)
#define FTM_FILTER_CH3FVAL_MASK                  (0xF000U)
#define FTM_FILTER_CH3FVAL_SHIFT                 (12U)
#define FTM_FILTER_CH3FVAL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FILTER_CH3FVAL_SHIFT)) & FTM_FILTER_CH3FVAL_MASK)

/*! @name FLTCTRL - Fault Control */
#define FTM_FLTCTRL_FAULT0EN_MASK                (0x1U)
#define FTM_FLTCTRL_FAULT0EN_SHIFT               (0U)
#define FTM_FLTCTRL_FAULT0EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FAULT0EN_SHIFT)) & FTM_FLTCTRL_FAULT0EN_MASK)
#define FTM_FLTCTRL_FAULT1EN_MASK                (0x2U)
#define FTM_FLTCTRL_FAULT1EN_SHIFT               (1U)
#define FTM_FLTCTRL_FAULT1EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FAULT1EN_SHIFT)) & FTM_FLTCTRL_FAULT1EN_MASK)
#define FTM_FLTCTRL_FAULT2EN_MASK                (0x4U)
#define FTM_FLTCTRL_FAULT2EN_SHIFT               (2U)
#define FTM_FLTCTRL_FAULT2EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FAULT2EN_SHIFT)) & FTM_FLTCTRL_FAULT2EN_MASK)
#define FTM_FLTCTRL_FAULT3EN_MASK                (0x8U)
#define FTM_FLTCTRL_FAULT3EN_SHIFT               (3U)
#define FTM_FLTCTRL_FAULT3EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FAULT3EN_SHIFT)) & FTM_FLTCTRL_FAULT3EN_MASK)
#define FTM_FLTCTRL_FFLTR0EN_MASK                (0x10U)
#define FTM_FLTCTRL_FFLTR0EN_SHIFT               (4U)
#define FTM_FLTCTRL_FFLTR0EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FFLTR0EN_SHIFT)) & FTM_FLTCTRL_FFLTR0EN_MASK)
#define FTM_FLTCTRL_FFLTR1EN_MASK                (0x20U)
#define FTM_FLTCTRL_FFLTR1EN_SHIFT               (5U)
#define FTM_FLTCTRL_FFLTR1EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FFLTR1EN_SHIFT)) & FTM_FLTCTRL_FFLTR1EN_MASK)
#define FTM_FLTCTRL_FFLTR2EN_MASK                (0x40U)
#define FTM_FLTCTRL_FFLTR2EN_SHIFT               (6U)
#define FTM_FLTCTRL_FFLTR2EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FFLTR2EN_SHIFT)) & FTM_FLTCTRL_FFLTR2EN_MASK)
#define FTM_FLTCTRL_FFLTR3EN_MASK                (0x80U)
#define FTM_FLTCTRL_FFLTR3EN_SHIFT               (7U)
#define FTM_FLTCTRL_FFLTR3EN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FFLTR3EN_SHIFT)) & FTM_FLTCTRL_FFLTR3EN_MASK)
#define FTM_FLTCTRL_FFVAL_MASK                   (0xF00U)
#define FTM_FLTCTRL_FFVAL_SHIFT                  (8U)
#define FTM_FLTCTRL_FFVAL(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTCTRL_FFVAL_SHIFT)) & FTM_FLTCTRL_FFVAL_MASK)

/*! @name QDCTRL - Quadrature Decoder Control And Status */
#define FTM_QDCTRL_QUADEN_MASK                   (0x1U)
#define FTM_QDCTRL_QUADEN_SHIFT                  (0U)
#define FTM_QDCTRL_QUADEN(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_QUADEN_SHIFT)) & FTM_QDCTRL_QUADEN_MASK)
#define FTM_QDCTRL_TOFDIR_MASK                   (0x2U)
#define FTM_QDCTRL_TOFDIR_SHIFT                  (1U)
#define FTM_QDCTRL_TOFDIR(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_TOFDIR_SHIFT)) & FTM_QDCTRL_TOFDIR_MASK)
#define FTM_QDCTRL_QUADIR_MASK                   (0x4U)
#define FTM_QDCTRL_QUADIR_SHIFT                  (2U)
#define FTM_QDCTRL_QUADIR(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_QUADIR_SHIFT)) & FTM_QDCTRL_QUADIR_MASK)
#define FTM_QDCTRL_QUADMODE_MASK                 (0x8U)
#define FTM_QDCTRL_QUADMODE_SHIFT                (3U)
#define FTM_QDCTRL_QUADMODE(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_QUADMODE_SHIFT)) & FTM_QDCTRL_QUADMODE_MASK)
#define FTM_QDCTRL_PHBPOL_MASK                   (0x10U)
#define FTM_QDCTRL_PHBPOL_SHIFT                  (4U)
#define FTM_QDCTRL_PHBPOL(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_PHBPOL_SHIFT)) & FTM_QDCTRL_PHBPOL_MASK)
#define FTM_QDCTRL_PHAPOL_MASK                   (0x20U)
#define FTM_QDCTRL_PHAPOL_SHIFT                  (5U)
#define FTM_QDCTRL_PHAPOL(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_PHAPOL_SHIFT)) & FTM_QDCTRL_PHAPOL_MASK)
#define FTM_QDCTRL_PHBFLTREN_MASK                (0x40U)
#define FTM_QDCTRL_PHBFLTREN_SHIFT               (6U)
#define FTM_QDCTRL_PHBFLTREN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_PHBFLTREN_SHIFT)) & FTM_QDCTRL_PHBFLTREN_MASK)
#define FTM_QDCTRL_PHAFLTREN_MASK                (0x80U)
#define FTM_QDCTRL_PHAFLTREN_SHIFT               (7U)
#define FTM_QDCTRL_PHAFLTREN(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_QDCTRL_PHAFLTREN_SHIFT)) & FTM_QDCTRL_PHAFLTREN_MASK)

/*! @name CONF - Configuration */
#define FTM_CONF_NUMTOF_MASK                     (0x1FU)
#define FTM_CONF_NUMTOF_SHIFT                    (0U)
#define FTM_CONF_NUMTOF(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CONF_NUMTOF_SHIFT)) & FTM_CONF_NUMTOF_MASK)
#define FTM_CONF_BDMMODE_MASK                    (0xC0U)
#define FTM_CONF_BDMMODE_SHIFT                   (6U)
#define FTM_CONF_BDMMODE(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CONF_BDMMODE_SHIFT)) & FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN_MASK                     (0x200U)
#define FTM_CONF_GTBEEN_SHIFT                    (9U)
#define FTM_CONF_GTBEEN(x)                       (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CONF_GTBEEN_SHIFT)) & FTM_CONF_GTBEEN_MASK)
#define FTM_CONF_GTBEOUT_MASK                    (0x400U)
#define FTM_CONF_GTBEOUT_SHIFT                   (10U)
#define FTM_CONF_GTBEOUT(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_CONF_GTBEOUT_SHIFT)) & FTM_CONF_GTBEOUT_MASK)

/*! @name FLTPOL - FTM Fault Input Polarity */
#define FTM_FLTPOL_FLT0POL_MASK                  (0x1U)
#define FTM_FLTPOL_FLT0POL_SHIFT                 (0U)
#define FTM_FLTPOL_FLT0POL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTPOL_FLT0POL_SHIFT)) & FTM_FLTPOL_FLT0POL_MASK)
#define FTM_FLTPOL_FLT1POL_MASK                  (0x2U)
#define FTM_FLTPOL_FLT1POL_SHIFT                 (1U)
#define FTM_FLTPOL_FLT1POL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTPOL_FLT1POL_SHIFT)) & FTM_FLTPOL_FLT1POL_MASK)
#define FTM_FLTPOL_FLT2POL_MASK                  (0x4U)
#define FTM_FLTPOL_FLT2POL_SHIFT                 (2U)
#define FTM_FLTPOL_FLT2POL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTPOL_FLT2POL_SHIFT)) & FTM_FLTPOL_FLT2POL_MASK)
#define FTM_FLTPOL_FLT3POL_MASK                  (0x8U)
#define FTM_FLTPOL_FLT3POL_SHIFT                 (3U)
#define FTM_FLTPOL_FLT3POL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_FLTPOL_FLT3POL_SHIFT)) & FTM_FLTPOL_FLT3POL_MASK)

/*! @name SYNCONF - Synchronization Configuration */
#define FTM_SYNCONF_HWTRIGMODE_MASK              (0x1U)
#define FTM_SYNCONF_HWTRIGMODE_SHIFT             (0U)
#define FTM_SYNCONF_HWTRIGMODE(x)                (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_HWTRIGMODE_SHIFT)) & FTM_SYNCONF_HWTRIGMODE_MASK)
#define FTM_SYNCONF_CNTINC_MASK                  (0x4U)
#define FTM_SYNCONF_CNTINC_SHIFT                 (2U)
#define FTM_SYNCONF_CNTINC(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_CNTINC_SHIFT)) & FTM_SYNCONF_CNTINC_MASK)
#define FTM_SYNCONF_INVC_MASK                    (0x10U)
#define FTM_SYNCONF_INVC_SHIFT                   (4U)
#define FTM_SYNCONF_INVC(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_INVC_SHIFT)) & FTM_SYNCONF_INVC_MASK)
#define FTM_SYNCONF_SWOC_MASK                    (0x20U)
#define FTM_SYNCONF_SWOC_SHIFT                   (5U)
#define FTM_SYNCONF_SWOC(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SWOC_SHIFT)) & FTM_SYNCONF_SWOC_MASK)
#define FTM_SYNCONF_SYNCMODE_MASK                (0x80U)
#define FTM_SYNCONF_SYNCMODE_SHIFT               (7U)
#define FTM_SYNCONF_SYNCMODE(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SYNCMODE_SHIFT)) & FTM_SYNCONF_SYNCMODE_MASK)
#define FTM_SYNCONF_SWRSTCNT_MASK                (0x100U)
#define FTM_SYNCONF_SWRSTCNT_SHIFT               (8U)
#define FTM_SYNCONF_SWRSTCNT(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SWRSTCNT_SHIFT)) & FTM_SYNCONF_SWRSTCNT_MASK)
#define FTM_SYNCONF_SWWRBUF_MASK                 (0x200U)
#define FTM_SYNCONF_SWWRBUF_SHIFT                (9U)
#define FTM_SYNCONF_SWWRBUF(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SWWRBUF_SHIFT)) & FTM_SYNCONF_SWWRBUF_MASK)
#define FTM_SYNCONF_SWOM_MASK                    (0x400U)
#define FTM_SYNCONF_SWOM_SHIFT                   (10U)
#define FTM_SYNCONF_SWOM(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SWOM_SHIFT)) & FTM_SYNCONF_SWOM_MASK)
#define FTM_SYNCONF_SWINVC_MASK                  (0x800U)
#define FTM_SYNCONF_SWINVC_SHIFT                 (11U)
#define FTM_SYNCONF_SWINVC(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SWINVC_SHIFT)) & FTM_SYNCONF_SWINVC_MASK)
#define FTM_SYNCONF_SWSOC_MASK                   (0x1000U)
#define FTM_SYNCONF_SWSOC_SHIFT                  (12U)
#define FTM_SYNCONF_SWSOC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_SWSOC_SHIFT)) & FTM_SYNCONF_SWSOC_MASK)
#define FTM_SYNCONF_HWRSTCNT_MASK                (0x10000U)
#define FTM_SYNCONF_HWRSTCNT_SHIFT               (16U)
#define FTM_SYNCONF_HWRSTCNT(x)                  (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_HWRSTCNT_SHIFT)) & FTM_SYNCONF_HWRSTCNT_MASK)
#define FTM_SYNCONF_HWWRBUF_MASK                 (0x20000U)
#define FTM_SYNCONF_HWWRBUF_SHIFT                (17U)
#define FTM_SYNCONF_HWWRBUF(x)                   (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_HWWRBUF_SHIFT)) & FTM_SYNCONF_HWWRBUF_MASK)
#define FTM_SYNCONF_HWOM_MASK                    (0x40000U)
#define FTM_SYNCONF_HWOM_SHIFT                   (18U)
#define FTM_SYNCONF_HWOM(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_HWOM_SHIFT)) & FTM_SYNCONF_HWOM_MASK)
#define FTM_SYNCONF_HWINVC_MASK                  (0x80000U)
#define FTM_SYNCONF_HWINVC_SHIFT                 (19U)
#define FTM_SYNCONF_HWINVC(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_HWINVC_SHIFT)) & FTM_SYNCONF_HWINVC_MASK)
#define FTM_SYNCONF_HWSOC_MASK                   (0x100000U)
#define FTM_SYNCONF_HWSOC_SHIFT                  (20U)
#define FTM_SYNCONF_HWSOC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SYNCONF_HWSOC_SHIFT)) & FTM_SYNCONF_HWSOC_MASK)

/*! @name INVCTRL - FTM Inverting Control */
#define FTM_INVCTRL_INV0EN_MASK                  (0x1U)
#define FTM_INVCTRL_INV0EN_SHIFT                 (0U)
#define FTM_INVCTRL_INV0EN(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_INVCTRL_INV0EN_SHIFT)) & FTM_INVCTRL_INV0EN_MASK)
#define FTM_INVCTRL_INV1EN_MASK                  (0x2U)
#define FTM_INVCTRL_INV1EN_SHIFT                 (1U)
#define FTM_INVCTRL_INV1EN(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_INVCTRL_INV1EN_SHIFT)) & FTM_INVCTRL_INV1EN_MASK)
#define FTM_INVCTRL_INV2EN_MASK                  (0x4U)
#define FTM_INVCTRL_INV2EN_SHIFT                 (2U)
#define FTM_INVCTRL_INV2EN(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_INVCTRL_INV2EN_SHIFT)) & FTM_INVCTRL_INV2EN_MASK)
#define FTM_INVCTRL_INV3EN_MASK                  (0x8U)
#define FTM_INVCTRL_INV3EN_SHIFT                 (3U)
#define FTM_INVCTRL_INV3EN(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_INVCTRL_INV3EN_SHIFT)) & FTM_INVCTRL_INV3EN_MASK)

/*! @name SWOCTRL - FTM Software Output Control */
#define FTM_SWOCTRL_CH0OC_MASK                   (0x1U)
#define FTM_SWOCTRL_CH0OC_SHIFT                  (0U)
#define FTM_SWOCTRL_CH0OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH0OC_SHIFT)) & FTM_SWOCTRL_CH0OC_MASK)
#define FTM_SWOCTRL_CH1OC_MASK                   (0x2U)
#define FTM_SWOCTRL_CH1OC_SHIFT                  (1U)
#define FTM_SWOCTRL_CH1OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH1OC_SHIFT)) & FTM_SWOCTRL_CH1OC_MASK)
#define FTM_SWOCTRL_CH2OC_MASK                   (0x4U)
#define FTM_SWOCTRL_CH2OC_SHIFT                  (2U)
#define FTM_SWOCTRL_CH2OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH2OC_SHIFT)) & FTM_SWOCTRL_CH2OC_MASK)
#define FTM_SWOCTRL_CH3OC_MASK                   (0x8U)
#define FTM_SWOCTRL_CH3OC_SHIFT                  (3U)
#define FTM_SWOCTRL_CH3OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH3OC_SHIFT)) & FTM_SWOCTRL_CH3OC_MASK)
#define FTM_SWOCTRL_CH4OC_MASK                   (0x10U)
#define FTM_SWOCTRL_CH4OC_SHIFT                  (4U)
#define FTM_SWOCTRL_CH4OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH4OC_SHIFT)) & FTM_SWOCTRL_CH4OC_MASK)
#define FTM_SWOCTRL_CH5OC_MASK                   (0x20U)
#define FTM_SWOCTRL_CH5OC_SHIFT                  (5U)
#define FTM_SWOCTRL_CH5OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH5OC_SHIFT)) & FTM_SWOCTRL_CH5OC_MASK)
#define FTM_SWOCTRL_CH6OC_MASK                   (0x40U)
#define FTM_SWOCTRL_CH6OC_SHIFT                  (6U)
#define FTM_SWOCTRL_CH6OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH6OC_SHIFT)) & FTM_SWOCTRL_CH6OC_MASK)
#define FTM_SWOCTRL_CH7OC_MASK                   (0x80U)
#define FTM_SWOCTRL_CH7OC_SHIFT                  (7U)
#define FTM_SWOCTRL_CH7OC(x)                     (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH7OC_SHIFT)) & FTM_SWOCTRL_CH7OC_MASK)
#define FTM_SWOCTRL_CH0OCV_MASK                  (0x100U)
#define FTM_SWOCTRL_CH0OCV_SHIFT                 (8U)
#define FTM_SWOCTRL_CH0OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH0OCV_SHIFT)) & FTM_SWOCTRL_CH0OCV_MASK)
#define FTM_SWOCTRL_CH1OCV_MASK                  (0x200U)
#define FTM_SWOCTRL_CH1OCV_SHIFT                 (9U)
#define FTM_SWOCTRL_CH1OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH1OCV_SHIFT)) & FTM_SWOCTRL_CH1OCV_MASK)
#define FTM_SWOCTRL_CH2OCV_MASK                  (0x400U)
#define FTM_SWOCTRL_CH2OCV_SHIFT                 (10U)
#define FTM_SWOCTRL_CH2OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH2OCV_SHIFT)) & FTM_SWOCTRL_CH2OCV_MASK)
#define FTM_SWOCTRL_CH3OCV_MASK                  (0x800U)
#define FTM_SWOCTRL_CH3OCV_SHIFT                 (11U)
#define FTM_SWOCTRL_CH3OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH3OCV_SHIFT)) & FTM_SWOCTRL_CH3OCV_MASK)
#define FTM_SWOCTRL_CH4OCV_MASK                  (0x1000U)
#define FTM_SWOCTRL_CH4OCV_SHIFT                 (12U)
#define FTM_SWOCTRL_CH4OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH4OCV_SHIFT)) & FTM_SWOCTRL_CH4OCV_MASK)
#define FTM_SWOCTRL_CH5OCV_MASK                  (0x2000U)
#define FTM_SWOCTRL_CH5OCV_SHIFT                 (13U)
#define FTM_SWOCTRL_CH5OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH5OCV_SHIFT)) & FTM_SWOCTRL_CH5OCV_MASK)
#define FTM_SWOCTRL_CH6OCV_MASK                  (0x4000U)
#define FTM_SWOCTRL_CH6OCV_SHIFT                 (14U)
#define FTM_SWOCTRL_CH6OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH6OCV_SHIFT)) & FTM_SWOCTRL_CH6OCV_MASK)
#define FTM_SWOCTRL_CH7OCV_MASK                  (0x8000U)
#define FTM_SWOCTRL_CH7OCV_SHIFT                 (15U)
#define FTM_SWOCTRL_CH7OCV(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_SWOCTRL_CH7OCV_SHIFT)) & FTM_SWOCTRL_CH7OCV_MASK)

/*! @name PWMLOAD - FTM PWM Load */
#define FTM_PWMLOAD_CH0SEL_MASK                  (0x1U)
#define FTM_PWMLOAD_CH0SEL_SHIFT                 (0U)
#define FTM_PWMLOAD_CH0SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH0SEL_SHIFT)) & FTM_PWMLOAD_CH0SEL_MASK)
#define FTM_PWMLOAD_CH1SEL_MASK                  (0x2U)
#define FTM_PWMLOAD_CH1SEL_SHIFT                 (1U)
#define FTM_PWMLOAD_CH1SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH1SEL_SHIFT)) & FTM_PWMLOAD_CH1SEL_MASK)
#define FTM_PWMLOAD_CH2SEL_MASK                  (0x4U)
#define FTM_PWMLOAD_CH2SEL_SHIFT                 (2U)
#define FTM_PWMLOAD_CH2SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH2SEL_SHIFT)) & FTM_PWMLOAD_CH2SEL_MASK)
#define FTM_PWMLOAD_CH3SEL_MASK                  (0x8U)
#define FTM_PWMLOAD_CH3SEL_SHIFT                 (3U)
#define FTM_PWMLOAD_CH3SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH3SEL_SHIFT)) & FTM_PWMLOAD_CH3SEL_MASK)
#define FTM_PWMLOAD_CH4SEL_MASK                  (0x10U)
#define FTM_PWMLOAD_CH4SEL_SHIFT                 (4U)
#define FTM_PWMLOAD_CH4SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH4SEL_SHIFT)) & FTM_PWMLOAD_CH4SEL_MASK)
#define FTM_PWMLOAD_CH5SEL_MASK                  (0x20U)
#define FTM_PWMLOAD_CH5SEL_SHIFT                 (5U)
#define FTM_PWMLOAD_CH5SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH5SEL_SHIFT)) & FTM_PWMLOAD_CH5SEL_MASK)
#define FTM_PWMLOAD_CH6SEL_MASK                  (0x40U)
#define FTM_PWMLOAD_CH6SEL_SHIFT                 (6U)
#define FTM_PWMLOAD_CH6SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH6SEL_SHIFT)) & FTM_PWMLOAD_CH6SEL_MASK)
#define FTM_PWMLOAD_CH7SEL_MASK                  (0x80U)
#define FTM_PWMLOAD_CH7SEL_SHIFT                 (7U)
#define FTM_PWMLOAD_CH7SEL(x)                    (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_CH7SEL_SHIFT)) & FTM_PWMLOAD_CH7SEL_MASK)
#define FTM_PWMLOAD_LDOK_MASK                    (0x200U)
#define FTM_PWMLOAD_LDOK_SHIFT                   (9U)
#define FTM_PWMLOAD_LDOK(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << FTM_PWMLOAD_LDOK_SHIFT)) & FTM_PWMLOAD_LDOK_MASK)


/*!
 * @}
 */ /* end of group FTM_Register_Masks */


/* FTM - Peripheral instance base addresses */
/** Peripheral FTM0 base address */
#define FTM0_BASE                                (0x40038000u)
/** Peripheral FTM0 base pointer */
#define FTM0                                     ((cyghwr_hal_kinteis_ftm_t *)FTM0_BASE)
/** Peripheral FTM1 base address */
#define FTM1_BASE                                (0x40039000u)
/** Peripheral FTM1 base pointer */
#define FTM1                                     ((cyghwr_hal_kinteis_ftm_t *)FTM1_BASE)
/** Peripheral FTM2 base address */
#define FTM2_BASE                                (0x4003A000u)
/** Peripheral FTM2 base pointer */
#define FTM2                                     ((cyghwr_hal_kinteis_ftm_t *)FTM2_BASE)
/** Peripheral FTM3 base address */
#define FTM3_BASE                                (0x400B9000u)
/** Peripheral FTM3 base pointer */
#define FTM3                                     ((cyghwr_hal_kinteis_ftm_t *)FTM3_BASE)
/** Array initializer of FTM peripheral base addresses */
#define FTM_BASE_ADDRS                           { FTM0_BASE, FTM1_BASE, FTM2_BASE, FTM3_BASE }
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0, FTM1, FTM2, FTM3 }
/** Interrupt vectors for the FTM peripheral type */
#define FTM_IRQS    

#endif // CYGONCE_HAL_VAR_IO_FTM_H
