#ifndef CYGONCE_HAL_VAR_IO_PDB_H
#define CYGONCE_HAL_VAR_IO_PDB_H
//==========================================================================
//
//      var_io_pdb.h
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
// Date:        2013-06-14
// Purpose:     Freescale PDB definitions.
// Description:
//
//
//####DESCRIPTIONEND####
//==========================================================================

#define CYGADDR_IO_PDB_FREESCALE_PDB0_BASE (0x40036000)

#define    CYGHWR_DEV_FREESCALE_PDB_SC        0       // PDB Status and Control
#define    CYGHWR_DEV_FREESCALE_PDB_MOD       4       // PDB Modulus
#define    CYGHWR_DEV_FREESCALE_PDB_CNT       8       // PDB Count
#define    CYGHWR_DEV_FREESCALE_PDB_IDLY      12      // PDB Interrupt Delay

#define    CYGHWR_DEV_FREESCALE_PDB_CH0C1     16      // PDB Channel 0 Control
#define    CYGHWR_DEV_FREESCALE_PDB_CH0S      20      // PDB Channel 0 Status
#define    CYGHWR_DEV_FREESCALE_PDB_CH0DLY0   24      // PDB Channel 0 Delay 0
#define    CYGHWR_DEV_FREESCALE_PDB_CH0DLY1   28      // PDB Channel 0 Delay 1

#define    CYGHWR_DEV_FREESCALE_PDB_CH1C1     32      // PDB Channel 1 Control
#define    CYGHWR_DEV_FREESCALE_PDB_CH1S      36      // PDB Channel 1 Status
#define    CYGHWR_DEV_FREESCALE_PDB_CH1DLY0   40      // PDB Channel 1 Delay 0
#define    CYGHWR_DEV_FREESCALE_PDB_CH1DLY1   44      // PDB Channel 1 Delay 1

#define    CYGHWR_DEV_FREESCALE_PDB_CH2C1     48      // PDB Channel 2 Control
#define    CYGHWR_DEV_FREESCALE_PDB_CH2S      52      // PDB Channel 2 Status
#define    CYGHWR_DEV_FREESCALE_PDB_CH2DLY0   56      // PDB Channel 2 Delay 0
#define    CYGHWR_DEV_FREESCALE_PDB_CH2DLY1   60      // PDB Channel 2 Delay 1

#define    CYGHWR_DEV_FREESCALE_PDB_CH3C1     64      // PDB Channel 3 Control
#define    CYGHWR_DEV_FREESCALE_PDB_CH3S      68      // PDB Channel 3 Status
#define    CYGHWR_DEV_FREESCALE_PDB_CH3DLY0   72      // PDB Channel 3 Delay 0
#define    CYGHWR_DEV_FREESCALE_PDB_CH3DLY1   76      // PDB Channel 3 Delay 1

#define    CYGHWR_DEV_FREESCALE_PDB_DACINTC0  80      // PDB Dac Interval Trigger 0 Control
#define    CYGHWR_DEV_FREESCALE_PDB_DACINT0   84      // PDB DAC Interval 0

#define    CYGHWR_DEV_FREESCALE_PDB_DACINTC1  88      // PDB Dac Interval Trigger 1 Control
#define    CYGHWR_DEV_FREESCALE_PDB_DACINT1   92      // PDB DAC Interval 1

#define    CYGHWR_DEV_FREESCALE_PDB_POEN      96      // PDB Pulse-Out 0 Enable
#define    CYGHWR_DEV_FREESCALE_PDB_PO0DLY    100     // Pulse-Out 0 Delay
#define    CYGHWR_DEV_FREESCALE_PDB_PO1DLY    104     // Pulse-Out 1 Delay
#define    CYGHWR_DEV_FREESCALE_PDB_PO2DLY    108     // Pulse-Out 2 Delay
#define    CYGHWR_DEV_FREESCALE_PDB_PO3DLY    112     // Pulse-Out 3 Delay


#define CYGHWR_DEV_FREESCALE_PDB_SC_LOOPS      (0x000C0000)
#define CYGHWR_DEV_FREESCALE_PDB_SC_PDBEIE     (0x00020000)
#define CYGHWR_DEV_FREESCALE_PDB_SC_SWTRIG     (0x00010000)
#define CYGHWR_DEV_FREESCALE_PDB_SC_DMAEN      (0x00008000)
#define CYGHWR_DEV_FREESCALE_PDB_SC_PRESCALER  (0x00007000)
#define CYGHWR_DEV_FREESCALE_PDB_SC_TRGSEL     (0x00000F00)
#define CYGHWR_DEV_FREESCALE_PDB_SC_PDBEN      (0x00000080)
#define CYGHWR_DEV_FREESCALE_PDB_SC_PDBIF      (0x00000040)
#define CYGHWR_DEV_FREESCALE_PDB_SC_PDBIE      (0x00000020)
#define CYGHWR_DEV_FREESCALE_PDB_SC_MULT       (0x0000000C)
#define CYGHWR_DEV_FREESCALE_PDB_SC_CONT       (0x00000002)
#define CYGHWR_DEV_FREESCALE_PDB_SC_LDOK       (0x00000001)

#define CYGHWR_DEV_FREESCALE_PDB_MOD_MOD       (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_PDB_CNT_CNT       (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_PDB_IDLY_IDL      (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_PDB_CHC1_BB       (0x00FF0000)
#define CYGHWR_DEV_FREESCALE_PDB_CHC1_TOS      (0x0000FF00)
#define CYGHWR_DEV_FREESCALE_PDB_CHC1_EN       (0x000000FF)

#define CYGHWR_DEV_FREESCALE_PDB_CHS_CV        (0x00FF0000)
#define CYGHWR_DEV_FREESCALE_PDB_CHS_ERR       (0x000000FF)

#define CYGHWR_DEV_FREESCALE_PDB_CHDLY0_DLY    (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_PDB_CHDLY1_DLY    (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_PDB_DACINTC_EXT   (0x00000002)
#define CYGHWR_DEV_FREESCALE_PDB_DACINTC_TOE   (0x00000001)

#define CYGHWR_DEV_FREESCALE_PDB_DACINT_INT    (0x0000FFFF)

#define CYGHWR_DEV_FREESCALE_PDB_POEN_POEN     (0x000000FF)

#define CYGHWR_DEV_FREESCALE_PDB_PODLY_DLY1    (0x0000FF00)
#define CYGHWR_DEV_FREESCALE_PDB_PODLY_DLY2    (0x000000FF)















#endif // CYGONCE_HAL_VAR_IO_PDB_H
