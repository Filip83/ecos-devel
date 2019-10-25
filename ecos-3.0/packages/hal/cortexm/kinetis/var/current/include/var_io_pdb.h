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


/* ----------------------------------------------------------------------------
   -- PDB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

   /*!
	* @addtogroup PDB_Peripheral_Access_Layer PDB Peripheral Access Layer
	* @{
	*/

	/** PDB - Register Layout Typedef */
typedef struct cyghwr_hal_kinteis_pdb_s{
	cyg_uint32 SC;                                /**< Status and Control register, offset: 0x0 */
	cyg_uint32 MOD;                               /**< Modulus register, offset: 0x4 */
	cyg_uint32 CNT;                               /**< Counter register, offset: 0x8 */
	cyg_uint32 IDLY;                              /**< Interrupt Delay register, offset: 0xC */
	struct {                                         /* offset: 0x10, array step: 0x28 */
		cyg_uint32 C1;                                /**< Channel n Control register 1, array offset: 0x10, array step: 0x28 */
		cyg_uint32 S;                                 /**< Channel n Status register, array offset: 0x14, array step: 0x28 */
		cyg_uint32 DLY[2];                            /**< Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x28, index2*0x4 */
		uint8_t RESERVED_0[24];
	} CH[2];
	cyg_uint8 RESERVED_0[240];
	struct {                                         /* offset: 0x150, array step: 0x8 */
		cyg_uint32 INTC;                              /**< DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8 */
		cyg_uint32 INT;                               /**< DAC Interval n register, array offset: 0x154, array step: 0x8 */
	} DAC[2];
	uint8_t RESERVED_1[48];
	cyg_uint32 POEN;                              /**< Pulse-Out n Enable register, offset: 0x190 */
	cyg_uint32 PODLY[2];                          /**< Pulse-Out n Delay register, array offset: 0x194, array step: 0x4 */
} cyghwr_hal_kinteis_pdb_t;

/* ----------------------------------------------------------------------------
   -- PDB Register Masks
   ---------------------------------------------------------------------------- */

   /*!
	* @addtogroup PDB_Register_Masks PDB Register Masks
	* @{
	*/

	/*! @name SC - Status and Control register */
#define PDB_SC_LDOK_MASK                         (0x1U)
#define PDB_SC_LDOK_SHIFT                        (0U)
#define PDB_SC_LDOK(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_LDOK_SHIFT)) & PDB_SC_LDOK_MASK)
#define PDB_SC_CONT_MASK                         (0x2U)
#define PDB_SC_CONT_SHIFT                        (1U)
#define PDB_SC_CONT(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_CONT_SHIFT)) & PDB_SC_CONT_MASK)
#define PDB_SC_MULT_MASK                         (0xCU)
#define PDB_SC_MULT_SHIFT                        (2U)
#define PDB_SC_MULT(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_MULT_SHIFT)) & PDB_SC_MULT_MASK)
#define PDB_SC_PDBIE_MASK                        (0x20U)
#define PDB_SC_PDBIE_SHIFT                       (5U)
#define PDB_SC_PDBIE(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_PDBIE_SHIFT)) & PDB_SC_PDBIE_MASK)
#define PDB_SC_PDBIF_MASK                        (0x40U)
#define PDB_SC_PDBIF_SHIFT                       (6U)
#define PDB_SC_PDBIF(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_PDBIF_SHIFT)) & PDB_SC_PDBIF_MASK)
#define PDB_SC_PDBEN_MASK                        (0x80U)
#define PDB_SC_PDBEN_SHIFT                       (7U)
#define PDB_SC_PDBEN(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_PDBEN_SHIFT)) & PDB_SC_PDBEN_MASK)
#define PDB_SC_TRGSEL_MASK                       (0xF00U)
#define PDB_SC_TRGSEL_SHIFT                      (8U)
#define PDB_SC_TRGSEL(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_TRGSEL_SHIFT)) & PDB_SC_TRGSEL_MASK)
#define PDB_SC_PRESCALER_MASK                    (0x7000U)
#define PDB_SC_PRESCALER_SHIFT                   (12U)
#define PDB_SC_PRESCALER(x)                      (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_PRESCALER_SHIFT)) & PDB_SC_PRESCALER_MASK)
#define PDB_SC_DMAEN_MASK                        (0x8000U)
#define PDB_SC_DMAEN_SHIFT                       (15U)
#define PDB_SC_DMAEN(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_DMAEN_SHIFT)) & PDB_SC_DMAEN_MASK)
#define PDB_SC_SWTRIG_MASK                       (0x10000U)
#define PDB_SC_SWTRIG_SHIFT                      (16U)
#define PDB_SC_SWTRIG(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_SWTRIG_SHIFT)) & PDB_SC_SWTRIG_MASK)
#define PDB_SC_PDBEIE_MASK                       (0x20000U)
#define PDB_SC_PDBEIE_SHIFT                      (17U)
#define PDB_SC_PDBEIE(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_PDBEIE_SHIFT)) & PDB_SC_PDBEIE_MASK)
#define PDB_SC_LDMOD_MASK                        (0xC0000U)
#define PDB_SC_LDMOD_SHIFT                       (18U)
#define PDB_SC_LDMOD(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_SC_LDMOD_SHIFT)) & PDB_SC_LDMOD_MASK)

/*! @name MOD - Modulus register */
#define PDB_MOD_MOD_MASK                         (0xFFFFU)
#define PDB_MOD_MOD_SHIFT                        (0U)
#define PDB_MOD_MOD(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_MOD_MOD_SHIFT)) & PDB_MOD_MOD_MASK)

/*! @name CNT - Counter register */
#define PDB_CNT_CNT_MASK                         (0xFFFFU)
#define PDB_CNT_CNT_SHIFT                        (0U)
#define PDB_CNT_CNT(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_CNT_CNT_SHIFT)) & PDB_CNT_CNT_MASK)

/*! @name IDLY - Interrupt Delay register */
#define PDB_IDLY_IDLY_MASK                       (0xFFFFU)
#define PDB_IDLY_IDLY_SHIFT                      (0U)
#define PDB_IDLY_IDLY(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << PDB_IDLY_IDLY_SHIFT)) & PDB_IDLY_IDLY_MASK)

/*! @name C1 - Channel n Control register 1 */
#define PDB_C1_EN_MASK                           (0xFFU)
#define PDB_C1_EN_SHIFT                          (0U)
#define PDB_C1_EN(x)                             (((cyg_uint32)(((cyg_uint32)(x)) << PDB_C1_EN_SHIFT)) & PDB_C1_EN_MASK)
#define PDB_C1_TOS_MASK                          (0xFF00U)
#define PDB_C1_TOS_SHIFT                         (8U)
#define PDB_C1_TOS(x)                            (((cyg_uint32)(((cyg_uint32)(x)) << PDB_C1_TOS_SHIFT)) & PDB_C1_TOS_MASK)
#define PDB_C1_BB_MASK                           (0xFF0000U)
#define PDB_C1_BB_SHIFT                          (16U)
#define PDB_C1_BB(x)                             (((cyg_uint32)(((cyg_uint32)(x)) << PDB_C1_BB_SHIFT)) & PDB_C1_BB_MASK)

/* The count of PDB_C1 */
#define PDB_C1_COUNT                             (2U)

/*! @name S - Channel n Status register */
#define PDB_S_ERR_MASK                           (0xFFU)
#define PDB_S_ERR_SHIFT                          (0U)
#define PDB_S_ERR(x)                             (((cyg_uint32)(((cyg_uint32)(x)) << PDB_S_ERR_SHIFT)) & PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            (0xFF0000U)
#define PDB_S_CF_SHIFT                           (16U)
#define PDB_S_CF(x)                              (((cyg_uint32)(((cyg_uint32)(x)) << PDB_S_CF_SHIFT)) & PDB_S_CF_MASK)

/* The count of PDB_S */
#define PDB_S_COUNT                              (2U)

/*! @name DLY - Channel n Delay 0 register..Channel n Delay 1 register */
#define PDB_DLY_DLY_MASK                         (0xFFFFU)
#define PDB_DLY_DLY_SHIFT                        (0U)
#define PDB_DLY_DLY(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_DLY_DLY_SHIFT)) & PDB_DLY_DLY_MASK)

/* The count of PDB_DLY */
#define PDB_DLY_COUNT                            (2U)

/* The count of PDB_DLY */
#define PDB_DLY_COUNT2                           (2U)

/*! @name INTC - DAC Interval Trigger n Control register */
#define PDB_INTC_TOE_MASK                        (0x1U)
#define PDB_INTC_TOE_SHIFT                       (0U)
#define PDB_INTC_TOE(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_INTC_TOE_SHIFT)) & PDB_INTC_TOE_MASK)
#define PDB_INTC_EXT_MASK                        (0x2U)
#define PDB_INTC_EXT_SHIFT                       (1U)
#define PDB_INTC_EXT(x)                          (((cyg_uint32)(((cyg_uint32)(x)) << PDB_INTC_EXT_SHIFT)) & PDB_INTC_EXT_MASK)

/* The count of PDB_INTC */
#define PDB_INTC_COUNT                           (2U)

/*! @name INT - DAC Interval n register */
#define PDB_INT_INT_MASK                         (0xFFFFU)
#define PDB_INT_INT_SHIFT                        (0U)
#define PDB_INT_INT(x)                           (((cyg_uint32)(((cyg_uint32)(x)) << PDB_INT_INT_SHIFT)) & PDB_INT_INT_MASK)

/* The count of PDB_INT */
#define PDB_INT_COUNT                            (2U)

/*! @name POEN - Pulse-Out n Enable register */
#define PDB_POEN_POEN_MASK                       (0xFFU)
#define PDB_POEN_POEN_SHIFT                      (0U)
#define PDB_POEN_POEN(x)                         (((cyg_uint32)(((cyg_uint32)(x)) << PDB_POEN_POEN_SHIFT)) & PDB_POEN_POEN_MASK)

/*! @name PODLY - Pulse-Out n Delay register */
#define PDB_PODLY_DLY2_MASK                      (0xFFFFU)
#define PDB_PODLY_DLY2_SHIFT                     (0U)
#define PDB_PODLY_DLY2(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << PDB_PODLY_DLY2_SHIFT)) & PDB_PODLY_DLY2_MASK)
#define PDB_PODLY_DLY1_MASK                      (0xFFFF0000U)
#define PDB_PODLY_DLY1_SHIFT                     (16U)
#define PDB_PODLY_DLY1(x)                        (((cyg_uint32)(((cyg_uint32)(x)) << PDB_PODLY_DLY1_SHIFT)) & PDB_PODLY_DLY1_MASK)

/* The count of PDB_PODLY */
#define PDB_PODLY_COUNT                          (2U)


/*!
 * @}
 */ /* end of group PDB_Register_Masks */


 /* PDB - Peripheral instance base addresses */
 /** Peripheral PDB0 base address */
#define PDB0_BASE                                (0x40036000u)
/** Peripheral PDB0 base pointer */
#define PDB0                                     ((cyghwr_hal_kinteis_pdb_t *)PDB0_BASE)
/** Array initializer of PDB peripheral base addresses */
#define PDB_BASE_ADDRS                           { PDB0_BASE }
/** Array initializer of PDB peripheral base pointers */
#define PDB_BASE_PTRS                            { PDB0 }
/** Interrupt vectors for the PDB peripheral type */
//#define PDB_IRQS                                 { PDB0_IRQn }

/*!
 * @}
 */ /* end of group PDB_Peripheral_Access_Layer */













#endif // CYGONCE_HAL_VAR_IO_PDB_H
