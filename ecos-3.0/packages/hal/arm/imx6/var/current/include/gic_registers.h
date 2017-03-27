/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <cyg/infra/cyg_type.h>         // base types

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Offsets to the GIC registers.
enum _gic_base_offsets
{
    kGICDBaseOffset = 0x1000,   //!< GIC distributor offset.
    kGICCBaseOffset = 0x100     //!< GIC CPU interface offset.
};

//! @brief GIC distributor registers.
//!
//! Uses the GICv2 register names, but does not include GICv2 registers.
//!
//! The IPRIORITYRn and ITARGETSRn registers are byte accessible, so their types are cyg_uint8
//! instead of cyg_uint32 to reflect this. These members are indexed directly with the interrupt
//! number.
struct _gicd_registers
{
    cyg_uint32 CTLR;              //!< Distributor Control Register.
    cyg_uint32 TYPER;             //!< Interrupt Controller Type Register.
    cyg_uint32 IIDR;              //!< Distributor Implementer Identification Register.
    cyg_uint32 _reserved0[29];
    cyg_uint32 IGROUPRn[8];       //!< Interrupt Group Registers.
    cyg_uint32 _reserved1[24];
    cyg_uint32 ISENABLERn[32];    //!< Interrupt Set-Enable Registers.
    cyg_uint32 ICENABLERn[32];    //!< Interrupt Clear-Enable Registers.
    cyg_uint32 ISPENDRn[32];      //!< Interrupt Set-Pending Registers.
    cyg_uint32 ICPENDRn[32];      //!< Interrupt Clear-Pending Registers.
    cyg_uint32 ICDABRn[32];       //!< Active Bit Registers.
    cyg_uint32 _reserved2[32];
    cyg_uint8 IPRIORITYRn[255 * sizeof(cyg_uint32)];  //!< Interrupt Priority Registers. (Byte accessible)
    cyg_uint32 _reserved3;
    cyg_uint8 ITARGETSRn[255 * sizeof(cyg_uint32)];   //!< Interrupt Processor Targets Registers. (Byte accessible)
    cyg_uint32 _reserved4;
    cyg_uint32 ICFGRn[64];        //!< Interrupt Configuration Registers.
    cyg_uint32 _reserved5[128];
    cyg_uint32 SGIR;              //!< Software Generated Interrupt Register
};

//! @brief Bitfields constants for the GICD_CTLR register.
enum _gicd_ctlr_fields
{
    kBM_GICD_CTLR_EnableGrp1 = (1 << 1),
    kBM_GICD_CTLR_EnableGrp0 = (1 << 0)
};

//! @brief Bitfields constants for the GICD_SGIR register.
enum _gicd_sgir_fields
{
    kBP_GICD_SGIR_TargetListFilter = 24,
    kBM_GICD_SGIR_TargetListFilter = (0x3 << kBP_GICD_SGIR_TargetListFilter),
    
    kBP_GICD_SGIR_CPUTargetList = 16,
    kBM_GICD_SGIR_CPUTargetList = (0xff << kBP_GICD_SGIR_CPUTargetList),
    
    kBP_GICD_SGIR_NSATT = 15,
    kBM_GICD_SGIR_NSATT = (1 << kBP_GICD_SGIR_NSATT),
    
    kBP_GICD_SGIR_SGIINTID = 0,
    kBM_GICD_SGIR_SGIINTID = 0xf
};

//! @brief GIC CPU interface registers.
//!
//! Uses the GICv2 register names. Does not include GICv2 registers.
struct _gicc_registers
{
    cyg_uint32 CTLR;  //!< CPU Interface Control Register.
    cyg_uint32 PMR;   //!< Interrupt Priority Mask Register.
    cyg_uint32 BPR;   //!< Binary Point Register.
    cyg_uint32 IAR;   //!< Interrupt Acknowledge Register.
    cyg_uint32 EOIR;  //!< End of Interrupt Register.
    cyg_uint32 RPR;   //!< Running Priority Register.
    cyg_uint32 HPPIR; //!< Highest Priority Pending Interrupt Register.
    cyg_uint32 ABPR;  //!< Aliased Binary Point Register. (only visible with a secure access)
    cyg_uint32 _reserved[56];
    cyg_uint32 IIDR;  //!< CPU Interface Identification Register.
};

//! @brief Bitfields constants for the GICC_CTLR register.
enum _gicc_ctlr_fields
{
    kBP_GICC_CTLR_EnableS = 0,
    kBM_GICC_CTLR_EnableS = (1 << 0),
    
    kBP_GICC_CTLR_EnableNS = 1,
    kBM_GICC_CTLR_EnableNS = (1 << 1),
    
    kBP_GICC_CTLR_AckCtl = 2,
    kBM_GICC_CTLR_AckCtl = (1 << 2),
    
    kBP_GICC_CTLR_FIQEn = 3,
    kBM_GICC_CTLR_FIQEn = (1 << 3),
    
    kBP_GICC_CTLR_SBPR = 4,
    kBM_GICC_CTLR_SBPR = (1 << 4)
};

//! @brier Type for the GIC distributor registers.
typedef volatile struct _gicd_registers gicd_t;

//! @brier Type for the GIC CPU interface registers.
typedef volatile struct _gicc_registers gicc_t;

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
