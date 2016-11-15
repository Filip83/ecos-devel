/* 
 * File:   smc_ncs0_cfg.h
 * Author: filip
 *
 * Created on 13. listopad 2014, 16:28
 */

#ifndef SMC_NCS5_CFG_H
#define	SMC_NCS5_CFG_H

//! SMC Peripheral Memory Size in Bytes
#define EXT_SM_SIZE            0x200001

//! SMC Data Bus Width
#define SMC_DBW                16

//! Whether 8-bit SM chips are connected on the SMC
#define SMC_8_BIT_CHIPS        FALSE



// NCS setup time. Unit: ns.
#define NCS_WR_SETUP            0

// NCS pulse time. Unit: ns.
#define NCS_WR_PULSE            90

// NCS hold time. Unit: ns.
#define NCS_WR_HOLD             10

// NWE setup time. Unit: ns.
#define NWE_SETUP               20

// NWE pulse time. Unit: ns.
#define NWE_PULSE               60

// NWE hold time. Unit: ns.
#define NWE_HOLD                20

// Write cycle time. Unit: ns.
#define NWE_CYCLE               Max((NCS_WR_SETUP + NCS_WR_PULSE + NCS_WR_HOLD),(NWE_SETUP + NWE_PULSE + NWE_HOLD))

// NCS setup time. Unit: ns.
#define NCS_RD_SETUP            0

// NCS pulse time. Unit: ns.
#define NCS_RD_PULSE            240

// NCS hold time. Unit: ns.
#define NCS_RD_HOLD             30

// NRD setup time. Unit: ns.
#define NRD_SETUP               30

// NRD pulse time. Unit: ns.
#define NRD_PULSE               210

// NRD hold time. Unit: ns.
#define NRD_HOLD                30

// Read cycle time. Unit: ns.
#define NRD_CYCLE               Max((NCS_RD_SETUP + NCS_RD_PULSE + NCS_RD_HOLD),(NRD_SETUP + NRD_PULSE + NRD_HOLD))



// Data float time
#define TDF_CYCLES              0
#define TDF_OPTIM               DISABLED

// Page mode
#define PAGE_MODE               DISABLED
#define PAGE_SIZE               0

//! Whether read is controlled by NCS or by NRD
#define NCS_CONTROLLED_READ     FALSE

//! Whether write is controlled by NCS or by NWE
#define NCS_CONTROLLED_WRITE    FALSE

//! Whether to use the NWAIT pin
#define NWAIT_MODE              AVR32_SMC_EXNW_MODE_DISABLED

#endif	/* SMC_NCS5_CFG_H */

