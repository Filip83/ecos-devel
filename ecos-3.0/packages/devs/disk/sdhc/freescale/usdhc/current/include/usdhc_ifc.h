#include <cyg/infra/cyg_type.h>

/*
 * Copyright (c) 2011-2012, Freescale Semiconductor, Inc.
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

//!@addtogroup diag_usdhc
//!@{

/*!
 * @file usdhc_ifc.h
 * @brief uSDHC driver public interface. 
 */

#ifndef __USDHC_IFC_H__
#define __USDHC_IFC_H__

//#include "sdk.h"
#include <cyg/io/registers/regsusdhc.h>

//////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAIL
#define FAIL 1
#endif

//! @brief boot part
typedef enum {
    EMMC_PART_USER,
    EMMC_PART_BOOT1,
    EMMC_PART_BOOT2
} emmc_part_e;

//! @brief eMMC bus width
typedef enum {
    EMMC_BOOT_SDR1,
    EMMC_BOOT_SDR4,
    EMMC_BOOT_SDR8,
    EMMC_BOOT_DDR4,
    EMMC_BOOT_DDR8
} emmc_bus_width_e;

//////////////////////////////////////////////////////////////////////////////////
// API
//////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif
/*!
 * @brief Set Card access mode
 *
 * @param sdma Whether to enable ADMA when read/write from/to card.
 *      If enabled, then use ADMA for transfer, or else, use polling IO.
 * @param intr Whether use interrupt to indicate end of transfer
 *      If enabled, will attach the status to interrupt, or else, poll the status.
 */
extern void set_card_access_mode(cyg_uint32 sdma, cyg_uint32 intr);

/*!
 * @brief Returns whether ADMA mode is currently enabled.
 */
extern cyg_uint32 read_usdhc_adma_mode(void);

/*!
 * @brief Returns whether interrupt mode is currently enabled.
 */
extern cyg_uint32 read_usdhc_intr_mode(void); 

/*!
 * @brief Read the data transfer status(only in interrupt mode)
 * 
 * @param   instance       Instance number of the uSDHC module.
 * @param   status         Store the readback status. 0: busy, 1: success, 2: error
 *
 * @return 0 if successful; non-zero otherwise
 */
extern cyg_int32 usdhc_xfer_result(cyg_uint32 instance, int *status);

/*!
 * @brief Wait for the transfer complete. It covers the interrupt mode, DMA mode and PIO mode
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
extern cyg_int32 usdhc_wait_xfer_done(cyg_uint32 instance);

//! @name Board support functions
//!
//! These functions are called by the driver in order to factor out board
//! specific functionality. They must be defined by the board support
//! library or the application.
//@{
//! @brief Configure IOMUX for the USDHC driver.
void usdhc_iomux_config(cyg_int32 instance);
//@}

#if defined (__cplusplus)
}
#endif

//! @}
#endif  /*__USDHC_IFC_H__ */ 
/////////////////////////////////////////////////////////////////////////////////
// EOF
/////////////////////////////////////////////////////////////////////////////////

