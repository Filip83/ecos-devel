/* This source file is part of the ATMEL AVR-UC3-SoftwareFramework-1.7.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief SMC on EBI driver for AVR32 UC3.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a SMC module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */

#include <cyg/hal/utils/preprocessor/mrepeat.h>
#include <cyg/hal/gpio.h>
#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>
#include <pkgconf/system.h>
#include <cyg/hal/smc.h>
#include CYGBLD_HAL_PLATFORM_H

#ifndef TRUE
#define TRUE			1
#endif
#ifndef FALSE
#define FALSE			0
#endif

#ifndef ENABLED
#define ENABLED			1
#endif
#ifndef DISABLED
#define DISABLED		0
#endif

// Configure the SM Controller with SM setup and timing information for all chip select
#define SMC_CS_SETUP(ncs) { \
  cyg_uint32 nwe_setup    = ((NWE_SETUP    * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 ncs_wr_setup = ((NCS_WR_SETUP * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 nrd_setup    = ((NRD_SETUP    * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 ncs_rd_setup = ((NCS_RD_SETUP * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 nwe_pulse    = ((NWE_PULSE    * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 ncs_wr_pulse = ((NCS_WR_PULSE * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 nrd_pulse    = ((NRD_PULSE    * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 ncs_rd_pulse = ((NCS_RD_PULSE * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 nwe_cycle    = ((NWE_CYCLE    * hsb_mhz_up + 999) / 1000); \
  cyg_uint32 nrd_cycle    = ((NRD_CYCLE    * hsb_mhz_up + 999) / 1000); \
                                                                 \
  /* Some coherence checks...                             */     \
  /* Ensures CS is active during Rd or Wr                 */     \
  if( ncs_rd_setup + ncs_rd_pulse < nrd_setup + nrd_pulse )      \
    ncs_rd_pulse = nrd_setup + nrd_pulse - ncs_rd_setup;         \
  if( ncs_wr_setup + ncs_wr_pulse < nwe_setup + nwe_pulse )      \
    ncs_wr_pulse = nwe_setup + nwe_pulse - ncs_wr_setup;         \
                                                                 \
  /* ncs_hold = n_cycle - ncs_setup - ncs_pulse           */     \
  /* n_hold   = n_cycle - n_setup - n_pulse               */     \
  /*                                                      */     \
  /* All holds parameters must be positive or null, so:   */     \
  /* nwe_cycle shall be >= ncs_wr_setup + ncs_wr_pulse    */     \
  if( nwe_cycle < ncs_wr_setup + ncs_wr_pulse )                  \
    nwe_cycle = ncs_wr_setup + ncs_wr_pulse;                     \
                                                                 \
  /* nwe_cycle shall be >= nwe_setup + nwe_pulse          */     \
  if( nwe_cycle < nwe_setup + nwe_pulse )                        \
    nwe_cycle = nwe_setup + nwe_pulse;                           \
                                                                 \
  /* nrd_cycle shall be >= ncs_rd_setup + ncs_rd_pulse    */     \
  if( nrd_cycle < ncs_rd_setup + ncs_rd_pulse )                  \
    nrd_cycle = ncs_rd_setup + ncs_rd_pulse;                     \
                                                                 \
  /* nrd_cycle shall be >= nrd_setup + nrd_pulse          */     \
  if( nrd_cycle < nrd_setup + nrd_pulse )                        \
    nrd_cycle = nrd_setup + nrd_pulse;                           \
                                                                 \
  AVR32_SMC.cs[ncs].setup = (nwe_setup    << AVR32_SMC_SETUP0_NWE_SETUP_OFFSET) | \
                            (ncs_wr_setup << AVR32_SMC_SETUP0_NCS_WR_SETUP_OFFSET) | \
                            (nrd_setup    << AVR32_SMC_SETUP0_NRD_SETUP_OFFSET) | \
                            (ncs_rd_setup << AVR32_SMC_SETUP0_NCS_RD_SETUP_OFFSET); \
  AVR32_SMC.cs[ncs].pulse = (nwe_pulse    << AVR32_SMC_PULSE0_NWE_PULSE_OFFSET) | \
                            (ncs_wr_pulse << AVR32_SMC_PULSE0_NCS_WR_PULSE_OFFSET) | \
                            (nrd_pulse    << AVR32_SMC_PULSE0_NRD_PULSE_OFFSET) | \
                            (ncs_rd_pulse << AVR32_SMC_PULSE0_NCS_RD_PULSE_OFFSET); \
  AVR32_SMC.cs[ncs].cycle = (nwe_cycle    << AVR32_SMC_CYCLE0_NWE_CYCLE_OFFSET) | \
                            (nrd_cycle    << AVR32_SMC_CYCLE0_NRD_CYCLE_OFFSET); \
  AVR32_SMC.cs[ncs].mode = (((NCS_CONTROLLED_READ) ? AVR32_SMC_MODE0_READ_MODE_NCS_CONTROLLED : \
                           AVR32_SMC_MODE0_READ_MODE_NRD_CONTROLLED) << AVR32_SMC_MODE0_READ_MODE_OFFSET) | \
                       +    (((NCS_CONTROLLED_WRITE) ? AVR32_SMC_MODE0_WRITE_MODE_NCS_CONTROLLED : \
                           AVR32_SMC_MODE0_WRITE_MODE_NWE_CONTROLLED) << AVR32_SMC_MODE0_WRITE_MODE_OFFSET) | \
                           (NWAIT_MODE << AVR32_SMC_MODE0_EXNW_MODE_OFFSET) | \
                           (((SMC_8_BIT_CHIPS) ? AVR32_SMC_MODE0_BAT_BYTE_WRITE : \
                           AVR32_SMC_MODE0_BAT_BYTE_SELECT) << AVR32_SMC_MODE0_BAT_OFFSET) | \
                           (((SMC_DBW <= 8 ) ? AVR32_SMC_MODE0_DBW_8_BITS  : \
                           (SMC_DBW <= 16) ? AVR32_SMC_MODE0_DBW_16_BITS : \
                           AVR32_SMC_MODE0_DBW_32_BITS) << AVR32_SMC_MODE0_DBW_OFFSET) | \
                           (TDF_CYCLES << AVR32_SMC_MODE0_TDF_CYCLES_OFFSET) | \
                           (TDF_OPTIM << AVR32_SMC_MODE0_TDF_MODE_OFFSET) | \
                           (PAGE_MODE << AVR32_SMC_MODE0_PMEN_OFFSET) | \
                           (PAGE_SIZE << AVR32_SMC_MODE0_PS_OFFSET); \
  smc_tab_cs_size[ncs] = (cyg_uint8)EXT_SM_SIZE; \
  }

static cyg_uint8 smc_tab_cs_size[6];

static void smc_enable_muxed_pins(void);


void smc_init(unsigned long hsb_hz)
{
  unsigned long hsb_mhz_up = (hsb_hz + 999999) / 1000000;

//! Whether to use the NCS0 pin
#ifdef CYGNUM_HAL_SMC_NCS0_ENABLED
  #include CYGNUM_HAL_SMC_NCS0_CFG_H

  // Setup SMC for NCS0
  SMC_CS_SETUP(0)

  #ifdef SMC_DBW_GLOBAL
    #if (SMC_DBW_GLOBAL < SMC_DBW)
        #undef  SMC_DBW_GLOBAL
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
    #endif
  #else
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
  #endif

  #ifdef SMC_8_BIT_CHIPS_GLOBAL
    #if (SMC_8_BIT_CHIPS_GLOBAL < SMC_8_BIT)
        #undef  SMC_8_BIT_CHIPS_GLOBAL
        #if     (SMC_8_BIT_CHIPS == TRUE)
          #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
        #elif   (SMC_8_BIT_CHIPS == FALSE)
          #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
        #else
          #error  error in SMC_8_BIT_CHIPS size
        #endif
    #endif
  #else
      #if     (SMC_8_BIT_CHIPS == TRUE)
        #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
      #elif   (SMC_8_BIT_CHIPS == FALSE)
        #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
      #else
        #error  error in SMC_8_BIT_CHIPS size
      #endif
  #endif

  #ifdef NWAIT_MODE_GLOBAL
    #if (NWAIT_MODE_GLOBAL < NWAIT_MODE)
        #undef  NWAIT_MODE_GLOBAL
        #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
        #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
        #else
          #error  error in NWAIT_MODE size
        #endif
    #endif
  #else
      #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
      #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
      #else
        #error  error in NWAIT_MODE size
      #endif
  #endif

  #undef EXT_SM_SIZE
  #undef SMC_DBW
  #undef SMC_8_BIT_CHIPS
  #undef NWE_SETUP
  #undef NCS_WR_SETUP
  #undef NRD_SETUP
  #undef NCS_RD_SETUP
  #undef NCS_WR_PULSE
  #undef NWE_PULSE
  #undef NCS_RD_PULSE
  #undef NRD_PULSE
  #undef NCS_WR_HOLD
  #undef NWE_HOLD
  #undef NWE_CYCLE
  #undef NCS_RD_HOLD
  #undef NRD_CYCLE
  #undef TDF_CYCLES
  #undef TDF_OPTIM
  #undef PAGE_MODE
  #undef PAGE_SIZE
  #undef NCS_CONTROLLED_READ
  #undef NCS_CONTROLLED_WRITE
  #undef NWAIT_MODE
#endif


//! Whether to use the NCS1 pin
#ifdef CYGNUM_HAL_SMC_NCS1_ENABLED
  #include CYGNUM_HAL_SMC_NCS1_CFG_H

  // Enable SM mode for CS1 if necessary.
  AVR32_HMATRIXB.sfr[AVR32_EBI_HMATRIX_NR] &= ~(1 << AVR32_EBI_SDRAM_CS);
  AVR32_HMATRIXB.sfr[AVR32_EBI_HMATRIX_NR];

  // Setup SMC for NCS1
  SMC_CS_SETUP(1)

  #ifdef SMC_DBW_GLOBAL
    #if (SMC_DBW_GLOBAL < SMC_DBW)
        #undef  SMC_DBW_GLOBAL
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
    #endif
  #else
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
  #endif

  #ifdef SMC_8_BIT_CHIPS_GLOBAL
    #if (SMC_8_BIT_CHIPS_GLOBAL < SMC_8_BIT)
        #undef  SMC_8_BIT_CHIPS_GLOBAL
        #if     (SMC_8_BIT_CHIPS == TRUE)
          #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
        #elif   (SMC_8_BIT_CHIPS == FALSE)
          #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
        #else
          #error  error in SMC_8_BIT_CHIPS size
        #endif
    #endif
  #else
      #if     (SMC_8_BIT_CHIPS == TRUE)
        #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
      #elif   (SMC_8_BIT_CHIPS == FALSE)
        #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
      #else
        #error  error in SMC_8_BIT_CHIPS size
      #endif
  #endif

  #ifdef NWAIT_MODE_GLOBAL
    #if (NWAIT_MODE_GLOBAL < NWAIT_MODE)
        #undef  NWAIT_MODE_GLOBAL
        #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
        #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
        #else
          #error  error in NWAIT_MODE size
        #endif
    #endif
  #else
      #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
      #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
      #else
        #error  error in NWAIT_MODE size
      #endif
  #endif

  #undef EXT_SM_SIZE
  #undef SMC_DBW
  #undef SMC_8_BIT_CHIPS
  #undef NWE_SETUP
  #undef NCS_WR_SETUP
  #undef NRD_SETUP
  #undef NCS_RD_SETUP
  #undef NCS_WR_PULSE
  #undef NWE_PULSE
  #undef NCS_RD_PULSE
  #undef NRD_PULSE
  #undef NCS_WR_HOLD
  #undef NWE_HOLD
  #undef NWE_CYCLE
  #undef NCS_RD_HOLD
  #undef NRD_CYCLE
  #undef TDF_CYCLES
  #undef TDF_OPTIM
  #undef PAGE_MODE
  #undef PAGE_SIZE
  #undef NCS_CONTROLLED_READ
  #undef NCS_CONTROLLED_WRITE
  #undef NWAIT_MODE
#endif

//! Whether to use the NCS2 pin
#ifdef CYGNUM_HAL_SMC_NCS2_ENABLED
  #include CYGNUM_HAL_SMC_NCS2_CFG_H

  // Setup SMC for NCS2
  SMC_CS_SETUP(2)

  #ifdef SMC_DBW_GLOBAL
    #if (SMC_DBW_GLOBAL < SMC_DBW)
        #undef  SMC_DBW_GLOBAL
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
    #endif
  #else
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
  #endif

  #ifdef SMC_8_BIT_CHIPS_GLOBAL
    #if (SMC_8_BIT_CHIPS_GLOBAL < SMC_8_BIT)
        #undef  SMC_8_BIT_CHIPS_GLOBAL
        #if     (SMC_8_BIT_CHIPS == TRUE)
          #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
        #elif   (SMC_8_BIT_CHIPS == FALSE)
          #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
        #else
          #error  error in SMC_8_BIT_CHIPS size
        #endif
    #endif
  #else
      #if     (SMC_8_BIT_CHIPS == TRUE)
        #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
      #elif   (SMC_8_BIT_CHIPS == FALSE)
        #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
      #else
        #error  error in SMC_8_BIT_CHIPS size
      #endif
  #endif

  #ifdef NWAIT_MODE_GLOBAL
    #if (NWAIT_MODE_GLOBAL < NWAIT_MODE)
        #undef  NWAIT_MODE_GLOBAL
        #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
        #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
        #else
          #error  error in NWAIT_MODE size
        #endif
    #endif
  #else
      #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
      #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
      #else
        #error  error in NWAIT_MODE size
      #endif
  #endif


  #undef EXT_SM_SIZE
  #undef SMC_DBW
  #undef SMC_8_BIT_CHIPS
  #undef NWE_SETUP
  #undef NCS_WR_SETUP
  #undef NRD_SETUP
  #undef NCS_RD_SETUP
  #undef NCS_WR_PULSE
  #undef NWE_PULSE
  #undef NCS_RD_PULSE
  #undef NRD_PULSE
  #undef NCS_WR_HOLD
  #undef NWE_HOLD
  #undef NWE_CYCLE
  #undef NCS_RD_HOLD
  #undef NRD_CYCLE
  #undef TDF_CYCLES
  #undef TDF_OPTIM
  #undef PAGE_MODE
  #undef PAGE_SIZE
  #undef NCS_CONTROLLED_READ
  #undef NCS_CONTROLLED_WRITE
  #undef NWAIT_MODE
#endif

//! Whether to use the NCS3 pin
#ifdef CYGNUM_HAL_SMC_NCS3_ENABLED
  #include CYGNUM_HAL_SMC_NCS3_CFG_H

  // Setup SMC for NCS3
  SMC_CS_SETUP(3)

  #ifdef SMC_DBW_GLOBAL
    #if (SMC_DBW_GLOBAL < SMC_DBW)
        #undef  SMC_DBW_GLOBAL
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
    #endif
  #else
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
  #endif

  #ifdef SMC_8_BIT_CHIPS_GLOBAL
    #if (SMC_8_BIT_CHIPS_GLOBAL < SMC_8_BIT)
        #undef  SMC_8_BIT_CHIPS_GLOBAL
        #if     (SMC_8_BIT_CHIPS == TRUE)
          #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
        #elif   (SMC_8_BIT_CHIPS == FALSE)
          #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
        #else
          #error  error in SMC_8_BIT_CHIPS size
        #endif
    #endif
  #else
      #if     (SMC_8_BIT_CHIPS == TRUE)
        #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
      #elif   (SMC_8_BIT_CHIPS == FALSE)
        #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
      #else
        #error  error in SMC_8_BIT_CHIPS size
      #endif
  #endif

  #ifdef NWAIT_MODE_GLOBAL
    #if (NWAIT_MODE_GLOBAL < NWAIT_MODE)
        #undef  NWAIT_MODE_GLOBAL
        #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
        #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
        #else
          #error  error in NWAIT_MODE size
        #endif
    #endif
  #else
      #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
      #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
      #else
        #error  error in NWAIT_MODE size
      #endif
  #endif


  #undef EXT_SM_SIZE
  #undef SMC_DBW
  #undef SMC_8_BIT_CHIPS
  #undef NWE_SETUP
  #undef NCS_WR_SETUP
  #undef NRD_SETUP
  #undef NCS_RD_SETUP
  #undef NCS_WR_PULSE
  #undef NWE_PULSE
  #undef NCS_RD_PULSE
  #undef NRD_PULSE
  #undef NCS_WR_HOLD
  #undef NWE_HOLD
  #undef NWE_CYCLE
  #undef NCS_RD_HOLD
  #undef NRD_CYCLE
  #undef TDF_CYCLES
  #undef TDF_OPTIM
  #undef PAGE_MODE
  #undef PAGE_SIZE
  #undef NCS_CONTROLLED_READ
  #undef NCS_CONTROLLED_WRITE
  #undef NWAIT_MODE
#endif

//! Whether to use the NCS4 pin
#ifdef CYGNUM_HAL_SMC_NCS4_ENABLED
  #include CYGNUM_HAL_SMC_NCS4_CFG_H

  // Setup SMC for NCS4
  SMC_CS_SETUP(4)

  #ifdef SMC_DBW_GLOBAL
    #if (SMC_DBW_GLOBAL < SMC_DBW)
        #undef  SMC_DBW_GLOBAL
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
    #endif
  #else
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
  #endif

  #ifdef SMC_8_BIT_CHIPS_GLOBAL
    #if (SMC_8_BIT_CHIPS_GLOBAL < SMC_8_BIT)
        #undef  SMC_8_BIT_CHIPS_GLOBAL
        #if     (SMC_8_BIT_CHIPS == TRUE)
          #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
        #elif   (SMC_8_BIT_CHIPS == FALSE)
          #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
        #else
          #error  error in SMC_8_BIT_CHIPS size
        #endif
    #endif
  #else
      #if     (SMC_8_BIT_CHIPS == TRUE)
        #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
      #elif   (SMC_8_BIT_CHIPS == FALSE)
        #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
      #else
        #error  error in SMC_8_BIT_CHIPS size
      #endif
  #endif

  #ifdef NWAIT_MODE_GLOBAL
    #if (NWAIT_MODE_GLOBAL < NWAIT_MODE)
        #undef  NWAIT_MODE_GLOBAL
        #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
        #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
        #else
          #error  error in NWAIT_MODE size
        #endif
    #endif
  #else
      #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
      #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
      #else
        #error  error in NWAIT_MODE size
      #endif
  #endif


  #undef EXT_SM_SIZE
  #undef SMC_DBW
  #undef SMC_8_BIT_CHIPS
  #undef NWE_SETUP
  #undef NCS_WR_SETUP
  #undef NRD_SETUP
  #undef NCS_RD_SETUP
  #undef NCS_WR_PULSE
  #undef NWE_PULSE
  #undef NCS_RD_PULSE
  #undef NRD_PULSE
  #undef NCS_WR_HOLD
  #undef NWE_HOLD
  #undef NWE_CYCLE
  #undef NCS_RD_HOLD
  #undef NRD_CYCLE
  #undef TDF_CYCLES
  #undef TDF_OPTIM
  #undef PAGE_MODE
  #undef PAGE_SIZE
  #undef NCS_CONTROLLED_READ
  #undef NCS_CONTROLLED_WRITE
  #undef NWAIT_MODE
#endif

//! Whether to use the NCS5 pin
#ifdef CYGNUM_HAL_SMC_NCS5_ENABLED
  #include CYGNUM_HAL_SMC_NCS5_CFG_H

  // Setup SMC for NCS5
  SMC_CS_SETUP(5)

  #ifdef SMC_DBW_GLOBAL
    #if (SMC_DBW_GLOBAL < SMC_DBW)
        #undef  SMC_DBW_GLOBAL
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
    #endif
  #else
        #if     (SMC_DBW == 8)
            #define SMC_DBW_GLOBAL                  8
        #elif   (SMC_DBW == 16)
            #define SMC_DBW_GLOBAL                  16
        #elif   (SMC_DBW == 32)
            #define SMC_DBW_GLOBAL                  32
        #else
          #error  error in SMC_DBW size
        #endif
  #endif

  #ifdef SMC_8_BIT_CHIPS_GLOBAL
    #if (SMC_8_BIT_CHIPS_GLOBAL < SMC_8_BIT)
        #undef  SMC_8_BIT_CHIPS_GLOBAL
        #if     (SMC_8_BIT_CHIPS == TRUE)
          #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
        #elif   (SMC_8_BIT_CHIPS == FALSE)
          #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
        #else
          #error  error in SMC_8_BIT_CHIPS size
        #endif
    #endif
  #else
      #if     (SMC_8_BIT_CHIPS == TRUE)
        #define SMC_8_BIT_CHIPS_GLOBAL            TRUE
      #elif   (SMC_8_BIT_CHIPS == FALSE)
        #define SMC_8_BIT_CHIPS_GLOBAL            FALSE
      #else
        #error  error in SMC_8_BIT_CHIPS size
      #endif
  #endif

  #ifdef NWAIT_MODE_GLOBAL
    #if (NWAIT_MODE_GLOBAL < NWAIT_MODE)
        #undef  NWAIT_MODE_GLOBAL
        #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
        #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
          #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
        #else
          #error  error in NWAIT_MODE size
        #endif
    #endif
  #else
      #if     (NWAIT_MODE == AVR32_SMC_EXNW_MODE_DISABLED)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_DISABLED
      #elif   (NWAIT_MODE == AVR32_SMC_EXNW_MODE_FROZEN)
        #define NWAIT_MODE_GLOBAL            AVR32_SMC_EXNW_MODE_FROZEN
      #else
        #error  error in NWAIT_MODE size
      #endif
  #endif


  #undef EXT_SM_SIZE
  #undef SMC_DBW
  #undef SMC_8_BIT_CHIPS
  #undef NWE_SETUP
  #undef NCS_WR_SETUP
  #undef NRD_SETUP
  #undef NCS_RD_SETUP
  #undef NCS_WR_PULSE
  #undef NWE_PULSE
  #undef NCS_RD_PULSE
  #undef NRD_PULSE
  #undef NCS_WR_HOLD
  #undef NWE_HOLD
  #undef NWE_CYCLE
  #undef NCS_RD_HOLD
  #undef NRD_CYCLE
  #undef TDF_CYCLES
  #undef TDF_OPTIM
  #undef PAGE_MODE
  #undef PAGE_SIZE
  #undef NCS_CONTROLLED_READ
  #undef NCS_CONTROLLED_WRITE
  #undef NWAIT_MODE
#endif
  // Put the multiplexed MCU pins used for the SM under control of the SMC.
  smc_enable_muxed_pins();
}

/*! \brief Puts the multiplexed MCU pins used for the SMC
 *
 */
static void smc_enable_muxed_pins(void)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[2];
    gpio_set_module_pin(AVR32_EBI_DATA_0_PIN,AVR32_EBI_DATA_0_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_1_PIN,AVR32_EBI_DATA_1_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_2_PIN,AVR32_EBI_DATA_2_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_3_PIN,AVR32_EBI_DATA_3_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_4_PIN,AVR32_EBI_DATA_4_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_5_PIN,AVR32_EBI_DATA_5_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_6_PIN,AVR32_EBI_DATA_6_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_7_PIN,AVR32_EBI_DATA_7_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_8_PIN,AVR32_EBI_DATA_8_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_9_PIN,AVR32_EBI_DATA_9_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_10_PIN,AVR32_EBI_DATA_10_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_11_PIN,AVR32_EBI_DATA_11_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_12_PIN,AVR32_EBI_DATA_12_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_13_PIN,AVR32_EBI_DATA_13_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_14_PIN,AVR32_EBI_DATA_14_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_DATA_15_PIN,AVR32_EBI_DATA_15_FUNCTION);

    gpio_set_module_pin(AVR32_EBI_ADDR_0_PIN,AVR32_EBI_ADDR_0_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_1_PIN,AVR32_EBI_ADDR_1_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_2_PIN,AVR32_EBI_ADDR_2_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_3_PIN,AVR32_EBI_ADDR_3_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_4_PIN,AVR32_EBI_ADDR_4_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_5_PIN,AVR32_EBI_ADDR_5_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_6_PIN,AVR32_EBI_ADDR_6_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_7_PIN,AVR32_EBI_ADDR_7_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_8_PIN,AVR32_EBI_ADDR_8_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_9_PIN,AVR32_EBI_ADDR_9_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_10_PIN,AVR32_EBI_ADDR_10_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_11_PIN,AVR32_EBI_ADDR_11_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_12_PIN,AVR32_EBI_ADDR_12_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_13_PIN,AVR32_EBI_ADDR_13_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_14_PIN,AVR32_EBI_ADDR_14_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_ADDR_15_PIN,AVR32_EBI_ADDR_15_FUNCTION);

    gpio_set_module_pin(AVR32_EBI_NCS_1_PIN,AVR32_EBI_NCS_1_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_NRD_PIN,AVR32_EBI_NRD_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_NWE0_PIN,AVR32_EBI_NWE0_FUNCTION);
    gpio_set_module_pin(AVR32_EBI_NWE1_PIN,AVR32_EBI_NWE1_FUNCTION);

    // Disable GPIO control.
    gpio_port->gperc = 0xfff80000;

    gpio_port = (avr32_gpio_port_t*)&AVR32_GPIO.port[3];
    // Disable GPIO control.
    gpio_port->gperc = 0x1e0fdfff;
}

unsigned char smc_get_cs_size(unsigned char cs)
{
  return smc_tab_cs_size[cs];
}
