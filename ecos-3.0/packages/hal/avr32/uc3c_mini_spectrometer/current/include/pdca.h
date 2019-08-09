/*****************************************************************************
 *
 * \file
 *
 * \brief PDCA driver for AVR32 UC3.
 *
 * This file defines a useful set of functions for the PDCA interface on AVR32
 * devices.
 *
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/


#ifndef _PDCA_H_
#define _PDCA_H_

/**
 * \defgroup group_avr32_drivers_pdca MEMORY - PDCA - Peripheral DMA Controller
 *
 * The Peripheral DMA controller (PDCA) transfers data between on-chip
 * peripheral modules such as USART, SPI, SSC and on- and off-chip memories.
 *
 * \{
 */

#include <cyg/hal/avr32/io.h>

#define PDMA_CH0      0
#define PDMA_CH1      1
#define PDMA_CH2      2
#define PDMA_CH3      3
#define PDMA_CH4      4
#define PDMA_CH5      5
#define PDMA_CH6      6
#define PDMA_CH7      7
#define PDMA_CH8      8
#define PDMA_CH9      9
#define PDMA_CH10     10
#define PDMA_CH11     11
#define PDMA_CH12     12
#define PDMA_CH13     13
#define PDMA_CH14     14
#define PDMA_CH15     15
//! Size of PDCA transfer: byte.
#define PDCA_TRANSFER_SIZE_BYTE               AVR32_PDCA_BYTE

//! Size of PDCA transfer: half-word.
#define PDCA_TRANSFER_SIZE_HALF_WORD          AVR32_PDCA_HALF_WORD

//! Size of PDCA transfer: word.
#define PDCA_TRANSFER_SIZE_WORD               AVR32_PDCA_WORD

/*! \name PDCA Driver Status Codes
 */
//! @{
#define PDCA_SUCCESS 0
#define PDCA_INVALID_ARGUMENT -1
//! @}

/*! \name PDCA Transfer Status Codes
 */
//! @{
#define PDCA_TRANSFER_ERROR                   AVR32_PDCA_TERR_MASK
#define PDCA_TRANSFER_COMPLETE                AVR32_PDCA_TRC_MASK
#define PDCA_TRANSFER_COUNTER_RELOAD_IS_ZERO  AVR32_PDCA_RCZ_MASK
//! @}


//! PDCA channel options.
typedef struct
{
  //! Memory address.
  volatile  void         *addr          ;
  //! Transfer counter.
            unsigned int  size          ;
  //! Next memory address.
  volatile  void         *r_addr        ;
  //! Next transfer counter.
            unsigned int  r_size        ;
  //! Select peripheral ID.
            unsigned int  pid           ;
  //! Select the size of the transfer (byte, half-word or word).
            unsigned int  transfer_size ;
#if (AVR32_PDCA_H_VERSION >= 120)
// Note: the options in this preprocessor section are only available from the PDCA IP version 1.2.0 on.
  //! Enable (\c 1) or disable (\c 0) the transfer upon event trigger.
            unsigned char etrig         ;
#endif
} pdca_channel_options_t;


#define CYG_PDMA_GET_CHANEL( _num_)            \
		&AVR32_PDCA.channel[_num_];


#define CYG_PDAM_SET_CHANEL(_chan_, _saddr_, _size_, _pid_, _raddr_, _rsize_, _tsize_)  \
  {                                                                                     \
  CYG_INTERRUPT_STATE old_intr;                                                         \
  HAL_DISABLE_INTERRUPTS(old_intr);                                                     \
  _chan_->mar = (cyg_uint32)_saddr_;                                                    \
  _chan_->tcr = _size_;                                                                 \
  _chan_->psr = _pid_;                                                                  \
  _chan_->marr = _raddr_;                                                               \
  _chan_->tcrr = _rsize_;                                                               \
  _chan_->mr = _tsize_ << AVR32_PDCA_SIZE_OFFSET;                                       \
  _chan_->cr = AVR32_PDCA_ECLR_MASK;                                                    \
  _chan_->isr;                                                                          \
  HAL_RESTORE_INTERRUPTS(old_intr);                                                     \
  }
  
#define CYG_PDAM_SET_CHANEL_SAME(_chan_, _saddr_,  _pid_, _tsize_)   \
  {                                                                  \
  CYG_INTERRUPT_STATE old_intr;                                      \
  HAL_DISABLE_INTERRUPTS(old_intr);                                  \
  _chan_->mar = (cyg_uint32)_saddr_;                                 \
  _chan_->tcr = 1;                                                   \
  _chan_->psr = _pid_;                                               \
  _chan_->marr = (cyg_uint32)_saddr_;                                \
  _chan_->tcrr = 1;                                                  \
  _chan_->mr = _tsize_ << AVR32_PDCA_SIZE_OFFSET                     \
		| AVR32_PDCA_RING_MASK;                              \
  _chan_->cr = AVR32_PDCA_ECLR_MASK;                                 \
  _chan_->isr;                                                       \
  HAL_RESTORE_INTERRUPTS(old_intr);                                  \
  }
  
#define CYG_PDMA_ENABLE_CHANEL(_chan_)	\
	_chan_->cr = AVR32_PDCA_TEN_MASK;
	
#define CYG_PDMA_DISABLE_CHANEL(_chan_)	    \
 CYG_PDMA_DISABLE_INTERRUPT(_chan_);        \
 _chan_->cr = AVR32_PDCA_TDIS_MASK
 
#define CYG_PDMA_CHANEL_CLR_ERROR(_chan_)   \
  _chan_->cr = AVR32_PDCA_ECLR_MASK;
	
#define CYG_PDMA_IS_ENABLED(_chan_) _chan_->sr & 0x1

#define CYG_PDAM_IS_ERROR(_chan_)  \
	(_chan_->isr >> AVR32_PDCA_ISR0_TERR_OFFSET) & 0x1
	
#define CYG_PDMA_IS_TRANSFER_COMPLETE(_chan_)  \
(_chan_->isr >> AVR32_PDCA_ISR0_TRC_OFFSET) & 0x1

#define CYG_PDAM_IS_RELOUD_COUNT_ZERO(_chan_)  \
(_chan_->isr >> AVR32_PDCA_ISR0_RCZ_OFFSET) & 0x1

#define CYG_PDMA_ENABLE_INTERRUPT(_chan_, _terr_, _complete_, _reloadzero_)   \
	_chan_->ier = (_terr_ << AVR32_PDCA_IER0_TERR_OFFSET) |               \
	 (_complete_ << AVR32_PDCA_IER0_TRC_OFFSET)                           \
	 | (_reloadzero_ << AVR32_PDCA_IER0_RCZ_OFFSET)	
	 
#define CYG_PDMA_DISABLE_INTERRUPT(_chan_)  	_chan_->idr = 0x07

#define CYG_PDMA_INTERRUPT_STATUS(_chan_)      _chan_->isr

#define CYG_PDMA_RELOAD(_chan_, _addr_, _size_)         \
  {                                                     \
  CYG_INTERRUPT_STATE old_intr;			        \
  HAL_DISABLE_INTERRUPTS(old_intr);		        \
  _chan_->marr = _addr_;			        \
  _chan_->tcrr = _size_;			        \
  HAL_RESTORE_INTERRUPTS(old_intr);                     \
  }

#define CYG_PDMA_FORCE_RELOAD(_chan_)       \
  {                                         \
  CYG_INTERRUPT_STATE old_intr;		    \
  HAL_DISABLE_INTERRUPTS(old_intr);	    \
  _chan_->tcr = 0;			    \
  HAL_RESTORE_INTERRUPTS(old_intr);         \
  }

#endif  // _PDCA_H_
