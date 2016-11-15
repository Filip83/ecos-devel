//==========================================================================
//
//      avr32_serial.c
//
//      Atmel AV32UC3C Serial I/O Interface Module (PDMA driven)
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
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
// Author(s):     Filip
// Contributors:  
// Original data: gthomas
// Date:          2014-05-10
// Purpose:       Atmel AVR32UC3C Serial I/O module (PDMA driven version)
// Description: 
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/hal/gpio.h>
#include <cyg/hal/avr32/io.h>

#include <pkgconf/hal.h>
#include <pkgconf/infra.h>
#include <pkgconf/system.h>
#include <pkgconf/io_serial.h>
#include <pkgconf/io.h>
#include <pkgconf/kernel.h>

#include <cyg/io/io.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/io/devtab.h>
#include <cyg/io/serial.h>
#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/pdca.h>
#include <cyg/hal/board_config.h>

externC void * memcpy( void *, const void *, size_t );

#ifndef CYGPKG_IO_SERIAL_AVR32

#include "avr32_serial.h"


#define RCVBUF_EXTRA        16
#define RCV_TIMEOUT         2000

#define SIFLG_NONE          0x00
#define SIFLG_TX_READY      0x01
#define SIFLG_XMIT_BUSY     0x02
#define SIFLG_XMIT_CONTINUE 0x04


typedef struct avr32_serial_info {
    volatile                        avr32_usart_t  *usart_dev;
    CYG_WORD                        int_num;
    CYG_WORD                        int_num_dma_rx;
    CYG_WORD                        int_num_dma_tx;
    cyg_uint32                      stat;
    int                             transmit_size;
    cyg_interrupt                   serial_interrupt;
    cyg_handle_t                    serial_interrupt_handle;
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    cyg_interrupt                   serial_dma_tx_interrupt;
    cyg_handle_t                    serial_dma_tx_interrupt_handle;
    cyg_uint8                       *rcv_buffer[2];
    cyg_uint8                       dma_tx_ch;
    cyg_uint8                       dma_pid_tx;
    volatile avr32_pdca_channel_t   *pdca_tx_channel;

    cyg_interrupt                   serial_dma_rx_interrupt;
    cyg_handle_t                    serial_dma_rx_interrupt_handle;
    cyg_uint8                       dma_rx_ch;
    cyg_uint8                       dma_pid_rx;
    volatile avr32_pdca_channel_t   *pdca_rx_channel;
    cyg_uint8                       curbuf;
    cyg_uint16                      ping_pong_size;
    cyg_uint8                       *end;
    cyg_bool                        trans_rdy;
#endif
    cyg_uint8                       flags;
} avr32_serial_info;

static bool avr32_serial_init(struct cyg_devtab_entry *tab);
static bool avr32_serial_putc_interrupt(serial_channel *chan, unsigned char c);
#if (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL0) && CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL1) && CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL2) && CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL3) && CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE == 0)
static bool avr32_serial_putc_polled(serial_channel *chan, unsigned char c);
#endif
static Cyg_ErrNo avr32_serial_lookup(struct cyg_devtab_entry **tab, 
                                    struct cyg_devtab_entry *sub_tab,
                                    const char *name);
static unsigned char avr32_serial_getc_interrupt(serial_channel *chan);
#if (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL0) && CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL1) && CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL2) && CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL3) && CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE == 0)
static unsigned char avr32_serial_getc_polled(serial_channel *chan);
#endif
static Cyg_ErrNo avr32_serial_set_config(serial_channel *chan, cyg_uint32 key,
                                        const void *xbuf, cyg_uint32 *len);
static void avr32_serial_start_xmit(serial_channel *chan);
static void avr32_serial_stop_xmit(serial_channel *chan);

static cyg_uint32 avr32_serial_ISR(cyg_vector_t vector, cyg_addrword_t data);
static void       avr32_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data);

#if (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL0) && CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE > 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL1) && CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE > 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL2) && CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE > 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL3) && CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE > 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE > 0) 
static SERIAL_FUNS(avr32_serial_funs_interrupt, 
                   avr32_serial_putc_interrupt, 
                   avr32_serial_getc_interrupt,
                   avr32_serial_set_config,
                   avr32_serial_start_xmit,
                   avr32_serial_stop_xmit
    );
#endif

#if (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL0) && CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL1) && CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL2) && CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL3) && CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE == 0)
static SERIAL_FUNS(avr32_serial_funs_polled, 
                   avr32_serial_putc_polled, 
                   avr32_serial_getc_polled,
                   avr32_serial_set_config,
                   avr32_serial_start_xmit,
                   avr32_serial_stop_xmit
    );
#endif

#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL0
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
static cyg_uint8 avr32_serial_rcv_buffer_0
    [2][CYGNUM_IO_SERIAL_AVR32_SERIAL0_PING_PONG_BUFSIZE];
#endif
static avr32_serial_info avr32_serial_info0 = {
    usart_dev       : (avr32_usart_t*) AVR32_USART0_ADDRESS,
    int_num         : CYGNUM_HAL_VECTOR_USART0,
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    int_num_dma_rx  : CYGNUM_HAL_VECTOR_PDMA_CH4,
    dma_rx_ch       : 4,
    dma_pid_rx      : AVR32_PDCA_PID_USART0_RX,
    int_num_dma_tx  : CYGNUM_HAL_VECTOR_PDMA_CH5,
    dma_pid_tx      : AVR32_PDCA_PID_USART0_TX,
    dma_tx_ch       : 5,
    ping_pong_size  : CYGNUM_IO_SERIAL_AVR32_SERIAL0_PING_PONG_BUFSIZE,
    rcv_buffer      : {avr32_serial_rcv_buffer_0[0], avr32_serial_rcv_buffer_0[1]},
    trans_rdy       : false
#endif
};

#if CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE > 0
static unsigned char avr32_serial_out_buf0[CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE];
static unsigned char avr32_serial_in_buf0[CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE];

static SERIAL_CHANNEL_USING_INTERRUPTS(
                    avr32_serial_channel0,
                    avr32_serial_funs_interrupt, 
                    avr32_serial_info0,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL0_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT,
                    &avr32_serial_out_buf0[0], sizeof(avr32_serial_out_buf0),
                    &avr32_serial_in_buf0[0], sizeof(avr32_serial_in_buf0)
    );
#else
static SERIAL_CHANNEL(
                    avr32_serial_channel0,
                    avr32_serial_funs_polled, 
                    avr32_serial_info0,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL0_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT
    );
#endif

DEVTAB_ENTRY(avr32_serial_io0, 
             CYGDAT_IO_SERIAL_AVR32_SERIAL0_NAME,
             0,                     // Does not depend on a lower level interface
             &cyg_io_serial_devio, 
             avr32_serial_init, 
             avr32_serial_lookup,   // Serial driver may need initializing
             &avr32_serial_channel0
    );
#endif //  CYGPKG_IO_SERIAL_AVR32_SERIAL1

#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL1
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
static cyg_uint8 avr32_serial_rcv_buffer_1
    [2][CYGNUM_IO_SERIAL_AVR32_SERIAL1_PING_PONG_BUFSIZE];
#endif
static avr32_serial_info avr32_serial_info1 = {
    usart_dev       : (avr32_usart_t*) AVR32_USART1_ADDRESS,
    int_num         : CYGNUM_HAL_VECTOR_USART1,
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    int_num_dma_rx  : CYGNUM_HAL_VECTOR_PDMA_CH6,
    dma_rx_ch       : 6,
    dma_pid_rx      : AVR32_PDCA_PID_USART1_RX,
    int_num_dma_tx  : CYGNUM_HAL_VECTOR_PDMA_CH7,
    dma_pid_tx      : AVR32_PDCA_PID_USART1_TX,
    dma_tx_ch       : 7,
    ping_pong_size   : CYGNUM_IO_SERIAL_AVR32_SERIAL1_PING_PONG_BUFSIZE,
    rcv_buffer      : {avr32_serial_rcv_buffer_1[0], avr32_serial_rcv_buffer_1[1]},
    trans_rdy       : false
#endif
};
#if CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE > 0
static unsigned char avr32_serial_out_buf1[CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE];
static unsigned char avr32_serial_in_buf1[CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE];

static SERIAL_CHANNEL_USING_INTERRUPTS(
                    avr32_serial_channel1,
                    avr32_serial_funs_interrupt, 
                    avr32_serial_info1,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL1_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT,
                    &avr32_serial_out_buf1[0], sizeof(avr32_serial_out_buf1),
                    &avr32_serial_in_buf1[0], sizeof(avr32_serial_in_buf1)
    );
#else
static SERIAL_CHANNEL(
                    avr32_serial_channel1,
                    avr32_serial_funs_polled, 
                    avr32_serial_info1,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL1_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT
    );
#endif

DEVTAB_ENTRY(avr32_serial_io1, 
             CYGDAT_IO_SERIAL_AVR32_SERIAL1_NAME,
             0,                     // Does not depend on a lower level interface
             &cyg_io_serial_devio, 
             avr32_serial_init, 
             avr32_serial_lookup,     // Serial driver may need initializing
             &avr32_serial_channel1
    );
#endif //  CYGPKG_IO_SERIAL_AVR32_SERIAL1


#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL2
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
static cyg_uint8 avr32_serial_rcv_buffer_2
    [2][CYGNUM_IO_SERIAL_AVR32_SERIAL2_PING_PONG_BUFSIZE];
#endif
static avr32_serial_info avr32_serial_info2 = {
    usart_dev       : (avr32_usart_t*) AVR32_USART2_ADDRESS,
    int_num         : CYGNUM_HAL_VECTOR_USART2,
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    int_num_dma_rx  : CYGNUM_HAL_VECTOR_PDMA_CH8,
    dma_rx_ch       : 8,
    dma_pid_rx      : AVR32_PDCA_PID_USART2_RX,
    int_num_dma_tx  : CYGNUM_HAL_VECTOR_PDMA_CH9,
    dma_pid_tx      : AVR32_PDCA_PID_USART2_TX,
    dma_tx_ch       : 9,
    ping_pong_size   : CYGNUM_IO_SERIAL_AVR32_SERIAL2_PING_PONG_BUFSIZE,
    rcv_buffer      : {avr32_serial_rcv_buffer_2[0], avr32_serial_rcv_buffer_2[1]},
    trans_rdy       : false
#endif
};

#if CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE > 0
static unsigned char avr32_serial_out_buf2[CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE];
static unsigned char avr32_serial_in_buf2[CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE];

static SERIAL_CHANNEL_USING_INTERRUPTS(
                    avr32_serial_channel2,
                    avr32_serial_funs_interrupt, 
                    avr32_serial_info2,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL2_BAUD),0x0181
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT,
                    &avr32_serial_out_buf2[0], sizeof(avr32_serial_out_buf2),
                    &avr32_serial_in_buf2[0], sizeof(avr32_serial_in_buf2)
    );
#else
static SERIAL_CHANNEL(
                    avr32_serial_channel2,
                    avr32_serial_funs_polled, 
                    avr32_serial_info2,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL2_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT
    );
#endif

DEVTAB_ENTRY(avr32_serial_io2, 
             CYGDAT_IO_SERIAL_AVR32_SERIAL2_NAME,
             0,                     // Does not depend on a lower level interface
             &cyg_io_serial_devio, 
             avr32_serial_init, 
             avr32_serial_lookup,     // Serial driver may need initializing
             &avr32_serial_channel2
    );
#endif //  CYGPKG_IO_SERIAL_AVR32_SERIAL2

#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL3
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
static cyg_uint8 avr32_serial_rcv_buffer_3
    [2][CYGNUM_IO_SERIAL_AVR32_SERIAL3_PING_PONG_BUFSIZE];
#endif
static avr32_serial_info avr32_serial_info3 = {
    usart_dev       : (avr32_usart_t*) AVR32_USART3_ADDRESS,
    int_num         : CYGNUM_HAL_VECTOR_USART3,
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    int_num_dma_rx  : CYGNUM_HAL_VECTOR_PDMA_CH10,
    dma_rx_ch       : 10,
    dma_pid_rx      : AVR32_PDCA_PID_USART3_RX,
    int_num_dma_tx  : CYGNUM_HAL_VECTOR_PDMA_CH11,
    dma_pid_tx      : AVR32_PDCA_PID_USART3_TX,
    dma_tx_ch       : 11,
    ping_pong_size   : CYGNUM_IO_SERIAL_AVR32_SERIAL3_PING_PONG_BUFSIZE,
    rcv_buffer      : {avr32_serial_rcv_buffer_3[0], avr32_serial_rcv_buffer_3[1]},
    trans_rdy       : false
#endif
};

#if CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE > 0
static unsigned char avr32_serial_out_buf3[CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE];
static unsigned char avr32_serial_in_buf3[CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE];

static SERIAL_CHANNEL_USING_INTERRUPTS(
            avr32_serial_channel3,AVR32_PIN_PC15
            avr32_serial_funs_interrupt, 
            avr32_serial_info3,
            CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL3_BAUD),
            CYG_SERIAL_STOP_DEFAULT,
            CYG_SERIAL_PARITY_DEFAULT,
            CYG_SERIAL_WORD_LENGTH_DEFAULT,
            CYG_SERIAL_FLAGS_DEFAULT,
            &avr32_serial_out_buf3[0], sizeof(avr32_serial_out_buf3),
            &avr32_serial_in_buf3[0], sizeof(avr32_serial_in_buf3)
    );
#else
static SERIAL_CHANNEL(
                    avr32_serial_channel3,
                    avr32_serial_funs_polled, 
                    avr32_serial_info3,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL3_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT
    );
#endif

DEVTAB_ENTRY(avr32_serial_io3, 
             CYGDAT_IO_SERIAL_AVR32_SERIAL3_NAME,
             0,                     // Does not depend on a lower level interface
             &cyg_io_serial_devio, 
             avr32_serial_init, 
             avr32_serial_lookup,     // Serial driver may need initializing
             &avr32_serial_channel3
    );
#endif //  CYGPKG_IO_SERIAL_AVR32_SERIAL3

#if defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && defined(CYGNUM_HAL_VECTOR_USART4)
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
static cyg_uint8 avr32_serial_rcv_buffer_4
    [2][RCYGNUM_IO_SERIAL_AVR32_SERIAL4_PING_PONG_BUFSIZE];
#endif
static avr32_serial_info avr32_serial_info4 = {
    usart_dev       : (avr32_usart_t*) AVR32_USART4_ADDRESS,
    int_num         : CYGNUM_HAL_VECTOR_USART4,
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    int_num_dma_rx  : CYGNUM_HAL_VECTOR_PDMA_CH12,
    dma_rx_ch       : 12,
    dma_pid_rx      : AVR32_PDCA_PID_USART4_RX,
    int_num_dma_tx  : CYGNUM_HAL_VECTOR_PDMA_CH13,
    dma_pid_tx      : AVR32_PDCA_PID_USART4_TX,
    dma_tx_ch       : 13,
    ping_pong_size   : CYGNUM_IO_SERIAL_AVR32_SERIAL4_PING_PONG_BUFSIZE,
    rcv_buffer      : {avr32_serial_rcv_buffer_4[0], avr32_serial_rcv_buffer_4[1]},
    trans_rdy       : false
#endif
};

#if CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE > 0
static unsigned char avr32_serial_out_buf4[CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE];
static unsigned char avr32_serial_in_buf4[CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE];

static SERIAL_CHANNEL_USING_INTERRUPTS(
            avr32_serial_channel4,
            avr32_serial_funs_interrupt, 
            avr32_serial_info4,
            CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL4_BAUD),
            CYG_SERIAL_STOP_DEFAULT,
            CYG_SERIAL_PARITY_DEFAULT,
            CYG_SERIAL_WORD_LENGTH_DEFAULT,
            CYG_SERIAL_FLAGS_DEFAULT,
            &avr32_serial_out_buf4[0], sizeof(avr32_serial_out_buf4),
            &avr32_serial_in_buf4[0], sizeof(avr32_serial_in_buf4)
    );
#else
static SERIAL_CHANNEL(
                    avr32_serial_channel4,
                    avr32_serial_funs_polled,
                    avr32_serial_info4,
                    CYG_SERIAL_BAUD_RATE(CYGNUM_IO_SERIAL_AVR32_SERIAL4_BAUD),
                    CYG_SERIAL_STOP_DEFAULT,
                    CYG_SERIAL_PARITY_DEFAULT,
                    CYG_SERIAL_WORD_LENGTH_DEFAULT,
                    CYG_SERIAL_FLAGS_DEFAULT
    );
#endif

DEVTAB_ENTRY(avr32_serial_io4, 
             CYGDAT_IO_SERIAL_AVR32_SERIAL4_NAME,
             0,                     // Does not depend on a lower level interface
             &cyg_io_serial_devio, 
             avr32_serial_init, 
             avr32_serial_lookup,     // Serial driver may need initializing
             &avr32_serial_channel4
    );
#endif //  CYGPKG_IO_SERIAL_AVR32_SERIAL4


// Internal function to actually configure the hardware to desired baud rate, etc.
static bool
avr32_serial_config_port(serial_channel *chan, cyg_serial_info_t *new_config, bool init)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *)chan->dev_priv;

    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyg_uint32 parity = select_parity[new_config->parity];
    cyg_uint32 word_length = 
            select_word_length[new_config->word_length-CYGNUM_SERIAL_WORD_LENGTH_5];
    cyg_uint32 stop_bits = select_stop_bits[new_config->stop - 1]; 
    CYG_ADDRESS usart_addr = (CYG_ADDRESS)avr32_chan->usart_dev;

    if ((word_length == 0xFF) ||
        (parity == 0xFF) ||
        (stop_bits == 0xFF)) {
        return false;  // Unsupported configuration
    }

    if(usart_addr == AVR32_USART0_ADDRESS)
    {
#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL0
#if CYG_HAL_USART0_TXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART0_TXD_PIN,
                CYG_HAL_USART0_TXD_FUNCTION);
#endif
#if CYG_HAL_USART0_RXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART0_RXD_PIN, 
                CYG_HAL_USART0_RXD_FUNCTION);
#endif
    #ifdef CYGNUM_IO_SERIAL_AVR32_SERIAL0_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART0_RTS_PIN, 
                CYG_DEVS_USART0_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART0_CTS_PIN, 
                CYG_DEVS_USART0_CTS_FUNCTION);
    #endif
#endif
    }else if(usart_addr == AVR32_USART1_ADDRESS)		
    {
#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL1
#if CYG_HAL_USART1_TXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART1_TXD_PIN,
                CYG_HAL_USART1_TXD_FUNCTION);
#endif
#if CYG_HAL_USART1_RXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART1_RXD_PIN, 
                CYG_HAL_USART1_RXD_FUNCTION);
#endif
    #ifdef  CYGNUM_IO_SERIAL_AVR32_SERIAL1_MODEM_MODE
        gpio_enable_module_pin(CYG_HAL_AVR32_USART1_DTR_PIN,
                CYG_HAL_AVR32_USART1_DTR_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_AVR32_USART1_DSR_PIN, 
                CYG_HAL_AVR32_USART1_DSR_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_AVR32_USART1_DCD_PIN,
                CYG_HAL_AVR32_USART1_DCD_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_AVR32_USART1_RI_PIN,  
                CYG_HAL_AVR32_USART1_RI_FUNCTION);
     #endif
     #if defined(CYGNUM_IO_SERIAL_AVR32_SERIAL1_FLOW_CONTROL) || \
         defined(CYGNUM_IO_SERIAL_AVR32_SERIAL1_MODEM_MODE)
        gpio_enable_module_pin(CYG_HAL_AVR32_USART1_RTS_PIN, 
                CYG_HAL_AVR32_USART1_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_HAL_AVR32_USART1_CTS_PIN, 
                CYG_HAL_AVR32_USART1_CTS_FUNCTION);
     #endif
#endif
    }		
    else if(usart_addr == AVR32_USART2_ADDRESS)
    {
#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL2
#if CYG_HAL_USART2_TXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART2_TXD_PIN,
                CYG_HAL_USART2_TXD_FUNCTION);
#endif
#if CYG_HAL_USART2_RXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART2_RXD_PIN, 
                CYG_HAL_USART2_RXD_FUNCTION);
#endif
    #ifdef CYGNUM_IO_SERIAL_AVR32_SERIAL2_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART2_RTS_PIN, 
                CYG_DEVS_USART2_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART2_CTS_PIN, 
                CYG_DEVS_USART2_CTS_FUNCTION);
    #endif
#endif
    }	
    else if(usart_addr == AVR32_USART3_ADDRESS)
    {
#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL3
#if CYG_HAL_USART3_TXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART3_TXD_PIN,
                CYG_HAL_USART3_TXD_FUNCTION);
#endif
#if CYG_HAL_USART3_RXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART3_RXD_PIN, 
                CYG_HAL_USART3_RXD_FUNCTION);
#endif
    #ifdef CYGNUM_IO_SERIAL_AVR32_SERIAL3_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART3_RTS_PIN, 
                CYG_DEVS_USART3_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART3_CTS_PIN, 
                CYG_DEVS_USART3_CTS_FUNCTION);
    #endif
#endif
    }	
#ifdef AVR32_USART4_ADDRESS
    else if(usart_addr == AVR32_USART4_ADDRESS)
    {
#ifdef CYGPKG_IO_SERIAL_AVR32_SERIAL4
#if CYG_HAL_USART4_TXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART4_TXD_PIN,
                CYG_HAL_USART4_TXD_FUNCTION);
#endif
#if CYG_HAL_USART4_RXD_ENABLED == PIN_ENABLE
        gpio_enable_module_pin(CYG_HAL_USART4_RXD_PIN, 
                CYG_HAL_USART4_RXD_FUNCTION);
#endif
    #ifdef CYGNUM_IO_SERIAL_AVR32_SERIAL4_FLOW_CONTROL
        gpio_enable_module_pin(CYG_DEVS_USART4_RTS_PIN, 
                CYG_DEVS_USART4_RTS_FUNCTION);
        gpio_enable_module_pin(CYG_DEVS_USART4_CTS_PIN, 
                CYG_DEVS_USART4_CTS_FUNCTION);
    #endif
#endif
    }
#endif
    else
    {
        CYG_ASSERT(false,"Unknownd USART address\n");
    }
    // Reset device
    dev->cr = AVR32_USART_CR_RSTRX_MASK | 
              AVR32_USART_CR_RSTTX_MASK | 
              AVR32_USART_CR_RSTSTA_MASK ;

    // Configuration
    if((new_config->flags & CYGNUM_SERIAL_FLOW_RTSCTS_RX) ||
        (new_config->flags & CYGNUM_SERIAL_FLOW_RTSCTS_TX))
        dev->mr = stop_bits | parity | word_length | 
                AVR32_USART_MODE_HARDWARE | AVR32_USART_MR_OVER_MASK;
    else if((new_config->flags & CYGNUM_SERIAL_FLOW_DSRDTR_RX) ||
        (new_config->flags & CYGNUM_SERIAL_FLOW_DSRDTR_TX))
        dev->mr = stop_bits | parity | word_length | 
                AVR32_USART_MODE_MODEM | AVR32_USART_MR_OVER_MASK;
    else
    {
        dev->mr = stop_bits | parity | word_length | 
                AVR32_USART_MODE_NORMAL | AVR32_USART_MR_OVER_MASK;
	dev->cr = AVR32_USART_CR_RTSEN_MASK;
    }
 
    // Baud rate
    dev->brgr = select_baud[new_config->baud];
    // Disable all interrupts
    dev->idr = 0xFFFFFFFF;

    
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    // Start receiver
    avr32_chan->curbuf = 0;
    avr32_chan->pdca_tx_channel = CYG_PDMA_GET_CHANEL(avr32_chan->dma_tx_ch);
    avr32_chan->pdca_rx_channel = CYG_PDMA_GET_CHANEL(avr32_chan->dma_rx_ch);

    CYG_PDAM_SET_CHANEL(avr32_chan->pdca_rx_channel,
                        avr32_chan->rcv_buffer[0],
                        avr32_chan->ping_pong_size - 1,
                        avr32_chan->dma_pid_rx,
                        avr32_chan->rcv_buffer[1],
                        avr32_chan->ping_pong_size - 1,
                        0);


    dev->rtor = RCV_TIMEOUT;
#ifdef CYGINT_IO_SERIAL_LINE_STATUS_HW
    dev->ier  = /*AVR32_USART_RXRDY_MASK |*/ AVR32_USART_OVER_MASK |
                  AVR32_USART_FRAME_MASK | AVR32_USART_PARE_MASK   |
                  AVR32_USART_RXBRK_MASK | AVR32_USART_TIMEOUT_MASK;
   #ifdef CYGOPT_IO_SERIAL_FLOW_CONTROL_HW
      dev->ier = AVR32_USART_CTSIC_MASK | AVR32_USART_DCDIC_MASK |
                 AVR32_USART_DSRIC_MASK | AVR32_USART_RIIC_MASK  |
                 AVR32_USART_TIMEOUT_MASK;
   #endif
#else
    dev->ier  = /*AVR32_USART_RXRDY_MASK |*/ AVR32_USART_TIMEOUT_MASK;
#endif

    CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_rx_channel);
    
    // Enable RX and TX
    dev->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK |
              AVR32_USART_RTSEN_MASK   | AVR32_USART_STTTO_MASK;
				  
    CYG_PDMA_ENABLE_INTERRUPT(avr32_chan->pdca_rx_channel,false,
                                false, true);
#else
    dev->rtor = RCV_TIMEOUT;
    #ifdef CYGINT_IO_SERIAL_LINE_STATUS_HW
        dev->ier  =   AVR32_USART_RXRDY_MASK | AVR32_USART_OVER_MASK |
                      AVR32_USART_FRAME_MASK | AVR32_USART_PARE_MASK |
                      AVR32_USART_RXBRK_MASK | AVR32_USART_TIMEOUT_MASK;
       #ifdef CYGOPT_IO_SERIAL_FLOW_CONTROL_HW
           dev->ier = AVR32_USART_CTSIC_MASK | AVR32_USART_DCDIC_MASK |
                      AVR32_USART_DSRIC_MASK | AVR32_USART_RIIC_MASK  |
                      AVR32_USART_TIMEOUT_MASK;
       #endif
    #else
        dev->ier  = AVR32_USART_RXRDY_MASK | AVR32_USART_TIMEOUT_MASK;
    #endif

    // Enable RX and TX
    dev->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK |
              AVR32_USART_RTSEN_MASK   | AVR32_USART_STTTO_MASK;

#endif           
    if (new_config != &chan->config) {
        chan->config = *new_config;
    }

    return true;
}

// Function to initialize the device.  Called at bootstrap time.
static bool 
avr32_serial_init(struct cyg_devtab_entry *tab)
{
    serial_channel * const chan = (serial_channel *) tab->priv;
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    int res;

#ifdef CYGDBG_IO_INIT
    diag_printf("AVR32 SERIAL init - dev: %x.%d\n", avr32_chan->usart_dev, avr32_chan->int_num);
#endif
   
    avr32_chan->flags = SIFLG_NONE;
    avr32_chan->stat  = 0;
    (chan->callbacks->serial_init)(chan);  // Really only required for interrupt driven devices
    if (chan->out_cbuf.len != 0) {
        //serial interrupt
        cyg_drv_interrupt_create(avr32_chan->int_num,
                                 4,                      // Priority
                                 (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                                 avr32_serial_ISR,
                                 avr32_serial_DSR,
                                 &avr32_chan->serial_interrupt_handle,
                                 &avr32_chan->serial_interrupt);
        cyg_drv_interrupt_attach(avr32_chan->serial_interrupt_handle);
 #ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER   
        //rx pdma interrupt
        cyg_drv_interrupt_create(avr32_chan->int_num_dma_rx,
                                 4,                      // Priority
                                 (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                                 avr32_serial_ISR,
                                 avr32_serial_DSR,
                                 &avr32_chan->serial_dma_rx_interrupt_handle,
                                 &avr32_chan->serial_dma_rx_interrupt);
        cyg_drv_interrupt_attach(avr32_chan->serial_dma_rx_interrupt_handle);

        //tx pdma interrupt
        cyg_drv_interrupt_create(avr32_chan->int_num_dma_tx,
                                 4,                      // Priority
                                 (cyg_addrword_t)chan,   // Data item passed to interrupt handler
                                 avr32_serial_ISR,
                                 avr32_serial_DSR,
                                 &avr32_chan->serial_dma_tx_interrupt_handle,
                                 &avr32_chan->serial_dma_tx_interrupt);
        cyg_drv_interrupt_attach(avr32_chan->serial_dma_tx_interrupt_handle);
#endif      

    }
    res = avr32_serial_config_port(chan, &chan->config, true);
    return res;
}

// This routine is called when the device is "looked" up (i.e. attached)
static Cyg_ErrNo 
avr32_serial_lookup(struct cyg_devtab_entry **tab, 
                  struct cyg_devtab_entry *sub_tab,
                  const char *name)
{
    serial_channel * const chan = (serial_channel *) (*tab)->priv;
    
    (chan->callbacks->serial_init)(chan);  // Really only required for interrupt driven devices
    return ENOERR;
}

// Send a character to the device output buffer.
// Return 'true' if character is sent to device
static bool
avr32_serial_putc_interrupt(serial_channel *chan, unsigned char c)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    const bool res = (avr32_chan->flags & SIFLG_XMIT_BUSY) == 0;
    
    if (res)
    {
        volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
        dev->thr = c;
        avr32_chan->flags |= SIFLG_XMIT_BUSY;
    }
    return res;
}

#if (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL0) && CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL1) && CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL2) && CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL3) && CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE == 0)
static bool
avr32_serial_putc_polled(serial_channel *chan, unsigned char c)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    
    while (!dev->CSR.usart_mode.txrdy)
        CYG_EMPTY_STATEMENT;
    dev->thr = c;
    return true;
}
#endif

// Fetch a character from the device input buffer, waiting if necessary
static unsigned char 
avr32_serial_getc_interrupt(serial_channel *chan)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    CYG_WORD32 c;

    // Read data
    c = dev->rhr;
    return (unsigned char) c;
}

#if (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL0) && CYGNUM_IO_SERIAL_AVR32_SERIAL0_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL1) && CYGNUM_IO_SERIAL_AVR32_SERIAL1_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL2) && CYGNUM_IO_SERIAL_AVR32_SERIAL2_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL3) && CYGNUM_IO_SERIAL_AVR32_SERIAL3_BUFSIZE == 0) \
 || (defined(CYGPKG_IO_SERIAL_AVR32_SERIAL4) && CYGNUM_IO_SERIAL_AVR32_SERIAL4_BUFSIZE == 0)
static unsigned char 
avr32_serial_getc_polled(serial_channel *chan)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    CYG_WORD32 c;

    while (!dev->CSR.usart_mode.rxrdy)
        CYG_EMPTY_STATEMENT;
    // Read data
    c = dev->rhr;
    return (unsigned char) c;
}
#endif
// Set up the device characteristics; baud rate, etc.
static Cyg_ErrNo
avr32_serial_set_config(serial_channel *chan, cyg_uint32 key,
                       const void *xbuf, cyg_uint32 *len)
{
    switch (key) {
    case CYG_IO_SET_CONFIG_SERIAL_INFO:
    {
        cyg_serial_info_t *config = (cyg_serial_info_t *)xbuf;
        if ( *len < sizeof(cyg_serial_info_t) ) {
            return -EINVAL;
        }
        *len = sizeof(cyg_serial_info_t);
        if ( true != avr32_serial_config_port(chan, config, false) )
            return -EINVAL;
    }
    break;
    case CYGOPT_IO_SERIAL_TIMEOUT:
    {
        avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
        volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
        cyg_uint32 timeout = *((cyg_uint32 *)xbuf);
        if ( *len < sizeof(cyg_uint32) )
            return -EINVAL;

        dev->rtor = timeout;
        dev->cr   = AVR32_USART_RETTO_MASK;
    }
      break;
#ifdef CYGOPT_IO_SERIAL_FLOW_CONTROL_HW
    case CYG_IO_SET_CONFIG_SERIAL_HW_RX_FLOW_THROTTLE:
    {
        // RX flow control involves just the RTS line. Most of the
        // work is done by the hardware depending on the state of
        // the fifo. This option serves mainly to drop RTS if
        // higher-level code is running out of buffer space, even
        // if the fifo is not yet full.
        avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
        volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
        cyg_uint32 *f = (cyg_uint32 *)xbuf;
        unsigned char mask=0;
        if ( *len < sizeof(*f) )
            return -EINVAL;

        if ( chan->config.flags & CYGNUM_SERIAL_FLOW_RTSCTS_RX )
            mask = AVR32_USART_RTSDIS_MASK;
        else
            mask = AVR32_USART_RTSEN_MASK;
        if ( chan->config.flags & CYGNUM_SERIAL_FLOW_DSRDTR_RX )
            mask |= AVR32_USART_DTRDIS_MASK;
        else
            mask |= AVR32_USART_DTREN_MASK;

        dev->cr = mask; 
    }
        break;
        
    case CYG_IO_SET_CONFIG_SERIAL_HW_FLOW_CONFIG:
      break;
#endif
    default:
        return -EINVAL;
    }
    return ENOERR;
}

// Enable the transmitter on the device
static void
avr32_serial_start_xmit(serial_channel *chan)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    unsigned char * chars;
    xmt_req_reply_t res;
    //cyg_drv_dsr_lock();
    // wait until CTS is inactive 
    while(dev->csr & AVR32_USART_CSR_CTS_MASK)
    {
      hal_delay_us(10);
    }
    if ((avr32_chan->flags & SIFLG_XMIT_CONTINUE) == 0) {
        res = (chan->callbacks->data_xmt_req)(chan, 0xffff, &avr32_chan->transmit_size, &chars);
        switch (res)
        {
            case CYG_XMT_OK:
                CYG_PDAM_SET_CHANEL(avr32_chan->pdca_tx_channel,chars,
                  avr32_chan->transmit_size,avr32_chan->dma_pid_tx,0,0,0);
                CYG_PDMA_ENABLE_INTERRUPT(avr32_chan->pdca_tx_channel,true,
                  true, false);
                CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_tx_channel);

                avr32_chan->flags |= SIFLG_XMIT_CONTINUE;

                break;
            case CYG_XMT_DISABLED:
                (chan->callbacks->xmt_char)(chan);  // Kick transmitter
                avr32_chan->flags |= SIFLG_XMIT_CONTINUE;
                dev->idr = AVR32_USART_CSR_TXRDY_MASK;
                break;
            default:
                // No data or unknown error - can't do anything about it
                break;
        }
    }
    //cyg_drv_dsr_unlock();
#else
	//cyg_drv_dsr_lock();
	avr32_chan->flags |= SIFLG_XMIT_CONTINUE;
	dev->ier           = AVR32_USART_CSR_TXRDY_MASK;
	//cyg_drv_dsr_unlock();
#endif
}

// Disable the transmitter on the device
static void 
avr32_serial_stop_xmit(serial_channel *chan)
{
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    CYG_PDMA_DISABLE_CHANEL(avr32_chan->pdca_tx_channel);
    dev->idr = AVR32_USART_CSR_TXRDY_MASK;
    avr32_chan->flags &= ~SIFLG_XMIT_CONTINUE;	
}

// Serial I/O - low level interrupt handler (ISR)
static void       
avr32_serial_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    serial_channel *chan = (serial_channel *)data;
    avr32_serial_info * const avr32_chan = (avr32_serial_info *)chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyg_uint32 _csr, _ier = 0;

    _csr = dev->csr;

#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    //If some input data transfer is ongoing between ecos initialization
    //and channel opening the ping-pong buffers can be full witch can
    //produce some errors
    if(avr32_chan->trans_rdy)
    {
        const cyg_uint8 cb = avr32_chan->curbuf, nb = cb ^ 0x01;
        cyg_uint8 * p = avr32_chan->rcv_buffer[cb], * end = avr32_chan->end;
        
        CYG_ASSERT((end - p) <= avr32_chan->ping_pong_size ,
                "Transfer is longer than buffer dma end\n");

        while (p < end) 
        {
            rcv_req_reply_t res;
            int space_avail;
            unsigned char *space;

            res = (chan->callbacks->data_rcv_req)(
              chan,
              end - avr32_chan->rcv_buffer[cb],
              &space_avail,
              &space
            );

            switch (res)
            {
                case CYG_RCV_OK:
                    memcpy(space, p, space_avail);
                    (chan->callbacks->data_rcv_done)(chan, space_avail);
                    p += space_avail;
                    break;
                case CYG_RCV_DISABLED:
                    (chan->callbacks->rcv_char)(chan, *p++);
                    break;
                default:
                    // Buffer full or unknown error, can't do anything about it
                    // Discard data
                    p = end;
                    break;
            }      
        }

        avr32_chan->trans_rdy = false;
        avr32_chan->curbuf = nb;
    }
	
#else
    if(_csr & AVR32_USART_CSR_RXRDY_MASK)
    {
        unsigned char c;
        c = dev->rhr;
        (chan->callbacks->rcv_char)(chan, c);
        _ier |= AVR32_USART_IER_RXRDY_MASK;
    }		  
#endif

#ifdef CYGINT_IO_SERIAL_BLOCK_TRANSFER
    if (CYG_PDMA_IS_TRANSFER_COMPLETE(avr32_chan->pdca_tx_channel) &&
	    CYG_PDMA_IS_ENABLED(avr32_chan->pdca_tx_channel)) 
    {
        (chan->callbacks->data_xmt_done)(chan, avr32_chan->transmit_size);
        if (avr32_chan->flags & SIFLG_XMIT_CONTINUE) {
            unsigned char * chars;
            xmt_req_reply_t res;

            res = (chan->callbacks->data_xmt_req)(
                    chan, 0xffff, &avr32_chan->transmit_size, &chars);

            switch (res)
            {
                case CYG_XMT_OK:
                    CYG_PDAM_SET_CHANEL(avr32_chan->pdca_tx_channel,chars,
                      avr32_chan->transmit_size,avr32_chan->dma_pid_tx,0,0,0);
                    CYG_PDMA_ENABLE_INTERRUPT(avr32_chan->pdca_tx_channel,true,
                      true, false);
                    CYG_PDMA_ENABLE_CHANEL(avr32_chan->pdca_tx_channel);

                    avr32_chan->flags |= SIFLG_XMIT_CONTINUE;
                    break;
                default:
                    // No data or unknown error - can't do anything about it
                    // CYG_XMT_DISABLED should not happen here!
                    break;
            }
        }
    }
#else
    if(_csr & AVR32_USART_CSR_TXRDY_MASK)  
    {
        avr32_chan->flags &= ~SIFLG_XMIT_BUSY;  
        (chan->callbacks->xmt_char)(chan);
        _ier |= AVR32_USART_IER_TXRDY_MASK;
    }
#endif
#ifdef CYGINT_IO_SERIAL_LINE_STATUS_HW
    if(_csr & AVR32_USART_CSR_OVRE_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_OVERRUNERR;
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_OVRE_MASK;
    }
    if(_csr & AVR32_USART_CSR_FRAME_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_FRAMEERR;
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_FRAME_MASK;
    }

    if(_csr & AVR32_USART_CSR_PARE_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_FRAMEERR;
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_PARE_MASK;
    }
    if(_csr & AVR32_USART_CSR_RXBRK_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_BREAK;
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_RXBRK_MASK;
    }
#ifdef CYGOPT_IO_SERIAL_FLOW_CONTROL_HW
    if(_csr & AVR32_USART_CSR_CTSIC_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_FLOW;
        stat.value = (0 != (dev->CSR.usart_mode.cts));
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_CTSIC_MASK;
    }
    if(_csr & AVR32_USART_CSR_DCDIC_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_CARRIERDETECT;
        stat.value = (0 != (dev->CSR.usart_mode.dcd));
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_DCDIC_MASK;
    }
    if(_csr & AVR32_USART_CSR_DSRIC_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_FLOW;
        stat.value = (0 != (dev->CSR.usart_mode.dsr));
        (chan->callbacks->indicate_status)(chan, &stat );
        _ier |= AVR32_USART_IER_DSRIC_MASK;
    }
    if(_csr & AVR32_USART_CSR_RIIC_MASK)
    {
        cyg_serial_line_status_t stat;
        stat.which = CYGNUM_SERIAL_STATUS_RINGINDICATOR;
        stat.value = (0 != (dev->CSR.usart_mode.ri));
        _ier |= AVR32_USART_IER_RIIC_MASK;
    }
    
#endif
#endif
    dev->cr = AVR32_USART_CR_RSTSTA_MASK;
    dev->ier = _ier;   
}


// Serial I/O - high level interrupt handler (DSR)
static cyg_uint32
avr32_serial_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    serial_channel * const chan = (serial_channel *) data;
    avr32_serial_info * const avr32_chan = (avr32_serial_info *) chan->dev_priv;
    volatile avr32_usart_t  *dev = avr32_chan->usart_dev;
    cyg_uint32 ret = CYG_ISR_HANDLED;
    // Check if we have an interrupt pending - note that the interrupt
    // is pending of the low bit of the isr is *0*, not 1.
    
    dev->idr = dev->csr & dev->imr;
    
    if(dev->csr & AVR32_USART_CSR_TIMEOUT_MASK)
    {
       // set data buffer length
       const cyg_uint8 cb = avr32_chan->curbuf;
       if(avr32_chan->rcv_buffer[cb] != avr32_chan->pdca_rx_channel->mar)
       {
            avr32_chan->end = avr32_chan->pdca_rx_channel->mar;
            CYG_PDMA_FORCE_RELOAD(avr32_chan->pdca_rx_channel);
            CYG_PDMA_RELOAD(avr32_chan->pdca_rx_channel,avr32_chan->rcv_buffer[cb],
				avr32_chan->ping_pong_size - 1);
            avr32_chan->trans_rdy = true;
            ret |= CYG_ISR_CALL_DSR;
       }
    }
    
    
    if(CYG_PDMA_INTERRUPT_STATUS(avr32_chan->pdca_rx_channel))
    {
        // CYG_PDMA_DISABLE_INTERRUPT(avr32_chan->pdca_rx_channel);
        const cyg_uint8 cb = avr32_chan->curbuf;
        avr32_chan->end = avr32_chan->rcv_buffer[cb] + avr32_chan->ping_pong_size;
        CYG_PDMA_RELOAD(avr32_chan->pdca_rx_channel,avr32_chan->rcv_buffer[cb],
                             avr32_chan->ping_pong_size - 1);
         avr32_chan->trans_rdy = true;
         dev->cr = AVR32_USART_STTTO_MASK | AVR32_USART_RETTO_MASK;
         dev->ier = AVR32_USART_IER_TIMEOUT_MASK;
         ret |= CYG_ISR_CALL_DSR;
    }

    if(CYG_PDMA_IS_ENABLED(avr32_chan->pdca_tx_channel))
    {
      if(CYG_PDMA_INTERRUPT_STATUS(avr32_chan->pdca_tx_channel))
      {
            CYG_PDMA_DISABLE_INTERRUPT(avr32_chan->pdca_tx_channel);
            ret |= CYG_ISR_CALL_DSR;
      }
    }

    dev->cr = AVR32_USART_STTTO_MASK | AVR32_USART_RETTO_MASK;
    dev->ier = AVR32_USART_IER_TIMEOUT_MASK;  
    return ret;
}
#endif

