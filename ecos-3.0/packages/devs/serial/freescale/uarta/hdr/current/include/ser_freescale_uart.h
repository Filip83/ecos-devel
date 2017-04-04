#ifndef CYGONCE_DEVS_SERIAL_FREESCALE_UART_H
#define CYGONCE_DEVS_SERIAL_FREESCALE_UART_H
//==========================================================================
//
//      ser_freescale_uart.h
//
//      Freescale UART I/O definitions.
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
// Author(s):   Mike Jones <mjones@proclivis.com)
// Contributors:Ilija Kocho <ilijak@siva.com.mk>
// Date:        2011-02-05
// Purpose:     Freescale UART I/O definitions.
// Description:
//
//
//####DESCRIPTIONEND####
//==========================================================================

enum {
    CYGHWR_DEV_FREESCALE_UART_URXD = 0x00,       // UART Receiver Register
    CYGHWR_DEV_FREESCALE_UART_UTXD = 0x40,       // UART Transmitter Register
    CYGHWR_DEV_FREESCALE_UART_UCR1 = 0x80,       // UART Control Register 1
    CYGHWR_DEV_FREESCALE_UART_UCR2 = 0x84,       // UART Control Register 2
    CYGHWR_DEV_FREESCALE_UART_UCR3 = 0x88,       // UART Control Register 3
    CYGHWR_DEV_FREESCALE_UART_UCR4 = 0x8C,       // UART Control Register 4
    CYGHWR_DEV_FREESCALE_UART_UFCR = 0x90,       // UART FIFO Control Register
    CYGHWR_DEV_FREESCALE_UART_USR1 = 0x94,       // UART Status Register 1
    CYGHWR_DEV_FREESCALE_UART_USR2 = 0x98,       // UART Status Register 2
    CYGHWR_DEV_FREESCALE_UART_UESC = 0x9C,       // UART Escape Character Register
    CYGHWR_DEV_FREESCALE_UART_UTIM = 0xA0,       // Escape Timer Register
    CYGHWR_DEV_FREESCALE_UART_UBIR = 0xA4,       // UART FBRM Incremental Register
    CYGHWR_DEV_FREESCALE_UART_UBMR = 0xA8,       // UART BRM Modulator Register
    CYGHWR_DEV_FREESCALE_UART_UBRC = 0xAC,       // UART Baud Rate Count Register
    CYGHWR_DEV_FREESCALE_UART_ONEMS = 0xB0,      // UART One Millisecond Register
    CYGHWR_DEV_FREESCALE_UART_UTS = 0xB4,        // UART Test Register
    CYGHWR_DEV_FREESCALE_UART_UMCR = 0xB8        // UART RS-485 Mode Control Register

};

// CYGHWR_IO_FREESCALE_UART_BAUD_SET(__uart_p, _baud_) should be provided by HAL.
// CYGHWR_IO_FREESCALE_UART_PIN(__pin) should be provided by HAL.
// CYGHWR_IO_FREESCALE_UARTn_CLOCK should be provided by HAL.
// CYGHWR_IO_FREESCALE_UARTn_PIN_RX should be provided by HAL.
// CYGHWR_IO_FREESCALE_UARTn_PIN_TX should be provided by HAL.
// CYGHWR_IO_FREESCALE_UARTn_PIN_RTS should be provided by HAL.
// CYGHWR_IO_FREESCALE_UARTn_PIN_CTS should be provided by HAL.


#define CYGHWR_DEV_FREESCALE_UART_URXD_CHARRDY     (0x8000)
#define CYGHWR_DEV_FREESCALE_UART_URXD_ERR         (0x4000)
#define CYGHWR_DEV_FREESCALE_UART_URXD_OVRRUN      (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_URXD_FRMERR      (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_URXD_BRK         (0x0800)
#define CYGHWR_DEV_FREESCALE_UART_URXD_PRERR       (0x0400)

#define CYGHWR_DEV_FREESCALE_UART_UCR1_ADEN        (0x8000)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_ADBR        (0x4000)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_TRDYEN      (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_IDEN        (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_ICD         (0x0C00)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_RRDYEN      (0x0200)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_RXDMAEN     (0x0100)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_IREN        (0x0080)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_TXMPTYEN    (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_RTSDEN      (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_SNDBRK      (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_TXDMAEN     (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_ATDMAEN     (0x0004)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_DOZE        (0x0002)
#define CYGHWR_DEV_FREESCALE_UART_UCR1_UARTEN      (0x0001)

#define CYGHWR_DEV_FREESCALE_UART_UCR2_ESCI        (0x8000)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_IRTS        (0x4000)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_CTSC        (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_CTS         (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_ESCEN       (0x0800)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_RTEC        (0x0600)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_PREN        (0x0100)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_PROE        (0x0080)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_STPB        (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_WS          (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_RTSEN       (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_ATEN        (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_TXEN        (0x0004)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_RXEN        (0x0002)
#define CYGHWR_DEV_FREESCALE_UART_UCR2_SRST        (0x0001)

#define CYGHWR_DEV_FREESCALE_UART_UCR3_DPEC        (0xC000)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_DTREN       (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_PARERREN    (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_FRAERREN    (0x0800)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_DSR         (0x0400)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_DCD         (0x0200)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_RI          (0x0100)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_ADNIMP      (0x0080)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_RXDSEN      (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_AIRENTEN    (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_AWAKEN      (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_DTRDEN      (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_RXDMUXSEL   (0x0004)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_INVT        (0x0002)
#define CYGHWR_DEV_FREESCALE_UART_UCR3_ACIEN       (0x0001)

#define CYGHWR_DEV_FREESCALE_UART_UCR4_CTSTL       (0xFC00)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_ENVR        (0x0200)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_ENIRI       (0x0100)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_WKEN        (0x0080)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_IDDMAEN     (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_IRSC        (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_LPBYP       (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_TCEN        (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_BKEN        (0x0004)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_OREN        (0x0002)
#define CYGHWR_DEV_FREESCALE_UART_UCR4_DREN        (0x0001)

#define CYGHWR_DEV_FREESCALE_UART_UFCR_TXTL        (0xFC00)
#define CYGHWR_DEV_FREESCALE_UART_UFCR_RFDIV       (0x0380)
#define CYGHWR_DEV_FREESCALE_UART_UFCR_DCEDTE      (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_UFCR_RXTL        (0x003F)

#define CYGHWR_DEV_FREESCALE_UART_USR1_PARITYERR   (0x8000)
#define CYGHWR_DEV_FREESCALE_UART_USR1_RTSS        (0x4000)
#define CYGHWR_DEV_FREESCALE_UART_USR1_TRDY        (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_USR1_RTSD        (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_USR1_ESCF        (0x0800)
#define CYGHWR_DEV_FREESCALE_UART_USR1_FRAMERR     (0x0400)
#define CYGHWR_DEV_FREESCALE_UART_USR1_RRDY        (0x0200)
#define CYGHWR_DEV_FREESCALE_UART_USR1_AGTIM       (0x0100)
#define CYGHWR_DEV_FREESCALE_UART_USR1_RDTD        (0x0080)
#define CYGHWR_DEV_FREESCALE_UART_USR1_RXDS        (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_USR1_AIRINT      (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_USR1_AWAKE       (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_USR1_SAD         (0x0008)

#define CYGHWR_DEV_FREESCALE_UART_USR2_ADET        (0x8000)
#define CYGHWR_DEV_FREESCALE_UART_USR2_TXFE        (0x4000)
#define CYGHWR_DEV_FREESCALE_UART_USR2_DTRF        (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_USR2_IDLE        (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_USR2_ACST        (0x0800)
#define CYGHWR_DEV_FREESCALE_UART_USR2_RIDELT      (0x0400)
#define CYGHWR_DEV_FREESCALE_UART_USR2_RIIN        (0x0200)
#define CYGHWR_DEV_FREESCALE_UART_USR2_IRINT       (0x0100)
#define CYGHWR_DEV_FREESCALE_UART_USR2_WAKE        (0x0080)
#define CYGHWR_DEV_FREESCALE_UART_USR2_DCDELT      (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_USR2_DCDIN       (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_USR2_RTSF        (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_USR2_TXDC        (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_USR2_BRCD        (0x0004)
#define CYGHWR_DEV_FREESCALE_UART_USR2_ORE         (0x0002)
#define CYGHWR_DEV_FREESCALE_UART_USR2_RDR         (0x0001)

#define CYGHWR_DEV_FREESCALE_UART_UTS_FRCPERR      (0x2000)
#define CYGHWR_DEV_FREESCALE_UART_UTS_LOOP         (0x1000)
#define CYGHWR_DEV_FREESCALE_UART_UTS_DBGEN        (0x0800)
#define CYGHWR_DEV_FREESCALE_UART_UTS_LOOPIR       (0x0400)
#define CYGHWR_DEV_FREESCALE_UART_UTS_RXDBG        (0x0200)
#define CYGHWR_DEV_FREESCALE_UART_UTS_TXEMPTY      (0x0040)
#define CYGHWR_DEV_FREESCALE_UART_UTS_RXEMPTY      (0x0020)
#define CYGHWR_DEV_FREESCALE_UART_UTS_TXFULL       (0x0010)
#define CYGHWR_DEV_FREESCALE_UART_UTS_RXFULL       (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_UTS_SOFTRST      (0x0001)

#define CYGHWR_DEV_FREESCALE_UART_UMCR_SADEN       (0x0008)
#define CYGHWR_DEV_FREESCALE_UART_UMCR_TXB8        (0x0004)
#define CYGHWR_DEV_FREESCALE_UART_UMCR_SLAM        (0x0002)
#define CYGHWR_DEV_FREESCALE_UART_UMCR_MDEN        (0x0001)

#define UART_MODULE_CLK(x) ((x) == HW_UART1 ? UART1_MODULE_CLK : (x) == HW_UART2 ? UART2_MODULE_CLK : (x) == HW_UART3 ? UART3_MODULE_CLK : (x) == HW_UART4 ? UART4_MODULE_CLK : -1)

/*!
 * @brief   Obtain UART reference frequency
 *
 * @param   instance the UART instance number.
 * @return  reference frequency in Hz
 */
cyg_uint32 uart_get_reffreq(cyg_uint32 instance);


#endif // CYGONCE_DEVS_SERIAL_FREESCALE_UART_H
