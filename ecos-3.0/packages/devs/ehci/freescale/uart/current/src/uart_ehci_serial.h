#ifndef CYGONCE_AVR32_SERIAL_H
#define CYGONCE_AVR32_SERIAL_H

// ====================================================================
//
//      avr32_serial.h
//
//      Device I/O - Description of Atmel AVR32UC3C serial hardware
//
// ====================================================================
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
// ====================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Filip
// Contributors: 
// Date:         2012-11-24
// Purpose:      Internal interfaces for serial I/O drivers
// Description:
//
//####DESCRIPTIONEND####
//
// ====================================================================

// Description of serial ports on Atmel AVR32UC3C

#define CYGOPT_IO_SERIAL_TIMEOUT      0x0182

static const cyg_uint32 select_word_length[] = {
    (AVR32_USART_CHRL_5 << AVR32_USART_CHRL_OFFSET),
    (AVR32_USART_CHRL_6 << AVR32_USART_CHRL_OFFSET),
    (AVR32_USART_CHRL_7 << AVR32_USART_CHRL_OFFSET),
    (AVR32_USART_CHRL_8 << AVR32_USART_CHRL_OFFSET)
};

static const cyg_uint32 select_stop_bits[] = {
    (AVR32_USART_NBSTOP_1 << AVR32_USART_NBSTOP_OFFSET),     // 1 stop bit
    (AVR32_USART_NBSTOP_1_5 << AVR32_USART_NBSTOP_OFFSET),   // 1.5 stop bit
    (AVR32_USART_NBSTOP_2 << AVR32_USART_NBSTOP_OFFSET),     // 2 stop bits
};

static const cyg_uint32 select_parity[] = {
    (AVR32_USART_PAR_NONE << AVR32_USART_PAR_OFFSET),  // No parity
    (AVR32_USART_PAR_EVEN << AVR32_USART_PAR_OFFSET),  // Even parity
    (AVR32_USART_PAR_ODD << AVR32_USART_PAR_OFFSET),   // Odd parity
    (AVR32_USART_PAR_MARK << AVR32_USART_PAR_OFFSET),  // Mark (1) parity
    (AVR32_USART_PAR_SPACE << AVR32_USART_PAR_OFFSET)  // Space (0) parity
};

// Assume the UART is driven 1/16 CPU frequency
#define UART_CLOCK    ((CYGHWR_HAL_AVR32_CPU_FREQ)*1.0e6)

#define DIVISOR(baud) ((int)((UART_CLOCK)/(16.*baud)+0.5))

static const cyg_int32 select_baud[] = {
    0,      // Unused
    DIVISOR(50),     // 50
    DIVISOR(75),     // 75
    DIVISOR(110),    // 110
    DIVISOR(134.5),  // 134.5
    DIVISOR(150),    // 150
    DIVISOR(200),    // 200
    DIVISOR(300),    // 300
    DIVISOR(600),    // 600
    DIVISOR(1200),   // 1200
    DIVISOR(1800),   // 1800
    DIVISOR(2400),   // 2400
    DIVISOR(3600),   // 3600
    DIVISOR(4800),   // 4800
    DIVISOR(7200),   // 7200
    DIVISOR(9600),   // 9600
    DIVISOR(14400),  // 14400
    DIVISOR(19200),  // 19200
    DIVISOR(38400),  // 38400
    DIVISOR(57600),  // 57600
    DIVISOR(115200), // 115200
    DIVISOR(230400), // 230400
};

#endif // CYGONCE_AVR32_SERIAL_H
