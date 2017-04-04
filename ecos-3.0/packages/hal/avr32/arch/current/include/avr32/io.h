/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Standard part header file.
 *
 * This file includes the appropriate part header file according to the defined
 * MCU.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr32@atmel.com
 *
 ******************************************************************************/

/* Copyright (c) 2007-2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _IO_H_
#define _IO_H_

#if (defined __GNUC__)
#if (1 == 0)
#elif (defined __AVR32_7000__)
#include <cyg/hal/avr32/ap7000.h>
#elif (defined __AVR32_7001__)
#include <cyg/hal/avr32/ap7001.h>
#elif (defined __AVR32_7002__)
#include <cyg/hal/avr32/ap7002.h>
#elif (defined __AVR32_7200__)
#include <cyg/hal/avr32/ap7200.h>
#elif (defined __AVR32_UC3A0128__)
#include <cyg/hal/avr32/uc3a0128.h>
#elif (defined __AVR32_UC3A0256__)
#include <cyg/hal/avr32/uc3a0256.h>
#elif (defined __AVR32_UC3A0512__)
#include <cyg/hal/avr32/uc3a0512.h>
#elif (defined __AVR32_UC3A1512__)
#include <cyg/hal/avr32/uc3a1512.h>
#elif (defined __AVR32_UC3A1256__)
#include <cyg/hal/avr32/uc3a1256.h>
#elif (defined __AVR32_UC3A1128__)
#include <cyg/hal/avr32/uc3a1128.h>
#elif (defined __AVR32_UC3A364__)
#include <cyg/hal/avr32/uc3a364.h>
#elif (defined __AVR32_UC3A3128__)
#include <cyg/hal/avr32/uc3a3128.h>
#elif (defined __AVR32_UC3A3256__)
#include <cyg/hal/avr32/uc3a3256.h>
#elif (defined __AVR32_UC3A364S__)
#include <cyg/hal/avr32/uc3a364s.h>
#elif (defined __AVR32_UC3A3128S__)
#include <cyg/hal/avr32/uc3a3128s.h>
#elif (defined __AVR32_UC3A3256S__)
#include <cyg/hal/avr32/uc3a3256s.h>
#elif (defined __AVR32_UC3A464__)
#include <cyg/hal/avr32/uc3a464.h>
#elif (defined __AVR32_UC3A4128__)
#include <cyg/hal/avr32/uc3a4128.h>
#elif (defined __AVR32_UC3A4256__)
#include <cyg/hal/avr32/uc3a4256.h>
#elif (defined __AVR32_UC3A464S__)
#include <cyg/hal/avr32/uc3a464s.h>
#elif (defined __AVR32_UC3A4128S__)
#include <cyg/hal/avr32/uc3a4128s.h>
#elif (defined __AVR32_UC3A4256S__)
#include <cyg/hal/avr32/uc3a4256s.h>
#elif (defined __AVR32_UC3B064__)
#include <cyg/hal/avr32/uc3b064.h>
#elif (defined __AVR32_UC3B0128__)
#include <cyg/hal/avr32/uc3b0128.h>
#elif (defined __AVR32_UC3B0256__)
#include <cyg/hal/avr32/uc3b0256.h>
#elif (defined __AVR32_UC3B0512__)
#include <cyg/hal/avr32/uc3b0512.h>
#elif (defined __AVR32_UC3B164__)
#include <cyg/hal/avr32/uc3b164.h>
#elif (defined __AVR32_UC3B1128__)
#include <cyg/hal/avr32/uc3b1128.h>
#elif (defined __AVR32_UC3B1256__)
#include <cyg/hal/avr32/uc3b1256.h>
#elif (defined __AVR32_UC3B1512__)
#include <cyg/hal/avr32/uc3b1512.h>
#elif (defined __AVR32_UC3C0512C__)
#include <cyg/hal/avr32/uc3c0512c.h>
#elif (defined __AVR32_UC3C0256C__)
#include <cyg/hal/avr32/uc3c0256c.h>
#elif (defined __AVR32_UC3C0128C__)
#include <cyg/hal/avr32/uc3c0128c.h>
#elif (defined __AVR32_UC3C064C__)
#include <cyg/hal/avr32/uc3c064c.h>
#elif (defined __AVR32_UC3C1512C__)
#include <cyg/hal/avr32/uc3c1512c.h>
#elif (defined __AVR32_UC3C1256C__)
#include <cyg/hal/avr32/uc3c1256c.h>
#elif (defined __AVR32_UC3C1128C__)
#include <cyg/hal/avr32/uc3c1128c.h>
#elif (defined __AVR32_UC3C164C__)
#include <cyg/hal/avr32/uc3c164c.h>
#elif (defined __AVR32_UC3C2512C__)
#include <cyg/hal/avr32/uc3c2512c.h>
#elif (defined __AVR32_UC3C2256C__)
#include <cyg/hal/avr32/uc3c2256c.h>
#elif (defined __AVR32_UC3C2128C__)
#include <cyg/hal/avr32/uc3c2128c.h>
#elif (defined __AVR32_UC3C264C__)
#include <cyg/hal/avr32/uc3c264c.h>
#elif (defined __AVR32_UC64D3__)
#include <cyg/hal/avr32/uc64d3.h>
#elif (defined __AVR32_UC128D3__)
#include <cyg/hal/avr32/uc128d3.h>
#elif (defined __AVR32_UC64D4__)
#include <cyg/hal/avr32/uc64d4.h>
#elif (defined __AVR32_UC128D4__)
#include <cyg/hal/avr32/uc128d4.h>
#elif (defined __AVR32_UC3L016__)
#include <cyg/hal/avr32/uc3l016.h>
#elif (defined __AVR32_UC3L032__)
#include <cyg/hal/avr32/uc3l032.h>
#elif (defined __AVR32_UC3L064__)
#include <cyg/hal/avr32/uc3l064.h>
#elif (defined __AVR32_UC3L0256__)
#include <cyg/hal/avr32/uc3l0256.h>
#elif (defined __AVR32_UC3L0128__)
#include <cyg/hal/avr32/uc3l0128.h>
#elif (defined __AVR32_UC256L3U__)
#include <cyg/hal/avr32/uc256l3u.h>
#elif (defined __AVR32_UC128L3U__)
#include <cyg/hal/avr32/uc128l3u.h>
#elif (defined __AVR32_UC64L3U__)
#include <cyg/hal/avr32/uc64l3u.h>
#elif (defined __AVR32_UC256L4U__)
#include <cyg/hal/avr32/uc256l4u.h>
#elif (defined __AVR32_UC128L4U__)
#include <cyg/hal/avr32/uc128l4u.h>
#elif (defined __AVR32_UC64L4U__)
#include <cyg/hal/avr32/uc64l4u.h>
#elif (defined __AVR32_MXT1664S__)
#include <cyg/hal/avr32/mxt1664s.h>
#elif (defined __AVR32_MXT1664ENG__)
#include <cyg/hal/avr32/mxt1664eng.h>
#elif (defined __AVR32_MXT768E__)
#include <cyg/hal/avr32/mxt768e.h>
#elif (defined __AVR32_MXT540E__)
#include <cyg/hal/avr32/mxt540e.h>
#else
#  error No known AVR32 part defined
#endif /* __AVR32_<PART>__ */
#elif (defined __ICCAVR32__) ||(defined __AAVR32__)
#if (1 == 0)
#elif (defined __AT32AP7000__)
#include <cyg/hal/avr32/ioap7000.h>
#elif (defined __AT32AP7001__)
#include <cyg/hal/avr32/ioap7001.h>
#elif (defined __AT32AP7002__)
#include <cyg/hal/avr32/ioap7002.h>
#elif (defined __AT32AP7200__)
#include <cyg/hal/avr32/ioap7200.h>
#elif (defined __AT32UC3A0128__)
#include <cyg/hal/avr32/iouc3a0128.h>
#elif (defined __AT32UC3A0256__)
#include <cyg/hal/avr32/iouc3a0256.h>
#elif (defined __AT32UC3A0512__)
#include <cyg/hal/avr32/iouc3a0512.h>
#elif (defined __AT32UC3A1512__)
#include <cyg/hal/avr32/iouc3a1512.h>
#elif (defined __AT32UC3A1256__)
#include <cyg/hal/avr32/iouc3a1256.h>
#elif (defined __AT32UC3A1128__)
#include <cyg/hal/avr32/iouc3a1128.h>
#elif (defined __AT32UC3A364__)
#include <cyg/hal/avr32/iouc3a364.h>
#elif (defined __AT32UC3A3128__)
#include <cyg/hal/avr32/iouc3a3128.h>
#elif (defined __AT32UC3A3256__)
#include <cyg/hal/avr32/iouc3a3256.h>
#elif (defined __AT32UC3A364S__)
#include <cyg/hal/avr32/iouc3a364s.h>
#elif (defined __AT32UC3A3128S__)
#include <cyg/hal/avr32/iouc3a3128s.h>
#elif (defined __AT32UC3A3256S__)
#include <cyg/hal/avr32/iouc3a3256s.h>
#elif (defined __AT32UC3A464__)
#include <cyg/hal/avr32/iouc3a464.h>
#elif (defined __AT32UC3A4128__)
#include <cyg/hal/avr32/iouc3a4128.h>
#elif (defined __AT32UC3A4256__)
#include <cyg/hal/avr32/iouc3a4256.h>
#elif (defined __AT32UC3A464S__)
#include <cyg/hal/avr32/iouc3a464s.h>
#elif (defined __AT32UC3A4128S__)
#include <cyg/hal/avr32/iouc3a4128s.h>
#elif (defined __AT32UC3A4256S__)
#include <cyg/hal/avr32/iouc3a4256s.h>
#elif (defined __AT32UC3B064__)
#include <cyg/hal/avr32/iouc3b064.h>
#elif (defined __AT32UC3B0128__)
#include <cyg/hal/avr32/iouc3b0128.h>
#elif (defined __AT32UC3B0256__)
#include <cyg/hal/avr32/iouc3b0256.h>
#elif (defined __AT32UC3B0512__)
#include <cyg/hal/avr32/iouc3b0512.h>
#elif (defined __AT32UC3B164__)
#include <cyg/hal/avr32/iouc3b164.h>
#elif (defined __AT32UC3B1128__)
#include <cyg/hal/avr32/iouc3b1128.h>
#elif (defined __AT32UC3B1256__)
#include <cyg/hal/avr32/iouc3b1256.h>
#elif (defined __AT32UC3B1512__)
#include <cyg/hal/avr32/iouc3b1512.h>
#elif (defined __AT32UC3C0512C__)
#include <cyg/hal/avr32/iouc3c0512c.h>
#elif (defined __AT32UC3C0256C__)
#include <cyg/hal/avr32/iouc3c0256c.h>
#elif (defined __AT32UC3C0128C__)
#include <cyg/hal/avr32/iouc3c0128c.h>
#elif (defined __AT32UC3C064C__)
#include <cyg/hal/avr32/iouc3c064c.h>
#elif (defined __AT32UC3C1512C__)
#include <cyg/hal/avr32/iouc3c1512c.h>
#elif (defined __AT32UC3C1256C__)
#include <cyg/hal/avr32/iouc3c1256c.h>
#elif (defined __AT32UC3C1128C__)
#include <cyg/hal/avr32/iouc3c1128c.h>
#elif (defined __AT32UC3C164C__)
#include <cyg/hal/avr32/iouc3c164c.h>
#elif (defined __AT32UC3C2512C__)
#include <cyg/hal/avr32/iouc3c2512c.h>
#elif (defined __AT32UC3C2256C__)
#include <cyg/hal/avr32/iouc3c2256c.h>
#elif (defined __AT32UC3C2128C__)
#include <cyg/hal/avr32/iouc3c2128c.h>
#elif (defined __AT32UC3C264C__)
#include <cyg/hal/avr32/iouc3c264c.h>
#elif (defined __ATUC64D3__)
#include <cyg/hal/avr32/iouc64d3.h>
#elif (defined __ATUC128D3__)
#include <cyg/hal/avr32/iouc128d3.h>
#elif (defined __ATUC64D4__)
#include <cyg/hal/avr32/iouc64d4.h>
#elif (defined __ATUC128D4__)
#include <cyg/hal/avr32/iouc128d4.h>
#elif (defined __AT32UC3L016__)
#include <cyg/hal/avr32/iouc3l016.h>
#elif (defined __AT32UC3L032__)
#include <cyg/hal/avr32/iouc3l032.h>
#elif (defined __AT32UC3L064__)
#include <cyg/hal/avr32/iouc3l064.h>
#elif (defined __AT32UC3L0256__)
#include <cyg/hal/avr32/iouc3l0256.h>
#elif (defined __AT32UC3L0128__)
#include <cyg/hal/avr32/iouc3l0128.h>
#elif (defined __ATUC256L3U__)
#include <cyg/hal/avr32/iouc256l3u.h>
#elif (defined __ATUC128L3U__)
#include <cyg/hal/avr32/iouc128l3u.h>
#elif (defined __ATUC64L3U__)
#include <cyg/hal/avr32/iouc64l3u.h>
#elif (defined __ATUC256L4U__)
#include <cyg/hal/avr32/iouc256l4u.h>
#elif (defined __ATUC128L4U__)
#include <cyg/hal/avr32/iouc128l4u.h>
#elif (defined __ATUC64L4U__)
#include <cyg/hal/avr32/iouc64l4u.h>
#elif (defined __ATmXT1664S__)
#include <cyg/hal/avr32/mxt1664s.h>
#elif (defined __ATmXT1664ENG__)
#include <cyg/hal/avr32/mxt1664eng.h>
#elif (defined __ATmXT768E__)
#include <cyg/hal/avr32/mxt768e.h>
#elif (defined __ATmXT540E__)
#include <cyg/hal/avr32/mxt540e.h>
#  else
#    error No known AVR32 part defined
#  endif  /* __AT32<PART>__ */
#else
#   error Unknown compiler
#endif /* __GNUC__ */

#endif /* _IO_H_ */
