#ifndef CYGONCE_HAL_VAR_IO_H
#define CYGONCE_HAL_VAR_IO_H
//=============================================================================
//
//      var_io.h
//
//      Variant specific registers
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003, 2005, 2006, 2009 Free Software Foundation, Inc.
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):      Mike Jones
// Contributor(s): ITR-GmbH, Ant Micro <www.antmicro.com>
// Date:           2013-08-08
// Purpose:        iMXQ variant specific registers
// Description:
// Usage:          #include <cyg/hal/var_io.h>
//
//####DESCRIPTIONEND####
//
//=============================================================================

#include <cyg/hal/plf_io.h>

//=============================================================================
// DEVS:
// Following macros may also be, and usually are borrwed by some device drivers.
//-----------------------------------------------------------------------------
#include <cyg/hal/var_io_devs.h>

//=============================================================================
// IOMUX:
// Following macros may also be, and usually are borrwed by some device drivers.
//-----------------------------------------------------------------------------

// IOV
#define IO_1P2V 0x2
#define IO_1P5V 0x3

/*
The sizes of each field.

ARG         SIZE
----------------
TYPE        5
SION        1
ALT         3
HYS         1
PUS         2
PUE         1
PKE         1
ODE         1
ODT         3
IOV         2
SPEED       2
DSE         3
SRE         1
DAISY       2
---------------
TOT         28
*/

#define CYGHWR_HAL_FREESCALE_PIN_CSI0_DATA10	1
#define CYGHWR_HAL_FREESCALE_PIN_CSI0_DATA11	2

#define CYGHWR_HAL_FREESCALE_PIN_ENET_MDIO 	3
#define CYGHWR_HAL_FREESCALE_PIN_ENET_MDC 	4
#define CYGHWR_HAL_FREESCALE_PIN_RGMII_RD1	5
#define CYGHWR_HAL_FREESCALE_PIN_RGMII_RD0	6
#define CYGHWR_HAL_FREESCALE_PIN_RGMII_TD0	7
#define CYGHWR_HAL_FREESCALE_PIN_RGMII_TD1	8
#define CYGHWR_HAL_FREESCALE_PIN_RGMII0_RD3	9
#define CYGHWR_HAL_FREESCALE_PIN_RGMII0_RD2	10
#define CYGHWR_HAL_FREESCALE_PIN_RGMII0_RXC	11
#define CYGHWR_HAL_FREESCALE_PIN_RGMII0_TD2	12
#define CYGHWR_HAL_FREESCALE_PIN_RGMII0_TXC	13
#define CYGHWR_HAL_FREESCALE_PIN_RGMII0_TD3	14
#define CYGHWR_HAL_FREESCALE_PIN_ENET_CRS_DV	15
#define CYGHWR_HAL_FREESCALE_PIN_ENET_TX_CLK	16
#define CYGHWR_HAL_FREESCALE_PIN_ENET_RX_CTL	17
#define CYGHWR_HAL_FREESCALE_PIN_ENET_TX_CTL	18
#define CYGHWR_HAL_FREESCALE_PIN_ENET_RX_DATA1	19
#define CYGHWR_HAL_FREESCALE_PIN_ENET_REF_CLK 	20
#define CYGHWR_HAL_FREESCALE_PIN_ENET_TX_EN	21
#define CYGHWR_HAL_FREESCALE_PIN_EIM_D29	22
#define CYGHWR_HAL_FREESCALE_PIN_I2C_SDA0	23
#define CYGHWR_HAL_FREESCALE_PIN_I2C_SDA1	24
#define CYGHWR_HAL_FREESCALE_PIN_I2C_SDA2	25
#define CYGHWR_HAL_FREESCALE_PIN_I2C_SCL0	26
#define CYGHWR_HAL_FREESCALE_PIN_I2C_SCL1	27
#define CYGHWR_HAL_FREESCALE_PIN_I2C_SCL2	28
#define CYGHWR_HAL_FREESCALE_PIN_DISP0_DAT20 29

#define CYGHWR_HAL_FREESCALE_PIN_FUN_RX 1
#define CYGHWR_HAL_FREESCALE_PIN_FUN_TX 2
#define CYGHWR_HAL_FREESCALE_PIN_FUN_RTS 3
#define CYGHWR_HAL_FREESCALE_PIN_FUN_CTL 4

#define CYGHWR_HAL_FREESCALE_PERIPH_UART 1

#define CYGHWR_HAL_IMX6_PIN_(__type, __sion, __alt, __hys, __pus, __pue, __pke, __ode, __odt, __iov, __spd, __dse, __sre, __daisy) \
         ( __type << 23  \
         | __sion << 22 \
         | __alt << 19  \
         | __hys << 18  \
         | __pus << 16  \
         | __pue << 15  \
         | __pke << 14   \
         | __ode << 13   \
         | __odt << 10  \
         | __iov << 8   \
         | __spd << 6   \
         | __dse << 3   \
         | __sre << 2   \
         | __daisy)

#define CYGHWR_HAL_IMX6_PIN_TYPE(__pin) (((__pin) >> 23) & 0x1f)
#define CYGHWR_HAL_IMX6_PIN_SION(__pin) (((__pin) >> 22) & 0x1)
#define CYGHWR_HAL_IMX6_PIN_ALT(__pin) (((__pin) >> 19) & 0x7)
#define CYGHWR_HAL_IMX6_PIN_HYS(__pin) (((__pin) >> 18) & 0x1)
#define CYGHWR_HAL_IMX6_PIN_PUS(__pin) (((__pin) >> 16) & 0x3)
#define CYGHWR_HAL_IMX6_PIN_PUE(__pin) (((__pin) >> 15) & 0x1)
#define CYGHWR_HAL_IMX6_PIN_PKE(__pin) (((__pin) >> 14) & 0x1)
#define CYGHWR_HAL_IMX6_PIN_ODE(__pin) (((__pin) >> 13) & 0x1)
#define CYGHWR_HAL_IMX6_PIN_ODT(__pin) (((__pin) >> 10) & 0x7)
#define CYGHWR_HAL_IMX6_PIN_IOV(__pin) (((__pin) >> 8) & 0x3)
#define CYGHWR_HAL_IMX6_PIN_SPD(__pin) (((__pin) >> 6) & 0x3)
#define CYGHWR_HAL_IMX6_PIN_DSE(__pin) (((__pin) >> 3) & 0x7)
#define CYGHWR_HAL_IMX6_PIN_SRE(__pin) (((__pin) >> 2) & 0x1)
#define CYGHWR_HAL_IMX6_PIN_DAISY(__pin) ((__pin) & 0x03)
#define CYGHWR_HAL_IMX6_PIN_NONE (0xffffffff)

#define HW_IOMUXC_SW_MUX_CTL_PAD_DATA_WR(__type) \
    HW_IOMUXC_SW_MUX_CTL_PAD_ ## __type ## _WR

#define HW_IOMUXC_SW_PAD_CTL_PAD_DATA_WR(__type) \
    HW_IOMUXC_SW_PAD_CTL_PAD_ ## __type ## _WR

#define HW_IOMUXC_UART_UART_SELECT_INPUT_WR(__num, __type) \
    HW_IOMUXC_UART ## __num ## _UART_ ## __type ## _SELECT_INPUT_WR

//---------------------------------------------------------------------------
// Clock distribution
// The following encodes the control register and clock bit number
// into clock configuration descriptor (CLKCD).
#define CYGHWR_HAL_IMX6_SIM_CCGR(__reg,__bit0,__val0,__bit1,__val1) (((__reg) & 0xF) | \
                                                                    (((__bit0) << 4) &  0x00F0) | \
                                                                    (((__bit1) << 8) &  0x0F00) | \
                                                                    (((__val0) << 12) & 0x3000) | \
                                                                    (((__val1) << 14) & 0xC000))

// Macros to extract encoded values.
#define CYGHWR_HAL_IMX6_SIM_CCGR_REG(__clkcd) (((__clkcd) & 0xF))
#define CYGHWR_HAL_IMX6_SIM_CCGR_BIT0(__clkcd) (((__clkcd) >> 4) & 0xF)
#define CYGHWR_HAL_IMX6_SIM_CCGR_BIT1(__clkcd) (((__clkcd) >> 8) & 0xF)
#define CYGHWR_HAL_IMX6_SIM_CCGR_VAL0(__clkcd) (((__clkcd) >> 12) & 0x3)
#define CYGHWR_HAL_IMX6_SIM_CCGR_VAL1(__clkcd) (((__clkcd) >> 14) & 0x3)

#ifndef __ASSEMBLER__

// Pin configuration related functions
__externC void  hal_set_pin_function(cyg_uint32 pin);

// Functions and macros to enable/disable clocks.
#define CYGHWR_HAL_CCGR_NONE (0xFFFFFFFF)
__externC void hal_clock_enable(cyg_uint32 clkcd);
__externC void hal_clock_disable(cyg_uint32 clkcd);

#define CYGHWR_HAL_CLOCK_ENABLE(__clkcd) hal_clock_enable(__clkcd)
#define CYGHWR_HAL_CLOCK_DISABLE(__clkcd) hal_clock_disable(__clkcd)

#endif

#include <cyg/hal/var_io_clkgat.h>

//-----------------------------------------------------------------------------
// end of var_io.h
#endif // CYGONCE_HAL_VAR_IO_H
