#ifndef CYGONCE_HAL_PLF_IO_H
#define CYGONCE_HAL_PLF_IO_H
//=============================================================================
//
//      plf_io.h
//
//      Freescale Wandboard board specific registers
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2008 Free Software Foundation, Inc.
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
// Author(s):    Mike Jones
// Date:         2013-08-08
// Purpose:      Freescale Wandboard specific registers
// Description:
// Usage:        #include <cyg/hal/plf_io.h>
//
//####DESCRIPTIONEND####
//
//=============================================================================
#include <pkgconf/hal_arm_wandboard.h>
#include <cyg/hal/iomux_define.h>

// UART PINs
#ifndef CYGHWR_HAL_IMX6_UART0_PIN_RX
#define CYGHWR_HAL_IMX6_UART0_PIN_RX CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_CSI0_DATA11, SION_DISABLED, ALT3, HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, BV_IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT_DAISY__CSI0_DATA11_ALT3)

#define CYGHWR_HAL_IMX6_UART0_PIN_TX CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_CSI0_DATA10, SION_DISABLED, ALT3, HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, 0)

#define CYGHWR_HAL_IMX6_UART0_PIN_RTS CYGHWR_HAL_IMX6_PIN_NONE
#define CYGHWR_HAL_IMX6_UART0_PIN_CTS CYGHWR_HAL_IMX6_PIN_NONE

#define CYGHWR_IO_FREESCALE_UART0_PIN_RX CYGHWR_HAL_IMX6_UART0_PIN_RX
#define CYGHWR_IO_FREESCALE_UART0_PIN_TX CYGHWR_HAL_IMX6_UART0_PIN_TX
#define CYGHWR_IO_FREESCALE_UART0_PIN_RTS CYGHWR_HAL_IMX6_UART0_PIN_RTS
#define CYGHWR_IO_FREESCALE_UART0_PIN_CTS CYGHWR_HAL_IMX6_UART0_PIN_CTS
#endif

// Add include to ENET to help macros

#define CYGSEM_DEVS_ETH_FREESCALE_ENET_INCLUDE <cyg/hal/registers/regsiomuxc.h>

// ENET PINs

// MDIO
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_MDIO CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_MDIO, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
	BV_IOMUXC_ENET_MAC0_MDIO_SELECT_INPUT_DAISY__ENET_MDIO_ALT1)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_MDC CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_MDC, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
	0)

// Both RMII and MII interface

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXD1 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII_RD1, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, ODT_OFF, IO_1P5V, 0, DSE_40OHM, 0, \
	BV_IOMUXC_ENET_MAC0_RX_DATA1_SELECT_INPUT_DAISY__RGMII_RD1_ALT1)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXD0 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII_RD0, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, ODT_OFF, IO_1P5V, 0, DSE_40OHM, 0, \
	BV_IOMUXC_ENET_MAC0_RX_DATA0_SELECT_INPUT_DAISY__RGMII_RD0_ALT1)

//#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXEN CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII_TXEN, SION_DISABLED, ALT5, \
//	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
//	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXD0 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII_TD0, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, 0, IO_1P5V, 0, DSE_40OHM, 0, \
	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXD1 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII_TD1, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, 0, IO_1P5V, 0, DSE_40OHM, 0, \
	0)

// MII interface only
#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXD3 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII0_RD3, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, ODT_OFF, IO_1P5V, 0, DSE_40OHM, 0, \
	BV_IOMUXC_ENET_MAC0_RX_DATA3_SELECT_INPUT_DAISY__RGMII_RD3_ALT1)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXD2 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII0_RD2, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, ODT_OFF, IO_1P5V, 0, DSE_40OHM, 0, \
	BV_IOMUXC_ENET_MAC0_RX_DATA2_SELECT_INPUT_DAISY__RGMII_RD2_ALT1)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXCLK CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII0_RXC, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PD, PUE_PULL, PKE_ENABLED, 0, ODT_OFF, IO_1P5V, 0, DSE_40OHM, 0, \
	BV_IOMUXC_ENET_MAC0_RX_CLK_SELECT_INPUT_DAISY__RGMII_RXC_ALT1)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXD2 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII0_TD2, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, 0, IO_1P5V, 0, DSE_40OHM, 0, \
	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXCLK CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII0_TXC, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PD, PUE_PULL, PKE_ENABLED, 0, 0, IO_1P5V, 0, DSE_40OHM, 0, \
	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXD3 CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_RGMII0_TD3, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, 0, 0, IO_1P5V, 0, DSE_40OHM, 0, \
	0)

//#define CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_CRS CYGHWR_HAL_IMX6_PIN_(0, 0, 0, \
//	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
//	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXEN CYGHWR_HAL_IMX6_PIN_(0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0)

// Original SD
#define CYGHWR_IO_FREESCALE_ENET0_PIN_REF_CLK CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_REF_CLK, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
	0)
// Linux
//#define CYGHWR_IO_FREESCALE_ENET0_PIN_REF_CLK CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_REF_CLK, SION_DISABLED, ALT1, \
//	HYS_DISABLED, PUS_100KOHM_PU, PUE_PULL, PKE_DISABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_FAST, \
//	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RX_CTL CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_RX_CTL, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PD, PUE_PULL, PKE_ENABLED, 0, ODT_OFF, IO_1P5V, 0, DSE_40OHM, 0, \
	BV_IOMUXC_ENET_MAC0_RX_EN_SELECT_INPUT_DAISY__RGMII_RX_CTL_ALT1)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_TX_CTL CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_TX_CTL, SION_DISABLED, ALT1, \
	HYS_ENABLED, PUS_100KOHM_PD, PUE_PULL, PKE_ENABLED, 0, 0, IO_1P5V, 0, DSE_40OHM, 0, \
	0)

// Sabre SD values to be moved to there later
//#define CYGHWR_IO_FREESCALE_ENET0_PIN_RGMII_NRST CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_CRS_DV, SION_DISABLED, ALT5, \
//	HYS_DISABLED, PUS_100KOHM_PD, PUE_KEEP, PKE_DISABLED, ODE_DISABLED, 0, 0, 0, DSE_240OHM, SRE_SLOW, \
//	0)

//#define CYGHWR_IO_FREESCALE_ENET0_PIN_RGMII_INT CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_RX_DATA1, SION_DISABLED, ALT5, \
//	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
//	0)

//#define CYGHWR_IO_FREESCALE_ENET0_PIN_WOL_INT CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_TX_EN, SION_DISABLED, ALT5, \
//	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
//	0)

// Wandboard vales to remain here.
#define CYGHWR_IO_FREESCALE_ENET0_PIN_RGMII_NRST CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_EIM_D29, SION_DISABLED, ALT5, \
	HYS_DISABLED, PUS_100KOHM_PD, PUE_KEEP, PKE_DISABLED, ODE_DISABLED, 0, 0, 0, DSE_240OHM, SRE_SLOW, \
	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_RGMII_INT CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_ENET_TX_EN, SION_DISABLED, ALT5, \
	HYS_ENABLED, PUS_100KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
	0)

#define CYGHWR_IO_FREESCALE_ENET0_PIN_WOL_INT CYGHWR_HAL_IMX6_PIN_(0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0)


/*
Mapping from pin names to macros.

ENET_MDC	ENET_MDC 	(V20)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_MDC
ENET_MDIO	ENET_MDIO 	(V23)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_MDIO
ENET_TX_CLK	ENET_REF_CLK 	(V22)	CYGHWR_IO_FREESCALE_ENET0_PIN_1588_CLKIN
RGMII_RD0 	RGMII_RD0 	(C24)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXD0
RGMII_RD1	RGMII_RD1 	(B23)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXD1
RGMII_RD2	RGMII_RD2 	(B24)	CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXD2
RGMII_RD3	RGMII_RD3 	(D23)	CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXD3
RGMII_RXC	RGMII_RXC 	(B25)	CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_RXCLK
RGMII_RX_CTL	RGMII_RX_CTL 	(D22)							Phy has nibble
RGMII_TD0	RGMII_TD0 	(C22)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXD0
RGMII_TD1	RGMII_TD1 	(F20)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXD1
RGMII_TD2	RGMII_TD2 	(E21)	CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXD2
RGMII_TD3	RGMII_TD3 	(A24)	CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXD3
RGMII_TXC	RGMII_TXC 	(D21)	CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_TXCLK
RGMII_TX_CTL	RGMII_TX_CTL 	(C23)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_TXEN	Enet has nibble
RGMII_INT	ENET_RX_DATA1 	(W22)							Has ethernet data
ETH_WOL_INT	ENET_TX_EN 	(V21)							Enet has nibble
		ENET_RX_EN								Phy has nibble
// Not available
RGMII_NRST	ENET_CRS_DV 	(U21)	CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_CRS_DV
CYGHWR_IO_FREESCALE_ENET0_PIN_RMII0_RXER	(W23)	1588 Event 2 out
CYGHWR_IO_FREESCALE_ENET0_PIN_MIIO_TXER		(P5) 	Send/sent illegal signals	GPIO_19
CYGHWR_IO_FREESCALE_ENET0_PIN_MII0_COL		(U6)	Asserted for collision		KEY_ROW1
*/


#if defined(CYGHWR_HAL_FREESCALE_ENET0_E0_1588_PORT_B)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR0  1
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR1  1
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR2  C1
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR3  1
#elif defined(CYGHWR_HAL_FREESCALE_ENET0_E0_1588_PORT_C)
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR0  1
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR1  1
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR2  1
# define CYGHWR_IO_FREESCALE_ENET0_PIN_E0_1588_TMR3  1
#endif

// I2C
// I2C Pins

# define CYGHWR_IO_I2C_FREESCALE_I2C0_PIN_SDA CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_I2C_SDA0, SION_ENABLED, ALT1, \
		HYS_ENABLED, PUS_22KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
		BV_IOMUXC_I2C1_SDA_IN_SELECT_INPUT_DAISY__EIM_DATA28_ALT1)
# define CYGHWR_IO_I2C_FREESCALE_I2C0_PIN_SCL CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_I2C_SCL0, SION_ENABLED, ALT6, \
		HYS_ENABLED, PUS_22KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
		BV_IOMUXC_I2C1_SCL_IN_SELECT_INPUT_DAISY__EIM_DATA21_ALT6)

# define CYGHWR_IO_I2C_FREESCALE_I2C1_PIN_SDA CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_I2C_SDA1, SION_ENABLED, ALT4, \
		HYS_ENABLED, PUS_22KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
		BV_IOMUXC_I2C2_SDA_IN_SELECT_INPUT_DAISY__KEY_ROW3_ALT4)
# define CYGHWR_IO_I2C_FREESCALE_I2C1_PIN_SCL CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_I2C_SCL1, SION_ENABLED, ALT4, \
		HYS_ENABLED, PUS_22KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
		BV_IOMUXC_I2C2_SCL_IN_SELECT_INPUT_DAISY__KEY_COL3_ALT4)

# define CYGHWR_IO_I2C_FREESCALE_I2C2_PIN_SDA CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_I2C_SDA2, SION_ENABLED, ALT6, \
		HYS_ENABLED, PUS_22KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
		BV_IOMUXC_I2C3_SDA_IN_SELECT_INPUT_DAISY__GPIO16_ALT6)
# define CYGHWR_IO_I2C_FREESCALE_I2C2_PIN_SCL CYGHWR_HAL_IMX6_PIN_(CYGHWR_HAL_FREESCALE_PIN_I2C_SCL2, SION_ENABLED, ALT6, \
		HYS_ENABLED, PUS_22KOHM_PU, PUE_PULL, PKE_ENABLED, ODE_DISABLED, 0, 0, SPD_100MHZ, DSE_40OHM, SRE_SLOW, \
		BV_IOMUXC_I2C3_SCL_IN_SELECT_INPUT_DAISY__GPIO05_ALT6)

//----------------------------------------------------------------------
// The platform needs this initialization during the
// hal_hardware_init() function in the varient HAL.
#ifndef __ASSEMBLER__
extern void hal_plf_hardware_init(void);
#define HAL_PLF_HARDWARE_INIT() \
    hal_plf_hardware_init()

#endif  //__ASSEMBLER__ 

//-----------------------------------------------------------------------------
// end of plf_io.h
#endif // CYGONCE_HAL_PLF_IO_H
