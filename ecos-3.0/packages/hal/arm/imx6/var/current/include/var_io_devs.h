#ifndef CYGONCE_HAL_VAR_IO_DEVS_H
#define CYGONCE_HAL_VAR_IO_DEVS_H
//===========================================================================
//
//      var_io_devs.h
//
//      Variant specific registers
//
//===========================================================================
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
//===========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):     Mike Jones <mike@proclivis.com>
// Date:          2013-08-08
// Purpose:       iMX6 variant IO provided to various device drivers
// Description:
// Usage:         #include <cyg/hal/var_io.h> //var_io.h includes this file
//
//####DESCRIPTIONEND####
//
//===========================================================================

//#include <cyg/infra/cyg_type.h>

//=============================================================================
// DEVS:
// Following macros may be, and usually are borrwed by some device drivers.

// Peripheral clock [Hz];
//__externC cyg_uint32 hal_get_peripheral_clock(void);

//-----------------------------------------------------------------------------
// Freescale UART
// Borrow some HAL resources to Freescale UART driver
// UART  macros are used by both:
//      src/hal_diag.c
//      devs/serial/<version>/src/ser_freescale_uarta.c

#define CYGADDR_IO_SERIAL_FREESCALE_UART0_BASE  0x02020000
#define CYGADDR_IO_SERIAL_FREESCALE_UART1_BASE  0x021E8000
#define CYGADDR_IO_SERIAL_FREESCALE_UART2_BASE  0x021EC000
#define CYGADDR_IO_SERIAL_FREESCALE_UART3_BASE  0x021F0000
#define CYGADDR_IO_SERIAL_FREESCALE_UART4_BASE  0x021F4000

// UART Clock gating

#define CYGHWR_IO_FREESCALE_UART0_CLOCK CYGHWR_HAL_IMX6_SIM_CCGR_UART0
#define CYGHWR_IO_FREESCALE_UART1_CLOCK CYGHWR_HAL_CCGR_NONE
#define CYGHWR_IO_FREESCALE_UART2_CLOCK CYGHWR_HAL_CCGR_NONE
#define CYGHWR_IO_FREESCALE_UART3_CLOCK CYGHWR_HAL_CCGR_NONE
#define CYGHWR_IO_FREESCALE_UART4_CLOCK CYGHWR_HAL_CCGR_NONE

#define CYGHWR_HAL_FREESCALE_UART0_CLOCK CYGHWR_IO_FREESCALE_UART0_CLOCK
#define CYGHWR_HAL_FREESCALE_UART1_CLOCK CYGHWR_IO_FREESCALE_UART1_CLOCK
#define CYGHWR_HAL_FREESCALE_UART2_CLOCK CYGHWR_IO_FREESCALE_UART2_CLOCK
#define CYGHWR_HAL_FREESCALE_UART3_CLOCK CYGHWR_IO_FREESCALE_UART3_CLOCK
#define CYGHWR_HAL_FREESCALE_UART4_CLOCK CYGHWR_IO_FREESCALE_UART4_CLOCK

// UART PIN configuration
// Note: May be overriden by plf_io.h

#define CYGHWR_HAL_FREESCALE_PORT_PIN_NONE CYGHWR_HAL_IMX6_PIN_NONE

#ifndef CYGHWR_IO_FREESCALE_UART0_PIN_RX
# define CYGHWR_IO_FREESCALE_UART0_PIN_RX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART0_PIN_TX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART0_PIN_RTS CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART0_PIN_CTS CYGHWR_HAL_IMX6_PIN_NONE
#endif

#ifndef CYGHWR_IO_FREESCALE_UART1_PIN_RX
# define CYGHWR_IO_FREESCALE_UART1_PIN_RX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART1_PIN_TX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART1_PIN_RTS CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART1_PIN_CTS CYGHWR_HAL_IMX6_PIN_NONE
#endif

#ifndef CYGHWR_IO_FREESCALE_UART2_PIN_RX
# define CYGHWR_IO_FREESCALE_UART2_PIN_RX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART2_PIN_TX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART2_PIN_RTS CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART2_PIN_CTS CYGHWR_HAL_IMX6_PIN_NONE
#endif

#ifndef CYGHWR_IO_FREESCALE_UART3_PIN_RX
# define CYGHWR_IO_FREESCALE_UART3_PIN_RX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART3_PIN_TX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART3_PIN_RTS CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART3_PIN_CTS CYGHWR_HAL_IMX6_PIN_NONE
#endif

#ifndef CYGHWR_IO_FREESCALE_UART4_PIN_RX
# define CYGHWR_IO_FREESCALE_UART4_PIN_RX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART4_PIN_TX CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART4_PIN_RTS CYGHWR_HAL_IMX6_PIN_NONE
# define CYGHWR_IO_FREESCALE_UART4_PIN_CTS CYGHWR_HAL_IMX6_PIN_NONE
#endif

//---------------------------------------------------------------------------
// ENET
// Lend some HAL dependent functions to the Ethernet device driver
#define CYGADDR_IO_ETH_FREESCALE_ENET0_BASE  (0x02188000)

// Clock gating
// Note that hal_ccm_init enables the enet pll and clock gating. May
// want to enhance this to cover all the required settings and leave
// and disable the pll in hal_ccm_init.
#define CYGHWR_IO_FREESCALE_ENET0_CLOCK CYGHWR_HAL_IMX6_SIM_CCGR_ENET0

// Lend some HAL dependent functions to the UART serial device driver

#ifndef __ASSEMBLER__

// Configure pin
__externC void hal_imx6_config_pin(cyg_uint32 pin);

// Configure pin daisy
__externC void hal_imx6_config_pin_daisy(cyg_uint32 periph, cyg_uint32 type, cyg_uint32 port, cyg_uint32 pin);

// Set baud rate
__externC void hal_imx6_uart_setbaud( CYG_ADDRESS uart, cyg_uint32 baud );

# define CYGHWR_IO_FREESCALE_ENET_PIN(__pin) \
        hal_imx6_config_pin(__pin)

# define CYGHWR_IO_FREESCALE_UART_BAUD_SET(__uart_p, _baud_) \
        hal_imx6_uart_setbaud(__uart_p, _baud_)

#define CYGHWR_IO_UART_PIN_CONFIG(__pin) \
        hal_imx6_config_pin(__pin)

#define CYGHWR_IO_UART_PIN_CONFIG_DAISY(__periph, __type, __port, __pin) \
        hal_imx6_config_pin_daisy(__periph, __type, __port, __pin)

#define CYGHWR_IO_CLOCK_ENABLE(__clkcd) \
        hal_clock_enable(__clkcd)

#define CYGHWR_IO_PHY_RESET(__phy) \
        hal_reset_phy()

#define CYGHWR_IO_PHY_INIT(__phy) \
	ar8031_init(__phy)

#define CYGHWR_IO_CLOCK(__clkcd) \
        hal_get_peri_clock(__clkcd)

#endif

//---------------------------------------------------------------------------
// I2C
// Lend some HAL dependent macros to I2C device driver
// Base pointers
#define CYGADDR_IO_I2C_FREESCALE_I2C0_BASE  (0x021A0000)
#define CYGADDR_IO_I2C_FREESCALE_I2C1_BASE  (0x021A4000)
#define CYGADDR_IO_I2C_FREESCALE_I2C2_BASE  (0x021A8000)

// Clocking
#define CYGHWR_IO_I2C_FREESCALE_I2C_CLOCK hal_get_peripheral_clock()
#define CYGHWR_IO_FREESCALE_I2C0_CLK  CYGHWR_HAL_IMX6_SIM_SCGC_I2C0
#define CYGHWR_IO_FREESCALE_I2C1_CLK  CYGHWR_HAL_IMX6_SIM_SCGC_I2C1
#define CYGHWR_IO_FREESCALE_I2C2_CLK  CYGHWR_HAL_IMX6_SIM_SCGC_I2C2

// Pins
# define CYGHWR_IO_FREESCALE_I2C_PIN(__pin) hal_set_pin_function(__pin)

#ifndef CYGHWR_IO_FREESCALE_I2C_FREQUENCY_TABLE
// Use define because this gets included by Vectors.S
//typedef cyg_uint16 dev_i2c_freescale_frequency_entry_t;
#define dev_i2c_freescale_frequency_entry_t cyg_uint16
#define CYGHWR_IO_FREESCALE_I2C_FREQUENCY_TABLE1                                             \
    30, 32, 36, 42, 48, 52, 60, 72, 80, 88, 104, 128, 144, 160, 192, 240,                   \
	288, 320, 384, 480, 576, 640, 768, 960, 1152, 1280, 1536, 1920, 2304, 2560, 3072, 3840

#define CYGHWR_IO_FREESCALE_I2C_FREQUENCY_TABLE2                                            \
	22, 24, 26, 28, 32, 36, 40, 44, 48, 56, 64, 72, 80, 96, 112, 128,                       \
	160, 192, 224, 256, 320, 384, 448, 512, 640, 768, 896, 1024, 1280, 1536, 1792, 2048

#endif // CYGHWR_IO_FREESCALE_I2C_FREQUENCY_TABLE

// end of var_io_devs.h
#endif // CYGONCE_HAL_VAR_IO_DEVS_H
