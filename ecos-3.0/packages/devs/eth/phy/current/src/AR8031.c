//==========================================================================
//
//      dev/AR8031.c
//
//      Ethernet transceiver (PHY) support for Qualcomm AR8031
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2008 Free Software Foundation, Inc.                        
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
// Author(s):    Mike Jones <mike@proclivis.com>
// Contributors:
// Date:         2013-02-13
// Purpose:
// Description:  Support for ethernet PHY Qualcomm KSZA8031
//
//
//####DESCRIPTIONEND####
//
//==========================================================================


//==========================================================================
//                                INCLUDES
//==========================================================================
#include <pkgconf/system.h>
#include <pkgconf/devs_eth_phy.h>

#include <cyg/infra/cyg_type.h>

#include <cyg/hal/hal_arch.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_tables.h>

#include <cyg/io/eth_phy.h>
#include <cyg/io/eth_phy_dev.h>


//==========================================================================
//                                DEFINES
//==========================================================================

#define AR8031_MII_STAT_REG              0x01
// 0 if down since last read
#define AR8031_MII_PHY_STAT_LINK_UP      0x0004
#define AR8031_AUTO_COMPLETED            0x0020

#define AR8031_MII_STAT_SPEC_REG         0x11
#define AR8031_MII_PHY_STAT_DUPLEX_FULL  0x2000
#define AR8031_MII_PHY_STAT_100MB        0x4000
#define AR8031_MII_PHY_STAT_1000MB       0x8000
// Real time link up/dn
#define AR8031_MII_PHY_STAT_SPEC_LINK_UP 0x0400

void
ar8031_init (eth_phy_access_t * f)
{
	unsigned short val;

	// Not that applying a soft reset may change registers
	// that the ethernet driver previously initialized. So don't
	// use the wand version without checking the datasheet and
	// driver code.
#if 0
    // This code came from file board-wand.c from the Wandboard Linux SDK.

    /* Enable AR8031 125MHz clk */
	_eth_phy_write(f, 0x0d, f->phy_addr, 0x0007); /* Set device address to 7*/
	_eth_phy_write(f, 0x00, f->phy_addr, 0x8000); /* Apply by soft reset */
    hal_delay_us(500);

    _eth_phy_write(f, 0x0e, f->phy_addr, 0x8016); /* set mmd reg */
    _eth_phy_write(f, 0x0d, f->phy_addr, 0x4007); /* apply */

    _eth_phy_read(f, 0xe, f->phy_addr, &val);
    val &= 0xffe7;
    val |= 0x18;
    _eth_phy_write(f, 0xe, f->phy_addr, val);
    _eth_phy_write(f, 0x0d, f->phy_addr, 0x4007); /* Post data */

    /* Introduce random tx clock delay. Why is this needed? */
    _eth_phy_write(f, 0x1d, f->phy_addr, 0x5);
    _eth_phy_read(f, 0x1e, f->phy_addr, &val);
    val |= 0x0100;
    _eth_phy_write(f, 0x1e, f->phy_addr, val);
#endif

#if 1

    // This code came from file board-mx6q_sabresd.c from the Wandboard Linux SDK.

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
    _eth_phy_write(f, 0xd, f->phy_addr, 0x3);
    _eth_phy_write(f, 0xe, f->phy_addr, 0x805d);
    _eth_phy_write(f, 0xd, f->phy_addr, 0x4003);
	_eth_phy_read(f, 0xe, f->phy_addr, &val);
	val &= ~(0x1 << 8);
	_eth_phy_write(f, 0xe, f->phy_addr, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	_eth_phy_write(f, 0xd, f->phy_addr, 0x7);
	_eth_phy_write(f, 0xe, f->phy_addr, 0x8016);
	_eth_phy_write(f, 0xd, f->phy_addr, 0x4007);
	_eth_phy_read(f, 0xe, f->phy_addr, &val);

	val &= 0xffe7;
	val |= 0x18;
	_eth_phy_write(f, 0xe, f->phy_addr, val);

	/* Introduce tx clock delay */
	_eth_phy_write(f, 0x1d, f->phy_addr, 0x5);
	_eth_phy_read(f, 0x1e, f->phy_addr, &val);
	val |= 0x0100;
	_eth_phy_write(f, 0x1e, f->phy_addr, val);

	/*check phy power*/
	_eth_phy_read(f, 0x0, f->phy_addr, &val);

	if (val & (1 << 11))
		_eth_phy_write(f, 0x0, f->phy_addr, (val & ~(1 << 11)));
#endif
}


//==========================================================================
// Query the 100BASE-TX PHY Control Register and return a status bitmap
// indicating the state of the physical connection
//==========================================================================

#ifdef  CYGDBG_DEVS_ETH_PHY
void
ar8031_diag (eth_phy_access_t * f)
{

  cyg_uint32 i;
  cyg_uint16 reg;

  eth_phy_printf ("AR8031 MIIM Register setings:\n");

  for (i = 0; i < 0x15; i++) {
    if (i % 2 == 0) {
      _eth_phy_read (f, i, f->phy_addr, &reg);
      eth_phy_printf ("r%02x: %04x ", i, reg);
    } else {
      _eth_phy_read (f, i, f->phy_addr, &reg);
      eth_phy_printf ("%04x\n", reg);
    }
  }
}
#endif

static bool
ar8031_stat (eth_phy_access_t * f, int *state)
{

#ifdef  CYGDBG_DEVS_ETH_PHY
  ar8031_diag (f);
#endif
    unsigned short phy_state;
    int tries;
    int auto_completed = 1;

    if (_eth_phy_read(f, AR8031_MII_STAT_REG, f->phy_addr, &phy_state))
    {

        if ((phy_state & AR8031_AUTO_COMPLETED) == 0)
        { 
            auto_completed = 0;
            eth_phy_printf("... waiting for auto-negotiation");
            for (tries = 0;  tries < CYGINT_DEVS_ETH_PHY_AUTO_NEGOTIATION_TIME;  tries++)
            {
                if (_eth_phy_read(f, AR8031_MII_STAT_REG, f->phy_addr, &phy_state))
                {
                    if ((phy_state & AR8031_AUTO_COMPLETED) != 0)
                    {                        
                        auto_completed = 1;
                        break;
                    }
                }
                CYGACC_CALL_IF_DELAY_US(1000000);   // 1 second
                eth_phy_printf(".");
            }
            eth_phy_printf("\n");
        }  
      
        if(auto_completed)
        {
          _eth_phy_read(f, AR8031_MII_STAT_SPEC_REG, f->phy_addr, &phy_state);
          if(phy_state & AR8031_MII_PHY_STAT_SPEC_LINK_UP) 
            *state |= ETH_PHY_STAT_LINK;
          if(phy_state & AR8031_MII_PHY_STAT_1000MB )
            *state |= ETH_PHY_STAT_1000MB;
          if(phy_state & AR8031_MII_PHY_STAT_100MB)
            *state |= ETH_PHY_STAT_100MB;
          if(phy_state & AR8031_MII_PHY_STAT_DUPLEX_FULL)
            *state |= ETH_PHY_STAT_FDX;
          return true;
        }

    }
    return false;
}
// Added 0x4 to end to match what was seen during debug. I believe it is a version number.
_eth_phy_dev ("Atheros AR8031", 0x004dd074, ar8031_stat)
