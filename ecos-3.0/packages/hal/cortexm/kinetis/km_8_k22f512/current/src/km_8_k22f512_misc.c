//==========================================================================
//
//      twr_k60f120m_misc.c
//
//      Cortex-M4 TWR-K60N512 EVAL HAL functions
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2012, 2013 Free Software Foundation, Inc.                        
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
// Author(s):      Ilija Kocho <ilijak@siva.com.mk>
// Contrib(s):     Mike Jones <mjones@proclivis.com>
// Date:           2013-06-02
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>
#include <pkgconf/hal_cortexm.h>
#include <pkgconf/hal_cortexm_kinetis.h>
#include <pkgconf/hal_cortexm_kinetis_km_8_k22f512.h>
#ifdef CYGPKG_KERNEL
#include <pkgconf/kernel.h>
#endif

#include <cyg/hal/drv_api.h>
#include <cyg/io/spi.h>
#include <cyg/io/spi_freescale_dspi.h>

#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_arch.h>           // HAL header
#include <cyg/hal/hal_intr.h>           // HAL header

static inline void hal_misc_init(void);

// DATA and BSS locations
__externC cyg_uint32 __ram_data_start;
__externC cyg_uint32 __ram_data_end;
__externC cyg_uint32 __rom_data_start;
__externC cyg_uint32 __sram_data_start;
__externC cyg_uint32 __sram_data_end;
__externC cyg_uint32 __srom_data_start;
__externC cyg_uint32 __bss_start;
__externC cyg_uint32 __bss_end;

//==========================================================================
// System init
//
// This is run to set up the basic system, including GPIO setting,
// clock feeds, power supply, and memory initialization. This code
// runs before the DATA is copied from ROM and the BSS cleared, hence
// it cannot make use of static variables or data tables.

__externC void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_system_init( void )
{
#if !defined(CYG_HAL_STARTUP_RAM)
    cyghwr_hal_kinetis_pmc_t *pmc_p = CYGHWR_HAL_KINETIS_PMC_P;

    hal_wdog_disable();
    hal_misc_init();

    // if ACKISO is set you must clear ackiso before calling pll_init
    //    or pll init hangs waiting for OSC to initialize
    if(pmc_p->regsc & CYGHWR_HAL_KINETIS_PMC_REGSC_ACKISO_M)
        pmc_p->regsc |= CYGHWR_HAL_KINETIS_PMC_REGSC_ACKISO_M;

    hal_start_clocks();
#endif
}

//===========================================================================
// hal_misc_init
//===========================================================================
#define CYGHWR_HAL_KINETIS_SIM_SCGC5_PORT_M           \
            (CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTA_M |   \
             CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTB_M |   \
             CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTC_M |   \
             CYGHWR_HAL_KINETIS_SIM_SCGC5_PORTD_M)    \


static inline void CYGOPT_HAL_KINETIS_MISC_FLASH_SECTION_ATTR
hal_misc_init(void)
{
    cyghwr_hal_kinetis_sim_t *sim_p = CYGHWR_HAL_KINETIS_SIM_P;
    cyghwr_hal_kinetis_mpu_t *mpu_p = CYGHWR_HAL_KINETIS_MPU_P;

    // Enable some peripherals' clocks.
    sim_p->scgc5 |= CYGHWR_HAL_KINETIS_SIM_SCGC5_PORT_M;

    // Disable MPU
    mpu_p->cesr = 0;
    
    // Enable miscelciums GPIO pins
    hal_set_pin_function(CYGHWR_IO_FREESCALE_XTAL_PIN_EXTAL0);
    hal_set_pin_function(CYGHWR_IO_FREESCALE_XTAL_PIN_XTAL0);
    
    hal_set_pin_function(CYGHWR_IO_FREESCALE_LCD_A0);
    CYGHWR_IO_DIR_LCD_A0;
    CYGNUM_DEVS_FRAMEBUF_ST7565_A0_PIN_LOW;
    
    hal_set_pin_function(CYGHWR_IO_FRESCALE_PIN_NSHUTD);
    CYGHWR_IO_DIR_PIN_NSHUTD;
    CYGHWR_IO_CLEAR_PIN_NSHUTD;
    
    hal_set_pin_function(CYGHWR_IO_FRESCALE_PIN_ANL_ON);
    CYGHWR_IO_DIR_PIN_ANL_ON;
    CYGHWR_IO_CLEAR_PIN_ANL_ON;
    
    hal_set_pin_function(CYGHWR_IO_FRESCALE_PIN_LCD_BL);
    CYGHWR_IO_DIR_PIN_LCD_BL;
    CYGHWR_IO_CLEAR_PIN_LCD_BL;
    
    hal_set_pin_function(CYGHWR_IO_FRESCALE_PIN_PWOFF);
    CYGHWR_IO_DIR_PIN_PWOFF;
    CYGHWR_IO_CLEAR_PIN_PWOFF;
}


//==========================================================================

__externC void hal_platform_init( void )
{
}

//! CPU start up error flags
cyg_uint64	_hw_error = 0;


#ifndef BOOT_LOADER

#include <cyg/io/fm25vxx.h>

// Underlaying Freescale DSPI device
CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    cyg_spi_wallclock_pcf2129a,             // Device name
    0,                      // SPI bus
    CYGHWR_DEVS_RTC_PCF2129A_DEV0_SPI_CS,   // Dev num (CS)
    PCF2129A_SPI_FRAME_SIZE,                // Frame size
    PCF2129A_SPI_CLOCK_POL,                 // Clock pol
    PCF2129A_SPI_CLOCK_PHASE,               // Clock phase
    CYGHWR_DEVS_RTC_PCF2129A_DEV0_SPEED,    // Clock speed (Hz)
    CYGHWR_DEVS_RTC_PCF2129A_DEV0_CS_DLY,   // CS assert delay
    CYGHWR_DEVS_RTC_PCF2129A_DEV0_CS_DLY,   // CS negate delay
    CYGHWR_DEVS_RTC_PCF2129A_DEV0_CS_DLY,   // Delay between transfers
    CYGHWR_DEVS_RTC_PCF2129A_DEV0_CS_DLY_UN,// Delay unit (100 or 1000 ns)
    PCF2129A_SPI_DBR_DEV0                   // Use double baud rate
);



CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    lcd_spi_device,                         // Device name
    0,                      // SPI bus
    CYGHWR_DEVS_LCD_DEV1_SPI_CS,            // Dev num (CS)
    LCD_SPI_FRAME_SIZE,                     // Frame size
    LCD_SPI_CLOCK_POL,                      // Clock pol
    LCD_SPI_CLOCK_PHASE,                    // Clock phase
    CYGHWR_DEVS_LCD_DEV1_SPEED,             // Clock speed (Hz)
    CYGHWR_DEVS_LCD_DEV1_CS_DLY,            // CS assert delay
    CYGHWR_DEVS_LCD_DEV1_CS_DLY,            // CS negate delay
    CYGHWR_DEVS_LCD_DEV1_CS_DLY,            // Delay between transfers
    CYGHWR_DEVS_LCD_DEV1_CS_DLY_UN,         // Delay unit (100 or 1000 ns)
    LCD_SPI_DBR_DEV1                        // Use double baud rate
);

CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    fm25vxx_spi_dev0,                       // Device name
    0,                      // SPI bus
    CYGHWR_DEVS_FRAM_FM25VXX_DEV2_SPI_CS,   // Dev num (CS)
    FM25VXX_SPI_FRAME_SIZE,                 // Frame size
    FM25VXX_SPI_CLOCK_POL,                  // Clock pol
    FM25VXX_SPI_CLOCK_PHASE,                // Clock phase
    CYGHWR_DEVS_FRAM_FM25VXX_DEV2_SPEED,    // Clock speed (Hz)
    CYGHWR_DEVS_FRAM_FM25VXX_DEV2_CS_DLY,   // CS assert delay
    CYGHWR_DEVS_FRAM_FM25VXX_DEV2_CS_DLY,   // CS negate delay
    CYGHWR_DEVS_FRAM_FM25VXX_DEV2_CS_DLY,   // Delay between transfers
    CYGHWR_DEVS_FRAM_FM25VXX_DEV2_CS_DLY_UN,// Delay unit (100 or 1000 ns)
    FM25VXX_SPI_DBR_DEV2                    // Use double baud rate
);

CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    adc_spi_device,                         // Device name
    1,                      // SPI bus
    CYGHWR_DEVS_ADC_AD7124_DEV0_SPI_CS,     // Dev num (CS)
    AD7124_SPI_FRAME_SIZE,                  // Frame size
    AD7124_SPI_CLOCK_POL,                   // Clock pol
    AD7124_SPI_CLOCK_PHASE,                 // Clock phase
    CYGHWR_DEVS_ADC_AD7124_DEV0_SPEED,      // Clock speed (Hz)
    CYGHWR_DEVS_ADC_AD7124_DEV0_CS_DLY,     // CS assert delay
    CYGHWR_DEVS_ADC_AD7124_DEV0_CS_DLY,     // CS negate delay
    CYGHWR_DEVS_ADC_AD7124_DEV0_CS_DLY,     // Delay between transfers
    CYGHWR_DEVS_ADC_AD7124_DEV0_CS_DLY_UN,  // Delay unit (100 or 1000 ns)
    AD7124_SPI_DBR_DEV0                     // Use double baud rate
);

//externC cyg_spi_freescale_dspi_device_t adc_spi_device;

CYG_DEVS_SPI_FREESCALE_DSPI_DEVICE(
    dac_control_spi_device,                 // Device name
    1,                      // SPI bus
    CYGHWR_DEVS_DAC_TLV320_DEV1_SPI_CS,     // Dev num (CS)
    TLV320_SPI_FRAME_SIZE,                  // Frame size
    TLV320_SPI_CLOCK_POL,                   // Clock pol
    TLV320_SPI_CLOCK_PHASE,                 // Clock phase
    CYGHWR_DEVS_DAC_TLV320_DEV1_SPEED,      // Clock speed (Hz)
    CYGHWR_DEVS_DAC_TLV320_DEV1_CS_DLY,     // CS assert delay
    CYGHWR_DEVS_DAC_TLV320_DEV1_CS_DLY,     // CS negate delay
    CYGHWR_DEVS_DAC_TLV320_DEV1_CS_DLY,     // Delay between transfers
    CYGHWR_DEVS_DAC_TLV320_DEV1_CS_DLY_UN,  // Delay unit (100 or 1000 ns)
    TLV320_SPI_DBR_DEV1                     // Use double baud rate
);

//externC cyg_spi_freescale_dspi_device_t dac_control_spi_device;
/**
* Definition of FLASH drivers structure for FLASH and FRAM
*/
CYG_DEVS_FLASH_SPI_FM25VXX_DRIVER(fm25wxx_spi_fram,FRAM_FM25WXX_BASE_ADDRESS,&fm25vxx_spi_dev0);

#endif

//==========================================================================

#ifdef CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS

#include CYGHWR_MEMORY_LAYOUT_H

//--------------------------------------------------------------------------
// Accesses to areas not backed by real devices or memory can cause
// the CPU to hang.
//
// The following table defines the memory areas that GDB is allowed to
// touch. All others are disallowed.
// This table needs to be kept up to date with the set of memory areas
// that are available on the board.

static struct {
    CYG_ADDRESS         start;          // Region start address
    CYG_ADDRESS         end;            // End address (last byte)
} hal_data_access[] =
{
    { CYGMEM_REGION_ram,        CYGMEM_REGION_ram+CYGMEM_REGION_ram_SIZE-1        }, // System bus RAM partition
#ifdef CYGMEM_REGION_sram
    { CYGMEM_REGION_sram,       CYGMEM_REGION_sram+CYGMEM_REGION_sram_SIZE-1      }, // On-chip SRAM
#elif defined CYGMEM_REGION_sram_l
    { CYGMEM_REGION_sram_l,     CYGMEM_REGION_sram_l+CYGMEM_REGION_sram_l_SIZE-1  }, // On-chip SRAM lower bank
#endif
#ifdef CYGMEM_REGION_ramcod
    { CYGMEM_REGION_ramcod,      CYGMEM_REGION_ramcod+CYGMEM_REGION_ramcod_SIZE-1 }, // Code bus RAM partition
#endif
#ifdef CYGMEM_REGION_ramnc
    { CYGMEM_REGION_ramnc,      CYGMEM_REGION_ramnc+CYGMEM_REGION_ramnc_SIZE-1    }, // Non cachable RAM partition
#endif
#ifdef CYGMEM_REGION_flash
    { CYGMEM_REGION_flash,      CYGMEM_REGION_flash+CYGMEM_REGION_flash_SIZE-1    }, // On-chip flash
#endif
#ifdef CYGMEM_REGION_rom
    { CYGMEM_REGION_rom,        CYGMEM_REGION_rom+CYGMEM_REGION_rom_SIZE-1        }, // External flash [currently none]
#endif
    { 0xE0000000,               0x00000000-1                                      }, // Cortex-M peripherals
    { 0x40000000,               0x60000000-1                                      }  // Chip specific peripherals
};

__externC int cyg_hal_stub_permit_data_access( void* addr, cyg_uint32 count )
{
    int i;
    for( i = 0; i < sizeof(hal_data_access)/sizeof(hal_data_access[0]); i++ ) {
        if( ((CYG_ADDRESS)addr >= hal_data_access[i].start) &&
            ((CYG_ADDRESS)addr+count) <= hal_data_access[i].end)
            return true;
    }
    return false;
}

#endif // CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS

//==========================================================================

#ifdef CYGPKG_REDBOOT
#include <redboot.h>
#include CYGHWR_MEMORY_LAYOUT_H

//--------------------------------------------------------------------------
// Memory layout
//
// We report the on-chip SRAM and external SRAM.

void
cyg_plf_memory_segment(int seg, unsigned char **start, unsigned char **end)
{
    switch (seg) {
    case 0:
        *start = (unsigned char *)CYGMEM_REGION_ram;
        *end = (unsigned char *)(CYGMEM_REGION_ram + CYGMEM_REGION_ram_SIZE);
        break;
#ifdef CYGMEM_REGION_sram
    case 1:
        *start = (unsigned char *)CYGMEM_REGION_sram;
        *end = (unsigned char *)(CYGMEM_REGION_sram + CYGMEM_REGION_sram_SIZE);
        break;
#endif
#ifdef CYGMEM_REGION_sram_l
# define CASE_CODE 3
# define CASE_RAMNC 4
    case 2:
        *start = (unsigned char *)CYGMEM_REGION_sram_l;
        *end = (unsigned char *)(CYGMEM_REGION_sram_l + CYGMEM_REGION_sram_l_SIZE);
        break;
#else
# define CASE_CODE 2
# define CASE_RAMNC 3
#endif

#ifdef CYGMEM_REGION_ramcod
    case CASE_CODE:
        *start = (unsigned char *)CYGMEM_REGION_ramcod;
        *end = (unsigned char *)(CYGMEM_REGION_ramcod + CYGMEM_REGION_ramcod_SIZE);
        break;
#endif

#ifdef CYGMEM_REGION_ramnc
    case CASE_RAMNC:
        *start = (unsigned char *)CYGMEM_REGION_ramnc;
        *end = (unsigned char *)(CYGMEM_REGION_ramnc + CYGMEM_REGION_ramnc_SIZE);
        break;
#endif
    default:
        *start = *end = NO_MEMORY;
        break;
    }
} // cyg_plf_memory_segment()

#endif // CYGPKG_REDBOOT

//==========================================================================
// EOF twr_k60f120m_misc.c
