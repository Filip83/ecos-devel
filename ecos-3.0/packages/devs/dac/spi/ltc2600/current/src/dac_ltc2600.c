//==========================================================================
//
//      dac_ltc2600.c
//
//      DAC driver for LTC2600
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2009, 2011 Free Software Foundation, Inc.
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
// Date:         2013-06-13
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/system.h>
#include <pkgconf/devs_dac_spi_ltc2600.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/io/spi.h>
#include <cyg/io/dac.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/dac_ltc2600.h>

// Enabling timing check may interfere with operation of the DAC. To use
// this remove J15 jumper and put a scope on pin 2.
#define DAC2600_TIMING_DEBUG 0

// Enabling causes driver to mask interrupts in the timing device instead
// of eCos.
#define DAC2600_HW_INT_MASK 1

//-----------------------------------------------------------------------------
// Diagnostic support
// Switch the #if to 1 to generate some diagnostic messages.

#ifdef CYGPKG_DEVS_DAC_CORTEXM_LTC2600_TRACE
# include <cyg/infra/diag.h>
# define dac_diag( __fmt, ... ) diag_printf("DAC: %30s[%4d]: " __fmt, __FUNCTION__, __LINE__, ## __VA_ARGS__ );
#else
# define dac_diag( __fmt, ... ) 
#endif

#define CYGHWR_HAL_KINETIS_SIM_SCGC6_FTM_(_index_) \
        CYGHWR_HAL_KINETIS_SIM_SCGC6_FTM ## _index_ ## _M

#define CYGHWR_HAL_KINETIS_SIM_SCGC6_FTM(_index_) \
        CYGHWR_HAL_KINETIS_SIM_SCGC6_FTM_(_index_)

//=============================================================================
// Define ltc2600 SPI protocol.
//=============================================================================

//-----------------------------------------------------------------------------
// API function call forward references
static void ltc2600_spi_send_recieve(cyg_spi_device *dev, struct ltc2600_settings *settings, cyg_uint16 data);

static cyg_uint32 ltc2600_tmr_isr(cyg_vector_t vector, cyg_addrword_t data);
static void ltc2600_tmr_dsr(cyg_vector_t vector, cyg_ucount32 count,
                          cyg_addrword_t data);

static void ltc2600_dac_init_clock(void);
static void ltc2600_dac_init_device(cyg_dac_device *device);

static cyg_bool initialized;

__externC cyg_uint32 hal_ltc2600_pclk1;
__externC cyg_uint32 hal_ltc2600_pclk2;

//-----------------------------------------------------------------------------
// Utility function to setup and fetch data via SPI

static void
ltc2600_spi_send_recieve(cyg_spi_device *dev, struct ltc2600_settings *settings, cyg_uint16 data)
{
    const cyg_uint8 tx_buf[3] = { 
        0x03                << 4 | // Write and update
        settings->select,          // Address
        (data & 0xFF00) >> 8, data & 0xFF };
    cyg_uint8       rx_buf[3];

    // Carry out SPI transfer.
    cyg_spi_transaction_transfer(dev, true, 3, tx_buf, rx_buf, true);
}

//-----------------------------------------------------------------------------
// This function is called from the device IO infrastructure to initialize the
// device. It should perform any work needed to start up the device, short of
// actually starting the generation of output. This function will be called
// for each channel, so if there is initialization that only needs to be done
// once, such as creating and interrupt object, then care should be taken to do
// this. This function should also call cyg_dac_device_init() to initialize the
// generic parts of the driver.

bool
ltc2600_dac_init(struct cyg_devtab_entry *tab)
{
    cyg_dac_channel *chan = (cyg_dac_channel *) tab->priv;
    cyg_dac_device *device = chan->device;
    ltc2600_dac_info *info = device->dev_priv;
    
    dac_diag("Initializing device\n");
    
    // Initialize DAC clock
    if (!initialized) {
        ltc2600_dac_init_clock();
        initialized = true;
    }
    
    // Keep reference to channel
    info->chan[chan->channel] = chan;

    // Set a default mask
    info->chan_mask = 0x0000;

    // Initialize DAC device
    ltc2600_dac_init_device(device);

    // Set default rate
    ltc2600_dac_set_rate(chan, chan->device->config.rate);
            
    // Initialize TMR interrupt
    cyg_drv_interrupt_create(info->setup->tmr_int_vector,
                             info->setup->tmr_int_pri,
                            (cyg_addrword_t) device,
                            &ltc2600_tmr_isr,
                            &ltc2600_tmr_dsr,
                            &info->tmr_int_handle,
                            &info->tmr_int_data);
    cyg_drv_interrupt_attach(info->tmr_int_handle);
    cyg_drv_interrupt_unmask(info->setup->tmr_int_vector);

    // Initialize generic parts of DAC device
    cyg_dac_device_init(device);
    
    return true;
}

//-----------------------------------------------------------------------------
// This function is called when a client looks up or opens a channel. It should
// call cyg_dac_channel_init() to initialize the generic part of the channel.
// It should also perform any operations needed to start the channel generating
// output.

Cyg_ErrNo
ltc2600_dac_lookup(struct cyg_devtab_entry **tab,
                 struct cyg_devtab_entry *sub_tab,
                 const char *name)
{
    cyg_dac_channel *chan = (cyg_dac_channel *) (*tab)->priv;

    dac_diag("Opening device\n");
    
    // Initialize generic parts of the channel
    cyg_dac_channel_init(chan);

    // The generic DAC manual says: When a channel is first looked up or 
    // opened, then it is automatically enabled and data starts to
    // accumulate - so we start the channel now
    chan->enabled = true;
    ltc2600_dac_enable(chan);

    return ENOERR;
}

//-----------------------------------------------------------------------------
// This function is called from the generic DAC package to enable the channel
// in response to a CYG_IO_SET_CONFIG_DAC_ENABLE config operation. It should
// take any steps needed to start the channel generating output.

void
ltc2600_dac_enable(cyg_dac_channel *chan)
{
    ltc2600_dac_info *info = chan->device->dev_priv;
    cyg_uint32 reg;
    cyg_bool start;

    dac_diag("Enabling channel\n");

    start = !info->chan_mask;

    // Update the mask
    info->chan_mask |= (1 << chan->channel);
    
    // Start scanning when first channel was activated
    if (start) {
        // Enable timer
        dac_diag("Starting generation\n");
        // Enable the system clock and enable interrupts
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
        HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
        // Enable the PDB and load the registers at the same time. This is here in
        // case set_rate was called when the PDB was disabled.
        reg |= CYGHWR_DEV_FREESCALE_PDB_SC_PDBEN | CYGHWR_DEV_FREESCALE_PDB_SC_LDOK;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
        // Trigger the PDB so that it will run.
        reg |= CYGHWR_DEV_FREESCALE_PDB_SC_SWTRIG;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
#else
        reg = CYGHWR_DEV_FREESCALE_FTM_SC_TOIE | 0x01 << 3;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
#endif
    }
}

//-----------------------------------------------------------------------------
// This function is called from the generic DAC package to disable the channel
// in response to a CYG_IO_SET_CONFIG_DAC_DISABLE config operation. It should
// take any steps needed to stop the channel generating output.

void
ltc2600_dac_disable(cyg_dac_channel *chan)
{
    ltc2600_dac_info *info = chan->device->dev_priv;
    cyg_uint32 reg;
    
    dac_diag("Disabling channel\n");
    
    // Update the mask
    info->chan_mask &= ~(1 << chan->channel);
    
    // Stop scanning when no channel is active
    if (!info->chan_mask) {
        // Disable timer
        dac_diag("Stopping generation\n");
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
        HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
	reg &= ~CYGHWR_DEV_FREESCALE_PDB_SC_PDBEN;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);

#else
        // Disable the clock and interrupts
        reg = 0;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
#endif
    }
}

//-----------------------------------------------------------------------------
// This function is called from the generic DAC package to enable the channel
// in response to a CYG_IO_SET_CONFIG_DAC_RATE config operation. It should take
// any steps needed to change the output rate of the channel, or of the entire
// device. We use a timer channel to generate the interrupts for outputting the
// analog channel.

void
ltc2600_dac_set_rate( cyg_dac_channel *chan, cyg_uint32 rate)
{
    cyg_dac_device *device = chan->device;
    ltc2600_dac_info *info = device->dev_priv;
    cyg_uint32 clock;
    cyg_uint32 divider, counter;
    cyg_uint32 reg;

    dac_diag("Setting rate to %d\n", rate);

    device->config.rate = rate;
    
    // Get the system base clock speed.
    clock = CYGHWR_HAL_CORTEXM_KINETIS_CLK_PER_BUS;
    
    // The desired ideal divider ratio
    divider = clock / rate;

#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
    // Find the largest multiplier possible.
    cyg_uint32 multiplier_code, prescaler_code;
    cyg_uint32 multiplier, prescaler;

    multiplier_code = 0x03;
    prescaler_code = 0x07;

    multiplier = multiplier_code == 0 ? 1 : 10 * (1 << (multiplier_code-1));
    while (multiplier_code > 0 && multiplier > divider)
    {
        multiplier_code--;
        multiplier = multiplier_code == 0 ? 1 : 10 * (1 << (multiplier_code-1));
    }

    // The new target divider. This will generate rounding error.
    // May want to fix the divider at 2 if possible.
    divider /= multiplier;

    // Find the largest prescaler possible.
    prescaler = 1 << prescaler_code;
    while (prescaler_code > 0 && prescaler > divider)
    {
        prescaler_code--;
        prescaler = 1 << prescaler_code;
    }

    divider /= prescaler;

    // Find the counter value closest to the divider value without over flowing;
    if (divider > 0xFFFF)
        counter = 0xFFFF;
    else
        counter = divider;

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_MOD, counter);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_IDLY, counter);

    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
    reg = (reg & ~CYGHWR_DEV_FREESCALE_PDB_SC_PRESCALER) | (prescaler_code << 12);
    reg = (reg & ~CYGHWR_DEV_FREESCALE_PDB_SC_MULT) | (multiplier_code << 2);
    // This will only apply if the PDB is already enabled.
    reg |= CYGHWR_DEV_FREESCALE_PDB_SC_LDOK;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
#else
    // Find the counter value closest to the divider value without over flowing;
    if (divider > 0xFFFF)
        counter = 0xFFFF;
    else
        counter = divider;

    // Load the MOD register to set the interrupt frequency.
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_MOD, counter);

    // Sync the MOD register with a software trigger.
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SYNC, reg);
    reg |= CYGHWR_DEV_FREESCALE_FTM_SYNC_SWSYNC;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SYNC, reg);
#endif
}
//-----------------------------------------------------------------------------
// This function is called from the generic DAC package to set the DAC gain
// in response to a CYG_IO_SET_CONFIG_DAC_GAIN config operation.
void
ltc2600_dac_set_gain( cyg_dac_channel *chan, cyg_uint32 gain)
{
}

//-----------------------------------------------------------------------------
// This function is called from the generic DAC package to set the DAC polarity
// in response to a CYG_IO_SET_CONFIG_DAC_POLARITY config operation.
void
ltc2600_dac_set_polarity( cyg_dac_channel *chan, cyg_uint32 polarity)
{
}

//-----------------------------------------------------------------------------
// This function is the ISR attached to the FTM or PDB.

static cyg_uint32
ltc2600_tmr_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_dac_device *device = (cyg_dac_device *) data;
    ltc2600_dac_info *info = (ltc2600_dac_info *) device->dev_priv;
    ltc2600_dac_setup *setup = (ltc2600_dac_setup *) info->setup;
    cyg_uint32 reg;

#if DAC2600_HW_INT_MASK
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
        HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
        reg &= ~CYGHWR_DEV_FREESCALE_PDB_SC_PDBIE;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
#else
        HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
        reg &= ~CYGHWR_DEV_FREESCALE_FTM_SC_TOIE;
        HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
#endif
#else
    cyg_interrupt_mask(setup->tmr_int_vector);
#endif
    cyg_drv_interrupt_acknowledge(setup->tmr_int_vector);
    
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
    // Clear the PDB interrupt flag.
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
    reg &= ~CYGHWR_DEV_FREESCALE_PDB_SC_PDBIF;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);

#else
    // Clear the FTM timer overflow interrupt flag.
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
    reg &= ~CYGHWR_DEV_FREESCALE_FTM_SC_TOF;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
#endif

    return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
}

//-----------------------------------------------------------------------------
// This function is the DSR attached to the FTM or PDB.

static void
ltc2600_tmr_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_dac_device *device = (cyg_dac_device *) data;
    ltc2600_dac_info *info = (ltc2600_dac_info *) device->dev_priv;
    ltc2600_dac_setup *setup = (ltc2600_dac_setup *) info->setup;
    cyg_uint32 chan_active = info->chan_mask;
    cyg_dac_channel **chan = info->chan;
    cyg_dac_output_t output;
    cyg_uint32 res = 0;
    cyg_uint32 res_tot = CYG_ISR_HANDLED;
    cyg_uint32 reg;

#if DAC2600_TIMING_DEBUG
    // Toggle the gpio pin to measure the update rate with a scope.
    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(E, 5);
    HAL_DELAY_US(10);
    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(E, 5);
#endif

    while (chan_active) {
        if (chan_active & 0x1) {
            res = cyg_dac_take_output(*chan, &output);
            res_tot |= res;
#if !DAC2600_TIMING_DEBUG
            if (!(res & CYG_ISR_CALL_DSR)) {
                cyg_spi_transaction_begin(info->spi_dev);
                // Select our CS.
                CYGHWR_HAL_KINETIS_GPIO_SET_PIN(E, 27);
                CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(E, 5);
                ltc2600_spi_send_recieve(info->spi_dev, &setup->channel_settings[(*chan)->channel], output);
                cyg_spi_transaction_end(info->spi_dev);
            }
#endif
        }
        chan_active >>= 1;
        chan++;
    }

    if (res_tot & CYG_ISR_CALL_DSR)
        while (chan_active) {
            if (chan_active & 0x1)
                if ((*chan)->wakeup)
                    cyg_dac_wakeup(*chan);
            chan_active >>= 1;
            chan++;
        }

#if DAC2600_HW_INT_MASK
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
    reg |= CYGHWR_DEV_FREESCALE_PDB_SC_PDBIE;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
#else
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
    reg |= CYGHWR_DEV_FREESCALE_FTM_SC_TOIE;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);
#endif
#else
    cyg_interrupt_unmask(setup->tmr_int_vector);
#endif
}


//-----------------------------------------------------------------------------
// Initializes the DAC system clock.

static void
ltc2600_dac_init_clock(void)
{
#ifdef CYGHWR_DEVS_DAC_LTC2600_DAC1
    // Enable FTM clock via clock gating. This is never turned off.
    cyghwr_hal_kinetis_sim_t *sim_p = CYGHWR_HAL_KINETIS_SIM_P;
#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
    sim_p->scgc6 |= CYGHWR_HAL_KINETIS_SIM_SCGC6_PDB_M;
#else
    sim_p->scgc6 |= CYGHWR_HAL_KINETIS_SIM_SCGC6_FTM(CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR);
#endif
#endif
}

//-----------------------------------------------------------------------------
// Initializes an DAC device.

static void
ltc2600_dac_init_device(cyg_dac_device *device)
{
    ltc2600_dac_info *info = device->dev_priv;
    cyg_uint32 reg;

    // Setup the FTM.

#if (CYGNUM_DEVS_DAC_LTC2600_DAC1_TMR == 2)
    reg = 0x00000000;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_MOD, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_IDLY, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH0C1, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH0S, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH0DLY0, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH0DLY1, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH1C1, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH1S, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH1DLY0, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH1DLY1, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH2C1, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH2S, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH2DLY0, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH2DLY1, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH3C1, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH3S, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH3DLY0, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_CH3DLY1, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_DACINTC0, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_DACINT0, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_DACINTC1, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_DACINT1, reg);

    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_POEN, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_PO0DLY, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_PO1DLY, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_PO2DLY, reg);
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_PO3DLY, reg);

    reg = CYGHWR_DEV_FREESCALE_PDB_SC_TRGSEL | 
          CYGHWR_DEV_FREESCALE_PDB_SC_PDBIE  | 
          CYGHWR_DEV_FREESCALE_PDB_SC_CONT;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_PDB_SC, reg);
#else
    // No clock or interrupts by default.
    reg = 0;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SC, reg);

    // Disable write protect
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_MODE, reg);
    reg |= CYGHWR_DEV_FREESCALE_FTM_MODE_WPDIS;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_MODE, reg);
    // Enable FTM registers
    reg |= CYGHWR_DEV_FREESCALE_FTM_MODE_FTMEN;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_MODE, reg);
    // Enable write protect
    // The reference manual says to lock, read a one, set a one.
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_MODE, reg);
    reg |= CYGHWR_DEV_FREESCALE_FTM_MODE_WPDIS;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_MODE, reg);

    // Enable loading MOD and CNT registers.
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_PWMLOAD, reg);
    reg |= CYGHWR_DEV_FREESCALE_FTM_PWMLOAD_LDOK;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_PWMLOAD, reg);

    // Enable software triggering for MOD register update.
    HAL_READ_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SYNCONF, reg);
    reg |= CYGHWR_DEV_FREESCALE_FTM_SYNCONF_SWWRBUF;
    HAL_WRITE_UINT32(info->setup->dac_base + CYGHWR_DEV_FREESCALE_FTM_SYNCONF, reg);
#endif    

#if DAC2600_TIMING_DEBUG
    // Setup the GPIOB pins for debug.
    hal_set_pin_function(CYGHWR_HAL_KINETIS_PIN_CFG(E, 5, 0x1, 0x0, 0x0));
    CYGHWR_HAL_KINETIS_GPIO_SET_PIN(E, 5);
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(E, 5);
#else
    // Setup GPIO pins to control the CS mux.
    hal_set_pin_function(CYGHWR_HAL_KINETIS_PIN_CFG(E, 27, 0x1, 0x0, 0x0));
    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(E, 27);
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(E, 27);
    hal_set_pin_function(CYGHWR_HAL_KINETIS_PIN_CFG(E, 5, 0x1, 0x0, 0x0));
    CYGHWR_HAL_KINETIS_GPIO_CLEAR_PIN(E, 5);
    CYGHWR_HAL_KINETIS_GPIO_PIN_DDR_OUT(E, 5);
#endif

}

// End of dac_ltc2600.c
