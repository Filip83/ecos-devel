//==========================================================================
//
//      adc_kinetis.c
//
//      ADC driver for Freescale Kinetis on chip ADC
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
// Author(s):   Filip
// Contributors:
// Date:         2017-03-31
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================


//==========================================================================
//                                 INCLUDES
//==========================================================================
#include <cyg/hal/var_io.h>
#include <cyg/hal/var_gpio.h>

#include <pkgconf/system.h>
#include <pkgconf/devs_adc_kinetis.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/adc.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include "adc_hdr.h"

#if CYGPKG_DEVS_ADC_KINETIS_DEBUG_LEVEL > 0
   #define adc_printf(args...) diag_printf(args)
#else
   #define adc_printf(args...)
#endif

#define AVR32_MAX_ADC_CHAN                  9 //8 + temp sensor


#define BUS     0
#define ALTCK2  1
#define ALTCK   2
#define ADACK   3

#define T24ADCK 0
#define T16ADCK 1
#define T10ADCK 2
#define T6ADCK  3

#define R8BITS  0
#define R12BITS 1
#define R10BITS 2
#define R16BITS 3

#define DIV1    0
#define DIV2    1
#define DIV4    2
#define DIV8    3


//==========================================================================
//                                  DATA TYPES
//==========================================================================
typedef struct kinetis_adc_info
{
    volatile cyghwer_io_kinetis_adc_t *adc_base;         // base address of ADC peripheral
    cyg_vector_t            adc_vector;        // interrupt vector number
    int                     adc_intprio;       // interrupt priority of ADC interrupt
    cyg_uint32              timer_cnt;         // Timer value
    cyg_handle_t            int_handle;        // For initializing the interrupt
    cyg_interrupt           int_data;
    struct cyg_adc_channel  *channel[1]; // stores references to channel objects
    cyg_uint8               num_active_ch;     // Number of currently active chanels
    cyg_uint16              chan_mask;         // mask that indicates channels used
                                               // by ADC driver
} kinetis_adc_info;


//==========================================================================
//                               DECLARATIONS
//==========================================================================
static bool kinetis_adc_init(struct cyg_devtab_entry *tab);
static Cyg_ErrNo kinetis_adc_lookup(struct cyg_devtab_entry **tab,
                                    struct cyg_devtab_entry  *sub_tab,
                                    const char               *name);
static void kinetis_adc_enable( cyg_adc_channel *chan );
static void kinetis_adc_disable( cyg_adc_channel *chan );
static void kinetis_adc_set_rate( cyg_adc_channel *chan, cyg_uint32 rate );
static cyg_uint32 kinetis_adc_isr(cyg_vector_t vector, cyg_addrword_t data);
static void kinetis_adc_dsr(cyg_vector_t vector,
                            cyg_ucount32 count,
                            cyg_addrword_t data);

// -------------------------------------------------------------------------
// Driver functions:
CYG_ADC_FUNCTIONS(kinetis_adc_funs,
                  kinetis_adc_enable,
                  kinetis_adc_disable,
                  kinetis_adc_set_rate,
                  kinetis_adc_set_gain,
                  kinetis_adc_set_polarity,
                  kinetis_adc_set_mode);

#include CYGDAT_DEVS_ADC_KINETIS_INL // Instantiate ADCs


static void kinetis_adc_calibration(kinetis_adc_info *info)
{
    volatile cyghwer_io_kinetis_adc_t *base = info->adc_base;
    
    volatile cyg_uint32 tmp32; /* 'volatile' here is for the dummy read of ADCx_R[0] register. */

    /* Clear the CALF and launch the calibration. */
    base->SC3 |= ADC_SC3_CAL_MASK | ADC_SC3_CALF_MASK;
    while ((base->SC1[0] & ADC_SC1_COCO_MASK) == 0)
    {
        /* Check the CALF when the calibration is active. */
        if (base->SC3 & ADC_SC3_CALF_MASK)
        {
            CYG_FAIL("ADC calibration failed.\n");
            return;
        }
    }
    tmp32 = base->R[0]; /* Dummy read to clear COCO caused by calibration. */

    /* Check the CALF at the end of calibration. */
    if (base->SC3 & ADC_SC3_CALF_MASK)
    {
        CYG_FAIL("ADC calibration failed on completition.\n");
        return;
    }

    /* Calculate the calibration values. */
    tmp32 = base->CLP0 + base->CLP1 + base->CLP2 + base->CLP3 + base->CLP4 + base->CLPS;
    tmp32 = 0x8000U | (tmp32 >> 1U);
    base->PG = tmp32;

    tmp32 = base->CLM0 + base->CLM1 + base->CLM2 + base->CLM3 + base->CLM4 + base->CLMS;
    tmp32 = 0x8000U | (tmp32 >> 1U);
    base->MG = tmp32;

}

//==========================================================================
// This function is called from the device IO infrastructure to initialize
// the device. It should perform any work needed to start up the device,
// short of actually starting the generation of samples. This function will
// be called for each channel, so if there is initialization that only needs
// to be done once, such as creating and interrupt object, then care should
// be taken to do this. This function should also call cyg_adc_device_init()
// to initialize the generic parts of the driver.
//==========================================================================
static bool kinetis_adc_init(struct cyg_devtab_entry *tab)
{
    cyg_uint32 reg;
    cyg_adc_channel *chan   = (cyg_adc_channel *)tab->priv;
    cyg_adc_device *device  = chan->device;
    kinetis_adc_info *info    = device->dev_priv;
    volatile cyghwer_io_kinetis_adc_t *adc_dev = info->adc_base;
    

    if (!info->int_handle)
    {
        //Init ADC IO map
        //Init reference inputs those can not be multiplexed on other
        //IO pins
        //Configuration of io pins for slected channels
        
        CYGHWR_IO_CLOCK_ENABLE(CYGHWR_HAL_KINETIS_SIM_SCGC_ADC0);

        cyg_drv_interrupt_create(info->adc_vector,
                                 info->adc_intprio,
                                (cyg_addrword_t)device,
                                &kinetis_adc_isr,
                                &kinetis_adc_dsr,
                                &(info->int_handle),
                                &(info->int_data));
        cyg_drv_interrupt_attach(info->int_handle);

        // Enable ADC
#if CYGNUM_DEVS_ADC_KINETIS_ENABLE_LONG_SAMPLE_TIME == 1
        reg = ADC_CFG1_ADICLK(CYGNUM_DEVS_ADC_KINETIS_CLOCK_SOURCE) | 
              ADC_CFG1_MODE(CYGNUM_DEVS_ADC_KINETIS_RESOLUTION )    |
              ADC_CFG1_ADLSMP_MASK                                  |
              ADC_CFG1_ADIV(CYGNUM_DEVS_ADC_KINETIS_CLOCK_DIVIDE);
#else  
        reg = ADC_CFG1_ADICLK(CYGNUM_DEVS_ADC_KINETIS_CLOCK_SOURCE) | 
              ADC_CFG1_MODE(CYGNUM_DEVS_ADC_KINETIS_RESOLUTION )    |
              ADC_CFG1_ADIV(CYGNUM_DEVS_ADC_KINETIS_CLOCK_DIVIDE);
#endif
        adc_dev->CFG1 = reg;
                
        reg = adc_dev->CFG2 & ~(ADC_CFG2_ADACKEN_MASK | 
                             ADC_CFG2_ADHSC_MASK   | 
                             ADC_CFG2_ADLSTS_MASK);
        
#if CYGNUM_DEVS_ADC_KINETIS_ENABLE_LONG_SAMPLE_TIME == 1
        reg |= ADC_CFG2_ADLSTS(CYGNUM_DEVS_ADC_KINETIS_SELECT_LONG_SAMPLE_TIME);
#endif
        
#if CYGNUM_DEVS_ADC_KINETIS_ENABLE_LOW_POWER == 0
#if CYGNUM_DEVS_ADC_KINETIS_ENABLE_LONG_SAMPLE_TIME == 1
        reg |= ADC_CFG2_ADHSC_MASK;
#endif
#endif
        
#if CYGNUM_DEVS_ADC_KINETIS_ENABLE_ASYNC_CLOCK == 1
        reg |= ADC_CFG2_ADACKEN_MASK;
#endif
        adc_dev->CFG2 = reg;
        
        kinetis_adc_calibration(info);
       //
       // setup the default sample rate
       //
       //kinetis_adc_set_rate(chan, chan->device->config.rate);


    } // if (!info->int_handle)

    cyg_adc_device_init(device); // initialize generic parts of driver

    return true;
}


//==========================================================================
// This function is called when a client looks up or opens a channel. It
// should call cyg_adc_channel_init() to initialize the generic part of
// the channel. It should also perform any operations needed to start the
// channel generating samples.
//==========================================================================
static Cyg_ErrNo kinetis_adc_lookup(struct cyg_devtab_entry **tab,
                                    struct cyg_devtab_entry  *sub_tab,
                                    const char               *name)
{
    cyg_adc_channel  *chan     = (cyg_adc_channel *)(*tab)->priv;
    kinetis_adc_info *info     = chan->device->dev_priv;

    info->channel[chan->channel] = chan;
    cyg_adc_channel_init(chan); // initialize generic parts of channel

    //
    // The generic ADC manual says: When a channel is first looked up or
    // opened, then it is automatically enabled and samples start to
    // accumulate - so we start the channel now
    //
    chan->enabled = true;
    kinetis_adc_enable(chan);

    return ENOERR;
}


//==========================================================================
// This function is called from the generic ADC package to enable the
// channel in response to a CYG_IO_SET_CONFIG_ADC_ENABLE config operation.
// It should take any steps needed to start the channel generating samples
//==========================================================================
static void kinetis_adc_enable(cyg_adc_channel *chan)
{

    kinetis_adc_info *info      = chan->device->dev_priv;

    volatile kinetis_adcifb_t *base = info->adc_base;

    uint32_t sc1 = ADC_SC1_ADCH(CYGDAT_DEVS_ADC_KINETIS_CHANNEL_SOURCE); /* Set the channel number. */

    sc1 |= ADC_SC1_AIEN_MASK;
    base->SC1[0] = sc1;
    cyg_drv_interrupt_unmask(info->adc_vector);
    /* Enable chanell and sample */
}


//==========================================================================
// This function is called from the generic ADC package to enable the
// channel in response to a CYG_IO_SET_CONFIG_ADC_DISABLE config operation.
// It should take any steps needed to stop the channel generating samples.
//==========================================================================
static void kinetis_adc_disable(cyg_adc_channel *chan)
{
    kinetis_adc_info *info  = chan->device->dev_priv;

    volatile kinetis_adcifb_t *base = info->adc_base;
}


//==========================================================================
// This function is called from the generic ADC package to enable the
// channel in response to a CYG_IO_SET_CONFIG_ADC_RATE config operation.
// It should take any steps needed to change the sample rate of the channel,
// or of the entire device.
// We use a timer channel to generate the interrupts for sampling the
// analog channels. Sampling rate is number of samples per second.
//==========================================================================
static void kinetis_adc_set_rate( cyg_adc_channel *chan, cyg_uint32 rate)
{
    cyg_adc_device   *device = chan->device;
    kinetis_adc_info *info   = (kinetis_adc_info *)device->dev_priv;
    volatile kinetis_adcifb_t *base = info->adc_base;

}

//-----------------------------------------------------------------------------
// This function is called from the generic ADC package to set the ADC gain
// in response to a CYG_IO_SET_CONFIG_ADC_GAIN config operation.
void
kinetis_adc_set_gain( cyg_adc_channel *chan, cyg_uint32 gain)
{
}

//-----------------------------------------------------------------------------
// This function is called from the generic ADC package to set the ADC polarity
// in response to a CYG_IO_SET_CONFIG_ADC_POLARITY config operation.
void
kinetis_adc_set_polarity( cyg_adc_channel *chan, cyg_uint32 polarity)
{
}

//-----------------------------------------------------------------------------
// This function is called from the generic ADC package to set the ADC polarity
// in response to a CYG_IO_SET_CONFIG_ADC_POLARITY config operation. Two
// polarities are supported: unipolar and bipolar.
void
kinetis_adc_set_mode( cyg_adc_channel *chan, cyg_uint32 mode)
{
}


//==========================================================================
// This function is the ISR attached to the ADC device's interrupt vector.
// It is responsible for reading samples from the channels and passing them
// on to the generic layer. It needs to check each channel for data, and call
// cyg_adc_receive_sample() for each new sample available, and then ready the
// device for the next interrupt.
//==========================================================================
static cyg_uint32 kinetis_adc_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_adc_device   *device = (cyg_adc_device *) data;
    kinetis_adc_info *info   = (kinetis_adc_info *)device->dev_priv;
    volatile kinetis_adcifb_t *base = info->adc_base;
    cyg_uint32        res = 0;

    cyg_uint32    result = base->R[0];

    cyg_drv_interrupt_mask(info->adc_vector);
    cyg_drv_interrupt_acknowledge(info->adc_vector);
    
    res |= CYG_ISR_HANDLED
        |  cyg_adc_receive_sample(info->channel[0], result&0xffff);


    adc_dev->icr = AVR32_ADCIFB_ICR_DRDY_MASK;

    return 0;
}


//==========================================================================
// This function is the DSR attached to the ADC device's interrupt vector.
// It is called by the kernel if the ISR return value contains the
// CYG_ISR_HANDLED bit. It needs to call cyg_adc_wakeup() for each channel
// that has its wakeup field set.
//==========================================================================
static void kinetis_adc_dsr(cyg_vector_t vector,
                            cyg_ucount32 count,
                            cyg_addrword_t data)
{
    cyg_adc_device *device         = (cyg_adc_device *) data;
    kinetis_adc_info *info           = device->dev_priv;
    volatile kinetis_adcifb_t *adc_dev = info->adc_base;
    cyg_uint8      active_channels = 1;
    cyg_uint8      chan_no         = 0;

    while (active_channels)
    {
        if (active_channels & 0x01)
        {
            if(info->channel[chan_no]->wakeup)
            {
                cyg_adc_wakeup(info->channel[chan_no]);
            }
        }
        chan_no++;
        active_channels >>= 1;
    }
}

//---------------------------------------------------------------------------
// eof adc_adcifb.c
