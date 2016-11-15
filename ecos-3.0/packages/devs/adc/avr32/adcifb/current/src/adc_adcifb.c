//==========================================================================
//
//      adc_adcifb.c
//
//      ADC driver for AVR32 on chip ADC
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
// Date:         2016-11-01
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================


//==========================================================================
//                                 INCLUDES
//==========================================================================
#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>

#include <pkgconf/system.h>
#include <pkgconf/devs_adc_avr32_adcifb.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/adc.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>



#if CYGPKG_DEVS_ADC_AVR32_ADCIFB_DEBUG_LEVEL > 0
   #define avr32_adc_printf(args...) diag_printf(args)
#else
   #define avr32_adc_printf(args...)
#endif

#define AVR32_MAX_ADC_CHAN                  9 //8 + temp sensor


#define CYGNUM_ADC_CLK_FREQ    1000000*CYGHWR_HAL_AVR32_CPU_FREQ
#define AVR32_ADC_CHER_CHx(_ch_)  (0x1 << _ch_)
#define AVR32_ADC_CHER_CDRx(_ch_) (_ch_ << 2)



//==========================================================================
//                                  DATA TYPES
//==========================================================================
typedef struct avr32_adc_info
{
    volatile avr32_adcifb_t *adc_base;         // base address of ADC peripheral
    cyg_uint16              adc_prescal;       // ADC prescal value
    cyg_uint8               adc_startup_time;  // ADC Startup Time value
    cyg_uint8               adc_shtim;         // ADC SHTIM value
    cyg_vector_t            adc_vector;        // interrupt vector number
    int                     adc_intprio;       // interrupt priority of ADC interrupt
    cyg_uint32              timer_cnt;         // Timer value
    cyg_uint8               resolution;
    cyg_handle_t            int_handle;        // For initializing the interrupt
    cyg_interrupt           int_data;
    struct cyg_adc_channel  *channel[AVR32_MAX_ADC_CHAN]; // stores references to channel objects
    cyg_uint8               num_active_ch;     // Number of currently active chanels
    cyg_uint16              chan_mask;         // mask that indicates channels used
                                               // by ADC driver
} avr32_adc_info;


//==========================================================================
//                               DECLARATIONS
//==========================================================================
static bool avr32_adc_init(struct cyg_devtab_entry *tab);
static Cyg_ErrNo avr32_adc_lookup(struct cyg_devtab_entry **tab,
                                    struct cyg_devtab_entry  *sub_tab,
                                    const char               *name);
static void avr32_adc_enable( cyg_adc_channel *chan );
static void avr32_adc_disable( cyg_adc_channel *chan );
static void avr32_adc_set_rate( cyg_adc_channel *chan, cyg_uint32 rate );
static cyg_uint32 avr32_adc_isr(cyg_vector_t vector, cyg_addrword_t data);
static void avr32_adc_dsr(cyg_vector_t vector,
                            cyg_ucount32 count,
                            cyg_addrword_t data);

// -------------------------------------------------------------------------
// Driver functions:
CYG_ADC_FUNCTIONS( avr32_adc_funs,
                   avr32_adc_enable,
                   avr32_adc_disable,
                   avr32_adc_set_rate );


#include CYGDAT_DEVS_ADC_AVR32_ADCIFB_INL // Instantiate ADCs



//==========================================================================
// This function is called from the device IO infrastructure to initialize
// the device. It should perform any work needed to start up the device,
// short of actually starting the generation of samples. This function will
// be called for each channel, so if there is initialization that only needs
// to be done once, such as creating and interrupt object, then care should
// be taken to do this. This function should also call cyg_adc_device_init()
// to initialize the generic parts of the driver.
//==========================================================================
static bool avr32_adc_init(struct cyg_devtab_entry *tab)
{
    cyg_adc_channel *chan   = (cyg_adc_channel *)tab->priv;
    cyg_adc_device *device  = chan->device;
    avr32_adc_info *info    = device->dev_priv;
    volatile avr32_adcifb_t *adc_dev = info->adc_base;
    

    if (!info->int_handle)
    {
        //Init ADC IO map
        //Init reference inputs those can not be multiplexed on other
        //IO pins
        //Configuration of io pins for slected channels
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL0
        gpio_enable_module_pin(AVR32_ADCIFB_AD_0_PIN, AVR32_ADCIFB_AD_0_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL1
        gpio_enable_module_pin(AVR32_ADCIFB_AD_1_PIN, AVR32_ADCIFB_AD_1_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL2
        gpio_enable_module_pin(AVR32_ADCIFB_AD_2_PIN, AVR32_ADCIFB_AD_2_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL3
        gpio_enable_module_pin(AVR32_ADCIFB_AD_3_PIN, AVR32_ADCIFB_AD_3_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL4
        gpio_enable_module_pin(AVR32_ADCIFB_AD_4_PIN, AVR32_ADCIFB_AD_4_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL5
        gpio_enable_module_pin(AVR32_ADCIFB_AD_5_PIN, AVR32_ADCIFB_AD_5_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL6
        gpio_enable_module_pin(AVR32_ADCIFB_AD_6_PIN, AVR32_ADCIFB_AD_6_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL7
        gpio_enable_module_pin(AVR32_ADCIFB_AD_7_PIN, AVR32_ADCIFB_AD_7_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_ADCIFB_ADC0_CHANNEL8
        gpio_enable_module_pin(AVR32_ADCIFB_AD_8_PIN, AVR32_ADCIFB_AD_8_FUNCTION);
        #endif

        cyg_drv_interrupt_create(info->adc_vector,
                                 info->adc_intprio,
                                (cyg_addrword_t)device,
                                &avr32_adc_isr,
                                &avr32_adc_dsr,
                                &(info->int_handle),
                                &(info->int_data));
        cyg_drv_interrupt_attach(info->int_handle);

        // Enable ADC
        adc_dev->cr = AVR32_ADCIFB_CR_EN_MASK | AVR32_ADCIFB_CR_SWRST_MASK;
        while(!(adc_dev->sr & AVR32_ADCIFB_SR_EN_MASK));
       // Configure Clock  (rounded up)
#ifdef CYGNUM_DEVS_ADC_AVR32_ADCIFB_ADC0_SLEEP_ENABLE
        adc_dev->acr =  ((info->adc_shtim           << 
                        AVR32_ADCIFB_ACR_SHTIM_OFFSET)  &
                        AVR32_ADCIFB_ACR_SHTIM_MASK)    |
                        ((info->adc_startup_time    << 
                        AVR32_ADCIFB_ACR_STARTUP_OFFSET)&
                        AVR32_ADCIFB_ACR_STARTUP_MASK)  |
                        ((info->adc_prescal         << 
                        AVR32_ADCIFB_ACR_PRESCAL_OFFSET)&
                        AVR32_ADCIFB_ACR_PRESCAL_MASK)  |
                        ((info->resolution          << 
                        AVR32_ADCIFB_ACR_RES_OFFSET)    &
                        AVR32_ADCIFB_ACR_RES_MASK)      |
                        1;
#else
        adc_dev->acr =  ((info->adc_shtim           << 
                        AVR32_ADCIFB_ACR_SHTIM_OFFSET)  &
                        AVR32_ADCIFB_ACR_SHTIM_MASK)    |
                        ((info->adc_startup_time    << 
                        AVR32_ADCIFB_ACR_STARTUP_OFFSET)&
                        AVR32_ADCIFB_ACR_STARTUP_MASK)  |
                        ((info->adc_prescal         << 
                        AVR32_ADCIFB_ACR_PRESCAL_OFFSET)&
                        AVR32_ADCIFB_ACR_PRESCAL_MASK)  |
                        ((info->resolution          << 
                        AVR32_ADCIFB_ACR_RES_OFFSET)    &
                        AVR32_ADCIFB_ACR_RES_MASK)      |
                        0;
#endif
                
       //
       // setup the default sample rate
       //
       avr32_adc_set_rate(chan, chan->device->config.rate);


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
static Cyg_ErrNo avr32_adc_lookup(struct cyg_devtab_entry **tab,
                                    struct cyg_devtab_entry  *sub_tab,
                                    const char               *name)
{
    cyg_adc_channel  *chan     = (cyg_adc_channel *)(*tab)->priv;
    avr32_adc_info *info     = chan->device->dev_priv;

    info->channel[chan->channel] = chan;
    cyg_adc_channel_init(chan); // initialize generic parts of channel

    //
    // The generic ADC manual says: When a channel is first looked up or
    // opened, then it is automatically enabled and samples start to
    // accumulate - so we start the channel now
    //
    chan->enabled = true;
    avr32_adc_enable(chan);

    return ENOERR;
}


//==========================================================================
// This function is called from the generic ADC package to enable the
// channel in response to a CYG_IO_SET_CONFIG_ADC_ENABLE config operation.
// It should take any steps needed to start the channel generating samples
//==========================================================================
static void avr32_adc_enable(cyg_adc_channel *chan)
{

    avr32_adc_info *info      = chan->device->dev_priv;

    volatile avr32_adcifb_t *adc_dev = info->adc_base;

    //
    // Unmask interrupt as soon as 1 channel is enable
    //
    if (!adc_dev->chsr)
    {
       // Clar all pending statuses
       adc_dev->icr = ~0;

    }

    adc_dev->cher = AVR32_ADC_CHER_CHx(chan->channel);

    adc_dev->ier = AVR32_ADCIFB_IER_DRDY_MASK;
    /* Enable chanell and sample */
}


//==========================================================================
// This function is called from the generic ADC package to enable the
// channel in response to a CYG_IO_SET_CONFIG_ADC_DISABLE config operation.
// It should take any steps needed to stop the channel generating samples.
//==========================================================================
static void avr32_adc_disable(cyg_adc_channel *chan)
{
    avr32_adc_info *info  = chan->device->dev_priv;

    volatile avr32_adcifb_t *adc_dev = info->adc_base;

    adc_dev->chdr = AVR32_ADC_CHER_CHx(chan->channel);
    
    //
    // If no channel is enabled the we disable interrupts now
    //
    if (!adc_dev->chsr)
    {
       // Clar all pending statuses
       adc_dev->icr = ~0;
       // Disable startup interrupt
       adc_dev->idr = ~0;

    }
}


//==========================================================================
// This function is called from the generic ADC package to enable the
// channel in response to a CYG_IO_SET_CONFIG_ADC_RATE config operation.
// It should take any steps needed to change the sample rate of the channel,
// or of the entire device.
// We use a timer channel to generate the interrupts for sampling the
// analog channels. Sampling rate is number of samples per second.
//==========================================================================
static void avr32_adc_set_rate( cyg_adc_channel *chan, cyg_uint32 rate)
{
    cyg_adc_device   *device = chan->device;
    avr32_adc_info *info   = (avr32_adc_info *)device->dev_priv;
    volatile avr32_adcifb_t *adc_dev = info->adc_base;

    if(rate > 0)
    {
        cyg_uint32 adc_clkf = CYGNUM_ADC_CLK_FREQ/
                             (2*( CYGNUM_DEVS_ADC_AVR32_ADCIFB_ADC0_PRESCAL + 1));

        cyg_uint32 timer_value   = adc_clkf/rate;

        if(rate > adc_clkf)
        {
            timer_value = 0;
            avr32_adc_printf("ADCIFB timer, rate too high!");
        }

        if( timer_value > 0xffff )
        {
           timer_value = 0xffff;
           avr32_adc_printf("ADCIFB timer, rate too low!");
        }

        device->config.rate = rate;
        info->timer_cnt = timer_value;

        // Set timer values and enable timer trigger
        adc_dev->trgr = timer_value << 16 | 0x05;


        avr32_adc_printf("ADCIFB Timer settings %d, %d",
                info->timer_clk, info->timer_cnt);

        return;
    }
    else
    {
        // sofware start of conversion
        adc_dev->trgr = 0;
    }
}


//==========================================================================
// This function is the ISR attached to the ADC device's interrupt vector.
// It is responsible for reading samples from the channels and passing them
// on to the generic layer. It needs to check each channel for data, and call
// cyg_adc_receive_sample() for each new sample available, and then ready the
// device for the next interrupt.
//==========================================================================
static cyg_uint32 avr32_adc_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_adc_device   *device = (cyg_adc_device *) data;
    avr32_adc_info *info   = (avr32_adc_info *)device->dev_priv;
    volatile avr32_adcifb_t *adc_dev = info->adc_base;
    cyg_uint32        res = 0;

    cyg_uint32    result = adc_dev->lcdr;
    cyg_uint8 channel_no = (result >> 16)&0x0f;
    
    res |= CYG_ISR_HANDLED
        |  cyg_adc_receive_sample(info->channel[channel_no], result&0xff);


    adc_dev->icr = AVR32_ADCIFB_ICR_DRDY_MASK;

    return 0;
}


//==========================================================================
// This function is the DSR attached to the ADC device's interrupt vector.
// It is called by the kernel if the ISR return value contains the
// CYG_ISR_HANDLED bit. It needs to call cyg_adc_wakeup() for each channel
// that has its wakeup field set.
//==========================================================================
static void avr32_adc_dsr(cyg_vector_t vector,
                            cyg_ucount32 count,
                            cyg_addrword_t data)
{
    cyg_adc_device *device         = (cyg_adc_device *) data;
    avr32_adc_info *info           = device->dev_priv;
    volatile avr32_adcifb_t *adc_dev = info->adc_base;
    cyg_uint8      active_channels = adc_dev->chsr;
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
