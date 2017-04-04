//==========================================================================
//
//      adc_uc3c.c
//
//      ADC driver for AVR32UC3C on chip ADC
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
// Date:         2012-11-15
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
#include <pkgconf/devs_adc_avr32_uc3c.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/adc.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>



#if CYGPKG_DEVS_ADC_AVR32_UC3C_DEBUG_LEVEL > 0
   #define avr32_adc_printf(args...) diag_printf(args)
#else
   #define avr32_adc_printf(args...)
#endif

#define AVR32_MAX_ADC_CHAN 8


#define AVR32_FLASHC_FACTORY_PAGE_ADDRESS    0x80800200
#define AVR32_FLASHC_FROW_OCAL_WORD          4
#define AVR32_FLASHC_FROW_OCAL_MASK          0x3F000000
#define AVR32_FLASHC_FROW_OCAL_OFFSET        24
#define AVR32_FLASHC_FROW_GCAL_WORD          4
#define AVR32_FLASHC_FROW_GCAL_MASK          0x00007FFF
#define AVR32_FLASHC_FROW_GCAL_OFFSET        0
#define AVR32_FLASHC_FROW_GAIN0_WORD         8
#define AVR32_FLASHC_FROW_GAIN0_MASK         0x000003FF
#define AVR32_FLASHC_FROW_GAIN0_OFFSET       0
#define AVR32_FLASHC_FROW_GAIN1_WORD         8
#define AVR32_FLASHC_FROW_GAIN1_MASK         0x03FF0000
#define AVR32_FLASHC_FROW_GAIN1_OFFSET       16

#ifndef CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_MUX_SETTLE_TIME
#define CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_MUX_SETTLE_TIME     0
#endif

#ifndef CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_EXTERNAL_REFERENCE
#define CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_EXTERNAL_REFERENCE  0
#endif

#ifndef CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_SAH_DISABLE
#define CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_SAH_DISABLE         0
#endif

#ifndef CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_SLEEP_DISABLE
#define CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_SLEEP_DISABLE       0
#endif


#define INT1V          0
#define INT0_6VDDANA   1
#define EXTADCERF0     2
#define EXTADCREF1     3

/** \name Positive Inputs used by the ADC
 * @{
 */
#define AVR32_ADCIFA_INP_ADCIN0              0
#define AVR32_ADCIFA_INP_ADCIN1              1
#define AVR32_ADCIFA_INP_ADCIN2              2
#define AVR32_ADCIFA_INP_ADCIN3              3
#define AVR32_ADCIFA_INP_ADCIN4              4
#define AVR32_ADCIFA_INP_ADCIN5              5
#define AVR32_ADCIFA_INP_ADCIN6              6
#define AVR32_ADCIFA_INP_ADCIN7              7
#define AVR32_ADCIFA_INP_DAC0_INT            8
#define AVR32_ADCIFA_INP_TSENSE              9
#define AVR32_ADCIFA_INP_GNDANA              10
/** @} */

/** \name Negative Inputs used by the ADC
 * @{
 */
#define AVR32_ADCIFA_INN_ADCIN8              0
#define AVR32_ADCIFA_INN_ADCIN9              1
#define AVR32_ADCIFA_INN_ADCIN10             2
#define AVR32_ADCIFA_INN_ADCIN11             3
#define AVR32_ADCIFA_INN_ADCIN12             4
#define AVR32_ADCIFA_INN_ADCIN13             5
#define AVR32_ADCIFA_INN_ADCIN14             6
#define AVR32_ADCIFA_INN_ADCIN15             7
#define AVR32_ADCIFA_INN_DAC1_INT            8
#define AVR32_ADCIFA_INN_GNDANA              9
/** @} */

#define CYGNUM_ADC_CLK_FREQ    1000000*CYGHWR_HAL_AVR32_CPU_FREQ
#define AVR32_ADC_CHER_CHx(_ch_)  (0x1 << _ch_)
#define AVR32_ADC_CHER_CDRx(_ch_) (_ch_ << 2)



//==========================================================================
//                                  DATA TYPES
//==========================================================================
typedef struct avr32_adc_info
{
    volatile avr32_adcifa_t *adc_base;         // base address of ADC peripheral
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

static void avr32_adc_configure_sequencer0(cyg_adc_channel *chan);

// -------------------------------------------------------------------------
// Driver functions:
CYG_ADC_FUNCTIONS( avr32_adc_funs,
                   avr32_adc_enable,
                   avr32_adc_disable,
                   avr32_adc_set_rate );


#include CYGDAT_DEVS_ADC_AVR32_UC3C_INL // Instantiate ADCs

// -------------------------------------------------------------------------
// Configure conversion sequencer.
static void avr32_adc_configure_sequencer0(cyg_adc_channel *chan)
{
    int i,j;
    cyg_uint8 g[8]  = {0};
    cyg_uint8 mp[8] = {0};
    cyg_uint8 mn[8] = {0};
    avr32_adc_info *info      = chan->device->dev_priv;

    volatile avr32_adcifa_t *adc_dev = info->adc_base;

    cyg_uint8 active_channels = info->chan_mask;

    info->num_active_ch = 0;

    //Set the imputs for coresponding enabled chanels and
    //desired gain
    //The conversion sequncer goes throw n sequnces
    //and convert imput chanel based on imput multiplexers

    for(i = 0, j = 0; i < AVR32_MAX_ADC_CHAN; i++)
    {
        if( active_channels  & 0x01)
        {
            mp[j] = info->channel[i]->channel;
            mn[j] = AVR32_ADCIFA_INN_GNDANA;
            g[info->channel[i]->channel] = info->channel[i]->gain;
            info->num_active_ch++;
            j++;
        }

        active_channels  >>= 1;
    }

     //Configure sequencer
    if(info->timer_cnt == 0)
      adc_dev->seqcfg0 = ((info->num_active_ch - 1) << AVR32_ADCIFA_CNVNB_OFFSET) |
			  (info->resolution << AVR32_ADCIFA_SRES_OFFSET)           |
			  (0 << AVR32_ADCIFA_SEQCFG0_TRGSEL_OFFSET);
    else
      adc_dev->seqcfg0 = ((info->num_active_ch - 1) << AVR32_ADCIFA_CNVNB_OFFSET) |
		    (info->resolution << AVR32_ADCIFA_SRES_OFFSET)           |
		    (1 << AVR32_ADCIFA_SEQCFG0_TRGSEL_OFFSET);


    //Set appropriate imputs for sequenc
    adc_dev->inpsel00 = mp[0]        |
                       (mp[1] << 8)  |
                       (mp[2] << 16) |
                       (mp[3] << 24);

    adc_dev->inpsel10 = mp[4]        |
                       (mp[5] << 8)  |
                       (mp[6] << 16) |
                       (mp[7] << 24);

     adc_dev->innsel00 = mn[0]        |
                        (mn[1] << 8)  |
                        (mn[2] << 16) |
                        (mn[3] << 24);

    adc_dev->innsel10 = mn[4]        |
                       (mn[5] << 8)  |
                       (mn[6] << 16) |
                       (mn[7] << 24);

      //Set gains
     adc_dev->shg0 = g[0]  | (g[1] << 4)  |
              (g[2] << 8)  | (g[3] << 12) |
              (g[4] << 16) | (g[5] << 20) |
              (g[6] << 24) | (g[7] << 28);

    // Enable timer
    if(info->timer_cnt > 0)
    {
        adc_dev->cr = AVR32_ADCIFA_TSTART_MASK;
        adc_dev->itimer = info->timer_cnt;
    }
    else
    {
        adc_dev->cr = AVR32_ADCIFA_SOC0_MASK;
    }    
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
static bool avr32_adc_init(struct cyg_devtab_entry *tab)
{
    cyg_adc_channel *chan   = (cyg_adc_channel *)tab->priv;
    cyg_adc_device *device  = chan->device;
    avr32_adc_info *info    = device->dev_priv;
    volatile avr32_adcifa_t *adc_dev = info->adc_base;
    

    if (!info->int_handle)
    {
        //Init ADC IO map
        //Init reference inputs those can not be multiplexed on other
        //IO pins
        //gpio_enable_module_pin(AVR32_ADCREF0_PIN, AVR32_ADCREF0_FUNCTION);
        gpio_enable_module_pin(AVR32_ADCREFP_PIN, AVR32_ADCREFP_FUNCTION);
        gpio_enable_module_pin(AVR32_ADCREFN_PIN, AVR32_ADCREFN_FUNCTION);
        //Configuration of io pins for slected channels
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL0
        gpio_enable_module_pin(AVR32_ADCIN0_PIN, AVR32_ADCIN0_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL1
        gpio_enable_module_pin(AVR32_ADCIN1_PIN, AVR32_ADCIN1_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL2
        gpio_enable_module_pin(AVR32_ADCIN2_PIN, AVR32_ADCIN2_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL3
        gpio_enable_module_pin(AVR32_ADCIN3_PIN, AVR32_ADCIN3_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL4
        gpio_enable_module_pin(AVR32_ADCIN4_PIN, AVR32_ADCIN4_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL5
        gpio_enable_module_pin(AVR32_ADCIN5_PIN, AVR32_ADCIN5_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL6
        gpio_enable_module_pin(AVR32_ADCIN6_PIN, AVR32_ADCIN6_FUNCTION);
        #endif
        #ifdef CYGPKG_DEVS_ADC_AVR32_UC3C_ADC0_CHANNEL7
        gpio_enable_module_pin(AVR32_ADCIN7_PIN, AVR32_ADCIN7_FUNCTION);
        #endif
        // Get Offset Calibration
        signed int adc_ocal =
          ((*(volatile signed int*)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS +
          AVR32_FLASHC_FROW_OCAL_WORD)) &  AVR32_FLASHC_FROW_OCAL_MASK) >>
          AVR32_FLASHC_FROW_OCAL_OFFSET;
        // Get Gain Calibration
        signed int adc_gcal =
          ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS +
          AVR32_FLASHC_FROW_GCAL_WORD)) &  AVR32_FLASHC_FROW_GCAL_MASK) >>
          AVR32_FLASHC_FROW_GCAL_OFFSET;
        //  Get S/H Calibration
        signed int adc_gain0 =
         ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS +
         AVR32_FLASHC_FROW_GAIN0_WORD)) &  AVR32_FLASHC_FROW_GAIN0_MASK) >>
         AVR32_FLASHC_FROW_GAIN0_OFFSET;

        signed int adc_gain1 =
         ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS +
         AVR32_FLASHC_FROW_GAIN1_WORD)) &  AVR32_FLASHC_FROW_GAIN1_MASK) >>
         AVR32_FLASHC_FROW_GAIN1_OFFSET;

        cyg_drv_interrupt_create(info->adc_vector,
                                 info->adc_intprio,
                                (cyg_addrword_t)device,
                                &avr32_adc_isr,
                                &avr32_adc_dsr,
                                &(info->int_handle),
                                &(info->int_data));
        cyg_drv_interrupt_attach(info->int_handle);

       // Configure Clock  (rounded up)
	adc_dev->ckdiv = (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_PRESCAL <<
                          AVR32_ADCIFA_CKDIV_CNT_OFFSET)          &
                          AVR32_ADCIFA_CKDIV_CNT_MASK;
                         
                       
        adc_dev->cfg = (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_STARTUP_TIME      <<
                       AVR32_ADCIFA_CFG_SUT_OFFSET)                       |
                      (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_MUX_SETTLE_TIME    <<
                       AVR32_ADCIFA_CFG_MUXSET_OFFSET)                    |
                      (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_EXTERNAL_REFERENCE <<
                       AVR32_ADCIFA_CFG_EXREF_OFFSET)                     |
                      (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_REFERENCE          <<
                       AVR32_ADCIFA_CFG_RS_OFFSET)                        |
                      (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_SAH_DISABLE        <<
                       AVR32_ADCIFA_CFG_SSMQ_OFFSET)                      |
                      (CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_SLEEP_DISABLE      <<
                       AVR32_ADCIFA_CFG_SLEEP_OFFSET);


	adc_dev->adccal =
              (((adc_ocal)<<AVR32_ADCIFA_ADCCAL_OCAL)&AVR32_ADCIFA_ADCCAL_OCAL_MASK)
            | (((adc_gcal)<<AVR32_ADCIFA_ADCCAL_GCAL)&AVR32_ADCIFA_ADCCAL_GCAL_MASK);

        adc_dev->shcal =
              (((adc_gain0)<<AVR32_ADCIFA_SHCAL_GAIN0)&AVR32_ADCIFA_SHCAL_GAIN0_MASK)
            | (((adc_gain1)<<AVR32_ADCIFA_SHCAL_GAIN1)&AVR32_ADCIFA_SHCAL_GAIN1_MASK);

        // Enable ADC
        adc_dev->cfg |= AVR32_ADCIFA_ADCEN_MASK;
        while(!(adc_dev->sr & AVR32_ADCIFA_SR_SUTD_MASK));
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

    volatile avr32_adcifa_t *adc_dev = info->adc_base;

    //
    // Unmask interrupt as soon as 1 channel is enable
    //
    if (!info->chan_mask)
    {
       // Clar all pending statuses
       adc_dev->scr = ~0;

    }

    info->chan_mask |= AVR32_ADC_CHER_CHx(chan->channel);
    avr32_adc_configure_sequencer0(chan);
    adc_dev->ier = AVR32_ADCIFA_IER_SEOS0_MASK;
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

    volatile avr32_adcifa_t *adc_dev = info->adc_base;

    //
    // If no channel is enabled the we disable interrupts now
    //
    if (!info->chan_mask)
    {
       // Clar all pending statuses
       adc_dev->scr = ~0;
       // Enable startup interrupt
       adc_dev->idr = ~0;

    }

    avr32_adc_configure_sequencer0(chan);
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
    volatile avr32_adcifa_t *adc_dev = info->adc_base;

    if(rate > 0)
    {
        cyg_uint32 adc_clkf = CYGNUM_ADC_CLK_FREQ/
                             (2*( CYGNUM_DEVS_ADC_AVR32_UC3C_ADC0_PRESCAL + 1));

        cyg_uint32 timer_value   = adc_clkf/rate;

        if(rate > adc_clkf)
        {
            timer_value = 0;
            avr32_adc_printf("UC3C ADC timer, rate too high!");
        }

        if( timer_value > 0x1ffff )
        {
           timer_value = 0x1ffff;
           avr32_adc_printf("UC3c ADC timer, rate too low!");
        }

        device->config.rate = rate;
        info->timer_cnt = timer_value;

        // Set timer values
        adc_dev->itimer = timer_value;


        avr32_adc_printf("AVR32 ADC Timer settings %d, %d",
                info->timer_clk, info->timer_cnt);

        return;
    }
    else
    {
        adc_dev->cr  = AVR32_ADCIFA_TSTOP_MASK;
        adc_dev->cr  = AVR32_ADCIFA_SOC0_MASK;
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
    int i = 0;
    cyg_adc_device   *device = (cyg_adc_device *) data;
    avr32_adc_info *info   = (avr32_adc_info *)device->dev_priv;
    volatile avr32_adcifa_t *adc_dev = info->adc_base;

    cyg_uint32        res = 0;

    cyg_uint8 active_channels = info->chan_mask;
    cyg_uint8 channel_no = 0;

    while (active_channels)
    {
        if (active_channels & 0x01)
        {

           res |= CYG_ISR_HANDLED
               |  cyg_adc_receive_sample(info->channel[channel_no],
                                     adc_dev->resx[i]);
           i++;
        } // if (active_channels & 0x01)
        active_channels >>= 1;
        channel_no++;
    } // while (active_channels)


    adc_dev->scr = AVR32_ADCIFA_IDR_SEOS0_MASK;

    return res;
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
    cyg_uint8      active_channels = info->chan_mask;
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
// eof adc_uc3c.c
