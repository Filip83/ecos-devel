//==========================================================================
//
//      spi_avr32.c
//
//      Atmel AVR32UC3C SPI driver 
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2009 Free Software Foundation, Inc.
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
// Author(s):      Filip
// Original data:  Savin Zlobec <savin@elatec.si> 
// Contributors:  
// Date:           2012-11-15
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/hal/avr32/io.h>
#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>
#include <pkgconf/devs_spi_isr_avr32_uc3c.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/io/spi.h>
#include <cyg/io/spi_avr32.h>
#include <cyg/error/codes.h>

// -------------------------------------------------------------------------
static void spi_avr32_init_bus(cyg_spi_avr32_bus_t * bus);
static void spi_avr32_init_device(cyg_spi_avr32_device_t * device);

static cyg_uint32 spi_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void spi_avr32_DSR(cyg_vector_t   vector, 
                          cyg_ucount32   count, 
                          cyg_addrword_t data);

static void spi_avr32_transaction_begin(cyg_spi_device *dev);

static void spi_avr32_transaction_transfer(cyg_spi_device  *dev,
                                           cyg_bool         polled,
                                           cyg_uint32       count,
                                           const cyg_uint8 *tx_data,
                                           cyg_uint8       *rx_data,
                                           cyg_bool         drop_cs);

static void spi_avr32_transaction_tick(cyg_spi_device *dev,
                                       cyg_bool        polled,
                                       cyg_uint32      count);

static void spi_avr32_transaction_end(cyg_spi_device* dev);

static int spi_avr32_get_config(cyg_spi_device *dev, 
                                cyg_uint32      key, 
                                void           *buf,
                                cyg_uint32     *len);

static int spi_avr32_set_config(cyg_spi_device *dev, 
                                cyg_uint32      key, 
                                const void     *buf, 
                                cyg_uint32     *len);

// -------------------------------------------------------------------------
// AVR32UC3C SPI BUS

#if defined(CYGHWR_DEVS_SPI_AVR32_UC3C_BUS0)
cyg_spi_avr32_bus_t cyg_spi_avr32_bus0 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_SPI0,
    .spi_dev                          = AVR32_SPI0_ADDRESS,
    .count                            = 0,
    .tx                               = NULL,
    .rx                               = NULL
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_spi_avr32_device_t, 0);
#endif
#if defined(CYGHWR_DEVS_SPI_AVR32_UC3C_BUS1)
cyg_spi_avr32_bus_t cyg_spi_avr32_bus1 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_SPI1,
    .spi_dev                          = AVR32_SPI1_ADDRESS,
    .count                            = 0,
    .tx                               = NULL,
    .rx                               = NULL
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_spi_avr32_device_t, 1);
#endif
// -------------------------------------------------------------------------

// If C constructor with init priority functionality is not in compiler,
// rely on spi_avr32_init.cxx to init us.
#ifndef CYGBLD_ATTRIB_C_INIT_PRI
# define CYGBLD_ATTRIB_C_INIT_PRI(x)
#endif

void /*CYGBLD_ATTRIB_C_INIT_PRI(CYG_INIT_BUS_SPI)*/
cyg_spi_avr32_bus_init(void)
{

#if defined(CYGHWR_DEVS_SPI_AVR32_UC3C_BUS0)

   // NOTE: here we let the SPI controller control 
   //       the data in, out and clock signals, but 
   //       we need to handle the chip selects manually 
   //       in order to achieve better chip select control 
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
   /*gpio_enable_module_pin(AVR32_SPI0_NPCS_0_PIN,AVR32_SPI0_NPCS_0_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI0_NPCS_1_PIN,AVR32_SPI0_NPCS_1_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI0_NPCS_2_PIN,AVR32_SPI0_NPCS_2_FUNCTION);*/
   gpio_enable_module_pin(AVR32_SPI0_NPCS_3_PIN,AVR32_SPI0_NPCS_3_FUNCTION);
   
   /*gpio_enable_pin_pull_up(AVR32_SPI0_NPCS_0_PIN);
   gpio_enable_pin_pull_up(AVR32_SPI0_NPCS_1_PIN);
   gpio_enable_pin_pull_up(AVR32_SPI0_NPCS_2_PIN);
   gpio_enable_pin_pull_up(AVR32_SPI0_NPCS_3_PIN);*/
   
   gpio_enable_module_pin(AVR32_SPI0_MISO_PIN,AVR32_SPI0_MISO_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI0_MOSI_PIN ,AVR32_SPI0_NPCS_1_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI0_SCK_PIN ,AVR32_SPI0_SCK_FUNCTION);

   spi_avr32_init_bus(&cyg_spi_avr32_bus0);
#endif
#if defined(CYGHWR_DEVS_SPI_AVR32_UC3C_BUS1)
   // NOTE: here we let the SPI controller control 
   //       the data in, out and clock signals, but 
   //       we need to handle the chip selects manually 
   //       in order to achieve better chip select control 
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
   gpio_enable_module_pin(AVR32_SPI1_NPCS_3_2_PIN,AVR32_SPI1_NPCS_3_2_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI1_NPCS_1_2_PIN,AVR32_SPI1_NPCS_1_2_FUNCTION);
   //gpio_enable_module_pin(AVR32_SPI1_NPCS_2_PIN,AVR32_SPI1_NPCS_2_FUNCTION);

   /*gpio_enable_pin_pull_up(AVR32_SPI1_NPCS_0_PIN);
   gpio_enable_pin_pull_up(AVR32_SPI1_NPCS_1_PIN);
   gpio_enable_pin_pull_up(AVR32_SPI1_NPCS_2_PIN);*/

   gpio_enable_module_pin(AVR32_SPI1_MOSI_1_PIN, AVR32_SPI1_MOSI_1_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI1_MISO_1_PIN, AVR32_SPI1_MISO_1_FUNCTION);
   gpio_enable_module_pin(AVR32_SPI1_SCK_1_PIN,  AVR32_SPI1_SCK_1_FUNCTION);
   spi_avr32_init_bus(&cyg_spi_avr32_bus1);
#endif
}

// -------------------------------------------------------------------------
static void spi_avr32_init_bus(cyg_spi_avr32_bus_t * spi_bus)
{
    cyg_uint32 ctr;
    // Create and attach SPI interrupt object
    cyg_drv_interrupt_create(spi_bus->interrupt_number,
                             0,                   
                             (cyg_addrword_t)spi_bus,   
                             &spi_avr32_ISR,
                             &spi_avr32_DSR,
                             &spi_bus->spi_interrupt_handle,
                             &spi_bus->spi_interrupt);

    cyg_drv_interrupt_attach(spi_bus->spi_interrupt_handle);

    // Init transfer mutex and condition
    cyg_drv_mutex_init(&spi_bus->transfer_mx);
    cyg_drv_cond_init(&spi_bus->transfer_cond, 
                      &spi_bus->transfer_mx);
   
    // Init flags
    spi_bus->transfer_end = true;
    spi_bus->cs_up        = false;
    
    // Soft reset the SPI controller
    spi_bus->spi_dev->cr = ( 1 << AVR32_SPI_CR_SWRST);

    spi_bus->spi_dev->mr = ( 1 << AVR32_SPI_MR_MSTR )   | 
                           ( 1 << AVR32_SPI_MR_MODFDIS) |
                           ( 1 << AVR32_SPI_MR_DLYBCS)  |
                                  AVR32_SPI_MR_PCS_MASK;

    // Call upper layer bus init
    CYG_SPI_BUS_COMMON_INIT(&spi_bus->spi_bus);
}

static cyg_uint32 
spi_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_uint32 stat;
	cyg_uint8 tmp;
    cyg_spi_avr32_bus_t * spi_bus = (cyg_spi_avr32_bus_t *)data;
    // Read the status register and disable
    // the SPI int events that have occurred
	
    stat = spi_bus->spi_dev->sr;
		
    tmp = spi_bus->spi_dev->rdr;
     
    if(spi_bus->count)
    {
        if(spi_bus->rx)
            *spi_bus->rx++ = tmp;
        if(--spi_bus->count) 
        {
            spi_bus->spi_dev->tdr = spi_bus->tx ? *spi_bus->tx++ : 0;
            return CYG_ISR_HANDLED;
        }
    }
     
     spi_bus->count = 0;
     spi_bus->tx = NULL;
     spi_bus->rx = NULL;
     
     spi_bus->spi_dev->idr = AVR32_SPI_IDR_RDRF_MASK;
     return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
	 
}

static void 
spi_avr32_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_spi_avr32_bus_t *spi_bus = (cyg_spi_avr32_bus_t *) data;
    
    // Read the status register and 
    // check for transfer completion
    
    // Transfer ended  
    spi_bus->transfer_end = true;
    cyg_drv_cond_signal(&spi_bus->transfer_cond);

}

static cyg_bool 
spi_avr32_calc_scbr(cyg_spi_avr32_device_t *dev)
{
    cyg_uint32 scbr;
    cyg_bool   res = true;
    
    // Calculate SCBR from baud rate
    
    scbr = (1000000*CYGHWR_HAL_AVR32_CPU_FREQ) / dev->cl_brate;
    if ((2*((cyg_uint32)(1000000*CYGHWR_HAL_AVR32_CPU_FREQ) % dev->cl_brate))
            >= dev->cl_brate)
        scbr++;

    if (scbr > 255)
    {
        dev->cl_scbr = 255;
        res = false;	
    }
    else
    {
        dev->cl_scbr  = (cyg_uint8)scbr;
    }
    
    return res;
}

static void spi_avr32_init_device(cyg_spi_avr32_device_t * device)
{
    cyg_spi_avr32_bus_t *spi_bus = (cyg_spi_avr32_bus_t *)device->spi_device.spi_bus;

    if(device->dev_num == 0)
    {
        spi_bus->spi_dev->csr0 = (((cyg_uint32)device->tr_bt_udly) << 24) | 
                                 (((cyg_uint32)device->cs_up_udly) << 16) |
                                 (((cyg_uint32)device->cl_scbr)    << 8)  | 
                                 (((cyg_uint32)device->bits) 
                                           << AVR32_SPI_CSR2_BITS_OFFSET) |
                                                AVR32_SPI_CSR2_CSAAT_MASK | 
                                 (device->cl_pha << 1)                    | 
                                  device->cl_pol;
    }
    else if(device->dev_num == 1)
    {
        spi_bus->spi_dev->csr1 = (((cyg_uint32)device->tr_bt_udly) << 24) | 
                                 (((cyg_uint32)device->cs_up_udly) << 16) |
                                 (((cyg_uint32)device->cl_scbr)    << 8)  | 
                                 (((cyg_uint32)device->bits) 
                                           << AVR32_SPI_CSR2_BITS_OFFSET) |
                                                AVR32_SPI_CSR2_CSAAT_MASK | 
                                 (device->cl_pha << 1)                    | 
                                  device->cl_pol;
    }
    else if(device->dev_num == 2) 
    {
        spi_bus->spi_dev->csr2 = (((cyg_uint32)device->tr_bt_udly) << 24) | 
                                 (((cyg_uint32)device->cs_up_udly) << 16) |
                                 (((cyg_uint32)device->cl_scbr)    << 8)  | 
                                 (((cyg_uint32)device->bits) 
                                           << AVR32_SPI_CSR2_BITS_OFFSET) |
                                                AVR32_SPI_CSR2_CSAAT_MASK | 
                                 (device->cl_pha << 1)                    | 
                                  device->cl_pol;
    }
    else if(device->dev_num == 3)
    {
        spi_bus->spi_dev->csr3 = (((cyg_uint32)device->tr_bt_udly) << 24) | 
                                 (((cyg_uint32)device->cs_up_udly) << 16) |
                                 (((cyg_uint32)device->cl_scbr)    << 8)  | 
                                 (((cyg_uint32)device->bits) 
                                           << AVR32_SPI_CSR2_BITS_OFFSET) |
                                                AVR32_SPI_CSR2_CSAAT_MASK | 
                                 (device->cl_pha << 1)                    | 
                                  device->cl_pol;
    }
}

static void
spi_avr32_set_npcs(cyg_spi_avr32_bus_t *spi_bus,int val)
{
#ifdef CYGHWR_DEVS_SPI_AVR32_UC3C_BUS0_PCSDEC
    spi_bus->spi_dev->mr &= ~AVR32_SPI_MR_PCS_MASK | 
                            (val << AVR32_SPI_MR_PCS_OFFSET);
#else
   spi_bus->spi_dev->mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + val));
#endif
}

static void
spi_avr32_start_transfer(cyg_spi_avr32_device_t *dev)
{
    cyg_spi_avr32_bus_t *spi_bus = (cyg_spi_avr32_bus_t *)dev->spi_device.spi_bus;
  
    if (spi_bus->cs_up)
        return;

    // Raise CS
    spi_avr32_set_npcs(spi_bus,dev->dev_num);
   
    spi_bus->cs_up = true;
}

static void
spi_avr32_drop_cs(cyg_spi_avr32_device_t *dev)
{
    cyg_spi_avr32_bus_t *spi_bus = (cyg_spi_avr32_bus_t *)dev->spi_device.spi_bus;
    
    if (!spi_bus->cs_up)
       return;
           
    // Drop CS
    spi_bus->spi_dev->mr |= AVR32_SPI_MR_PCS_MASK;
    spi_bus->spi_dev->cr  = AVR32_SPI_CR_LASTXFER_MASK;
    spi_bus->cs_up = false;
}

static void
spi_avr32_transfer(cyg_spi_avr32_device_t *dev,
                  cyg_uint32             count, 
                  const cyg_uint8       *tx_data,
                  cyg_uint8             *rx_data)
{
    cyg_spi_avr32_bus_t *spi_bus = (cyg_spi_avr32_bus_t *)dev->spi_device.spi_bus;
	
    if(!count) return;

    spi_bus->count = count;
    spi_bus->tx = tx_data;
    spi_bus->rx = rx_data;

    cyg_drv_mutex_lock(&spi_bus->transfer_mx);
    cyg_drv_dsr_lock();
    
    spi_bus->spi_dev->ier |= AVR32_SPI_IER_RDRF_MASK;
    spi_bus->spi_dev->tdr = spi_bus->tx ? *spi_bus->tx++ : 0;
    while(spi_bus->count)
          cyg_drv_cond_wait(&spi_bus->transfer_cond);
    spi_bus->spi_dev->idr = ~0;
    
    cyg_drv_dsr_unlock();
    cyg_drv_mutex_unlock(&spi_bus->transfer_mx);

    return;
}

static void
spi_avr32_transfer_polled(cyg_spi_avr32_device_t *dev, 
                          cyg_uint32             count,
                          const cyg_uint8        *tx_data,
                          cyg_uint8              *rx_data)
{
    cyg_uint8 tmp;
    cyg_spi_avr32_bus_t *spi_bus = (cyg_spi_avr32_bus_t *)dev->spi_device.spi_bus;

    if(!count) return;
	
    do 
    {
        spi_bus->spi_dev->tdr = tx_data ? *tx_data++ : 0;
        while(!(spi_bus->spi_dev->sr & AVR32_SPI_RDRF_MASK));
        tmp = spi_bus->spi_dev->rdr;
        if(rx_data)
            *rx_data++ = tmp;
        count--;
    } while(count);
}

// -------------------------------------------------------------------------

static void 
spi_avr32_transaction_begin(cyg_spi_device *dev)
{
    cyg_spi_avr32_device_t *avr32_spi_dev = (cyg_spi_avr32_device_t *) dev;    
    cyg_spi_avr32_bus_t *spi_bus = 
                (cyg_spi_avr32_bus_t *)avr32_spi_dev->spi_device.spi_bus;
    
    if (!avr32_spi_dev->init)
    {
        avr32_spi_dev->init = true;
        spi_avr32_calc_scbr(avr32_spi_dev);
	spi_avr32_init_device(avr32_spi_dev);
    }
    
    // Configure SPI channel 0 - this is the only channel we 
    // use for all devices since we drive chip selects manually
    spi_bus->spi_dev->cr = AVR32_SPI_CR_SPIEN_MASK;
    
}

static void 
spi_avr32_transaction_transfer(cyg_spi_device  *dev, 
                              cyg_bool         polled,  
                              cyg_uint32       count, 
                              const cyg_uint8 *tx_data, 
                              cyg_uint8       *rx_data, 
                              cyg_bool         drop_cs) 
{
    cyg_spi_avr32_device_t *avr32_spi_dev = (cyg_spi_avr32_device_t *) dev;

    // Select the device if not already selected
    spi_avr32_start_transfer(avr32_spi_dev);
 
    // Perform the transfer
    if (polled)
        spi_avr32_transfer_polled(avr32_spi_dev, count, tx_data, rx_data);
    else
        spi_avr32_transfer(avr32_spi_dev, count, tx_data, rx_data);

    // Deselect the device if requested
    if (drop_cs)
        spi_avr32_drop_cs(avr32_spi_dev);
}

static void 
spi_avr32_transaction_tick(cyg_spi_device *dev, 
                           cyg_bool        polled,  
                           cyg_uint32      count)
{
    const cyg_uint32 zeros[10] = { 0,0,0,0,0,0,0,0,0,0 };

    cyg_spi_avr32_device_t *avr32_spi_dev = (cyg_spi_avr32_device_t *) dev;
    
    // Transfer count zeros to the device - we don't touch the
    // chip select, the device could be selected or deselected.
    // It is up to the device driver to decide in wich state the
    // device will be ticked.
    
    while (count > 0)
    {
        int tcnt = count > 40 ? 40 : count;
        
        if (polled)
            spi_avr32_transfer_polled(avr32_spi_dev, tcnt, 
                                     (const cyg_uint8 *) zeros, NULL);
        else
            spi_avr32_transfer(avr32_spi_dev, tcnt, 
                              (const cyg_uint8 *) zeros, NULL);

        count -= tcnt;
    }
}

static void                    
spi_avr32_transaction_end(cyg_spi_device* dev)
{
    cyg_spi_avr32_device_t * avr32_spi_dev = (cyg_spi_avr32_device_t *)dev; 
    cyg_spi_avr32_bus_t *spi_bus = 
                (cyg_spi_avr32_bus_t *)avr32_spi_dev->spi_device.spi_bus;

    // Disable the SPI controller
    spi_bus->spi_dev->cr = AVR32_SPI_CR_SPIDIS_MASK;
   
    spi_avr32_drop_cs((cyg_spi_avr32_device_t *) dev);
}

static int                     
spi_avr32_get_config(cyg_spi_device *dev, 
                     cyg_uint32      key, 
                     void           *buf,
                     cyg_uint32     *len)
{
    cyg_spi_avr32_device_t *avr32_spi_dev = (cyg_spi_avr32_device_t *) dev;
    
    switch (key) 
    {
        case CYG_IO_GET_CONFIG_SPI_CLOCKRATE:
        {
            if (*len != sizeof(cyg_uint32))
                return -EINVAL;
            else
            {
                cyg_uint32 *cl_brate = (cyg_uint32 *)buf;
                *cl_brate = avr32_spi_dev->cl_brate; 
            }
        }
        break;
        default:
            return -EINVAL;
    }
    return ENOERR;
}

static int                     
spi_avr32_set_config(cyg_spi_device *dev, 
                     cyg_uint32      key, 
                     const void     *buf, 
                     cyg_uint32     *len)
{
    cyg_spi_avr32_device_t *avr32_spi_dev = (cyg_spi_avr32_device_t *) dev;
   
    switch (key) 
    {
        case CYG_IO_SET_CONFIG_SPI_CLOCKRATE:
        {
            if (*len != sizeof(cyg_uint32))
                return -EINVAL;
            else
            {
                cyg_uint32 cl_brate     = *((cyg_uint32 *)buf);
                cyg_uint32 old_cl_brate = avr32_spi_dev->cl_brate;
           
                avr32_spi_dev->cl_brate = cl_brate;
            
                if (!spi_avr32_calc_scbr(avr32_spi_dev))
                {
                    avr32_spi_dev->cl_brate = old_cl_brate;
                    spi_avr32_calc_scbr(avr32_spi_dev);
                    return -EINVAL;
                }
            }
        }
        break;
        default:
            return -EINVAL;
    }
    return ENOERR;
}

// -------------------------------------------------------------------------
// EOF spi_avr32.c
