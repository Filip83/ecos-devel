//==========================================================================
//
//      usart_spi_avr32.c
//
//      Atmel AVR32UC3C USART SPI driver
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
// Author(s):     Filip
// Date:          2012-11-15
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <cyg/hal/avr32/io.h>
#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>
#include <pkgconf/devs_usart_spi_avr32_uc3c.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/io/spi.h>
#include <cyg/io/usart_spi_avr32.h>
#include <cyg/error/codes.h>
#include <cyg/hal/gpio.h>
#include <cyg/hal/board_config.h>


// -------------------------------------------------------------------------
static void spi_avr32_init_bus(cyg_usart_spi_avr32_bus_t * bus);
static void spi_avr32_init_device(cyg_usart_spi_avr32_device_t * device);

static cyg_uint32 spi_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data);

static void spi_avr32_DSR(cyg_vector_t   vector,
                         cyg_ucount32   count,
                         cyg_addrword_t data);

static void spi_avr32_transaction_begin(cyg_usart_spi_avr32_device_t *dev);

static void spi_avr32_transaction_transfer(cyg_usart_spi_avr32_device_t  *dev,
                                          cyg_bool         polled,
                                          cyg_uint32       count,
                                          const cyg_uint8 *tx_data,
                                          cyg_uint8       *rx_data,
                                          cyg_bool         drop_cs);

static void spi_avr32_transaction_tick(cyg_usart_spi_avr32_device_t *dev,
                                      cyg_bool        polled,
                                      cyg_uint32      count);

static void spi_avr32_transaction_end(cyg_usart_spi_avr32_device_t* dev);

static int spi_avr32_get_config(cyg_usart_spi_avr32_device_t *dev,
                               cyg_uint32      key,
                               void           *buf,
                               cyg_uint32     *len);

static int spi_avr32_set_config(cyg_usart_spi_avr32_device_t *dev,
                               cyg_uint32      key,
                               const void     *buf,
                               cyg_uint32     *len);

// -------------------------------------------------------------------------
// AVR32UC3C SPI BUS

#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS0)
cyg_usart_spi_avr32_bus_t cyg_usart_spi_avr32_bus0 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_USART0,
    .spi_dev                          = AVR32_USART0_ADDRESS,
    .count                            = 0,
    .npcs_gpio_pin                    = {-1,-1,-1,-1}
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_usart_spi_avr32_device_t, 0);
#endif
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS1)
cyg_usart_spi_avr32_bus_t cyg_usart_spi_avr32_bus1 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_USART1,
    .spi_dev                          = AVR32_USART1_ADDRESS,
    .count                            = 0,
    .npcs_gpio_pin                    = {-1,-1,-1,-1}
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_usart_spi_avr32_device_t, 1);
#endif
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS2)
cyg_usart_spi_avr32_bus_t cyg_usart_spi_avr32_bus2 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_USART2,
    .spi_dev                          = AVR32_USART2_ADDRESS,
    .count                            = 0,
    .npcs_gpio_pin                    = {-1,-1,-1,-1}
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_usart_spi_avr32_device_t, 2);
#endif
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS3)
cyg_usart_spi_avr32_bus_t cyg_usart_spi_avr32_bus3 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_USART3,
    .spi_dev                          = AVR32_USART3_ADDRESS,
    .count                            = 0,
    .npcs_gpio_pin                    = {-1,-1,-1,-1}
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_usart_spi_avr32_device_t, 3);
#endif
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS4)
cyg_usart_spi_avr32_bus_t cyg_usart_spi_avr32_bus4 = {
    .spi_bus.spi_transaction_begin    = spi_avr32_transaction_begin,
    .spi_bus.spi_transaction_transfer = spi_avr32_transaction_transfer,
    .spi_bus.spi_transaction_tick     = spi_avr32_transaction_tick,
    .spi_bus.spi_transaction_end      = spi_avr32_transaction_end,
    .spi_bus.spi_get_config           = spi_avr32_get_config,
    .spi_bus.spi_set_config           = spi_avr32_set_config,
    .interrupt_number                 = CYGNUM_HAL_VECTOR_USART4,
    .spi_dev                          = AVR32_USART4_ADDRESS,
    .count                            = 0,
    .npcs_gpio_pin                    = {-1,-1,-1,-1}
};

CYG_SPI_DEFINE_BUS_TABLE(cyg_usart_spi_avr32_device_t, 4);
#endif

// -------------------------------------------------------------------------

// If C constructor with init priority functionality is not in compiler,
// rely on spi_avr32_init.cxx to init us.
#ifndef CYGBLD_ATTRIB_C_INIT_PRI
# define CYGBLD_ATTRIB_C_INIT_PRI(x)
#endif

void /*CYGBLD_ATTRIB_C_INIT_PRI(CYG_INIT_BUS_SPI)*/
cyg_usart_spi_avr32_bus_init(void)
{

#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS0)
   // NOTE: here we let the SPI controller control
   //       the data in, out and clock signals, but
   //       we need to handle the chip selects manually
   //       in order to achieve better chip select control
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
#if CYG_HAL_USART0_TXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART0_TXD_PIN, 
           CYG_HAL_USART0_TXD_FUNCTION);
#endif
#if CYG_HAL_USART0_RXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART0_RXD_PIN, 
           CYG_HAL_USART0_RXD_FUNCTION);
#endif
   gpio_enable_module_pin(CYG_HAL_USART0_CLK_PIN, CYG_HAL_USART0_CLK_FUNCTION);
   
    //GPI chip select
#if CYG_HAL_USART0_NPCS_0_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART0_NPCS_0_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus0.npcs_gpio_pin[0] = CYG_HAL_USART0_NPCS_0_GPIO;
#endif
   
#if CYG_HAL_USART0_NPCS_1_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART0_NPCS_1_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus0.npcs_gpio_pin[1] = CYG_HAL_USART0_NPCS_1_GPIO;
#endif
   
#if CYG_HAL_USART0_NPCS_2_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART0_NPCS_2_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus0.npcs_gpio_pin[2] = CYG_HAL_USART0_NPCS_2_GPIO;
#endif
   
#if CYG_HAL_USART0_NPCS_3_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART0_NPCS_3_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus0.npcs_gpio_pin[3] = CYG_HAL_USART0_NPCS_0_GPIO;
#endif

   spi_avr32_init_bus(&cyg_usart_spi_avr32_bus0);
#endif
   
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS1)
   // NOTE: here we let the SPI controller control
   //       the data in, out and clock signals, but
   //       we need to handle the chip selects manually
   //       in order to achieve better chip select control
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
#if CYG_HAL_USART1_TXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART1_TXD_PIN, 
           CYG_HAL_USART1_TXD_FUNCTION);
#endif
#if CYG_HAL_USART1_RXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART1_RXD_PIN, 
           CYG_HAL_USART1_RXD_FUNCTION);
#endif
   gpio_enable_module_pin(CYG_HAL_USART1_CLK_PIN, CYG_HAL_USART1_CLK_FUNCTION);
   
    //GPI chip select
#if CYG_HAL_USART1_NPCS_0_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART1_NPCS_0_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus1.npcs_gpio_pin[0] = CYG_HAL_USART1_NPCS_0_GPIO;
#endif
   
#if CYG_HAL_USART1_NPCS_1_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART1_NPCS_1_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus1.npcs_gpio_pin[1] = CYG_HAL_USART1_NPCS_1_GPIO;
#endif
   
#if CYG_HAL_USART1_NPCS_2_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART1_NPCS_2_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus1.npcs_gpio_pin[2] = CYG_HAL_USART1_NPCS_2_GPIO;
#endif
   
#if CYG_HAL_USART1_NPCS_3_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART1_NPCS_3_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus1.npcs_gpio_pin[3] = CYG_HAL_USART1_NPCS_3_GPIO;
#endif

   spi_avr32_init_bus(&cyg_usart_spi_avr32_bus1);
#endif
 #if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS2)
   // NOTE: here we let the SPI controller control
   //       the data in, out and clock signals, but
   //       we need to handle the chip selects manually
   //       in order to achieve better chip select control
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
#if CYG_HAL_USART2_TXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART2_TXD_PIN, 
           CYG_HAL_USART2_TXD_FUNCTION);
#endif
#if CYG_HAL_USART2_RXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART2_RXD_PIN, 
           CYG_HAL_USART2_RXD_FUNCTION);
#endif
   gpio_enable_module_pin(CYG_HAL_USART2_CLK_PIN, CYG_HAL_USART2_CLK_FUNCTION);
   
    //GPI chip select
#if CYG_HAL_USART2_NPCS_0_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART2_NPCS_0_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus2.npcs_gpio_pin[0] = CYG_HAL_USART2_NPCS_0_GPIO;
#endif
   
#if CYG_HAL_USART2_NPCS_1_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART2_NPCS_1_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus2.npcs_gpio_pin[1] = CYG_HAL_USART2_NPCS_1_GPIO;
#endif
   
#if CYG_HAL_USART2_NPCS_2_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART2_NPCS_2_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus2.npcs_gpio_pin[2] = CYG_HAL_USART2_NPCS_2_GPIO;
#endif
   
#if CYG_HAL_USART2_NPCS_3_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART2_NPCS_3_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus2.npcs_gpio_pin[3] = CYG_HAL_USART2_NPCS_3_GPIO;
#endif

   spi_avr32_init_bus(&cyg_usart_spi_avr32_bus2);
#endif
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS3)
   // NOTE: here we let the SPI controller control
   //       the data in, out and clock signals, but
   //       we need to handle the chip selects manually
   //       in order to achieve better chip select control
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
#if CYG_HAL_USART3_TXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART3_TXD_PIN, 
           CYG_HAL_USART3_TXD_FUNCTION);
#endif
#if CYG_HAL_USART3_RXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART3_RXD_PIN, 
           CYG_HAL_USART3_RXD_FUNCTION);
#endif
   gpio_enable_module_pin(CYG_HAL_USART3_CLK_PIN, CYG_HAL_USART3_CLK_FUNCTION);
   
    //GPI chip select
#if CYG_HAL_USART3_NPCS_0_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART3_NPCS_0_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus3.npcs_gpio_pin[0] = CYG_HAL_USART3_NPCS_0_GPIO;
#endif
   
#if CYG_HAL_USART3_NPCS_1_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART3_NPCS_1_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus3.npcs_gpio_pin[1] = CYG_HAL_USART3_NPCS_1_GPIO;
#endif
   
#if CYG_HAL_USART3_NPCS_2_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART3_NPCS_2_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus3.npcs_gpio_pin[2] = CYG_HAL_USART3_NPCS_2_GPIO;
#endif
   
#if CYG_HAL_USART3_NPCS_3_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART3_NPCS_3_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus3.npcs_gpio_pin[3] = CYG_HAL_USART3_NPCS_3_GPIO;
#endif

   spi_avr32_init_bus(&cyg_usart_spi_avr32_bus3);
#endif
#if defined(CYGHWR_DEVS_USART_SPI_AVR32_UC3C_BUS4)
   // NOTE: here we let the SPI controller control
   //       the data in, out and clock signals, but
   //       we need to handle the chip selects manually
   //       in order to achieve better chip select control
   //       in between transactions.

   // Put SPI MISO, MOSI and SPCK pins into peripheral mode
#if CYG_HAL_USART4_TXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART4_TXD_PIN, 
           CYG_HAL_USART4_TXD_FUNCTION);
#endif
#if CYG_HAL_USART4_RXD_ENABLED == PIN_ENABLE
   gpio_enable_module_pin(CYG_HAL_USART4_RXD_PIN, 
           CYG_HAL_USART4_RXD_FUNCTION);
#endif
   gpio_enable_module_pin(CYG_HAL_USART4_CLK_PIN, CYG_HAL_USART4_CLK_FUNCTION);
   
    //GPI chip select
#if CYG_HAL_USART4_NPCS_0_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART4_NPCS_0_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus4.npcs_gpio_pin[0] = CYG_HAL_USART4_NPCS_0_GPIO;
#endif
   
#if CYG_HAL_USART4_NPCS_1_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART4_NPCS_1_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus4.npcs_gpio_pin[1] = CYG_HAL_USART4_NPCS_1_GPIO;
#endif
   
#if CYG_HAL_USART4_NPCS_2_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART4_NPCS_2_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus4.npcs_gpio_pin[2] = CYG_HAL_USART4_NPCS_2_GPIO;
#endif
   
#if CYG_HAL_USART4_NPCS_3_GPIO_EABLED == PIN_ENABLE
   gpio_configure_pin(CYG_HAL_USART4_NPCS_3_GPIO, GPIO_DIR_OUTPUT | 
                                                  GPIO_PULL_UP    | 
                                                  GPIO_DRIVE_HIGH);
   cyg_usart_spi_avr32_bus4.npcs_gpio_pin[3] = CYG_HAL_USART4_NPCS_3_GPIO;
#endif

   spi_avr32_init_bus(&cyg_usart_spi_avr32_bus4);
#endif
}


// -------------------------------------------------------------------------

static void spi_avr32_init_bus(cyg_usart_spi_avr32_bus_t * spi_bus)
{
    
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

    // Call upper layer bus init
    CYG_SPI_BUS_COMMON_INIT(&spi_bus->spi_bus);
}

static cyg_uint32
spi_avr32_ISR(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_uint32 stat;
    cyg_uint8 tmp;
    cyg_usart_spi_avr32_bus_t * spi_bus =
            (cyg_usart_spi_avr32_bus_t *)data;
    // Read the status register and data register
	
    stat = spi_bus->spi_dev->csr;
		
    tmp = spi_bus->spi_dev->rhr;
     
    if(spi_bus->count && (spi_bus->spi_dev->csr & AVR32_USART_CSR_TXRDY_MASK))
    {
        if(spi_bus->rx)
           *spi_bus->rx++ = tmp;
        
        if(--spi_bus->count) 
        {
            spi_bus->spi_dev->thr = spi_bus->tx ? *spi_bus->tx++ : 0;
            return CYG_ISR_HANDLED;
        }
    }
     
    spi_bus->count = 0;
    spi_bus->tx = NULL;
    spi_bus->rx = NULL;
     
    spi_bus->spi_dev->idr = AVR32_USART_IDR_RXRDY_MASK;
    return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
}

static void
spi_avr32_DSR(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_usart_spi_avr32_bus_t *spi_bus = (cyg_usart_spi_avr32_bus_t *) data;

    // Read the status register and
    // check for transfer completion

    // Transfer ended
    spi_bus->transfer_end = true;
    cyg_drv_cond_signal(&spi_bus->transfer_cond);

}

static cyg_bool
spi_avr32_calc_scbr(cyg_usart_spi_avr32_device_t *dev)
{

    cyg_uint32 cd;
    cyg_bool   res = true;
    
    cd = (CYGHWR_HAL_AVR32_CPU_FREQ + dev->cl_brate / 2) / dev->cl_brate;

    if (cd < 4)
    {
        res = false;
        cd  = 4;
    }
    else if(cd > (1 << AVR32_USART_BRGR_CD_SIZE) - 1)
    {
        res = false;
        cd  = (1 << AVR32_USART_BRGR_CD_SIZE) - 1;
    }
        

    /*dev->spi_dev->mr = (usart->mr & ~AVR32_USART_MR_USCLKS_MASK) |
        AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET;

    dev->spi_dev->brgr = cd << AVR32_USART_BRGR_CD_OFFSET;*/
    dev->cl_scbr       = cd;

    return res;
}

static void spi_avr32_init_device(cyg_usart_spi_avr32_device_t * device)
{
    cyg_usart_spi_avr32_bus_t *spi_bus = 
            (cyg_usart_spi_avr32_bus_t *)device->spi_device.spi_bus;
	
	// Reset device
    spi_bus->spi_dev->cr = AVR32_USART_CR_RSTRX_MASK | 
                           AVR32_USART_CR_RSTTX_MASK | 
                           AVR32_USART_CR_RSTSTA_MASK ;
    
    spi_bus->spi_dev->brgr = device->cl_scbr << AVR32_USART_BRGR_CD_OFFSET;
    
    if(device->bits == 9)
    {
       // use USART undivided clock and enable clock to CLK pin
        spi_bus->spi_dev->mr = 
                (AVR32_USART_MR_USCLKS_MCK  << AVR32_USART_MR_USCLKS_OFFSET)  |
                (AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET)   |
                (AVR32_USART_MR_MODE_SPI_MASTER << AVR32_USART_MR_MODE_OFFSET)|
                ((device->cl_pha & 0x1) << AVR32_USART_MR_SYNC_OFFSET)        |
                ((device->cl_pol & 0x1) << AVR32_USART_MR_MSBF_OFFSET)        |
                AVR32_USART_MR_CLKO_MASK | AVR32_USART_MR_MODE9_MASK;
    }
    else
    {
        spi_bus->spi_dev->mr = 
                (AVR32_USART_MR_USCLKS_MCK  << AVR32_USART_MR_USCLKS_OFFSET)  |
                (AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET)   |
                (AVR32_USART_MR_MODE_SPI_MASTER << AVR32_USART_MR_MODE_OFFSET)|
                ((device->cl_pha & 0x1) << AVR32_USART_MR_SYNC_OFFSET)        |
                ((device->cl_pol & 0x1) << AVR32_USART_MR_MSBF_OFFSET)        |
                ((device->bits - 5) << AVR32_USART_MR_CHRL_OFFSET)            |
                AVR32_USART_MR_CLKO_MASK;
    }						  
}

static void
spi_avr32_set_npcs(cyg_usart_spi_avr32_bus_t *spi_bus,int val)
{
    CYG_ASSERT(val < 4, "AVR32 USART SPI chip slect number fail.");
    CYG_ASSERT(spi_bus->npcs_gpio_pin[val] != -1,
            "AVR32 USART SPI NCSP pin number not defined.");
    
    gpio_set_pin_low(spi_bus->npcs_gpio_pin[val]);
    
    spi_bus->cs_up = true;
}

static void
spi_avr32_start_transfer(cyg_usart_spi_avr32_device_t *dev)
{
    cyg_usart_spi_avr32_bus_t *spi_bus = (cyg_usart_spi_avr32_bus_t *)dev->spi_device.spi_bus;

    if (spi_bus->cs_up)
        return;

    // Force minimal delay between two transfers - in case two transfers
    // follow each other w/o delay, then we have to wait here in order for
    // the peripheral device to detect cs transition from inactive to active.
    //CYGACC_CALL_IF_DELAY_US(dev->tr_bt_udly);
    
    if(dev->bits == 9)
    {
       // use USART undivided clock and enable clock to CLK pin
        spi_bus->spi_dev->mr = 
                (AVR32_USART_MR_USCLKS_MCK  << AVR32_USART_MR_USCLKS_OFFSET)  |
                (AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET)   |
                (AVR32_USART_MR_MODE_SPI_MASTER << AVR32_USART_MR_MODE_OFFSET)|
                ((dev->cl_pha & 0x1) << AVR32_USART_MR_SYNC_OFFSET)        |
                ((dev->cl_pol & 0x1) << AVR32_USART_MR_MSBF_OFFSET)        |
                AVR32_USART_MR_CLKO_MASK | AVR32_USART_MR_MODE9_MASK;
    }
    else
    {
        spi_bus->spi_dev->mr = 
                (AVR32_USART_MR_USCLKS_MCK  << AVR32_USART_MR_USCLKS_OFFSET)  |
                (AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET)   |
                (AVR32_USART_MR_MODE_SPI_MASTER << AVR32_USART_MR_MODE_OFFSET)|
                ((dev->cl_pha & 0x1) << AVR32_USART_MR_SYNC_OFFSET)        |
                ((dev->cl_pol & 0x1) << AVR32_USART_MR_MSBF_OFFSET)        |
                ((dev->bits - 5) << AVR32_USART_MR_CHRL_OFFSET)            |
                AVR32_USART_MR_CLKO_MASK;
    }	

    // Raise CS
    spi_avr32_set_npcs(spi_bus,dev->dev_num);
}

static void
spi_avr32_drop_cs(cyg_usart_spi_avr32_device_t *dev)
{
    cyg_usart_spi_avr32_bus_t *spi_bus = (cyg_usart_spi_avr32_bus_t *)dev->spi_device.spi_bus;

    if (!spi_bus->cs_up)
       return;

    // Drop CS
    if(spi_bus->npcs_gpio_pin[0] != -1)
        gpio_set_pin_high(spi_bus->npcs_gpio_pin[0]);
    
    if(spi_bus->npcs_gpio_pin[1] != -1)
        gpio_set_pin_high(spi_bus->npcs_gpio_pin[1]);
    
    if(spi_bus->npcs_gpio_pin[2] != -1)
        gpio_set_pin_high(spi_bus->npcs_gpio_pin[2]);
    
    if(spi_bus->npcs_gpio_pin[3] != -1)
        gpio_set_pin_high(spi_bus->npcs_gpio_pin[3]);
    
    spi_bus->cs_up = false;
}

static void
spi_avr32_transfer(cyg_usart_spi_avr32_device_t *dev,
                  cyg_uint32             count,
                  const cyg_uint8       *tx_data,
                  cyg_uint8             *rx_data)
{
    cyg_usart_spi_avr32_bus_t *spi_bus = 
            (cyg_usart_spi_avr32_bus_t *)dev->spi_device.spi_bus;

    if(!count) return;

    spi_bus->count = count;
    spi_bus->tx    = tx_data;
    spi_bus->rx    = rx_data;

    cyg_drv_mutex_lock(&spi_bus->transfer_mx);
    cyg_drv_dsr_lock();

    spi_bus->spi_dev->ier = AVR32_USART_IER_RXRDY_MASK; 
    spi_bus->spi_dev->thr = spi_bus->tx ? *spi_bus->tx++ : 0;

    while(spi_bus->count)
            cyg_drv_cond_wait(&spi_bus->transfer_cond);
    spi_bus->spi_dev->idr = ~0;

    cyg_drv_dsr_unlock();
    cyg_drv_mutex_unlock(&spi_bus->transfer_mx);

    return;
}

static void
spi_avr32_transfer_polled(cyg_usart_spi_avr32_device_t *dev,
                         cyg_uint32             count,
                         const cyg_uint8       *tx_data,
                         cyg_uint8             *rx_data)
{
    cyg_uint8 tmp;
    cyg_usart_spi_avr32_bus_t *spi_bus = 
            (cyg_usart_spi_avr32_bus_t *)dev->spi_device.spi_bus;

    if(!count) return;
	 
    do {
        spi_bus->spi_dev->thr = tx_data ? *tx_data++ : 0;
        while(!(spi_bus->spi_dev->csr & AVR32_USART_CSR_RXRDY_MASK));
        tmp = spi_bus->spi_dev->rhr;
        if(rx_data)
            *rx_data++ = tmp;
        count--;
    } while(count);
}

// -------------------------------------------------------------------------

static void
spi_avr32_transaction_begin(cyg_usart_spi_avr32_device_t *dev)
{
    cyg_usart_spi_avr32_device_t *avr32_spi_dev = (cyg_usart_spi_avr32_device_t *) dev;
    cyg_usart_spi_avr32_bus_t *spi_bus =
      (cyg_usart_spi_avr32_bus_t *)avr32_spi_dev->spi_device.spi_bus;

    if (!avr32_spi_dev->init)
    {
        avr32_spi_dev->init = true;
        spi_avr32_calc_scbr(avr32_spi_dev);
	spi_avr32_init_device(avr32_spi_dev);
    }

    // Configure SPI channel 0 - this is the only channel we
    // use for all devices since we drive chip selects manually
    spi_bus->spi_dev->cr = AVR32_USART_CR_RXEN_MASK |
                           AVR32_USART_CR_TXEN_MASK;
}

static void
spi_avr32_transaction_transfer(cyg_usart_spi_avr32_device_t  *dev,
                              cyg_bool         polled,
                              cyg_uint32       count,
                              const cyg_uint8 *tx_data,
                              cyg_uint8       *rx_data,
                              cyg_bool         drop_cs)
{
    cyg_usart_spi_avr32_device_t *avr32_spi_dev = (cyg_usart_spi_avr32_device_t *) dev;

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
spi_avr32_transaction_tick(cyg_usart_spi_avr32_device_t *dev,
                          cyg_bool        polled,
                          cyg_uint32      count)
{
    const cyg_uint32 zeros[10] = { 0,0,0,0,0,0,0,0,0,0 };

    cyg_usart_spi_avr32_device_t *avr32_spi_dev = (cyg_usart_spi_avr32_device_t *) dev;

    // Transfer count zeros to the device - we don't touch the
    // chip select, the device could be selected or deselected.
    // It is up to the device driver to decide in which state the
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
spi_avr32_transaction_end(cyg_usart_spi_avr32_device_t* dev)
{
    cyg_usart_spi_avr32_device_t *avr32_spi_dev = 
            (cyg_usart_spi_avr32_device_t *) dev;
    
    cyg_usart_spi_avr32_bus_t *spi_bus =
      (cyg_usart_spi_avr32_bus_t *)avr32_spi_dev->spi_device.spi_bus;
    
    spi_bus->spi_dev->cr = AVR32_USART_CR_RXDIS_MASK |
                           AVR32_USART_CR_TXDIS_MASK;
    
    spi_avr32_drop_cs((cyg_usart_spi_avr32_device_t *) dev);
}

static int
spi_avr32_get_config(cyg_usart_spi_avr32_device_t *dev,
                    cyg_uint32      key,
                    void           *buf,
                    cyg_uint32     *len)
{
    cyg_usart_spi_avr32_device_t *avr32_spi_dev = (cyg_usart_spi_avr32_device_t *) dev;

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
spi_avr32_set_config(cyg_usart_spi_avr32_device_t *dev,
                    cyg_uint32      key,
                    const void     *buf,
                    cyg_uint32     *len)
{
    return -EINVAL;
}

// -------------------------------------------------------------------------
// EOF usart_spi_avr32.c
