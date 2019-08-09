/*
 * MGS1_board_startup.c
 *
 * Created: 15.5.2013 10:27:12
 *  Author: Filip
 */ 


#include <pkgconf/system.h>
#include <pkgconf/hal_avr32_uc3c_minisp.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/testcase.h>           // Test macros
#include <cyg/infra/cyg_ass.h>            // Assertion macros
#include <cyg/infra/diag.h>               // Diagnostic output

#include <cyg/hal/hal_arch.h>             // CYGNUM_HAL_STACK_SIZE_TYPICAL
#include <cyg/hal/hal_if.h>
//#include <cyg/kernel/kapi.h>



#include <cyg/io/spi.h>
#include <cyg/io/spi_avr32.h>
#include <cyg/io/flash.h>



#include <cyg/hal/avr32/io.h>
#include <cyg/hal/gpio.h>
#include <cyg/hal/MGS1_board_config.h>

//! Unlock PM register macro
#define PM_UNLOCK(reg)  (AVR32_PM.unlock = (AVR32_PM_UNLOCK_KEY_VALUE << AVR32_PM_UNLOCK_KEY_OFFSET) | (reg))

cyg_uint64	_hw_error = 0;

#ifndef BOOT_LOADER

#include <cyg/io/i2c.h>
#include <cyg/io/i2c_uc3c.h>

#include <cyg/io/fm25wxx.h>
#include <cyg/io/n25qxx.h>
/**
/* Definition of SPI device on chip select 0
/* for NX25QXX FLASH chip.
*/
cyg_spi_avr32_device_t flash_spi_device =
{
	.spi_device     = &cyg_spi_avr32_bus1,
	.dev_num        = FLASH_N25QXX_SPI_DEV_CS_NUM,
	.bits           = FLASH_N25QXX_SPI_DEV_BITS_NUM,
	.cl_pol         = FLASH_N25QXX_SPI_DEV_CPOL,
	.cl_pha         = FLASH_N25QXX_SPI_DEV_NCPHA,
	.cl_brate       = FLASH_N25QXX_SPI_DEV_BUD_RATE,
	.cs_up_udly     = FLASH_N25QXX_SPI_DEV_CSUP_DLY,
	.tr_bt_udly     = FLASH_N25QXX_SPI_DEV_BYTE_DLY
};

/**
/* Definition of SPI device on chip select 2
/* for FM25WXX FRAM chip.
*/
cyg_spi_avr32_device_t fram_spi_device =
{
	.spi_device     = &cyg_spi_avr32_bus1,
	.dev_num        = FRAM_FM25WXX_SPI_DEV_CS_NUM,
	.bits           = FRAM_FM25WXX_SPI_DEV_BITS_NUM,
	.cl_pol         = FRAM_FM25WXX_SPI_DEV_CPOL,
	.cl_pha         = FRAM_FM25WXX_SPI_DEV_NCPHA,
	.cl_brate       = FRAM_FM25WXX_SPI_DEV_BUD_RATE,
	.cs_up_udly     = FRAM_FM25WXX_SPI_DEV_CSUP_DLY,
	.tr_bt_udly     = FRAM_FM25WXX_SPI_DEV_BYTE_DLY
};

/**
* Definition of FLASH drivers structure for FLASH and FRAM
*/
CYG_DEVS_FLASH_SPI_FM25WXX_DRIVER(fm25wxx_spi_fram,FRAM_FM25WXX_BASE_ADDRESS,&fram_spi_device);
CYG_DEVS_FLASH_SPI_N25QXX_DRIVER(n25qxx_spi_flash,FLASH_N25QXX_BASE_ADDRESS,&flash_spi_device);

/**
/* Definition of I2C bus
*/
CYG_AVR32_I2C_BUS(i2c_bus0,&cyg_avr32_i2c_init,AVR32_TWIM0_ADDRESS,CYGNUM_HAL_VECTOR_TWIM0,
					I2C_MODULE_INTERRUPT_PRIO,I2C_MODULE_CLOCK_FREQUENCY,I2C_MODULE_BUSS_FREQ);
/**
/* Definition of I2C device for external RCT.
*/
CYG_I2C_DEVICE(cyg_i2c_wallclock_pcf2129a,&i2c_bus0,WALLCLOCK_RTC_I2C_BUS_ADDRESS,0,0);
#endif

#ifdef BOOT_LOADER
#include <cyg/io/dataflash.h>

cyg_spi_avr32_device_t data_flash_spi_device =
{
	.spi_device     = &cyg_spi_avr32_bus0,
	.dev_num        = FLASH_AT45DBXX_SPI_DEV_CS_NUM,
	.bits           = FLASH_AT45DBXX_SPI_DEV_BITS_NUM,
	.cl_pol         = FLASH_AT45DBXX_SPI_DEV_CPOL,
	.cl_pha         = FLASH_AT45DBXX_SPI_DEV_NCPHA,
	.cl_brate       = FLASH_AT45DBXX_SPI_DEV_BUD_RATE,
	.cs_up_udly     = FLASH_AT45DBXX_SPI_DEV_CSUP_DLY,
	.tr_bt_udly     = FLASH_AT45DBXX_SPI_DEV_BYTE_DLY
};



CYG_DATAFLASH_FLASH_DRIVER(at45dbxx_spi_flash, &data_flash_spi_device, 
		FLASH_AT45DBXX_BASE_ADDRESS, 0, 15);

#endif

void hal_board_init()
{
    cyg_uint32 *clrreg = (cyg_uint32*)0xA0000000;
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[1];
	int i;
        
        for(i = 0; i < 512; i++)
            clrreg[i] = 0xbfdfbfdb;
	/*for(i = 0; i < 20000; i++)
	{
		__asm ("NOP");
	}*/
	//zpnut pull-up pro 32k z rtc open-drain
	//gpio_enable_pin_pull_up(AVR32_PIN_PB00);
	//gpio_port->puers = -1;
	//Na vývodu je externí pull down
	//a pull-up je zpanuty při spuštění proto se njprve vypne
	//a potom nastaví jako výstup a naství na nulu
	gpio_configure_pin(BT_PW_EN_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	gpio_configure_pin(FPGA_PWEN_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	
	gpio_configure_pin(ANL_PW_EN_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	//gpio_configure_pin(FPGA_SUSPEND_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW); 
	
	gpio_configure_pin(FPGA_DUAL_BOOT_PIN, GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	gpio_configure_pin(HV_PWEN_PIN,   GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	gpio_configure_pin(FPGA_RESET_PIN,GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	gpio_configure_pin(LCD_BACKLIGHT_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	
	gpio_configure_pin(DAC_AMPLIFIER_EN_PIN,  GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	//Vzpnutí celého napájení v nule
	gpio_configure_pin(SW_POWER_DOWN_PIN, GPIO_PULL_UP | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	
	gpio_configure_pin(FPGA_RDY_PIN, GPIO_PULL_DOWN | GPIO_DIR_INPUT | GPIO_DRIVE_LOW);
	
	//gpio_configure_pin(OSC_FPGA_EN, GPIO_PULL_DOWN | GPIO_DRIVE_LOW | GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	//Bluetooth USART
	gpio_configure_pin(AVR32_PIN_PB11, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(AVR32_PIN_PB10, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(AVR32_USART1_TXD_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(AVR32_USART1_RXD_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(AVR32_USART1_RTS_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(AVR32_USART1_CTS_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);

	//natavení vývodu zběrnice. Vývody jsou ponechány jako vstupy po resetu, ale jsou
	//zapnuty pull-down odpory aby se nemohlo FPGA napájet z vstupu. 11000101
	
	gpio_port = &AVR32_GPIO.port[2];
	gpio_port->puerc = 0xfff80000;
	gpio_port->pders = 0xfff80000;
	
	gpio_port = &AVR32_GPIO.port[3];
	
	gpio_port->puerc = 0x1e0fdfff;
	gpio_port->pders = 0x1e0fdfff;
	
	
#ifdef BOOT_LOADER
	PM_UNLOCK(AVR32_PM_HSBMASK);
	AVR32_PM.hsbmask = 0x7ea;
	
	PM_UNLOCK(AVR32_PM_PBAMASK);
	AVR32_PM.pbamask = 0x1baf;
	
	PM_UNLOCK(AVR32_PM_PBBMASK);
	AVR32_PM.pbbmask = 0x1f;
	
	PM_UNLOCK(AVR32_PM_PBCMASK);
#ifdef DEBUG_IO
	AVR32_PM.pbcmask = 	0xcd;//0x45;
#else
    AVR32_PM.pbcmask = 	0x4d;//0x45;
#endif
	FPGA_POWER_ENABLE;
	FPGA_OSC_ENABLE;
	FPGA_RESET_DISABLE;
#else
	
	//disable clock for unused peripherals
	PM_UNLOCK(AVR32_PM_HSBMASK);
	AVR32_PM.hsbmask = 0x7ea;
					
	PM_UNLOCK(AVR32_PM_PBAMASK);   
	AVR32_PM.pbamask = 0x1baf | 0x800000 | 0x1000000;
		
	PM_UNLOCK(AVR32_PM_PBBMASK);
	AVR32_PM.pbbmask = 0x1f;
		
	PM_UNLOCK(AVR32_PM_PBCMASK);
#ifdef DEBUG_IO
	AVR32_PM.pbcmask = 	0xc5;//0x45; //poak asi vypnout i dbg usart
#else
    AVR32_PM.pbcmask = 	0x45;//0x45;
#endif
#endif
	
	
}					
