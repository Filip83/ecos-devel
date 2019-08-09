/*
 * MGS1_board_config.h
 *
 * Created: 15.5.2013 10:27:30
 *  Author: Filip
 */ 


#ifndef MGS1_BOARD_CONFIG_H_
#define MGS1_BOARD_CONFIG_H_



/**
/* Nastaveni cphol a ncpha viz dokumetace str. 660
/* SPI mod 0 odpov9d8 0,1
*/
#define FLASH_N25QXX_SPI_DEV_CS_NUM				0
#define FLASH_N25QXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FLASH_N25QXX_SPI_DEV_CPOL				0
#define FLASH_N25QXX_SPI_DEV_NCPHA				1
#define FLASH_N25QXX_SPI_DEV_BUD_RATE			16000000
#define FLASH_N25QXX_SPI_DEV_CSUP_DLY			0
#define FLASH_N25QXX_SPI_DEV_BYTE_DLY			0
#define FLASH_N25QXX_BASE_ADDRESS				0x20008000

#define FRAM_FM25WXX_SPI_DEV_CS_NUM				2
#define FRAM_FM25WXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FRAM_FM25WXX_SPI_DEV_CPOL				0
#define FRAM_FM25WXX_SPI_DEV_NCPHA				1
#define FRAM_FM25WXX_SPI_DEV_BUD_RATE			16000000
#define FRAM_FM25WXX_SPI_DEV_CSUP_DLY			0
#define FRAM_FM25WXX_SPI_DEV_BYTE_DLY			0
#define FRAM_FM25WXX_BASE_ADDRESS				0x20000000

#define FLASH_AT45DBXX_SPI_DEV_CS_NUM			0
#define FLASH_AT45DBXX_SPI_DEV_BITS_NUM			0 //0 -> 8
#define FLASH_AT45DBXX_SPI_DEV_CPOL				0
#define FLASH_AT45DBXX_SPI_DEV_NCPHA			1
#define FLASH_AT45DBXX_SPI_DEV_BUD_RATE			16000000
#define FLASH_AT45DBXX_SPI_DEV_CSUP_DLY			0
#define FLASH_AT45DBXX_SPI_DEV_BYTE_DLY			0
#define FLASH_AT45DBXX_BASE_ADDRESS				0x30000000

/**
/* I2C module settings
*/
#define I2C_MODULE_CLOCK_FREQUENCY				CYGHWR_HAL_AVR32_CPU_FREQ*1e6
#define I2C_MODULE_INTERRUPT_PRIO				0
#define I2C_MODULE_BUSS_FREQ					10e3

#define WALLCLOCK_RTC_I2C_BUS_ADDRESS			0x51

/**
/* GPIO pin definition macros
*/
#define BT_PW_EN_PIN			AVR32_PIN_PB13
#define LCD_BACKLIGHT_PIN		AVR32_PIN_PB22
#define HV_PWEN_PIN			AVR32_PIN_PB23
#define FPGA_RESET_PIN			AVR32_PIN_PC00
#define FPGA_RDY_PIN 			AVR32_PIN_PC17
#define SW_POWER_DOWN_PIN		AVR32_PIN_PD22
#define FPGA_PWEN_PIN			AVR32_PIN_PD23
#define ANL_PW_EN_PIN			AVR32_PIN_PB21
#define FPGA_SUSPEND_PIN		AVR32_PIN_PB26
#define FPGA_DUAL_BOOT_PIN		AVR32_PIN_PD30
#define DAC_AMPLIFIER_EN_PIN            AVR32_PIN_PB19

/**
/* Pin access and manipulation macros
*/

#define SYSTEM_POWER_DOWN						\
			gpio_set_pin_low(SW_POWER_DOWN_PIN)
			
#define LCD_BACKLIGHT_ENABLE					\
			gpio_set_pin_high(LCD_BACKLIGHT_PIN)
			
#define LCD_BACKLIGHT_DISABLE					\
			gpio_set_pin_low(LCD_BACKLIGHT_PIN)
			
#define BLUETOOTH_POWER_ENABLE					\
			gpio_set_pin_high(BT_PW_EN_PIN)
			
#define BLUETOOTH_POWER_DISABLE					\
			gpio_set_pin_low(BT_PW_EN_PIN)
			
			
#define HIGH_VOLTAGE_POWER_ENABLE				\
			gpio_set_pin_high(HV_PWEN_PIN)
			
#define HIGH_VOLTAGE_POWER_DISABLE				\
			gpio_set_pin_low(HV_PWEN_PIN)
			
#define ANALOG_VOLTAGE_POWER_ENABLE				\
			gpio_set_pin_high(ANL_PW_EN_PIN)
			
#define ANALOG_VOLTAGE_POWER_DISABLE				\
			gpio_set_pin_low(ANL_PW_EN_PIN)
			
#define FPGA_SUSPEND_ENABLE				\
			gpio_set_pin_high(FPGA_SUSPEND_PIN)
			
#define FPGA_SUSPEND_DISABLLE				\
			gpio_set_pin_low(FPGA_SUSPEND_PIN)

#define FPGA_POWER_ENABLE						\
			gpio_set_pin_low(FPGA_PWEN_PIN)
			
#define FPGA_POWER_DISABLE						\
			gpio_set_pin_high(FPGA_PWEN_PIN)
			
#define FPGA_DUAL_BOOT_START						\
			gpio_set_pin_high(FPGA_DUAL_BOOT_PIN)
			
#define FPGA_DUAL_BOOT_STOP						\
			gpio_set_pin_low(FPGA_DUAL_BOOT_PIN)
			
#define FPGA_OSC_ENABLE	

#define FPGA_OCS_DISABLE
			
#define FPGA_RESET_ENABLE						\
			gpio_set_pin_low(FPGA_RESET_PIN)
			
			
#define FPGA_RESET_DISABLE						\
			gpio_set_pin_high(FPGA_RESET_PIN)
			
#define BEEPER_AMPLIFIER_DISABLE		\
		       gpio_set_pin_low(DAC_AMPLIFIER_EN_PIN)
		 
#define BEEPER_AMPLIFIER_ENABLE		\
		       gpio_set_pin_high(DAC_AMPLIFIER_EN_PIN)
			
#define FPGA_RESET(reset) (reset) ? FPGA_RESET_ENABLE : FPGA_RESET_DISABLE

#define IS_FPGA_CONFIGURED	gpio_get_pin_value(FPGA_RDY_PIN)

#define FPGA_SMC_CHIP_SELCET_BASE_ADDRESS 0xD0000000

void hal_board_init();

#define SET_HW_ERRROR(_code_) _hw_error |= _code_

#define HW_ERROR_OSC0_NOT_RUNNING			0x0000000000000001ll
#define HW_ERROR_OSC1_NOT_RUNNING			0x0000000000000002ll
#define HW_ERROR_OSC32_NOT_RUNNING			0x0000000000000004ll
#define HW_ERROR_RC8M_NOT_RUNNING			0x0000000000000008ll
#define HW_ERROR_MAIN_CLOCK_NOT_RUNNIGN     0x0000000000000010ll

#define HW_ERROR_RTC_NO_RESPONSE		0x0000000000000020ll
#define HW_ERROR_RTC_CHECK_TIME			0x0000000000000040ll

#define HW_ERROR_MAIN_FRAM_NOT_INIT     0x0000000000000080ll
#define HW_ERROR_MAIN_FLASH_NOT_INIT     0x0000000000000100ll
#define HW_ERROR_MOUNT			 0x0000000000000200ll



#endif /* MGS1_BOARD_CONFIG_H_ */
