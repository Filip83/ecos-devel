#ifndef CYGONCE_HAL_VAR_INTR_H
#define CYGONCE_HAL_VAR_INTR_H
//==========================================================================
//
//      var_intr.h
//
//      HAL Interrupt and clock assignments for iMX6 variants
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2011 Free Software Foundation, Inc.                        
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
// Author(s):     Mike Jones
// Date:          2013-08-08
// Purpose:       Define Interrupt support
// Description:   The interrupt specifics for Freescale iMX6 variants are
//                defined here.
//
// Usage:         #include <cyg/hal/var_intr.h>
//                However applications should include using <cyg/hal/hal_intr.h>
//                instead to allow for platform overrides.
//
//####DESCRIPTIONEND####
//
//==========================================================================

//#include <cyg/hal/plf_intr.h>
#include <cyg/hal/hal_smp.h>
#include <cyg/hal/cortex_a9.h>

//==========================================================================

typedef enum {
    CYGNUM_HAL_INTERRUPT_SW_0 = 0, //!< Software interrupt 0.
    CYGNUM_HAL_INTERRUPT_SW_1 = 1, //!< Software interrupt 1.
    CYGNUM_HAL_INTERRUPT_SW_2 = 2, //!< Software interrupt 2.
    CYGNUM_HAL_INTERRUPT_SW_3 = 3, //!< Software interrupt 3.
    CYGNUM_HAL_INTERRUPT_SW_4 = 4, //!< Software interrupt 4.
    CYGNUM_HAL_INTERRUPT_SW_5 = 5, //!< Software interrupt 5.
    CYGNUM_HAL_INTERRUPT_SW_6 = 6, //!< Software interrupt 6.
    CYGNUM_HAL_INTERRUPT_SW_7 = 7, //!< Software interrupt 7.
    CYGNUM_HAL_INTERRUPT_SW_8 = 8, //!< Software interrupt 8.
    CYGNUM_HAL_INTERRUPT_SW_9 = 9, //!< Software interrupt 9.
    CYGNUM_HAL_INTERRUPT_SW_10 = 10,   //!< Software interrupt 10.
    CYGNUM_HAL_INTERRUPT_SW_11 = 11,   //!< Software interrupt 11.
    CYGNUM_HAL_INTERRUPT_SW_12 = 12,   //!< Software interrupt 12.
    CYGNUM_HAL_INTERRUPT_SW_13 = 13,   //!< Software interrupt 13.
    CYGNUM_HAL_INTERRUPT_SW_14 = 14,   //!< Software interrupt 14.
    CYGNUM_HAL_INTERRUPT_SW_15 = 15,   //!< Software interrupt 15.
    CYGNUM_HAL_INTERRUPT_RSVD_16 = 16, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_17 = 17, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_18 = 18, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_19 = 19, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_20 = 20, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_21 = 21, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_22 = 22, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_23 = 23, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_24 = 24, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_25 = 25, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_26 = 26, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_27 = 27, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_28 = 28, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_29 = 29, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_30 = 30, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_RSVD_31 = 31, //!< Reserved.
    CYGNUM_HAL_INTERRUPT_IOMUXC_GPR = 32,   //!< General Purpose Register 1 from IOMUXC. Used to notify cores on exception condition while boot.
    CYGNUM_HAL_INTERRUPT_CHEETAH_CSYSPWRUPREQ = 33,  //!< @todo Listed as DAP in RM
    CYGNUM_HAL_INTERRUPT_SDMA = 34,  //!< Logical OR of all 48 SDMA interrupt requests/events from all channels.
    CYGNUM_HAL_INTERRUPT_VPU_JPG = 35,   //!< JPEG codec interrupt request.
    CYGNUM_HAL_INTERRUPT_SNVS_LP_SET_PWR_OFF = 36,   //!< PMIC power off request.
    CYGNUM_HAL_INTERRUPT_IPU1_ERR = 37,  //!< IPU1 error interrupt request.
    CYGNUM_HAL_INTERRUPT_IPU1_FUNC = 38, //!< IPU1 sync interrupt request.
    CYGNUM_HAL_INTERRUPT_IPU2_ERR = 39,  //!< IPU2 error interrupt request.
    CYGNUM_HAL_INTERRUPT_IPU2_FUNC = 40, //!< IPU2 sync interrupt request.
    CYGNUM_HAL_INTERRUPT_GPU3D = 41, //!< GPU3D interrupt request.
    CYGNUM_HAL_INTERRUPT_GPU2D = 42, //!< Idle interrupt from GPU2D (for S/W power gating).
    CYGNUM_HAL_INTERRUPT_OPENVG_XAQ2 = 43,   //!< GPU2D general interrupt request.
    CYGNUM_HAL_INTERRUPT_VPU_IPI = 44,   //!< VPU interrupt request.
    CYGNUM_HAL_INTERRUPT_APBHDMA = 45,   //!< Logical OR of 4 signals: dma_chan[0-3]_irq, GPMI operation channel description complete interrupt.
    CYGNUM_HAL_INTERRUPT_EIM = 46,   //!< EIM interrupt request.
    CYGNUM_HAL_INTERRUPT_BCH = 47,   //!< BCH operation complete interrupt.
    CYGNUM_HAL_INTERRUPT_GPMI = 48,  //!< GPMI operation timeout error interrupt.
    CYGNUM_HAL_INTERRUPT_DTCP = 49,  //!< DTCP interrupt request.
    CYGNUM_HAL_INTERRUPT_VDOA = 50,  //!< Logical OR of VDOA interrupt requests.
    CYGNUM_HAL_INTERRUPT_SNVS = 51,  //!< SNVS consolidated interrupt.
    CYGNUM_HAL_INTERRUPT_SNVS_SEC = 52,  //!< SNVS security interrupt.
    CYGNUM_HAL_INTERRUPT_CSU = 53,   //!< CSU interrupt request 1. Indicates to the processor that one or more alarm inputs were asserted.
    CYGNUM_HAL_INTERRUPT_USDHC1 = 54,    //!< uSDHC1 (Enhanced SDHC) interrupt request.
    CYGNUM_HAL_INTERRUPT_USDHC2 = 55,    //!< uSDHC2 (Enhanced SDHC) interrupt request.
    CYGNUM_HAL_INTERRUPT_USDHC3 = 56,    //!< uSDHC3 (Enhanced SDHC) interrupt request.
    CYGNUM_HAL_INTERRUPT_USDHC4 = 57,    //!< uSDHC4 (Enhanced SDHC) interrupt request.
// TODO: Number all UARTS from 1 like SDK.
    CYGNUM_HAL_INTERRUPT_UART0 = 58, //!< Logical OR of UART1 interrupt requests.
    CYGNUM_HAL_INTERRUPT_UART1 = 59, //!< Logical OR of UART2 interrupt requests.
    CYGNUM_HAL_INTERRUPT_UART2 = 60, //!< Logical OR of UART3 interrupt requests.
    CYGNUM_HAL_INTERRUPT_UART3 = 61, //!< Logical OR of UART4 interrupt requests.
    CYGNUM_HAL_INTERRUPT_UART4 = 62, //!< Logical OR of UART5 interrupt requests.
    CYGNUM_HAL_INTERRUPT_ECSPI1 = 63,    //!< eCSPI1 interrupt request.
    CYGNUM_HAL_INTERRUPT_ECSPI2 = 64,    //!< eCSPI2 interrupt request.
    CYGNUM_HAL_INTERRUPT_ECSPI3 = 65,    //!< eCSPI3 interrupt request.
    CYGNUM_HAL_INTERRUPT_ECSPI4 = 66,    //!< eCSPI4 interrupt request.
    CYGNUM_HAL_INTERRUPT_ECSPI5 = 67,    //!< eCSPI5 interrupt request.
    // Changed I2C to be zero based to match I2C device.
    CYGNUM_HAL_INTERRUPT_I2C0 = 68,  //!< I2C1 interrupt request.
    CYGNUM_HAL_INTERRUPT_I2C1 = 69,  //!< I2C2 interrupt request.
    CYGNUM_HAL_INTERRUPT_I2C2 = 70,  //!< I2C3 interrupt request.
    CYGNUM_HAL_INTERRUPT_SATA = 71,  //!< SATA interrupt request.
    CYGNUM_HAL_INTERRUPT_USBOH3_UH1 = 72,    //!< USB Host 1 interrupt request.
    CYGNUM_HAL_INTERRUPT_USBOH3_UH2 = 73,    //!< USB Host 2 interrupt request.
    CYGNUM_HAL_INTERRUPT_USBOH3_UH3 = 74,    //!< USB Host 3 interrupt request.
    CYGNUM_HAL_INTERRUPT_USBOH3_UOTG = 75,   //!< USB OTG interrupt request.
    CYGNUM_HAL_INTERRUPT_USB_UTMI0 = 76, //!< UTMI0 interrupt request.
    CYGNUM_HAL_INTERRUPT_USB_UTMI1 = 77, //!< UTMI1 interrupt request.
    CYGNUM_HAL_INTERRUPT_SSI1 = 78,  //!< SSI1 interrupt request.
    CYGNUM_HAL_INTERRUPT_SSI2 = 79,  //!< SSI2 interrupt request.
    CYGNUM_HAL_INTERRUPT_SSI3 = 80,  //!< SSI3 interrupt request.
    CYGNUM_HAL_INTERRUPT_TEMPERATURE = 81,   //!< Temperature Sensor (temp. greater than threshold) interrupt request.
    CYGNUM_HAL_INTERRUPT_ASRC = 82,  //!< ASRC interrupt request.
    CYGNUM_HAL_INTERRUPT_ESAI = 83,  //!< ESAI interrupt request.
    CYGNUM_HAL_INTERRUPT_SPDIF = 84, //!< Logical OR of SPDIF TX and SPDIF RX interrupts.
    CYGNUM_HAL_INTERRUPT_MLB = 85,   //!< MLB error interrupt request.
    CYGNUM_HAL_INTERRUPT_PMU_ANA_BO = 86,    //!< PMU analog regulator brown-out interrupt request.
    CYGNUM_HAL_INTERRUPT_GPT = 87,   //!< Logical OR of GPT rollover interrupt line, input capture 1 & 2 lines, output compare 1, 2 & 3 interrupt lines.
    CYGNUM_HAL_INTERRUPT_EPIT1 = 88, //!< EPIT1 output compare interrupt.
    CYGNUM_HAL_INTERRUPT_EPIT2 = 89, //!< EPIT2 output compare interrupt.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT7 = 90,    //!< INT7 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT6 = 91,    //!< INT6 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT5 = 92,    //!< INT5 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT4 = 93,    //!< INT4 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT3 = 94,    //!< INT3 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT2 = 95,    //!< INT2 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT1 = 96,    //!< INT1 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT0 = 97,    //!< INT0 interrupt request.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT15_0 = 98, //!< Combined interrupt indication for GPIO1 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO1_INT31_16 = 99,    //!< Combined interrupt indication for GPIO1 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_GPIO2_INT15_0 = 100,    //!< Combined interrupt indication for GPIO2 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO2_INT31_16 = 101,   //!< Combined interrupt indication for GPIO2 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_GPIO3_INT15_0 = 102,    //!< Combined interrupt indication for GPIO3 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO3_INT31_16 = 103,   //!< Combined interrupt indication for GPIO3 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_GPIO4_INT15_0 = 104,    //!< Combined interrupt indication for GPIO4 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO4_INT31_16 = 105,   //!< Combined interrupt indication for GPIO4 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_GPIO5_INT15_0 = 106,    //!< Combined interrupt indication for GPIO5 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO5_INT31_16 = 107,   //!< Combined interrupt indication for GPIO5 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_GPIO6_INT15_0 = 108,    //!< Combined interrupt indication for GPIO6 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO6_INT31_16 = 109,   //!< Combined interrupt indication for GPIO6 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_GPIO7_INT15_0 = 110,    //!< Combined interrupt indication for GPIO7 signals 0 - 15.
    CYGNUM_HAL_INTERRUPT_GPIO7_INT31_16 = 111,   //!< Combined interrupt indication for GPIO7 signals 16 - 31.
    CYGNUM_HAL_INTERRUPT_WDOG1 = 112,    //!< WDOG1 timer reset interrupt request.
    CYGNUM_HAL_INTERRUPT_WDOG2 = 113,    //!< WDOG2 timer reset interrupt request.
    CYGNUM_HAL_INTERRUPT_KPP = 114,  //!< Key Pad interrupt request.
    CYGNUM_HAL_INTERRUPT_PWM1 = 115, //!< Cumulative interrupt line for PWM1. Logical OR of rollover, compare, and FIFO waterlevel crossing interrupts.
    CYGNUM_HAL_INTERRUPT_PWM2 = 116, //!< Cumulative interrupt line for PWM2. Logical OR of rollover, compare, and FIFO waterlevel crossing interrupts.
    CYGNUM_HAL_INTERRUPT_PWM3 = 117, //!< Cumulative interrupt line for PWM3. Logical OR of rollover, compare, and FIFO waterlevel crossing interrupts.
    CYGNUM_HAL_INTERRUPT_PWM4 = 118, //!< Cumulative interrupt line for PWM4. Logical OR of rollover, compare, and FIFO waterlevel crossing interrupts.
    CYGNUM_HAL_INTERRUPT_CCM_INT1 = 119, //!< CCM interrupt request 1.
    CYGNUM_HAL_INTERRUPT_CCM_INT2 = 120, //!< CCM interrupt request 2.
    CYGNUM_HAL_INTERRUPT_GPC_INT1 = 121, //!< GPC interrupt request 1.
    CYGNUM_HAL_INTERRUPT_GPC_INT2 = 122, //!< GPC interrupt request 2.
    CYGNUM_HAL_INTERRUPT_SRC = 123,  //!< SRC interrupt request.
    CYGNUM_HAL_INTERRUPT_CHEETAH_L2 = 124,   //!< Logical OR of all L2 interrupt requests.
    CYGNUM_HAL_INTERRUPT_CHEETAH_PARITY = 125,   //!< Parity Check error interrupt request.
    CYGNUM_HAL_INTERRUPT_CHEETAH_PERFORM = 126,  //!< Logical OR of Performance Unit interrupts.
    CYGNUM_HAL_INTERRUPT_CHEETAH_TRIGGER = 127,  //!< Logical OR of CTI trigger outputs.
    CYGNUM_HAL_INTERRUPT_SRC_CPU_WDOG = 128, //!< Combined CPU wdog interrupts (4x) out of SRC.
    CYGNUM_HAL_INTERRUPT_INTERRUPT_129 = 129,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_INTERRUPT_130 = 130,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_INTERRUPT_131 = 131,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_CSI_INTR1 = 132,    //!< MIPI CSI interrupt request 1.
    CYGNUM_HAL_INTERRUPT_CSI_INTR2 = 133,    //!< MIPI CSI interrupt request 2.
    CYGNUM_HAL_INTERRUPT_DSI = 134,  //!< MIPI DSI interrupt request.
    CYGNUM_HAL_INTERRUPT_HSI = 135,  //!< MIPI HSI interrupt request.
    CYGNUM_HAL_INTERRUPT_SJC = 136,  //!< SJC interrupt from General Purpose register.
    CYGNUM_HAL_INTERRUPT_CAAM_INT0 = 137,    //!< CAAM job ring 0 interrupt.
    CYGNUM_HAL_INTERRUPT_CAAM_INT1 = 138,    //!< CAAM job ring 1 interrupt.
    CYGNUM_HAL_INTERRUPT_INTERRUPT_139 = 139,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_TZASC1 = 140,   //!< ASC1 interrupt request.
    CYGNUM_HAL_INTERRUPT_TZASC2 = 141,   //!< ASC2 interrupt request.
    CYGNUM_HAL_INTERRUPT_FLEXCAN1 = 142, //!< FLEXCAN1 combined interrupt. Logical OR of ini_int_busoff, ini_int_error, ipi_int_mbor, ipi_int_rxwarning, ipi_int_txwarning and ipi_int_wakein.
    CYGNUM_HAL_INTERRUPT_FLEXCAN2 = 143, //!< FLEXCAN2 combined interrupt. Logical OR of ini_int_busoff, ini_int_error, ipi_int_mbor, ipi_int_rxwarning, ipi_int_txwarning and ipi_int_wakein.
    CYGNUM_HAL_INTERRUPT_PERFMON1 = 144,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_PERFMON2 = 145,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_PERFMON3 = 146,    //!< Reserved.
    CYGNUM_HAL_INTERRUPT_HDMI_TX = 147,  //!< HDMI master interrupt request.
    CYGNUM_HAL_INTERRUPT_HDMI_TX_WAKEUP = 148,   //!< HDMI CEC engine dedicated interrupt signal raised by a wake-up event.
    CYGNUM_HAL_INTERRUPT_MLB_AHB0 = 149, //!< Channels [31:0] interrupt requests.
    CYGNUM_HAL_INTERRUPT_ENET = 150, //!< MAC 0 IRQ, Logical OR of:
                        //! - MAC 0 Periodic Timer Overflow
                        //! - MAC 0 Time Stamp Available
                        //! - MAC 0 Payload Receive Error
                        //! - MAC 0 Transmit FIFO Underrun
                        //! - MAC 0 Collision Retry Limit
                        //! - MAC 0 Late Collision
                        //! - MAC 0 Ethernet Bus Error
                        //! - MAC 0 MII Data Transfer Done
                        //! - MAC 0 Receive Buffer Done
                        //! - MAC 0 Receive Frame Done
                        //! - MAC 0 Transmit Buffer Done
                        //! - MAC 0 Transmit Frame Done
                        //! - MAC 0 Graceful Stop
                        //! - MAC 0 Babbling Transmit Error
                        //! - MAC 0 Babbling Receive Error
                        //! - MAC 0 Wakeup Request [synchronous]
    CYGNUM_HAL_INTERRUPT_ENET_1588 = 151,    //!< ￼MAC 0 1588 Timer interrupt [synchronous] request.
    CYGNUM_HAL_INTERRUPT_PCIE_1 = 152,   //!< PCIe interrupt request 1.
    CYGNUM_HAL_INTERRUPT_PCIE_2 = 153,   //!< PCIe interrupt request 2.
    CYGNUM_HAL_INTERRUPT_PCIE_3 = 154,   //!< PCIe interrupt request 3.
    CYGNUM_HAL_INTERRUPT_PCIE_4 = 155,   //!< PCIe interrupt request 4.
    CYGNUM_HAL_INTERRUPT_DCIC1 = 156,    //!< Logical OR of DCIC1 interrupt requests.
    CYGNUM_HAL_INTERRUPT_DCIC2 = 157,    //!< Logical OR of DCIC2 interrupt requests.
    CYGNUM_HAL_INTERRUPT_MLB_AHB1 = 158, //!< Logical OR of channel[63:32] interrupt requests.
    CYGNUM_HAL_INTERRUPT_PMU_DIG_BO = 159,    //!< //!< PMU digital regulator brown-out interrupt request.
    IMX_INTERRUPT_COUNT = 160   //!< Total number of interrupts.
} IMX6ExtInterrupt_e;

#define CYGNUM_HAL_INTERRUPT_NVIC_MAX (CYGNUM_HAL_INTERRUPT_Reserved119)

// TODO: This seem to be dups. There is probably duplicate definitions in the ARM hal.
// If this was moved to a cortexa it would be fine.
#define CYGNUM_HAL_ISR_MIN            0
// In order to use the second line with the real max would require fixing the asserts
// in intr.cxx so they let through the arch defined value for none, which is -1.
// Therefore, the strategy here is to use NONE to keep the asserts ok, and use
// the real max for hal_misc.c filtering.
#define CYGNUM_HAL_ISR_MAX            CYGNUM_HAL_INTERRUPT_NONE
#define CYGNUM_HAL_ISR_REAL_MAX       CYGNUM_HAL_INTERRUPT_PMU_DIG_BO
#define CYGNUM_HAL_ISR_COUNT          IMX_INTERRUPT_COUNT

// TODO: Conflicts with hal_intr.h
//#define CYGNUM_HAL_VSR_MIN            0
//#ifndef CYGNUM_HAL_VSR_MAX
//#define CYGNUM_HAL_VSR_MAX           (CYGNUM_HAL_VECTOR_SYS_TICK+ 
//                                       CYGNUM_HAL_INTERRUPT_NVIC_MAX)
//#endif
//#define CYGNUM_HAL_VSR_COUNT          (CYGNUM_HAL_VSR_MAX+1)

#ifdef CYGPKG_HAL_SMP_SUPPORT

#define CYGNUM_HAL_SMP_CPU_INTERRUPT_VECTOR( _n_ ) (CYGNUM_HAL_INTERRUPT_SW_0+(_n_))

#endif

//==========================================================================
// Interrupt mask and config for variant-specific devices

// PORT Pin interrupts

#define CYGHWR_HAL_IMX6_PIN_IRQ_VECTOR(__pin) \
    (CYGNUM_HAL_INTERRUPT_PORTA + CYGHWR_HAL_IMX6_PIN_PORT(__pin))

//===========================================================================
// Interrupt resources exported by HAL to device drivers

// Export Interrupt vectors to serial driver.
// TODO: Make indexed by 1
#define CYGNUM_IO_SERIAL_FREESCALE_UART0_INT_VECTOR \
            CYGNUM_HAL_INTERRUPT_UART0
#define CYGNUM_IO_SERIAL_FREESCALE_UART1_INT_VECTOR \
            CYGNUM_HAL_INTERRUPT_UART1
#define CYGNUM_IO_SERIAL_FREESCALE_UART2_INT_VECTOR \
            CYGNUM_HAL_INTERRUPT_UART2
#define CYGNUM_IO_SERIAL_FREESCALE_UART3_INT_VECTOR \
            CYGNUM_HAL_INTERRUPT_UART3
#define CYGNUM_IO_SERIAL_FREESCALE_UART4_INT_VECTOR \
            CYGNUM_HAL_INTERRUPT_UART4

// Export Interrupt vectors to ENET driver.

#define CYGNUM_FREESCALE_ENET0_1588_TIMER_INT_VECTOR \
            CYGNUM_HAL_INTERRUPT_ENET_1588
#define CYGNUM_FREESCALE_ENET0_TRANSMIT_INT_VECTOR   \
            CYGNUM_HAL_INTERRUPT_ENET
#define CYGNUM_FREESCALE_ENET0_RECEIVE_INT_VECTOR    \
            CYGNUM_HAL_INTERRUPT_ENET
#define CYGNUM_FREESCALE_ENET0_ERROR_INT_VECTOR      \
            CYGNUM_HAL_INTERRUPT_ENET

//----------------------------------------------------------------------------
// Interrupt routines

#define HAL_GICD (gicd_t *) (hal_get_private_peripheral_base() + kGICDBaseOffset)
#define HAL_GICC (gicc_t *) (hal_get_private_peripheral_base() + kGICCBaseOffset)
#define HAL_IRQ_REG_OFFSET(__vector) (cyg_uint32) (__vector/32)
#define HAL_IRQ_REG_OFFSET2(__vector) (cyg_uint32) (__vector/16)
//#define HAL_IRQ_BIT_OFFSET(__vector) (cyg_uint32) (__vector & 0x1F);
//#define HAL_IRQ_BIT_OFFSET2(__vector) (cyg_uint32) ((__vector & 0x0F) << 1);
#define HAL_IRQ_GET_BIT_MASK(__vector) (cyg_uint32) (1 << (__vector & 0x1F))
#define HAL_IRQ_GET_BIT_MASK2(__vector) (cyg_uint32) (1 << ((__vector & 0x0F) << 1))

//! @brief
typedef enum {
    CPU_0,
    CPU_1,
    CPU_2,
    CPU_3,
} cpuid_e;

typedef enum
{
    UseTargetList = 0,
    AllOtherCPUs = 1,
    OnlyThisCPU = 2
}sgi_filter_e;

#if defined(__cplusplus)
extern "C" {
#endif

void hal_interrupt_mask(int vector);
void hal_interrupt_unmask(int vector);
void hal_interrupt_acknowledge(int vector);
void hal_interrupt_configure(int vector, int level, int up);
void hal_interrupt_set_level(int vector, int level);
void hal_disable_interrupt(cyg_uint32 vector);
void hal_enable_interrupt(cyg_uint32 vector);
void hal_interrupt_set_cpu_target(cyg_uint32 vector, unsigned cpu, cyg_bool enable);
void hal_interrupt_distributor_enable(cyg_bool enable);
void hal_interrupt_cpu_enable(cyg_bool enable);
void hal_interrupt_init(void);
void hal_interrupt_init_cpu(void);
void hal_interrupt_send_sgi(cyg_uint32 vector, cyg_uint32 target_list, cyg_uint32 filter_list);


#if defined(__cplusplus)
}
#endif

#ifdef CYGPKG_HAL_SMP_SUPPORT

// Additional SMP interrupt configuration support.

__externC void hal_interrupt_set_cpu( CYG_WORD32 vector, HAL_SMP_CPU_TYPE cpu );
__externC void hal_interrupt_get_cpu( CYG_WORD32 vector, HAL_SMP_CPU_TYPE *cpu );

#define HAL_INTERRUPT_SET_CPU( _vector_, _cpu_ )                \
    hal_interrupt_set_cpu_target(_vector_, _cpu_, true)


#define HAL_INTERRUPT_GET_CPU( _vector_, _cpu_ )                \
{                                                               \
    _cpu_ = hal_cpu_get_current();                              \
}


#endif

#endif // CYGONCE_HAL_VAR_INTR_H
// EOF var_intr.h
