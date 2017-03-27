#include <cyg/infra/cyg_type.h>

/*
 * Copyright (c) 2011-2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __USDHC_HOST_H__
#define __USDHC_HOST_H__

/*---------------------------------------- Macro -------------------------------------------*/

//#define ESDHC_SYSCTL_INITA            ((cyg_uint32 )0x08000000)
//#define ESDHC_SOFTWARE_RESET          ((cyg_uint32 )0x01000000)

//#define ESDHC_BUS_WIDTH_MASK          ((cyg_uint32 )0x00000006)
//#define ESDHC_ENDIAN_MODE_MASK        ((cyg_uint32 )0x00000030)

#define ESDHC_LITTLE_ENDIAN_MODE      ((cyg_uint32 )0x00000002)
#define ESDHC_HW_BIG_ENDIAN_MODE      ((cyg_uint32 )0x00000001)

//#define ESDHC_SYSCTL_SDCLKEN_MASK     ((cyg_uint32 )0x00000008)
//#define ESDHC_PRSSTAT_SDSTB_BIT       ((cyg_uint32 )0x00000008)
//#define ESDHC_SYSCTL_FREQ_MASK        ((cyg_uint32 )0x000FFFF0)
#define ESDHC_SYSCTL_FREQ_MASK          (BM_USDHC_SYS_CTRL_DVS | BM_USDHC_SYS_CTRL_SDCLKFS | BM_USDHC_SYS_CTRL_DTOCV)

//#define ESDHC_SYSCTL_INPUT_CLOCK_MASK ((cyg_uint32 )0x00000007)
//#define ESDHC_IRQSTATEN_DTOESEN       ((cyg_uint32 )0x00100000)

//#define ESDHC_SYSCTL_DTOCV_VAL        ((cyg_uint32 )0x000E0000)
#define ESDHC_SYSCTL_DTOCV_VAL        ((cyg_uint32 )0x0000000E)

//#define ESDHC_CLEAR_INTERRUPT         ((cyg_uint32 )0x117F01FF)
#define ESDHC_CLEAR_INTERRUPT         (BM_USDHC_INT_STATUS_CC | \
                                       BM_USDHC_INT_STATUS_TC | \
                                       BM_USDHC_INT_STATUS_BGE | \
                                       BM_USDHC_INT_STATUS_DINT | \
                                       BM_USDHC_INT_STATUS_BWR | \
                                       BM_USDHC_INT_STATUS_BRR | \
                                       BM_USDHC_INT_STATUS_CINS | \
                                       BM_USDHC_INT_STATUS_CRM | \
                                       BM_USDHC_INT_STATUS_CINT | \
                                       BM_USDHC_INT_STATUS_CTOE | \
                                       BM_USDHC_INT_STATUS_CCE | \
                                       BM_USDHC_INT_STATUS_CEBE | \
                                       BM_USDHC_INT_STATUS_CIE | \
                                       BM_USDHC_INT_STATUS_DTOE | \
                                       BM_USDHC_INT_STATUS_DCE | \
                                       BM_USDHC_INT_STATUS_DEBE | \
                                       BM_USDHC_INT_STATUS_AC12E | \
                                       BM_USDHC_INT_STATUS_DMAE)
                                       
//#define ESDHC_INTERRUPT_ENABLE        ((cyg_uint32 )0x007F013F)
#define ESDHC_INTERRUPT_ENABLE        (BM_USDHC_INT_STATUS_EN_CCSEN | \
                                       BM_USDHC_INT_STATUS_EN_TCSEN | \
                                       BM_USDHC_INT_STATUS_EN_BGESEN | \
                                       BM_USDHC_INT_STATUS_EN_DINTSEN | \
                                       BM_USDHC_INT_STATUS_EN_BWRSEN | \
                                       BM_USDHC_INT_STATUS_EN_BRRSEN | \
                                       BM_USDHC_INT_STATUS_EN_CTOESEN | \
                                       BM_USDHC_INT_STATUS_EN_CCESEN | \
                                       BM_USDHC_INT_STATUS_EN_CEBESEN | \
                                       BM_USDHC_INT_STATUS_EN_CIESEN | \
                                       BM_USDHC_INT_STATUS_EN_DTOESEN | \
                                       BM_USDHC_INT_STATUS_EN_DCESEN | \
                                       BM_USDHC_INT_STATUS_EN_DEBESEN)
                                       
//#define ESDHC_PRESENT_STATE_CIHB      ((cyg_uint32 )0x00000001)
//#define ESDHC_PRESENT_STATE_CDIHB     ((cyg_uint32 )0x00000002)

#define ESDHC_IDENT_DVS      0x8
#define ESDHC_IDENT_SDCLKFS  0x20
#define ESDHC_OPERT_DVS      0x3
#define ESDHC_OPERT_SDCLKFS  0x1
#define ESDHC_HS_DVS         0x1
#define ESDHC_HS_SDCLKFS     0x1

#define ESDHC_ONE_BIT_SUPPORT         0x0000000
#define ESDHC_ADMA_BD_BLOCK_NUM       2000

#define ESDHC_STATUS_CHK_TIMEOUT      1000  /*  1ms */

#define ESDHC_CIHB_CHK_COUNT          10    /* 10ms */
#define ESDHC_CDIHB_CHK_COUNT         100   /* 0.1s */
#define ESDHC_OPER_TIMEOUT_COUNT      10000    /* 10s */
#define ESDHC_DMA_TIMEOUT_COUNT       10000 /* 10s  */

//#define ESDHC_STATUS_END_CMD_RESP_MSK         ((cyg_uint32 )0x00000001)
//#define ESDHC_STATUS_TRANSFER_COMPLETE_MSK    ((cyg_uint32 )0x00000002)
//#define ESDHC_MMC_BOOT_REG_BOOT_EN            ((cyg_uint32 )0x00000040)
//#define ESDHC_STATUS_TIME_OUT_RESP_MSK        ((cyg_uint32 )0x00010000)
//#define ESDHC_STATUS_RESP_CRC_ERR_MSK         ((cyg_uint32 )0x00020000)
//#define ESDHC_STATUS_RESP_CMD_END_BIT_ERR_MSK ((cyg_uint32 )0x00040000)
//#define ESDHC_STATUS_RESP_CMD_INDEX_ERR_MSK   ((cyg_uint32 )0x00080000)
//#define ESDHC_STATUS_TIME_OUT_READ_MSK        ((cyg_uint32 )0x00100000)
//#define ESDHC_STATUS_READ_CRC_ERR_MSK         ((cyg_uint32 )0x00200000)
//#define ESDHC_STATUS_END_CMD_RESP_TIME_MSK    ((cyg_uint32 )0x100F0001)
#define ESDHC_STATUS_END_CMD_RESP_TIME_MSK      (BM_USDHC_INT_STATUS_CC | \
                                                 BM_USDHC_INT_STATUS_CTOE | \
                                                 BM_USDHC_INT_STATUS_CCE | \
                                                 BM_USDHC_INT_STATUS_CEBE | \
                                                 BM_USDHC_INT_STATUS_CIE | \
                                                 BM_USDHC_INT_STATUS_DMAE)

//#define ESDHC_STATUS_END_CMD_RESP_TC_TIME_MSK ((cyg_uint32 )0x100F0002)
#define ESDHC_STATUS_END_CMD_RESP_TC_TIME_MSK   (BM_USDHC_INT_STATUS_TC | \
                                                 BM_USDHC_INT_STATUS_CTOE | \
                                                 BM_USDHC_INT_STATUS_CCE | \
                                                 BM_USDHC_INT_STATUS_CEBE | \
                                                 BM_USDHC_INT_STATUS_CIE | \
                                                 BM_USDHC_INT_STATUS_DMAE)

//#define ESDHC_STATUS_END_DATA_RSP_TC_MASK     ((cyg_uint32 )0x00700002)
#define ESDHC_STATUS_END_DATA_RSP_TC_MASK       (BM_USDHC_INT_STATUS_TC | \
                                                 BM_USDHC_INT_STATUS_DTOE | \
                                                 BM_USDHC_INT_STATUS_DCE | \
                                                 BM_USDHC_INT_STATUS_DEBE)

//#define ESDHC_PRTCTL_DMAS_MASK        ((cyg_uint32 )0x00000300)
//#define ESDHC_PRTCTL_ADMA2_VAL        ((cyg_uint32 )0x00000200)
#define ESDHC_PRTCTL_ADMA2_VAL        ((cyg_uint32 )0x00000002)

//#define ESDHC_MIXER_CTRL_CMD_MASK     ((cyg_uint32 )0xFFFFFFC0)
#define ESDHC_MIXER_CTRL_CMD_MASK       (BM_USDHC_MIX_CTRL_NIBBLE_POS | \
                                         BM_USDHC_MIX_CTRL_AC23EN | \
                                         BM_USDHC_MIX_CTRL_EXE_TUNE | \
                                         BM_USDHC_MIX_CTRL_SMP_CLK_SEL | \
                                         BM_USDHC_MIX_CTRL_AUTO_TUNE_EN | \
                                         BM_USDHC_MIX_CTRL_FBCLK_SEL)
                                         

#define ESDHC_ADMA_BD_ACT             ((cyg_uint8)0x20)
#define ESDHC_ADMA_BD_END             ((cyg_uint8)0x02)
#define ESDHC_ADMA_BD_VALID           ((cyg_uint8)0x01)
#define ESDHC_ADMA_BD_MAX_LEN         ((unsigned short)(64*1024-512))

//#define ESDHC_INTSTAT_BRR             (0x00000020)
//#define ESDHC_INTSTAT_BWR             (0x00000010)
//#define ESDHC_PRSTAT_BREN             (0x00000800)
//#define ESDHC_PRSTAT_BWEN             (0x00000400)
#define ESDHC_FIFO_LENGTH             (0x80)
#define ESDHC_BLKATTR_WML_BLOCK       (0x80)
#define ESDHC_BLKATTR_WML_SWITCH      (0x10)
#define ESDHC_WML_WRITE_SHIFT         (16)

//#define ESDHC_DMAEN_SHIFT                       (0)
//#define ESDHC_BLOCK_COUNT_ENABLE_SHIFT          (1)
//#define ESDHC_AC12EN_SHIFT                      (2)
//#define ESDHC_DDREN_SHIFT                       (3)
//#define ESDHC_DATA_TRANSFER_SHIFT               (4)
//#define ESDHC_MULTI_SINGLE_BLOCK_SELECT_SHIFT   (5)

//#define ESDHC_RESPONSE_FORMAT_SHIFT             (16)
//#define ESDHC_CRC_CHECK_SHIFT                   (19)
//#define ESDHC_CMD_INDEX_CHECK_SHIFT             (20)
//#define ESDHC_DATA_PRESENT_SHIFT                (21)
//#define ESDHC_CMD_INDEX_SHIFT                   (24)

//#define ESDHC_BLKATTR_NOB_SHIFT                 (16)
//#define ESDHC_BLKATTR_BLKLEN_MASK               (0xFFFF)

/*------------------------------------- Enumeration ----------------------------------------*/
typedef enum {
    OPERATING_FREQ,
    IDENTIFICATION_FREQ,
    HS_FREQ
} sdhc_freq_t;

/*-------------------------------------- Structure -----------------------------------------*/

/* ADMA Buffer Descriptor */
typedef struct {
    cyg_uint8 attribute;
    cyg_uint8 reserved;
    cyg_int16 length;
    cyg_int32 address;
} adma_bd_t;

/* uSDHC Register Map */
/*
typedef struct {
    volatile cyg_uint32  dma_system_address;
    volatile cyg_uint32  block_attributes;
    volatile cyg_uint32  command_argument;
    volatile cyg_uint32  command_transfer_type;
    volatile cyg_uint32  command_response0;
    volatile cyg_uint32  command_response1;
    volatile cyg_uint32  command_response2;
    volatile cyg_uint32  command_response3;
    volatile cyg_uint32  data_buffer_access;
    volatile cyg_uint32  present_state;
    volatile cyg_uint32  protocol_control;
    volatile cyg_uint32  system_control;
    volatile cyg_uint32  interrupt_status;
    volatile cyg_uint32  interrupt_status_enable;
    volatile cyg_uint32  interrupt_signal_enable;
    volatile cyg_uint32  autocmd12_status;
    volatile cyg_uint32  host_controller_capabilities;
    volatile cyg_uint32  watermark_level;
    volatile cyg_uint32  mixer_control;
    cyg_uint32  reserved1[1];
    volatile cyg_uint32  force_event;
    volatile cyg_uint32  adma_error_status_register;
    volatile cyg_uint32  adma_system_address;
    cyg_uint32  reserved2[1];
    volatile cyg_uint32  dll_control_register;
    volatile cyg_uint32  dll_status_register;
    volatile cyg_uint32  clk_tuning_control;
    cyg_uint32  reserved3[21];
    volatile cyg_uint32  vendor_specific_register;
    volatile cyg_uint32  mmc_boot_register;
    cyg_uint32  reserved4[13];
    volatile cyg_uint32  host_controller_version;
} host_register, *host_register_ptr;
*/
/*---------------------------------------- Function Define ----------------------------------------*/
/*!
 * @brief uSDHC Controller initialization - enble clock and iomux configuration 
 *
 * @param instance     Instance number of the uSDHC module.
 */
extern void usdhc_host_init(cyg_uint32 instance);

/*!
 * @brief uSDHC Controller active initial 80 clocks
 *
 * @param instance     Instance number of the uSDHC module.
 */
extern void usdhc_host_init_active(cyg_uint32 instance);

/*!
 * @brief uSDHC Controller sends command
 *
 * @param instance     Instance number of the uSDHC module.
 * @param cmd          the command to be sent
 * 
 * @return             0 if successful; 1 otherwise
 */
extern cyg_int32 usdhc_host_send_cmd(cyg_uint32 instance, command_t * cmd);

/*!
 * @brief uSDHC Controller sets bus width
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param bus_width    Bus_width
 * 
 */
extern void usdhc_host_set_bus_width(cyg_uint32 instance, cyg_int32 bus_width);

/*!
 * @brief Reset uSDHC Controller
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param bus_width    Bus_width
 * @param endian_mode  Endain mode
 * 
 */
extern void usdhc_host_reset(cyg_uint32 instance, cyg_int32 bus_width, cyg_int32 endian_mode);

/*!
 * @brief uSDHC Controller reads responses
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param response     Responses from card
 */
extern void usdhc_host_read_response(cyg_uint32 instance, command_response_t * response);

/*!
 * @brief uSDHC Controller Clock configuration
 *
 * @param instance     Instance number of the uSDHC module.
 * @param frequency    The frequency to be set
 * 
 */
extern void usdhc_host_cfg_clock(cyg_uint32 instance, cyg_int32 frequency);

/*!
 * @brief uSDHC Controller configures block
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param blk_len      Block Length
 * @param nob          Block count for current transfer
 * @param wml          Read and write Watermark level 
 */
extern void usdhc_host_cfg_block(cyg_uint32 instance, cyg_int32 blk_len, cyg_int32 nob, cyg_int32 wml);

/*!
 * @brief uSDHC Controller Clears FIFO
 * 
 * @param instance     Instance number of the uSDHC module.
 */
extern void usdhc_host_clear_fifo(cyg_uint32 instance);

/*!
 * @brief uSDHC Controller Sets up for ADMA transfer
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param ptr          Pointer for destination
 * @param length       ADMA transfer length
 */
extern void usdhc_host_setup_adma(cyg_uint32 instance, cyg_int32 *ptr, cyg_int32 length);

/*!
 * @brief uSDHC Controller reads data
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param dst_ptr      Pointer for data destination
 * @param length       Data length to be reading
 * @param wml          Watermark for data reading
 * 
 * @return             0 if successful; 1 otherwise
 */
extern cyg_int32 usdhc_host_data_read(cyg_uint32 instance, cyg_int32 *dst_ptr, cyg_int32 length, cyg_int32 wml);

/*!
 * @brief uSDHC Controller writes data
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param src_ptr      Pointer of data source
 * @param length       Data length to be writen
 * @param wml          Watermark for data writing
 * 
 * @return             0 if successful; 1 otherwise
 */
extern cyg_int32 usdhc_host_data_write(cyg_uint32 instance, cyg_int32 *src_ptr, cyg_int32 length, cyg_int32 wml);

/*!
 * @brief uSDHC_1 isr handler
 *
 * @param    none
 */
//extern void usdhc1_isr(void);
extern cyg_uint32 usdhc1_isr(cyg_vector_t vector, cyg_addrword_t data);

/*!
 * @brief uSDHC_2 isr handler
 *
 * @param    none
 */
//extern void usdhc2_isr(void);
extern cyg_uint32 usdhc2_isr(cyg_vector_t vector, cyg_addrword_t data);

/*!
 * @brief uSDHC_3 isr handler
 *
 * @param    none
 */
//extern void usdhc3_isr(void);
extern cyg_uint32 usdhc3_isr(cyg_vector_t vector, cyg_addrword_t data);

/*!
 * @brief uSDHC_4 isr handler
 *
 * @param    none
 */
//extern void usdhc4_isr(void);
extern cyg_uint32 usdhc4_isr(cyg_vector_t vector, cyg_addrword_t data);

#endif

