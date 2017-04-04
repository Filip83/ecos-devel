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
#include <cyg/hal/hal_intr.h>

#include <cyg/io/usdhc.h>
#include <cyg/io/usdhc_ifc.h>
#include <cyg/io/usdhc_host.h>
#include <cyg/io/registers/regsusdhc.h>

extern void usdhc_gpio_config(cyg_uint32 instance);

/*---------------------------------------------- Static Fcuntion ------------------------------------------------*/
/*!
 * @brief Set data transfer width. 
 * Possible data transfer width is 1-bit, 4-bits or 8-bits
 *
 * @param instance     Instance number of the uSDHC module.
 * @param data_width   Data transfer width
 */
static void usdhc_set_data_transfer_width(cyg_uint32 instance, cyg_int32 data_width)
{
    HW_USDHC_PROT_CTRL(instance).U |= (data_width & BM_USDHC_PROT_CTRL_DTW);
}

/*!
 * @brief Set Endian Mode. 
 * The uSDHC supports all four modes in data transfer.
 * 00 --- Big Endian Mode
 * 01 --- Half Word Big Endian Mode
 * 10 --- Little Endian Mode
 * 11 --- Reserved
 *
 * @param instance     Instance number of the uSDHC module.
 * @param endian_mode  Endian Mode to be set
 */
static void usdhc_set_endianness(cyg_uint32 instance, cyg_int32 endian_mode)
{
    HW_USDHC_PROT_CTRL(instance).B.EMODE = endian_mode;
}

/*!
 * @brief Wait for command inhibit(CMD) and command inhibit(DAT) idle for 
 * issuing next SD/MMC command. 
 *
 * @param instance     Instance number of the uSDHC module.
 * @param data_present check command inhibit(DAT)
 * 
 * @return             0 if successful; 1 otherwise
 */
 static cyg_int32 usdhc_wait_cmd_data_lines(cyg_uint32 instance, cyg_int32 data_present)
 {
 	cyg_int32 count = ZERO;
 	/* Wait for release of CMD line */
 	while (HW_USDHC_PRES_STATE(instance).B.CIHB) {
 		if (count == ESDHC_CIHB_CHK_COUNT) {
            return FAIL;
 		}
 		count++;
 		hal_delay_us(ESDHC_STATUS_CHK_TIMEOUT);
 	}
 	
    /* If data present with command, wait for release of DATA lines */
    if (data_present == DATA_PRESENT) {
        count = ZERO;

        while (HW_USDHC_PRES_STATE(instance).B.CDIHB) {
            if (count == ESDHC_CDIHB_CHK_COUNT) {
                return FAIL;
            }

            count++;
            hal_delay_us(ESDHC_STATUS_CHK_TIMEOUT);
        }
    } 		
 	
 	return SUCCESS;
 }

/*!
 * @brief Command configuration
 *
 * @param instance     Instance number of the uSDHC module.
 * @param cmd          The command to be configured
 * 
 */
static void usdhc_cmd_cfg(cyg_uint32 instance, command_t * cmd)
{
    cyg_uint32 consist_status;

    /* Write Command Argument in Command Argument Register */
    HW_USDHC_CMD_ARG_WR(instance, cmd->arg);

    /* Clear the DMAS field */
    HW_USDHC_PROT_CTRL_CLR(instance, BM_USDHC_PROT_CTRL_DMASEL);

    /* If ADMA mode enabled and command with DMA, enable ADMA2 */
    if ((cmd->dma_enable == true) && (read_usdhc_adma_mode() == true)) {
    	BW_USDHC_PROT_CTRL_DMASEL(instance, ESDHC_PRTCTL_ADMA2_VAL);
    }

    /* Keep bit fields other than command setting intact */
    consist_status = HW_USDHC_MIX_CTRL(instance).U & ESDHC_MIXER_CTRL_CMD_MASK;

    HW_USDHC_MIX_CTRL(instance).U = (consist_status |
                                 ((cmd->dma_enable) << BP_USDHC_MIX_CTRL_DMAEN) |
                                 ((cmd->block_count_enable_check) << BP_USDHC_MIX_CTRL_BCEN) |
                                 ((cmd->acmd12_enable) << BP_USDHC_MIX_CTRL_AC12EN) | 
                                 ((cmd->ddren) << BP_USDHC_MIX_CTRL_DDR_EN) |
                                 ((cmd->data_transfer) << BP_USDHC_MIX_CTRL_DTDSEL) |
                                 ((cmd->multi_single_block) << BP_USDHC_MIX_CTRL_MSBSEL));

    HW_USDHC_CMD_XFR_TYP(instance).U = (((cmd->response_format) << BP_USDHC_CMD_XFR_TYP_RSPTYP) |
                                         ((cmd->crc_check) << BP_USDHC_CMD_XFR_TYP_CCCEN) |
                                         ((cmd->cmdindex_check) << BP_USDHC_CMD_XFR_TYP_CICEN) |
                                         ((cmd->data_present) << BP_USDHC_CMD_XFR_TYP_DPSEL) |
                                         ((cmd->command) << BP_USDHC_CMD_XFR_TYP_CMDINX));
}

/*!
 * @brief Wait for transfer complete and without error
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 */
static void usdhc_wait_end_cmd_resp_dma_intr(cyg_uint32 instance)
{
    cyg_int32 count = ZERO;

    while (!(HW_USDHC_INT_STATUS_RD(instance) & ESDHC_STATUS_END_CMD_RESP_TC_TIME_MSK)) {
        if (count == ESDHC_DMA_TIMEOUT_COUNT) {
            break;
        }

        count++;
        hal_delay_us(ESDHC_STATUS_CHK_TIMEOUT);
    }
} 

/*!
 * @brief Wait for command complete and without error
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 */
static void usdhc_wait_end_cmd_resp_intr(cyg_uint32 instance)
{
    cyg_int32 count = ZERO;

    while (!(HW_USDHC_INT_STATUS_RD(instance) & ESDHC_STATUS_END_CMD_RESP_TIME_MSK)) {
        if (count == ESDHC_OPER_TIMEOUT_COUNT) {
            break;
        }

        count++;
//        hal_delay_us(ESDHC_STATUS_CHK_TIMEOUT);
        hal_delay_us(100);
    }
} 

/*!
 * @brief uSDHC Controller Checks response
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
static cyg_int32 usdhc_check_response(cyg_uint32 instance)
{
    cyg_int32 status = FAIL;
 
    if (((HW_USDHC_INT_STATUS(instance).B.CC) ||
         (HW_USDHC_MMC_BOOT(instance).B.BOOT_EN)) &&
        !(HW_USDHC_INT_STATUS(instance).B.CTOE) &&
        !(HW_USDHC_INT_STATUS(instance).B.CCE) &&
        !(HW_USDHC_INT_STATUS(instance).B.CIE) &&
        !(HW_USDHC_INT_STATUS(instance).B.CEBE)) {
        status = SUCCESS;
    } else {

        /* Clear CIHB and CDIHB status */
        if ((HW_USDHC_PRES_STATE(instance).B.CIHB) ||
            (HW_USDHC_PRES_STATE(instance).B.CDIHB)) {
            HW_USDHC_SYS_CTRL(instance).B.RSTA = 1;
        }
    }

    return status;
} 

/*!
 * @brief uSDHC Controller Checks transfer
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
static cyg_int32 usdhc_check_transfer(cyg_uint32 instance)
{
    cyg_int32 status = FAIL;

    if ((HW_USDHC_INT_STATUS(instance).B.TC) &&
        !(HW_USDHC_INT_STATUS(instance).B.DTOE) &&
        !(HW_USDHC_INT_STATUS(instance).B.DCE)) {
        status = SUCCESS;
    }
    return status;
}

/*!
 * @brief  uSDHC handles interrupt
 *
 * @param idx     Index of uSDHC device.
 * 
 */
static void usdhc_handle_isr(cyg_int32 idx)
{   
    cyg_uint32 instance = REGS_USDHC_INSTANCE(usdhc_device[idx].reg_base);

    /* Set interrupt flag */
    if (HW_USDHC_INT_STATUS(instance).U & ESDHC_STATUS_END_DATA_RSP_TC_MASK) {
        usdhc_device[idx].status = INTR_TC;
    } else {
        usdhc_device[idx].status = INTR_ERROR;

        /* Clear CIHB or CDIHB */
        if ((HW_USDHC_PRES_STATE(instance).B.CIHB) ||
            (HW_USDHC_PRES_STATE(instance).B.CDIHB)) {
            HW_USDHC_SYS_CTRL(instance).B.RSTA = 1;
        }
    }

    /* Clear interrupt */
    HW_USDHC_INT_STATUS_WR(instance, ESDHC_CLEAR_INTERRUPT);

    /* Clear interrupt enable */
    HW_USDHC_INT_SIGNAL_EN_WR(instance, 0);
}

/*---------------------------------------------- Global Function ------------------------------------------------*/
/*!
 * @brief uSDHC_1 isr handler
 *
 * @vector    vector
 * @data
 */
cyg_uint32 usdhc1_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_drv_interrupt_mask(vector);
    cyg_drv_interrupt_acknowledge(vector);

    usdhc_handle_isr(USDHC_PORT1);

    cyg_drv_interrupt_unmask(vector);

    return CYG_ISR_HANDLED;
}

/*!
 * @brief uSDHC_2 isr handler
 *
 * @vector    vector
 * @data
 */
cyg_uint32 usdhc2_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_drv_interrupt_mask(vector);
    cyg_drv_interrupt_acknowledge(vector);

    usdhc_handle_isr(USDHC_PORT2);

    cyg_drv_interrupt_unmask(vector);

    return CYG_ISR_HANDLED;
}

/*!
 * @brief uSDHC_3 isr handler
 *
 * @vector    vector
 * @data
 */
cyg_uint32 usdhc3_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_drv_interrupt_mask(vector);
    cyg_drv_interrupt_acknowledge(vector);

    usdhc_handle_isr(USDHC_PORT3);

    cyg_drv_interrupt_unmask(vector);

    return CYG_ISR_HANDLED;
}

/*!
 * @brief uSDHC_4 isr handler
 *
 * @vector    vector
 * @data
 * */
cyg_uint32 usdhc4_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    cyg_drv_interrupt_mask(vector);
    cyg_drv_interrupt_acknowledge(vector);

    usdhc_handle_isr(USDHC_PORT4);

    cyg_drv_interrupt_unmask(vector);

    return CYG_ISR_HANDLED;
}

/*!
 * @brief uSDHC Controller initialization - enble clock and iomux configuration 
 *
 * @param instance     Instance number of the uSDHC module.
 */
void usdhc_host_init(cyg_uint32 instance)
{
    /* Enable Clock Gating */

    /* IOMUX Configuration */
    usdhc_iomux_config(instance);
    usdhc_gpio_config(instance);
}

/*!
 * @brief uSDHC Controller active initial 80 clocks
 *
 * @param instance     Instance number of the uSDHC module.
 */
void usdhc_host_init_active(cyg_uint32 instance)
{
    /* Send 80 clock ticks for card to power up */
    HW_USDHC_SYS_CTRL(instance).B.INITA = 1;

    /* Wait until INITA field cleared */
    while (BG_USDHC_SYS_CTRL_INITA(instance)) {
        ;
    }
}

/*!
 * @brief uSDHC Controller sends command
 *
 * @param instance     Instance number of the uSDHC module.
 * @param cmd          the command to be sent
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_host_send_cmd(cyg_uint32 instance, command_t * cmd)
{
    /* Clear Interrupt status register */
    HW_USDHC_INT_STATUS_WR(instance, ESDHC_CLEAR_INTERRUPT);

    /* Enable Interrupt */
    HW_USDHC_INT_STATUS_EN(instance).U |= ESDHC_INTERRUPT_ENABLE;

    /* Wait for CMD/DATA lines to be free */
    if (usdhc_wait_cmd_data_lines(instance, cmd->data_present) == FAIL) {
        return FAIL;
    }

    /* Clear interrupt status */
    HW_USDHC_INT_STATUS(instance).U |= ESDHC_STATUS_END_CMD_RESP_TIME_MSK;

    /* Enable interrupt when sending DMA commands */
    if ((read_usdhc_intr_mode() == true) && (cmd->dma_enable == true)) {
        cyg_int32 idx = usdhc_get_port(instance);

        /* Set interrupt flag to busy */
        usdhc_device[idx].status = INTR_BUSY;

        /* Enable uSDHC interrupt */
        HW_USDHC_INT_SIGNAL_EN_WR(instance, ESDHC_STATUS_END_DATA_RSP_TC_MASK);
    }

    /* Configure Command */
    usdhc_cmd_cfg(instance, cmd);

    /* If DMA Enabled */
    if (cmd->dma_enable == true) {
        /* Return in interrupt mode */
        if (read_usdhc_intr_mode() == true) {     
            return SUCCESS;
        }

        usdhc_wait_end_cmd_resp_dma_intr(instance);
    }

    /* Other command */
    else {
        usdhc_wait_end_cmd_resp_intr(instance);
    }

    /* Mask all interrupts */
    HW_USDHC_INT_SIGNAL_EN_WR(instance, ZERO);

    /* Check if an error occured */
    return usdhc_check_response(instance);
}

/*!
 * @brief uSDHC Controller Clock configuration
 *
 * @param instance     Instance number of the uSDHC module.
 * @param frequency    The frequency to be set
 * 
 */
void usdhc_host_cfg_clock(cyg_uint32 instance, cyg_int32 frequency)
{
	    /* Clear SDCLKEN bit, this bit is reserved in Rev D*/
    //esdhc_base->system_control &= ~ESDHC_SYSCTL_SDCLKEN_MASK;
    
    /* Wait until clock stable */
    while (!(HW_USDHC_PRES_STATE(instance).B.SDSTB)) {
        ;
    }

    /* Clear DTOCV, SDCLKFS, DVFS bits */
    HW_USDHC_SYS_CTRL(instance).U &= ~ESDHC_SYSCTL_FREQ_MASK;

    /* Wait until clock stable */
    while (!(HW_USDHC_PRES_STATE(instance).B.SDSTB)) {
        ;
    }

    /* Set frequency dividers */
    if (frequency == IDENTIFICATION_FREQ) {
        HW_USDHC_SYS_CTRL(instance).B.DVS = ESDHC_IDENT_DVS;
        BW_USDHC_SYS_CTRL_SDCLKFS(instance, ESDHC_IDENT_SDCLKFS);
    } else if (frequency == OPERATING_FREQ) {
        HW_USDHC_SYS_CTRL(instance).B.DVS = ESDHC_OPERT_DVS;
        BW_USDHC_SYS_CTRL_SDCLKFS(instance, ESDHC_OPERT_SDCLKFS);
    } else {
        HW_USDHC_SYS_CTRL(instance).B.DVS = ESDHC_HS_DVS;
        BW_USDHC_SYS_CTRL_SDCLKFS(instance, ESDHC_HS_SDCLKFS);
    }

    /* Wait until clock stable */
    while (!(HW_USDHC_PRES_STATE(instance).B.SDSTB)) {
        ;
    }

    /* Set DTOCV bit */
    BW_USDHC_SYS_CTRL_DTOCV(instance, ESDHC_SYSCTL_DTOCV_VAL);
}

/*!
 * @brief uSDHC Controller sets bus width
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param bus_width    Bus_width
 * 
 */
void usdhc_host_set_bus_width(cyg_uint32 instance, cyg_int32 bus_width)
{
    cyg_int32 width = bus_width >> ONE;

    usdhc_set_data_transfer_width(instance, width);
}

/*!
 * @brief Reset uSDHC Controller
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param bus_width    Bus_width
 * @param endian_mode  Endain mode
 * 
 */
void usdhc_host_reset(cyg_uint32 instance, cyg_int32 bus_width, cyg_int32 endian_mode)
{
    /* Reset the eSDHC by writing 1 to RSTA bit of SYSCTRL Register */
    HW_USDHC_SYS_CTRL(instance).B.RSTA = 1;

    /* Wait until RSTA field cleared */
    while (HW_USDHC_SYS_CTRL(instance).B.RSTA) {
        ;
    }

    /* Set default bus width to eSDHC */
    usdhc_host_set_bus_width(instance, bus_width);

    /* Set Endianness of eSDHC */
    usdhc_set_endianness(instance, endian_mode);
}

/*!
 * @brief uSDHC Controller reads responses
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param response     Responses from card
 */
void usdhc_host_read_response(cyg_uint32 instance, command_response_t * response)
{
    /* Read response from registers */
    response->cmd_rsp0 = HW_USDHC_CMD_RSP0_RD(instance);
    response->cmd_rsp1 = HW_USDHC_CMD_RSP1_RD(instance);
    response->cmd_rsp2 = HW_USDHC_CMD_RSP2_RD(instance);
    response->cmd_rsp3 = HW_USDHC_CMD_RSP3_RD(instance);
}

/*!
 * @brief uSDHC Controller configures block
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param blk_len      Block Length
 * @param nob          Block count for current transfer
 * @param wml          Read and write Watermark level 
 */
void usdhc_host_cfg_block(cyg_uint32 instance, cyg_int32 blk_len, cyg_int32 nob, cyg_int32 wml)
{
    /* Number of blocks and block length */
    BW_USDHC_BLK_ATT_BLKSIZE(instance, blk_len);
    BW_USDHC_BLK_ATT_BLKCNT(instance, nob);

    /* Watermark level - for DMA transfer */
    HW_USDHC_WTMK_LVL_WR(instance, wml);
}

/*!
 * @brief uSDHC Controller Clears FIFO
 * 
 * @param instance     Instance number of the uSDHC module.
 */
void usdhc_host_clear_fifo(cyg_uint32 instance)
{
    cyg_uint32 val, idx;

    /* If data present in Rx FIFO */
    if (HW_USDHC_INT_STATUS(instance).B.BRR) {
        /* Read from FIFO until empty */
        for (idx = 0; idx < ESDHC_FIFO_LENGTH; idx++) {
            val = HW_USDHC_DATA_BUFF_ACC_PORT_RD(instance);
        }
    }

    /* Remove compiler warning */
    val = val * 2;

    /* Maybe not necessary */
    HW_USDHC_INT_STATUS(instance).B.BRR = 1;
}

/*!
 * @brief uSDHC Controller Sets up for ADMA transfer
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param ptr          Pointer for destination
 * @param length       ADMA transfer length
 */
void usdhc_host_setup_adma(cyg_uint32 instance, cyg_int32 *ptr, cyg_int32 length)
{
    cyg_uint32 dst_ptr = (unsigned int)ptr;
    cyg_uint32 port, bd_id = 0;
    adma_bd_t *bd_addr;

    /* Get uSDHC port according to base address */
    port = usdhc_get_port(instance);

    /* Get BD pointer */
    bd_addr = (adma_bd_t *) usdhc_device[port].adma_ptr;

    /* Setup BD chain */
    while (length > 0) {
        bd_addr[bd_id].address = dst_ptr;
        if (length > ESDHC_ADMA_BD_MAX_LEN) {
            bd_addr[bd_id].length = ESDHC_ADMA_BD_MAX_LEN;
            bd_addr[bd_id].attribute = ESDHC_ADMA_BD_ACT | ESDHC_ADMA_BD_VALID;
            length -= ESDHC_ADMA_BD_MAX_LEN;
            dst_ptr += ESDHC_ADMA_BD_MAX_LEN;
        } else {
            bd_addr[bd_id].length = length;
            bd_addr[bd_id].attribute = ESDHC_ADMA_BD_ACT | ESDHC_ADMA_BD_VALID | ESDHC_ADMA_BD_END;
            length = 0;
        }

        bd_id++;
    }

    /* Setup BD pointer */
    HW_USDHC_ADMA_SYS_ADDR(instance).U = (unsigned int)bd_addr;

    /* Clear interrupt status */
    HW_USDHC_INT_STATUS_WR(instance, ESDHC_CLEAR_INTERRUPT);
}

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
cyg_int32 usdhc_host_data_read(cyg_uint32 instance, cyg_int32 *dst_ptr, cyg_int32 length, cyg_int32 wml)
{
    cyg_int32 idx, itr, loop;

    /* Enable Interrupt */
    HW_USDHC_INT_STATUS_EN(instance).U |= ESDHC_INTERRUPT_ENABLE;

    /* Read data to dst_ptr */
    loop = length / (4 * wml);
    for (idx = 0; idx < loop; idx++) {
        /* Wait until buffer ready */
        while (!(HW_USDHC_PRES_STATE(instance).B.BREN)) {
            ;
        }

        /* Read from FIFO watermark words */
        for (itr = 0; itr < wml; itr++) {
            *dst_ptr = HW_USDHC_DATA_BUFF_ACC_PORT_RD(instance);
            dst_ptr++;
        }
    }

    /* Read left data that not WML aligned */
    loop = (length % (4 * wml)) / 4;
    if (loop != 0) {
        /* Wait until buffer ready */
        while (!(HW_USDHC_PRES_STATE(instance).B.BREN)) {
            ;
        }

        /* Read the left to destination buffer */
        for (itr = 0; itr < loop; itr++) {
            *dst_ptr = HW_USDHC_DATA_BUFF_ACC_PORT_RD(instance);
            dst_ptr++;
        }

        /* Clear FIFO */
        for (; itr < wml; itr++) {
            idx = HW_USDHC_DATA_BUFF_ACC_PORT_RD(instance);
        }
    }

    /* Wait until transfer complete */
    while (!(HW_USDHC_INT_STATUS_RD(instance) & ESDHC_STATUS_END_DATA_RSP_TC_MASK)) ;

    /* Check if error happened */
    return usdhc_check_transfer(instance);
}

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
cyg_int32 usdhc_host_data_write(cyg_uint32 instance, cyg_int32 *src_ptr, cyg_int32 length, cyg_int32 wml)
{
    cyg_int32 idx, itr, loop;

    /* Enable Interrupt */
    HW_USDHC_INT_STATUS_EN(instance).U |= ESDHC_INTERRUPT_ENABLE;

    /* Write data from src_ptr */
    loop = length / (4 * wml);
    for (idx = 0; idx < loop; idx++) {
        /* Wait until write available */
        while (!(HW_USDHC_PRES_STATE(instance).B.BWEN)) {
            ;
        }

        /* Write to FIFO watermark words */
        for (itr = 0; itr < wml; itr++) {
            HW_USDHC_DATA_BUFF_ACC_PORT(instance).U = *src_ptr;
            src_ptr++;
        }
    }

    /* Write left data that not watermark aligned */
    loop = (length % (4 * wml)) / 4;
    if (loop != 0) {
        /* Wait until buffer ready */
        while (!(HW_USDHC_PRES_STATE(instance).B.BWEN)) {
            ;
        }

        /* Write the left from source buffer */
        for (itr = 0; itr < loop; itr++) {
            HW_USDHC_DATA_BUFF_ACC_PORT(instance).U = *src_ptr;
            src_ptr++;
        }

        /* Fill FIFO */
        for (; itr < wml; itr++) {
            HW_USDHC_DATA_BUFF_ACC_PORT(instance).U = idx;
        }
    }

    /* Wait for transfer complete */
    while (!(HW_USDHC_INT_STATUS_RD(instance) & ESDHC_STATUS_END_DATA_RSP_TC_MASK)) ;

    /* Check if error happened */
    return usdhc_check_transfer(instance);
}


