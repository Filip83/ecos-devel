/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
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
#include <cyg/io/devtab.h>

#include <cyg/io/usdhc_ifc.h>
#include <cyg/io/usdhc.h>
#include <cyg/io/usdhc_host.h>
#include <cyg/io/registers/regsusdhc.h>
#include <cyg/io/buffers.h>

/* Global Variables */

usdhc_inst_t usdhc_device[USDHC_NUMBER_PORTS] = {
    {
     REGS_USDHC1_BASE,          //register base
     USDHC_ADMA_BUFFER1,        //ADMA base
     usdhc1_isr,                //ISR
     0,                         //RCA
     0,                         //addressing mode
     CYGNUM_HAL_INTERRUPT_USDHC1,            //interrupt ID
     1,                         //status
     }
    ,

    {
     REGS_USDHC2_BASE,          //register base
     USDHC_ADMA_BUFFER2,        //ADMA base
     usdhc2_isr,                //ISR
     0,                         //RCA
     0,                         //addressing mode
     CYGNUM_HAL_INTERRUPT_USDHC2,            //interrupt ID
     1,                         //status
     }
    ,

    {
     REGS_USDHC3_BASE,          //register base
     USDHC_ADMA_BUFFER3,        //ADMA base
     usdhc3_isr,                //ISR
     0,                         //RCA
     0,                         //addressing mode
     CYGNUM_HAL_INTERRUPT_USDHC3,            //interrupt ID
     1,                         //status
     }
    ,

    {
     REGS_USDHC4_BASE,          //register base
     USDHC_ADMA_BUFFER4,        //ADMA base
     usdhc4_isr,                //ISR
     0,                         //RCA
     0,                         //addressing mode
     CYGNUM_HAL_INTERRUPT_USDHC4,            //interrupt ID
     1,                         //status
     }
    ,
};

/* Whether to enable Card Detect Test in card_init() */
cyg_int32 card_detect_test_en = 1;
extern bool usdhc_card_detected(cyg_uint32 instance);

/* Whether to enable Card Protect Test in card_init() */
cyg_int32 write_protect_test_en = 1;
extern bool usdhc_write_protected(cyg_uint32 instance);

/* Whether to enable ADMA */
cyg_int32 SDHC_ADMA_mode = false;

/* Whether to enable Interrupt */
cyg_int32 SDHC_INTR_mode = false;

/********************************************* Static Function ******************************************/
/*!
 * @brief Softreset card and then send CMD0 to card
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_software_reset(cyg_uint32 instance)
{
    command_t cmd;
    cyg_int32 response = FAIL;

    /* Configure CMD0 */
    usdhc_cmd_config(&cmd, CMD0, NO_ARG, READ, RESPONSE_NONE, DATA_PRESENT_NONE, false, false);

    /* Issue CMD0 to Card */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        response = SUCCESS;
    }

    return response;
}

/*!
 * @brief Enable interrupt for the card
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */

cyg_int32 usdhc_init_interrupt(cyg_uint32 instance)
{
    cyg_int32 idx = usdhc_get_port(instance);

    if (idx == USDHC_NUMBER_PORTS) {
        return FAIL;
    }

    cyg_handle_t        handle;        /* returned handle                   */
    cyg_interrupt       intr;           /* put interrupt here                */

    cyg_drv_interrupt_create(usdhc_device[idx].intr_id,
                             8,
                             // Data item passed to interrupt handler
                             0,
                             usdhc_device[idx].isr,
                             NULL,
                             &handle,
                             &intr);

    // This driver only uses CPU 0 for interrupts.
#ifdef CYGPKG_KERNEL_SMP_SUPPORT
    cyg_drv_interrupt_set_cpu(usdhc_device[idx].intr_id, 0);
#endif
    cyg_drv_interrupt_attach(handle);
    cyg_drv_interrupt_unmask(usdhc_device[idx].intr_id);

    return SUCCESS;
}

/********************************************* Global Function ******************************************/
/*!
 * @brief Set Card access mode
 *
 * @param mode     Set card access mode
 * 
 * @return           
 */
extern void set_card_access_mode(cyg_uint32 sdma, cyg_uint32 intr)
{
	/* Whether to enable ADMA */
    SDHC_ADMA_mode = sdma;

	/* Whether to enable Interrupt */
    SDHC_INTR_mode = intr; 	
}
 
cyg_uint32 read_usdhc_adma_mode(void)
{
	return SDHC_ADMA_mode;
}
cyg_uint32 read_usdhc_intr_mode(void) 
{
	return SDHC_INTR_mode;
}

cyg_int32 usdhc_get_csd(cyg_uint32 instance, cyg_uint8 *csd)
{
    command_t cmd;
    command_response_t response;
    cyg_int32 card_address, port, status = FAIL;

    /* Get port from base address */
    port = usdhc_get_port(instance);

    /* Get RCA */
    card_address = usdhc_device[port].rca << RCA_SHIFT;

    /* Configure CMD9 for MMC/SD card */
    usdhc_cmd_config(&cmd, CMD9, card_address, READ, RESPONSE_136, DATA_PRESENT_NONE, true, false);

    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        response.format = RESPONSE_136;
        usdhc_host_read_response(instance, &response);

        csd[15] = 0;
        csd[14] = response.cmd_rsp0 & 0x000000FF;
        csd[13] = (response.cmd_rsp0 >> 8) & 0x000000FF;
        csd[12] = (response.cmd_rsp0 >> 16) & 0x000000FF;
        csd[11] = (response.cmd_rsp0 << 24) & 0x000000FF;
        csd[10] = response.cmd_rsp1 & 0x000000FF;
        csd[9] = (response.cmd_rsp1 >> 8) & 0x000000FF;
        csd[8] = (response.cmd_rsp1 >> 16) & 0x000000FF;
        csd[7] = (response.cmd_rsp1 >> 24) & 0x000000FF;
        csd[6] = response.cmd_rsp2 & 0x000000FF;
        csd[5] = (response.cmd_rsp2 >> 8) & 0x000000FF;
        csd[4] = (response.cmd_rsp2 >> 16) & 0x000000FF;
        csd[3] = (response.cmd_rsp2 >> 24) & 0x000000FF;
        csd[2] = response.cmd_rsp3 & 0x000000FF;
        csd[1] = (response.cmd_rsp3 >> 8) & 0x000000FF;
        csd[0] = (response.cmd_rsp3 >> 16) & 0x000000FF;

        status = SUCCESS;
    }

    return status;
}

/*!
 * @brief Build up command
 *
 * @param cmd      IPointer of command to be build up.
 * @param index    Command index.
 * @param argument Argument of the command.
 * @param transfer Command transfer type - Read, Write or SD command.
 * @param format   Command response format
 * @param data     0 - no data present, 1 - data present.
 * @param src      0 - no CRC check, 1 - do CRC check
 * @param cmdindex 0 - no check on command index, 1 - Check comamnd index
 */
void usdhc_cmd_config(command_t * cmd, cyg_int32 index, cyg_int32 argument, xfer_type_t transfer,
                     response_format_t format, data_present_select data,
                     crc_check_enable crc, cmdindex_check_enable cmdindex)
{
    cmd->command = index;
    cmd->arg = argument;
    cmd->data_transfer = transfer;
    cmd->response_format = format;
    cmd->data_present = data;
    cmd->crc_check = crc;
    cmd->cmdindex_check = cmdindex;
    cmd->dma_enable = false;
    cmd->block_count_enable_check = false;
    cmd->multi_single_block = SINGLE;
    cmd->acmd12_enable = false;
    cmd->ddren = false;

    /* Multi Block R/W Setting */
    if ((CMD18 == index) || (CMD25 == index)) {
        if (SDHC_ADMA_mode == true) {
            cmd->dma_enable = true;
        }

        cmd->block_count_enable_check = true;
        cmd->multi_single_block = MULTIPLE;
        cmd->acmd12_enable = true;
    }
}

/*!
 * @brief Get Card CID
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_get_cid(cyg_uint32 instance, cyg_uint8 *cid)
{
    command_t cmd;
    cyg_int32 status = FAIL;
    command_response_t response;

    /* Configure CMD2 */
    usdhc_cmd_config(&cmd, CMD2, NO_ARG, READ, RESPONSE_136, DATA_PRESENT_NONE, true, false);

    /* Send CMD2 */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        response.format = RESPONSE_136;
        usdhc_host_read_response(instance, &response);

        cid[15] = 0;
        cid[14] = response.cmd_rsp0 & 0x000000FF;
        cid[13] = (response.cmd_rsp0 >> 8) & 0x000000FF;
        cid[12] = (response.cmd_rsp0 >> 16) & 0x000000FF;
        cid[11] = (response.cmd_rsp0 << 24) & 0x000000FF;
        cid[10] = response.cmd_rsp1 & 0x000000FF;
        cid[9] = (response.cmd_rsp1 >> 8) & 0x000000FF;
        cid[8] = (response.cmd_rsp1 >> 16) & 0x000000FF;
        cid[7] = (response.cmd_rsp1 >> 24) & 0x000000FF;
        cid[6] = response.cmd_rsp2 & 0x000000FF;
        cid[5] = (response.cmd_rsp2 >> 8) & 0x000000FF;
        cid[4] = (response.cmd_rsp2 >> 16) & 0x000000FF;
        cid[3] = (response.cmd_rsp2 >> 24) & 0x000000FF;
        cid[2] = response.cmd_rsp3 & 0x000000FF;
        cid[1] = (response.cmd_rsp3 >> 8) & 0x000000FF;
        cid[0] = (response.cmd_rsp3 >> 16) & 0x000000FF;

        status = SUCCESS;
    }

    return status;
}

/*!
 * @brief Toggle the card between the standby and transfer states
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_enter_trans(cyg_uint32 instance)
{
    command_t cmd;
    cyg_int32 card_address, port, status = FAIL;

    /* Get port from base address */
    port = usdhc_get_port(instance);

    /* Get RCA */
    card_address = usdhc_device[port].rca << RCA_SHIFT;

    /* Configure CMD7 */
    usdhc_cmd_config(&cmd, CMD7, card_address, READ, RESPONSE_48_CHECK_BUSY,
                    DATA_PRESENT_NONE, true, true);

    /* Send CMD7 */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        /* Check if the card in TRAN state */
        if (usdhc_trans_status(instance) == SUCCESS) {
            status = SUCCESS;
        }
    }

    return status;
}

/*!
 * @brief Addressed card send its status register
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_trans_status(cyg_uint32 instance)
{
    command_t cmd;
    command_response_t response;
    cyg_int32 card_state, card_address, port, status = FAIL;

    /* Get Port */
    port = usdhc_get_port(instance);

    /* Get RCA */
    card_address = usdhc_device[port].rca << RCA_SHIFT;

    /* Configure CMD13 */
    usdhc_cmd_config(&cmd, CMD13, card_address, READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);

    /* Send CMD13 */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        /* Get Response */
        response.format = RESPONSE_48;
        usdhc_host_read_response(instance, &response);

        /* Read card state from response */
        card_state = CURR_CARD_STATE(response.cmd_rsp0);
        if (card_state == TRAN) {
            status = SUCCESS;
        }
    }

    return status;
}

/*!
 * @brief Get the card port 
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             The index of port
 */
cyg_int32 usdhc_get_port(cyg_uint32 instance)
{
    cyg_int32 idx;
    cyg_int32 base_address = REGS_USDHC_BASE(instance);
    
    for (idx = 0; idx < USDHC_NUMBER_PORTS; idx++) {
        if (usdhc_device[idx].reg_base == base_address) {
            break;
        }
    }

    return idx;
}

/*!
 * @brief Set block length (in bytes) for read and write
 *
 * @param instance     Instance number of the uSDHC module.
 * @param len          Block length to be set
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_set_blklen(cyg_uint32 instance, cyg_int32 len)
{
    command_t cmd;
    cyg_int32 status = FAIL;

    /* Configure CMD16 */
    usdhc_cmd_config(&cmd, CMD16, len, READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);

    /* Send CMD16 */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        status = SUCCESS;
    } 
    
    return status;
}

void usdhc_buffer_flush(void *buffer, cyg_int32 length)
{
// FIXME
//    if(!arm_dcache_state_query())
//        return;

//        arm_dcache_flush_mlines(buffer, length);
//        arm_dcache_invalidate_mlines(buffer, length);
}

/*!
 * @brief Read data from card
 *
 * @param instance     Instance number of the uSDHC module.
 * @param dst_ptr      Data destination pointer
 * @param length       Data length to be read
 * @param offset       Data reading offset
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_data_read(cyg_uint32 instance, cyg_uint8 *dst_ptr, cyg_int32 length, cyg_uint32 offset)
{
    cyg_int32 port, sector;
    command_t cmd;

    /* Get uSDHC port according to instance */
    port = usdhc_get_port(instance);
    if (port == USDHC_NUMBER_PORTS) {
        return FAIL;
    }

    /* Get sector number */
    if (SDHC_ADMA_mode == true) {
        /* For DMA mode, length should be sector aligned */
        if ((length % BLK_LEN) != 0) {
            length = length + BLK_LEN - (length % BLK_LEN);
        }

        sector = length / BLK_LEN;
    } else {
        /* For PIO mode, not neccesary */
        sector = length / BLK_LEN;

        if ((length % BLK_LEN) != 0) {
            sector++;
        }
    }

    /* Offset should be sectors */
    if (usdhc_device[port].addr_mode == SECT_MODE) {
        offset = offset / BLK_LEN;
    }

    /* Set block length to card */
    if (usdhc_set_blklen(instance, BLK_LEN) == FAIL) {
        return FAIL;
    }

    /* Clear Rx FIFO */
    usdhc_host_clear_fifo(instance);

    /* Configure block length/number and watermark */
    usdhc_host_cfg_block(instance, BLK_LEN, sector, ESDHC_BLKATTR_WML_BLOCK);

    /* If DMA mode enabled, configure BD chain */
    if (SDHC_ADMA_mode == true) {
        usdhc_host_setup_adma(instance, dst_ptr, length);
        usdhc_buffer_flush(dst_ptr, length);
    }

    /* Use CMD18 for multi-block read */
    usdhc_cmd_config(&cmd, CMD18, offset, READ, RESPONSE_48, DATA_PRESENT, true, true);

    /* Send CMD18 */
    if (usdhc_host_send_cmd(instance, &cmd) == FAIL) {
        return FAIL;
    } else {
        /* In polling IO mode, manually read data from Rx FIFO */
        if (SDHC_ADMA_mode == false) {

            if (usdhc_host_data_read(instance, dst_ptr, length, ESDHC_BLKATTR_WML_BLOCK) == FAIL) {
                return FAIL;
            }
        }
    }

    return SUCCESS;
}

/*!
 * @brief Write data to card
 *
 * @param instance     Instance number of the uSDHC module.
 * @param src_ptr      Data source pointer
 * @param length       Data length to be writen
 * @param offset       Data writing offset
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_data_write(cyg_uint32 instance, cyg_uint8 *src_ptr, cyg_int32 length, cyg_int32 offset)
{
    cyg_int32 port, sector;
    command_t cmd;

    /* Get uSDHC port according to base address */
    port = usdhc_get_port(instance);
    if (port == USDHC_NUMBER_PORTS) {
        return FAIL;
    }

    /* Get sector number */
    if (SDHC_ADMA_mode == true) {
        /* For DMA mode, length should be sector aligned */
        if ((length % BLK_LEN) != 0) {
            length = length + BLK_LEN - (length % BLK_LEN);
        }

        sector = length / BLK_LEN;
    } else {
        /* For PIO mode, not neccesary */
        sector = length / BLK_LEN;

        if ((length % BLK_LEN) != 0) {
            sector++;
        }
    }

    /* Offset should be sectors */
    if (usdhc_device[port].addr_mode == SECT_MODE) {
        offset = offset / BLK_LEN;
    }

    /* Set block length to card */
    if (usdhc_set_blklen(instance, BLK_LEN) == FAIL) {
        return FAIL;
    }

    /* Configure block length/number and watermark */
    usdhc_host_cfg_block(instance, BLK_LEN, sector, ESDHC_BLKATTR_WML_BLOCK << ESDHC_WML_WRITE_SHIFT);

    /* If DMA mode enabled, configure BD chain */
    if (SDHC_ADMA_mode == true) {
        usdhc_host_setup_adma(instance, src_ptr, length);
        usdhc_buffer_flush(src_ptr, length);
    }

    /* Use CMD25 for multi-block write */
    usdhc_cmd_config(&cmd, CMD25, offset, WRITE, RESPONSE_48, DATA_PRESENT, true, true);

    /* Send CMD25 */
    if (usdhc_host_send_cmd(instance, &cmd) == FAIL) {
        return FAIL;
    } else {
        if (SDHC_ADMA_mode == false) {

            if (usdhc_host_data_write(instance, src_ptr, length, ESDHC_BLKATTR_WML_BLOCK) == FAIL) {
                return FAIL;
            }
        }

    }

    return SUCCESS;
}

/*!
 * @brief Get card status
 *
 * @param instance     Instance number of the uSDHC module.
 * @param result       Card status
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_xfer_result(cyg_uint32 instance, cyg_int32 *result)
{
    cyg_int32 idx = usdhc_get_port(instance);

    if (idx == USDHC_NUMBER_PORTS) {
        return FAIL;
    }

    *result = usdhc_device[idx].status;

    return SUCCESS;
}

/*!
 * @brief Wait for the transfer complete. It covers the interrupt mode, DMA mode and PIO mode
 *
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 usdhc_wait_xfer_done(cyg_uint32 instance)
{
    cyg_int32 usdhc_status = 0;
    cyg_int32 timeout = 0x40000000;
    if(SDHC_INTR_mode)
    {
        while (timeout--) {
            usdhc_xfer_result(instance, &usdhc_status);
            if (usdhc_status == 1)
                return SUCCESS;
        }
     } else {
            /*do nothing, since in PIO/DMA mode, will check the flag in host send data*/
            return SUCCESS;
     }

     return FAIL;
}

