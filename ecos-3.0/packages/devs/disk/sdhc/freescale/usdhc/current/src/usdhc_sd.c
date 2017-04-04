#define SUCCESS (0)
#define FAIL (1)

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

#include <cyg/io/usdhc.h>
#include <cyg/io/usdhc_host.h>
#include <cyg/io/registers/regsusdhc.h>

const cyg_uint32 sd_ocr_value[SD_OCR_VALUE_COUNT] = {
    SD_OCR_VALUE_HV_HC,
    SD_OCR_VALUE_LV_HC,
    SD_OCR_VALUE_HV_LC,
};

const cyg_uint32 sd_if_cmd_arg[SD_IF_CMD_ARG_COUNT] = {
    SD_IF_HV_COND_ARG,
    SD_IF_LV_COND_ARG,
};

/********************************************* Static Function ******************************************/
/*!
 * @brief Read RCA (relative card address) of SD card
 * 
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 sd_get_rca(cyg_uint32 instance)
{
    command_t cmd;
    cyg_int32 port, card_state, status = FAIL;
    command_response_t response;

    /* Check uSDHC Port */
    port = usdhc_get_port(instance);
    if (port == USDHC_NUMBER_PORTS) {
        return status;
    }

    /* Configure CMD3 */
    usdhc_cmd_config(&cmd, CMD3, NO_ARG, READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);

    /* Send CMD3 */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        response.format = RESPONSE_48;
        usdhc_host_read_response(instance, &response);

        /* Set RCA to Value Read */
        usdhc_device[port].rca = response.cmd_rsp0 >> RCA_SHIFT;

        /* Check the IDENT card state */
        card_state = CURR_CARD_STATE(response.cmd_rsp0);

        if (card_state == IDENT) {
            status = SUCCESS;
        }
    }

    return status;
}

/*!
 * @brief Set SD card bus width
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param data_width   Data transfer width
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 sd_set_bus_width(cyg_uint32 instance, cyg_int32 bus_width)
{
    command_t cmd;
    cyg_int32 port, address, status = FAIL;
    command_response_t response;

    /* Check uSDHC Port */
    port = usdhc_get_port(instance);
    if (port == USDHC_NUMBER_PORTS) {
        return status;
    }

    address = usdhc_device[port].rca << RCA_SHIFT;

    /* Check Bus Width */
    if ((bus_width != FOUR) && (bus_width != ONE)) {
        return status;
    }

    /* Configure CMD55 */
    usdhc_cmd_config(&cmd, CMD55, address, READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);

    /* Send ACMD6 */
    if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
        /* Check Response of Application Command */
        response.format = RESPONSE_48;
        usdhc_host_read_response(instance, &response);

        if (response.cmd_rsp0 & SD_R1_STATUS_APP_CMD_MSK) {
            bus_width = bus_width >> ONE;

            /* Configure ACMD6 */
            usdhc_cmd_config(&cmd, ACMD6, bus_width, READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);

            /* Send ACMD6 */
            if (usdhc_host_send_cmd(instance, &cmd) == SUCCESS) {
                status = SUCCESS;
            }
        }
    }

    return status;
}

/********************************************* Global Function ******************************************/
/*!
 * @brief Initialize SD - Get Card ID, Set RCA, Frequency and bus width.
 * 
 * @param instance     Instance number of the uSDHC module.
 * @param bus_width    bus width to be configured.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 sd_init(cyg_uint32 instance, cyg_int32 bus_width, cyg_uint8 *cid, cyg_uint8 *csd)
{
    cyg_int32 status = FAIL;

    /* Read CID */
	if (usdhc_get_cid(instance, cid) == SUCCESS) {

        /* Obtain RCA */
        if (sd_get_rca(instance) == SUCCESS) {

            usdhc_get_csd(instance, csd);

            /* Enable Operating Freq */
            usdhc_host_cfg_clock(instance, OPERATING_FREQ);

            if (bus_width == EIGHT) {
                bus_width = FOUR;
            }

            /* Enter Transfer State */
            if (usdhc_enter_trans(instance) == SUCCESS) {

                /* Set Bus Width for SD card */
                if (sd_set_bus_width(instance, bus_width) == SUCCESS) {
                    /* Set Bus Width for Controller */
                    usdhc_host_set_bus_width(instance, bus_width);

                    /* Set High Speed Here */
                    {
                        status = SUCCESS;
                    }
                }
            }
        }
    }

    return status;
}

/*!
 * @brief Valid the voltage.
 * 
 * @param instance     Instance number of the uSDHC module.
 * 
 * @return             0 if successful; 1 otherwise
 */
cyg_int32 sd_voltage_validation(cyg_uint32 instance, cyg_uint32 *revision)
{
    command_t cmd;
    command_response_t response;
    cyg_int32 port, loop = ZERO, status = FAIL;
    cyg_uint32 ocr_value = ZERO, card_usable = FAIL;

    /* Check uSDHC Port from Base Address */
    port = usdhc_get_port(instance);
    if (port == USDHC_NUMBER_PORTS) {
        return status;
    }

    while (loop < SD_IF_CMD_ARG_COUNT) {
        usdhc_cmd_config(&cmd, CMD8, sd_if_cmd_arg[loop], READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);
        if (usdhc_host_send_cmd(instance, &cmd) == FAIL) {
            loop++;

            if ((loop >= SD_IF_CMD_ARG_COUNT) && (loop < SD_OCR_VALUE_COUNT)) {
                /* Card is of SD-1.x spec with LC */
                ocr_value = sd_ocr_value[loop];
                *revision = 1;
                card_usable = SUCCESS;
            }
        } else {
            /* Card is supporting SD spec version >= 2.0 */
            response.format = RESPONSE_48;
            usdhc_host_read_response(instance, &response);
            *revision = 2;

            /* Check if response lies in the expected volatge range */
            if ((response.cmd_rsp0 & sd_if_cmd_arg[loop]) == sd_if_cmd_arg[loop]) {
                ocr_value = sd_ocr_value[loop];
                card_usable = SUCCESS;

                break;
            } else {
                ocr_value = ZERO;
                card_usable = FAIL;

                break;
            }
        }
    }

    loop = ZERO;

    if (card_usable == FAIL) {
        return status;
    }

    while ((loop < SD_VOLT_VALID_COUNT) && (status == FAIL)) {
        usdhc_cmd_config(&cmd, CMD55, ZERO, READ, RESPONSE_48, DATA_PRESENT_NONE, true, true);

        if (usdhc_host_send_cmd(instance, &cmd) == FAIL) {
            break;
        } else {
            usdhc_cmd_config(&cmd, ACMD41, ocr_value, READ, RESPONSE_48, DATA_PRESENT_NONE, false, false);

            if (usdhc_host_send_cmd(instance, &cmd) == FAIL) {
                break;
            } else {
                /* Check Response */
                response.format = RESPONSE_48;
                usdhc_host_read_response(instance, &response);

                /* Check Busy Bit Cleared or NOT */
                if (response.cmd_rsp0 & CARD_BUSY_BIT) {
                    /* Check card is HC or LC from card response */
                    if ((response.cmd_rsp0 & SD_OCR_HC_RES) == SD_OCR_HC_RES) {
                        usdhc_device[port].addr_mode = SECT_MODE;
                    } else {
                        usdhc_device[port].addr_mode = BYTE_MODE;
                    }

                    status = SUCCESS;
                } else {
                    loop++;
                    hal_delay_us(SD_VOLT_VALID_DELAY);
                }
            }
        }
    }

    return status;
}

