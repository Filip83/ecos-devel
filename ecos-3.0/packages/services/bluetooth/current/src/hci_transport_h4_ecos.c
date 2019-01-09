/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

/*
 *  hci_h4_transport_dma.c
 *
 *  HCI Transport implementation for basic H4 protocol for blocking UART write and IRQ-driven blockwise RX
 *
 *  Created by Matthias Ringwald on 4/29/09.
 */

#include <stdio.h>
#include <string.h>
#include "btlib/btstack_debug.h"
#include "btlib/hci.h"
#include "btlib/hci_transport.h"
#include "btlib/btstack_run_loop.h"
#include "btlib/btstack_run_loop_ecos.h"


#include <cyg/infra/diag.h>
#include <cyg/io/ehci_serial.h>
#include <cyg/hal/plf_io.h>

// assert pre-buffer for packet type is available
#if !defined(HCI_OUTGOING_PRE_BUFFER_SIZE) || (HCI_OUTGOING_PRE_BUFFER_SIZE == 0)
#error HCI_OUTGOING_PRE_BUFFER_SIZE not defined. Please update hci.h
#endif

#ifdef ENABLE_EHCILL
#error "HCI Transport H4 DMA does not support eHCILL. Please use hci_transport_h4_ehcill_dma.c instead."
#endif 


typedef enum {
    H4_W4_PACKET_TYPE = 1,
    H4_W4_EVENT_HEADER,
    H4_W4_ACL_HEADER,
    H4_W4_SCO_HEADER,
    H4_W4_PAYLOAD,
    H4_PACKET_RECEIVED
} H4_STATE;

typedef enum {
    TX_IDLE = 1,
    TX_W4_PACKET_SENT,
    TX_DONE
} TX_STATE;

typedef struct hci_transport_h4 {
    hci_transport_t transport;
    btstack_data_source_t *ds;
	ehci_serial_channel *chan;
} hci_transport_h4_t;

// prototypes
static void h4_process(btstack_data_source_t *ds, btstack_data_source_callback_type_t callback_type);
static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size); 
static void h4_block_received(void);
static void h4_block_sent(void);
static int h4_open(void);
static int h4_close(void);
static void h4_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size));
static int h4_can_send_packet_now(uint8_t packet_type);
static int h4_send_packet(uint8_t packet_type, uint8_t *packet, int size);
static int h4_set_baudrate(uint32_t baudrate);

static void ehci_init(ehci_serial_channel *chan);

// packet reader state machine
static  H4_STATE h4_state;
static int read_pos;
static int bytes_to_read;

 // bigger than largest packet
static uint8_t hci_packet_prefixed[HCI_INCOMING_PRE_BUFFER_SIZE + HCI_PACKET_BUFFER_SIZE];
static uint8_t * hci_packet = &hci_packet_prefixed[HCI_INCOMING_PRE_BUFFER_SIZE];

// tx state
static TX_STATE tx_state;

// static hci_transport_h4_t * hci_transport_h4 = NULL;
static  void (*packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size) = dummy_handler;

static btstack_data_source_t hci_transport_h4_dma_ds;

// hci_transport for use by hci
static  hci_transport_h4_t hci_transport_h4_dma = {
    {
  /*  .transport.name                          = */  "H4_EMBEDDED",
  /*  .transport.init                          = */  NULL,
  /*  .transport.open                          = */  h4_open,
  /*  .transport.close                         = */  h4_close,
  /*  .transport.register_packet_handler       = */  h4_register_packet_handler,
  /*  .transport.can_send_packet_now           = */  h4_can_send_packet_now,
  /*  .transport.send_packet                   = */  h4_send_packet,
  /*  .transport.set_baudrate                  = */  h4_set_baudrate,
  /*  .transport.reset_link                    = */  NULL,    
    },
  /*  .ds                                      = */  &hci_transport_h4_dma_ds
};



ehci_serial_callbacks_t cyg_io_ehci_serial_callbacks =
{
		ehci_init,
		h4_block_received,
		h4_block_sent
};


void ehci_init(ehci_serial_channel *chan)
{
    hci_transport_h4_dma.chan = chan;
}

static void h4_init_sm(void)
{
    h4_state = H4_W4_PACKET_TYPE;
    read_pos = 0;
    bytes_to_read = 1;
	hci_transport_h4_dma.chan->funs->block_receive(hci_transport_h4_dma.chan,
								hci_packet, bytes_to_read);
}

cyg_devtab_entry_t *t;
cyg_io_handle_t *handle;
ehci_serial_channel *chan;

static int h4_open(void)
{

	if(cyg_io_lookup("/dev/ehci",&handle) != ENOERR)
	{
		CYG_FAIL("Open ehci\n");
	}

	t = (cyg_devtab_entry_t*)handle;
	chan = (ehci_serial_channel*)t->priv;
		chan->callbacks->ehci_init(chan);
    // set up data_source
    btstack_run_loop_set_data_source_handler(&hci_transport_h4_dma_ds, &h4_process);
    btstack_run_loop_enable_data_source_callbacks(&hci_transport_h4_dma_ds, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&hci_transport_h4_dma_ds);
    
    //
    h4_init_sm();
    tx_state = TX_IDLE;

    return 0;
}

static int h4_close(void)
{
    // remove data source
    btstack_run_loop_disable_data_source_callbacks(&hci_transport_h4_dma_ds, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_remove_data_source(&hci_transport_h4_dma_ds);
    // close device 
    // ...
    //hci_transport_h4_dma.chan->funs->block_colse(hci_transport_h4_dma.chan);
    return 0;
}

static void h4_block_received(void)
{
    read_pos += bytes_to_read;
    
    // act
    switch (h4_state) {
        case H4_W4_PACKET_TYPE:
            switch (hci_packet[0]) {
                case HCI_EVENT_PACKET:
                    h4_state = H4_W4_EVENT_HEADER;
                    bytes_to_read = HCI_EVENT_HEADER_SIZE;
                    break;
                case HCI_ACL_DATA_PACKET:
                    h4_state = H4_W4_ACL_HEADER;
                    bytes_to_read = HCI_ACL_HEADER_SIZE;
                    break;
                case HCI_SCO_DATA_PACKET:
                    h4_state = H4_W4_SCO_HEADER;
                    bytes_to_read = HCI_SCO_HEADER_SIZE;
                    break;
                default:
                    log_error("h4_process: invalid packet type 0x%02x", hci_packet[0]);
                    h4_init_sm();
                    return;
            }
            break;
            
        case H4_W4_EVENT_HEADER:
            bytes_to_read = hci_packet[2];
            if (bytes_to_read == 0) {
                h4_state = H4_PACKET_RECEIVED; 
                break;
            }
            h4_state = H4_W4_PAYLOAD;
            break;
            
        case H4_W4_ACL_HEADER:
            bytes_to_read = little_endian_read_16( hci_packet, 3);
            // check ACL length
            if (HCI_ACL_HEADER_SIZE + bytes_to_read >  HCI_PACKET_BUFFER_SIZE){
                log_error("h4_process: invalid ACL payload len %u - only space for %u", bytes_to_read, HCI_PACKET_BUFFER_SIZE - HCI_ACL_HEADER_SIZE);
                h4_init_sm();
                return;              
            }
            if (bytes_to_read == 0) {
                h4_state = H4_PACKET_RECEIVED; 
                break;
            }
            h4_state = H4_W4_PAYLOAD;
            break;
        
        case H4_W4_SCO_HEADER:
            bytes_to_read = hci_packet[3];
            if (bytes_to_read == 0) {
                h4_state = H4_PACKET_RECEIVED; 
                break;
            }
            h4_state = H4_W4_PAYLOAD;
            break;            

        case H4_W4_PAYLOAD:
            h4_state = H4_PACKET_RECEIVED;
            bytes_to_read = 0;
            // trigger run loop
            btstack_run_loop_ecos_trigger();
            break;
            
        default:
            bytes_to_read = 0;
            break;
    }
    
    // read next block
    if (bytes_to_read) 
    {
        hci_transport_h4_dma.chan->funs->block_receive(hci_transport_h4_dma.chan,
				&hci_packet[read_pos], bytes_to_read);
		
    }
}

static void h4_block_sent(void)
{
    switch (tx_state){
        case TX_W4_PACKET_SENT:
            tx_state = TX_DONE;
            // trigger run loop
            btstack_run_loop_ecos_trigger();
            break;
        default:
            break;
    }
}

static void h4_register_packet_handler(void (*handler)(uint8_t packet_type, 
        uint8_t *packet, uint16_t size))
{
    packet_handler = handler;
}

static void h4_process(btstack_data_source_t *ds, 
        btstack_data_source_callback_type_t callback_type) 
{
    
    // notify about packet sent
    if (tx_state == TX_DONE){
        // reset state
        tx_state = TX_IDLE;
        uint8_t event[] = { HCI_EVENT_TRANSPORT_PACKET_SENT, 0 };
        packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
    }

    if (h4_state != H4_PACKET_RECEIVED) return;
            
    packet_handler(hci_packet[0], &hci_packet[1], read_pos-1);

    h4_init_sm();
                                
    return;
}

static int h4_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
    // write in progress
    if (tx_state != TX_IDLE)
    {
        log_error("h4_send_packet with tx_state = %u, type %u, data %02x %02x %02x", tx_state, packet_type, packet[0], packet[1], packet[2]);
        return -1;
    }
    
    // store packet type before actual data and increase size
    size++;
    packet--;
    *packet = packet_type;

    tx_state = TX_W4_PACKET_SENT;
    hci_transport_h4_dma.chan->funs->block_send(hci_transport_h4_dma.chan,
														packet, size);
    
    return 0;
}

static int h4_set_baudrate(uint32_t baudrate)
{
    cyg_serial_info_t uart_cofig;
    uint32_t          len = sizeof(cyg_serial_info_t);
    log_info("h4_set_baudrate - set baud %lu", baudrate);
    uart_cofig.baud = baudrate;
    uart_cofig.stop = CYGNUM_SERIAL_STOP_1;
    uart_cofig.parity = CYGNUM_SERIAL_PARITY_NONE;
    uart_cofig.word_length = CYGNUM_SERIAL_WORD_LENGTH_8;
    uart_cofig.flags = CYGNUM_SERIAL_FLOW_RTSCTS_RX | CYGNUM_SERIAL_FLOW_RTSCTS_TX;

    hci_transport_h4_dma.chan->funs->set_config(
            hci_transport_h4_dma.chan,CYG_IO_SET_CONFIG_SERIAL_INFO,
            &uart_cofig,&len);
	return 0;
}

static int h4_can_send_packet_now(uint8_t packet_type)
{
    return tx_state == TX_IDLE;

}

static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size)
{
}

// get h4 singleton
const hci_transport_t * hci_transport_h4_instance(
                const btstack_uart_block_t * uart_driver)
{ 
    return &hci_transport_h4_dma.transport;
}
