/*
 * deamon.c
 *
 *  Created on: 15. 4. 2018
 *      Author: filip
 */
//#include "btstack.h"
#include "btlib/classic/btstack_link_key_db_memory_save.h"

#include <cyg/io/io.h>
#include <cyg/io/devtab.h>
#include <cyg/io/ehci_serial.h>
//#include <cyg/kernel/kapi.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <btlib/deamon/deamon_io.h>


#include <errno.h>
#include <string.h>
#include <stdio.h>

channel_state_t l_channels[BLUETOOTH_EAMON_CHANNELS_COUNT] =
{
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 0
	{
		.client = NULL,
		.io_ch_id = 0,
		.channel_type = NotUsed,
		.connection_status = NoConnected,
		.channel_cid = 0,
		.l2cap_psm = 0,
		.sdp_record = NULL,
		.power_mode = HCI_POWER_OFF,
		.discoverable = 0
	}
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 1
	,{
		.client = NULL,
		.io_ch_id = 1,
		.channel_type = NotUsed,
		.connection_status = NoConnected,
		.channel_cid = 0,
		.l2cap_psm = 0,
		.sdp_record = NULL,
		.power_mode = HCI_POWER_OFF,
		.discoverable = 0
	}
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 2
	,{
		.client = NULL,
		.io_ch_id = 2,
		.channel_type = NotUsed,
		.connection_status = NoConnected,
		.channel_cid = 0,
		.l2cap_psm = 0,
		.sdp_record = NULL,
		.power_mode = HCI_POWER_OFF,
		.discoverable = 0
	}
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 3
	,{
		.client = NULL,
		.io_ch_id = 3,
		.channel_type = NotUsed,
		.connection_status = NoConnected,
		.channel_cid = 0,
		.l2cap_psm = 0,
		.sdp_record = NULL,
		.power_mode = HCI_POWER_OFF,
		.discoverable = 0
	}
#endif
};

client_state_t client_state =
{
		.initialised = false,
		.process_started = false,
		.power_mode  = HCI_POWER_OFF,
		.num_channels = BLUETOOTH_EAMON_CHANNELS_COUNT,
		.channels_list = l_channels,
		.global_enable = 0,
		.power_management_sleep = 0,
		.InqueryInProgress = false,
		.attribute_value_buffer_size = 1000,
		.devices_count = 0
};
// Thread data
cyg_handle_t bt_thread_handle;
cyg_thread   bt_thread_data;
// TODO: Typical stack size
cyg_uint8    bt_thrad_stack[8192];



static btstack_packet_callback_registration_t hci_event_callback_registration;

externC void btstack_run_loop_ecos_trigger(void);
void bt_Start(void);

int btstack_command_handler(channel_state_t *channel, uint32_t key, uint8_t *packet, uint16_t size);
void btstack_command_handler_on_bt_thread(struct btstack_data_source *ds, btstack_data_source_callback_type_t callback_type);


static channel_state_t * connection_for_l2cap_cid(uint16_t cid){
	int i ;
	for(i = 0; i < client_state.num_channels; i++)
	{
		if((client_state.channels_list[i].channel_cid == cid) &&
			(client_state.channels_list[i].channel_type == Type_L2Cap ))
			return &client_state.channels_list[i];
	}
    return NULL;
}

static channel_state_t * connection_for_incomming_l2cap_cid(uint16_t cid){
	int i ;
	for(i = 0; i < client_state.num_channels; i++)
	{
		if((client_state.channels_list[i].channel_cid == cid) &&
		   (client_state.channels_list[i].channel_type == L2Cap_Service))
			return &client_state.channels_list[i];
	}
    return NULL;
}


static const uint8_t removeServiceRecordHandleAttributeIDList[] = { 0x36, 0x00, 0x05, 0x0A, 0x00, 0x01, 0xFF, 0xFF };

// register a service record
// pre: AttributeIDs are in ascending order
// pre: ServiceRecordHandle is first attribute and is not already registered in database
// @returns status
static uint32_t daemon_sdp_create_and_register_service(uint8_t * record){

    // create new handle
    uint32_t record_handle = sdp_create_service_record_handle();

    // calculate size of new service record: DES (2 byte len)
    // + ServiceRecordHandle attribute (UINT16 UINT32) + size of existing attributes
    uint16_t recordSize =  3 + (3 + 5) + de_get_data_size(record);

    // alloc memory for new service record
    uint8_t * newRecord = malloc(recordSize);
    if (!newRecord) return 0;

    // create DES for new record
    de_create_sequence(newRecord);

    // set service record handle
    de_add_number(newRecord, DE_UINT, DE_SIZE_16, 0);
    de_add_number(newRecord, DE_UINT, DE_SIZE_32, record_handle);

    // add other attributes
    sdp_append_attributes_in_attributeIDList(record, (uint8_t *) removeServiceRecordHandleAttributeIDList, 0, recordSize, newRecord);

    uint8_t status = sdp_register_service(newRecord);

    if (status) {
        free(newRecord);
        return 0;
    }

    return record_handle;
}

static channel_state_t * connection_for_rfcomm_cid(uint16_t cid){
	int i ;
	for(i = 0; i < client_state.num_channels; i++)
	{
		if((client_state.channels_list[i].channel_cid == cid) &&
		   (client_state.channels_list[i].channel_type &Type_RfCom))
			return &client_state.channels_list[i];
	}
	return NULL;
}

static channel_state_t * connection_for_rfcomm_id(uint16_t id){
	int i ;
	for(i = 0; i < client_state.num_channels; i++)
	{
		if((client_state.channels_list[i].rfcomm_id == id) &&
		   (client_state.channels_list[i].channel_type &Type_RfCom))
			return &client_state.channels_list[i];
	}
	return NULL;
}

static channel_state_t * connection_for_incomming_rfcomm(uint16_t cid){
	int i ;
	for(i = 0; i < client_state.num_channels; i++)
	{
		if((client_state.channels_list[i].channel_cid == cid) &&
		   (client_state.channels_list[i].channel_type == RfCom_Service ||
		    client_state.channels_list[i].channel_type == RfCom_Service_WithCredits))
			return &client_state.channels_list[i];
	}
	return NULL;
}

static void hci_emit_btstack_version(uint8_t *event){
    log_info("DAEMON_EVENT_VERSION %u.%u", BTSTACK_MAJOR, BTSTACK_MINOR);
    /*event[0] = DAEMON_EVENT_VERSION;
    event[1] = sizeof(event) - 2;*/
    event[0] = BTSTACK_MAJOR;
    event[1] = BTSTACK_MINOR;
    *((uint16_t*)(event + 2)) = 3257; // last SVN commit on Google Code + 1

}


static int clients_require_discoverable(client_state_t *client){
	int i;
    for (i = 0; i < client->num_channels; i++){
        if (client->channels_list[i].discoverable) {
            return 1;
        }
    }
    return 0;
}

static int clients_require_power(client_state_t *client){
	int i;
    for (i = 0; i < client->num_channels; i++){
        if (client->channels_list[i].power_mode == HCI_POWER_ON) {
            return 1;
        }
    }
    return 0;
}

static void clients_clear_power_request(client_state_t *client){
	int i;
    for (i = 0; i < client->num_channels; i++){
    	client->channels_list[i].power_mode = HCI_POWER_OFF;
    }
}

static btstack_linked_list_bt_device_t * clents_create_add_remote_device_to_list(void){
	btstack_linked_list_bt_device_t *new_device = NULL;
	btstack_linked_list_iterator_t it;
    btstack_linked_list_iterator_init(&it, &client_state.remote_devices);

    new_device = malloc(sizeof(btstack_linked_list_bt_device_t));
    if (!new_device) return NULL;
    memset(new_device,0,sizeof(btstack_linked_list_bt_device_t));
    btstack_linked_list_add(&client_state.remote_devices, (btstack_linked_item_t *) new_device);
    client_state.devices_count++;
    return new_device;
}

static void clents_remove_and_free_remote_device_list(void){
    btstack_linked_list_iterator_t it;
    btstack_linked_list_iterator_init(&it, &client_state.remote_devices);
    while (btstack_linked_list_iterator_has_next(&it)){
    	btstack_linked_list_bt_device_t * item = (btstack_linked_list_bt_device_t*) btstack_linked_list_iterator_next(&it);
    	if(item->value.name != NULL)
    		free(item->value.name);
        btstack_linked_list_remove(&client_state.remote_devices, (btstack_linked_item_t *) item);
        free(item);
    }

    client_state.devices_count = 0;
}

static btstack_linked_list_bt_device_t * clents_get_remote_device_for_address(bd_addr_t addr){
    btstack_linked_list_iterator_t cl;
    btstack_linked_list_iterator_init(&cl, &client_state.remote_devices);
    while (btstack_linked_list_iterator_has_next(&cl)){
    	btstack_linked_list_bt_device_t * item = (btstack_linked_list_bt_device_t *) btstack_linked_list_iterator_next(&cl);
        if(!bd_addr_cmp(item->value.address,addr))
        	return item;
    }
    return NULL;
}

static void clents_add_remote_device_name(bd_addr_t addr, char *name, int len)
{
	char *item = NULL;
	btstack_linked_list_bt_device_t *dev = clents_get_remote_device_for_address(addr);
	if(dev == NULL)
		return;
	item = malloc(len+1);
	if(item == NULL)
		return;

	memcpy(item,name,len);
	item[len] = 0;
	if(dev->value.name != NULL)
		free(dev->value.name);

	dev->value.name = item;
	dev->value.state = REMOTE_NAME_FETCHED;
}

static int clents_remote_device_continue_name_requests(void)
{
	//int ret = 0;
    btstack_linked_list_iterator_t cl;
    btstack_linked_list_iterator_init(&cl, &client_state.remote_devices);
    while (btstack_linked_list_iterator_has_next(&cl)){
    	btstack_linked_list_bt_device_t * item = (btstack_linked_list_bt_device_t *) btstack_linked_list_iterator_next(&cl);
        if(item->value.state == REMOTE_NAME_REQUEST)
        {
        	item->value.state = REMOTE_NAME_INQUIRED;
        	log_info("REMOTE_NAME_INQUIRED\n");
        	hci_send_cmd(&hci_remote_name_request, item->value.address,
        			item->value.pageScanRepetitionMode, 0, item->value.clockOffset | 0x8000);
        	return 1;
        }
    }
    return 0;
}

static int clents_remote_device_is_complet_name_requests(void)
{
	int ret = 0;
	btstack_linked_list_iterator_t cl;
	btstack_linked_list_iterator_init(&cl, &client_state.remote_devices);
	while (btstack_linked_list_iterator_has_next(&cl)){
		btstack_linked_list_bt_device_t * item = (btstack_linked_list_bt_device_t *) btstack_linked_list_iterator_next(&cl);
		if(item->value.state != REMOTE_NAME_FETCHED)
		{
			ret |= 1;
		}
	}
	return ret;
}

static btstack_linked_list_bt_service_t * clents_create_add_remote_device_service_to_list(uint8_t ch_id, char *name){
	btstack_linked_list_bt_service_t *new_service = NULL;
	btstack_linked_list_iterator_t it;
    btstack_linked_list_iterator_init(&it, &client_state.remote_device_service);

    new_service = malloc(sizeof(btstack_linked_list_bt_service_t));
    if (!new_service) return NULL;
    memset(new_service,0,sizeof(btstack_linked_list_bt_service_t));

    new_service->value.port = ch_id;
    int len = strlen(name);
    new_service->value.name = malloc(len);
    if(new_service->value.name == NULL)
    {
    	free(new_service);
    	return NULL;
    }
    strcpy(new_service->value.name,name);
	btstack_linked_list_add(&client_state.remote_device_service, (btstack_linked_item_t *) new_service);
    return new_service;
}

static void clents_remove_and_free_remote_device_service_list(void){
	btstack_linked_list_iterator_t it;
    btstack_linked_list_iterator_init(&it, &client_state.remote_device_service);
    while (btstack_linked_list_iterator_has_next(&it)){
    	btstack_linked_list_bt_service_t * item = (btstack_linked_list_bt_service_t*) btstack_linked_list_iterator_next(&it);
    	if(item->value.name != NULL)
    		free(item->value.name);
        btstack_linked_list_remove(&client_state.remote_device_service, (btstack_linked_item_t *) item);
        free(item);
    }
}

static void store_found_service(const char * name, uint8_t port){
	log_info("APP: Service name: '%s', RFCOMM port %u\n", name, port);
    clents_create_add_remote_device_service_to_list(port, (char*)name);
}

///////
static void handle_sdp_rfcomm_service_result(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
	switch (hci_event_packet_get_type(packet)){
        case SDP_EVENT_QUERY_RFCOMM_SERVICE:

        	store_found_service(sdp_event_query_rfcomm_service_get_name(packet),
        	                    sdp_event_query_rfcomm_service_get_rfcomm_channel(packet));
        	break;
        case SDP_EVENT_QUERY_COMPLETE:
            // already HCI Events, just forward them

        	cyg_mutex_lock(&client_state.global_mutex);
        	client_state.global_ioctl_status =  sdp_event_query_complete_get_status(packet);
        	cyg_cond_signal(&client_state.global_cond);
        	cyg_mutex_unlock(&client_state.global_mutex);
            break;
        default:
            break;
    }
}

static void sdp_client_assert_buffer(int size){
    if (size > client_state.attribute_value_buffer_size){
        log_error("SDP attribute value buffer size exceeded: available %d, required %d", client_state.attribute_value_buffer_size, size);
    }
}

// define new packet type SDP_CLIENT_PACKET
static void handle_sdp_client_query_result(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    int event_len;

    switch (hci_event_packet_get_type(packet)){
        case SDP_EVENT_QUERY_ATTRIBUTE_BYTE:
            sdp_client_assert_buffer(sdp_event_query_attribute_byte_get_attribute_length(packet));
            client_state.attribute_value[sdp_event_query_attribute_byte_get_data_offset(packet)] = sdp_event_query_attribute_byte_get_data(packet);
            if ((uint16_t)(sdp_event_query_attribute_byte_get_data_offset(packet)+1) == sdp_event_query_attribute_byte_get_attribute_length(packet)){
                log_info_hexdump(client_state.attribute_value, sdp_event_query_attribute_byte_get_attribute_length(packet));

                int event_len = 1 + 3 * 2 + sdp_event_query_attribute_byte_get_attribute_length(packet);
                uint8_t event[event_len];
                event[0] = SDP_EVENT_QUERY_ATTRIBUTE_VALUE;
                little_endian_store_16(event, 1, sdp_event_query_attribute_byte_get_record_id(packet));
                little_endian_store_16(event, 3, sdp_event_query_attribute_byte_get_attribute_id(packet));
                little_endian_store_16(event, 5, (uint16_t)sdp_event_query_attribute_byte_get_attribute_length(packet));
                memcpy(&event[7], client_state.attribute_value, sdp_event_query_attribute_byte_get_attribute_length(packet));
                hci_dump_packet(SDP_CLIENT_PACKET, 0, event, event_len);
                //socket_connection_send_packet(sdp_client_query_connection, SDP_CLIENT_PACKET, 0, event, event_len);
            }
            break;
        case SDP_EVENT_QUERY_COMPLETE:
            event_len = packet[1] + 2;
            hci_dump_packet(HCI_EVENT_PACKET, 0, packet, event_len);
            //socket_connection_send_packet(sdp_client_query_connection, HCI_EVENT_PACKET, 0, packet, event_len);
            break;
    }
}

static void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size)
{
	uint16_t cid;
	channel_state_t *connection;
	switch (packet_type)
	{
	case L2CAP_DATA_PACKET:
		connection = connection_for_l2cap_cid(channel);
		if (!connection) return;

		cyg_mutex_lock(&connection->read_queue.mutex);
		queue_werite(&connection->read_queue,packet,size);
		cyg_cond_signal(&connection->read_queue.cond);
		cyg_mutex_unlock(&connection->read_queue.mutex);
		break;
	case L2CAP_EVENT_CAN_SEND_NOW:
		connection = connection_for_l2cap_cid(channel);
		if (!connection) return;

		cyg_mutex_lock(&connection->ch_mutex);
		if(connection->write_queue.len != 0)
		{
			rfcomm_send(connection->channel_cid,connection->write_queue.data,
					connection->write_queue.len);
			cyg_cond_signal(&connection->ch_cond);
		}

		cyg_mutex_unlock(&connection->ch_mutex);

		break;
	case L2CAP_EVENT_CHANNEL_OPENED:
		cid = l2cap_event_channel_opened_get_local_cid(packet);
		log_info("L2CAP_EVENT_CHANNEL_OPENED %d\n",cid);
		connection = connection_for_l2cap_cid(cid);
		if (!connection) break;
		cyg_mutex_lock(&connection->ch_mutex);
		if (packet[2]) {
			connection->connection_status = NoConnected;
			connection->channel_type      = NotUsed;
			connection->io_status         = -packet[2];
		} else {
			connection->connection_status = Connected;
			connection->discoverable      = 0;
			gap_discoverable_control(clients_require_discoverable(connection->client));
			connection->io_status         = ENOERR;
		}
		cyg_cond_signal(&connection->ch_cond);
		cyg_mutex_unlock(&connection->ch_mutex);
		break;
	case L2CAP_EVENT_INCOMING_CONNECTION:
		log_info("L2CAP_EVENT_INCOMING_CONNECTION\n");
		break;
	case L2CAP_EVENT_CHANNEL_CLOSED:
		cid = l2cap_event_channel_closed_get_local_cid(packet);
		log_info("L2CAP_EVENT_CHANNEL_CLOSED %d\n", cid);
		connection = connection_for_l2cap_cid(cid);
		if (!connection) break;
		cyg_mutex_lock(&connection->ch_mutex);
		if(connection->channel_type & Type_L2Cap_Service)
		{
			connection->connection_status = WaitingForConnection;
			connection->discoverable      = 1;
			gap_discoverable_control(clients_require_discoverable(connection->client));
		}
		else
		{
			connection->connection_status = NoConnected;
			connection->channel_type      = NotUsed;
		}
		connection->io_status             = ENOERR;
		cyg_cond_broadcast(&connection->ch_cond);
		cyg_mutex_unlock(&connection->ch_mutex);
		break;
	}
}
static void rfcomm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size)
{
	uint16_t cid;
	uint8_t  rfcomm_id;
	uint8_t event;
	channel_state_t *connection;
	switch (packet_type)
	{
	case HCI_EVENT_PACKET:
		event = hci_event_packet_get_type(packet);
		switch(event)
		{
		case RFCOMM_EVENT_CHANNEL_OPENED:
			cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
			log_info("RFCOMM_EVENT_CHANNEL_OPENED %d\n",cid);
			connection = connection_for_rfcomm_cid(cid);
			if (!connection) break;
			cyg_mutex_lock(&connection->ch_mutex);
			if (packet[2]) {
				connection->connection_status = NoConnected;
				connection->channel_type      = NotUsed;
				connection->io_status         = -packet[2];
			} else {
				connection->connection_status = Connected;
				connection->discoverable      = 0;
				gap_discoverable_control(clients_require_discoverable(connection->client));
				connection->io_status         = ENOERR;
			}
			cyg_cond_signal(&connection->ch_cond);
			cyg_mutex_unlock(&connection->ch_mutex);
			break;
		case RFCOMM_EVENT_CHANNEL_CLOSED:
			cid = rfcomm_event_channel_closed_get_rfcomm_cid(packet);
			log_info("RFCOMM_EVENT_CHANNEL_CLOSED %d\n", cid);
			connection = connection_for_rfcomm_cid(cid);
			if (!connection) break;
			cyg_mutex_lock(&connection->ch_mutex);
			cyg_mutex_lock(&connection->read_queue.mutex);
			if(connection->channel_type & Type_Rfcom_Service)
			{
				connection->connection_status = WaitingForConnection;
				connection->discoverable      = 1;
				gap_discoverable_control(clients_require_discoverable(connection->client));
			}
			else
			{
				connection->connection_status = NoConnected;
				connection->channel_type      = NotUsed;
			}
			connection->io_status             = ENOERR;
			cyg_cond_broadcast(&connection->ch_cond);
			cyg_cond_broadcast(&connection->read_queue.cond);
			cyg_mutex_unlock(&connection->read_queue.mutex);
			cyg_mutex_unlock(&connection->ch_mutex);
			break;
		case RFCOMM_EVENT_INCOMING_CONNECTION:
			// TODO pokud to bude takhle tak budou blokvoany oeprace i pro
			// ostatni kanaly
			cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
			rfcomm_id = rfcomm_event_incoming_connection_get_server_channel(packet);
			log_info("RFCOMM_EVENT_INCOMING_CONNECTION %d, %d\n",rfcomm_id,cid);

			connection = connection_for_rfcomm_id(rfcomm_id);
			if(connection == NULL)
			{
				rfcomm_decline_connection(cid);
				client_state.global_ioctl_status = ENODEV;
			}
			else
			{
				connection->channel_cid = cid;
				rfcomm_accept_connection(cid);
				client_state.global_ioctl_status = ENOERR;
			}
			break;
		case RFCOMM_EVENT_CAN_SEND_NOW:
			cid = rfcomm_event_can_send_now_get_rfcomm_cid(packet);
			//log_info("RFCOMM_EVENT_CAN_SEND_NOW %d\n", cid);
			connection = connection_for_rfcomm_cid(cid);
			if (!connection) return;

			if(cyg_thread_self() == ((client_state_t*)connection->client)->thread_handle)
			{
				cyg_mutex_lock(&connection->ch_mutex);;

				if(connection->write_queue.len != 0)
				{
					if(rfcomm_send(connection->channel_cid,connection->write_queue.data,
							connection->write_queue.len))
					{
						log_info("Error send\n");
					}
					connection->write_queue.complete = true;
					cyg_cond_signal(&connection->ch_cond);
				}

				cyg_mutex_unlock(&connection->ch_mutex);
			}
			else
			{
				if(connection->write_queue.len != 0)
				{
					if(rfcomm_send(connection->channel_cid,connection->write_queue.data,
							connection->write_queue.len))
					{
						log_info("Error send\n");
					}
					connection->write_queue.complete = true;
					//cyg_cond_signal(&connection->ch_cond);
				}
			}

			break;
		}
		break;

	case RFCOMM_DATA_PACKET:
		connection = connection_for_rfcomm_cid(channel);
		if (!connection) return;

		if(size != 0)
		{
			cyg_mutex_lock(&connection->read_queue.mutex);
			queue_werite(&connection->read_queue,packet,size);
			cyg_cond_signal(&connection->read_queue.cond);
			cyg_mutex_unlock(&connection->read_queue.mutex);
		}
		break;
	}
}



// TODO: vyskouset jestli pri unregiter service se i odpoiji kdzztak pak volat disconnect

void btstack_command_handler_on_bt_thread(struct btstack_data_source *ds, btstack_data_source_callback_type_t callback_type)
{
	ds->ret = btstack_command_handler(ds->channel,ds->key,ds->packet,ds->size);
}

int btstack_command_handler(channel_state_t *channel, uint32_t key, uint8_t *packet, uint16_t size){

	int       ret = ENOERR;
    bd_addr_t addr;
    uint16_t cid;
    uint16_t psm;
    //uint16_t service_channel;
    uint16_t mtu;
    uint16_t uuid;
    uint16_t inquiry_interval;
    uint8_t  reason;
    uint8_t  rfcomm_channel;
    uint8_t  rfcomm_credits;
    uint32_t service_record_handle;
    uint8_t status;
    uint8_t  * data;
    uint16_t   timeout;

    uint16_t serviceSearchPatternLen;
    uint16_t attributeIDListLen;

    client_state_t *client = (client_state_t*)channel->client;

    // BTstack internal commands - 16 Bit OpCode, 8 Bit ParamLen, Params...
    switch (key){
        case BTSTACK_GET_STATE:
        	log_info("BTSTACK_GET_STATE");
        	cyg_mutex_lock(&client_state.global_mutex);
            hci_emit_state();
            cyg_cond_wait(&client_state.global_cond);
            packet[0] = client_state.global_enable;
            cyg_mutex_unlock(&client_state.global_mutex);
            return ENOERR;
            break;
        case BTSTACK_SET_POWER_MODE:
            log_info("BTSTACK_SET_POWER_MODE %u", packet[0]);
            channel->power_mode = packet[0];
            // handle merged state
            cyg_mutex_lock(&client_state.global_mutex);
            if(clients_require_power(client))
            {
            	hci_power_control(HCI_POWER_ON);
            }
            else
            {
            	hci_power_control(HCI_POWER_OFF);
            }
            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case BTSTACK_GET_VERSION:
        	log_info("BTSTACK_GET_VERSION");
        	cyg_mutex_lock(&client_state.global_mutex);
            hci_emit_btstack_version(packet);
            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case BTSTACK_SET_SYSTEM_BLUETOOTH_ENABLED:
        case BTSTACK_GET_SYSTEM_BLUETOOTH_ENABLED:
        	log_info("BTSTACK_GET_SYSTEM_BLUETOOTH_ENABLED");
        	cyg_mutex_lock(&client_state.global_mutex);
        	packet[0] = client->global_enable;
        	cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case BTSTACK_SET_DISCOVERABLE:
        	log_info("BTSTACK_SET_DISCOVERABLE discoverable %u)", packet[0]);
        	cyg_mutex_lock(&client_state.global_mutex);
            channel->discoverable = packet[0];
            // merge state
            gap_discoverable_control(clients_require_discoverable(client));
            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case BTSTACK_SET_BLUETOOTH_ENABLED:
            log_info("BTSTACK_SET_BLUETOOTH_ENABLED: %u\n", packet[0]);
            cyg_mutex_lock(&client_state.global_mutex);
            if (packet[0]) {
                // global enable
            	if(client->global_enable)
            	{
            		cyg_mutex_unlock(&client_state.global_mutex);
            		return ENOERR;
            	}
                client->global_enable = 1;
                hci_power_control(HCI_POWER_ON);
            } else {
            	if(!clients_require_power(client))
            	{
					client->global_enable = 0;
					hci_power_control(HCI_POWER_OFF);
            	}
            	else
            	{
            		return EACCES;
            	}
            }
            cyg_cond_wait(&client_state.global_cond);
            ret = client_state.global_ioctl_status;
            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case BTSTACK_INQUERY_START:
        	cyg_mutex_lock(&client_state.global_mutex);

        	if(!client->InqueryInProgress)
        	{
        		int i ;
        		ret = EBUSY;
        		client->InqueryInProgress = false;
				inquiry_interval = *((uint16_t*)(packet));
				for(i = 0; i < 5; i++)
				{
					if (!hci_can_send_command_packet_now())
					{
						cyg_thread_delay(100);
						continue;
					}
					log_info("BTSTACK_INQUERY_START: %u\n",inquiry_interval);
					clents_remove_and_free_remote_device_list();
					hci_send_cmd(&hci_inquiry, HCI_INQUIRY_LAP, inquiry_interval, 0);
					client->InqueryInProgress = true;
					cyg_mutex_unlock(&client_state.global_mutex);
					return ENOERR;
				}
				log_info("BTSTACK_INQUERY_START: Failed\n");
        	}
        	else
        	{
        		log_info("BTSTACK_INQUERY_START: In progress\n");
        		ret = EBUSY;
        	}
        	cyg_mutex_unlock(&client_state.global_mutex);
        	break;
        case BTSTACK_IS_INQUERY_COMPLETE:
        	log_info("BTSTACK_IS_INQUERY_COMPLETE: %u\n", client->InqueryInProgress);
        	cyg_mutex_lock(&client_state.global_mutex);
        	if(client->InqueryInProgress)
        	{
        		packet[0] = 1;
        	}
        	else
        	{
        		packet[0] = 0;
        	}
        	cyg_mutex_unlock(&client_state.global_mutex);
        	break;
        case BTSTACK_GET_INQUERY_DEVICE_COUNT:
        	cyg_mutex_lock(&client_state.global_mutex);
        	if(!client->InqueryInProgress)
        	{
        		log_info("BTSTACK_GET_INQUERY_DEVICE_COUNT: %u\n", client->devices_count);
        		packet[0] = client->devices_count;
        		ret = ENOERR;
        	}
        	else
        	{
        		log_info("BTSTACK_GET_INQUERY_DEVICE_COUNT: Busy\n");
        		ret = EBUSY;
        	}
        	cyg_mutex_unlock(&client_state.global_mutex);
        	break;
        case BTSTACK_GET_INQUERY_DEVICES:
        	cyg_mutex_lock(&client_state.global_mutex);
        	if(!client->InqueryInProgress)
			{
        		btstack_linked_list_t** list = (btstack_linked_list_t**)packet;
        		log_info("BTSTACK_GET_INQUERY_DEVICES\n");
        		*list = (btstack_linked_list_t*) &client->remote_devices;
			}
			else
			{
				log_info("BTSTACK_GET_INQUERY_DEVICES: Busy\n");
				ret = EBUSY;
			}
        	cyg_mutex_unlock(&client_state.global_mutex);
        	break;
        case L2CAP_CREATE_CHANNEL:
        	if(channel->channel_type != NotUsed)
        	{
        		log_info("L2CAP_CREATE_CHANNEL: channel busy\n");
        		ret = EBUSY;
        	}
        	else
        	{
        		cyg_mutex_lock(&client_state.global_mutex);
        		ioctl_l2cap_create_channel_t *ioctl = (ioctl_l2cap_create_channel_t*)packet;
        		memcpy(addr,ioctl->addr,6);
				psm = ioctl->psm;
				mtu = ioctl->mtu;
				if(mtu == 0)
					mtu = 150;
				status = l2cap_create_channel(l2cap_packet_handler, addr, psm, mtu, &channel->channel_cid);
				cyg_mutex_unlock(&client_state.global_mutex);
				if(status)
				{
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
					ret = -status;
					log_info("L2CAP_CREATE_CHANNEL ERROR: %d\n",status);
				}

				if(!status)
				{
					// Unlock for other threads beacose
					// we will be waiting for connection completion
					cyg_mutex_lock(&channel->ch_mutex);
					channel->channel_type = L2Cap;
					channel->connection_status = WaitingForConnection;
					cyg_cond_wait(&channel->ch_cond);
					ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);
					log_info("L2CAP_CREATE_CHANNEL: created\n");
				}
        	}
            break;
        case L2CAP_DISCONNECT:
        	if(channel->channel_type != Type_L2Cap)
        	{
        		log_info("L2CAP_CREATE_CHANNEL: No L2Cap channel\n");
        		ret = ENODEV;
        	}
        	else
        	{
        		if(channel->connection_status == NoConnected)
        		{
        			log_info("L2CAP_CREATE_CHANNEL: Not connected\n");
        			ret = ENOERR;
        		}
        		else if(channel->connection_status == WaitingForConnection)
        		{
        			log_info("L2CAP_CREATE_CHANNEL: Waiting for connection\n");
        			ret = EBUSY;
        		}
        		else
        		{
        			reason = packet[0];
        			cyg_mutex_lock(&client_state.global_mutex);
					l2cap_disconnect(channel->channel_cid, reason);
					cyg_mutex_unlock(&client_state.global_mutex);

					cyg_mutex_lock(&channel->ch_mutex);
					while(channel->connection_status == Connected)
						cyg_cond_wait(&channel->ch_cond);
					ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);
					log_info("L2CAP_CREATE_CHANNEL: Disconnected: %u\n",reason);
        		}
        	}
            break;
        case L2CAP_REGISTER_SERVICE:
        	if(channel->channel_type != NotUsed)
        	{
        		log_info("L2CAP_REGISTER_SERVICE: Channel in use\n");
        		ret = EBUSY;
        	}
        	else
        	{
        		cyg_mutex_lock(&client_state.global_mutex);
        		ioctl_l2cap_register_service_t *ioctl = (ioctl_l2cap_register_service_t*)packet;
				psm = ioctl->psm;
				mtu = ioctl->mtu;
				status = l2cap_register_service(l2cap_packet_handler, psm, mtu, LEVEL_0);
				channel->l2cap_psm = psm;
				cyg_mutex_unlock(&client_state.global_mutex);

				if(status)
				{
					log_info("L2CAP_REGISTER_SERVICE: ERROR %d\n",status);
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
					ret = -status;
				}
				else
				{
					log_info("L2CAP_REGISTER_SERVICE: Service created\n");
					channel->channel_type = L2Cap_Service;
					channel->connection_status = WaitingForConnection;
				}
        	}
            break;
        case L2CAP_UNREGISTER_SERVICE:
        	if(channel->channel_type == L2Cap_Service)
        	{
        		log_info("L2CAP_UNREGISTER_SERVICE: Service unregistered\n");
				l2cap_unregister_service(channel->l2cap_psm);

				if(channel->connection_status == NoConnected ||
				   channel->connection_status == WaitingForConnection)
				{
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
				}
				else
				{
					cyg_mutex_lock(&channel->ch_mutex);
					while(channel->connection_status == Connected)
						cyg_cond_signal(&channel->ch_cond);
					ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);

					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
				}
        	}
        	else
        	{
        		log_info("L2CAP_UNREGISTER_SERVICE: Not a l2cap service\n");
        		ret = ENODEV;
        	}
            break;
        case RFCOMM_CREATE_CHANNEL:
        	if(channel->channel_type != NotUsed)
			{
        		log_info("RFCOMM_CREATE_CHANNEL: Channel in use\n");
				ret = EBUSY;
			}
        	else
        	{
        		cyg_mutex_lock(&client_state.global_mutex);
        		ioctl_rfcomm_create_channel_t *ioctl = (ioctl_rfcomm_create_channel_t*)packet;
				memcpy(addr,ioctl->addr,6);
				rfcomm_channel = ioctl->rfcomm_channel_id;
				status = rfcomm_create_channel(&rfcomm_packet_handler, addr, rfcomm_channel, &channel->channel_cid);
				cyg_mutex_unlock(&client_state.global_mutex);
				if (status){
					log_info("RFCOMM_CREATE_CHANNEL: ERROR %d\n",status);
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
					ret = -status;
				} else {
					cyg_mutex_lock(&channel->ch_mutex);

					channel->channel_type = RfCom;
					channel->connection_status = WaitingForConnection;
					channel->rfcomm_id = rfcomm_channel;
					while(channel->connection_status == WaitingForConnection)
						cyg_cond_wait(&channel->ch_cond);
					if(channel->io_status == -4)
						ret = ENOTCONN;
					else
						ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);

					log_info("RFCOMM_CREATE_CHANNEL: Channel created %d\n",rfcomm_channel);
				}
        	}
            break;
        case RFCOMM_CREATE_CHANNEL_WITH_CREDITS:
        	if(channel->channel_type != NotUsed)
			{
        		log_info("RFCOMM_CREATE_CHANNEL_WITH_CREDITS: Channel in use\n");
				ret = EBUSY;
			}
        	else
        	{
        		cyg_mutex_lock(&client_state.global_mutex);
        		ioctl_rfcomm_create_channel_with_credits_t *ioctl =
        				(ioctl_rfcomm_create_channel_with_credits_t*)packet;
        		memcpy(addr,ioctl->addr,6);
				rfcomm_channel = ioctl->rfcomm_channel_id;
				rfcomm_credits = ioctl->credits;
				status = rfcomm_create_channel_with_initial_credits(&rfcomm_packet_handler, addr, rfcomm_channel, rfcomm_credits, &cid );
				cyg_mutex_unlock(&client_state.global_mutex);
				if (status){
					log_info("RFCOMM_CREATE_CHANNEL_WITH_CREDITS: ERROR %d\n",status);
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
					ret = -status;
				} else {
					cyg_mutex_lock(&channel->ch_mutex);
					channel->channel_type = RfCom_WithCredits;;
					channel->connection_status = WaitingForConnection;
					channel->rfcomm_id = rfcomm_channel;
					while(channel->connection_status == WaitingForConnection)
						cyg_cond_signal(&channel->ch_cond);
					ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);
					log_info("RFCOMM_CREATE_CHANNEL_WITH_CREDITS: Channel created %d\n",rfcomm_channel);
				}
        	}
            break;
        case RFCOMM_DISCONNECT:
        	if((channel->channel_type & Type_RfCom) == 0)
        	{
        		log_info("RFCOMM_DISCONNECT: Channel not a rfcomm\n");
        		ret = ENODEV;
        	}
        	else
        	{
        		if(channel->connection_status == NoConnected)
				{
        			log_info("RFCOMM_DISCONNECT: Channel not connected\n");
					ret = ENOERR;
				}
				else if(channel->connection_status == WaitingForConnection)
				{
					log_info("RFCOMM_DISCONNECT: Channel waiting for connection\n");
					ret = EBUSY;
				}
				else
				{
					cyg_mutex_lock(&client_state.global_mutex);
					rfcomm_disconnect(channel->channel_cid);
					cyg_mutex_unlock(&client_state.global_mutex);

					ret = ENOERR;

					cyg_mutex_lock(&channel->ch_mutex);
					while(channel->connection_status != NoConnected)
						cyg_cond_wait(&channel->ch_cond);
					ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);
					log_info("RFCOMM_DISCONNECT: Channel disconnected\n");
				}
        	}
            break;
        case RFCOMM_REGISTER_SERVICE:
        	if(channel->channel_type != NotUsed)
			{
        		log_info("RFCOMM_REGISTER_SERVICE: Channel is in use\n");
				ret = EBUSY;
			}
			else
        	{
				cyg_mutex_lock(&client_state.global_mutex);
				ioctl_rfcomm_register_service_t *ioctl = (ioctl_rfcomm_register_service_t*)packet;
				rfcomm_channel = ioctl->rfcomm_channel_id;
				mtu = ioctl->mtu;
				status = rfcomm_register_service(&rfcomm_packet_handler, rfcomm_channel, mtu);
				cyg_mutex_unlock(&client_state.global_mutex);
				if (status){
					log_info("RFCOMM_REGISTER_SERVICE: ERROR %d\n",status);
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
					ret = -status;
				} else {
					channel->channel_type      = RfCom_Service;
					channel->connection_status = WaitingForConnection;
					channel->rfcomm_id         = rfcomm_channel;
					channel->power_mode        = HCI_POWER_ON;
					log_info("RFCOMM_REGISTER_SERVICE: Service registered\n");
					log_info("RFCOMM_REGISTER_SERVICE: Channel: %d, mtu: %d\n",rfcomm_channel, mtu);
				}
        	}

        	return ret;
            break;
        case RFCOMM_REGISTER_SERVICE_WITH_CREDITS:
        	if(channel->channel_type != NotUsed)
			{
        		log_info("RFCOMM_REGISTER_SERVICE_WITH_CREDITS: Channel is in use\n");
				ret = EBUSY;
			}
			else
        	{
				cyg_mutex_lock(&client_state.global_mutex);
				ioctl_rfcomm_register_service_with_credits_t *ioctl =
						(ioctl_rfcomm_register_service_with_credits_t*)packet;
				rfcomm_channel = ioctl->rfcomm_channel_id;
				mtu = ioctl->mtu;
				rfcomm_credits = ioctl->credits;
				status = rfcomm_register_service_with_initial_credits(&rfcomm_packet_handler, rfcomm_channel, mtu, rfcomm_credits);
				cyg_mutex_unlock(&client_state.global_mutex);
				if (status){
					log_info("RFCOMM_REGISTER_SERVICE_WITH_CREDITS: ERROR %d\n",status);
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
					ret = -status;
				} else {
					channel->channel_type = RfCom_Service_WithCredits;
					channel->connection_status = WaitingForConnection;
					channel->rfcomm_id = rfcomm_channel;
					log_info("RFCOMM_REGISTER_SERVICE_WITH_CREDITS: Service registered\n");
					log_info("RFCOMM_REGISTER_SERVICE_WITH_CREDITS: Channel: %d, mtu: %d, credits: %d\n",rfcomm_channel, mtu, rfcomm_credits);
				}
        	}
            break;
        case RFCOMM_UNREGISTER_SERVICE:
        	if(channel->channel_type & Type_Rfcom_Service)
        	{
        		cyg_mutex_lock(&client_state.global_mutex);
        		rfcomm_unregister_service(channel->rfcomm_id);
        		cyg_mutex_unlock(&client_state.global_mutex);
        		if(channel->connection_status == NoConnected ||
				   channel->connection_status == WaitingForConnection)
				{
        			log_info("RFCOMM_UNREGISTER_SERVICE: Done %d\n",channel->rfcomm_id);
					channel->channel_type = NotUsed;
					channel->connection_status = NoConnected;
				}
				else
				{
					cyg_mutex_lock(&channel->ch_mutex);
					while(channel->connection_status != NoConnected)
						cyg_cond_signal(&channel->ch_cond);
					ret = channel->io_status;
					cyg_mutex_unlock(&channel->ch_mutex);
					log_info("RFCOMM_UNREGISTER_SERVICE: Done %d\n",channel->rfcomm_id);
				}
        	}
        	else
			{
        		log_info("RFCOMM_UNREGISTER_SERVICE: Channel is not a rfcomm\n");
				ret = ENODEV;
			}

            break;
        case BTSTACK_WAIT_FOR_IN_CONNECTION:
        	if(channel->channel_type & (Type_Rfcom_Service | Type_L2Cap_Service))
        	{
        		if(channel->connection_status == WaitingForConnection)
        		{
        			timeout = *((int*)packet);
					log_info("BTSTACK_WAIT_FOR_IN_CONNECTION: Timeout %d\n",timeout);

					cyg_mutex_lock(&channel->ch_mutex);
					if(timeout == 0)
					{
						while(channel->connection_status != Connected)
							cyg_cond_wait(&channel->ch_cond);
					}
					else
					{
						cyg_uint32 cur = cyg_current_time();
						cyg_cond_timed_wait(&channel->ch_cond,cur + timeout);
						if(channel->connection_status != Connected)
							ret = ETIME;
					}
					cyg_mutex_unlock(&channel->ch_mutex);

        		}
        		else if(channel->connection_status == Connected)
        		{
        			log_info("BTSTACK_WAIT_FOR_IN_CONNECTION: Connected\n");
        		}
        		else
        		{
        			log_info("BTSTACK_WAIT_FOR_IN_CONNECTION: Not waiting for connection\n");
        			ret = ENODEV;
        		}
        	}
        	else
        	{
        		log_info("BTSTACK_WAIT_FOR_IN_CONNECTION: Not a service channel\n");
        		ret = ENODEV;
        	}
        	break;
        case RFCOMM_GRANT_CREDITS:
        	if(channel->channel_type == RfCom_Service_WithCredits)
        	{
        		cyg_mutex_lock(&channel->ch_mutex);
				rfcomm_credits = packet[0];
				log_info("RFCOMM_GRANT_CREDITS: %d\n", rfcomm_credits);
				rfcomm_grant_credits(channel->channel_cid, rfcomm_credits);
				cyg_mutex_unlock(&channel->ch_mutex);
        	}
        	else
			{
        		log_info("RFCOMM_GRANT_CREDITS: Not a rfcomm service with credits\n");
				ret = ENODEV;
			}
        	return ret;
            break;
        case SDP_REGISTER_SERVICE_RECORD:
        	log_info("SDP_REGISTER_SERVICE_RECORD size %u\n", size);
        	cyg_mutex_lock(&client_state.global_mutex);
            service_record_handle = daemon_sdp_create_and_register_service(&packet[0]);
            if (service_record_handle){
            	channel->sdp_record = (service_record_item_t*)service_record_handle;
                //daemon_add_client_sdp_service_record_handle(connection, service_record_handle);
                //sdp_emit_service_registered(connection, service_record_handle, 0);
            	ret = ENOERR;
            } else {
            	ret = -1; // nevim co lepsiho
            }
            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case SDP_UNREGISTER_SERVICE_RECORD:
            service_record_handle = channel->sdp_record;
            log_info("SDP_UNREGISTER_SERVICE_RECORD handle 0x%x ", service_record_handle);
            cyg_mutex_lock(&client_state.global_mutex);
            data = sdp_get_record_for_handle(service_record_handle);
            if(data != NULL)
				free(data);
            sdp_unregister_service(channel->sdp_record);

            channel->sdp_record = NULL;
            ret = ENOERR;

            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case SDP_CLIENT_QUERY_RFCOMM_SERVICES:
        	cyg_mutex_lock(&client_state.global_mutex);
        	ioctl_sdp_client_query_rfcomm_services_t *ioctl =
        			(ioctl_sdp_client_query_rfcomm_services_t*)packet;
            memcpy(addr,ioctl->addr,6);

            clents_remove_and_free_remote_device_service_list();
            uuid = ioctl->uuid;
            status = sdp_client_query_rfcomm_channel_and_name_for_uuid(
            		&handle_sdp_rfcomm_service_result, addr, uuid);

            if(status)
            {
            	log_info("SDP_CLIENT_QUERY_RFCOMM_SERVICES ERROR %d\n", status);
            	ret = -status;
            }
            else
            {
            	cyg_cond_wait(&client_state.global_cond);
            	ret = client_state.global_ioctl_status;
            	log_info("SDP_CLIENT_QUERY_RFCOMM_SERVICES done\n");
            }
            cyg_mutex_unlock(&client_state.global_mutex);
            break;
        case BTSTACK_GET_RFCOMM_SERVICES:
        	log_info("BTSTACK_GET_RFCOMM_SERVICES\n");
			btstack_linked_list_t** list = (btstack_linked_list_t**)packet;
			*list = (btstack_linked_list_t*) &client->remote_device_service;
			break;
        case SDP_CLIENT_QUERY_SERVICES:
        	log_info("SDP_CLIENT_QUERY_SERVICES\n");
        	// TODO: neni komplet a asi ani nefici

            //reverse_bd_addr(&packet[0], addr);
            //sdp_client_query_connection = connection;

            serviceSearchPatternLen = de_get_len(&packet[6]);
            memcpy(client->serviceSearchPattern, &packet[6], serviceSearchPatternLen);

            attributeIDListLen = de_get_len(&packet[6+serviceSearchPatternLen]);
            memcpy(client->attributeIDList, &packet[6+serviceSearchPatternLen], attributeIDListLen);

            status = sdp_client_query(&handle_sdp_client_query_result, addr,
            		(uint8_t*)client->serviceSearchPattern, (uint8_t*)client->attributeIDList);
            if(status)
			{
				client->global_ioctl_status = -status;
			}
			else
			{
				client->global_ioctl_status = ENOERR;
			}
            break;
        default:
            log_error("Error: command %u not implemented:", 0/*READ_CMD_OCF(packet)*/);
            ret = ENOSUPP;
            break;
    }

    return ret;
}

void daemon_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    //uint16_t cid;
    //uint8_t  rfcomm_channel_nr;
    int i;
    uint8_t event;
    bd_addr_t addr;
    //channel_state_t *connection;


    switch (packet_type) {
        case HCI_EVENT_PACKET:
            //deamon_status_event_handler(packet, size);
            event = hci_event_packet_get_type(packet);

            switch (event)
            {
            case HCI_EVENT_PIN_CODE_REQUEST:
			   // pre-ssp: inform about pin code request
               log_info("Pin code request - using '0000'\n");
			   hci_event_pin_code_request_get_bd_addr(packet, addr);
			   hci_send_cmd(&hci_pin_code_request_reply, &addr, 4, "0000");
			   break;

            case HCI_EVENT_USER_CONFIRMATION_REQUEST:
			   // ssp: inform about user confirmation request
               log_info("SSP User Confirmation Request with numeric value '%06d""'\n", little_endian_read_32(packet, 8));
               log_info("SSP User Confirmation Auto accept\n");
			   break;

			case BTSTACK_EVENT_STATE:
				// BTstack activated, get started
				if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
				{
					if(client_state.global_enable)
					{
						log_info("BTSTACK_EVENT_STATE: HCI_STATE_WORKING\n");
						cyg_mutex_lock(&client_state.global_mutex);
						client_state.global_ioctl_status = ENOERR;
						cyg_cond_signal(&client_state.global_cond);
						cyg_mutex_unlock(&client_state.global_mutex);
					}
				}
				if (btstack_event_state_get_state(packet) == HCI_STATE_OFF)
				{
					if(!client_state.global_enable)
					{
						log_info("BTSTACK_EVENT_STATE: HCI_STATE_OFF\n");
						if(client_state.initialised)
						{
							cyg_mutex_lock(&client_state.global_mutex);
							client_state.global_ioctl_status = ENOERR;
							cyg_cond_signal(&client_state.global_cond);
							cyg_mutex_unlock(&client_state.global_mutex);
						}
						else
						{
							client_state.initialised = true;
						}
					}
				}
				break;

			case HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS:
				// ACL buffer freed...
				//daemon_retry_parked();
				// no need to tell clients
				return;
				break;

			case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE:
#if 1
				log_info("HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE\n");;
				reverse_bd_addr(&packet[3], addr);
				// TODO: check
				// fix for invalid remote names - terminate on 0xff
			/*	for (i=0; i<248;i++)
				{
					if (packet[9+i] == 0xff){
						packet[9+i] = 0;
						break;
					}
				}
				packet[9+248] = 0;*/
				if (!packet[2])
				{
					clents_add_remote_device_name(addr, (char*)&packet[9], strlen((char*)&packet[9]));
				}
				/*else
				{
					diag_printf("NOK\n");
				}*/

				if(clents_remote_device_continue_name_requests() == 0)
				{
					client_state.InqueryInProgress = false;
					log_info("INIP HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE\n");
				}
#endif
				break;

			case HCI_EVENT_INQUIRY_RESULT:
			case HCI_EVENT_INQUIRY_RESULT_WITH_RSSI:
			case HCI_EVENT_EXTENDED_INQUIRY_RESPONSE:
			{
#if 1
				// then send cached remote names
				for (i=0; i<packet[2];i++)
				{
					bt_device_t * device ;
					btstack_linked_list_bt_device_t *item;
					reverse_bd_addr(&packet[3 + i * 6], addr);
					item = clents_get_remote_device_for_address(addr);
					if(item != NULL) continue;   // already in our list

					item = clents_create_add_remote_device_to_list();
					device = &item->value;

					memcpy(device->address, addr, 6);
					device->pageScanRepetitionMode = packet [3 + packet[2]*(6)         + i*1];

					if (event == HCI_EVENT_INQUIRY_RESULT)
					{
						device->classOfDevice = little_endian_read_24(packet, 3 + packet[2]*(6+1+1+1)   + i*3);
						device->clockOffset =   little_endian_read_16(packet, 3 + packet[2]*(6+1+1+1+3) + i*2) & 0x7fff;
						device->rssi  = 0;
					} else
					{
						device->classOfDevice = little_endian_read_24(packet, 3 + packet[2]*(6+1+1)     + i*3);
						device->clockOffset =   little_endian_read_16(packet, 3 + packet[2]*(6+1+1+3)   + i*2) & 0x7fff;
						device->rssi  =                               packet [3 + packet[2]*(6+1+1+3+2) + i*1];
					}

					device->state = REMOTE_NAME_REQUEST;

					if (event == HCI_EVENT_EXTENDED_INQUIRY_RESPONSE)
					{
						uint8_t * eir_data = &packet[3 + (6+1+1+3+2) + 1];
						ad_context_t context;
						for (ad_iterator_init(&context, 240, eir_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context))
						{
							uint8_t data_type    = ad_iterator_get_data_type(&context);
							uint8_t data_size    = ad_iterator_get_data_len(&context);
							const uint8_t * data = ad_iterator_get_data(&context);
							// Prefer Complete Local Name over Shortend Local Name
							switch (data_type)
							{
								case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
									if (device->state == REMOTE_NAME_FETCHED) break;
								case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
									device->state = REMOTE_NAME_FETCHED;
									clents_add_remote_device_name(addr,data,data_size);
									break;
								default:
									break;
							}
						}
					}


					log_info("Device found: %s with COD: 0x%06x, pageScan %d, clock offset 0x%04x", bd_addr_to_str(addr),
							(unsigned int) device->classOfDevice, device->pageScanRepetitionMode, device->clockOffset);
					if (event >= HCI_EVENT_INQUIRY_RESULT_WITH_RSSI)
					{
						log_info(", rssi 0x%02x", device->rssi);
					}
					if (device->state == REMOTE_NAME_FETCHED)
					{
						log_info(", name '%s'", device->name);
					}
					log_info("\n");
				}

#endif
			}
				break;
				
			case HCI_EVENT_INQUIRY_COMPLETE:
				log_info("HCI_EVENT_INQUIRY_COMPLETE\n");
				if(clents_remote_device_continue_name_requests() == 0)
				{
					client_state.InqueryInProgress = false;
					log_info("INIP HCI_EVENT_INQUIRY_COMPLETE\n");
				}
				break;
			default:
				break;
            }
            break;
        default:
            break;
    }

}

static void bt_control_init(const void *config)
{

}

static int bt_control_on(void)
{
	CYGHWR_IO_SET_PIN_NSHUTD;
	//cyg_thread_delay(300);
	return 0;
}

static int bt_control_off(void)
{
	CYGHWR_IO_CLEAR_PIN_NSHUTD;
	return 0;
}

static int bt_control_slepp(void)
{
	return 0;
}

static int bt_control_wake(void)
{
	return 0;
}

static void bt_register_for_power_notifications(void (*cb)(POWER_NOTIFICATION_t event))
{
}


btstack_control_t hardware_control =
{
	.init   = &bt_control_init,
	.on     = &bt_control_on,
	.off    = &bt_control_off,
	.sleep  = &bt_control_slepp,
	.wake   = &bt_control_wake,
	.register_for_power_notifications = &bt_register_for_power_notifications

};

 void bt_thread(cyg_addrword_t data)
 {
	 // Control data soruce
	int i;
	btstack_data_source_t ds;

	cyg_io_handle_t *handle;
	hci_transport_config_uart_t config = {
		HCI_TRANSPORT_CONFIG_UART,
		115200,
		1000000,  // main baudrate
		1,        // flow control
		"/dev/ser3",
	};
	ehci_serial_channel *chan;
	cyg_devtab_entry_t *t;


	if(cyg_io_lookup("/dev/ehci",(void**)&handle) != ENOERR)
	{
		CYG_FAIL("Open ehci\n");
	}

	t = (cyg_devtab_entry_t*)handle;
	chan = (ehci_serial_channel*)t->priv;
	chan->callbacks->ehci_init(chan);

	/// GET STARTED with BTstack ///
	btstack_memory_init();
	btstack_run_loop_init(btstack_run_loop_ecos_get_instance());

	// init HCI
	//const btstack_uart_block_t * uart_driver = btstack_uart_block_ecos_instance();
	const hci_transport_t * transport = hci_transport_h4_instance((btstack_uart_block_t*)chan);

	btstack_chipset_cc256x_set_power(5);
	hci_init(transport, (void*) &config);
#if BLUETOOTH_DEAMON_LINK_KEY_MEMOORY == 1
	hci_set_link_key_db(btstack_link_key_db_memory_instance());
#else
	hci_set_link_key_db(btstack_link_key_db_memory_instance_save());
#endif
	hci_set_chipset(btstack_chipset_cc256x_instance());;
	hci_set_control(&hardware_control);

	// register for HCI events
	hci_event_callback_registration.callback = &daemon_packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

	/*cyg_mutex_init(&client_state.global_mutex);

	cyg_cond_init(&client_state.global_cond,&client_state.global_mutex);*/

	for(i = 0; i < client_state.num_channels; i++)
	{
		client_state.channels_list[i].client = &client_state;
		queue_int(&client_state.channels_list[i].read_queue);

		cyg_mutex_init(&client_state.channels_list[i].read_queue.mutex);
		cyg_cond_init(&client_state.channels_list[i].read_queue.cond,
				&client_state.channels_list[i].read_queue.mutex);

		cyg_mutex_init(&client_state.channels_list[i].ch_mutex);
		cyg_cond_init(&client_state.channels_list[i].ch_cond, &client_state.channels_list[i].ch_mutex);
	}

	/////
	/*btstack_run_loop_set_data_source_fd(&ds, 0);
	btstack_run_loop_set_data_source_handler(&ds, &btstack_command_handler_on_bt_thread);
	btstack_run_loop_enable_data_source_callbacks(&ds, DATA_SOURCE_CALLBACK_READ);
	btstack_run_loop_add_data_source(&ds);*/
	///

	client_state.thread_handle = cyg_thread_self();

	l2cap_init();

	rfcomm_init();

	sdp_init();

	gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
	gap_set_local_name("KM-8");
	gap_set_class_of_device(0xff);

	hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

	// turn on!
	hci_power_control(HCI_POWER_OFF);

	cyg_mutex_lock(&client_state.global_mutex);
	client_state.process_started = true;
	cyg_cond_signal(&client_state.global_cond);
	cyg_mutex_unlock(&client_state.global_mutex);

	btstack_run_loop_execute();

	CYG_FAIL("BT thread unexpected end");
 }

 void bt_Start()
 {

	cyg_mutex_init(&client_state.global_mutex);

	cyg_cond_init(&client_state.global_cond,&client_state.global_mutex);

	cyg_thread_create(
		4,
		bt_thread,
		(cyg_addrword_t) 0,
		"BT",
		bt_thrad_stack,
		sizeof(bt_thrad_stack),
		&bt_thread_handle,
		&bt_thread_data
	);
	//client_state.initialised = true;
	cyg_thread_resume(bt_thread_handle);

	cyg_mutex_lock(&client_state.global_mutex);
	while(client_state.process_started == false)
		cyg_cond_wait(&client_state.global_cond);
	cyg_mutex_unlock(&client_state.global_mutex);
 }



