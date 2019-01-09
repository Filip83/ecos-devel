/*
 * fileio_connction.h
 *
 *  Created on: 14. 4. 2018
 *      Author: filip
 */

#ifndef SRC_DEAMON_FILEIO_CONNECTION_H_
#define SRC_DEAMON_FILEIO_CONNECTION_H_

#include "btlib/btstack_run_loop.h"

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#include <pkgconf/bluetooth_lib.h>

#include "btlib/btstack.h"
#include <cyg/io/io.h>
#include <cyg/kernel/kapi.h>

// Configuration
#define BLUETOOTH_DEAMON_USE_BUFFERED_READ_QUEUE			CYGOPT_BLUETOOTH_ECOS_DEAMON_USE_RAD_QUEUE
#define BLUETOOTH_DEAMON_BUFFER_SIZE						CYGINT_BLUETOOTH_ECOS_DEAMON_RAD_QUEUE_SIZE
#define BLUETOOTH_EAMON_CHANNELS_COUNT						CYGINT_BLUETOOTH_ECOS_DEAMON_CHANNELS_COUNT
#define BLUETOOTH_DEAMON_LINK_KEY_MEMOORY					CYGINT_BLUETOOTH_ECOS_DEAMON_STORE_LINK_KEY_TO_MEMORY

typedef enum bt_inquery_state_s
{
	REMOTE_NAME_REQUEST, REMOTE_NAME_INQUIRED, REMOTE_NAME_FETCHED
}bt_inquery_state_t;

typedef struct bt_device_s {
    bd_addr_t  		address;
    char            *name;
    uint16_t   		clockOffset;
    uint32_t   		classOfDevice;
    uint8_t    		pageScanRepetitionMode;
    uint8_t    		rssi;
    bt_inquery_state_t state;
}bt_device_t;

typedef struct btstack_linked_list_bt_device {
    btstack_linked_item_t   item;
    bt_device_t             value;
} btstack_linked_list_bt_device_t;

typedef struct sdp_client_query_rfcomm_service_event_s
{
	uint8_t		port;
	char        *name;
}sdp_client_query_rfcomm_service_t;

typedef struct btstack_linked_list_bt_service {
    btstack_linked_item_t   			item;
    sdp_client_query_rfcomm_service_t   value;
} btstack_linked_list_bt_service_t;

typedef enum
{
	NotUsed = 0, L2Cap = 1, L2Cap_Service = 2, RfCom = 4,
	RfCom_WithCredits = 8, RfCom_Service = 16,
	RfCom_Service_WithCredits = 32, Type_L2Cap = 3,
	Type_RfCom = 60, Type_L2Cap_Service = 2, Type_Rfcom_Service = 48
}e_channel_type_t;

typedef enum
{
	NoConnected, WaitingForConnection, Connected
}e_connection_status;

typedef struct io_queue_buf_s
{
	uint8_t buffer[BLUETOOTH_DEAMON_BUFFER_SIZE];
	int     size;
	int     front;
	int     count;
	int     status;
}io_queue_buf_t;

typedef struct io_queue_s
{
	io_queue_buf_t read_queue;
	uint8_t 	   *data;
	int      	   len;
	int      		copleted_len;
	cyg_mutex_t 	data_lock;
	cyg_sem_t       ready_for_input_data;
	cyg_sem_t       cmplet_sem;
	cyg_mutex_t     cond_mutex;
	cyg_cond_t      cond;
}io_queue_t;

typedef struct {
    void         		*client;

    uint32_t     		io_ch_id;
    int                 io_status;
    cyg_mutex_t  		ch_mutex;
    cyg_cond_t    		ch_cond;

    e_channel_type_t    channel_type;
    e_connection_status connection_status;
    uint16_t           	channel_cid;
    uint16_t           	rfcomm_id;
    uint16_t           	l2cap_psm;

    service_record_item_t *sdp_record;
    //btstack_linked_list_gatt_client_helper_t * gatt_con;
    // power mode
    HCI_POWER_MODE power_mode;

    // discoverable
    uint8_t        discoverable;



   /* io_queue_t   q_write;
    io_queue_t   q_read;*/

    io_queue_t   queue_io;
} channel_state_t;


// bluetooth deamon data structrue
typedef struct
{
	// power mode
	cyg_bool        initialised;
	HCI_POWER_MODE  power_mode;
	uint32_t        num_channels;
	channel_state_t *channels_list;

	cyg_mutex_t 	global_mutex;
	cyg_cond_t      global_cond;
	cyg_uint32      global_ioctl_status;

	uint32_t        global_enable;
	int             power_management_sleep;
	bool            InqueryInProgress;

	uint8_t 		serviceSearchPattern[200];
	uint8_t 		attributeIDList[50];
	uint8_t   		attribute_value[1000];
	int 			attribute_value_buffer_size;// = 1000;

	uint8_t               devices_count;
	btstack_linked_list_t remote_devices;
	uint8_t               services_count;
	btstack_linked_list_t remote_device_service;

}client_state_t;

typedef struct ioctl_l2cap_create_channel_s
{
	bd_addr_t addr;
	uint16_t  psm;
	uint16_t  mtu;
}ioctl_l2cap_create_channel_t;

typedef struct ioctl_l2cap_register_service_s
{
	uint16_t  psm;
	uint16_t  mtu;
}ioctl_l2cap_register_service_t;

typedef struct ioctl_rfcomm_create_channel_s
{
	bd_addr_t addr;
	uint8_t   rfcomm_channel_id;
}ioctl_rfcomm_create_channel_t;

typedef struct ioctl_rfcomm_create_channel_with_credits_s
{
	bd_addr_t addr;
	uint8_t   rfcomm_channel_id;
	uint8_t   credits;
}ioctl_rfcomm_create_channel_with_credits_t;

typedef struct ioctl_rfcomm_register_service_s
{
	uint8_t   rfcomm_channel_id;
	uint16_t  mtu;  // 0xffff mtu limited by l2cap
}ioctl_rfcomm_register_service_t;

typedef struct ioctl_rfcomm_register_service_with_credits_s
{
	uint8_t   rfcomm_channel_id;
	uint16_t  mtu;  // 0xffff mtu limited by l2cap
	uint8_t   credits;
}ioctl_rfcomm_register_service_with_credits_t;

typedef struct ioctl_sdp_client_query_rfcomm_services_s
{
	bd_addr_t addr;
	uint16_t  uuid;
}ioctl_sdp_client_query_rfcomm_services_t;

bool queue_int(io_queue_buf_t *queue);
bool queue_empty(io_queue_buf_t *queue);
bool queue_full(io_queue_buf_t *queue);
bool queue_add(io_queue_buf_t* queue, uint8_t data);
bool queue_remove(io_queue_buf_t* queue, uint8_t *data);
bool queue_werite(io_queue_buf_t* queue, uint8_t *data, int len);
int queue_read(io_queue_buf_t* queue, uint8_t *data, int len);

#if defined __cplusplus
}
#endif
#endif /* SRC_DEAMON_FILEIO_CONNECTION_H_ */
