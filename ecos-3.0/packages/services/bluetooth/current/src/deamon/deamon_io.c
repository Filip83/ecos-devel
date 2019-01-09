/*
 * fileio_connection.c
 *
 *  Created on: 14. 4. 2018
 *      Author: filip
 */

#include "btlib/deamon/deamon_io.h"

#include <cyg/io/io.h>
#include <cyg/io/devtab.h>
#include <cyg/io/ehci_serial.h>
#include <cyg/kernel/kapi.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include "btlib/hci.h"
#include "btlib/btstack_debug.h"

#include "btlib/btstack_config.h"

#include "btlib/btstack.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>

#define MAX_PENDING_CONNECTIONS 10


// Thread data
cyg_handle_t bt_thread_handle;
cyg_thread   bt_thread_data;
cyg_uint8    bt_thrad_stack[CYGNUM_HAL_STACK_SIZE_TYPICAL];

// BT client data
extern client_state_t client_state;
extern channel_state_t l_channels[BLUETOOTH_EAMON_CHANNELS_COUNT];

// IO functions prototypes
Cyg_ErrNo cyg_bt_write(cyg_io_handle_t handle, const void *buf, cyg_uint32 *len);
Cyg_ErrNo cyg_bt_read(cyg_io_handle_t handle, void *buf, cyg_uint32 *len);
cyg_bool  cyg_bt_select(cyg_io_handle_t handle, cyg_uint32 which, CYG_ADDRWORD info);
Cyg_ErrNo cyg_bt_get_config(cyg_io_handle_t handle, cyg_uint32 key, void *buf, cyg_uint32 *len);
Cyg_ErrNo cyg_bt_set_config(cyg_io_handle_t handle, cyg_uint32 key, const void *buf, cyg_uint32 *len);

bool  cyg_bt_inti(struct cyg_devtab_entry *tab);
Cyg_ErrNo cyg_bt_lookup(struct cyg_devtab_entry **tab, struct cyg_devtab_entry *sub_tab,
												const char *name);

DEVIO_TABLE(bt_funs,
		    &cyg_bt_write,
			&cyg_bt_read,
			NULL,
			&cyg_bt_get_config,
			&cyg_bt_set_config);

#if BLUETOOTH_EAMON_CHANNELS_COUNT > 0
DEVTAB_ENTRY(bt_devtab,
		    "/dev/bt0",
			NULL,
			&bt_funs,
			&cyg_bt_inti,
			&cyg_bt_lookup,
			&l_channels[0]);
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 1

DEVTAB_ENTRY(bt_devtab1,
		    "/dev/bt1",
			NULL,
			&bt_funs,
			&cyg_bt_inti,
			&cyg_bt_lookup,
			&l_channels[1]);
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 2

DEVTAB_ENTRY(bt_devtab2,
		    "/dev/bt2",
			NULL,
			&bt_funs,
			&cyg_bt_inti,
			&cyg_bt_lookup,
			&l_channels[2]);
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 3

DEVTAB_ENTRY(bt_devtab3,
		    "/dev/bt3",
			NULL,
			&bt_funs,
			&cyg_bt_inti,
			&cyg_bt_lookup,
			&l_channels[3]);
#endif
#if BLUETOOTH_EAMON_CHANNELS_COUNT > 4 || BLUETOOTH_EAMON_CHANNELS_COUNT < 1
#error "Wrong number of bluetooth deamon channels count!"
#endif

externC void bt_Start(void);

/*
 * create channel - vrati chýbu pokud se nepovede pøipojení v nìjakém intervalu
 * pokud se povede a je zapotøebí párování vrátí odpovidající kód mìla by být
 * funkce pro naètení typu párování a jeho parametrù následnì funkce, která
 * provode párování.
 *
 * regiser service - vytvoøí službu a v pøípadì že je vše OK ukonèíse. Mìly by
 * být funkce která kontroluje stav pøipojení kanálu s timeoutem. Ta vraci uspìch
 * cybu nebo požadavek na párovná, které se porovede stejnì jako v pøedchozím
 * kroku.
 */



bool  cyg_bt_inti(struct cyg_devtab_entry *tab)
{
	if(client_state.process_started)
	{
		CYGHWR_IO_CLEAR_PIN_NSHUTD;
		bt_Start();
	}

	return true;
}

Cyg_ErrNo cyg_bt_lookup(struct cyg_devtab_entry **tab, struct cyg_devtab_entry *sub_tab,
												const char *name)
{
	channel_state_t * channel = (channel_state_t*)(*tab)->priv;
	if(!client_state.initialised)
	{
		CYGHWR_IO_CLEAR_PIN_NSHUTD;
		bt_Start();
	}

	return ENOERR;
}


Cyg_ErrNo cyg_bt_write(cyg_io_handle_t handle, const void *buf, cyg_uint32 *len)
{
	cyg_uint32 ret = ENOERR;
	cyg_devtab_entry_t *dev_tab = (cyg_devtab_entry_t*)handle;
	channel_state_t   *channel = (channel_state_t*)dev_tab->priv;
	client_state_t     *client = (client_state_t*)channel->client;

	cyg_uint32 length    = *len;
	cyg_uint8 *buf_start = (cyg_uint8*)buf;
	cyg_uint8 *buf_pos   = buf_start;

	cyg_uint32 payload_size;

	// TODO: dat neco lepsiho
	if(length != 0)
	{
		if(channel->connection_status == Connected)
		{
			if(channel->channel_type & Type_RfCom)
				payload_size = rfcomm_get_max_frame_size(channel->channel_cid)/2;
			else
				payload_size = l2cap_get_remote_mtu_for_local_cid(channel->channel_cid)/2;

			do
			{

				channel->write_queue.data = buf_pos;
				if(length > payload_size)
				{
					channel->write_queue.len = payload_size;
					buf_pos += payload_size;
					length  -= payload_size;
				}
				else
				{
					channel->write_queue.len  = length;
					buf_pos += length;
					length  -= length;
				}

				cyg_mutex_lock(&channel->ch_mutex);
				// Global operation lock
				cyg_mutex_lock(&client_state.global_mutex);
				channel->write_queue.complete = false;
				if((channel->channel_type & Type_RfCom))
				{
					rfcomm_request_can_send_now_event(channel->channel_cid);
				}

				if((channel->channel_type & Type_L2Cap))
				{
					l2cap_request_can_send_now_event(channel->channel_cid);
				}
				while(!channel->write_queue.complete)
					cyg_cond_wait(&channel->ch_cond);
				cyg_mutex_unlock(&client_state.global_mutex);


				//cyg_cond_wait(&channel->ch_cond);
				cyg_mutex_unlock(&channel->ch_mutex);
				if(channel->connection_status != Connected)
				{
					channel->write_queue.len = 0;
					*len = 0;
					ret = ENOTCONN;
					break;
				}

				cyg_thread_delay(10);

			}while(length);


		}
		else
		{
			channel->write_queue.len = 0;
			*len = 0;
			ret = ENOTCONN;
		}
	}

	return ret;
}

Cyg_ErrNo cyg_bt_read(cyg_io_handle_t handle, void *buf, cyg_uint32 *len)
{
	cyg_uint32 ret = ENOERR;
	cyg_devtab_entry_t *dev_tab = (cyg_devtab_entry_t*)handle;
	channel_state_t   *channel = (channel_state_t*)dev_tab->priv;
	client_state_t     *client = (client_state_t*)channel->client;

	int length = *len;
	int completed_length = 0;
	cyg_uint8 *dest = (cyg_uint8*)buf;

	if(*len != 0)
	{
		if(channel->connection_status == Connected)
		{
			//cyg_mutex_lock(&channel->ch_mutex);
			do
			{
				cyg_mutex_lock(&channel->read_queue.mutex);
				while((completed_length = queue_read(&channel->read_queue,dest, length)) == 0)
				{
					cyg_cond_wait(&channel->read_queue.cond);
					if(channel->connection_status != Connected)
						break;
				}

				length -= completed_length;
				dest   += completed_length;
				cyg_mutex_unlock(&channel->read_queue.mutex);

				if(channel->connection_status != Connected)
				{
					// queue flush
					diag_printf("No con\n");
					queue_int(&channel->read_queue);
					ret = ENOTCONN;
					*len = completed_length;
					break;
				}

				if(channel->read_queue.status)
				{
					// queue flush overflow
					diag_printf("queue full\n");
					queue_int(&channel->read_queue);
					ret = ENOTCONN;
					*len = *len -length;//completed_length;
					break;
				}

			}while(length);
			//cyg_mutex_unlock(&channel->ch_mutex);
		}
		else
		{
			*len = 0;
			//channel->len = 0;
			ret = ENOTCONN;
		}
	}
	return ret;
}

Cyg_ErrNo cyg_bt_set_config(cyg_io_handle_t handle, cyg_uint32 key, const void *buf, cyg_uint32 *len)
{
	cyg_devtab_entry_t *dev_tab = (cyg_devtab_entry_t*)handle;
	channel_state_t   *channel = (channel_state_t*)dev_tab->priv;

	return btstack_command_handler(channel,key, (uint8_t*)buf, *len);
}

Cyg_ErrNo cyg_bt_get_config(cyg_io_handle_t handle, cyg_uint32 key,void *buf, cyg_uint32 *len)
{
	cyg_devtab_entry_t *dev_tab = (cyg_devtab_entry_t*)handle;
	channel_state_t   *channel = (channel_state_t*)dev_tab->priv;

	return btstack_command_handler(channel,key, (uint8_t*)buf, *len);
}

// Queue used to buffer received data
bool queue_int(io_queue_buf_t *queue)
{
	queue->size = sizeof(queue->buffer);
	queue->front = 0;
	queue->count = 0;
	queue->status = 0;
}

bool queue_empty(io_queue_buf_t *queue)
{
	return queue->count == 0;
}

bool queue_full(io_queue_buf_t *queue)
{
	return queue->count == queue->size;
}


bool queue_add(io_queue_buf_t* queue, uint8_t data)
{
	if(queue_full(queue))
	{
		return false;
	}
	else
	{
		// find index where insert will occur
		int end = (queue->front + queue->count) % queue->size;
		queue->buffer[end] = data;
		queue->count++;
		return true;
	}
}

bool queue_remove(io_queue_buf_t* queue, uint8_t *data)
{
	if(queue_empty(queue))
	{
		return false;
	}
	else
	{
		*data = queue->buffer[queue->front];

		queue->front = queue->front == queue->size ? 0 : queue->front + 1;
		queue->count--;
		return true;
	}
}

bool queue_werite(io_queue_buf_t* queue, uint8_t *data, int len)
{
	int i = 0;
	while(queue_full(queue) == false && i < len)
	{
		int end = (queue->front + queue->count) % queue->size;
		queue->buffer[end] = data[i++];
		queue->count++;
	}

	if(i == len)
	{
		queue->status = 0;
		return true;
	}
	else
	{
		queue->status = 1;
		return false;
	}
}

int queue_read(io_queue_buf_t* queue, uint8_t *data, int len)
{
	int i = 0;
	while(queue_empty(queue) == false && i < len)
	{
		data[i++] = queue->buffer[queue->front];

		queue->front = queue->front == queue->size ? 0 : queue->front + 1;
		queue->count--;
	}

	return i;
}
////////////////////////////////////////////////////////////////////////////////

