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
 *  btstack.h
 *  Convenience header to include all public APIs
 */


#ifndef __BTSTACK_H
#define __BTSTACK_H

#include "btlib/btstack_config.h"

#include "btlib/bluetooth.h"
#include "btlib/bluetooth_data_types.h"
#include "btlib/bluetooth_gatt.h"
#include "btlib/bluetooth_sdp.h"
#include "btlib/bluetooth_company_id.h"
#include "btlib/ad_parser.h"
#include "btlib/btstack_control.h"
#include "btlib/btstack_debug.h"
#include "btlib/btstack_event.h"
#include "btlib/btstack_defines.h"
#include "btlib/btstack_linked_list.h"
#include "btlib/btstack_memory.h"
#include "btlib/btstack_memory_pool.h"
#include "btlib/btstack_run_loop.h"
#include "btlib/btstack_util.h"
#include "btlib/gap.h"
#include "btlib/hci.h"
#include "btlib/hci_cmd.h"
#include "btlib/hci_dump.h"
#include "btlib/hci_transport.h"
#include "btlib/l2cap.h"
#include "btlib/l2cap_signaling.h"

#ifdef ENABLE_BLE
#include "btlib/ble/ancs_client.h"
#include "btlib/ble/att_db.h"
#include "btlib/ble/att_db_util.h"
#include "btlib/ble/att_dispatch.h"
#include "btlib/ble/att_server.h"
#include "btlib/ble/gatt_client.h"
#include "btlib/ble/le_device_db.h"
#include "btlib/ble/sm.h"
#endif

// #ifdef HAVE_CLASSIC
#include "btlib/classic/bnep.h"
#include "btlib/classic/btstack_link_key_db.h"
#include "btlib/classic/device_id_server.h"
#include "btlib/classic/hfp.h"
#include "btlib/classic/hfp_ag.h"
#include "btlib/classic/hfp_hf.h"
#include "btlib/classic/hsp_ag.h"
#include "btlib/classic/hsp_hs.h"
#include "btlib/classic/pan.h"
#include "btlib/classic/rfcomm.h"
#include "btlib/classic/sdp_client.h"
#include "btlib/classic/sdp_client_rfcomm.h"
#include "btlib/classic/sdp_server.h"
#include "btlib/classic/sdp_util.h"
#include "btlib/classic/spp_server.h"
// #endif

#endif  // __BTSTACK_H
 
