//
// btstack_config.h for msp-exp430f5438-cc2564b port
//

#ifndef __BTSTACK_CONFIG
#define __BTSTACK_CONFIG

#include <pkgconf/bluetooth_lib.h>

// Port related features
#define HAVE_INIT_SCRIPT
#define HAVE_EMBEDDED_TICK
#define HAVE_MALLOC
#define HAVE_ECOS

#if 0
// BTstack features that can be enabled
//#define ENABLE_BLE
#define ENABLE_CLASSIC
#define ENABLE_LOG_INTO_HCI_DUMP
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
//#define ENABLE_LOG_DEBUG
//#define ENABLE_EHCILL


// BTstack configuration. buffers, sizes, ...
#define HCI_ACL_PAYLOAD_SIZE                        52
#define MAX_NR_BNEP_CHANNELS                        MAX_SPP_CONNECTIONS
#define MAX_NR_BNEP_SERVICES                        1
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES   10
#define MAX_NR_GATT_CLIENTS                         0
#define MAX_NR_GATT_SUBCLIENTS                      0
#define MAX_NR_HCI_CONNECTIONS                      MAX_SPP_CONNECTIONS
#define MAX_NR_HFP_CONNECTIONS                      0
#define MAX_NR_L2CAP_CHANNELS                       (1+MAX_SPP_CONNECTIONS)
#define MAX_NR_L2CAP_SERVICES                       2
#define MAX_NR_RFCOMM_CHANNELS                      MAX_SPP_CONNECTIONS
#define MAX_NR_RFCOMM_MULTIPLEXERS                  MAX_SPP_CONNECTIONS
#define MAX_NR_RFCOMM_SERVICES                      1
#define MAX_NR_SERVICE_RECORD_ITEMS                 1
#define MAX_NR_SM_LOOKUP_ENTRIES                    3
#define MAX_NR_WHITELIST_ENTRIES                    1
#define MAX_SPP_CONNECTIONS                         2
#else
// BTstack features that can be enabled
#ifdef CYGOPT_BLUETOOTH_ENABLE_BLE
#define ENABLE_BLE
#endif
#ifdef CYGOPT_BLUETOOTH_ENABLE_CLASIC
#define ENABLE_CLASSIC
#endif

#ifdef CYGOPT_BLUETOOTH_LOG_ERROR
#define ENABLE_LOG_ERROR
#endif
#ifdef CYGOPT_BLUETOOTH_LOG_INFO
#define ENABLE_LOG_INFO
#endif
#ifdef CYGOPT_BLUETOOTH_LOG_DEBUG
#define ENABLE_LOG_DEBUG
#endif

#define ENABLE_LOG_INTO_HCI_DUMP
//#define ENABLE_EHCILL

// BTstack configuration. buffers, sizes, ...
#define HCI_ACL_PAYLOAD_SIZE                        CYGINT_BLUETOOTH_ACL_PAYLOAD_SIZE

#ifdef ENABLE_CLASSIC
#define MAX_NR_BNEP_CHANNELS                        MAX_SPP_CONNECTIONS
#define MAX_NR_BNEP_SERVICES                        CYGINT_BLUETOOTH_BNEP_SERVICES
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES   10
#define MAX_NR_HCI_CONNECTIONS                      MAX_SPP_CONNECTIONS
#define MAX_NR_HFP_CONNECTIONS                      CYGINT_BLUETOOTH_HFP_CONNECTION
#define MAX_NR_L2CAP_CHANNELS                       (1+MAX_SPP_CONNECTIONS)
#define MAX_NR_L2CAP_SERVICES                       CYGINT_BLUETOOTH_L2CAP_SERVICES
#define MAX_NR_RFCOMM_CHANNELS                      MAX_SPP_CONNECTIONS
#define MAX_NR_RFCOMM_MULTIPLEXERS                  MAX_SPP_CONNECTIONS
#define MAX_NR_RFCOMM_SERVICES                      CYGINT_BLUETOOTH_RFCOMM_SERVICES
#define MAX_NR_SERVICE_RECORD_ITEMS                 CYGINT_BLUETOOTH_SERVICE_RECORD_ITEMS
#define MAX_SPP_CONNECTIONS                         CYGINT_BLUETOOTH_SPP_CONNECTIONS
#endif

#ifdef ENABLE_BLE
#define MAX_NR_GATT_CLIENTS                         CYGINT_BLUETOOTH_GATT_CLIENTS
#define MAX_NR_GATT_SUBCLIENTS                      CYGINT_BLUETOOTH_GATT_SUB_CLIENTS
#define MAX_NR_SM_LOOKUP_ENTRIES                    CYGINT_BLUETOOTH_SM_LOOKUP_ENTRIES
#define MAX_NR_WHITELIST_ENTRIES                    CYGINT_BLUETOOTH_WHITELIST_ENTRIES
#endif
#endif
// 

#endif

