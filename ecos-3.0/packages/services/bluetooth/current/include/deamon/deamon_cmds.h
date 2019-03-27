/*
 * deamon_cmds.h
 *
 *  Created on: 14. 4. 2018
 *      Author: filip
 */

#ifndef SRC_DEAMON_DEAMON_CMDS_H_
#define SRC_DEAMON_DEAMON_CMDS_H_

#define BTSTACK_INQUERY_START				0x09
#define BTSTACK_IS_INQUERY_COMPLETE  		0x0a
#define BTSTACK_GET_INQUERY_DEVICE_COUNT	0x0b
#define BTSTACK_GET_INQUERY_DEVICES     	0x0c
#define BTSTACK_WAIT_FOR_IN_CONNECTION      0x0d
#define BTSTACK_ABORT						0x0e
#define BTSTACK_GET_RFCOMM_SERVICES         0x0f


#define BTSTACK_MAJOR                       0
#define BTSTACK_MINOR						1


#endif /* SRC_DEAMON_DEAMON_CMDS_H_ */
