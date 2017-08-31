/*
 * usbs_kinetis.h
 *
 *  Created on: 11. 8. 2017
 *      Author: Filip
 */

#ifndef USBS_KINETIS_H_
#define USBS_KINETIS_H_

#include <pkgconf/hal.h>
#include <pkgconf/devs_usb_kinetis.h>

#include <cyg/io/usb/usbs.h>


extern usbs_control_endpoint    ep0;

#define USB_GET_STATUS_DEVICE_MASK (0x03U)

/*! @brief Defines USB device endpoint status mask */
#define USB_GET_STATUS_ENDPOINT_MASK (0x03U)

/*! @brief Defines USB device status size when the host request to get device status */
#define USB_DEVICE_STATUS_SIZE (0x02U)

/*! @brief Defines USB device interface status size when the host request to get interface status */
#define USB_INTERFACE_STATUS_SIZE (0x02U)

/*! @brief Defines USB device endpoint status size when the host request to get endpoint status */
#define USB_ENDPOINT_STATUS_SIZE (0x02U)

/*! @brief Defines USB device configuration size when the host request to get current configuration */
#define USB_CONFIGURE_SIZE (0X01U)

void usbs_kinetis_endpoint_start(usbs_rx_endpoint * pep);
void usbs_kinetis_endpoint_set_halted(usbs_rx_endpoint * pep,
                                          cyg_bool new_value);

void usbs_kinetis_init(void);
#endif /* USBS_KINETIS_H_ */
