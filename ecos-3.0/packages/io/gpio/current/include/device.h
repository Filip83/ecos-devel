// The 'features' section in 'target.json' is now used to create the device's hardware preprocessor switches.
// Check the 'features' section of the target description in 'targets.json' for more details.
/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_DEVICE_H
#define MBED_DEVICE_H

#include <pkgconf/io_gpio.h>

#ifndef CYGPKG_IO_GPIO_PORT_DEVICES_OUT
#define CYGPKG_IO_GPIO_PORT_DEVICES_OUT 0
#endif 

#ifndef CYGPKG_IO_GPIO_PORT_DEVICES_IN
#define CYGPKG_IO_GPIO_PORT_DEVICES_IN 0
#endif 

#ifndef CYGPKG_IO_GPIO_PORT_DEVICES_IN_OUT
#define CYGPKG_IO_GPIO_PORT_DEVICES_IN_OUT 0
#endif 


#if defined(CYGPKG_IO_GPIO_PORT_DEVICES)
#define DEVICE_PORTOUT		CYGPKG_IO_GPIO_PORT_DEVICES_OUT
#define DEVICE_PORTIN		CYGPKG_IO_GPIO_PORT_DEVICES_IN
#define DEVICE_PORTINOUT	CYGPKG_IO_GPIO_PORT_DEVICES_IN_OUT
#endif

#ifndef CYGPKG_IO_GPIO_PWM
#define CYGPKG_IO_GPIO_PWM 0
#endif 

#ifndef CYGPKG_IO_GPIO_ANALOG_OUT
#define CYGPKG_IO_GPIO_ANALOG_OUT 0
#endif 

#ifndef CYGPKG_IO_GPIO_ANALOG_IN
#define CYGPKG_IO_GPIO_ANALOG_IN 0
#endif 

#define DEVICE_PWMOUT		CYGPKG_IO_GPIO_PWM
#define DEVICE_ANALOGOUT	CYGPKG_IO_GPIO_ANALOG_OUT
#define DEVICE_ANALOGIN		CYGPKG_IO_GPIO_ANALOG_IN

#define DEVICE_ID_LENGTH       24

#include <cyg/io/gpio/objects.h>

#endif
