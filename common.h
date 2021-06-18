/* Copyright (C) 2021 Sam Bazley
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

enum {
	USB_PACKET_FAULT,
	USB_PACKET_TEMP,
};

enum thermo_fault {
	THERMO_NO_FAULT = 0,
	THERMO_UNKNOWN,
	THERMO_OPEN_CIRCUIT,
	THERMO_SHORT_TO_VCC,
	THERMO_SHORT_TO_GND,
	THERMO_D3,
	THERMO_D17,
};

struct usb_packet_fault {
	uint8_t length;
	uint8_t type;
	uint8_t fault;
} __attribute__((packed));

struct usb_packet_temp {
	uint8_t length;
	uint8_t type;
	float temp;
	float target;
} __attribute__((packed));

#endif /* COMMON_H */
