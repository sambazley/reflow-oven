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

#ifndef SPI_THERMO_H
#define SPI_THERMO_H

#include <stdint.h>

struct thermo_data {
	volatile uint8_t OC : 1;
	volatile uint8_t SCG : 1;
	volatile uint8_t SCV : 1;
	volatile uint8_t _1 : 1;
	volatile uint16_t ref_temp : 12;
	volatile uint8_t fault : 1;
	volatile uint8_t _2 : 1;
	volatile uint16_t thermo_temp : 14;
} __attribute__((packed));

void spi_thermo_recv(struct thermo_data *thermo);
int thermo_fault_check(struct thermo_data thermo);
void spi_thermo_init();

#endif /* SPI_THERMO_H */
