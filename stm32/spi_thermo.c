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

#include "spi_thermo.h"
#include "common.h"
#include "io.h"
#include <stm32f0xx.h>

void spi_thermo_recv(struct thermo_data *thermo)
{
	GPIOA->ODR &= ~(1 << CS);

	while (SPI1->SR & SPI_SR_RXNE) {
		volatile char x = SPI1->DR;
		(void) x;
	}

	SPI1->CR1 |= SPI_CR1_SPE;

	for (int i = 0; i < 4; i++) {
		if (i == 3) {
			SPI1->CR1 &= ~SPI_CR1_SPE;
		}

		while (!(SPI1->SR & SPI_SR_RXNE)) {
			__NOP();
		}

		((volatile uint8_t *) thermo)[3 - i] = SPI1->DR;
	}

	GPIOA->ODR |= (1 << CS);

	while (SPI1->SR & SPI_SR_BSY) {
		__NOP();
	}

	while (SPI1->SR & SPI_SR_FRLVL) {
		volatile char x = SPI1->DR;
		(void) x;
	}
}

int thermo_fault_check(struct thermo_data thermo)
{
	if (thermo.OC) {
		return THERMO_OPEN_CIRCUIT;
	}
	if (thermo.SCG) {
		return THERMO_SHORT_TO_GND;
	}
	if (thermo.SCV) {
		return THERMO_SHORT_TO_VCC;
	}
	if (thermo._1) {
		return THERMO_D3;
	}
	if (thermo._2) {
		return THERMO_D17;
	}

	if (thermo.fault || thermo.thermo_temp == 0x1FFF) {
		return THERMO_UNKNOWN;
	}

	return THERMO_NO_FAULT;
}

void spi_thermo_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR1 |= SPI_CR1_BR_2;
	SPI1->CR2 &= ~SPI_CR2_DS_3;
	SPI1->CR2 |= SPI_CR2_FRXTH;
	SPI1->CR1 |= SPI_CR1_RXONLY;
}
