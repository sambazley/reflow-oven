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

#include "io.h"
#include "oven.h"
#include "usb.h"
#include <stm32f0xx.h>

//milliseconds
#define INTERVAL 500

static void timer_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	TIM14->PSC = 7999;
	TIM14->ARR = INTERVAL;
	TIM14->CNT = 0;

	TIM14->DIER |= TIM_DIER_UIE;

	TIM14->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM14_IRQn);
}

void TIM14_Irq()
{
	oven_tick(INTERVAL / 1000.f);

	TIM14->SR &= ~TIM_SR_UIF;
	TIM14->CNT = 0;
	TIM14->CR1 |= TIM_CR1_CEN;
}

void boot()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (1 << (2 * LED_B))
		| (1 << (2 * LED_G))
		| (1 << (2 * RELAY))
		| (1 << (2 * CS));

	GPIOA->MODER |= (2 << (2 * CLK))
		| (2 << (2 * MISO));

	GPIOA->OTYPER |= (1 << LED_G) | (1 << LED_B); //open drain

	GPIOA->ODR = 0;

	spi_thermo_init();
	oven_init();

	usb_init();

	timer_init();

	while (1) {
		__NOP();
	}
}
