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

#include "oven.h"
#include "common.h"
#include "io.h"
#include "spi_thermo.h"
#include "usb.h"
#include <stm32f0xx.h>
#include <string.h>

#define TP(time, temp) {time, temp, 0.f}

static volatile struct temp_point {
	int end_time, end_temp;
	float gradient;
} temp_data [] = {
	TP(0, 0),
	TP(0, 0),
	TP(90, 90),
	TP(180, 130),
	TP(210, 138),
	TP(245, 165),
	TP(265, 165),
	TP(500, 0),
	TP(501, 0)
}, *curr = 0, *last_curr = 0;

#define TEMP_COUNT (sizeof(temp_data) / sizeof(struct temp_point))

static float kp = 1.f, kd;

struct pid {
	float error, prev_error;
	float target;
} pid, next_pid;

static volatile int oven_en = 0;

void oven_enable(int en)
{
	struct thermo_data thermo;
	spi_thermo_recv(&thermo); //flush
	spi_thermo_recv(&thermo);

	temp_data[0].end_temp = thermo.thermo_temp / 4;
	temp_data[1].end_temp = thermo.thermo_temp / 4;
	temp_data[1].end_time = thermo.thermo_temp / 4;

	for (uint8_t i = 1; i < TEMP_COUNT; i++) {
		volatile struct temp_point *tp = &temp_data[i], *prev = tp - 1;
		tp->gradient = (tp->end_temp - prev->end_temp) / (float) (tp->end_time - prev->end_time);
	}

	oven_en = en;
}

static float calculate_output(struct pid *pid, volatile struct temp_point *tp,
		float t, float sp, float interval)
{
	volatile struct temp_point *prev = tp - 1;

	float time_elapsed = t - prev->end_time;
	pid->target = prev->end_temp + time_elapsed * tp->gradient;

	pid->error = ((pid->target - sp) + pid->error * 4.f) / 5.f;
	float derivative = (pid->error - pid->prev_error) / interval;

	pid->prev_error = pid->error;

	return kp * pid->error + kd * derivative;
}

void oven_tick(float interval)
{
	static float progress = 0.f;

	struct thermo_data thermo;
	enum thermo_fault fault;

	GPIOA->ODR |= (1 << LED_G) | (1 << LED_B);

	if (!oven_en) {
		GPIOA->ODR &= ~((1 << LED_B) | (1 << LED_G));
		return;
	}

	spi_thermo_recv(&thermo);
	fault = thermo_fault_check(thermo);

	if (fault) {
		GPIOA->ODR &= ~(1 << RELAY);
		usb_send_fault(fault);
		oven_en = 0;
		progress = 0;
		return;
	}

	for (unsigned int i = 1; i < TEMP_COUNT; i++) {
		if (temp_data[i].end_time > progress) {
			curr = &temp_data[i];
			break;
		}
	}

	if (curr != last_curr) {
		memcpy(&pid, &next_pid, sizeof(struct pid));
		memset(&next_pid, 0, sizeof(struct pid));
	}

	last_curr = curr;

	if (curr == temp_data + TEMP_COUNT - 1) {
		GPIOA->ODR &= ~((1 << RELAY) | (1 << LED_G));
		usb_send_temp(0, 0);
		oven_en = 0;
		progress = 0;
		return;
	}

	float temp = thermo.thermo_temp / 4.f;
	kd = 10.f;
	float output = calculate_output(&pid, curr, progress, temp, interval);
	if ((curr + 1)->gradient > curr->gradient) {
		kd = 18.f;
	} else {
		kd = 13.f;
	}
	float next_output = calculate_output(&next_pid, curr + 1, progress, temp, interval);

	if ((curr + 1)->gradient > curr->gradient) {
		if (next_output > output) {
			output = next_output;
		}
	} else if ((curr + 1)->gradient >= 0) {
		if (next_output < output) {
			output = next_output;
		}
	}

	if (output > 0) {
		GPIOA->ODR |= (1 << RELAY);
		GPIOA->ODR &= ~(1 << LED_G);
	} else {
		GPIOA->ODR &= ~(1 << RELAY);
	}

	usb_send_temp(temp, pid.target);

	if ((int) progress % 2) {
		GPIOA->ODR &= ~(1 << LED_B);
	}

	progress += interval;
}

void oven_init()
{
	for (uint8_t i = 1; i < TEMP_COUNT; i++) {
		volatile struct temp_point *tp = &temp_data[i], *prev = tp - 1;
		tp->gradient = (tp->end_temp - prev->end_temp) / (float) (tp->end_time - prev->end_time);
	}
}
