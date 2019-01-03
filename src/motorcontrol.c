/*
 * motorcontrol.c - Control the Cortex FET switches for 2-wire motor ports 1 and 10, and
 * low-level routines for the eight supervisor-controlled motor ports 2-9
 *
 * Copyright (c) 2011-2016, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include <FreeRTOS.h>
#include <supervisor.h>
#include <periph.h>

// Motor flags for needing update
#define MOTOR_FLAG_1 1
#define MOTOR_FLAG_10 2

// motorControlGet - Gets the last sent PWM value of a channel 1..10 from 0 to 255
uint8_t motorControlGet(uint32_t channel) {
	return EmuMotor_get(channel - 1);
}

// motorControlSet - Sets the PWM value of a channel 1..10 from 0 to 255
void motorControlSet(uint32_t channel, uint8_t value) {
	EmuMotor_set(channel - 1, value);
}

// motorControlStop - Stops all motors
void motorControlStop() {
	EmuMotor_stop();
}
