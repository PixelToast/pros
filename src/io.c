/*
 * io.c - Basic I/O functions for Cortex (lowest level functions, not Wiring compatible)
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
#include <periph.h>

// ADC data storage
uint16_t adcDataIn[16];

// adcRead - Reads a channel 0-15 from the ADC
uint16_t adcRead(uint32_t channel) {
	return adcDataIn[channel & (uint32_t)0xF];
}
