/*
 * encoder.c - Interrupt and quadrature encoder functions
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
#include <kernel.h>
#include <periph.h>

// Sensor status storage
Sensor_TypeDef _sensorState[BOARD_NR_DIGITAL_IO];

// DEAD code - replaced by Assembly ISRs in encoder_isr.s
#if 0
// Interrupt routine for the "bottom" connection to a quadrature encoder
static void _encoderISRBottom(uint32_t pin) {
	Encoder_TypeDef *encoder = &encState[pin - 1];
	// pin == encoder->portBottom
	volatile bool value = digitalRead(pin);
	// Store flags for later
	uint16_t flags = encoder->flags;
	if (flags & (uint16_t)0x02) {
		uint8_t top = encoder->portTop;
		// Read in other port value
		volatile bool p2 = digitalRead(top);
		bool reverse = (flags & (uint16_t)0x01) != 0;
		int32_t offset;
		// Look at count for top port
		if (reverse == p2)
			offset = 1;
		else
			offset = -1;
		// Reverse if necessary, up/down if necessary
		if (!value) offset = -offset;
		encState[top - 1].count += offset;
	}
}

// Interrupt routine for the "top" connection to a quadrature encoder
static void _encoderISRTop(uint32_t pin) {
	Encoder_TypeDef *encoder = &encState[pin - 1];
	volatile bool p2 = digitalRead(pin);
	// Store flags for later
	uint16_t flags = encoder->flags;
	if (flags & (uint16_t)0x02) {
		// Read in other port value
		volatile bool value = digitalRead(encoder->portBottom);
		bool reverse = (flags & (uint16_t)0x01) != 0;
		int32_t offset;
		// Look at count for top port
		if (reverse == p2)
			offset = 1;
		else
			offset = -1;
		// Reverse if necessary, up/down if necessary
		if (value) offset = -offset;
		encoder->count += offset;
	}
}
#else
// Assembly function prototypes
void _encoderISRBottom(unsigned char pin);
void _encoderISRTop(unsigned char pin);
#endif

// encoderGet - Gets the value of the encoder
int encoderGet(Encoder enc) {
	Sensor_TypeDef *encoder = (Sensor_TypeDef*)enc;
	if (encoder)
		return (int)encoder->value;
	return 0;
}

// encoderInit - Initializes and enables a quadrature encoder on two digital ports
Encoder encoderInit(unsigned char portTop, unsigned char portBottom, bool reverse) {
	Sensor_TypeDef *encoder, *slave;
	// Change to 0..11 range
	uint32_t it = portTop - 1, ib = portBottom - 1;
	// Check range
	if (it < BOARD_NR_DIGITAL_IO && it != 9 && _sensorState[it].flags == 0 &&
			ib < BOARD_NR_DIGITAL_IO && ib != 9 && _sensorState[ib].flags == 0) {
		_enterCritical();
		{
			uint16_t flags = (uint16_t)0x0002 | (uint16_t)(reverse ? 0x01 : 0x00);
			encoder = &_sensorState[it];
			slave = &_sensorState[ib];
			// Set pins to input pull-up
			pinMode(portTop, DDR_INPUT_PULLUP);
			pinMode(portBottom, DDR_INPUT_PULLUP);
			// Set state of master pin (top)
			encoder->flags = flags;
			encoder->portTop = (uint8_t)portTop;
			encoder->portBottom = (uint8_t)portBottom;
			encoder->value = 0;
			encoder->lastValue = 0;
			// Set slaved state
			slave->flags = flags;
			slave->portTop = (uint8_t)portTop;
			slave->portBottom = (uint8_t)portBottom;
			// Interrupt per-port on either rising or falling edge
			ioSetInterrupt(portTop, INTERRUPT_EDGE_BOTH, _encoderISRTop);
			ioSetInterrupt(portBottom, INTERRUPT_EDGE_BOTH, _encoderISRBottom);
		}
		_exitCritical();
		return (Encoder)encoder;
	}
	return NULL;
}

// encoderReset - Resets the encoder to zero
void encoderReset(Encoder enc) {
	Sensor_TypeDef *encoder = (Sensor_TypeDef*)enc;
	if (encoder) {
		_enterCritical();
		{
			encoder->value = 0;
			encoder->lastValue = 0;
		}
		_exitCritical();
	}
}

// encoderShutdown - Stops and disables the encoder
void encoderShutdown(Encoder enc) {
	Sensor_TypeDef *encoder = (Sensor_TypeDef*)enc;
	if (encoder) {
		_enterCritical();
		{
			// Stop
			encoder->flags = 0;
			_sensorState[encoder->portBottom].flags = 0;
			// Clear interrupts
			ioClearInterrupt(encoder->portTop);
			ioClearInterrupt(encoder->portBottom);
		}
		_exitCritical();
	}
}

// ioClearInterrupt - Disables interrupts on the specified pin
void ioClearInterrupt(unsigned char pin) {
	pin--;
	if (pin < 12 && pin != 9) {
		// Avoid having the OS swap this out
		_enterCritical();
		{
			EmuGPIO_SetInterrupt(pin, 0);
		}
		_exitCritical();
	}
}

// ioSetInterrupt - Sets up an interrupt to occur on the specified pin, and resets count & time
// Provide NULL for handler for standard interrupts, or pass function pointer for custom
void ioSetInterrupt(unsigned char pin, unsigned char edges, InterruptHandler handler) {
	pin--;
	if (pin < BOARD_NR_DIGITAL_IO && pin != 9) {
		// Avoid having the OS swap this out during init
		_enterCritical();
		{
			// Configure freely, we won't have an issue here since interrupt is masked
			Sensor_TypeDef *state = &_sensorState[pin];
			state->eventTrigger = handler;
			EmuGPIO_SetInterrupt(pin, edges);
		}
		_exitCritical();
	}
}

// Fires off the external interrupt logic for a given pin
static INLINE void triggerEXTI(uint32_t pin) {
	InterruptHandler handler = _sensorState[pin].eventTrigger;
	if (handler != NULL)
		handler(pin + 1);
}

IRQ ISR_EXTI0() {
	triggerEXTI(EMULATOR->R1);
}

IRQ ISR_EXTI1() {}
IRQ ISR_EXTI9_5() {}
IRQ ISR_EXTI15_10() {}
