/*
 * i2c.c - I2C implementation
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

#include <semphr.h>
#include <FreeRTOS.h>
#include <periph.h>
#include <emulator.h>

// I2C statuses
#define I2C_READ_BIT ((uint8_t)0x01)
#define I2C_STATUS_ERR ((uint8_t)0x01)
#define I2C_STATUS_RESTART ((uint8_t)0x02)
#define I2C_STATUS_NOSTOP ((uint8_t)0x04)

// I2C status to use
typedef struct {
	// Up or down?
	volatile uint8_t status;
	// Target I2C address
	volatile uint8_t address;
	// Byte count left
	volatile uint16_t count;
	// Byte buffer to use
	volatile uint8_t *buffer;
	// I2C sync semaphore
	volatile Semaphore sync;
} I2CStatus_TypeDef;

volatile I2CStatus_TypeDef i2cState;

// I2C status flag initialization
void _i2cInit() {
	Semaphore sync = semaphoreCreate();
	semaphoreTake(sync, 0UL);
	i2cState.sync = sync;
	i2cState.status = 0;
}

// _i2cRead - Reads the specified number of data bytes from the specified address
static bool _i2cRead(uint8_t addr, uint8_t *data, uint16_t count) {
	volatile I2CStatus_TypeDef *state = &i2cState;

	EmuI2C_startRead(addr, (uint32_t)data, count);

	do {
		// Wait until BUSY flag is reset (until a STOP is generated)
		semaphoreTake(i2cState.sync, 2);
		// Error occurred?
		if (state->status & I2C_STATUS_ERR)
			return false;
	} while (i2cState.count > 0);
	return true;
}

// Writes data to the I2C interface, observing the NOSTOP flag for repeated-start generation
static bool _i2cWrite(uint8_t addr, uint8_t *data, uint16_t count) {
	uint8_t status;
	volatile I2CStatus_TypeDef *state = &i2cState;

	EmuI2C_startWrite(addr, (uint32_t)data, count);

	do {
		// Wait until BUSY flag is reset (until a STOP is generated) or repeated-start bit set
		semaphoreTake(i2cState.sync, 2);
		status = state->status;
		// Error occurred?
		if (status & I2C_STATUS_ERR)
			return false;
	} while (i2cState.count > 0);
	return true;
}

// i2cRead - Reads the specified number of data bytes from the specified address
bool i2cRead(uint8_t addr, uint8_t *data, uint16_t count) {
	return _i2cRead(addr, data, count);
}

// i2cReadRegister - Reads the specified amount of data from the given register address on
// the specified I2C address
bool i2cReadRegister(uint8_t addr, uint8_t reg, uint8_t *value, uint16_t count) {
	i2cState.status |= I2C_STATUS_NOSTOP;
	// Write out the location we want and read in data
	return _i2cWrite(addr, &reg, 1) && _i2cRead(addr, value, count);
}

// i2cSetAddress - Sets the Cortex's I2C address
void i2cSetAddress(uint8_t addr) {
	// Limit to 7 bits
	addr &= 0xFE;
	EmuI2C_setAddr(addr);
}

// i2cWrite - Writes the specified number of data bytes to the specified address
bool i2cWrite(uint8_t addr, uint8_t *data, uint16_t count) {
	i2cState.status &= ~I2C_STATUS_NOSTOP;
	return _i2cWrite(addr, data, count);
}

// i2cWriteRegister - Writes the specified data to a register on the specified I2C address
bool i2cWriteRegister(uint8_t addr, uint8_t reg, uint16_t value) {
	uint8_t data[2];
	// Write register, value
	data[0] = reg;
	data[1] = value;
	// Write information
	return i2cWrite(addr, data, 2);
}

// _i2cEnd - Helper for stop bit checking to clean up the I2C state
void _i2cEnd() {
	bool cs = false;
	// Unfortunately, no irq for "I2C bus is free" or "stop bit set"; thus we wait off the
	// stop bit using a timer since we cannot fail to send a stop bit if we are the master
	semaphoreGiveISR(i2cState.sync, &cs);
	if (cs)
		_taskYield();
}

// I2C1 event interrupt
void ISR_I2C1_EV() {
	volatile I2CStatus_TypeDef *state = &i2cState;
	state->count = 0;
}

// I2C1 error interrupt
void ISR_I2C1_ER() {
	bool cs = false;
	i2cState.status |= I2C_STATUS_ERR;

	semaphoreGiveISR(i2cState.sync, &cs);
	if (cs)
		_taskYield();
}
