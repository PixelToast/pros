/*
 * supervisor.c - Functions for interfacing with the NXP supervisor processor responsible for
 * handling the gory details of VEXnet
 *
 * Copyright (c) 2011-2017, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include <comm.h>
#include <invoke.h>
#include <kernel.h>
#include <supervisor.h>
#include <periph.h>

// A magic number for communication!
#define SV_MAGIC ((uint16_t)0xC917)

// Release-ready?
#if (FW_VERSION_TYPE == RELEASE)
#define FW_DISCLAIMER ""
#else
#define FW_DISCLAIMER "This is a BETA version of PROS and may have issues.\r\n"
#endif

// Buffers for SPI communications (16 words = 32 bytes)
volatile uint16_t spiBufferRX[16];
// Flag bits for the FMS
volatile uint16_t svFlags;
// Last flag bits for the FMS
static volatile uint16_t svLastFlags;
// Value used to fix backup battery + power expander problems
static uint16_t svPowerFlags;
// Index counter used for the interrupt-driven transfer
static uint8_t svIndex;
// Packet number, wraps around every 256th packet
static uint8_t svPacket;
// Team name to report when asked for configuration
char svTeamName[8];

// standaloneModeEnable - enable standalone operation
void standaloneModeEnable() {
	// set flag in outgoing SPI buffer to enable standlone mode
	EmuComp_enableStandalone();
}

// svInit - Initialize supervisor communications
void svInit() {
	EmuComp_init();

	uint8_t i;

	svPacket = (uint8_t)0x00;
	// Reset all flags
	svFlags = (uint32_t)0x00000000;
	svLastFlags = (uint32_t)0x00000000;
	svPowerFlags = (uint16_t)0x0000;
	svIndex = (uint8_t)0x00;

	// Load in neutral joystick values
	for (i = 0; i < 6; i++) {
		SV_IN->joystick[0].axis[i] = (uint8_t)0x7F;
		SV_IN->joystick[1].axis[i] = (uint8_t)0x7F;
	}
	// Load in no buttons pressed
	SV_IN->joystick[0].button56 = (uint8_t)0x00;
	SV_IN->joystick[0].button78 = (uint8_t)0x00;
	SV_IN->joystick[1].button56 = (uint8_t)0x00;
	SV_IN->joystick[1].button78 = (uint8_t)0x00;
	// Default team name shows up during firmware flashing
	svTeamName[0] = 'P';
	svTeamName[1] = 'R';
	svTeamName[2] = 'O';
	svTeamName[3] = 'S';
	svTeamName[4] = ' ';
	svTeamName[5] = ' ';
	svTeamName[6] = ' ';
	svTeamName[7] = ' ';
}

// svSynchronize - Waits for the supervisor to synchronize, then prints the startup message
void svSynchronize() {
	while (!(svFlags & SV_CONNECTED)) __sleep();
	// HELLO message!
	kwait(50);
	print("\r\nPowered by PROS " FW_VERSION "\r\n" FW_DISCLAIMER
		"\r\nPROS (C)2011-2018 Purdue ACM SIGBOTS\r\n");
	print("This program has ABSOLUTELY NO WARRANTY, not even an implied\r\n"
		"warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\r\n\r\n");
}

// ISR_SPI1 - Interrupt-driven routine to handle master communication
void ISR_SPI1() {
	if (EMULATOR->R1 == 0) {
		svFlags |= SV_CONNECTED;
	} else if (EMULATOR->R1 == 1) {
		uint8_t temp; uint16_t flags = 0;
		EmuComp_setName(svTeamName);
		EmuComp_getStatus((void*)spiBufferRX);

		temp = SV_IN->gameStatus;
		if (temp & (uint8_t)0x08) {
			// FMS is connected
			flags |= SV_FMS;
			if (temp & (uint8_t)0x40)
				// Autonomous mode
				flags |= SV_AUTONOMOUS;
			if (!(temp & (uint8_t)0x80))
				// Enabled
				flags |= SV_ENABLED;
		}
		if (flags != svLastFlags) {
			// Something changed! Wake up the daemon to take action
			svLastFlags = flags;
			invWakeup();
		}

		// Backup battery fix
		if (SV_IN->mainBattery <= 32) {
			if (svPowerFlags < 64) svPowerFlags++;
		} else {
			if (svPowerFlags > 0) svPowerFlags--;
		}

		svFlags = flags;
	}
}
