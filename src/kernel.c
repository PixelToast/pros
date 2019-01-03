/*
 * kernel.c - PROS Kernel and I/O initialization functions
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

#include <comm.h>
#include <fs.h>
#include <kernel.h>
#include <invoke.h>
#include <periph.h>
#include <supervisor.h>
#include <taskpriv.h>
#include <watchdog.h>
#include <emulator.h>

// Low-resolution clock
extern volatile uint32_t _clockLowRes;

// libc init functions
//extern void __libc_init_array();

static const char * const hex = "0123456789ABCDEF";
static void _fault_dumpInt(const char* prefix, uint32_t num) {
	while (*prefix != 0) EmuSerial_putc(1, *prefix++);
	EmuSerial_putc(1, '0');
	EmuSerial_putc(1, 'x');
  unsigned char i = 32;
  while (i) {
      i -= 4;
			EmuSerial_putc(1, hex[(num >> i) % 16]);
  }
	EmuSerial_putc(1, '\r');
	EmuSerial_putc(1, '\n');
}

// Error messages for _exit
// Since SDIV by 0 is configured not to fault, no divide by 0 can occur here
static const char * const errorMessages[] = {
	"Segmentation fault\n", "Segmentation fault\n", "Illegal instruction\n",
	"Stack overflow\n", "System task failure\n"
};

static const char * const crashMessage =
	"\r\nThe VEX Cortex has stopped working!\r\nError cause: ";

uint32_t _fault_dumpPC(int status, volatile uint32_t *faultStack) {
    __disable_irq();
    _fault_dumpInt("r0 = ", faultStack[0]);
    _fault_dumpInt("r1 = ", faultStack[1]);
    _fault_dumpInt("r2 = ", faultStack[2]);
    _fault_dumpInt("r3 = ", faultStack[3]);
    _fault_dumpInt("r12 = ", faultStack[4]);
    _fault_dumpInt("lr = ", faultStack[5]);
    _fault_dumpInt("pc = ", faultStack[6]);
    _fault_dumpInt("psr = ", faultStack[7]);

		char c; char const * buffer = crashMessage;

		// Dump it out over debug USART manually
		while ((c = *buffer++) != 0) {
			EmuSerial_putc(1, c);
		}
		// Dump message
		if ((unsigned int)status < 5U) {
			buffer = errorMessages[status];
			while ((c = *buffer++) != 0)
				EmuSerial_putc(1, c);
		}

    __reset();
}

// some ASM that calls our _fault_dumpPC function with a pointer to the stack that caused the fault
static void _fault_test(int status) __attribute__( ( naked ) );
static void _fault_test(int status) {
    __asm volatile (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r1, msp                                             \n"
        " mrsne r1, psp                                             \n"
        " ldr r2, [r1, #24]                                         \n"
        " ldr r3, handler2_address_const                            \n"
        " bx r3                                                     \n"
        " handler2_address_const: .word _fault_dumpPC               \n"
    );
}

// _exit - Exits the program with an error message
void __attribute__((noreturn)) _exit(int status) {
	_fault_test(status);

	// Reset CPU
	__reset();
}

// Stub functions for libc
void _init() { }
void exit(int status) { _exit(status); }

// ISR_HardFault - Hardware fault routine
IRQ ISR_HardFault() {
	_exit(STATUS_HARDFAULT);
}

// ISR_BusFault - Bus fault routine
IRQ ISR_BusFault() {
	_exit(STATUS_SEGFAULT);
}

// ISR_UsageFault - Illegal instruction fault routine
IRQ ISR_UsageFault() {
	_exit(STATUS_ILLEGAL_INSTRUCTION);
}

// ISR_SysTick - Interrupt service routine for the SysTick interrupt
IRQ ISR_SysTick() {
	_clockLowRes++;
	_taskTick();
}

// kwait - Goes to sleep for the specified time in milliseconds
void kwait(clock_t millis) {
	clock_t dest = timeLowRes() + millis;
	while (timeLowRes() < dest) __sleep();
}

// A quick memcpy implementation to satisfy GCC
void* memcpy(void *restrict to, void *restrict from, size_t size) {
	int32_t *src = (int32_t*)from, *dst = (int32_t*)to;
	for (; size >= sizeof(int32_t); size -= sizeof(int32_t))
		// Copy int (galloping)
		*(dst++) = *(src++);
	int8_t *s = (int8_t*)src, *d = (int8_t*)dst;
	// Copy bytes
	while (size--)
		*(d++) = *(s++);
	return to;
}

// startKernel - Start the PROS kernel
void startKernel() {
#ifndef NO_FILESYSTEM
	const uint32_t csr = RCC->CSR;
#endif
	// Initalize system and peripherals
	initMCU();
	// Set up supervisor
	svInit();
	// Excised due to problems with __libc_init_array symbol undefined on gcc/4.7.1
	//__libc_init_array();
	// User I/O initialization, really should be empty in most cases
	initializeIO();
	// Watchdog initialization, resets cortex when it locks up
	watchdogStart();
#ifndef NO_FILESYSTEM
	// File system scan with the TRIM enabled/disabled
	fsScan((csr & RCC_CSR_PORRSTF) ? true : false);
	// Only when we get a successful scan and clear off do we wipe out the POR flags
	RCC->CSR |= RCC_CSR_RMVF;
#endif
	// Wait for the supervisor to come up
	svSynchronize();
	// Start user initialization (task can be created but will not be started before scheduler)
	invStartInit();
	// Start the scheduler
	taskStartScheduler();
	// In case creating the idle task fails
	_exit(STATUS_CANT_START_SYSTEM);
}
