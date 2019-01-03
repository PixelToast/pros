/*
 * system.c - Low-level system control functions for interrupts, clocks, and peripheral init
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
#include <kernel.h>
#include <periph.h>
#include <semphr.h>
#include <emulator.h>

// Low-resolution clock
volatile uint32_t _clockLowRes;

// Prototype this from ultrasonic.c
void _ultrasonicTimeout();

// Prototype this from io.c
extern uint16_t adcDataIn[16];

#if 0
// intDisable - Disables a Cortex peripheral interrupt [not a negative system interrupt]
static INLINE void intDisable(IRQn_Type irq) {
	NVIC->ICER[(uint32_t)irq >> 0x05] = (uint32_t)(1 << ((uint32_t)irq & 0x1F));
}
#endif

// intEnable - Enables a Cortex peripheral interrupt
static INLINE void intEnable(IRQn_Type irq) {
	NVIC->ISER[(uint32_t)irq >> 0x05] = (uint32_t)(1 << ((uint32_t)irq & 0x1F));
}

// intSetPriority - Sets the priority for an interrupt; 15 is lowest priority, 0 is highest
static INLINE void intSetPriority(IRQn_Type irq, uint32_t priority) {
	if (irq < 0)
		SCB->SHP[((uint32_t)irq & 0x0F) - 4] = (priority << 4) & 0xFF;
	else
		NVIC->IP[(uint32_t)irq] = (priority << 4) & 0xFF;
}

// initClocks - Starts the MCU clocks at the intended speed (72 MHz)
// No startup timeout is enforced since the entire Cortex will come crashing down if the clocks
// cannot get to the correct speed; might as well hang. In the future, a "clock monitor fail"
// mode might be implemented if this extremely, extremely remote possibility becomes an issue.
static INLINE void initClocks() {
	uint32_t temp;
	// Clear interrupt pending bits
	RCC->CIR = 0x009F0000;
	// Reset Sleep Control register to zero to avoid unwanted deep sleep
	SCB->SCR = 0x00000000;
	// Turn on the HSE (8 MHz)
	temp = RCC->CR;
	temp |= RCC_CR_HSEON;
	temp &= ~RCC_CR_HSEBYP;
	RCC->CR = temp;
	RCC->CFGR &= ~RCC_CFGR_SW;
	// Wait for HSE to start up
	while (!(RCC->CR & RCC_CR_HSERDY));

	// APB1 is 36 MHz and APB2 is 72 MHz
	// ADC clock is 12 MHz (72 MHz / 6)
	// PLL enabled from HSE = 8 MHz * 9 = 72 MHz
	RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PLLMUL_9 |
		RCC_CFGR_PLLSRC_HSE;
	// Turn PLL on
	RCC->CR |= RCC_CR_PLLON;
	// Wait for PLL to start up
	while (!(RCC->CR & RCC_CR_PLLRDY));
	// Select PLL as system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	// Wait for system clock to become the PLL
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	// Reset system clocks
	_clockLowRes = 0;
}

// initEXTI - Initializes the external interrupts
static INLINE void initEXTI() {
	// Mask all interrupts and events
	EXTI->IMR = 0;
	EXTI->EMR = 0;
	// PD0, PD1 supply EXTI0, EXTI1; PA2, PA3 supply EXTI2, EXTI3
	AFIO->EXTICR[0] = (uint32_t)0x0033;
	// PA4, PA5 supply EXTI4, EXTI5; PC6, PC7 supply EXTI6, EXTI7
	AFIO->EXTICR[1] = (uint32_t)0x2200;
	// PE8..11 supplies EXTI8..11
	AFIO->EXTICR[2] = (uint32_t)0x4444;
	// PE12..14 supplies EXTI12..14
	AFIO->EXTICR[3] = (uint32_t)0x0444;
	// Disable on falling and rising edges
	EXTI->FTSR = 0;
	EXTI->RTSR = 0;
	// Clear all pending external interrupts
	EXTI->PR = (uint32_t)0x0007FFFF;
}

// initInterrupts - Initializes the NVIC (interrupt system)
static INLINE void initInterrupts() {
	// Vector table is at start of Flash
	SCB->VTOR = FLASH_BASE;
	// Priority group #3 configuration
	SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_3;
	// IRQ channel -2 (PendSV) enable
	intSetPriority(PendSV_IRQn, 14);
	// IRQ channel -5 (SV call) enable
	intSetPriority(SVCall_IRQn, 13);
	// IRQ channel 46 (Timer #8 capture/compare) enable, very high priority
	intSetPriority(TIM8_CC_IRQn, 2);
	intEnable(TIM8_CC_IRQn);
	// IRQ channel 37 (USART #1) enable
	intSetPriority(USART1_IRQn, 12);
	intEnable(USART1_IRQn);
	// IRQ channel 38 (USART #2) enable
	intSetPriority(USART2_IRQn, 12);
	intEnable(USART2_IRQn);
	// IRQ channel 39 (USART #3) enable
	intSetPriority(USART3_IRQn, 12);
	intEnable(USART3_IRQn);
	// IRQ channel (SPI #1) enable
	intSetPriority(SPI1_IRQn, 11);
	intEnable(SPI1_IRQn);
	// IRQ channel 6 (Pin change interrupt 0) enable, high priority
	intSetPriority(EXTI0_IRQn, 3);
	intEnable(EXTI0_IRQn);
	// IRQ channel 7 (Pin change interrupt 1) enable, high priority
	intSetPriority(EXTI1_IRQn, 3);
	intEnable(EXTI1_IRQn);
	// IRQ channel 23 (Pin change interrupts 5-9) enable, high priority
	intSetPriority(EXTI9_5_IRQn, 3);
	intEnable(EXTI9_5_IRQn);
	// IRQ channel 40 (Pin change interrupts 10-15) enable, high priority
	intSetPriority(EXTI15_10_IRQn, 3);
	intEnable(EXTI15_10_IRQn);
	// IRQ channel 31 (I2C1 event) enable, very high priority not to be interrupted (it's fast)
	intSetPriority(I2C1_EV_IRQn, 2);
	intEnable(I2C1_EV_IRQn);
	// IRQ channel 32 (I2C1 error) enable, high priority
	intSetPriority(I2C1_ER_IRQn, 3);
	intEnable(I2C1_ER_IRQn);
	// IRQ channel 58 (DMA2 channel3 TX complete) enable, high priority
	intSetPriority(DMA2_Channel3_IRQn, 3);
	intEnable(DMA2_Channel3_IRQn);
	// SysTick fires every 9000 clock cycles (72M / 8 / 9K = 1K/s = 1 ms)
	SysTick->LOAD = 8999;
	// Set priority for SysTick interrupt to a low priority (second from bottom)
	intSetPriority(SysTick_IRQn, 13);
	// Reset counter to zero
	SysTick->VAL = 0;
	// Turn on, enable interrupt, select clock / 8
	SysTick->CTRL = SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE;
	// Enable the faults
	SCB->SHCSR = SCB_SHCSR_USGFAULTENA | SCB_SHCSR_BUSFAULTENA;
	__enable_irq();
	__enable_fault_irq();
}

// Bits to toggle in the reset register to reset I/O ports to defaults
#define _PORT_RESET_BITS (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | \
	RCC_APB2RSTR_IOPCRST | RCC_APB2RSTR_IOPDRST | RCC_APB2RSTR_IOPERST | \
	RCC_APB2RSTR_AFIORST)

// initPorts - Initializes the GPIO ports
static INLINE void initPorts() {
	uint32_t temp;
	// Enable clocks to all I/O ports and AFIO
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
		RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN | RCC_APB2ENR_AFIOEN;
	// Reset the I/O ports and AFIO
	temp = RCC->APB2RSTR;
	RCC->APB2RSTR = temp | _PORT_RESET_BITS;
	__dsb();
	// Clear reset
	RCC->APB2RSTR = temp;

	EmuGPIO_SetDir(PIN_ANALOG_1, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_2, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_3, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_4, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_5, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_6, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_7, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_8, DDR_INPUT_ANALOG);
	EmuGPIO_SetDir(PIN_ANALOG_9, DDR_INPUT_ANALOG);

	// Turn on GPIOB 8 & 9 [I2C SCL, SDA] pre-emptively
	EmuGPIO_SetDir(PIN_I2C1_SCL, DDR_OUTPUT);
	EmuGPIO_SetDir(PIN_I2C1_SDA, DDR_OUTPUT);
	EmuGPIO_SetOutput(PIN_I2C1_SCL, 1);
	EmuGPIO_SetOutput(PIN_I2C1_SDA, 1);

	// Set up GPIOC 10 [UART2 TX] as an alternate function output push-pull
	EmuGPIO_SetDir(PIN_UART2_TX, DDR_AFO);
	// Set up GPIOC 11 [UART2 RX] as a floating input
	EmuGPIO_SetDir(PIN_UART2_RX, DDR_INPUT_FLOATING);

	motorControlStop();
	// Set up GPIOD 5 [UART1 TX] as an alternate function output push-pull
	EmuGPIO_SetDir(PIN_UART1_TX, DDR_AFO);
	// Set up GPIOD 6 [UART1 RX] as a floating input
	EmuGPIO_SetDir(PIN_UART1_RX, DDR_INPUT_FLOATING);

	// Remap the following: USART2, USART3, TIM4, I2C1, TIM1
	AFIO->MAPR = AFIO_MAPR_TIM4_REMAP | AFIO_MAPR_USART2_REMAP | AFIO_MAPR_USART3_REMAP |
		AFIO_MAPR_I2C1_REMAP | AFIO_MAPR_TIM1_REMAP_FULL;
	AFIO->MAPR2 = 0x0000;
	// GPIOF and GPIOG don't have actual pins, so no need to set them to pullups
}

// initSerial - Sets up the USARTs for communication through the UART ports and debug terminal
static INLINE void initSerial() {
	// Ready the buffers
	usartBufferInit();

	EmuSerial_init(1, 230113, 0);
}

// initTimers - Initializes the TIM modules for the desired interrupt frequencies
static INLINE void initTimers() {
	uint32_t temp;
	// Turn on TIM4-6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM6EN;
	// Turn on TIM8 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM1EN;
	// Reset TIM4-6
	temp = RCC->APB1RSTR;
	RCC->APB1RSTR = temp | (RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_TIM5RST | RCC_APB1RSTR_TIM6RST);
	__dsb();
	// Clear reset
	RCC->APB1RSTR = temp;
	// Reset TIM8
	temp = RCC->APB2RSTR;
	RCC->APB2RSTR = temp | RCC_APB2RSTR_TIM8RST | RCC_APB2RSTR_TIM1RST;
	__dsb();
	// Clear reset
	RCC->APB2RSTR = temp;
	// Divide by 641, count up to 111
	// Do not set max to 127 - this is set up to match the Motor Controller 29s!
	TIM4->ARR = (uint16_t)110;
	TIM4->PSC = (uint16_t)640;
	// Enable OC1-OC4 as output compare, preload enabled, active then inactive when counting up
	// ("PWM mode 1")
	TIM4->CCMR1 = (uint16_t)0x6868;
	TIM4->CCMR2 = (uint16_t)0x6868;
	// Activate OC1-OC4 as active low signals (with the CCMR1-2 settings, this causes a right-
	// aligned active high pulse to appear on the output)
	TIM4->CCER = (uint16_t)0x3333;
	// Turn on TIM4
	TIM4->CR1 = TIM_CR1_CEN;
	// Maximum period, no divider [clocked from another timer]
	TIM5->ARR = (uint16_t)0xFFFF;
	TIM5->PSC = (uint16_t)0;
	// Enable slave mode to ITR3 (Timer 8)
	TIM5->SMCR = TIM_SMCR_SMS_EXTERNAL | TIM_SMCR_TS_ITR3;
	// Enable update interrupt
	TIM5->DIER = TIM_DIER_UIE;
	// Turn on TIM5
	TIM5->CR1 = TIM_CR1_CEN;
	// Maximum period, divide by 72
	TIM8->ARR = (uint16_t)0xFFFF;
	TIM8->PSC = (uint16_t)71;
	// On every overflow, trigger next timer
	TIM8->CR2 = TIM_CR2_MMS_UPDATE;
	// Turn on TIM8
	TIM8->CR1 = TIM_CR1_CEN;
	// Maximum period, divide by 2 (later configurable by user)
	TIM1->ARR = (uint16_t)0xFFFF;
	TIM1->PSC = (uint16_t)1;
	// Set up CCR1-4 as output compare PWM channels ("PWM mode 1")
	TIM1->CCMR1 = (uint16_t)0x6868;
	TIM1->CCMR2 = (uint16_t)0x6868;
	// Reset OC1-OC4 to off (can be enabled by the user)
	TIM1->CCER = (uint16_t)0;
	// Enable PWM outputs, set up break/dead time register
	TIM1->BDTR = (uint16_t)0x8100;
	// Turn on TIM1, enable preload of period register
	TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	// Set up TIM6 for the DAC for the Wave driver
	// We divide 36000000 by 900 = 20 KHz update rate
	TIM6->PSC = 3;
	TIM6->ARR = 449;
	// At this rate we exhaust the 256 sample buffer in about 6.5ms
	// Since the reload takes <<1ms, we are fine on cpu usage
	TIM6->CR1 = TIM_CR1_ARPE;
	TIM6->CR2 = TIM_CR2_MMS_UPDATE;
}

// initMCU - Initialize the MCU
void initMCU() {
	initClocks();
	initPorts();
	initEXTI();
	initSerial();
	initTimers();
	initInterrupts();
	EmuGPIO_ADCInit((uint32_t)adcDataIn);
}

// ISR_TIM8_CC - Timer 8 capture/compare (microsecond queue)
void ISR_TIM8_CC() {
	uint16_t sr = TIM8->SR & TIM8->DIER;
	if (sr & TIM_SR_CC1IF) {
		// CC1 fired one-time
		TIM8->DIER &= ~TIM_DIER_CC1IE;
	}
	if (sr & TIM_SR_CC2IF) {
		// CC2 fired one-time
		TIM8->DIER &= ~TIM_DIER_CC2IE;
	}
	if (sr & TIM_SR_CC3IF) {
		// CC3 fired one-time
		TIM8->DIER &= ~TIM_DIER_CC3IE;
	}
	if (sr & TIM_SR_CC4IF)
		// CC4 fired one-time, special handling due to recursive scheduling
		_ultrasonicTimeout();
}

// timeLowRes - Gets the low-resolution tick count (milliseconds)
clock_t timeLowRes() {
	return _clockLowRes;
}

// timeHighRes - Gets the high-resolution tick count (microseconds)
clock_t timeHighRes() {
	uint16_t highBits = TIM5->CNT, lowBits = TIM8->CNT, secondHighBits = TIM5->CNT;
	if (secondHighBits != highBits && lowBits < 0x7FFF)
		// Rollover compensation
		highBits = secondHighBits;
	return (clock_t)(((uint32_t)highBits << 16) | (uint32_t)lowBits);
}
