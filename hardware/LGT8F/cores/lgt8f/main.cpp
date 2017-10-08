/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <avr/wdt.h>
//#include <Arduino.h>
#include <wiring_private.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }

#if defined(__LGT8FX8E__) || defined(__LGT8FX8P__)
void __patch_wdt(void) \
	     __attribute__((naked)) \
	     __attribute__((section(".init3")));
void __patch_wdt(void)
{
	MCUSR = 0;
	wdt_disable();
}
#endif

//#pragma __attribute__(always_inline)
void unlockWrite(volatile uint8_t *p, uint8_t val)
{
	uint8_t _o_sreg = SREG;
	volatile uint8_t *cp = p; 

	if(p == &PMX1)
		cp = &PMX0;
	cli();
	*cp = 0x80;
	*p = val;
	SREG = _o_sreg;
}

void atomicWriteWord(volatile uint8_t *p, uint16_t val)
{
	uint8_t _o_sreg = SREG;

	cli();
	*(p+1) = (uint8_t)(val >> 8);
	nop();
	*p = (uint8_t)val;
	SREG = _o_sreg;
}

void sysClock(uint8_t mode)
{
	if(mode == EXT_OSC) {
		// enable external crystal
		GPIOR0 = PMCR | 0x04;
		PMCR = 0x80;
		PMCR = GPIOR0;
		
		// waiting for crystal stable
		delay(20);

		// switch to external crystal
		GPIOR0 = (PMCR & 0x9f) | 0x20;
		PMCR = 0x80;
		PMCR = GPIOR0;

		// set to right prescale
		CLKPR = 0x80;
		CLKPR = 0x00;	
	} else if(mode == INT_OSC) {
		// prescaler settings
		CLKPR = 0x80;
		CLKPR = 0x01;	

		// switch to internal crystal
		GPIOR0 = PMCR & 0x9f;
		PMCR = 0x80;
		PMCR = GPIOR0;

		// disable external crystal
		GPIOR0 = PMCR & 0xfb;
		PMCR = 0x80;
		PMCR = GPIOR0;
	}
}

#if defined(__LGT8FX8P__) || defined(__LGT8FX8E__)
// Log(HSP v3.7): PWM working mode
// Function:
//	wmode: pwm working mode
//		- PWM_MODE_NORMAL: normal single output
//		- PWM_MODE_COMPM0: complementary dual output 
//		- PWM_MODE_COMPM1: complementary dual output (inverted)
//	fmode: pwm frequency settings
//		- PWM_FREQ_SLOWER: slow range
//		- PWM_FREQ_FASTER: fast range 
//	dband: dead band settings
//		- only valid for complementary working mode 
void pwmMode(uint8_t pin, uint8_t wmode, uint8_t fmode, uint8_t dband)
{
	uint8_t timer = digitalPinToTimer(pin) & 0xf0;

	if(timer == 0x10) {	// TIMER0
	} else if(timer == 0x20) { // TIMER1
	} else if(timer == 0x30) { // TIMER2
			// TIMER8 is 8bit only, default frequency = 16000000/(64 * 256) = 976.5Hz
			// TODO: Maybe it's good idea to make a possibility to modify its prescale
			// - so we can get a littler more faster or slower frequency
	} else if(timer == 0x40) { // TIMER3
	}
}

// Log(HSP v3.7): enhanced PWM settings
// Function:
//	- set PWM frequency (unit: Hz), return maximum duty cycle 
// Note: 
//	- only PWM Timer1/Timer3 support frequency update
uint16_t pwmFrequency(uint8_t pin, uint32_t fhz)
{
	uint16_t value = (uint16_t) ((F_CPU >> 1) / fhz);

	uint8_t timer = digitalPinToTimer(pin) & 0xf0;

	if((timer == 0x20) {	// TIMER1
			atomicWriteWord(&ICR1L, value);	
			/*	
			if(fmode == FREQ_BOOST) {
				sbi(TCKCSR, F2XEN);
				delayMicroseconds(10);
				sbi(TCKCSR, TC2XS1);
			} else if(bit_is_set(TCKCSR, TC2XS1)) {
				cbi(TCKCSR, TC2XS1);
				delayMicroseconds(10);
				cbi(TCKCSR, F2XEN);
			}
			*/
	} else if(timer == 0x40) { // TIMER3
			atomicWriteWord(&ICR3L, value);
	}

	return value;
}
#endif

void lgt8fx8x_init()
{
#if defined(__LGT8FX8E__)
// store ivref calibration 
	GPIOR1 = VCAL1;
	GPIOR2 = VCAL2;

#if defined(__LGT8F_SSOP20__)
	GPIOR0 = PMXCR | 0x07;
	PMXCR = 0x80;
	PMXCR = GPIOR0;
#endif

// enable 1KB E2PROM 
	ECCR = 0x80;
	ECCR = 0x40;

// clock source settings
	if((VDTCR & 0x0C) == 0x0C) {
		// switch to external crystal
		sysClock(EXT_OSC);
	} else {
		CLKPR = 0x80;
		CLKPR = 0x01;
	}
#else
	// enable 32KRC for WDT
	GPIOR0 = PMCR | 0x10;
	PMCR = 0x80;
	PMCR = GPIOR0;

	// clock scalar to 16MHz
	CLKPR = 0x80;
	CLKPR = 0x01;
#endif
}

int main(void)
{
#if defined(__LGT8F__)
	lgt8fx8x_init();
#endif	

	init();

	initVariant();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}
