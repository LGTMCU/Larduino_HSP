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
	*(p + 1) = (uint8_t)(val >> 8);
	nop(); nop(); nop();
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
//		- PWM_MODE_COMPM2: complementary dual output 
//		- PWM_MODE_COMPM3: complementary dual output (inverted)
//	fmode: pwm frequency settings
//		- PWM_FREQ_SLOW: slow range
//		- PWM_FREQ_NORMAL: normal range
//		- PWM_FREQ_FAST: fast range 
//		- PWM_FREQ_BOOST: boost target frequency by x4
//	dband: dead band settings
//		- only valid for complementary working mode 
// note:
//		- Timer 2 is used for system tick, so don't touch!!
//static uint8_t tmr1_boost_en = 0;
//static uint8_t tmr3_boost_en = 0;

void pwmMode(uint8_t pin, uint8_t wmode, uint8_t fmode, uint8_t dband)
{
	volatile uint8_t *pTCCRX = 0;

	uint8_t timer = digitalPinToTimer(pin) & 0xf0;

	if(timer == TIMER0) { // TIMER0
		pTCCRX = &TCCR0B;
		if(wmode == PWM_MODE_NORMAL) {
			cbi(TCCR0B, DTEN0);
			cbi(TCCR0A, COM0B0);
		} else {
			sbi(TCCR0B, DTEN0);
			TCCR0A = (TCCR0A & ~_BV(COM0B0)) | (wmode & 0x10);
			DTR0 = ((dband & 0xf) << 4) | (dband & 0xf);
		}

		if((fmode & PWM_FREQ_BOOST) == PWM_FREQ_BOOST) {
			// enable frequency boost (x4) mode
			sbi(TCKCSR, F2XEN);
			delayMicroseconds(10);
			sbi(TCKCSR, TC2XS0);					
		} else if(bit_is_set(TCKCSR, TC2XS0)) {
			cbi(TCKCSR, TC2XS0);
			delayMicroseconds(10);
			cbi(TCKCSR, F2XEN);				
		}
	} else if(timer == TIMER1) { // TIMER1
		pTCCRX = &TCCR1B;
		if(wmode == PWM_MODE_NORMAL) {
			cbi(TCCR1C, DTEN1);
			cbi(TCCR1A, COM1B0);
		} else {
			sbi(TCCR1C, DTEN1);
			TCCR1A = (TCCR1A & ~_BV(COM1B0)) | (wmode & 0x10);
			DTR1L = dband;
			DTR1H = dband;
		}
		if((fmode & PWM_FREQ_BOOST) == PWM_FREQ_BOOST) {
			sbi(TCKCSR, F2XEN);
			delayMicroseconds(10);
			sbi(TCKCSR, TC2XS1);
		} else if(bit_is_set(TCKCSR, TC2XS1)) {
			cbi(TCKCSR, TC2XS1);
			delayMicroseconds(10);
			cbi(TCKCSR, F2XEN);
		}		
	} else if(timer == TIMER3) { // TIMER3
		pTCCRX = &TCCR3B;
		if(wmode == PWM_MODE_NORMAL) {
			cbi(TCCR3C, DTEN3);
			cbi(TCCR3A, COM3B0);
		} else {
			sbi(TCCR3C, DTEN3);
			TCCR3A = (TCCR3A & ~_BV(COM3B0)) | (wmode & 0x10);
			DTR3A = dband;
			DTR3B = dband;
		}
	}

	if(pTCCRX == 0) return;

	if((fmode & 0x7f) == PWM_FREQ_SLOW) {
		*pTCCRX = (*pTCCRX & 0xf8) | PWM_FREQ_SLOW;	// prescale = 1024 (slowest mode)
	} else if((fmode & 0x7f) == PWM_FREQ_FAST) {
		*pTCCRX = (*pTCCRX & 0xf8) | PWM_FREQ_FAST; // prescale = 1 (fastest mode)
	} else if ((fmode & 0x7f) == PWM_FREQ_NORMAL) {
		*pTCCRX = (*pTCCRX & 0xf8) | PWM_FREQ_NORMAL;	// prescale = 64 (default)
	}
}

// Log(HSP v3.7): enhanced PWM settings
// Function:
//	- set PWM frequency (unit: Hz), return maximum duty cycle 
// Note: 
//	- only PWM Timer1/Timer3 support frequency update
uint16_t pwmFrequency(uint8_t pin, uint32_t fhz)
{
	uint16_t icrx = 0;
	uint8_t csxs = 0;
	uint8_t boost = 0;
	volatile uint8_t *pICRX = 0;

	uint8_t timer = digitalPinToTimer(pin) & 0xf0;

	// Note for TIMER0 
	// ============================================================================
	// timer 0 working in FPWM mode which TOP is fixed to 0xFF
	// so we can change its prescale to set frequency range (fast/normal/slow)
	// fast mode:	16000000/(1*256) = 62.5K, support boost up to 62.5x4 = 250KHz
	// normal mode:	16000000/(64*256) = 976Hz, support boost up to 3.9KHz
	// slow mode:	16000000/(1024*256) = 61Hz, support boost up to 244Hz
	// ============================================================================

	if(timer == TIMER1) { // TIMER1
		pICRX = &ICR1L;
		csxs = TCCR1B & 0x7;
		boost = bit_is_set(TCKCSR, TC2XF1);
	} else if(timer == TIMER3) { // TIMER3
		pICRX = &ICR3L;
		csxs = TCCR3B & 0x7;
	}

	if(pICRX == 0) return 0xff;

	// DO NOT try to merge the two cases, compiler will try to 
	// optimize the divider if either of oprands is constant value
	if(boost == 0) {
		if(csxs == PWM_FREQ_FAST) { // fast mode
			icrx = (uint16_t) ((F_CPU >> 1) / fhz);
		} else if(csxs == PWM_FREQ_NORMAL) { // normal mode
			icrx = (uint16_t) ((F_CPU >> 7) / fhz);
		} else if(csxs == PWM_FREQ_SLOW) { // slow mode
			icrx = (uint16_t) ((F_CPU >> 11) / fhz);
		}
	} else {
		if(csxs == PWM_FREQ_FAST) { // fast mode
			icrx = (uint16_t) ((64000000UL >> 1) / fhz);
		} else if(csxs == PWM_FREQ_NORMAL) { // normal mode
			icrx = (uint16_t) ((64000000UL >> 7) / fhz);
		} else if(csxs == PWM_FREQ_SLOW) { // slow mode
			icrx = (uint16_t) ((64000000UL >> 11) / fhz);
		}	
	}
	
	atomicWriteWord(pICRX, icrx);

	return icrx;
}

// Log(HSP v3.7):
// Function:
//	- return frequency (in Hz) by give PWM resolution (bits width of duty)
// Note: 
//	- timer0/2 works in FPWM mode, pwm frequency is fixed by given mode
//	- timer1/3 works in PCPWM mode, means frequency reduced by a half
uint32_t pwmResolution(uint8_t pin, uint8_t resBits)
{
	uint8_t csxs = 0;
	uint8_t boost = 0;
	uint32_t freq = 0x0UL;

	uint8_t timer = digitalPinToTimer(pin) & 0xf0;

	if(timer != TIMER1 && timer != TIMER3)
		return 0x0UL;

	if(timer == TIMER1) { // TIMER1
		csxs = TCCR1B & 0x7;
		boost = bit_is_set(TCKCSR, TC2XF1);
	} else if(timer == TIMER3) { // TIMER3
		csxs = TCCR3B & 0x7;
	}	
	
	if(boost != 0) {
		if(csxs == PWM_FREQ_FAST) {
			freq = (64000000UL >> 1) / (1 << resBits);
		} else if(csxs == PWM_FREQ_SLOW) {
			freq = (64000000UL >> 11) / (1 << resBits);
		} else { // PWM_FREQ_NORMAL
			freq = (64000000UL >> 7) / (1 << resBits);
		}
	} else {
		if(csxs == PWM_FREQ_FAST) {
			freq = (F_CPU >> 1) / (1 << resBits);
		} else if(csxs == PWM_FREQ_SLOW) {
			freq = (F_CPU >> 11) / (1 << resBits);
		} else { // PWM_FREQ_NORMAL
			freq = (F_CPU >> 7) / (1 << resBits);
		}
	}

	// update pwm frequency
	pwmFrequency(pin, freq);

	return freq;
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
