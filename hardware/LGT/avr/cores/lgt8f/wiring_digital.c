/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  Modified 28 September 2010 by Mark Sproul

  $Id: wiring.c 248 2007-02-03 15:36:30Z mellis $
*/

#define ARDUINO_MAIN
#include "wiring_private.h"
#include "pins_arduino.h"

void pinMode(uint8_t pin, uint8_t mode)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

#if defined(__LGT8FX8E__)
	if(mode == ANALOG) {
		if(pin == DAC0) {
			GPIOR0 = IOCR | 0x08;
			IOCR = 0x80;
			IOCR = GPIOR0;
			ACSR |= 0x40;		
		} else if(pin == DAC1) {
			GPIOR0 = IOCR | 0x10;
			IOCR = 0x80;
			IOCR = GPIOR0;
			AC1SR |= 0x40;
		}
	}
#elif defined(__LGT8FX8P__)
	if((mode == ANALOG) && (pin == DAC0)) { // enable DAC
		DACON |= 0xC;
	} else if(pin == DAC0) {
		DACON = 0x3;	// disable DAC
	} else if(pin == E0 || pin == E2) {
		MCUSR = 0xff;
		MCUSR = 0xff;	// disable SWD/SWC for E0/E2	
	} else if(pin == E6) {
		GPIOR0 = PMX2 | 0x2;
		PMX2 = 0x80;	
		PMX2 = GPIOR0;	// enable PE6 for GPIO
	}
#endif

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else if(LGT_NOT_DACO(pin) || (mode == OUTPUT)) {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}

// Forcing this inline keeps the callers from having to push their own stuff
// on the stack. It is a good performance win and only takes 1 more byte per
// user than calling. (It will take more bytes on the 168.)
//
// But shouldn't this be moved into pinMode? Seems silly to check and do on
// each digitalread or write.
//
// Mark Sproul:
// - Removed inline. Save 170 bytes on atmega1280
// - changed to a switch statment; added 32 bytes but much easier to read and maintain.
// - Added more #ifdefs, now compiles for atmega645
//
//static inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline));
//static inline void turnOffPWM(uint8_t timer)
static void turnOffPWM(uint8_t timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TIMER0B) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

void digitalWrite(uint8_t pin, uint8_t val)
{
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

int digitalRead(uint8_t pin)
{
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

void digitalToggle(uint8_t pin)
{
#if defined(__LGT8FX8P__)
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	*portInputRegister(port) = bit;
#elif defined(__LGT8FX8E__)
	if(digitalRead(pin) == LOW) {
		digitalWrite(pin, HIGH);
	} else {
		digitalWrite(pin, LOW);
	}
#endif
}

// Log(HSP v3.7): Exntenced PWM output
// 
#if defined(__LGT8FX8P__) || defined(__LGT8F8XE__)
// Note: it's still compatible with analogWrite() for PWM funciton
void pwmWrite(uint8_t pin, uint16_t val)
{
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.

	// duty cycle validation: settings should not overflow of PWM frequency
	uint16_t icr1_value = (ICR1H << 8) + ICR1L;

	if(val > icr1_value) 
		val = icr1_value;
	
	switch(digitalPinToTimer(pin)) {
		// XXX fix needed for atmega8
		#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		case TIMER0A:
			// connect pwm to pin on timer 0
			sbi(TCCR0, COM00);
			OCR0 = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A: // D6
			// connect pwm to pin on timer 0, channel A
			sbi(DDRD, PD6);
			cbi(TCCR0B, OC0AS);
			sbi(TCCR0A, COM0A1);
			OCR0A = val; // set pwm duty
			break;
		#if defined(__LGTF8XP48__)			
		case TIMER0AX: // E4
			sbi(DDRE, PE4);
			sbi(TCCR0B, OC0AS);
			sbi(TCCR0A, COM0A1);
			OCR0A = val;
			break;
		#endif
		#endif
		#if defined(TCCR0A) && defined(COM0B1)
		case TIMER0B: // D5
			// connect pwm to pin on timer 0, channel B
		#if defined(__LGTF8XP48__)
			unlockWrite(&PMX0, (PMX0 & ~_BV(C0BF3)));
		#endif		
			sbi(DDRD, PD5);
			sbi(TCCR0A, COM0B1);
			OCR0B = val; // set pwm duty
			break;
		#if defined(__LGTF8XP48__)
		case TIMER0BX: // F3
			sbi(DDRF, PF3);
			unlockWrite(&PMX0, (PMX0 | _BV(C0BF3)));
			sbi(TCCR0A, COM0B1);
			OCR0B = val;
			break;
		#endif
		#endif

		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A: // B1
			// connect pwm to pin on timer 1, channel A
			sbi(TCCR1A, COM1A1);
			//OCR1A = val; // set pwm duty
			atomicWriteWord(&OCR1AL, val);
			#if defined(__LGT8FX8P__)	
			unlockWrite(&PMX0, (PMX0 & ~_BV(C1AF5)));
			sbi(DDRB, PB2);
			#endif			
			break;
		#if defined(__LGT8FX8P__)
			// F5 for LGT8F328P/QFP48
			// E5 for LGT8F328P/QFP32 (tied with F5)
		case TIMER1AX: 
		#if defined (__LGT8F328P32__)
			cbi(DDRE, PE5);
		#endif	
			sbi(TCCR1A, COM1A1);
			//OCR1A = val;
			atomicWriteWord(&OCR1AL, val);
			unlockWrite(&PMX0, (PMX0 | _BV(C1AF5)));
			sbi(DDRF, PF5);			
		#endif	
		#endif

		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B: // B2
			// connect pwm to pin on timer 1, channel B			
			sbi(TCCR1A, COM1B1);
			//OCR1B = val; // set pwm duty
			atomicWriteWord(&OCR1BL, val);
			#if defined(__LGT8FX8P__)
			unlockWrite(&PMX0, (PMX0 & ~_BV(C1BF4)));		
			sbi(DDRB, PB2);
			#endif			
			break;
		
		#if defined(__LGT8FX8P__)
		// F4 for LGT8F328P/QFP48
		// E4 for LGT8F328P/QFP32 (tied with F4)
		case TIMER1BX:	
		#if defined(__LGT8FX8P32__)
			cbi(DDRE, PE4);
		#endif
			unlockWrite(&PMX0, (PMX0 | _BV(C1BF4)));
			sbi(TCCR1A, COM1B1);
			//OCR1B = val;
			atomicWriteWord(&OCR1BL, val);
			sbi(DDRF, PF4);	
			break;
		#endif
		#endif

		#if defined(TCCR2) && defined(COM21)
		case TIMER2:
			// connect pwm to pin on timer 2
			sbi(TCCR2, COM21);
			OCR2 = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2A: // B3
			// connect pwm to pin on timer 2, channel A
		#if defined(__LGT8FX8P48__)
			//cbi(DDRF, PF6);
			unlockWrite(&PMX1, (PMX1 & ~_BV(C2AF6)));
		#endif
			sbi(DDRB, PB3);
			sbi(TCCR2A, COM2A1);
			OCR2A = val; // set pwm duty
			break;
		#if defined(__LGT8F8P48__)
		case TIMER2AX: // F6
			
			break;
		#endif	
		#endif

		#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2B: // D3
			// connect pwm to pin on timer 2, channel B
			sbi(TCCR2A, COM2B1);
			OCR2B = val; // set pwm duty
			break;
		#if defined(__LGT8F8P48__)
		case TIMER2BX: // F7
			break;
		#endif
		#endif

		#if defined(TCCR3A) && defined(COM3A1)
		case TIMER3A:
			// connect pwm to pin on timer 3, channel A
			sbi(TCCR3A, COM3A1);
			OCR3A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3B:
			// connect pwm to pin on timer 3, channel B
			sbi(TCCR3A, COM3B1);
			OCR3B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3C1)
		case TIMER3C:
			// connect pwm to pin on timer 3, channel C
			sbi(TCCR3A, COM3C1);
			OCR3C = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR4A)
		case TIMER4A:
			//connect pwm to pin on timer 4, channel A
			sbi(TCCR4A, COM4A1);
			#if defined(COM4A0)		// only used on 32U4
			cbi(TCCR4A, COM4A0);
			#endif
			OCR4A = val;	// set pwm duty
			break;
		#endif
		
		#if defined(TCCR4A) && defined(COM4B1)
		case TIMER4B:
			// connect pwm to pin on timer 4, channel B
			sbi(TCCR4A, COM4B1);
			OCR4B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR4A) && defined(COM4C1)
		case TIMER4C:
			// connect pwm to pin on timer 4, channel C
			sbi(TCCR4A, COM4C1);
			OCR4C = val; // set pwm duty
			break;
		#endif
			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:				
			// connect pwm to pin on timer 4, channel D
			sbi(TCCR4C, COM4D1);
			#if defined(COM4D0)		// only used on 32U4
			cbi(TCCR4C, COM4D0);
			#endif
			OCR4D = val;	// set pwm duty
			break;
		#endif
						
		#if defined(TCCR5A) && defined(COM5A1)
		case TIMER5A:
			// connect pwm to pin on timer 5, channel A
			sbi(TCCR5A, COM5A1);
			OCR5A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5B1)
		case TIMER5B:
			// connect pwm to pin on timer 5, channel B
			sbi(TCCR5A, COM5B1);
			OCR5B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5C1)
		case TIMER5C:
			// connect pwm to pin on timer 5, channel C
			sbi(TCCR5A, COM5C1);
			OCR5C = val; // set pwm duty
			break;
		#endif
		#if defined(__LGT8FX8E__) || defined(__LGT8FX8P__)
		case LGTDAO0:
			DAL0 = val; 
			break;
		#endif
		#if defined(__LGT8FX8E__)
		case LGTDAO1:
			DAL1 = val;
			break;
		#endif
		case NOT_ON_TIMER:
		default:
			if (val < 128) {
				digitalWrite(pin, LOW);
			} else {
				digitalWrite(pin, HIGH);
			}
	}
}

#endif // __LGT8FX8E__ || __LGT8FX8P__
