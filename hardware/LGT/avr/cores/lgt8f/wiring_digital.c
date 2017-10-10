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

	// Log(HSP v3.7): 
	//  - turn off pwm output for we want to control i/o directly.
	//  - note: analogWrite/pwmWrite will open i/o for output automatically.
	pwmTurnOff(pin);
	// Log(HSP v3.7): END

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
static void __turnOffPWM(uint8_t timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1AX: 
		#if defined(__LGT8FX8P32__)
			cbi(DDRF, PF5); // turn/off i/o of PWM
			sbi(DDRE, PE5); // turn/on i/o of user view
		#endif
		case TIMER1A: cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1BX:
		#if defined(__LGT8FX8P32__)
			cbi(DDRF, PF4); // turn/off i/o of PWM
			sbi(DDRE, PE4); // turn/on i/o of user view
		#endif
		case TIMER1B: cbi(TCCR1A, COM1B1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2: cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A:
		case TIMER0AX: cbi(TCCR0A, COM0A1);    break;
		#endif		
		#if defined(TIMER0B) && defined(COM0B1)
		case TIMER0B:
		case TIMER0BX: cbi(TCCR0A, COM0B1);    break;
		#endif

		#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2AX:
		case TIMER2A: cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2BX:
		case TIMER2B: cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)		
		case  TIMER3A: // only for LGT8FX8P/QFP32/(SSOP20)?
		#if defined(__LGT8FX8P32__)
			cbi(DDRF, PF1);	// turn off i/o of pwm
			sbi(DDRD, PD1); // turn on i/o of user view
		#endif  
		case TIMER3AX:
		case TIMER3AA: cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3B: 
		#if defined(__LGT8FX8P32__)
			cbi(DDRF, PF2); // turn off i/o of pwm
			sbi(DDRD, PD2); // turn on i/o of user view
		#endif
		case TIMER3BX: cbi(TCCR3A, COM3B1);    break;
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

// Log(HSP v3.7):
// - turn off pwm output
// - set i/o to proper driver state
void pwmTurnOff(uint8_t pin)
{
	uint8_t timer = digitalPinToTimer(pin);
	if(timer != NOT_ON_TIMER) __turnOffPWM(timer);
}
// Log(HSP v3.7): END

void digitalWrite(uint8_t pin, uint8_t val)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	// Log(HSP v3.7): 
	//  - move to pinMode() to make I/O writer more faster
	//  - it should be no problem to turn/off PWM if we set i/o direction explicitly!
	//uint8_t timer = digitalPinToTimer(pin);
	//if(timer != NOT_ON_TIMER) __turnOffPWM(timer);
	// Log(HSP v3.7): END

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
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	// Log(HSP v3.7): 
	//  - think that: why we do read pin if we set it to pwm mode ??
	//	- Maybe, it should be in case of some very special application,
	//  - so, it's your matter to turn/off pwm before read.
	//	- You can turn/off pwm by pwmTurnOff() or pinMode()
	// The More:
	//	- Infect, we can read pin even in pwm cycle. that is useful if we want to
	//	- test current pwm output status (if we run fast enough!!)
	//uint8_t timer = digitalPinToTimer(pin);
	//if (timer != NOT_ON_TIMER) pwmTurnOff(timer);
	// Log(HSP v3.7): END

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
// Note: you can keep on use analogWrite() for compatible purpose!
#if defined(__LGT8FX8P__) || defined(__LGT8F8XE__)
void pwmWrite(uint8_t pin, uint16_t val)
{
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.

	// duty cycle validation: settings should not overflow of PWM frequency
	
	switch(digitalPinToTimer(pin)) {
		// XXX fix needed for atmega8
		#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		case TIMER0A:
			// connect pwm to pin on timer 0
			sbi(TCCR0, COM00);
			OCR0 = (uint8_t)val; // set pwm duty
			break;
		#endif

		#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A: // D6
			// connect pwm to pin on timer 0, channel A
			OCR0A = (uint8_t)val; // set pwm duty
			sbi(TCCR0A, COM0A1);
			cbi(TCCR0B, OC0AS);	//*****
			sbi(DDRD, PD6);
			break;
		#if defined(__LGT8FXP48__)			
		case TIMER0AX: // E4
			OCR0A = (uint8_t)val;						
			sbi(TCCR0A, COM0A1);
			sbi(TCCR0B, OC0AS);	 //*****
			sbi(DDRE, PE4);
			break;
		#endif
		#endif
		#if defined(TCCR0A) && defined(COM0B1)
		case TIMER0B: // D5
			// connect pwm to pin on timer 0, channel B			
			OCR0B = (uint8_t)val; // set pwm duty			
			sbi(TCCR0A, COM0B1);				
			#if defined(__LGT8FXP48__)
			unlockWrite(&PMX0, (PMX0 & ~_BV(C0BF3)));
			#endif					
			sbi(DDRD, PD5);
			break;
		#if defined(__LGT8FXP48__)
		case TIMER0BX: // F3
			OCR0B = (uint8_t)val;
			sbi(TCCR0A, COM0B1);
			unlockWrite(&PMX0, (PMX0 | _BV(C0BF3)));
			sbi(DDRF, PF3);
			break;
		#endif
		#endif

		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A: // B1
			// connect pwm to pin on timer 1, channel A
			//OCR1A = val; // set pwm duty
			atomicWriteWord(&OCR1AL, val);
			sbi(TCCR1A, COM1A1);
			#if defined(__LGT8FX8P__)	
			unlockWrite(&PMX0, (PMX0 & ~_BV(C1AF5)));
			#endif			
			sbi(DDRB, PB1);
			break;
		#if defined(__LGT8FX8P__)
			// F5 for LGT8F328P/QFP48
			// E5 for LGT8F328P/QFP32 (tied with F5)
		case TIMER1AX: 
		#if defined (__LGT8FX8P32__)
			cbi(DDRE, PE5);
		#endif	
			//OCR1A = val;
			atomicWriteWord(&OCR1AL, val);
			sbi(TCCR1A, COM1A1);
			unlockWrite(&PMX0, (PMX0 | _BV(C1AF5)));
			sbi(DDRF, PF5);			
		#endif	
		#endif

		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B: // B2
			// connect pwm to pin on timer 1, channel B			
			//OCR1B = val; // set pwm duty
			atomicWriteWord(&OCR1BL, val);
			sbi(TCCR1A, COM1B1);
			#if defined(__LGT8FX8P__)
			unlockWrite(&PMX0, (PMX0 & ~_BV(C1BF4)));		
			#endif			
			sbi(DDRB, PB2);
			break;
		
		#if defined(__LGT8FX8P__)
		// F4 for LGT8F328P/QFP48
		// E4 for LGT8F328P/QFP32 (tied with F4)
		case TIMER1BX:	
		#if defined(__LGT8FX8P32__)
			cbi(DDRE, PE4);
		#endif
			//OCR1B = val;
			atomicWriteWord(&OCR1BL, val);
			sbi(TCCR1A, COM1B1);			
			unlockWrite(&PMX0, (PMX0 | _BV(C1BF4)));
			sbi(DDRF, PF4);	
			break;
		#endif
		#endif

		#if defined(TCCR2) && defined(COM21)
		case TIMER2:
			// connect pwm to pin on timer 2
			OCR2 = (uint8_t)val; // set pwm duty			
			sbi(TCCR2, COM21);
			break;
		#endif

		#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2A: // B3
			// connect pwm to pin on timer 2, channel A
			OCR2A = (uint8_t)val; // set pwm duty
			sbi(TCCR2A, COM2A1);			
			#if defined(__LGT8FX8P48__)
			unlockWrite(&PMX1, (PMX1 & ~_BV(C2AF6)));
			#endif			
			sbi(DDRB, PB3);			
			break;
		#if defined(__LGT8F8P48__)
		case TIMER2AX: // F6
			OCR2A = (uint8_t)val;			
			sbi(TCCR2A, COM2A1);
			unlockWrite(&PMX1, (PMX1 | _BV(C2AF6)));
			sbi(DDRF, PF6);
			break;
		#endif	
		#endif

		#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2B: // D3
			// connect pwm to pin on timer 2, channel B	
			OCR2B = (uint8_t)val; // set pwm duty
			sbi(TCCR2A, COM2B1);
			#if defined(__LGT8FX8P48__)
			unlockWrite(&PMX1, (PMX1 & ~_BV(C2BF7)));
			#endif
			sbi(DDRD, PD3);
			break;
		#if defined(__LGT8F8P48__)
		case TIMER2BX: // F7
			OCR2B = (uint8_t)val;
			sbi(TCCR2A, COM2B1);
			unlockWrite(&PMX1, (PMX1 | _BV(C2BF7)));
			sbi(DDRF, PF7);
			break;
		#endif
		#endif

		#if defined(TCCR3A) && defined(COM3A1)
		case TIMER3A: // D1 tied with F1
			// connect pwm to pin on timer 3, channel A
			cbi(UCSR0B, TXEN0);
			cbi(DDRD, PD1);
			atomicWriteWord(&OCR3AL, val);
			sbi(TCCR3A, COM3A1);
			sbi(DDRF, PF1);			
			break;
		#if defined(__LGT8FX8P48__)
		case TIMER3AX: // F1 standalone
			atomicWriteWord(&OCR3AL, val);
			sbi(TCCR3A, COM3A1);
			unlockWrite(&PMX1, (PMX1 & ~_BV(C3AC)));
			sbi(DDRF, PF1);
			break;
		case TIMER3AA: // OC3A/ACO standalone
			atomicWriteWord(&OCR3AL, val);
			unlockWrite(&PMX1, (PMX1 | _BV(C3AC)));
			sbi(TCCR3A, COM3A1);
			break;
		#endif	
		#endif

		#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3BX: // F2 tied with D2
		#if defined(__LGT8FX8P32__)
		case TIMER3B:
			cbi(DDRD, PD2);
		#endif
			// connect pwm to pin on timer 3, channel B
			//OCR3B = val; // set pwm duty
			atomicWriteWord(&OCR3BL, val);
			sbi(TCCR3A, COM3B1);
			sbi(DDRF, PF2);
			break;
		#endif

		#if defined(__LGT8FX8P48__)
		#if defined(TCCR3A) && defined(COM3C1)
		case TIMER3C: // F3 
			// connect pwm to pin on timer 3, channel C
			//OCR3C = val; // set pwm duty
			atomicWriteWord(&OCR3CL, val);
			sbi(TCCR3A, COM3C1);
			sbi(DDRF, PF3);
			break;
		#endif
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
