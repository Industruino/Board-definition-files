/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

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

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
*************************************************/

/*
  Modified 26 March 2013 by Justin Mattair
     for MattairTech MT-DB-U6 boards (www.mattairtech.com)
*/

/*
  Tone.cpp now supports multiple simultaneous tone generation (one tone per timer).
  The MT-DB-U6 currently supports up to 4 simultaneous tones using timers 3, 1, 2,
  and 0 if not using the RTC, otherwise, timers 3, 1, and 0 are used for 3 tones.
  The MT-DB-U4 currently supports up to 3 simultaneous tones using timers 3, 1, and 0.
  A future release may support a fourth tone from timer 4. The MT-DB-U2 and MT-DB-U1
  support 2 simultaneous tones using timers 1 and 0. Note that timer 0 has a lower
  accuracy for tone generation because it is 8-bit (timers 3 and 1 are 16-bit). Note
  also that use of timer 0 temporarily disables the use of delay(), USB autoflushing,
  and proper USB LED handling, all of which will return to normal operation once the
  tone stops playing. Thus, timer 0 is set with the lowest priority. For example, if
  generating DTMF tones on the MT-DB-U4, timers 3 and 1 will be used. However, the
  MT-DB-U2 and MT-DB-U1 will both use timer 0 for the second tone.

  If timer 0 is used, delay() should not be called while timer 0 is generating a tone.
  Instead, use _delay_ms(), which is included with avr-libc. If sending data to the
  USB host (ie: using Serial.print()) before or during timer 0 tone generation, then
  it must be manually flushed with Serial.flush() prior to calling tone() and after
  any subsequent printing during tone generation. Otherwise, some data may not be
  sent until the tone stops and autoflushing returns to normal operation. The USB
  LED handling (if enabled) will also be disrupted during timer 0 tone generation.
  During this time, the LED will not be able to change state. If USB traffic occurs,
  the blink will be delayed until tone generation stops.
*/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
#define TCCR2A TCCR2
#define TCCR2B TCCR2
#define COM2A1 COM21
#define COM2A0 COM20
#define OCR2A OCR2
#define TIMSK2 TIMSK
#define OCIE2A OCIE2
#define TIMER2_COMPA_vect TIMER2_COMP_vect
#define TIMSK1 TIMSK
#endif

// timerx_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)

#if !defined(__AVR_ATmega8__)
volatile long timer0_toggle_count;
volatile uint8_t *timer0_pin_port;
volatile uint8_t timer0_pin_mask;
#endif

volatile long timer1_toggle_count;
volatile uint8_t *timer1_pin_port;
volatile uint8_t timer1_pin_mask;

#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__) && !defined(USE_RTC)
volatile long timer2_toggle_count;
volatile uint8_t *timer2_pin_port;
volatile uint8_t timer2_pin_mask;
#endif

#if defined(TIMSK3)
volatile long timer3_toggle_count;
volatile uint8_t *timer3_pin_port;
volatile uint8_t timer3_pin_mask;
#endif

#if defined(TIMSK4) && !defined(__AVR_ATmega32U4__)
volatile long timer4_toggle_count;
volatile uint8_t *timer4_pin_port;
volatile uint8_t timer4_pin_mask;
#endif

#if defined(TIMSK5)
volatile long timer5_toggle_count;
volatile uint8_t *timer5_pin_port;
volatile uint8_t timer5_pin_mask;
#endif


// Leave timer 0 to last.
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define AVAILABLE_TONE_PINS 1
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 2 /*, 3, 4, 5, 1, 0 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255, 255, 255, 255 */ };

#elif defined(__AVR_ATmega8__)
#define AVAILABLE_TONE_PINS 1
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 2 /*, 1 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255 */ };

#elif defined(__AVR_ATmega32U4__)
#define AVAILABLE_TONE_PINS 3
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 3, 1, 0 };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255, 255, 255 };

#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#if defined(USE_RTC)
#define AVAILABLE_TONE_PINS 3
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 3, 1, 0 };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255, 255, 255 };
#else
#define AVAILABLE_TONE_PINS 4
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 3, 1, 2, 0 };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255, 255, 255, 255 };
#endif

#elif defined(__AVR_ATmega32U2__) || defined(__AVR_AT90USB162__)
#define AVAILABLE_TONE_PINS 2
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 1, 0 };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255, 255 };

#else
#define AVAILABLE_TONE_PINS 1
const uint8_t __attribute__((section(".progmem.data"))) tone_pin_to_timer_PGM[] = { 2 /*, 1, 0*/ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255*/ };
#endif

static uint8_t timer_pins[6];


static int8_t toneBegin(uint8_t _pin)
{
  int8_t _timer = -1;

  // if we're already using the pin, the timer should be configured.  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == _pin) {
      return pgm_read_byte(tone_pin_to_timer_PGM + i);
    }
  }
  
  // search for an unused timer.
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == 255) {
      tone_pins[i] = _pin;
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
	timer_pins[_timer] = _pin;
      break;
    }
  }
  
  if (_timer != -1)
  {
    // Set timer specific stuff
    // All timers in CTC mode
    // 8 bit timers will require changing prescalar values
    // whereas 16 bit timers are set to either ck/1 or ck/64 prescalar
    switch (_timer)
    {
      #if defined(TCCR0A) && defined(TCCR0B)
      case 0:
        // 8 bit timer
	TIMSK0 &= ~(1 << TOIE0);	// disable timer/delays interrupt
	TIMSK0 &= ~(1 << OCIE0B);	// disable USB tasks interrupt
        TCCR0A = (1 << WGM01);		// CTC mode
        TCCR0B = (1 << CS00);		// ck/1
        timer0_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer0_pin_mask = digitalPinToBitMask(_pin);
        break;
      #endif

      #if defined(TCCR1A) && defined(TCCR1B) && defined(WGM12)
      case 1:
        // 16 bit timer
        TCCR1A = 0;
        TCCR1B = 0;
        bitWrite(TCCR1B, WGM12, 1);
        bitWrite(TCCR1B, CS10, 1);
        timer1_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer1_pin_mask = digitalPinToBitMask(_pin);
        break;
      #endif
      
      #if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__)
      #if defined(TCCR2A) && defined(TCCR2B)
      #if !defined(USE_RTC)
      case 2:
        // 8 bit timer
        TCCR2A = 0;
        TCCR2B = 0;
        bitWrite(TCCR2A, WGM21, 1);
        bitWrite(TCCR2B, CS20, 1);
        timer2_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer2_pin_mask = digitalPinToBitMask(_pin);
        break;
      #endif
      #endif
      #endif
      
      #if defined(TCCR3A) && defined(TCCR3B) &&  defined(TIMSK3)
      case 3:
        // 16 bit timer
        TCCR3A = 0;
        TCCR3B = 0;
        bitWrite(TCCR3B, WGM32, 1);
        bitWrite(TCCR3B, CS30, 1);
        timer3_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer3_pin_mask = digitalPinToBitMask(_pin);
        break;
      #endif

      #if defined(TCCR4A) && defined(TCCR4B) &&  defined(TIMSK4) && !defined(__AVR_ATmega32U4__)
      case 4:
        // 16 bit timer
        TCCR4A = 0;
        TCCR4B = 0;
        #if defined(WGM42)
          bitWrite(TCCR4B, WGM42, 1);
//        #elif defined(CS43)
//          #warning this may not be correct (it isn't, no CTS on U4 timer 4)
          // atmega32u4
//          bitWrite(TCCR4B, CS43, 1);
        #endif
        bitWrite(TCCR4B, CS40, 1);
        timer4_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer4_pin_mask = digitalPinToBitMask(_pin);
        break;
      #endif

      #if defined(TCCR5A) && defined(TCCR5B) &&  defined(TIMSK5)
      case 5:
        // 16 bit timer
        TCCR5A = 0;
        TCCR5B = 0;
        bitWrite(TCCR5B, WGM52, 1);
        bitWrite(TCCR5B, CS50, 1);
        timer5_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer5_pin_mask = digitalPinToBitMask(_pin);
        break;
      #endif
    }
  }

  return _timer;
}



// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  uint8_t prescalarbits = 0b001;
  long toggle_count = 0;
  uint32_t ocr = 0;
  int8_t _timer;

  _timer = toneBegin(_pin);

  if (_timer >= 0)
  {
    // Set the pinMode as OUTPUT
    pinMode(_pin, OUTPUT);
    
    if (_timer == 0)
    {
        uint16_t timer0PrescalerValues[] = { 1, 8, 64, 256, 1024 };
	prescalarbits = 0b000;

	for (uint8_t i=0; i < sizeof(timer0PrescalerValues); i++) {
          ocr = F_CPU / frequency / 2 / timer0PrescalerValues[i] - 1;
          prescalarbits++;
	  if (ocr <= 255) {
	    break;
	  }
	}

      TCCR0B = prescalarbits;
    }
    // if we are using 8 bit timer 2, scan through prescalars to find the best fit
    #if defined(TCCR2A)
      #if !defined(USE_RTC)
    else if (_timer == 2)
    {
      uint16_t timer2PrescalerValues[] = { 1, 8, 32, 64, 128, 256, 1024 };
	prescalarbits = 0b000;

	for (uint8_t i=0; i < sizeof(timer2PrescalerValues); i++) {
          ocr = F_CPU / frequency / 2 / timer2PrescalerValues[i] - 1;
          prescalarbits++;
	  if (ocr <= 255) {
	    break;
	  }
	}

      TCCR2B = prescalarbits;
    }
    #endif
    #endif
    else
    {
      // two choices for the 16 bit timers: ck/1 or ck/64
      ocr = F_CPU / frequency / 2 - 1;

      prescalarbits = 0b001;
      if (ocr > 0xffff)
      {
        ocr = F_CPU / frequency / 2 / 64 - 1;
        prescalarbits = 0b011;
      }

      if (_timer == 1)
      {
#if defined(TCCR1B)
        TCCR1B = (TCCR1B & 0b11111000) | prescalarbits;
#endif
      }
#if defined(TCCR3B)
      else if (_timer == 3)
        TCCR3B = (TCCR3B & 0b11111000) | prescalarbits;
#endif
#if defined(TCCR4B) && !defined(__AVR_ATmega32U4__)
      else if (_timer == 4)
        TCCR4B = (TCCR4B & 0b11111000) | prescalarbits;
#endif
#if defined(TCCR5B)
      else if (_timer == 5)
        TCCR5B = (TCCR5B & 0b11111000) | prescalarbits;
#endif

    }
    

    // Calculate the toggle count
    if (duration > 0)
    {
      toggle_count = 2 * frequency * duration / 1000;
    }
    else
    {
      toggle_count = -1;
    }

    // Set the OCR for the given timer,
    // set the toggle count,
    // then turn on the interrupts
    switch (_timer)
    {
#if defined(OCR0B) && defined(TIMSK0) && defined(OCIE0B)
      case 0:
        OCR0A = ocr;
        timer0_toggle_count = toggle_count;
	TIMSK0 |= (1 << OCIE0A);	// enable tone interrupt
        break;
#endif
      case 1:
#if defined(OCR1A) && defined(TIMSK1) && defined(OCIE1A)
        OCR1A = ocr;
        timer1_toggle_count = toggle_count;
        bitWrite(TIMSK1, OCIE1A, 1);
#elif defined(OCR1A) && defined(TIMSK) && defined(OCIE1A)
        // this combination is for at least the ATmega32
        OCR1A = ocr;
        timer1_toggle_count = toggle_count;
        bitWrite(TIMSK, OCIE1A, 1);
#endif
        break;

#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__)
#if defined(OCR2A) && defined(TIMSK2) && defined(OCIE2A)
#if !defined(USE_RTC)
      case 2:
        OCR2A = ocr;
        timer2_toggle_count = toggle_count;
        bitWrite(TIMSK2, OCIE2A, 1);
        break;
#endif
#endif
#endif

#if defined(TIMSK3)
      case 3:
        OCR3A = ocr;
        timer3_toggle_count = toggle_count;
        bitWrite(TIMSK3, OCIE3A, 1);
        break;
#endif

#if defined(TIMSK4) && !defined(__AVR_ATmega32U4__)
      case 4:
        OCR4A = ocr;
        timer4_toggle_count = toggle_count;
        bitWrite(TIMSK4, OCIE4A, 1);
        break;
#endif

#if defined(OCR5A) && defined(TIMSK5) && defined(OCIE5A)
      case 5:
        OCR5A = ocr;
        timer5_toggle_count = toggle_count;
        bitWrite(TIMSK5, OCIE5A, 1);
        break;
#endif

    }
  }
}


void disableTimer(uint8_t _timer)
{
  switch (_timer)
  {
    case 0:
      #if defined(TIMSK0)
          TIMSK0 &= ~(1 << OCIE0A);			// disable tone interrupt
	  TCCR0A = ((1 << WGM01) | (1 << WGM00));	// fast PWM
	  TCCR0B = ((1 << CS01) | (1 << CS00));		// ck/64
	  TIMSK0 |= (1 << TOIE0);			// re-enable timer/delay interrupt
	  TIMSK0 |= (1 << OCIE0B);			// re-enable USB tasks interrupt
      #elif defined(TIMSK)
        TIMSK = 0; // atmega32
      #endif
      break;

#if defined(TIMSK1) && defined(OCIE1A)
    case 1:
      bitWrite(TIMSK1, OCIE1A, 0);
	  TCCR1A = 0;
        TCCR1B = 0;
      
      // set timer 1 prescale factor to 64
	  TCCR1B |= (1 << CS11);
	  TCCR1B |= (1 << CS10);

	  // put timer 1 in 8-bit phase correct pwm mode
	  TCCR1A |= (1 << WGM10);

      break;
#endif

#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__)
#if !defined(USE_RTC)
    case 2:
      #if defined(TIMSK2) && defined(OCIE2A)
        bitWrite(TIMSK2, OCIE2A, 0); // disable interrupt
      #endif
      #if defined(TCCR2A) && defined(WGM20)
        TCCR2A = (1 << WGM20);
      #endif
      #if defined(TCCR2B) && defined(CS22)
        TCCR2B = (TCCR2B & 0b11111000) | (1 << CS22);
      #endif
      #if defined(OCR2A)
        OCR2A = 0;
      #endif
        break;
#endif
#endif

#if defined(TIMSK3)
    case 3:
	bitWrite(TIMSK3, OCIE3A, 0);
	TCCR3A = 0;
      TCCR3B = 0;
	TCCR3B |= (1 << CS31);		// set timer 3 prescale factor to 64
	TCCR3B |= (1 << CS30);
	TCCR3A |= (1 << WGM30);		// put timer 3 in 8-bit phase correct pwm mode
    break;
#endif

#if defined(TIMSK4) && !defined(__AVR_ATmega32U4__)
    case 4:
	bitWrite(TIMSK4, OCIE4A, 0);
      break;
#endif

#if defined(TIMSK5)
    case 5:
      TIMSK5 = 0;
      break;
#endif
  }
}


void noTone(uint8_t _pin)
{
  int8_t _timer = -1;
  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == _pin) {
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      tone_pins[i] = 255;
    }
  }
  
  disableTimer(_timer);

  digitalWrite(_pin, 0);
}


#if !defined(__AVR_ATmega8__)
ISR(TIMER0_COMPA_vect)
{
  if (timer0_toggle_count != 0)
  {
    // toggle the pin
    *timer0_pin_port ^= timer0_pin_mask;

    if (timer0_toggle_count > 0)
      timer0_toggle_count--;
  }
  else
  {
    noTone(timer_pins[0]);
  }
}
#endif

ISR(TIMER1_COMPA_vect)
{
  if (timer1_toggle_count != 0)
  {
    // toggle the pin
    *timer1_pin_port ^= timer1_pin_mask;

    if (timer1_toggle_count > 0)
      timer1_toggle_count--;
  }
  else
  {
    noTone(timer_pins[1]);
  }
}

#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__)
#if !defined(USE_RTC)
ISR(TIMER2_COMPA_vect)
{

  if (timer2_toggle_count != 0)
  {
    // toggle the pin
    *timer2_pin_port ^= timer2_pin_mask;

    if (timer2_toggle_count > 0)
      timer2_toggle_count--;
  }
  else
  {
    noTone(timer_pins[2]);
  }
}
#endif
#endif

#if !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__)
ISR(TIMER3_COMPA_vect)
{
  if (timer3_toggle_count != 0)
  {
    // toggle the pin
    *timer3_pin_port ^= timer3_pin_mask;

    if (timer3_toggle_count > 0)
      timer3_toggle_count--;
  }
  else
  {
    noTone(timer_pins[3]);
  }
}
#endif

#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__) && !defined(__AVR_AT90USB646__) && !defined(__AVR_AT90USB1286__)
ISR(TIMER4_COMPA_vect)
{
  if (timer4_toggle_count != 0)
  {
    // toggle the pin
    *timer4_pin_port ^= timer4_pin_mask;

    if (timer4_toggle_count > 0)
      timer4_toggle_count--;
  }
  else
  {
    noTone(timer_pins[4]);
  }
}
#endif

#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega32U2__) && !defined(__AVR_AT90USB162__) && !defined(__AVR_AT90USB646__) && !defined(__AVR_AT90USB1286__)
ISR(TIMER5_COMPA_vect)
{
  if (timer5_toggle_count != 0)
  {
    // toggle the pin
    *timer5_pin_port ^= timer5_pin_mask;

    if (timer5_toggle_count > 0)
      timer5_toggle_count--;
  }
  else
  {
    noTone(timer_pins[5]);
  }
}
#endif
