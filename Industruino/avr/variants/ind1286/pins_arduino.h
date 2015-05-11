/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

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

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/


/*
  Modified 20 February 2015 by Loic De Buck
     for Industruino kits (www.industruino.com)
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7



#define TX_RX_LED_INIT          DDRB |= (1<<7)
#define TXLED1                  PORTB |= (1<<7)
#define TXLED0                  PORTB &= ~(1<<7)
#define RXLED1                  PORTB |= (1<<7)
#define RXLED0                  PORTB &= ~(1<<7)


static const uint8_t SDA = 2;
static const uint8_t SCL = 3;

// Map SPI port to 'new' pins D14..D17
static const uint8_t SS   = 17;
static const uint8_t MOSI = 16;
static const uint8_t MISO = 14;
static const uint8_t SCK  = 15;

// Mapping of analog pins as digital I/O
// A10-A13 share with digital pins



static const uint8_t A0 = 0;  //  D18
static const uint8_t A1 = 1;    // GND
static const uint8_t A2 = 2;    // GND
static const uint8_t A3 = 3;    // GND
static const uint8_t A4 = 4;    // GND
static const uint8_t A5 = 5;    // GND
static const uint8_t A6 = 6;    // GND
static const uint8_t A7 = 7;    // GND
static const uint8_t A8 = 8;    // GND
static const uint8_t A9 = 9;    // GND
static const uint8_t A10 = 10;  // D10
static const uint8_t A11 = 11;  // D11
static const uint8_t A12 = 12;  // D12
static const uint8_t A13 = 13;  // D13
static const uint8_t A14 = 14;  // GND
static const uint8_t A15 = 15;  // GND
static const uint8_t A16 = 16;  // GND
static const uint8_t A17 = 17;  // GND
static const uint8_t A18 = 18;  // GND
static const uint8_t A19 = 19;  // GND
static const uint8_t A20 = 20;  // GND
static const uint8_t A21 = 21;  // GND
static const uint8_t A22 = 22;  // GND
static const uint8_t A23 = 23;  // GND
static const uint8_t A24 = 24;  // GND
static const uint8_t A25 = 25;  // GND
static const uint8_t A26 = 26;  // GND
static const uint8_t A27 = 27;  // GND
static const uint8_t A28 = 28;  // GND
static const uint8_t A29 = 29;  // GND
static const uint8_t A30 = 30;  // GND


extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#define NUM_DIGITAL_PINS            30
#define NUM_ANALOG_INPUTS           30


// Including Timer 0

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 6 || (p) == 7 || (p) == 8)


// PCINT 0-7 are on pins 14-27
#define digitalPinToPCICR(p)    ((((p) >= 14 && (p) <= 17) || ((p) >= 23 && (p) <= 25)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 14 && (p) <= 17) || ((p) >= 23 && (p) <= 25)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 14 && (p) <= 17) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
    (uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PD, /* 0, RX */
        PD, /* 1, TX */
        PD, /* 2, SDA */
        PD, /* 3, SCL */
        PC, /* 4, D4 */
        PC, /* 5, D5 */
        PC, /* 6, D6 */
        PC, /* 7, D7 */
        PC, /* 8, D8 */
        PC, /* 9, D9 */
        PF, /* 10,D10 */
        PF, /* 11,D11 */
        PF, /* 12,D12 */
        PF, /* 13,D13 */
        PB, /* 14,MISO */
        PB, /* 15,SCLK */
        PB, /* 16,MOSI */
        PB, /* 17,SS */
        PF, /* 18 */
        PE, /* 19,D19 */
        PF, /* 20,D20*/
        PF, /* 21,D21 */
        PF, /* 22,D22 */
        PB, /* 23,D23 */
        PB, /* 24,D24 */
        PB, /* 25,D25 */
        PB, /* 26,D26 */
        PE, /* 27, D27, Jumper */
        PE, /* 28, D28, RTC crystal */
        PE, /* 29, D29, RTC crystal */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(2), /* PD2 */
        _BV(3), /* PD3 */
        _BV(1), /* PD1 */
        _BV(0), /* PD0 */
        _BV(2), /* PC2 */
        _BV(3), /* PC3 */
        _BV(4), /* PC4 */
        _BV(5), /* PC5 */
        _BV(6), /* PC6 */
        _BV(7), /* PC7 */
        _BV(7), /* PF7 */
        _BV(6), /* PF6 */
        _BV(5), /* PF5 */
        _BV(4), /* PF4 */
        _BV(3), /* PB3 */
        _BV(1), /* PB1 */
        _BV(2), /* PB2 */
        _BV(0), /* PB0 */
        _BV(3), /* PF3 */
        _BV(6), /* PE6 */
        _BV(2), /* PF2 */
        _BV(1), /* PF1 */
        _BV(0), /* PF0 */
        _BV(4), /* PB4 */
        _BV(5), /* PB5 */
        _BV(6), /* PB6 */
        _BV(7), /* PB7 */
        _BV(2), /* PE2 */
        _BV(4), /* PE4 */
        _BV(5), /* PE5 */
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER2B,
        TIMER0B,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER3C,
        TIMER3B,
        TIMER3A,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER2A,
        TIMER1A,
        TIMER1B,
        TIMER1C,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
};

// These are the actual MUX bits from the datasheet
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7
#define REFERENCE 30
#define GROUND 31


const uint8_t PROGMEM analog_pin_to_channel_PGM[30] = {
    ADC3,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    ADC7,
    ADC6,
    ADC5,
    ADC4,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    GROUND,
    REFERENCE,
};




#endif /* ARDUINO_MAIN */
#endif /* Pins_Arduino_h */



