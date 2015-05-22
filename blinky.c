/*

 Blinky Earrings
 Copyright 2013 Nicholas W. Sayer
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 
*/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

// Where in the EEPROM do we store the pattern number?
#define EE_PATTERN_NUM 0

PROGMEM volatile unsigned char * const LED_PORT[] = { &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTB, &PORTB, &PORTB };
PROGMEM const unsigned char LED_PIN[] = { PA0, PA1, PA2, PA3, PA7, PB0, PB1, PB2 };

#define BUTTON_PORT PINA
#define BUTTON_MASK _BV(PINA5)

// If we see any state change on the button, we ignore all changes for this long
#define BUTTON_DEBOUNCE_INTERVAL 50
// How long does the button have to stay down before we call it a LONG push?
#define BUTTON_LONG_START 250

unsigned int pattern;

struct pattern_entry {
  unsigned char mask;
  unsigned long duration;
};

// Circle in 800 ms, sleep 1 second.
PROGMEM const struct pattern_entry circle_pattern[] = {
  { 0B00000001, 50 },
  { 0, 50 },
  { 0B00000010, 50 },
  { 0, 50 },
  { 0B00000100, 50 },
  { 0, 50 },
  { 0B00001000, 50 },
  { 0, 50 },
  { 0B00010000, 50 },
  { 0, 50 },
  { 0B00100000, 50 },
  { 0, 50 },
  { 0B01000000, 50 },
  { 0, 50 },
  { 0B10000000, 50 },
  { 0, 1000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry back_circle_pattern[] = {
  { 0B10000000, 50 },
  { 0, 50 },
  { 0B01000000, 50 },
  { 0, 50 },
  { 0B00100000, 50 },
  { 0, 50 },
  { 0B00010000, 50 },
  { 0, 50 },
  { 0B00001000, 50 },
  { 0, 50 },
  { 0B00000100, 50 },
  { 0, 50 },
  { 0B00000010, 50 },
  { 0, 50 },
  { 0B00000001, 50 },
  { 0, 1000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry skip_around[] = {
  { 0B10000000, 50 },
  { 0, 50 },
  { 0B00010000, 50 },
  { 0, 50 },
  { 0B00000010, 50 },
  { 0, 50 },
  { 0B01000000, 50 },
  { 0, 50 },
  { 0B00001000, 50 },
  { 0, 50 },
  { 0B00000001, 50 },
  { 0, 50 },
  { 0B00100000, 50 },
  { 0, 50 },
  { 0B00000100, 50 },
  { 0, 5000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry blink_all[] = {
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0, 2000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry pulse_all[] = {
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0B10000000, 2 },
  { 0B00000010, 2 },
  { 0B00010000, 2 },
  { 0B00001000, 2 },
  { 0B00100000, 2 },
  { 0B00000100, 2 },
  { 0B01000000, 2 },
  { 0B00000001, 2 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 10 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 20 },
  { 0B10000000, 1 },
  { 0B00000010, 1 },
  { 0B00010000, 1 },
  { 0B00001000, 1 },
  { 0B00100000, 1 },
  { 0B00000100, 1 },
  { 0B01000000, 1 },
  { 0B00000001, 1 },
  { 0, 1000 },
  { 0, 0 }
};

PROGMEM struct pattern_entry* const patterns[] = {
  (struct pattern_entry* const)circle_pattern,
  (struct pattern_entry* const)back_circle_pattern,
  (struct pattern_entry* const)skip_around,
  (struct pattern_entry* const)blink_all,
  (struct pattern_entry* const)pulse_all
};
// How many patterns are there?
#define PATTERN_COUNT 5
//#define PATTERN_COUNT (sizeof(patterns) / sizeof(struct pattern_entry *))

#define EVENT_NONE 0
#define EVENT_SHORT_PUSH 1
#define EVENT_LONG_PUSH 2

volatile unsigned long millis_counter;
volatile unsigned int irq_cycle_pos;

int place_in_pattern;
unsigned long milli_of_next_act, button_debounce_time, button_press_time;
unsigned char ignoring_button;

// To turn 1 MHz into 1 kHz, we divide by 64 with the divisor, then by 15 5/8.
#define LONG_IRQ_CYCLE_COUNT 5
#define TOTAL_IRQ_CYCLE_COUNT 8
// The counting is zero based *and* inclusive
#define IRQ_CYCLE_LENGTH (15 - 1)

EMPTY_INTERRUPT(PCINT0_vect)

ISR(TIM0_COMPA_vect) {
  millis_counter++;
  irq_cycle_pos++;
  if (irq_cycle_pos == LONG_IRQ_CYCLE_COUNT) OCR0A = IRQ_CYCLE_LENGTH; // set short divisor
  if (irq_cycle_pos >= TOTAL_IRQ_CYCLE_COUNT) {
    OCR0A = IRQ_CYCLE_LENGTH + 1;
    irq_cycle_pos = 0;
  }
}

unsigned long millis() {
  // We need to be careful to prevent the ISR from
  // altering the value while we're reading it.
  cli();
  unsigned long out = millis_counter;
  sei();
  return out;
}

unsigned int checkEvent() {
  unsigned long now = millis();
  if (button_debounce_time != 0) {
    if (now - button_debounce_time < BUTTON_DEBOUNCE_INTERVAL) {
      // debounce is in progress
      return EVENT_NONE;
    } else {
      // debounce is over
      button_debounce_time = 0;
    }
  }
  if (!(BUTTON_PORT & BUTTON_MASK)) {
    // Button is down
    if (ignoring_button) return EVENT_NONE; // We're ignoring this press.
    if (button_press_time == 0) { // this is the start of a press.
      button_debounce_time = button_press_time = now;
    }
    return EVENT_NONE; // We don't know what this button-push is going to be yet
  } else {
    // Button released
    ignoring_button = 0;
    if (button_press_time == 0) return EVENT_NONE; // It wasn't down anyway.
    // We are now ending a button-push. First, start debuncing.
    button_debounce_time = now;
    unsigned long push_duration = now - button_press_time;
    button_press_time = 0;
    if (push_duration > BUTTON_LONG_START) {
      return EVENT_LONG_PUSH;
    } else {
      return EVENT_SHORT_PUSH;
    }
  }
}

void sleepNow() {
  // All LEDs off
  for(int i = 0; i < 8; i++) {
    *((unsigned char *)pgm_read_ptr(&(LED_PORT[i]))) &= ~_BV(pgm_read_byte(&(LED_PIN[i])));
  }
  cli();
  power_timer0_disable();
  PCMSK0 |= _BV(PCINT5);
  GIMSK |= _BV(PCIE0);
  sleep_enable();
  sei();
  sleep_cpu();

  // And now we wake up

  sleep_disable();
  PCMSK0 &= ~_BV(PCINT5);
  GIMSK &= ~_BV(PCIE0);
  power_timer0_enable();
  button_debounce_time = millis();
  ignoring_button = 1;
  place_in_pattern = -1;
  milli_of_next_act = 0;
}

void main() {
  // power management setup
  ADCSRA = 0; // DIE, ADC! DIE!!!
  power_adc_disable();
  power_usi_disable();
  power_timer1_disable();
  ACSR = _BV(ACD);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // millisecond timer setup
  TCCR0A = _BV(WGM01); // mode 2 - CTC
  TCCR0B = _BV(CS01) | _BV(CS00); // prescale = 64
  TIMSK0 = _BV(OCIE0A); // OCR0A interrupt only.
  OCR0A = IRQ_CYCLE_LENGTH + 1;
  millis_counter = 0;
  irq_cycle_pos = 0;
  sei();
  
  // I/O port setup 
  DDRA = _BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3) | _BV(PA7);
  DDRB = _BV(PB0) | _BV(PB1) | _BV(PB2);
  PORTA = _BV(PA5); // enable the pull-up on the button pin
  PORTB = 0;
  
  pattern = eeprom_read_byte(EE_PATTERN_NUM);
  if (pattern >= PATTERN_COUNT) pattern = 0;
  place_in_pattern = -1;
  milli_of_next_act = 0;
  button_debounce_time = 0;
  button_press_time = 0;
  ignoring_button = 0;

  while(1) {
    unsigned long now = millis();
    unsigned int event = checkEvent();
    switch(event) {
      case EVENT_LONG_PUSH:
        sleepNow();
        continue;
      case EVENT_SHORT_PUSH:
        if (++pattern >= PATTERN_COUNT) pattern = 0;
        place_in_pattern = -1;
        milli_of_next_act = 0;
        eeprom_write_byte(EE_PATTERN_NUM, pattern);
        continue;
    }
    if (milli_of_next_act == 0 || now > milli_of_next_act) {
      // It's time to go to the next step in the pattern
      place_in_pattern++;
      const struct pattern_entry *our_pattern = (struct pattern_entry *)pgm_read_ptr(&(patterns[pattern]));
      struct pattern_entry entry;
      memcpy_P(&entry, (void*)(&(our_pattern[place_in_pattern])), sizeof(struct pattern_entry));
      if (entry.duration == 0) {
        // A duration of 0 is the end of the pattern.
        place_in_pattern = 0;   
        memcpy_P(&entry, (void*)(&(our_pattern[place_in_pattern])), sizeof(struct pattern_entry));
      }
      // Set the LEDs as per the current pattern entry
      for(int i = 0; i < 8; i++) {
        if (entry.mask & (1 << i))
          *((unsigned char *)pgm_read_ptr(&(LED_PORT[i]))) |= _BV(pgm_read_byte(&(LED_PIN[i])));
        else
          *((unsigned char *)pgm_read_ptr(&(LED_PORT[i]))) &= ~_BV(pgm_read_byte(&(LED_PIN[i])));
      }
      milli_of_next_act = now + entry.duration;
    }
  }
}
