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

#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>


// Where in the EEPROM do we store the pattern number?
#define EE_PATTERN_NUM 0

#define LED_0 0
#define LED_1 1
#define LED_2 2
#define LED_3 3
#define LED_4 7
#define LED_5 10
#define LED_6 9
#define LED_7 8
byte LED_PINS[] = { LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7 };

#define BUTTON_PIN 5

#define UNUSED_1 4
#define UNUSED_2 6

// If we see any state change on the button, we ignore all changes for this long
#define BUTTON_DEBOUNCE_INTERVAL 50
// How long does the button have to stay down before we call it a LONG push?
#define BUTTON_LONG_START 250

unsigned int pattern;

struct pattern_entry {
  byte mask;
  unsigned long duration;
};

// Circle in 800 ms, sleep 1 second.
PROGMEM const struct pattern_entry circle_pattern[] = {
  { B00000001, 50 },
  { 0, 50 },
  { B00000010, 50 },
  { 0, 50 },
  { B00000100, 50 },
  { 0, 50 },
  { B00001000, 50 },
  { 0, 50 },
  { B00010000, 50 },
  { 0, 50 },
  { B00100000, 50 },
  { 0, 50 },
  { B01000000, 50 },
  { 0, 50 },
  { B10000000, 50 },
  { 0, 1000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry back_circle_pattern[] = {
  { B10000000, 50 },
  { 0, 50 },
  { B01000000, 50 },
  { 0, 50 },
  { B00100000, 50 },
  { 0, 50 },
  { B00010000, 50 },
  { 0, 50 },
  { B00001000, 50 },
  { 0, 50 },
  { B00000100, 50 },
  { 0, 50 },
  { B00000010, 50 },
  { 0, 50 },
  { B00000001, 50 },
  { 0, 1000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry skip_around[] = {
  { B10000000, 50 },
  { 0, 50 },
  { B00010000, 50 },
  { 0, 50 },
  { B00000010, 50 },
  { 0, 50 },
  { B01000000, 50 },
  { 0, 50 },
  { B00001000, 50 },
  { 0, 50 },
  { B00000001, 50 },
  { 0, 50 },
  { B00100000, 50 },
  { 0, 50 },
  { B00000100, 50 },
  { 0, 5000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry blink_all[] = {
  { B10000000, 1 },
  { B00000010, 1 },
  { B00010000, 1 },
  { B00001000, 1 },
  { B00100000, 1 },
  { B00000100, 1 },
  { B01000000, 1 },
  { B00000001, 1 },
  { 0, 2000 },
  { 0, 0 }
};

PROGMEM const struct pattern_entry *patterns[] = {
  circle_pattern,
  back_circle_pattern,
  skip_around,
  blink_all
};
// How many patterns are there?
#define PATTERN_COUNT 4
//#define PATTERN_COUNT (sizeof(patterns) / sizeof(struct pattern_entry *))

#define EVENT_NONE 0
#define EVENT_SHORT_PUSH 1
#define EVENT_LONG_PUSH 2

int place_in_pattern;
unsigned long milli_of_next_act, button_debounce_time, button_press_time;
byte ignoring_button;

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
  if (digitalRead(BUTTON_PIN) == LOW) {
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

ISR(PCINT0_vect) {
  // no-op
}

void sleepNow() {
  // All LEDs off
  for(int i = 0; i < 8; i++) {
    digitalWrite(LED_PINS[i], LOW);
  }
  cli();
  power_timer0_disable();
  PCMSK0 |= _BV(PCINT5);
  GIMSK |= _BV(PCIE0);
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
  PCMSK0 &= 0xff ^ _BV(PCINT5);
  GIMSK &= 0xff ^ _BV(PCIE0);
  power_timer0_enable();
  button_debounce_time = millis();
  ignoring_button = 1;
  place_in_pattern = -1;
  milli_of_next_act = 0;
}

void setup() {
  ADCSRA = 0; // DIE, ADC! DIE!!!
  power_adc_disable();
  power_usi_disable();
  power_timer1_disable();
  ACSR = _BV(ACD);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(UNUSED_1, OUTPUT);
  digitalWrite(UNUSED_1, LOW);
  pinMode(UNUSED_2, OUTPUT);
  digitalWrite(UNUSED_2, LOW);
  for(int i = 0; i < 8; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  
  pattern = EEPROM.read(EE_PATTERN_NUM);
  if (pattern >= PATTERN_COUNT) pattern = 0;
  place_in_pattern = -1;
  milli_of_next_act = 0;
  button_debounce_time = 0;
  button_press_time = 0;
  ignoring_button = 0;
}

void loop() {
  unsigned long now = millis();
  unsigned int event = checkEvent();
  switch(event) {
    case EVENT_LONG_PUSH:
      sleepNow();
      return;
    case EVENT_SHORT_PUSH:
      if (++pattern >= PATTERN_COUNT) pattern = 0;
      place_in_pattern = -1;
      milli_of_next_act = 0;
      EEPROM.write(EE_PATTERN_NUM, pattern);
      return;
  }
  if (milli_of_next_act == 0 || now > milli_of_next_act) {
    // It's time to go to the next step in the pattern
    place_in_pattern++;
    const struct pattern_entry *our_pattern = (struct pattern_entry *)pgm_read_word(&(patterns[pattern]));
    struct pattern_entry entry;
    memcpy_P(&entry, (void*)(&(our_pattern[place_in_pattern])), sizeof(struct pattern_entry));
    if (entry.duration == 0) {
      // A duration of 0 is the end of the pattern.
      place_in_pattern = 0;   
      memcpy_P(&entry, (void*)(&(our_pattern[place_in_pattern])), sizeof(struct pattern_entry));
    }
    // Set the LEDs as per the current pattern entry
    for(int i = 0; i < 8; i++)
      digitalWrite(LED_PINS[i], (entry.mask & (1 << i))?HIGH:LOW);
    milli_of_next_act = now + entry.duration;
  }
}
