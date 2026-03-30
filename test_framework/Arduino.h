#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5

#define PROGMEM
#define pgm_read_word(address) (*(const uint16_t*)(address))
#define pgm_read_byte(address) (*(const uint8_t*)(address))

void reset_cycle_count();
uint64_t get_cycle_count();
void add_cycles(uint32_t c);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long millis();
unsigned long micros();

long map(long x, long in_min, long in_max, long out_min, long out_max);

template<class T, class L, class H>
auto constrain(T amt, L low, H high) -> T {
    return (amt < (T)low) ? (T)low : ((amt > (T)high) ? (T)high : amt);
}

using std::min;
using std::max;

// Mock ISR and AVR registers
extern uint8_t TCCR0A;
extern uint8_t TCCR0B;
extern uint8_t TIMSK0;
extern uint8_t TCNT0;
extern uint8_t TCCR1A;
extern uint8_t TCCR1B;
extern uint8_t TIMSK1;
extern uint16_t TCNT1;
extern uint16_t ICR1;
extern uint16_t OCR1A;
extern uint16_t OCR1B;

#define CS00 0
#define TOIE0 0
#define CS11 1
#define WGM11 1
#define WGM12 2
#define WGM13 3
#define COM1A1 7
#define COM1B1 5

#define F_CPU 16000000L

#define TIMER0_OVF_vect 0
#define ISR(vector) extern "C" void vector_##vector()
#define sei()

void analogReadResolution(int res);

// Avoid conflict with math.h y1
#define y1 arduino_y1
#define y2 arduino_y2

#endif
