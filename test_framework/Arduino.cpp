#include "Arduino.h"
#include <chrono>
#include <thread>
#include <map>

std::map<uint8_t, uint8_t> pinModes;
std::map<uint8_t, int> pinValues;
int analogResolution = 10;
uint64_t total_cycles = 0;
uint64_t processing_cycles = 0;

uint8_t TCCR0A = 0, TCCR0B = 0, TIMSK0 = 0, TCNT0 = 0;
uint8_t TCCR1A = 0, TCCR1B = 0, TIMSK1 = 0;
uint16_t TCNT1 = 0, ICR1 = 0, OCR1A = 0, OCR1B = 0;
uint8_t DDRB = 0;

void reset_cycle_count() { total_cycles = 0; processing_cycles = 0; }
uint64_t get_cycle_count() { return processing_cycles; }
uint64_t get_total_cycles() { return total_cycles; }
void add_cycles(uint32_t c) { total_cycles += c; processing_cycles += c; }

void pinMode(uint8_t pin, uint8_t mode) { pinModes[pin] = mode; }
void digitalWrite(uint8_t pin, uint8_t val) { pinValues[pin] = val; add_cycles(2); }
int digitalRead(uint8_t pin) { add_cycles(4); return pinValues[pin]; }

int analogRead(uint8_t pin) {
    add_cycles(1600); // Realistic AVR analogRead (~100us)
    if (pin == A1) {
        int val;
        if (std::cin >> val) return val;
        return 0;
    }
    if (pin == A0) return (analogResolution == 8) ? 128 : 512;
    return 0;
}

void analogWrite(uint8_t pin, int val) {
    add_cycles(20); // More realistic
    printf("%d %d\n", (int)pin, val);
}

void delay(unsigned long ms) { total_cycles += (uint64_t)ms * 16000; }
void delayMicroseconds(unsigned int us) { total_cycles += (uint64_t)us * 16; }
unsigned long millis() { return total_cycles / 16000; }
unsigned long micros() { return total_cycles / 16; }

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  add_cycles(50);
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void analogReadResolution(int res) { analogResolution = res; }
