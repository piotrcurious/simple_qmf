#include "Arduino.h"
#include <chrono>
#include <thread>
#include <map>

std::map<uint8_t, uint8_t> pinModes;
std::map<uint8_t, int> pinValues;
int analogResolution = 10; // Default 10 bit

uint8_t TCCR0A = 0;
uint8_t TCCR0B = 0;
uint8_t TIMSK0 = 0;
uint8_t TCNT0 = 0;

uint8_t TCCR1A = 0;
uint8_t TCCR1B = 0;
uint8_t TIMSK1 = 0;
uint16_t TCNT1 = 0;
uint16_t ICR1 = 0;
uint16_t OCR1A = 0;
uint16_t OCR1B = 0;

void pinMode(uint8_t pin, uint8_t mode) {
    pinModes[pin] = mode;
}

void digitalWrite(uint8_t pin, uint8_t val) {
    pinValues[pin] = val;
}

int digitalRead(uint8_t pin) {
    return pinValues[pin];
}

int analogRead(uint8_t pin) {
    if (pin == A1) {
        int val;
        if (std::cin >> val) {
            // Signal generator produces 0 to 1023
            if (analogResolution == 8) return val >> 2;
            return val;
        }
        return 0;
    }
    if (pin == A0) {
        // High knob value for high sampling rate
        return (analogResolution == 8) ? 255 : 1023;
    }
    return 0;
}

void analogWrite(uint8_t pin, int val) {
    std::cout << (int)pin << " " << val << std::endl;
}

void delay(unsigned long ms) {
}

void delayMicroseconds(unsigned int us) {
}

unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

unsigned long micros() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void analogReadResolution(int res) {
    analogResolution = res;
}
