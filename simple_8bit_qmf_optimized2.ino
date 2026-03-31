#include "coefficients.h"

// ISR-driven 8-bit QMF
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
volatile int16_t x[B];
volatile uint8_t i = 0;

extern "C" void vector_0() {
  static uint32_t last_m = 0;
  uint32_t now = micros();
  if (now - last_m < 1000000 / SAMPLING_RATE) return;
  last_m = now;

  x[i] = analogRead(A1);

  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < QMF_N; j++) {
    int32_t input = (int32_t)x[(i - j) & MASK] - 128;
    y1 += (int32_t)(int16_t)pgm_read_word(&h_fixed[j]) * input;
    y2 += (int32_t)(int16_t)pgm_read_word(&g_fixed[j]) * input;
  }

  analogWrite(LP_PIN, constrain(((y1 + 16384L) >> 15) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 16384L) >> 15) + 128, 0, 255));

  i = (i + 1) & MASK;
}

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  delay(1);
}
