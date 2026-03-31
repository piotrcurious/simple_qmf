#include "db6_coeffs.h"
#define SAMPLING_RATE 2000

// db6_qmf_fixed.ino - 6-tap Daubechies 6 filter
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
uint32_t last_micros = 0;

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  uint32_t now = micros();
  if (now - last_micros < 1000000 / SAMPLING_RATE) return;
  last_micros = now;

  x[i] = analogRead(A1);

  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < DB6_N; j++) {
    int32_t in = (int32_t)x[(i - j) & MASK] - 128;
    y1 += (int32_t)(int16_t)pgm_read_word(&h6_fixed[j]) * in;
    y2 += (int32_t)(int16_t)pgm_read_word(&g6_fixed[j]) * in;
  }

  // DB6 gain sum(abs) is ~1.85.
  // Max output +/- 128 * 1.85 * 32768 = +/- 7.7e6.
  // >> 16 maps to +/- 118.
  analogWrite(LP_PIN, constrain(((y1 + 32768L) >> 16) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 32768L) >> 16) + 128, 0, 255));

  i = (i + 1) & MASK;
}
