#include "db8_coeffs.h"
#define SAMPLING_RATE 2000

// db8_qmf_fixed.ino - 8-tap Daubechies 8 filter
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
  for (uint8_t j = 0; j < DB8_N; j++) {
    int32_t in = (int32_t)x[(i - j) & MASK] - 128;
    y1 += (int32_t)(int16_t)pgm_read_word(&h8_fixed[j]) * in;
    y2 += (int32_t)(int16_t)pgm_read_word(&g8_fixed[j]) * in;
  }

  // DB8 gain sum(abs) is ~1.95.
  // >> 16 maps to +/- 125.
  // Using rounding (+ 32768) before shift.
  analogWrite(LP_PIN, constrain(((y1 + 32768L) >> 16) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 32768L) >> 16) + 128, 0, 255));

  i = (i + 1) & MASK;
}
