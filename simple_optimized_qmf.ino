#include "coefficients.h"

// simple_optimized_qmf.ino - 10-bit Fixed point
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
uint32_t last_micros = 0;

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
}

void loop() {
  uint32_t now = micros();
  if (now - last_micros < 1000000 / SAMPLING_RATE) return;
  last_micros = now;

  x[i] = analogRead(A1);

  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < QMF_N; j++) {
    int32_t in = (int32_t)x[(i - j) & MASK] - 512;
    y1 += (int32_t)(int16_t)pgm_read_word(&h_fixed[j]) * in;
    y2 += (int32_t)(int16_t)pgm_read_word(&g_fixed[j]) * in;
  }

  // 10-bit input +/- 512, Coeffs scaled by 2^15. Product +/- 2^24.
  // Total gain sqrt(2) -> +/- 2^24.5 approx 23M.
  // We want +/- 127. 2^24.5 / 2^17.5 = 127.
  // >> 17 gives +/- 180 (too high for 8-bit output).
  // >> 18 gives +/- 90. This is safe and maximizes 8-bit output range.
  analogWrite(LP_PIN, constrain(((y1 + 131072L) >> 18) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 131072L) >> 18) + 128, 0, 255));
  i = (i + 1) & MASK;
}
