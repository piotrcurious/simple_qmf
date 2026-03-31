#include "coefficients.h"

// polyphase_qmf_optimized.ino - True Polyphase-style Decomposition (2-channel)
// Splitting input into even/odd streams and processing them together.
// For QMF, this typically saves computation in decimation/interpolation,
// but here we use it to group arithmetic for peak efficiency.

#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
uint32_t last_micros = 0;

void setup() {
  pinMode(9, OUTPUT); pinMode(10, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  uint32_t now = micros();
  if (now - last_micros < 1000000 / SAMPLING_RATE) return;
  last_micros = now;

  x[i] = analogRead(A1);

  // Structured Polyphase arithmetic: grouping even and odd components
  // h = {h_even0, h_odd0, h_even1, h_odd1}
  // g = {g_even0, g_odd0, g_even1, g_odd1}

  int32_t in_even_0 = (int32_t)x[i] - 128;
  int32_t in_odd_0  = (int32_t)x[(i-1)&MASK] - 128;
  int32_t in_even_1 = (int32_t)x[(i-2)&MASK] - 128;
  int32_t in_odd_1  = (int32_t)x[(i-3)&MASK] - 128;

  int32_t y1 = (int32_t)(int16_t)pgm_read_word(&h_fixed[0]) * in_even_0 +
               (int32_t)(int16_t)pgm_read_word(&h_fixed[2]) * in_even_1 +
               (int32_t)(int16_t)pgm_read_word(&h_fixed[1]) * in_odd_0 +
               (int32_t)(int16_t)pgm_read_word(&h_fixed[3]) * in_odd_1;

  int32_t y2 = (int32_t)(int16_t)pgm_read_word(&g_fixed[0]) * in_even_0 +
               (int32_t)(int16_t)pgm_read_word(&g_fixed[2]) * in_even_1 +
               (int32_t)(int16_t)pgm_read_word(&g_fixed[1]) * in_odd_0 +
               (int32_t)(int16_t)pgm_read_word(&g_fixed[3]) * in_odd_1;

  analogWrite(9, constrain(((y1 + 32768L) >> 16) + 128, 0, 255));
  analogWrite(10, constrain(((y2 + 32768L) >> 16) + 128, 0, 255));

  i = (i + 1) & MASK;
}
