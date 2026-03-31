#include "coefficients.h"

// overoptimized_8bit_qmf.ino - Unrolled with PROGMEM coefficients
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
uint32_t last_micros = 0;

void setup() {
  DDRB |= (1 << 1) | (1 << 2);
  analogReadResolution(8);
}

void loop() {
  uint32_t now = micros();
  if (now - last_micros < 1000000 / SAMPLING_RATE) return;
  last_micros = now;

  uint8_t idx = i;
  int32_t in0 = (int32_t)analogRead(A1) - 128;
  x[idx] = (int16_t)(in0 + 128);
  int32_t in1 = (int32_t)x[(idx - 1) & MASK] - 128;
  int32_t in2 = (int32_t)x[(idx - 2) & MASK] - 128;
  int32_t in3 = (int32_t)x[(idx - 3) & MASK] - 128;

  // Unrolled DB4 (Orthogonal) using pgm_read_word for maintainability
  // Further optimization: load coefficients into local registers once
  static int16_t rh[4], rg[4];
  static bool first = true;
  if (first) {
    for(int j=0; j<4; j++) {
      rh[j] = (int16_t)pgm_read_word(&h_fixed[j]);
      rg[j] = (int16_t)pgm_read_word(&g_fixed[j]);
    }
    first = false;
  }

  int32_t y1 = (int32_t)rh[0] * in0 + (int32_t)rh[1] * in1 + (int32_t)rh[2] * in2 + (int32_t)rh[3] * in3;
  int32_t y2 = (int32_t)rg[0] * in0 + (int32_t)rg[1] * in1 + (int32_t)rg[2] * in2 + (int32_t)rg[3] * in3;

  analogWrite(9, constrain(((y1 + 32768L) >> 16) + 128, 0, 255));
  analogWrite(10, constrain(((y2 + 32768L) >> 16) + 128, 0, 255));

  i = (idx + 1) & MASK;
}
