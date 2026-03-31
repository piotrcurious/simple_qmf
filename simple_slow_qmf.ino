#include "coefficients.h"

// simple_slow_qmf.ino - Floating point implementation
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
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
  float y1 = 0, y2 = 0;
  for (int j = 0; j < QMF_N; j++) {
    float in = (float)x[(i - j + B) % B] - 512.0;
    y1 += h_float[j] * in;
    y2 += g_float[j] * in;
  }
  // Standard DB4 gain is ~1.41 (sqrt(2)).
  // Sum of absolute values is ~1.67.
  // For +/- 512 input, max output is +/- 856.
  // We need to scale down to fit within 0..255 (offset 128).
  // 856 / 8.0 = 107. 128 +/- 107 is 21 to 235. Safe.
  analogWrite(LP_PIN, constrain((int)(y1 / 8.0 + 128.5), 0, 255));
  analogWrite(HP_PIN, constrain((int)(y2 / 8.0 + 128.5), 0, 255));
  i = (i + 1) % B;
}
