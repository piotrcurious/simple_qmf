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
  // For +/- 512 input, output can be +/- 724.
  // We need to scale down to +/- 127 to fit 0..255.
  // 724 / 5.7 = 127. Let's use 6.0 for safety.
  analogWrite(LP_PIN, constrain((int)(y1 / 6.0 + 128.5), 0, 255));
  analogWrite(HP_PIN, constrain((int)(y2 / 6.0 + 128.5), 0, 255));
  i = (i + 1) % B;
}
