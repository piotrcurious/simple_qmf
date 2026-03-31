#include "coefficients.h"

const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
int32_t arduino_y1, arduino_y2;
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

  arduino_y1 = 0; arduino_y2 = 0;
  for (uint8_t j = 0; j < QMF_N; j++) {
    int32_t input = (int32_t)x[(i - j) & MASK] - 128;
    arduino_y1 += (int32_t)(int16_t)pgm_read_word(&h_fixed[j]) * input;
    arduino_y2 += (int32_t)(int16_t)pgm_read_word(&g_fixed[j]) * input;
  }

  analogWrite(LP_PIN, constrain(((arduino_y1 + 16384L) >> 15) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((arduino_y2 + 16384L) >> 15) + 128, 0, 255));

  i = (i + 1) & MASK;
}
