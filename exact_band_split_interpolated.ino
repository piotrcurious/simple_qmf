#include "coefficients.h"

// exact_band_split_interpolated.ino - Multi-profile scaling with fixed interpolation
#define N 4
#define R 5
const int16_t h_set[R][N] PROGMEM = {
  {15826, 27411, 7345, -4240},
  {14826, 26411, 8345, -3240},
  {13826, 25411, 9345, -2240},
  {12826, 24411, 10345, -1240},
  {11826, 23411, 11345, -240}
};

const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
int16_t h_curr[N];
int16_t g_curr[N];
uint8_t prev_knob = 255;
uint32_t last_micros = 0;

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
  // Initial coefficients
  for(int j=0; j<N; j++) h_curr[j] = 0;
}

void loop() {
  uint32_t now = micros();
  if (now - last_micros < 1000000 / SAMPLING_RATE) return;
  last_micros = now;

  static uint8_t poll_counter = 0;
  if (++poll_counter >= 100) {
    poll_counter = 0;
    uint8_t knob = analogRead(A0);
    if (knob != prev_knob) {
    prev_knob = knob;
    uint16_t sw = 256 / (R - 1);
    uint8_t r1 = knob / sw;
    if (r1 >= R - 1) r1 = R - 2;
    uint8_t r2 = r1 + 1;
    uint32_t t = (uint32_t)(knob - (r1 * sw)) * 256 / sw;

    for (uint8_t j = 0; j < N; j++) {
      h_curr[j] = (int16_t)(((256L - t) * (int16_t)pgm_read_word(&h_set[r1][j]) + t * (int16_t)pgm_read_word(&h_set[r2][j])) >> 8);
    }
      // g[n] = (-1)^n * h[N-1-n]
      g_curr[0] = h_curr[3];
      g_curr[1] = -h_curr[2];
      g_curr[2] = h_curr[1];
      g_curr[3] = -h_curr[0];
    }
  }

  x[i] = analogRead(A1);
  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    int32_t in = (int32_t)x[(i - j) & MASK] - 128;
    y1 += (int32_t)h_curr[j] * in;
    y2 += (int32_t)g_curr[j] * in;
  }

  analogWrite(LP_PIN, constrain(((y1 + 32768L) >> 16) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 32768L) >> 16) + 128, 0, 255));
  i = (i + 1) & MASK;
}
