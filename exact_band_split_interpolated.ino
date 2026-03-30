
// exact_band_split_interpolated.ino - Fixed-point interpolation
#define N 4
#define R 10
const int16_t h_set[R][N] PROGMEM = {
  {15826, 27411, 7345, -4240}, {15622, 27207, 7551, -4061}, {15212, 26795, 7964, -3701},
  {14594, 26179, 8585, -3163}, {13771, 25357, 9414, -2451}, {12745, 24332, 10444, -1571},
  {11516, 23101, 11674, -533}, {10085, 21670, 13106, 650}, {8456, 20042, 14742, 1968},
  {6651, 18176, 16641, 3450}
};
const int16_t g_set[R][N] PROGMEM = {
 {-4240, -7345, 27411, -15826}, {-4061, -7551, 27207, -15622}, {-3701, -7964, 26795, -15212},
 {-3163, -8585, 26179, -14594}, {-2451, -9414, 25357, -13771}, {-1571, -10444, 24332, -12745},
 {-533, -11674, 23101, -11516}, {650, -13106, 21670, -10085}, {1968, -14742, 20042, -8456},
 {3450, -16641, 18176, -6651}
};

const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
uint8_t prev_knob = 255;

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  uint8_t knob = analogRead(A0);
  static int16_t h_curr[N], g_curr[N];
  if (knob != prev_knob) {
    prev_knob = knob;
    uint16_t freq = map(knob, 0, 255, 80, 2000);
    uint8_t r1 = (freq < 100) ? 0 : (freq > 1000) ? R - 1 : (freq - 100) / 100;
    uint8_t r2 = (r1 < R - 1) ? r1 + 1 : r1;
    // Linear interpolation in fixed point (using 256 as factor)
    uint16_t t = (freq - (100 + r1 * 100)) * 256 / 100;
    if (t > 256) t = 256;
    for (uint8_t j = 0; j < N; j++) {
      h_curr[j] = (int16_t)(((256L - t) * (int16_t)pgm_read_word(&h_set[r1][j]) + t * (int16_t)pgm_read_word(&h_set[r2][j])) >> 8);
      g_curr[j] = (int16_t)(((256L - t) * (int16_t)pgm_read_word(&g_set[r1][j]) + t * (int16_t)pgm_read_word(&g_set[r2][j])) >> 8);
    }
  }

  x[i] = analogRead(A1);
  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    y1 += (int32_t)h_curr[j] * (x[(i - j) & MASK] - 128);
    y2 += (int32_t)g_curr[j] * (x[(i - j) & MASK] - 128);
  }
  analogWrite(LP_PIN, constrain(((y1 + 16384L) >> 15) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 16384L) >> 15) + 128, 0, 255));
  i = (i + 1) & MASK;
}
