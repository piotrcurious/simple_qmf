
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
void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
}
void loop() {
  uint16_t freq = map(analogRead(A0), 0, 255, 80, 2000);
  uint8_t r1 = (freq < 100) ? 0 : (freq > 1000) ? R - 1 : (freq - 100) / 100;
  uint8_t r2 = (r1 < R - 1) ? r1 + 1 : r1;
  float t = (freq - (100.0 + r1 * 100.0)) / 100.0;
  if (t < 0) t = 0; if (t > 1) t = 1;
  x[i] = analogRead(A1);
  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    int32_t c_h = (int32_t)((1.0 - t) * (int16_t)pgm_read_word(&h_set[r1][j]) + t * (int16_t)pgm_read_word(&h_set[r2][j]));
    int32_t c_g = (int32_t)((1.0 - t) * (int16_t)pgm_read_word(&g_set[r1][j]) + t * (int16_t)pgm_read_word(&g_set[r2][j]));
    y1 += c_h * (x[(i - j) & MASK] - 128);
    y2 += c_g * (x[(i - j) & MASK] - 128);
  }
  analogWrite(LP_PIN, ((y1 + 16384L) >> 15) + 128);
  analogWrite(HP_PIN, ((y2 + 16384L) >> 15) + 128);
  i = (i + 1) & MASK;
}
