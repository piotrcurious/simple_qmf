
#define N 4
const int16_t h[N] PROGMEM = {15826, 27411, 7345, -4240};
const int16_t g[N] PROGMEM = {-4240, -7345, 27411, -15826};
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
int32_t arduino_y1, arduino_y2;
uint8_t i = 0;
void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
}
void loop() {
  x[i] = analogRead(A1);
  arduino_y1 = 0; arduino_y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    arduino_y1 += (int32_t)(int16_t)pgm_read_word(&h[j]) * (x[(i - j) & MASK] - 128);
    arduino_y2 += (int32_t)(int16_t)pgm_read_word(&g[j]) * (x[(i - j) & MASK] - 128);
  }
  analogWrite(LP_PIN, ((arduino_y1 + 16384L) >> 15) + 128);
  analogWrite(HP_PIN, ((arduino_y2 + 16384L) >> 15) + 128);
  i = (i + 1) & MASK;
}
