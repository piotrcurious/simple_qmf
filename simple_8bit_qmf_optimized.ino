
#define N 4
const int16_t h[N] PROGMEM = {15826, 27411, 7345, -4240};
const int16_t g[N] PROGMEM = {-4240, -7345, 27411, -15826};
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;
void setup() {
  pinMode(9, OUTPUT); pinMode(10, OUTPUT);
  analogReadResolution(8);
}
void loop() {
  uint32_t now = micros();
  static uint32_t last = 0;
  if (now - last < 500) return;
  last = now;
  x[i] = analogRead(A1);
  int32_t y1 = 0, y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    int32_t in = (int32_t)x[(i - j) & MASK] - 128;
    y1 += (int32_t)(int16_t)pgm_read_word(&h[j]) * in;
    y2 += (int32_t)(int16_t)pgm_read_word(&g[j]) * in;
  }
  // Shift 16 gives +/- 106. Fits in 0..255.
  analogWrite(9, constrain(((y1 + 32768L) >> 16) + 128, 0, 255));
  analogWrite(10, constrain(((y2 + 32768L) >> 16) + 128, 0, 255));
  i = (i + 1) & MASK;
}
