
// polyphase_qmf_optimized.ino - Correct Polyphase Decomposition
#define N 4
const int16_t h[N] = {15826, 27411, 7345, -4240};
const int16_t g[N] = {-4240, -7345, 27411, -15826};

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
    y1 += (int32_t)h[j] * in;
    y2 += (int32_t)g[j] * in;
  }
  analogWrite(9, constrain(((y1 + 16384L) >> 15) + 128, 0, 255));
  analogWrite(10, constrain(((y2 + 16384L) >> 15) + 128, 0, 255));
  i = (i + 1) & MASK;
}
