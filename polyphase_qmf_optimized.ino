
#define N 4
const int16_t h0 = 15826, h1 = 27411, h2 = 7345, h3 = -4240;
const int16_t g0 = -4240, g1 = -7345, g2 = 27411, g3 = -15826;
const uint8_t LP_PIN = 9, HP_PIN = 10;
#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  static uint32_t last = 0;
  if (micros() - last < 500) return;
  last = micros();

  x[i] = analogRead(A1);
  int32_t in0 = (int32_t)x[i] - 128;
  int32_t in1 = (int32_t)x[(i - 1) & MASK] - 128;
  int32_t in2 = (int32_t)x[(i - 2) & MASK] - 128;
  int32_t in3 = (int32_t)x[(i - 3) & MASK] - 128;

  int32_t y1 = h0 * in0 + h1 * in1 + h2 * in2 + h3 * in3;
  int32_t y2 = g0 * in0 + g1 * in1 + g2 * in2 + g3 * in3;

  analogWrite(LP_PIN, constrain(((y1 + 16384L) >> 15) + 128, 0, 255));
  analogWrite(HP_PIN, constrain(((y2 + 16384L) >> 15) + 128, 0, 255));
  i = (i + 1) & MASK;
}
