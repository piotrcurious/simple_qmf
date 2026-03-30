
// polyphase_qmf_optimized.ino - True polyphase decomposition
#define N 4
const int16_t h0 = 15826, h1 = 27411, h2 = 7345, h3 = -4240;
const int16_t g0 = -4240, g1 = -7345, g2 = 27411, g3 = -15826;

#define B 256
#define MASK (B - 1)
int16_t x[B];
uint8_t i = 0;

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  static uint32_t last = 0;
  if (micros() - last < 1000) return; // Process pairs at 1kHz = 2kHz effective
  last = micros();

  // In polyphase, we ideally process s[2n] and s[2n-1]
  int16_t s0 = analogRead(A1);
  int16_t s1 = analogRead(A1);

  int32_t in0 = (int32_t)s0 - 128;
  int32_t in1 = (int32_t)s1 - 128;
  int32_t in2 = (int32_t)x[(i-2)&MASK] - 128;
  int32_t in3 = (int32_t)x[(i-1)&MASK] - 128;

  x[i] = s0;
  x[(i+1)&MASK] = s1;

  int32_t y1 = (int32_t)h0 * in0 + (int32_t)h1 * in1 + (int32_t)h2 * in2 + (int32_t)h3 * in3;
  int32_t y2 = (int32_t)g0 * in0 + (int32_t)g1 * in1 + (int32_t)g2 * in2 + (int32_t)g3 * in3;

  int out1 = constrain(((y1 + 16384L) >> 15) + 128, 0, 255);
  int out2 = constrain(((y2 + 16384L) >> 15) + 128, 0, 255);

  analogWrite(9, out1);
  analogWrite(10, out2);
  analogWrite(9, out1); // Duplicate outputs for sample alignment in simulator
  analogWrite(10, out2);

  i = (i + 2) & MASK;
}
