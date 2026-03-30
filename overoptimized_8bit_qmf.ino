
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
  DDRB |= (1 << 1) | (1 << 2);
  analogReadResolution(8);
}
void loop() {
  static uint32_t last = 0;
  if (micros() - last < 500) return;
  last = micros();
  uint8_t idx = i;
  int32_t in0 = (int32_t)analogRead(A1) - 128;
  x[idx] = (int16_t)(in0 + 128);
  int32_t in1 = (int32_t)x[(idx - 1) & MASK] - 128;
  int32_t in2 = (int32_t)x[(idx - 2) & MASK] - 128;
  int32_t in3 = (int32_t)x[(idx - 3) & MASK] - 128;
  int32_t y1 = 15826L * in0 + 27411L * in1 + 7345L * in2 - 4240L * in3;
  int32_t y2 = -4240L * in0 - 7345L * in1 + 27411L * in2 - 15826L * in3;
  analogWrite(9, constrain(((y1 + 16384L) >> 15) + 128, 0, 255));
  analogWrite(10, constrain(((y2 + 16384L) >> 15) + 128, 0, 255));
  i = (idx + 1) & MASK;
}
