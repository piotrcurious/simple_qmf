
// simple_slow_qmf.ino - DB4 Floating-point logic
#define N 4
float h[N] = {0.48296291, 0.83651630, 0.22414386, -0.12940952};
float g[N] = {-0.12940952, -0.22414386, 0.83651630, -0.48296291};
#define LP_PIN 9
#define HP_PIN 10
#define B 256
int16_t x[B];
uint8_t i = 0;
void setup() {
  pinMode(9, OUTPUT); pinMode(10, OUTPUT);
}
void loop() {
  uint32_t now = micros();
  static uint32_t last = 0;
  if (now - last < 500) return;
  last = now;
  x[i] = analogRead(A1);
  float y1 = 0, y2 = 0;
  for (int j = 0; j < N; j++) {
    float input = (float)x[(i - j + B) % B] - 512.0;
    y1 += h[j] * input;
    y2 += g[j] * input;
  }
  // Safe scaling: +/- 855 -> +/- 106.
  analogWrite(9, constrain((int)(y1 / 8.0 + 128.0), 0, 255));
  analogWrite(10, constrain((int)(y2 / 8.0 + 128.0), 0, 255));
  i = (i + 1) % B;
}
