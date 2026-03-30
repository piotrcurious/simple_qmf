
// simple_slow_qmf.ino - Floating-point logic
#define N 4
float h[N] = {0.482962913145, 0.836516303738, 0.224143868042, -0.129409522551};
float g[N] = {-0.129409522551, -0.224143868042, 0.836516303738, -0.482962913145};

#define LP_PIN 9
#define HP_PIN 10
#define B 256
int16_t x[B];
uint8_t i = 0;

void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
}

void loop() {
  uint32_t now = micros();
  static uint32_t last = 0;
  if (now - last < 500) return;
  last = now;

  x[i] = analogRead(A1);
  float y1_val = 0, y2_val = 0;
  for (int j = 0; j < N; j++) {
    float input = (float)x[(i - j + B) % B] - 512.0;
    y1_val += h[j] * input;
    y2_val += g[j] * input;
  }
  // Rounded scaling to PWM range (0..255).
  // Standard DB4 filter has gain 1. Input +/- 512 maps to +/- 512.
  // We scale to +/- 127 by dividing by 4.
  analogWrite(LP_PIN, constrain((int)(y1_val / 4.0 + 128.5), 0, 255));
  analogWrite(HP_PIN, constrain((int)(y2_val / 4.0 + 128.5), 0, 255));
  i = (i + 1) % B;
}
