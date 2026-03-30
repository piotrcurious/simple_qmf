
#define N 4
float h[N] = {0.482962913145, 0.836516303738, 0.224143868042, -0.129409522551};
float g[N] = {-0.129409522551, -0.224143868042, 0.836516303738, -0.482962913145};
#define LP_PIN 9
#define HP_PIN 10
#define B 256
int x[B];
int i = 0;
void setup() {
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
}
void loop() {
  x[i] = analogRead(A1);
  float y1_val = 0, y2_val = 0;
  for (int j = 0; j < N; j++) {
    float input = (float)x[(i - j + B) % B] - 512.0;
    y1_val += h[j] * input;
    y2_val += g[j] * input;
  }
  analogWrite(LP_PIN, (int)(y1_val / 4.0 + 128.5));
  analogWrite(HP_PIN, (int)(y2_val / 4.0 + 128.5));
  i = (i + 1) % B;
}
