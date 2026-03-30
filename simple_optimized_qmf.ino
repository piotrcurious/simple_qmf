
// Define the QMF filter coefficients
// Based on https://www.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html
#define N 4 // Number of coefficients
const int16_t h[N] PROGMEM = {15826, 27411, 7345, -4240}; // Lowpass filter (scaled by 2^15)
const int16_t g[N] PROGMEM = {-4240, -7345, 27411, -15826}; // Highpass filter (scaled by 2^15)

// Define the input and output pins
const uint8_t IN_PIN = A0; // Analog input pin for sampling rate knob
const uint8_t LP_PIN = 9; // PWM output pin for lowpass band
const uint8_t HP_PIN = 10; // PWM output pin for highpass band

// Define the buffer size and variables
#define B 256 // Buffer size (must be a power of 2)
#define MASK (B - 1) // Bitmask for wrapping around the buffer index
int16_t x[B]; // Input buffer (circular)
int32_t y1; // Lowpass output
int32_t y2; // Highpass output
uint8_t i = 0; // Buffer index

void setup() {
  pinMode(IN_PIN, INPUT);
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
}

void loop() {
  uint16_t sr = map(analogRead(IN_PIN), 0, 1023, 80, 2000);
  x[i] = analogRead(A1) - 512;

  y1 = 0;
  y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    y1 += (int32_t)(int16_t)pgm_read_word(&h[j]) * x[(i - j) & MASK];
    y2 += (int32_t)(int16_t)pgm_read_word(&g[j]) * x[(i - j) & MASK];
  }
  
  // Scaling: Input +/- 512, Coeffs 2^15. Accumulator +/- 28M.
  // Shift 18 to get +/- 106. Add 128 -> 22 to 234. No clipping.
  analogWrite(LP_PIN, constrain((y1 >> 18) + 128, 0, 255));
  analogWrite(HP_PIN, constrain((y2 >> 18) + 128, 0, 255));

  i = (i + 1) & MASK;
  delay(1000 / (sr ? sr : 1));
}
