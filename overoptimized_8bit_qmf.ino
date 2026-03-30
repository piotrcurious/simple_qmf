
// Define the QMF filter coefficients for different frequency ranges
// Based on https://www.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html
#define N 4 // Number of coefficients
#define R 10 // Number of frequency ranges
const int16_t h_coeffs[R][N] PROGMEM = { // Lowpass filters (scaled by 2^15)
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240},
  {15826, 27411, 7345, -4240}
};
const int16_t g_coeffs[R][N] PROGMEM = { // Highpass filters (scaled by 2^15)
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826},
 {-4240, -7345, 27411, -15826}
};

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

// Define a variable to store the desired frequency
uint16_t freq;

// Define a variable to store the previous knob value
uint8_t prev_knob = 255;

// Define a function to interpolate the QMF filter coefficients for the current frequency
void interpolateQMF(int16_t h_local[N], int16_t g_local[N], uint16_t freq) {
  
   uint8_t r1; // Index of lower frequency range 
   uint8_t r2; // Index of upper frequency range 
   uint16_t f1; // Lower frequency value
   uint16_t f2; // Upper frequency value
   float t; // Interpolation factor
   
   if (freq <= 100) {
     r1 = 0;
     r2 = 0;
   }
   else if (freq >= 1000) {
     r1 = R - 1;
     r2 = R - 1;
   }
   else {
     r1 = (freq - 100) / 100;
     r2 = r1 + 1;
   }
   
   if (r1 >= R) r1 = R - 1;
   if (r2 >= R) r2 = R - 1;

   f1 = 100 + r1 * 100;
   f2 = 100 + r2 * 100;
   
   t = (f2 == f1) ? 0 : (float)(freq - f1) / (f2 - f1);
   
   for (uint8_t j = 0; j < N; j++) {
     h_local[j] = (int16_t)((1.0f - t) * (int16_t)pgm_read_word(&h_coeffs[r1][j]) + t * (int16_t)pgm_read_word(&h_coeffs[r2][j]));
     g_local[j] = (int16_t)((1.0f - t) * (int16_t)pgm_read_word(&g_coeffs[r1][j]) + t * (int16_t)pgm_read_word(&g_coeffs[r2][j]));
   }
}

void setup() {
  pinMode(IN_PIN, INPUT);
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  analogReadResolution(8);
}

void loop() {
  uint8_t knob = analogRead(IN_PIN);
  static int16_t h_curr[N];
  static int16_t g_curr[N];
  static bool initialized = false;

  if (knob != prev_knob || !initialized) {
    prev_knob = knob;
    initialized = true;
    freq = map(knob, 0, 255, 80, 2000);
    interpolateQMF(h_curr, g_curr, freq);
  }
  
  x[i] = (int16_t)analogRead(A1) - 128;

  y1 = 0;
  y2 = 0;
  for (uint8_t j = 0; j < N; j++) {
    y1 += (int32_t)h_curr[j] * x[(i - j) & MASK];
    y2 += (int32_t)g_curr[j] * x[(i - j) & MASK];
  }
  
  analogWrite(LP_PIN, constrain((y1 >> 15) + 128, 0, 255));
  analogWrite(HP_PIN, constrain((y2 >> 15) + 128, 0, 255));

  i = (i + 1) & MASK;
}
