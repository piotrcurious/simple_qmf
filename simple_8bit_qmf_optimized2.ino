
// Define the QMF filter coefficients
// Based on https://www.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html
#define N 4 // Number of coefficients
const int16_t h[N] PROGMEM = {19712, 34176, 9152, -5280}; // Lowpass filter (scaled by 2^15)
const int16_t g[N] PROGMEM = {-5280, -9152, 34176, -19712}; // Highpass filter (scaled by 2^15)

// Define the input and output pins
const uint8_t IN_PIN = A0; // Analog input pin for sampling rate knob
const uint8_t LP_PIN = 9; // PWM output pin for lowpass band
const uint8_t HP_PIN = 10; // PWM output pin for highpass band

// Define the buffer size and variables
#define B 256 // Buffer size (must be a power of 2)
#define MASK (B - 1) // Bitmask for wrapping around the buffer index
int16_t x[B]; // Input buffer (circular)
int16_t y1; // Lowpass output
int16_t y2; // Highpass output
uint8_t i = 0; // Buffer index

void setup() {
  // Set the pin modes
  pinMode(IN_PIN, INPUT);
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  
  // Set the analog input resolution to 8 bits
  analogReadResolution(8);
}

void loop() {
  // Read the sampling rate from the knob (80-2000 Hz)
  uint16_t sr = map(analogRead(IN_PIN), 0, 255, 80, 2000);

  // Read the input signal and store it in the buffer
  x[i] = analogRead(A1);

  // Apply the QMF filters to the input buffer and store the results in the output buffers
  y1 = 0; // Initialize lowpass output
  y2 = 0; // Initialize highpass output
  for (uint8_t j = 0; j < N; j++) {
    // Sum the products of the filter coefficients and the input buffer values
    y1 += (int32_t)pgm_read_word(&h[j]) * x[(i - j) & MASK]; // Lowpass filter
    y2 += (int32_t)pgm_read_word(&g[j]) * x[(i - j) & MASK]; // Highpass filter
  }
  
  // Scale down the outputs by (2^15 / 4) and limit them to -63 to +63 range
  y1 = min(max(y1 >> 17, -63), +63); 
  y2 = min(max(y2 >> 17, -63), +63);

  // Write the output buffers to the PWM pins (scaled to 0-255)
  analogWrite(LP_PIN, y1 + 127); // Lowpass band
  analogWrite(HP_PIN, y2 + 127); // Highpass band

  // Increment the buffer index and wrap around if needed
  i = (i + 1) & MASK;

  // Wait for the next sample according to the sampling rate
  delay(1000 / sr);
}