
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

// Define a variable to store the desired frequency
uint16_t freq = 1000;

void setup() {
  // Set the pin modes
  pinMode(IN_PIN, INPUT);
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  
  // Set the analog input resolution to 8 bits
  analogReadResolution(8);
}

void loop() {
  // Read the knob value and map it to the desired frequency range (80-2000 Hz)
  freq = map(analogRead(IN_PIN), 0, 255, 80, 2000);

  // Read the input signal and store it in the buffer
  x[i] = (int16_t)analogRead(A1) - 128;

  // Apply the QMF filters to the input buffer and store the results in the output buffers
  y1 = 0; // Initialize lowpass output
  y2 = 0; // Initialize highpass output
  for (uint8_t j = 0; j < N; j++) {
    // Sum the products of the filter coefficients and the input buffer values
    y1 += (int32_t)(int16_t)pgm_read_word(&h[j]) * x[(i - j) & MASK]; // Lowpass filter
    y2 += (int32_t)(int16_t)pgm_read_word(&g[j]) * x[(i - j) & MASK]; // Highpass filter
  }

  // Scale down and map to 0-255.
  // Max y1 is ~ +/- 128 * 1.67 * 2^15 ~ +/- 7M.
  // 7M >> 17 is +/- 53. Fits comfortably in 0-255 centered at 128.
  analogWrite(LP_PIN, constrain((y1 >> 17) + 128, 0, 255));
  analogWrite(HP_PIN, constrain((y2 >> 17) + 128, 0, 255));

  // Increment the buffer index and wrap around if needed
  i = (i + 1) & MASK;
}
