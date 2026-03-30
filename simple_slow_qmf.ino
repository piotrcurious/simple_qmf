
// Define the QMF filter coefficients
// Based on https://www.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html
#define N 4 // Number of coefficients
float h[N] = {0.482962913145, 0.836516303738, 0.224143868042, -0.129409522551}; // Lowpass filter
float g[N] = {-0.129409522551, -0.224143868042, 0.836516303738, -0.482962913145}; // Highpass filter

// Define the input and output pins
#define IN_PIN A0 // Analog input pin for sampling rate knob
#define LP_PIN 9 // PWM output pin for lowpass band
#define HP_PIN 10 // PWM output pin for highpass band

// Define the buffer size and variables
#define B 256 // Buffer size
int x[B]; // Input buffer
int i = 0; // Buffer index

void setup() {
  // Set the pin modes
  pinMode(IN_PIN, INPUT);
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
}

void loop() {
  // Read the sampling rate from the knob (80-2000 Hz)
  int sr = map(analogRead(IN_PIN), 0, 1023, 80, 2000);

  // Read the input signal and store it in the buffer
  x[i] = analogRead(A1) - 512; // Center signal around 0

  // Apply the QMF filters to the input buffer and store the results in the output buffers
  float y1_val = 0; // Initialize lowpass output
  float y2_val = 0; // Initialize highpass output
  for (int j = 0; j < N; j++) {
    // Sum the products of the filter coefficients and the input buffer values
    y1_val += h[j] * x[(i - j + B) % B]; // Lowpass filter
    y2_val += g[j] * x[(i - j + B) % B]; // Highpass filter
  }

  // To achieve perfect reconstruction, the filters must be power-complementary.
  // In QMF, (y1 + y2) should reconstruct the input with delay.
  // The PWM range is 0-255. Input is +/- 512. Max output is ~ +/- 855.
  // Divide by 4.0 to keep signal in range +/- 213, then center at 128.
  analogWrite(LP_PIN, (int)(y1_val / 4.0) + 128);
  analogWrite(HP_PIN, (int)(y2_val / 4.0) + 128);

  // Increment the buffer index and wrap around if needed
  i = (i + 1) % B;

  // Wait for the next sample according to the sampling rate
  delay(1000 / (sr ? sr : 1));
}
