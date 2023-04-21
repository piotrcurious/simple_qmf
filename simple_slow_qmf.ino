
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
int y1[B]; // Lowpass output buffer
int y2[B]; // Highpass output buffer
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
  x[i] = analogRead(A1);

  // Apply the QMF filters to the input buffer and store the results in the output buffers
  y1[i] = 0; // Initialize lowpass output
  y2[i] = 0; // Initialize highpass output
  for (int j = 0; j < N; j++) {
    // Sum the products of the filter coefficients and the input buffer values
    y1[i] += h[j] * x[(i - j + B) % B]; // Lowpass filter
    y2[i] += g[j] * x[(i - j + B) % B]; // Highpass filter
  }

  // Write the output buffers to the PWM pins (scaled to 0-255)
  analogWrite(LP_PIN, map(y1[i], -1023, 1023, 0, 255)); // Lowpass band
  analogWrite(HP_PIN, map(y2[i], -1023, 1023, 0, 255)); // Highpass band

  // Increment the buffer index and wrap around if needed
  i = (i + 1) % B;

  // Wait for the next sample according to the sampling rate
  delay(1000 / sr);
}


//Source: Conversation with Bing, 4/21/2023
//(1) Add Quadrature Mirror and Biorthogonal Wavelet Filters. https://www.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html.
//(2) Quadrature mirror filter - Wikipedia. https://en.wikipedia.org/wiki/Quadrature_mirror_filter.
//(3) Add Quadrature Mirror and Biorthogonal Wavelet Filters. https://la.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html.
//(4) GitHub - JonHub/Filters: A realtime digital signal processing (DSP .... https://github.com/JonHub/Filters.
//(5) How to Clean Up Noisy Sensor Data With a Moving Average Filter - Maker Pro. https://maker.pro/arduino/tutorial/how-to-clean-up-noisy-sensor-data-with-a-moving-average-filter.
//(6) Stabilize Sensor Readings With Kalman Filter - Instructables. https://www.instructables.com/Stabilize-Sensor-Readings-With-Kalman-Filter/.
