
// Define the QMF filter coefficients for different frequency ranges
// Based on https://www.mathworks.com/help/wavelet/ug/add-quadrature-mirror-and-biorthogonal-wavelet-filters.html
#define N 4 // Number of coefficients
#define R 10 // Number of frequency ranges
const int16_t h[R][N] PROGMEM = { // Lowpass filters (scaled by 2^15)
  {19712, 34176, 9152, -5280}, // 80-100 Hz
  {19456, 33920, 9408, -5056}, // 100-200 Hz
  {18944, 33408, 9984, -4480}, // 200-300 Hz
  {18176, 32640, 10752, -3712}, // 300-400 Hz
  {17152, 31616, 11648, -2816}, // 400-500 Hz
  {15872, 30336, 12672, -1792}, // 500-600 Hz
  {14336, 28800, 13824, -640}, // 600-700 Hz
  {12544, 27008, 15072, +640}, // 700-800 Hz
  {10528, 24992, 16416, +2048}, // 800-900 Hz
  {8320, 22656, 17856, +3584} // 900-1000 Hz
};
const int16_t g[R][N] PROGMEM = { // Highpass filters (scaled by 2^15)
 {-5280,-9152,+34176,-19712}, //80-100 Hz
 {-5056,-9408,+33920,-19456}, //100-200 Hz
 {-4480,-9984,+33408,-18944}, //200-300 Hz
 {-3712,-10752,+32640,-18176}, //300-400 Hz
 {-2816,-11648,+31616,-17152}, //400-500 Hz
 {-1792,-12672,+30336,-15872}, //500-600 Hz
 {-640,-13824,+28800,-14336}, //600-700 Hz
 {+640,-15072,+27008,-12544}, //700-800 Hz
 {+2048,-16416,+24992,-10528}, //800-900 Hz
 {+3584,-17856,+22656,-8320} //900-1000 Hz
};

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

// Define a variable to store the desired frequency
uint16_t freq;

// Define a constant to store the desired period in timer ticks
#define PERIOD (1000000 / freq)

// Define a variable to store the remainder of the period
uint32_t rem = PERIOD;

// Define a function to interpolate the QMF filter coefficients for the current frequency
void interpolateQMF(int16_t h_interp[N], int16_t g_interp[N], uint16_t freq) {
  
   uint8_t r1; // Index of lower frequency range 
   uint8_t r2; // Index of upper frequency range 
   uint16_t f1; // Lower frequency value

  // Define a function to interpolate the QMF filter coefficients for the current frequency
  void interpolateQMF(int16_t h_interp[N], int16_t g_interp[N], uint16_t freq) {
    
     uint8_t r1; // Index of lower frequency range 
     uint8_t r2; // Index of upper frequency range 
     uint16_t f1; // Lower frequency value
     uint16_t f2; // Upper frequency value
     float t; // Interpolation factor
     
     // Find the nearest frequency ranges in the array
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
     
     // Find the corresponding frequency values
     f1 = 80 + r1 * 100;
     f2 = 80 + r2 * 100;
     
     // Calculate the interpolation factor
     t = (float)(freq - f1) / (f2 - f1);
     
     // Interpolate the QMF filter coefficients
     for (uint8_t j = 0; j < N; j++) {
       h_interp[j] = (int16_t)((1 - t) * pgm_read_word(&h[r1][j]) + t * pgm_read_word(&h[r2][j]));
       g_interp[j] = (int16_t)((1 - t) * pgm_read_word(&g[r1][j]) + t * pgm_read_word(&g[r2][j]));
     }
  }

void setup() {
  // Set the pin modes
  pinMode(IN_PIN, INPUT);
  pinMode(LP_PIN, OUTPUT);
  pinMode(HP_PIN, OUTPUT);
  
  // Set the analog input resolution to 8 bits
  analogReadResolution(8);
  
  // Set up timer0 with no prescaler and overflow interrupt enabled
  TCCR0A = 0; // Normal mode
  TCCR0B = (1 << CS00); // No prescaler
  TIMSK0 = (1 << TOIE0); // Overflow interrupt enabled
  
  // Enable global interrupts
  sei();
}

void loop() {
  // Read the knob value and map it to the desired frequency range (80-2000 Hz)
  freq = map(analogRead(IN_PIN), 0, 255, 80, 2000);

  // Read the input signal and store it in the buffer
  x[i] = analogRead(A1);

  // Interpolate the QMF filter coefficients for the current frequency
  int16_t h_interp[N];
  int16_t g_interp[N];
  interpolateQMF(h_interp, g_interp, freq);

  // Apply the QMF filters to the input buffer and store the results in the output buffers
  y1 = 0; // Initialize lowpass output
  y2 = 0; // Initialize highpass output
  for (uint8_t j = 0; j < N; j++) {
    // Sum the products of the filter coefficients and the input buffer values
    y1 += (int32_t)h_interp[j] * x[(i - j) & MASK]; // Lowpass filter
    y2 += (int32_t)g_interp[j] * x[(i - j) & MASK]; // Highpass filter
  }
  
  // Scale down the outputs by (2^15 / 4) and limit them to -63 to +63 range
  y1 = min(max(y1 >> 17, -63), +63); 
  y2 = min(max(y2 >> 17, -63), +63);

  // Write the output buffers to the PWM pins (scaled to 0-255)
  analogWrite(LP_PIN, y1 + 127); // Lowpass band
  analogWrite(HP_PIN, y2 + 127); // Highpass band

  // Increment the buffer index and wrap around if needed
  i = (i + 1) & MASK;
}

// Timer overflow interrupt service routine
ISR(TIMER0_OVF_vect) {
  
   if(rem < rem - TCNT0) { 
    TCNT0 -= rem; 
    rem += PERIOD;
    
    // Generate event here
    
   }
   else {
     rem -= TCNT0; // Subtract the timer value from the remainder
     TCNT0 = 0; // Reset the timer value
   }
}

//Source: Conversation with Bing, 4/21/2023
//(1) Scaling and wavelet filter - MATLAB qmf - MathWorks. https://www.mathworks.com/help/wavelet/ref/qmf.html.
//(2) Quadrature Mirror Filterbanks (QMF) - Stanford University. https://ccrma.stanford.edu/~jos/JFB/Quadrature_Mirror_Filterbanks_QMF.html.
//(3) Quadrature Mirror Filters (QMF) - Stanford University. https://ccrma.stanford.edu/~jos/sasp/Quadrature_Mirror_Filters_QMF.html.
