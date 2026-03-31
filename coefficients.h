// Auto-generated QMF coefficients (DB4, Standard Orthogonal)
#ifndef COEFFICIENTS_H
#define COEFFICIENTS_H

#define QMF_N 4
#define SAMPLING_RATE 2000

// Floating point coefficients (Sum = sqrt(2))
const float h_float[QMF_N] = { 0.48296291f, 0.83651630f, 0.22414387f, -0.12940952f };
const float g_float[QMF_N] = { -0.12940952f, -0.22414387f, 0.83651630f, -0.48296291f };

// Fixed point coefficients (scaled by 2^15)
const int16_t h_fixed[QMF_N] PROGMEM = { 15826, 27411, 7345, -4240 };
const int16_t g_fixed[QMF_N] PROGMEM = { -4240, -7345, 27411, -15826 };

#endif
