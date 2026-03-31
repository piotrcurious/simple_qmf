#ifndef DB8_COEFFS_H
#define DB8_COEFFS_H

#define DB8_N 8

// DB8 Coefficients
const float h8_float[DB8_N] = { 0.23037781f, 0.71484657f, 0.63088076f, -0.02798376f, -0.18703481f, 0.03084138f, 0.03288301f, -0.01059740f };
const float g8_float[DB8_N] = { -0.01059740f, -0.03288301f, 0.03084138f, 0.18703481f, -0.02798376f, -0.63088076f, 0.71484657f, -0.23037781f };

const int16_t h8_fixed[DB8_N] PROGMEM = { 7549, 23424, 20673, -917, -6129, 1011, 1078, -347 };
const int16_t g8_fixed[DB8_N] PROGMEM = { -347, -1078, 1011, 6129, -917, -20673, 23424, -7549 };

#endif
