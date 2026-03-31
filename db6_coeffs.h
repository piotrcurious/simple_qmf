#ifndef DB6_COEFFS_H
#define DB6_COEFFS_H

#define DB6_N 6

// DB6 Coefficients
const float h6_float[DB6_N] = { 0.33267055f, 0.80689151f, 0.45987750f, -0.13501102f, -0.08544127f, 0.03522629f };
const float g6_float[DB6_N] = { 0.03522629f, 0.08544127f, -0.13501102f, -0.45987750f, 0.80689151f, -0.33267055f };

const int16_t h6_fixed[DB6_N] PROGMEM = { 10901, 26440, 15069, -4424, -2800, 1154 };
const int16_t g6_fixed[DB6_N] PROGMEM = { 1154, 2800, -4424, -15069, 26440, -10901 };

#endif
