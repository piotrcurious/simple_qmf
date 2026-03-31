import numpy as np
from math import sqrt

def generate():
    # Daubechies 4 coefficients (orthogonal, sum=sqrt(2))
    h = [
        (1 + sqrt(3)) / (4 * sqrt(2)),
        (3 + sqrt(3)) / (4 * sqrt(2)),
        (3 - sqrt(3)) / (4 * sqrt(2)),
        (1 - sqrt(3)) / (4 * sqrt(2))
    ]

    # Correct QMF Highpass g[n] = (-1)^n * h[N-1-n]
    # h = [h[0], h[1], h[2], h[3]]
    # g = [h[3], -h[2], h[1], -h[0]] (or similar depending on phase)
    # Let's use g[n] = (-1)^n * h[3-n]
    # g[0] = h[3]
    # g[1] = -h[2]
    # g[2] = h[1]
    # g[3] = -h[0]
    g = [h[3], -h[2], h[1], -h[0]]

    h_fixed = [int(round(x * 32768)) for x in h]
    g_fixed = [int(round(x * 32768)) for x in g]

    with open("coefficients.h", "w") as f:
        f.write('// Auto-generated QMF coefficients (DB4, Standard Orthogonal)\n')
        f.write('#ifndef COEFFICIENTS_H\n')
        f.write('#define COEFFICIENTS_H\n\n')
        f.write('#define QMF_N 4\n')
        f.write('#define SAMPLING_RATE 2000\n\n')

        f.write('// Floating point coefficients (Sum = sqrt(2))\n')
        f.write(f'const float h_float[QMF_N] = {{ {h[0]:.8f}f, {h[1]:.8f}f, {h[2]:.8f}f, {h[3]:.8f}f }};\n')
        f.write(f'const float g_float[QMF_N] = {{ {g[0]:.8f}f, {g[1]:.8f}f, {g[2]:.8f}f, {g[3]:.8f}f }};\n\n')

        f.write('// Fixed point coefficients (scaled by 2^15)\n')
        f.write(f'const int16_t h_fixed[QMF_N] PROGMEM = {{ {h_fixed[0]}, {h_fixed[1]}, {h_fixed[2]}, {h_fixed[3]} }};\n')
        f.write(f'const int16_t g_fixed[QMF_N] PROGMEM = {{ {g_fixed[0]}, {g_fixed[1]}, {g_fixed[2]}, {g_fixed[3]} }};\n\n')

        f.write('#endif\n')

if __name__ == "__main__":
    generate()
