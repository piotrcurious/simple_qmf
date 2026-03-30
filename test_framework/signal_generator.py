import numpy as np
import sys

def generate_sine(freq, duration, fs, amplitude=511):
    t = np.arange(0, duration, 1/fs)
    signal = amplitude * np.sin(2 * np.pi * freq * t) + 512
    return signal.astype(int)

def generate_sweep(f_start, f_end, duration, fs, amplitude=511):
    t = np.arange(0, duration, 1/fs)
    # Correct chirp signal generation
    signal = amplitude * np.sin(2 * np.pi * (f_start * t + (f_end - f_start) * t**2 / (2 * duration))) + 512
    return signal.astype(int)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", choices=["sine", "sweep"], default="sweep")
    parser.add_argument("--f_start", type=float, default=10)
    parser.add_argument("--f_end", type=float, default=1000)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--fs", type=float, default=2000)
    args = parser.parse_args()

    if args.type == "sine":
        signal = generate_sine(args.f_start, args.duration, args.fs)
    else:
        signal = generate_sweep(args.f_start, args.f_end, args.duration, args.fs)

    for val in signal:
        print(val)
