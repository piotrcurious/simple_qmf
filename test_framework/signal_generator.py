import numpy as np
import sys

def generate_sine(freq, duration, fs, amplitude=511):
    t = np.arange(0, duration, 1/fs)
    signal = amplitude * np.sin(2 * np.pi * freq * t) + 512
    return signal.astype(int)

def generate_sweep(f_start, f_end, duration, fs, amplitude=511):
    t = np.arange(0, duration, 1/fs)
    signal = amplitude * np.sin(2 * np.pi * (f_start * t + (f_end - f_start) * t**2 / (2 * duration))) + 512
    return signal.astype(int)

def generate_noise(duration, fs, amplitude=511):
    samples = int(duration * fs)
    signal = amplitude * (2 * np.random.rand(samples) - 1) + 512
    return signal.astype(int)

def generate_impulse(duration, fs, amplitude=511):
    samples = int(duration * fs)
    signal = np.full(samples, 512)
    signal[samples // 4] = 512 + amplitude
    return signal.astype(int)

def generate_multi_tone(freqs, duration, fs, amplitude=511):
    t = np.arange(0, duration, 1/fs)
    signal = np.zeros_like(t)
    for f in freqs:
        signal += (amplitude / len(freqs)) * np.sin(2 * np.pi * f * t)
    signal += 512
    return signal.astype(int)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", choices=["sine", "sweep", "noise", "impulse", "multitone"], default="sweep")
    parser.add_argument("--f_start", type=float, default=10)
    parser.add_argument("--f_end", type=float, default=1000)
    parser.add_argument("--freqs", type=str, default="100,500,900")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--fs", type=float, default=2000)
    parser.add_argument("--amplitude", type=float, default=511)
    args = parser.parse_args()

    if args.type == "sine":
        signal = generate_sine(args.f_start, args.duration, args.fs, args.amplitude)
    elif args.type == "sweep":
        signal = generate_sweep(args.f_start, args.f_end, args.duration, args.fs, args.amplitude)
    elif args.type == "noise":
        signal = generate_noise(args.duration, args.fs, args.amplitude)
    elif args.type == "impulse":
        signal = generate_impulse(args.duration, args.fs, args.amplitude)
    elif args.type == "multitone":
        freqs = [float(f) for f in args.freqs.split(",")]
        signal = generate_multi_tone(freqs, args.duration, args.fs, args.amplitude)

    for val in signal:
        print(val)
