import sys
import numpy as np

def analyze_output(filename, input_filename, fs=4000):
    lp_data = []
    hp_data = []

    with open(filename, 'r') as f:
        for line in f:
            try:
                parts = line.split()
                if len(parts) == 2:
                    pin = int(parts[0])
                    val = int(parts[1])
                    if pin == 9: lp_data.append(val)
                    elif pin == 10: hp_data.append(val)
            except ValueError:
                continue

    input_data = []
    with open(input_filename, 'r') as f:
        for line in f:
            try:
                input_data.append(int(line.strip()))
            except ValueError:
                continue

    lp = np.array(lp_data)
    hp = np.array(hp_data)
    orig = np.array(input_data)

    min_len = min(len(lp), len(hp), len(orig))
    if min_len < 20:
        print(f"Insufficient data: {min_len} samples")
        return

    lp = lp[:min_len]
    hp = hp[:min_len]
    orig = orig[:min_len]

    start_idx = min_len // 10
    lp = lp[start_idx:]
    hp = hp[start_idx:]
    orig = orig[start_idx:]

    lp_c = lp - 128
    hp_c = hp - 128

    # Input is always 0..1023 in mock
    orig_c = orig - 512

    # DB4 Synthesis Filters
    h = np.array([0.482962913145, 0.836516303738, 0.224143868042, -0.129409522551])
    g = np.array([-0.129409522551, -0.224143868042, 0.836516303738, -0.482962913145])

    recon_lp = np.convolve(lp_c, h[::-1], mode='same')
    recon_hp = np.convolve(hp_c, g[::-1], mode='same')
    recon_qmf = (recon_lp + recon_hp)
    recon_sum = (lp_c + hp_c)

    best_snr = -100
    scales = np.concatenate([np.linspace(0.1, 15.0, 200), np.linspace(-15.0, -0.1, 200)])

    # Search including direct bands (in case one is inverted or dominant)
    for recon in [recon_sum, recon_qmf, lp_c, hp_c]:
        for scale in scales:
            for delay in range(-20, 21):
                if delay == 0:
                    target = orig_c
                    test_recon = recon * scale
                elif delay > 0:
                    target = orig_c[:-delay]
                    test_recon = recon[delay:] * scale
                else: # delay < 0
                    target = orig_c[-delay:]
                    test_recon = recon[:delay] * scale

                mse = np.mean((target - test_recon)**2)
                if mse > 1e-9:
                    snr = 10 * np.log10(np.mean(target**2) / mse)
                else:
                    snr = 100

                if snr > best_snr:
                    best_snr = snr

    print(f"Best Reconstruction SNR: {best_snr:.2f} dB")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("input")
    parser.add_argument("--plot", required=False)
    parser.add_argument("--fs", type=float, default=4000)
    args = parser.parse_args()
    analyze_output(args.output, args.input, args.fs)
