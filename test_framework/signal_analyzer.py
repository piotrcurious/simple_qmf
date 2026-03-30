import sys
import numpy as np

def analyze_output(filename, input_filename):
    lp_data = []
    hp_data = []

    with open(filename, 'r') as f:
        for line in f:
            try:
                parts = line.split()
                if len(parts) == 2:
                    pin = int(parts[0])
                    val = int(parts[1])
                    if pin == 9:
                        lp_data.append(val)
                    elif pin == 10:
                        hp_data.append(val)
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
    lp = lp[:min_len]
    hp = hp[:min_len]
    orig = orig[:min_len]

    if min_len == 0:
        print("No data collected")
        return

    # Skip first 10% to let filters stabilize
    start = min_len // 10
    lp = lp[start:]
    hp = hp[start:]
    orig = orig[start:]
    half = len(lp) // 2

    # Center PWM signals around 128
    lp_c = lp - 128
    hp_c = hp - 128

    # Normalize input
    if np.max(orig) > 300:
        orig_c = orig - 512
        # Mock environment ensures A1 is 0..1023
    else:
        orig_c = (orig - 128) * 4.0

    print(f"LP Mean Abs: {np.mean(np.abs(lp_c)):.2f}, HP Mean Abs: {np.mean(np.abs(hp_c)):.2f}")

    # QMF Trend
    lp_fh = np.mean(np.abs(lp_c[:half]))
    lp_sh = np.mean(np.abs(lp_c[half:]))
    hp_fh = np.mean(np.abs(hp_c[:half]))
    hp_sh = np.mean(np.abs(hp_c[half:]))

    # Debug info
    print(f"LP FH: {lp_fh:.2f}, LP SH: {lp_sh:.2f}")
    print(f"HP FH: {hp_fh:.2f}, HP SH: {hp_sh:.2f}")

    if lp_fh > lp_sh * 1.05 and hp_sh > hp_fh * 1.05:
        print("QMF Trend Verified: LP stronger in first half, HP stronger in second half.")
    else:
        print("QMF Trend NOT Verified.")

    # Reconstruction
    h = np.array([0.482962913145, 0.836516303738, 0.224143868042, -0.129409522551])
    g = np.array([-0.129409522551, -0.224143868042, 0.836516303738, -0.482962913145])

    recon_lp = np.convolve(lp_c, h[::-1], mode='same')
    recon_hp = np.convolve(hp_c, g[::-1], mode='same')
    recon = recon_lp + recon_hp

    best_snr = -100
    best_scale = 1.0
    best_delay = 0

    for scale in np.linspace(0.1, 10.0, 100):
        for delay in range(-5, 6):
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
            signal_power = np.mean(target**2)
            if mse > 0:
                snr = 10 * np.log10(signal_power / mse)
            else:
                snr = 100

            if snr > best_snr:
                best_snr = snr
                best_scale = scale
                best_delay = delay

    print(f"Best Reconstruction SNR: {best_snr:.2f} dB (Scale: {best_scale:.2f}, Delay: {best_delay})")
    if best_snr > 15:
        print("Reconstruction: GOOD")
    else:
        print("Reconstruction: POOR")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python signal_analyzer.py <output_file> <input_file>")
    else:
        analyze_output(sys.argv[1], sys.argv[2])
