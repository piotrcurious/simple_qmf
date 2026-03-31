import sys
import numpy as np
import matplotlib.pyplot as plt

def analyze_output(filename, input_filename, plot_prefix=None, fs=2000):
    lp_data = []; hp_data = []
    with open(filename, 'r') as f:
        for line in f:
            try:
                parts = line.split()
                if len(parts) == 2:
                    pin = int(parts[0]); val = int(parts[1])
                    if pin == 9: lp_data.append(val)
                    elif pin == 10: hp_data.append(val)
            except ValueError: continue

    input_data = []
    with open(input_filename, 'r') as f:
        for line in f:
            try: input_data.append(float(line.strip()))
            except ValueError: continue

    lp = np.array(lp_data); hp = np.array(hp_data); orig = np.array(input_data)
    min_len = min(len(lp), len(hp), len(orig))
    if min_len < 100: return
    lp = lp[:min_len]; hp = hp[:min_len]; orig = orig[:min_len]

    start_idx = min_len // 10
    lp_s = lp[start_idx:]; hp_s = hp[start_idx:]; orig_s = orig[start_idx:]
    lp_c = lp_s - 128; hp_c = hp_s - 128

    # Check if orig is 10-bit (0-1023) or 8-bit (0-255)
    if np.max(orig_s) > 300:
        orig_c = orig_s - 512
        # Standardize for comparison (map 10-bit +/- 512 to +/- 128)
        orig_c = orig_c / 4.0
    else:
        orig_c = orig_s - 128

    # DB4 synthesis/analysis coefficients for theoretical comparison
    if "db6" in plot_prefix:
        h_theory = np.array([0.33267055, 0.80689151, 0.45987750, -0.13501102, -0.08544127, 0.03522629])
        g_theory = np.array([0.03522629, 0.08544127, -0.13501102, -0.45987750, 0.80689151, -0.33267055])
    elif "db8" in plot_prefix:
        h_theory = np.array([0.23037781, 0.71484657, 0.63088076, -0.02798376, -0.18703481, 0.03084138, 0.03288301, -0.01059740])
        g_theory = np.array([-0.01059740, -0.03288301, 0.03084138, 0.18703481, -0.02798376, -0.63088076, 0.71484657, -0.23037781])
    else:
        h_theory = np.array([0.48296291, 0.83651630, 0.22414387, -0.12940952])
        g_theory = np.array([-0.12940952, -0.22414387, 0.83651630, -0.48296291])

    # Reconstruction
    recon_lp = np.convolve(lp_c, h_theory[::-1], mode='same')
    recon_hp = np.convolve(hp_c, g_theory[::-1], mode='same')
    recon_qmf = (recon_lp + recon_hp)
    recon_sum = (lp_c + hp_c)

    best_snr = -100; best_scale = 1.0; best_delay = 0; best_recon = recon_sum

    for r in [recon_sum, recon_qmf, lp_c, hp_c]:
        for scale in [1.0, 0.5, 2.0, 4.0, 0.25]: # Quick check for common scales
            for delay in range(-45, 46):
                if delay == 0: t_orig = orig_c; t_recon = r * scale
                elif delay > 0: t_orig = orig_c[:-delay]; t_recon = r[delay:] * scale
                else: t_orig = orig_c[-delay:]; t_recon = r[:delay] * scale
                mse = np.mean((t_orig - t_recon)**2)
                p_orig = np.mean(t_orig**2)
                if p_orig < 1e-10: snr = -100
                else: snr = 10 * np.log10(p_orig / (mse + 1e-12))
                if snr > best_snr:
                    best_snr = snr; best_scale = scale; best_delay = delay; best_recon = r

    # Fine tune scale
    for scale in np.linspace(best_scale*0.8, best_scale*1.2, 50):
        if best_delay == 0: t_orig = orig_c; t_recon = best_recon * scale
        elif best_delay > 0: t_orig = orig_c[:-best_delay]; t_recon = best_recon[best_delay:] * scale
        else: t_orig = orig_c[-best_delay:]; t_recon = best_recon[:best_delay] * scale
        mse = np.mean((t_orig - t_recon)**2)
        p_orig = np.mean(t_orig**2)
        if p_orig < 1e-10: snr = -100
        else: snr = 10 * np.log10(p_orig / (mse + 1e-12))
        if snr > best_snr:
            best_snr = snr; best_scale = scale

    print(f"Best Reconstruction SNR: {best_snr:.2f} dB")

    if plot_prefix:
        plt.figure(figsize=(16, 12))
        plt.subplot(3, 2, 1); plt.specgram(orig_c, Fs=fs, NFFT=128, noverlap=64, cmap='magma'); plt.title(f"Input Signal ({plot_prefix})")

        plt.subplot(3, 2, 2)
        n_fft = 1024; f_axis = np.fft.rfftfreq(n_fft, 1/fs)
        psd_orig = np.zeros(len(f_axis)); psd_lp = np.zeros(len(f_axis)); psd_hp = np.zeros(len(f_axis))
        win = np.hanning(n_fft); count = 0
        for start in range(0, len(lp_c)-n_fft, n_fft//2):
            psd_orig += np.abs(np.fft.rfft(orig_c[start:start+n_fft] * win))**2
            psd_lp += np.abs(np.fft.rfft(lp_c[start:start+n_fft] * win))**2
            psd_hp += np.abs(np.fft.rfft(hp_c[start:start+n_fft] * win))**2
            count += 1

        # Power Transfer Function Magnitude estimation
        # Use simple ratio of PSDs. sqrt because PSD is squared magnitude.
        eps = 1e-12
        mag_lp = np.sqrt(psd_lp / (psd_orig + eps))
        mag_hp = np.sqrt(psd_hp / (psd_orig + eps))

        # Scaling adjustment: the analogWrite process often scales things by some factor.
        # We can normalize so that the LP peak matches the theoretical peak (sqrt(2) = 3dB).
        # Actually, let's normalize LP at DC to 3.01 dB.
        scale_adj = np.sqrt(2.0) / (mag_lp[0] + eps)
        ref_lp = 20*np.log10(mag_lp * scale_adj + eps)
        ref_hp = 20*np.log10(mag_hp * scale_adj + eps)

        plt.plot(f_axis, ref_lp, label='LP (Measured)', linewidth=2)
        plt.plot(f_axis, ref_hp, label='HP (Measured)', linewidth=2)

        # Theoretical DB
        z = np.exp(-2j * np.pi * f_axis / fs)
        h_z = np.zeros(len(f_axis), dtype=complex)
        g_z = np.zeros(len(f_axis), dtype=complex)
        for i in range(len(h_theory)):
            h_z += h_theory[i] * (z**i)
            g_z += g_theory[i] * (z**i)

        plt.plot(f_axis, 20*np.log10(np.abs(h_z) + eps), 'k--', label='Theoretical', alpha=0.5)
        plt.plot(f_axis, 20*np.log10(np.abs(g_z) + eps), 'k--', alpha=0.5)

        plt.title("Magnitude Response (dB)"); plt.ylim(-60, 10); plt.grid(True); plt.legend()

        plt.subplot(3, 2, 3); plt.specgram(lp_c, Fs=fs, NFFT=128, noverlap=64, cmap='viridis'); plt.title("Lowpass Band")
        plt.subplot(3, 2, 5); plt.specgram(hp_c, Fs=fs, NFFT=128, noverlap=64, cmap='viridis'); plt.title("Highpass Band")

        plt.subplot(3, 2, 4)
        if best_delay == 0: v_orig = orig_c; v_recon = best_recon * best_scale
        elif best_delay > 0: v_orig = orig_c[:-best_delay]; v_recon = best_recon[best_delay:] * best_scale
        else: v_orig = orig_c[-best_delay:]; v_recon = best_recon[:best_delay] * best_scale
        plt.plot(v_orig[:400], label='Original', alpha=0.5); plt.plot(v_recon[:400], label='Reconstructed', alpha=0.7)
        plt.title(f"Reconstruction (SNR: {best_snr:.1f} dB)"); plt.legend()

        plt.subplot(3, 2, 6); plt.plot(v_orig[:400] - v_recon[:400], color='red'); plt.title("Residual Error (Zoomed)")
        plt.tight_layout(); plt.savefig(f"{plot_prefix}_analysis.png"); plt.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("output"); parser.add_argument("input")
    parser.add_argument("--plot", required=False); parser.add_argument("--fs", type=float, default=2000)
    args = parser.parse_args()
    analyze_output(args.output, args.input, args.plot, args.fs)
