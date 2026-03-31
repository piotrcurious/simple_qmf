import sys
import numpy as np
import matplotlib.pyplot as plt

def analyze_output(filename, input_filename, plot_prefix=None, fs=4000):
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
            try: input_data.append(int(line.strip()))
            except ValueError: continue

    lp = np.array(lp_data); hp = np.array(hp_data); orig = np.array(input_data)
    min_len = min(len(lp), len(hp), len(orig))
    if min_len < 100: return
    lp = lp[:min_len]; hp = hp[:min_len]; orig = orig[:min_len]

    start_idx = min_len // 10
    lp_s = lp[start_idx:]; hp_s = hp[start_idx:]; orig_s = orig[start_idx:]
    lp_c = lp_s - 128; hp_c = hp_s - 128
    if np.max(orig_s) > 300: orig_c = orig_s - 512
    else: orig_c = (orig_s - 128) * 4.0

    # DB4 Synthesis
    h = np.array([0.48296291, 0.83651630, 0.22414386, -0.12940952])
    g = np.array([-0.12940952, -0.22414386, 0.83651630, -0.48296291])

    recon_lp = np.convolve(lp_c, h[::-1], mode='same')
    recon_hp = np.convolve(hp_c, g[::-1], mode='same')
    recon_qmf = (recon_lp + recon_hp)
    recon_sum = (lp_c + hp_c)

    best_snr = -100; best_scale = 1.0; best_delay = 0; best_recon = recon_sum
    scales = np.concatenate([np.linspace(0.1, 8.0, 100), np.linspace(-8.0, -0.1, 100)])

    for r in [recon_sum, recon_qmf, lp_c, hp_c]:
        for scale in scales:
            for delay in range(-45, 46): # Wider delay for polyphase
                if delay == 0: t_orig = orig_c; t_recon = r * scale
                elif delay > 0: t_orig = orig_c[:-delay]; t_recon = r[delay:] * scale
                else: t_orig = orig_c[-delay:]; t_recon = r[:delay] * scale
                mse = np.mean((t_orig - t_recon)**2)
                if mse > 1e-9: snr = 10 * np.log10(np.mean(t_orig**2) / mse)
                else: snr = 100
                if snr > best_snr:
                    best_snr = snr; best_scale = scale; best_delay = delay; best_recon = r

    print(f"Best Reconstruction SNR: {best_snr:.2f} dB")

    if plot_prefix:
        plt.figure(figsize=(16, 12))
        plt.subplot(3, 2, 1); plt.specgram(orig_c, Fs=fs, NFFT=128, noverlap=64, cmap='magma'); plt.title(f"Input Signal ({plot_prefix})")
        plt.subplot(3, 2, 2)
        n_fft = 512; f_axis = np.fft.rfftfreq(n_fft, 1/fs)
        H_lp = []; H_hp = []
        for start in range(0, len(lp_c)-n_fft, n_fft//2):
            win_orig = np.abs(np.fft.rfft(orig_c[start:start+n_fft])) + 1e-6
            H_lp.append(np.abs(np.fft.rfft(lp_c[start:start+n_fft])) / win_orig)
            H_hp.append(np.abs(np.fft.rfft(hp_c[start:start+n_fft])) / win_orig)
        plt.plot(f_axis, 20*np.log10(np.mean(H_lp, axis=0) + 1e-6), label='LP')
        plt.plot(f_axis, 20*np.log10(np.mean(H_hp, axis=0) + 1e-6), label='HP')
        plt.title("Magnitude Response (dB)"); plt.ylim(-60, 10); plt.grid(True); plt.legend()
        plt.subplot(3, 2, 3); plt.specgram(lp_c, Fs=fs, NFFT=128, noverlap=64, cmap='viridis'); plt.title("Lowpass Band")
        plt.subplot(3, 2, 5); plt.specgram(hp_c, Fs=fs, NFFT=128, noverlap=64, cmap='viridis'); plt.title("Highpass Band")
        plt.subplot(3, 2, 4)
        if best_delay == 0: v_orig = orig_c; v_recon = best_recon * best_scale
        elif best_delay > 0: v_orig = orig_c[:-best_delay]; v_recon = best_recon[best_delay:] * best_scale
        else: v_orig = orig_c[-best_delay:]; v_recon = best_recon[:best_delay] * best_scale
        plt.plot(v_orig[:400], label='Original', alpha=0.5); plt.plot(v_recon[:400], label='Reconstructed', alpha=0.7)
        plt.title(f"Reconstruction (SNR: {best_snr:.1f} dB)"); plt.legend()
        plt.subplot(3, 2, 6); plt.plot(v_orig[:400] - v_recon[:400], color='red'); plt.title("Residual Error (Zoom)")
        plt.tight_layout(); plt.savefig(f"{plot_prefix}_analysis.png"); plt.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("output"); parser.add_argument("input")
    parser.add_argument("--plot", required=False); parser.add_argument("--fs", type=float, default=4000)
    args = parser.parse_args()
    analyze_output(args.output, args.input, args.plot, args.fs)
