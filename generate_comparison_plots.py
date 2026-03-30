import json
import matplotlib.pyplot as plt
import numpy as np

def generate_comparison_plots(results_file):
    with open(results_file, 'r') as f:
        data = json.load(f)

    labels = [d['file'].replace('.ino', '') for d in data]
    snr = [next(t['snr'] for t in d['tests'] if t['type'] == 'sweep') for d in data]
    cycles = [next(t['cycles'] for t in d['tests'] if t['type'] == 'sweep') for d in data]

    x = np.arange(len(labels))
    width = 0.35

    fig, ax1 = plt.subplots(figsize=(12, 6))

    color = 'tab:blue'
    ax1.set_xlabel('Implementation')
    ax1.set_ylabel('SNR (dB)', color=color)
    ax1.bar(x - width/2, snr, width, label='SNR', color=color, alpha=0.7)
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.set_ylim(0, max(snr) * 1.2)

    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Avg Cycles/Loop', color=color)
    ax2.bar(x + width/2, cycles, width, label='Cycles', color=color, alpha=0.7)
    ax2.tick_params(axis='y', labelcolor=color)
    ax2.set_ylim(0, max(cycles) * 1.2)

    plt.title('QMF Implementations: Performance vs. Accuracy')
    plt.xticks(x, labels, rotation=45)

    fig.tight_layout()
    plt.savefig('comparison_report.png')
    plt.close()

if __name__ == "__main__":
    generate_comparison_plots('test_results.json')
