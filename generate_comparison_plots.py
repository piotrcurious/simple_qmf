import json
import matplotlib.pyplot as plt
import numpy as np
import os

def generate_comparison_plots(results_file):
    with open(results_file, 'r') as f:
        data = json.load(f)

    if not data:
        return

    # Extract all test types
    test_types = set()
    for d in data:
        for t in d['tests']:
            test_types.add(t['type'])

    labels = [d['file'].replace('.ino', '') for d in data]
    x = np.arange(len(labels))
    width = 0.35

    for t_type in test_types:
        snr = []
        cycles = []
        for d in data:
            t = next((test for test in d['tests'] if test['type'] == t_type), None)
            if t:
                snr.append(t['snr'])
                cycles.append(t['cycles'])
            else:
                snr.append(0)
                cycles.append(0)

        fig, ax1 = plt.subplots(figsize=(14, 7))

        color = 'tab:blue'
        ax1.set_xlabel('Implementation')
        ax1.set_ylabel('SNR (dB)', color=color)
        ax1.bar(x - width/2, snr, width, label='SNR', color=color, alpha=0.7)
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.set_ylim(0, max(snr + [10]) * 1.2)

        ax2 = ax1.twinx()
        color = 'tab:red'
        ax2.set_ylabel('Avg Cycles/Sample', color=color)
        ax2.bar(x + width/2, cycles, width, label='Cycles', color=color, alpha=0.7)
        ax2.tick_params(axis='y', labelcolor=color)
        ax2.set_ylim(0, max(cycles + [100]) * 1.2)

        plt.title(f'QMF Performance vs. Accuracy: {t_type.capitalize()} Signal')
        plt.xticks(x, labels, rotation=45, ha='right')

        fig.tight_layout()
        plt.savefig(f'comparison_report_{t_type}.png')
        plt.close()

if __name__ == "__main__":
    generate_comparison_plots('test_results.json')
