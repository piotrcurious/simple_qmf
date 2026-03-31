import os
import subprocess
import re
import json

files = [
    "simple_slow_qmf.ino",
    "simple_optimized_qmf.ino",
    "simple_8bit_qmf_optimized.ino",
    "simple_8bit_qmf_optimized2.ino",
    "exact_band_split_8bit_qmf.ino",
    "exact_band_split_interpolated.ino",
    "overoptimized_8bit_qmf.ino",
    "polyphase_qmf_optimized.ino"
]

def test_file(ino_file):
    print(f"\n--- Testing {ino_file} ---")
    results = {"file": ino_file, "tests": []}

    cpp_file = ino_file.replace(".ino", ".cpp")
    with open(ino_file, 'r') as f_in, open(cpp_file, 'w') as f_out:
        f_out.write('#include "test_framework/Arduino.h"\n')
        f_out.write(f_in.read())

    try:
        subprocess.check_call([
            "g++", "-o", "test_bin",
            cpp_file, "test_framework/Arduino.cpp", "test_framework/main.cpp",
            "-I.", "-I./test_framework", "-Wno-narrowing", "-fpermissive"
        ])
    except subprocess.CalledProcessError:
        print(f"Compilation failed for {ino_file}")
        if os.path.exists(cpp_file): os.remove(cpp_file)
        return None

    with open(ino_file, 'r') as f:
        content = f.read()
        res = 8 if "analogReadResolution(8)" in content else 10
        # Use high amplitude (80% of full scale) to test clipping
        amp = 400 if res == 10 else 100

    test_configs = [
        {"type": "sweep", "f_start": "10", "f_end": "1000", "fs": "2000", "duration": "2.0", "amplitude": "400"}
    ]

    for cfg in test_configs:
        print(f"  Test: {cfg['type']} ({res}-bit)")
        cmd = ["python3", "test_framework/signal_generator.py", "--type", cfg["type"]]
        for k, v in cfg.items():
            if k != "type": cmd.extend([f"--{k}", v])

        with open("input_signal.txt", "w") as f:
            subprocess.check_call(cmd, stdout=f)

        # Generator always produces 10-bit (0-1023).
        # If we want 8-bit input for Arduino, we scale down to 0-255.
        if res == 8:
             with open("input_signal.txt", "r") as fin:
                 lines = fin.readlines()
             with open("input_signal.txt", "w") as fout:
                 for l in lines:
                     val = int(float(l.strip()) / 4.0)
                     fout.write(f"{val}\n")

        with open("input_signal.txt", "r") as f_in, open("output_signal.txt", "w") as f_out:
            # We must make sure test_bin reads at the correct cadence or just reads all.
            # The current mock main.cpp reads as fast as possible but loop() has timing.
            proc = subprocess.Popen(["./test_bin"], stdin=f_in, stdout=f_out, stderr=subprocess.PIPE, text=True)
            try:
                _, stderr = proc.communicate(timeout=20)
            except subprocess.TimeoutExpired:
                proc.kill(); stderr = ""

            perf_match = re.search(r"AVG_CYCLES_PER_LOOP: (\d+)", stderr)
            avg_cycles = int(perf_match.group(1)) if perf_match else 0

        plot_name = ino_file.replace(".ino", "")
        try:
            # fs matches sampling rate for analysis
            analysis_out = subprocess.check_output([
                "python3", "test_framework/signal_analyzer.py",
                "output_signal.txt", "input_signal.txt", "--plot", plot_name, "--fs", cfg.get("fs", "2000")
            ], text=True)

            snr_match = re.search(r"Best Reconstruction SNR: ([\-\d\.]+) dB", analysis_out)
            snr = float(snr_match.group(1)) if snr_match else -100.0
        except subprocess.CalledProcessError:
            snr = -100.0

        results["tests"].append({
            "type": cfg["type"],
            "cycles": avg_cycles,
            "snr": snr
        })
        print(f"    SNR: {snr:.2f} dB, Cycles: {avg_cycles}")

    os.remove(cpp_file)
    if os.path.exists("test_bin"): os.remove("test_bin")
    return results

if __name__ == "__main__":
    summary = []
    for f in files:
        if os.path.exists(f):
            res = test_file(f)
            if res: summary.append(res)

    print("\n\n" + "="*60)
    print("PERFORMANCE VS ACCURACY REPORT")
    print("="*60)
    print(f"{'Implementation':<35} | {'Sweep SNR':<10} | {'Cycles'}")
    print("-" * 60)
    for res in summary:
        sweep = next((t for t in res["tests"] if t["type"] == "sweep"), {"snr":0})
        cycles = sweep.get("cycles", 0)
        print(f"{res['file']:<35} | {sweep['snr']:>8.1f}dB | {cycles:>6}")

    with open("test_results.json", "w") as f:
        json.dump(summary, f, indent=2)
