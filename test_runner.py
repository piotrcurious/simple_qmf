import os
import subprocess

files = [
    "simple_slow_qmf.ino",
    "simple_optimized_qmf.ino",
    "simple_8bit_qmf_optimized.ino",
    "simple_8bit_qmf_optimized2.ino",
    "exact_band_split_8bit_qmf.ino",
    "exact_band_split_interpolated.ino",
    "overoptimized_8bit_qmf.ino"
]

def test_file(ino_file):
    print(f"\n--- Testing {ino_file} ---")

    # Preprocess .ino to .cpp
    cpp_file = ino_file.replace(".ino", ".cpp")
    with open(ino_file, 'r') as f_in, open(cpp_file, 'w') as f_out:
        f_out.write('#include "test_framework/Arduino.h"\n')
        f_out.write(f_in.read())

    # Compile
    try:
        subprocess.check_call([
            "g++", "-o", "test_bin",
            cpp_file, "test_framework/Arduino.cpp", "test_framework/main.cpp",
            "-I.", "-I./test_framework", "-Wno-narrowing", "-fpermissive"
        ])
    except subprocess.CalledProcessError:
        print(f"Compilation failed for {ino_file}")
        # Clean up
        if os.path.exists(cpp_file):
            os.remove(cpp_file)
        return

    # Generate input signal
    with open("input_signal.txt", "w") as f:
        subprocess.check_call([
            "python3", "test_framework/signal_generator.py",
            "--type", "sweep", "--f_start", "10", "--f_end", "2000", "--duration", "2.0", "--fs", "4000"
        ], stdout=f)

    # Run simulation
    with open("input_signal.txt", "r") as f_in, open("output_signal.txt", "w") as f_out:
        subprocess.check_call(["./test_bin"], stdin=f_in, stdout=f_out)

    # Analyze output
    subprocess.check_call(["python3", "test_framework/signal_analyzer.py", "output_signal.txt", "input_signal.txt"])

    # Clean up
    os.remove(cpp_file)
    if os.path.exists("test_bin"):
        os.remove("test_bin")

if __name__ == "__main__":
    for f in files:
        if os.path.exists(f):
            test_file(f)
        else:
            print(f"File {f} not found.")
