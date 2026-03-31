#include "Arduino.h"
#include <iostream>
#include <string>
#include <cstdio>

void setup();
void loop();

extern "C" {
    void vector_0() __attribute__((weak));
}

int main() {
    setup();
    reset_cycle_count();
    uint32_t total_iterations = 0;
    uint32_t active_loops = 0;

    uint64_t next_interrupt_cycles = 8000;
    while (std::cin.peek() != EOF && total_iterations < 2000000) {
        uint64_t before = get_cycle_count();
        loop();

        // Trigger simulated 2kHz interrupt
        if (get_total_cycles() >= next_interrupt_cycles) {
            if (vector_0) {
                vector_0();
            }
            next_interrupt_cycles += 8000;
        }

        if (get_cycle_count() > before) active_loops++;

        // Ensure time advances even if loop() is empty or polling
        if (get_total_cycles() < next_interrupt_cycles) {
            uint64_t advance = 160; // 10us
            if (get_total_cycles() + advance > next_interrupt_cycles) {
                advance = next_interrupt_cycles - get_total_cycles();
            }
            advance_time(advance);
        }

        total_iterations++;
    }

    uint64_t total = get_cycle_count();
    if (active_loops > 0) {
        std::cerr << "AVG_CYCLES_PER_SAMPLE: " << (total / active_loops) << std::endl;
    } else if (total_iterations > 0) {
        std::cerr << "AVG_CYCLES_PER_LOOP: " << (total / total_iterations) << std::endl;
    }
    return 0;
}
