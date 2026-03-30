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

    while (std::cin.peek() != EOF && total_iterations < 2000000) {
        add_cycles(10);

        if (vector_0 && (get_total_cycles() % 256 < 10)) {
             vector_0();
        }

        loop();
        total_iterations++;
    }

    uint64_t total = get_cycle_count();
    if (total_iterations > 0) {
        std::cerr << "AVG_CYCLES_PER_LOOP: " << (total / total_iterations) << std::endl;
    }
    return 0;
}
