#include "Arduino.h"
#include <iostream>
#include <string>

// To be linked with the .ino file's code
void setup();
void loop();

int main() {
    setup();
    while (std::cin.peek() != EOF) {
        loop();
    }
    return 0;
}
