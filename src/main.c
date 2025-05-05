// src/main.c
#include "stm32f413xx.h"
#include "core_cm4.h"

int main(void) {
    for (volatile int i = 0; i < 100000; ++i) {
        __NOP(); // Test core_cm4.h is working
    }

    while (1) {
        __WFI(); // Wait for interrupt
    }
}
