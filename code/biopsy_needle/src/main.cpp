#include <iostream>
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    sleep_ms(500);

    while(1) {
        sleep_ms(500);

        printf("Hello World!");

        // Clear terminal 
        printf("\033[1;1H\033[2J");
    }
    return 0;
}