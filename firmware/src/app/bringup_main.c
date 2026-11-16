#include <stdio.h>
#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    for (;;)
    {
        gpio_xor_mask(1u << 25);
        printf("alive\n");
        sleep_ms(500);
    }
}