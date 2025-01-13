#include <stdint.h>

static volatile uint32_t *uart = (void *) 0x01000004;
static volatile uint32_t *mcu_status = (void *) 0x01000000;

void recovery(void) {
    
}

void main(void) {
    // recovery boot path
    if ((*mcu_status) & 0x04) {
        recovery();
    }

    char *msg = "Hello world from UART!\n";

    while (*msg) {
        while ((*mcu_status) & 0x02);
        *uart = *msg;
        msg++;
    }

    while (1) {
        while (!((*mcu_status) & 0x01));

        *uart = *uart;
    }
}
