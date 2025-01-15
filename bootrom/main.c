#include <stdint.h>

static volatile uint32_t *mcu_status = (void *) 0x01000000;
static volatile uint32_t *uart = (void *) 0x01000004;
static volatile uint32_t *spi = (void *) 0x01000008;

void panic(void) {
    *mcu_status = 0;
    for(;;);
}

void putc(uint8_t c) {
    while ((*mcu_status) & 0x02);
    *uart = c;
}

uint8_t getc(void) {
    while (!((*mcu_status) & 0x01));
    return *uart;
}

uint8_t spix(uint8_t c) {
    while ((*mcu_status) & 0x20);
    *spi = c;
    while ((*mcu_status) & 0x20);
    return *spi;
}

void do_recovery(void) {
    putc('C');
    putc('I');
    putc('T');
    putc('A');
    putc('D');
    putc('E');
    putc('L');
    putc('0');
}

void recovery(void) {
    do_recovery();
    panic();
}

void main(void) {
    // recovery boot path
    if ((*mcu_status) & 0x04) {
        recovery();
    }
}
