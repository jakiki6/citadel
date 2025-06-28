#include <stdint.h>

static volatile uint32_t *mcu_status = (void *) 0x01000000;
static volatile uint32_t *uart = (void *) 0x01000004;

// CITADEL0 in little-endian
static uint64_t boot_magic = 0x304c454441544943;

static inline __attribute__((noreturn)) void panic(void) {
    *mcu_status = 0;
}

static inline void putc(uint8_t c) {
    while ((*mcu_status) & 0x02);
    *uart = c;
    while ((*mcu_status) & 0x02);
}

static inline uint8_t getc(void) {
    while (!((*mcu_status) & 0x01));
    return *uart;
}

void putcs(void *_buf, unsigned int count) {
    uint8_t *buf = (uint8_t *) _buf;
    while (count--) {
        putc(*buf);
        buf++;
    }
}

void getcs(void *_buf, unsigned int count) {
    uint8_t *buf = (uint8_t *) _buf;
    while (count--) {
        *buf = getc();
        buf++;
    }
}

static inline uint8_t rng() {
    uint8_t r = 0;

    for (int i = 0; i < 8; i++) {
        r <<= 1;
        r |= ((*mcu_status) & 0x08) ? 1 : 0;
    }

    return r;
}

void do_recovery(void) {
    putcs(&boot_magic, 8);
}

void recovery(void) {
    do_recovery();
    panic();
}

void __attribute__((__section__(".start"))) main(void) {
    // recovery boot path
    if ((*mcu_status) & 0x04) {
        recovery();
    }

    panic();
}
