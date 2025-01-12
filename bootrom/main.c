typedef unsigned int uint32_t;

static volatile uint32_t *uart = (void *) 0x01000004;
static volatile uint32_t *mcu_status = (void *) 0x01000000;

void recovery(void) {
    
}

void main(void) {
    // recovery boot path
    if ((*mcu_status) & 0x01) {
        recovery();
    }

    while (1) {
        while ((*mcu_status) & 4);
        *uart = 0x80;
    }
}
