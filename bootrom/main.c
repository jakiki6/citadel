static volatile char *tx = (char *) 0x01000004;
static volatile char *rx = (char *) 0x01000004;
static volatile char *mcu_status = (char *) 0x01000008;

void recovery(void) {

}

void main(void) {
    // recovery boot path
    if ((*mcu_status) & 0x02) {
        recovery();
    }


}
