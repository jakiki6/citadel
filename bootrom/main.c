static volatile char *tx = (char *) 0x01000004;
static volatile char *rx = (char *) 0x01000004;
static volatile char *rx_ready = (char *) 0x01000008;

void main() {
    char *msg = "Hello world!\n";
    while (*msg) {
        *tx = *msg;
        msg++;
    }

    while (!(*rx_ready));
}
