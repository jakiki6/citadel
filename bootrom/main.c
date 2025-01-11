static char *tx = (char *) 0x01000004;
static char *rx = (char *) 0x01000004;
static char *rx_ready = (char *) 0x01000008;

void main() {
    char *msg = "Hello world!\n";
    while (*msg) {
        *tx = *msg;
        msg++;
    }
}
