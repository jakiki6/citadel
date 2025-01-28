#define TRACE 0

#include <fcntl.h>

#include "Vcitadel.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

void spi_start() {

}

void spi_end() {

}

uint8_t spi_process(uint8_t in) {
    return 0x00;
}

int main(int argc, char **argv) {
    vluint64_t simtime = 0;
    uint8_t rx = 0;
    uint8_t tx = 0;
    int irx = 0;
    int itx = 0;
    int tx_delay = 100;
    uint8_t mosi = 0;
    uint8_t miso = 0;
    int old_cs = 1;
    int spi_ctr = 0;

    srand48(time(NULL));

    VerilatedContext *contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);

    Vcitadel *top = new Vcitadel{contextp};

#if TRACE
    Verilated::traceEverOn(true);
    VerilatedVcdC *m_trace = new VerilatedVcdC;
    top->trace(m_trace, 5);
    m_trace->open("waveform.vcd");
#endif

    top->r_clk = 0;
    top->rst_n = 0;
    top->rx = 1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--recovery") == 0) {
            top->recovery = 1;
        }
    }

    for (int i = 0; i < 200; i++) {
        top->r_clk = !top->r_clk;
        top->eval();
   }

    top->rst_n = 1;

    fcntl(fileno(stdin), F_SETFL, fcntl(fileno(stdin), F_GETFL) | O_NONBLOCK);
    while (!top->r_panic) {
        top->r_clk = !top->r_clk;

        if (top->cs != old_cs) {
            if (top->cs) {
                spi_end();
            } else {
                spi_start();
            }
        }

        old_cs = top->cs;

        if (top->r_clk) {
            top->rng = lrand48() & 1;

            if (itx) {
                itx--;
                top->rx = tx >> 7;
                tx <<= 1;

                if (itx == 0) {
                    tx_delay = 32;
                }
            } else if (tx_delay) {
                tx_delay--;
                top->rx = 1;
            } else {
                int c = getc(stdin);
                if (c >= 0) {
                    itx = 8;
                    tx = c;
                    top->rx = 0;
                }
            }
        } else {
            if (!top->cs) {
                top->miso = miso >> 7;
                miso <<= 1;
            }
        }

        top->eval();

#if TRACE
        m_trace->dump(simtime);
#endif
        simtime++;

        if (top->r_clk) {
            if (irx) {
                irx--;
                rx <<= 1;
                rx |= top->tx & 1;

                if (irx == 0) {
                    putc(rx, stdout);
                    fflush(stdout);
                }
            } else if (top->tx == 0) {
                irx = 8;
            }

            if (!top->cs) {
                mosi <<= 1;
                mosi |= top->mosi & 1;

                spi_ctr++;
                if (spi_ctr == 8) {
                    spi_ctr = 0;

                    miso = spi_process(mosi);
                }
            }
        }
    }

#if TRACE
    m_trace->close();
#endif

    delete top;
    delete contextp;

    return 0;
}
