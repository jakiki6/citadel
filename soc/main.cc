#define TRACE 0

#include <fcntl.h>

#include "Vcitadel.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

int main(int argc, char **argv) {
    vluint64_t simtime = 0;
    uint8_t rx = 0;
    uint8_t tx = 0;
    int irx = 0;
    int itx = 0;
    int tx_delay = 100;
    int old_sclk = 1;

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
        }
    }

#if TRACE
    m_trace->close();
#endif

    delete top;
    delete contextp;

    return 0;
}
