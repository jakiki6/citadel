#include <fcntl.h>

#include "Vcitadel.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

int main(int argc, char **argv) {
    vluint64_t simtime = 0;

    VerilatedContext *contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);

    Vcitadel *top = new Vcitadel{contextp};

    Verilated::traceEverOn(true);
    VerilatedVcdC *m_trace = new VerilatedVcdC;
    top->trace(m_trace, 5);
    m_trace->open("waveform.vcd");

    top->r_clk = 0;
    top->rst_n = 0;
    top->rx = 1;

    for (int i = 0; i < 200; i++) {
        top->r_clk = !top->r_clk;
        top->eval();
    }

    top->rst_n = 1;

    fcntl(fileno(stdin), F_SETFL, fcntl(fileno(stdin), F_GETFL) | O_NONBLOCK);
    while (!top->r_panic) {
        top->r_clk = !top->r_clk;
        top->eval();

        m_trace->dump(simtime);
        simtime++;

        if (simtime == 10000) break;
    }

    m_trace->close();

    delete top;
    delete contextp;

    return 0;
}
