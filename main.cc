#include "Vcitadel.h"
#include "verilated.h"

int main(int argc, char **argv) {
    VerilatedContext *contextp = new VerilatedContext;

    contextp->commandArgs(argc, argv);

    Vcitadel *top = new Vcitadel{contextp};

    top->r_clk = 0;
    top->rst_n = 0;

    for (int i = 0; i < 200; i++) {
        top->r_clk = !top->r_clk;
        top->eval();
    }

    top->rst_n = 1;

    while (!top->panic) {
        top->r_clk = !top->r_clk;
        top->eval();

        if (top->tx_ready && top->r_clk == 0) {
            putc(top->tx, stdout);
            fflush(stdout);
        }
    }

    delete top;
    delete contextp;

    return 0;
}
