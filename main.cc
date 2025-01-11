#include "Vcitadel.h"
#include "verilated.h"

int main(int argc, char **argv) {
    VerilatedContext *contextp = new VerilatedContext;

    contextp->commandArgs(argc, argv);

    Vcitadel *top = new Vcitadel{contextp};

    top->clk = 0;
    top->rst_n = 0;

    for (int i = 0; i < 200; i++) {
        top->clk = !top->clk;
        top->eval();
    }

    top->rst_n = 1;

    while (!top->exit) {
        top->clk = !top->clk;
        top->eval();

        if (top->tx_ready && top->clk == 0) {
            putc(top->tx, stdout);
            fflush(stdout);
        }
    }

    delete top;
    delete contextp;

    return 0;
}
