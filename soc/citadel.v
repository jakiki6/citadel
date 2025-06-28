`include "soc/picorv32.v"
`include "soc/uart.v"

module citadel #(
        SRAM_SIZE = 65536
    ) (
        // System clock + reset
        input                    r_clk,
        input                    rst_n,

        // IO
        input                    rx,
        output                   tx,

        // misc
        output                   r_panic,
        input                    recovery,
        input                    rng
    );

    reg  [ 7: 0] mem [SRAM_SIZE];

    initial begin
    `include "bootrom/bootrom.v"
    end

    reg panic;
    assign r_panic = panic;

    always @ (posedge r_clk) begin
        if (!rst_n) panic <= 0;
    end

    wire clk;
    assign clk = r_clk && !panic;

    reg mem_valid;
    reg mem_ready;

    reg [31: 0] mem_addr;
    reg [31: 0] mem_wdata;
    reg [ 3: 0] mem_wstrb;
    reg [31: 0] mem_rdata;

    reg uart0_we;
    reg uart0_re;
    wire [31:0] uart0_di;
    reg [31:0] uart0_do;
    wire uart0_wait;

    uart uart0 (
             .clk (clk),
             .rst_n (rst_n),

             .tx (tx),
             .rx (rx),

             .we (uart0_we),
             .re (uart0_re),
             .si (uart0_di),
             .so (uart0_do),
             .wa (uart0_wait)
         );

    wire core_panic;
    always @ (posedge core_panic) begin
        panic <= 1;
    end

    picorv32 #(
                 .STACKADDR (SRAM_SIZE - 4)
             ) core (
                 .clk (clk),
                 .resetn (rst_n),
                 .trap (core_panic),

                 .mem_valid (mem_valid),
                 .mem_ready (mem_ready),

                 .mem_addr  (mem_addr),
                 .mem_wdata (mem_wdata),
                 .mem_wstrb (mem_wstrb),
                 .mem_rdata (mem_rdata)
             );

    always @ (posedge clk) begin
        if (mem_valid && !mem_ready) begin
            if (mem_addr[31:24] == 8'b0 && mem_addr[23:0] < SRAM_SIZE) begin
                if (mem_wstrb == 4'b0000) begin
                    mem_rdata[ 7: 0] <= mem[mem_addr[23:0] + 0];
                    mem_rdata[15: 8] <= mem[mem_addr[23:0] + 1];
                    mem_rdata[23:16] <= mem[mem_addr[23:0] + 2];
                    mem_rdata[31:24] <= mem[mem_addr[23:0] + 3];
                end else begin
                    if (mem_wstrb[0]) mem[mem_addr[23:0] + 0] <= mem_wdata[ 7: 0];
                    if (mem_wstrb[1]) mem[mem_addr[23:0] + 1] <= mem_wdata[15: 8];
                    if (mem_wstrb[2]) mem[mem_addr[23:0] + 2] <= mem_wdata[23:16];
                    if (mem_wstrb[3]) mem[mem_addr[23:0] + 3] <= mem_wdata[31:24];
                end
            end else if (mem_addr == 32'h01000000) begin
                if (mem_wstrb != 4'b0) begin
                    panic <= 1;
                end else if (mem_wstrb[3:0] == 4'b0) begin
                    mem_rdata[31:0] <= 0;
                    mem_rdata[0] <= uart0_di != ~0;
                    mem_rdata[1] <= uart0_wait;
                    mem_rdata[2] <= recovery;
                    mem_rdata[3] <= rng;
                end
            end else if (mem_addr == 32'h01000004) begin
                if (mem_wstrb == 4'b0000) begin
                    mem_rdata <= uart0_di;
                    uart0_re <= 1;
                end else if (!uart0_wait) begin
                    uart0_do <= mem_wdata;
                    uart0_we <= 1;
                end
            end else begin
                panic <= 1;
            end

            mem_ready <= 1;
        end else begin
            mem_ready <= 0;
            uart0_re <= 0;
            uart0_we <= 0;
        end
    end

endmodule
