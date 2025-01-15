`include "soc/picorv32.v"
`include "soc/uart.v"
`include "soc/spi.v"

module citadel #(
        SRAM_SIZE = 65536
    ) (
        // System clock + reset
        input                    r_clk,
        input                    rst_n,

        // IO
        input                    rx,
        output                   tx,
        input                    miso,
        output                   mosi,
        output                   cs,

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

    reg [31:0] spi0_do;
    reg [31:0] spi0_di;
    reg spi0_ex;
    reg spi0_ack;
    reg spi0_wait;

    spi spi0 (
            .clk (clk),
            .rst_n (rst_n),

            .mosi (mosi),
            .miso (miso),
            .cs (cs),

            .si (spi0_di),
            .so (spi0_do),
            .ex (spi0_ex),
            .ack (spi0_ack),
            .wa (spi0_wait)
        );

    picorv32 #(
                 .BARREL_SHIFTER (1),
                 .TWO_CYCLE_COMPARE (1),
                 .TWO_CYCLE_ALU (1),
                 .COMPRESSED_ISA (1),
                 .ENABLE_MUL (1),
                 .ENABLE_FAST_MUL (1),
                 .ENABLE_DIV (1),
                 .STACKADDR (SRAM_SIZE - 4)
             ) core (
                 .clk (clk),
                 .resetn (rst_n),

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
                    mem_rdata[4] <= spi0_di != ~0;
                    mem_rdata[5] <= spi0_wait;
                end
            end else if (mem_addr == 32'h01000004) begin
                if (mem_wstrb == 4'b0000) begin
                    mem_rdata <= uart0_di;
                    uart0_re <= 1;
                end else if (!uart0_wait) begin
                    uart0_do <= mem_wdata;
                    uart0_we <= 1;
                end
            end else if (mem_addr == 32'h01000008) begin
                if (mem_wstrb == 4'b0000) begin
                    mem_rdata <= spi0_di;
                    spi0_ack <= 1;
                end else if (!spi0_wait) begin
                    spi0_do <= mem_wdata;
                    spi0_ex <= 1;
                end
            end else begin
                panic <= 1;
            end

            mem_ready <= 1;
        end else begin
            mem_ready <= 0;
            uart0_re <= 0;
            uart0_we <= 0;
            spi0_ack <= 0;
            spi0_ex <= 0;
        end
    end

endmodule
