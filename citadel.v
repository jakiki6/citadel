module citadel #(
    SRAM_SIZE = 65536
) (
    // System clock + reset
    input wire               r_clk,
    input wire               rst_n,

    // IO
    output wire [7:0]        tx,
    input  wire [7:0]        rx,
    output wire [0:0]        tx_ready,
    input  wire [0:0]        rx_ready,
    output wire [0:0]        rx_ack,

    // power
    output wire [0:0]        panic
);

reg  [ 7: 0] mem [SRAM_SIZE];

initial begin
    integer n;
    for (n = 0; n < SRAM_SIZE; n = n + 4) begin
        mem[n + 0] = 8'h00;
        mem[n + 1] = 8'h00;
        mem[n + 2] = 8'h00;
        mem[n + 3] = 8'h13;
    end

    $readmemh("bootrom/rom.hex", mem);
end

initial panic[0] <= 0;
wire clk;
assign clk = r_clk && !panic;

wire [ 0: 0] mem_valid;
wire [ 0: 0] mem_ready;

wire [31: 0] mem_addr;
wire [31: 0] mem_wdata;
wire [ 3: 0] mem_wstrb;
wire [31: 0] mem_rdata;

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
        end else if (mem_addr == 32'h01000000 && mem_wstrb && mem_wdata[7:0] == 8'h42) begin
            panic[0] <= 1;
        end else if (mem_addr == 32'h01000004) begin
            if (mem_wstrb == 4'b0000) begin
                if (rx_ready) begin
                    mem_rdata[31:0] <= rx;
                    rx_ack[0] <= 1;
                end
            end else begin
                tx[7:0] <= mem_wdata[7:0];
                tx_ready[0] <= 1;
            end
        end else if (mem_addr == 32'h01000008) begin
            if (mem_wstrb == 4'b0000) begin
                mem_rdata[31:0] <= rx_ready;
            end
        end else begin
            if (!(mem_wstrb == 4'b0000)) begin
                mem_rdata[31: 0] <= 32'b0;
            end
        end

        mem_ready[0] <= 1;
    end else begin
        mem_ready[0] <= 0;
        rx_ack[0] <= 0;
        tx_ready[0] <= 0;
    end
end

endmodule
