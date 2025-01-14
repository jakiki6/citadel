module spi (
	input clk,
	input rst_n,

	output mosi,
	input miso,

	input [31:0] clkdiv,

	input [7:0] so,
    output [7:0] si,
    input ex
);

reg [7:0] recv_buf;
reg [7:0] send_buf;
reg [3:0] exchange_ctr;
reg recv_buf_valid;

reg [16:0] div;

assign si = recv_buf_valid ? recv_buf : ~0;
assign tx = exchange_ctr > 0 ? send_buf[7] : 1;

always @ (posedge clk) begin
    if (!rst_n) begin
        recv_buf <= 0;
        send_buf <= 0;
        recv_buf_valid <= 0;
        exchange_ctr <= 0;
        div <= 0;
    end else if (div == 0) begin
        div <= clkdiv;

    end else begin
        div <= div - 1;
    end 
end

endmodule
