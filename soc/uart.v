module uart (
	input clk,
	input rst_n,

	output tx,
	input  rx,

	input  [31:0] clkdiv,

	input         re,
	input         we,
	input  [31:0] so,
	output [31:0] si,
	output        wa
);

reg [7:0] recv_buf;
reg [3:0] recv_ctr;
reg recv_buf_valid;

reg [9:0] send_buf;
reg [3:0] send_ctr;

reg [16:0] div;

assign si = recv_buf_valid ? recv_buf : ~0;
assign tx = send_ctr > 0 ? send_buf[9] : 1;
assign wa = we || (send_ctr > 0);

always @ (posedge clk) begin
    if (!rst_n) begin
        recv_buf <= 0;
        recv_ctr <= 0;
        recv_buf_valid <= 0;
        send_buf <= 0;
        send_ctr <= 0;
        div <= 0;
    end else if (div == 0) begin
        div <= clkdiv;

        if (re) begin
            recv_buf_valid <= 0;
        end

        if (we) begin
            send_buf <= (so << 1) | 1;

            send_ctr <= 10;
        end

        if (recv_ctr > 0) begin
            recv_ctr <= recv_ctr - 1;

            recv_buf <= recv_buf << 1;
            recv_buf[0] <= rx;

            if (recv_ctr == 1) begin
                recv_buf_valid <= 1;
            end
        end else if (rx == 0) begin
            recv_ctr <= 8;
        end

        if (send_ctr > 0) begin
            send_ctr <= send_ctr - 1;

            send_buf <= send_buf << 1;
        end
    end else begin
        div <= div - 1;
    end 
end

endmodule
