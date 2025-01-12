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

endmodule
