module spi (
        input clk,
        input rst_n,

        output mosi,
        input miso,
        output sclk,

        input [31:0] clkdiv,

        input [7:0] so,
        output [7:0] si,
        input ex,
        input ack,
        output wa
    );

    reg [7:0] recv_buf;
    reg [7:0] send_buf;
    reg [3:0] exchange_ctr;
    reg recv_buf_valid;

    reg [16:0] div;

    assign si = recv_buf_valid ? recv_buf : ~0;
    assign mosi = exchange_ctr > 0 ? send_buf[7] : 1;
    assign sclk = (clk && (exchange_ctr != 0)) | (exchange_ctr == 0);
    assign wa = exchange_ctr > 0;

    always @ (negedge clk) begin
        if (div == clkdiv) begin
            if (exchange_ctr > 0) begin
                exchange_ctr <= exchange_ctr - 1;

                recv_buf <= recv_buf << 1;
                recv_buf[0] <= miso;

                if (exchange_ctr == 1) begin
                    recv_buf_valid <= 1;
                end
            end
        end
    end

    always @ (posedge clk) begin
        if (ack) begin
            recv_buf_valid <= 0;
        end

        if (!rst_n) begin
            recv_buf <= 0;
            send_buf <= 0;
            recv_buf_valid <= 0;
            exchange_ctr <= 0;
            div <= 0;
        end else if (div == 0) begin
            div <= clkdiv;

            if (ex && exchange_ctr == 0) begin
                send_buf <= so;
                exchange_ctr <= 8;
                recv_buf_valid <= 0;
            end

            if (exchange_ctr > 0) begin
                send_buf <= send_buf << 1;
            end
        end else begin
            div <= div - 1;
        end
    end

endmodule
