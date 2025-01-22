/*
 *  PicoRV32 -- A Small RISC-V (RV32I) Processor Core
 *
 *  Copyright (C) 2015  Claire Xenia Wolf <claire@yosyshq.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Taken from https://github.com/YosysHQ/picorv32
 */

/* verilator lint_off WIDTH */
/* verilator lint_off PINMISSING */
/* verilator lint_off CASEOVERLAP */
/* verilator lint_off CASEINCOMPLETE */

`timescale 1 ns / 1 ps

/***************************************************************
 * picorv32
 ***************************************************************/

module picorv32 #(
        parameter [31:0] STACKADDR = 32'h ffff_ffff
    ) (
        input clk, resetn,
        output reg trap,

        output reg        mem_valid,
        output reg        mem_instr,
        input             mem_ready,

        output reg [31:0] mem_addr,
        output reg [31:0] mem_wdata,
        output reg [ 3:0] mem_wstrb,
        input      [31:0] mem_rdata
    );

    localparam integer regfile_size = 32;
    localparam integer regindex_bits = 5;

    reg [63:0] count_cycle, count_instr;
    reg [31:0] reg_pc, reg_next_pc, reg_op1, reg_op2, reg_out;
    reg [4:0] reg_sh;

    reg [31:0] next_insn_opcode;

    wire [31:0] next_pc;

    reg [31:0] cpuregs [0:regfile_size-1];

    wire mem_la_read;
    wire mem_la_write;
    reg [31:0] mem_la_addr;
    reg [31:0] mem_la_wdata;
    reg [3:0] mem_la_wstrb;

    integer i;

    // Internal PCPI Cores

    wire        pcpi_mul_wr;
    wire [31:0] pcpi_mul_rd;
    wire        pcpi_mul_wait;
    wire        pcpi_mul_ready;

    wire        pcpi_div_wr;
    wire [31:0] pcpi_div_rd;
    wire        pcpi_div_wait;
    wire        pcpi_div_ready;

    reg        pcpi_int_wr;
    reg [31:0] pcpi_int_rd;
    reg        pcpi_int_wait;
    reg        pcpi_int_ready;

    // Pico Co-Processor Interface (PCPI)
    reg pcpi_valid;
    reg [31:0] pcpi_insn;
    wire [31:0] pcpi_rs1;
    wire [31:0] pcpi_rs2;
    wire pcpi_wr;
    wire [31:0] pcpi_rd;
    wire pcpi_wait;
    wire pcpi_ready;

    picorv32_pcpi_fast_mul pcpi_mul (
                                       .clk       (clk            ),
                                       .resetn    (resetn         ),
                                       .pcpi_valid(pcpi_valid     ),
                                       .pcpi_insn (pcpi_insn      ),
                                       .pcpi_rs1  (pcpi_rs1       ),
                                       .pcpi_rs2  (pcpi_rs2       ),
                                       .pcpi_wr   (pcpi_mul_wr    ),
                                       .pcpi_rd   (pcpi_mul_rd    ),
                                       .pcpi_wait (pcpi_mul_wait  ),
                                       .pcpi_ready(pcpi_mul_ready ));

    picorv32_pcpi_div pcpi_div (
                                  .clk       (clk            ),
                                  .resetn    (resetn         ),
                                  .pcpi_valid(pcpi_valid     ),
                                  .pcpi_insn (pcpi_insn      ),
                                  .pcpi_rs1  (pcpi_rs1       ),
                                  .pcpi_rs2  (pcpi_rs2       ),
                                  .pcpi_wr   (pcpi_div_wr    ),
                                  .pcpi_rd   (pcpi_div_rd    ),
                                  .pcpi_wait (pcpi_div_wait  ),
                                  .pcpi_ready(pcpi_div_ready ));

    always @* begin
        pcpi_int_wr = 0;
        pcpi_int_rd = 32'bx;
        pcpi_int_wait  = |{pcpi_mul_wait,  pcpi_div_wait};
        pcpi_int_ready = |{pcpi_mul_ready, pcpi_div_ready};

        (* parallel_case *)
        case (1'b1)
            pcpi_mul_ready: begin
                pcpi_int_wr = pcpi_mul_wr;
                pcpi_int_rd = pcpi_mul_rd;
            end
            pcpi_div_ready: begin
                pcpi_int_wr = pcpi_div_wr;
                pcpi_int_rd = pcpi_div_rd;
            end
        endcase
    end


    // Memory Interface

    reg [1:0] mem_state;
    reg [1:0] mem_wordsize;
    reg [31:0] mem_rdata_word;
    reg [31:0] mem_rdata_q;
    reg mem_do_prefetch;
    reg mem_do_rinst;
    reg mem_do_rdata;
    reg mem_do_wdata;

    wire mem_xfer;
    reg mem_la_secondword, mem_la_firstword_reg, last_mem_valid;
    wire mem_la_firstword = (mem_do_prefetch || mem_do_rinst) && next_pc[1] && !mem_la_secondword;
    wire mem_la_firstword_xfer = mem_xfer && (!last_mem_valid ? mem_la_firstword : mem_la_firstword_reg);

    reg prefetched_high_word;
    reg clear_prefetched_high_word;
    reg [15:0] mem_16bit_buffer;

    wire [31:0] mem_rdata_latched_noshuffle;
    wire [31:0] mem_rdata_latched;

    wire mem_la_use_prefetched_high_word = mem_la_firstword && prefetched_high_word && !clear_prefetched_high_word;
    assign mem_xfer = (mem_valid && mem_ready) || (mem_la_use_prefetched_high_word && mem_do_rinst);

    wire mem_busy = |{mem_do_prefetch, mem_do_rinst, mem_do_rdata, mem_do_wdata};
    wire mem_done = resetn && ((mem_xfer && |mem_state && (mem_do_rinst || mem_do_rdata || mem_do_wdata)) || (&mem_state && mem_do_rinst)) &&
         (!mem_la_firstword || (~&mem_rdata_latched[1:0] && mem_xfer));

    assign mem_la_write = resetn && !mem_state && mem_do_wdata;
    assign mem_la_read = resetn && ((!mem_la_use_prefetched_high_word && !mem_state && (mem_do_rinst || mem_do_prefetch || mem_do_rdata)) ||
                                    (mem_xfer && (!last_mem_valid ? mem_la_firstword : mem_la_firstword_reg) && !mem_la_secondword && &mem_rdata_latched[1:0]));
    assign mem_la_addr = (mem_do_prefetch || mem_do_rinst) ? {next_pc[31:2] + mem_la_firstword_xfer, 2'b00} : {reg_op1[31:2], 2'b00};

    assign mem_rdata_latched_noshuffle = mem_xfer ? mem_rdata : mem_rdata_q;

    assign mem_rdata_latched = mem_la_use_prefetched_high_word ? {16'bx, mem_16bit_buffer} :
           mem_la_secondword ? {mem_rdata_latched_noshuffle[15:0], mem_16bit_buffer} :
           mem_la_firstword ? {16'bx, mem_rdata_latched_noshuffle[31:16]} : mem_rdata_latched_noshuffle;

    always @(posedge clk) begin
        if (!resetn) begin
            mem_la_firstword_reg <= 0;
            last_mem_valid <= 0;
        end else begin
            if (!last_mem_valid)
                mem_la_firstword_reg <= mem_la_firstword;
            last_mem_valid <= mem_valid && !mem_ready;
        end
    end

    always @* begin
        (* full_case *)
        case (mem_wordsize)
            0: begin
                mem_la_wdata = reg_op2;
                mem_la_wstrb = 4'b1111;
                mem_rdata_word = mem_rdata;
            end
            1: begin
                mem_la_wdata = {2{reg_op2[15:0]}};
                mem_la_wstrb = reg_op1[1] ? 4'b1100 : 4'b0011;
                case (reg_op1[1])
                    1'b0: mem_rdata_word = {16'b0, mem_rdata[15: 0]};
                    1'b1: mem_rdata_word = {16'b0, mem_rdata[31:16]};
                endcase
            end
            2: begin
                mem_la_wdata = {4{reg_op2[7:0]}};
                mem_la_wstrb = 4'b0001 << reg_op1[1:0];
                case (reg_op1[1:0])
                    2'b00: mem_rdata_word = {24'b0, mem_rdata[ 7: 0]};
                    2'b01: mem_rdata_word = {24'b0, mem_rdata[15: 8]};
                    2'b10: mem_rdata_word = {24'b0, mem_rdata[23:16]};
                    2'b11: mem_rdata_word = {24'b0, mem_rdata[31:24]};
                endcase
            end
        endcase
    end

    always @(posedge clk) begin
        if (mem_xfer) begin
            mem_rdata_q <= mem_rdata_latched;
            next_insn_opcode <= mem_rdata_latched;
        end

        if (mem_done && (mem_do_prefetch || mem_do_rinst)) begin
            case (mem_rdata_latched[1:0])
                2'b00: begin // Quadrant 0
                    case (mem_rdata_latched[15:13])
                        3'b000: begin // C.ADDI4SPN
                            mem_rdata_q[14:12] <= 3'b000;
                            mem_rdata_q[31:20] <= {2'b0, mem_rdata_latched[10:7], mem_rdata_latched[12:11], mem_rdata_latched[5], mem_rdata_latched[6], 2'b00};
                        end
                        3'b010: begin // C.LW
                            mem_rdata_q[31:20] <= {5'b0, mem_rdata_latched[5], mem_rdata_latched[12:10], mem_rdata_latched[6], 2'b00};
                            mem_rdata_q[14:12] <= 3'b 010;
                        end
                        3'b 110: begin // C.SW
                            {mem_rdata_q[31:25], mem_rdata_q[11:7]} <= {5'b0, mem_rdata_latched[5], mem_rdata_latched[12:10], mem_rdata_latched[6], 2'b00};
                            mem_rdata_q[14:12] <= 3'b 010;
                        end
                    endcase
                end
                2'b01: begin // Quadrant 1
                    case (mem_rdata_latched[15:13])
                        3'b 000: begin // C.ADDI
                            mem_rdata_q[14:12] <= 3'b000;
                            mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                        end
                        3'b 010: begin // C.LI
                            mem_rdata_q[14:12] <= 3'b000;
                            mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                        end
                        3'b 011: begin
                            if (mem_rdata_latched[11:7] == 2) begin // C.ADDI16SP
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[4:3],
                                                               mem_rdata_latched[5], mem_rdata_latched[2], mem_rdata_latched[6], 4'b 0000});
                            end else begin // C.LUI
                                mem_rdata_q[31:12] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                            end
                        end
                        3'b100: begin
                            if (mem_rdata_latched[11:10] == 2'b00) begin // C.SRLI
                                mem_rdata_q[31:25] <= 7'b0000000;
                                mem_rdata_q[14:12] <= 3'b 101;
                            end
                            if (mem_rdata_latched[11:10] == 2'b01) begin // C.SRAI
                                mem_rdata_q[31:25] <= 7'b0100000;
                                mem_rdata_q[14:12] <= 3'b 101;
                            end
                            if (mem_rdata_latched[11:10] == 2'b10) begin // C.ANDI
                                mem_rdata_q[14:12] <= 3'b111;
                                mem_rdata_q[31:20] <= $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
                            end
                            if (mem_rdata_latched[12:10] == 3'b011) begin // C.SUB, C.XOR, C.OR, C.AND
                                if (mem_rdata_latched[6:5] == 2'b00) mem_rdata_q[14:12] <= 3'b000;
                                if (mem_rdata_latched[6:5] == 2'b01) mem_rdata_q[14:12] <= 3'b100;
                                if (mem_rdata_latched[6:5] == 2'b10) mem_rdata_q[14:12] <= 3'b110;
                                if (mem_rdata_latched[6:5] == 2'b11) mem_rdata_q[14:12] <= 3'b111;
                                mem_rdata_q[31:25] <= mem_rdata_latched[6:5] == 2'b00 ? 7'b0100000 : 7'b0000000;
                            end
                        end
                        3'b 110: begin // C.BEQZ
                            mem_rdata_q[14:12] <= 3'b000;
                            { mem_rdata_q[31], mem_rdata_q[7], mem_rdata_q[30:25], mem_rdata_q[11:8] } <=
                            $signed({mem_rdata_latched[12], mem_rdata_latched[6:5], mem_rdata_latched[2],
                                     mem_rdata_latched[11:10], mem_rdata_latched[4:3]});
                        end
                        3'b 111: begin // C.BNEZ
                            mem_rdata_q[14:12] <= 3'b001;
                            { mem_rdata_q[31], mem_rdata_q[7], mem_rdata_q[30:25], mem_rdata_q[11:8] } <=
                            $signed({mem_rdata_latched[12], mem_rdata_latched[6:5], mem_rdata_latched[2],
                                     mem_rdata_latched[11:10], mem_rdata_latched[4:3]});
                        end
                    endcase
                end
                2'b10: begin // Quadrant 2
                    case (mem_rdata_latched[15:13])
                        3'b000: begin // C.SLLI
                            mem_rdata_q[31:25] <= 7'b0000000;
                            mem_rdata_q[14:12] <= 3'b 001;
                        end
                        3'b010: begin // C.LWSP
                            mem_rdata_q[31:20] <= {4'b0, mem_rdata_latched[3:2], mem_rdata_latched[12], mem_rdata_latched[6:4], 2'b00};
                            mem_rdata_q[14:12] <= 3'b 010;
                        end
                        3'b100: begin
                            if (mem_rdata_latched[12] == 0 && mem_rdata_latched[6:2] == 0) begin // C.JR
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:20] <= 12'b0;
                            end
                            if (mem_rdata_latched[12] == 0 && mem_rdata_latched[6:2] != 0) begin // C.MV
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:25] <= 7'b0000000;
                            end
                            if (mem_rdata_latched[12] != 0 && mem_rdata_latched[11:7] != 0 && mem_rdata_latched[6:2] == 0) begin // C.JALR
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:20] <= 12'b0;
                            end
                            if (mem_rdata_latched[12] != 0 && mem_rdata_latched[6:2] != 0) begin // C.ADD
                                mem_rdata_q[14:12] <= 3'b000;
                                mem_rdata_q[31:25] <= 7'b0000000;
                            end
                        end
                        3'b110: begin // C.SWSP
                            {mem_rdata_q[31:25], mem_rdata_q[11:7]} <= {4'b0, mem_rdata_latched[8:7], mem_rdata_latched[12:9], 2'b00};
                            mem_rdata_q[14:12] <= 3'b 010;
                        end
                    endcase
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (!resetn || trap) begin
            if (!resetn)
                mem_state <= 0;
            if (!resetn || mem_ready)
                mem_valid <= 0;
            mem_la_secondword <= 0;
            prefetched_high_word <= 0;
        end else begin
            if (mem_la_read || mem_la_write) begin
                mem_addr <= mem_la_addr;
                mem_wstrb <= mem_la_wstrb & {4{mem_la_write}};
            end
            if (mem_la_write) begin
                mem_wdata <= mem_la_wdata;
            end
            case (mem_state)
                0: begin
                    if (mem_do_prefetch || mem_do_rinst || mem_do_rdata) begin
                        mem_valid <= !mem_la_use_prefetched_high_word;
                        mem_instr <= mem_do_prefetch || mem_do_rinst;
                        mem_wstrb <= 0;
                        mem_state <= 1;
                    end
                    if (mem_do_wdata) begin
                        mem_valid <= 1;
                        mem_instr <= 0;
                        mem_state <= 2;
                    end
                end
                1: begin
                    if (mem_xfer) begin
                        if (mem_la_read) begin
                            mem_valid <= 1;
                            mem_la_secondword <= 1;
                            if (!mem_la_use_prefetched_high_word)
                                mem_16bit_buffer <= mem_rdata[31:16];
                        end else begin
                            mem_valid <= 0;
                            mem_la_secondword <= 0;
                            if (!mem_do_rdata) begin
                                if (~&mem_rdata[1:0] || mem_la_secondword) begin
                                    mem_16bit_buffer <= mem_rdata[31:16];
                                    prefetched_high_word <= 1;
                                end else begin
                                    prefetched_high_word <= 0;
                                end
                            end
                            mem_state <= mem_do_rinst || mem_do_rdata ? 0 : 3;
                        end
                    end
                end
                2: begin
                    if (mem_xfer) begin
                        mem_valid <= 0;
                        mem_state <= 0;
                    end
                end
                3: begin
                    if (mem_do_rinst) begin
                        mem_state <= 0;
                    end
                end
            endcase
        end

        if (clear_prefetched_high_word)
            prefetched_high_word <= 0;
    end


    // Instruction Decoder

    reg instr_lui, instr_auipc, instr_jal, instr_jalr;
    reg instr_beq, instr_bne, instr_blt, instr_bge, instr_bltu, instr_bgeu;
    reg instr_lb, instr_lh, instr_lw, instr_lbu, instr_lhu, instr_sb, instr_sh, instr_sw;
    reg instr_addi, instr_slti, instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai;
    reg instr_add, instr_sub, instr_sll, instr_slt, instr_sltu, instr_xor, instr_srl, instr_sra, instr_or, instr_and;
    reg instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh, instr_ecall_ebreak, instr_fence;
    wire instr_trap;

    reg [regindex_bits-1:0] decoded_rd, decoded_rs1;
    reg [4:0] decoded_rs2;
    reg [31:0] decoded_imm, decoded_imm_j;
    reg decoder_trigger;
    reg decoder_trigger_q;
    reg decoder_pseudo_trigger;
    reg decoder_pseudo_trigger_q;
    reg compressed_instr;

    reg is_lui_auipc_jal;
    reg is_lb_lh_lw_lbu_lhu;
    reg is_slli_srli_srai;
    reg is_jalr_addi_slti_sltiu_xori_ori_andi;
    reg is_sb_sh_sw;
    reg is_sll_srl_sra;
    reg is_lui_auipc_jal_jalr_addi_add_sub;
    reg is_slti_blt_slt;
    reg is_sltiu_bltu_sltu;
    reg is_beq_bne_blt_bge_bltu_bgeu;
    reg is_lbu_lhu_lw;
    reg is_alu_reg_imm;
    reg is_alu_reg_reg;
    reg is_compare;

    assign instr_trap = !{instr_lui, instr_auipc, instr_jal, instr_jalr,
            instr_beq, instr_bne, instr_blt, instr_bge, instr_bltu, instr_bgeu,
            instr_lb, instr_lh, instr_lw, instr_lbu, instr_lhu, instr_sb, instr_sh, instr_sw,
            instr_addi, instr_slti, instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai,
            instr_add, instr_sub, instr_sll, instr_slt, instr_sltu, instr_xor, instr_srl, instr_sra, instr_or, instr_and,
            instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh, instr_fence};

    wire is_rdcycle_rdcycleh_rdinstr_rdinstrh;
    assign is_rdcycle_rdcycleh_rdinstr_rdinstrh = |{instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh};

    always @(posedge clk) begin
        is_lui_auipc_jal <= |{instr_lui, instr_auipc, instr_jal};
        is_lui_auipc_jal_jalr_addi_add_sub <= |{instr_lui, instr_auipc, instr_jal, instr_jalr, instr_addi, instr_add, instr_sub};
        is_slti_blt_slt <= |{instr_slti, instr_blt, instr_slt};
        is_sltiu_bltu_sltu <= |{instr_sltiu, instr_bltu, instr_sltu};
        is_lbu_lhu_lw <= |{instr_lbu, instr_lhu, instr_lw};
        is_compare <= |{is_beq_bne_blt_bge_bltu_bgeu, instr_slti, instr_slt, instr_sltiu, instr_sltu};

        if (mem_do_rinst && mem_done) begin
            instr_lui     <= mem_rdata_latched[6:0] == 7'b0110111;
            instr_auipc   <= mem_rdata_latched[6:0] == 7'b0010111;
            instr_jal     <= mem_rdata_latched[6:0] == 7'b1101111;
            instr_jalr    <= mem_rdata_latched[6:0] == 7'b1100111 && mem_rdata_latched[14:12] == 3'b000;

            is_beq_bne_blt_bge_bltu_bgeu <= mem_rdata_latched[6:0] == 7'b1100011;
            is_lb_lh_lw_lbu_lhu          <= mem_rdata_latched[6:0] == 7'b0000011;
            is_sb_sh_sw                  <= mem_rdata_latched[6:0] == 7'b0100011;
            is_alu_reg_imm               <= mem_rdata_latched[6:0] == 7'b0010011;
            is_alu_reg_reg               <= mem_rdata_latched[6:0] == 7'b0110011;

            { decoded_imm_j[31:20], decoded_imm_j[10:1], decoded_imm_j[11], decoded_imm_j[19:12], decoded_imm_j[0] } <= $signed({mem_rdata_latched[31:12], 1'b0});

            decoded_rd <= mem_rdata_latched[11:7];
            decoded_rs1 <= mem_rdata_latched[19:15];
            decoded_rs2 <= mem_rdata_latched[24:20];

            compressed_instr <= 0;
            if (mem_rdata_latched[1:0] != 2'b11) begin
                compressed_instr <= 1;
                decoded_rd <= 0;
                decoded_rs1 <= 0;
                decoded_rs2 <= 0;

                { decoded_imm_j[31:11], decoded_imm_j[4], decoded_imm_j[9:8], decoded_imm_j[10], decoded_imm_j[6],
                  decoded_imm_j[7], decoded_imm_j[3:1], decoded_imm_j[5], decoded_imm_j[0] } <= $signed({mem_rdata_latched[12:2], 1'b0});

                case (mem_rdata_latched[1:0])
                    2'b00: begin // Quadrant 0
                        case (mem_rdata_latched[15:13])
                            3'b000: begin // C.ADDI4SPN
                                is_alu_reg_imm <= |mem_rdata_latched[12:5];
                                decoded_rs1 <= 2;
                                decoded_rd <= 8 + mem_rdata_latched[4:2];
                            end
                            3'b010: begin // C.LW
                                is_lb_lh_lw_lbu_lhu <= 1;
                                decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                decoded_rd <= 8 + mem_rdata_latched[4:2];
                            end
                            3'b110: begin // C.SW
                                is_sb_sh_sw <= 1;
                                decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                decoded_rs2 <= 8 + mem_rdata_latched[4:2];
                            end
                        endcase
                    end
                    2'b01: begin // Quadrant 1
                        case (mem_rdata_latched[15:13])
                            3'b000: begin // C.NOP / C.ADDI
                                is_alu_reg_imm <= 1;
                                decoded_rd <= mem_rdata_latched[11:7];
                                decoded_rs1 <= mem_rdata_latched[11:7];
                            end
                            3'b001: begin // C.JAL
                                instr_jal <= 1;
                                decoded_rd <= 1;
                            end
                            3'b 010: begin // C.LI
                                is_alu_reg_imm <= 1;
                                decoded_rd <= mem_rdata_latched[11:7];
                                decoded_rs1 <= 0;
                            end
                            3'b 011: begin
                                if (mem_rdata_latched[12] || mem_rdata_latched[6:2]) begin
                                    if (mem_rdata_latched[11:7] == 2) begin // C.ADDI16SP
                                        is_alu_reg_imm <= 1;
                                        decoded_rd <= mem_rdata_latched[11:7];
                                        decoded_rs1 <= mem_rdata_latched[11:7];
                                    end else begin // C.LUI
                                        instr_lui <= 1;
                                        decoded_rd <= mem_rdata_latched[11:7];
                                        decoded_rs1 <= 0;
                                    end
                                end
                            end
                            3'b100: begin
                                if (!mem_rdata_latched[11] && !mem_rdata_latched[12]) begin // C.SRLI, C.SRAI
                                    is_alu_reg_imm <= 1;
                                    decoded_rd <= 8 + mem_rdata_latched[9:7];
                                    decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                    decoded_rs2 <= {mem_rdata_latched[12], mem_rdata_latched[6:2]};
                                end
                                if (mem_rdata_latched[11:10] == 2'b10) begin // C.ANDI
                                    is_alu_reg_imm <= 1;
                                    decoded_rd <= 8 + mem_rdata_latched[9:7];
                                    decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                end
                                if (mem_rdata_latched[12:10] == 3'b011) begin // C.SUB, C.XOR, C.OR, C.AND
                                    is_alu_reg_reg <= 1;
                                    decoded_rd <= 8 + mem_rdata_latched[9:7];
                                    decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                    decoded_rs2 <= 8 + mem_rdata_latched[4:2];
                                end
                            end
                            3'b101: begin // C.J
                                instr_jal <= 1;
                            end
                            3'b110: begin // C.BEQZ
                                is_beq_bne_blt_bge_bltu_bgeu <= 1;
                                decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                decoded_rs2 <= 0;
                            end
                            3'b111: begin // C.BNEZ
                                is_beq_bne_blt_bge_bltu_bgeu <= 1;
                                decoded_rs1 <= 8 + mem_rdata_latched[9:7];
                                decoded_rs2 <= 0;
                            end
                        endcase
                    end
                    2'b10: begin // Quadrant 2
                        case (mem_rdata_latched[15:13])
                            3'b000: begin // C.SLLI
                                if (!mem_rdata_latched[12]) begin
                                    is_alu_reg_imm <= 1;
                                    decoded_rd <= mem_rdata_latched[11:7];
                                    decoded_rs1 <= mem_rdata_latched[11:7];
                                    decoded_rs2 <= {mem_rdata_latched[12], mem_rdata_latched[6:2]};
                                end
                            end
                            3'b010: begin // C.LWSP
                                if (mem_rdata_latched[11:7]) begin
                                    is_lb_lh_lw_lbu_lhu <= 1;
                                    decoded_rd <= mem_rdata_latched[11:7];
                                    decoded_rs1 <= 2;
                                end
                            end
                            3'b100: begin
                                if (mem_rdata_latched[12] == 0 && mem_rdata_latched[11:7] != 0 && mem_rdata_latched[6:2] == 0) begin // C.JR
                                    instr_jalr <= 1;
                                    decoded_rd <= 0;
                                    decoded_rs1 <= mem_rdata_latched[11:7];
                                end
                                if (mem_rdata_latched[12] == 0 && mem_rdata_latched[6:2] != 0) begin // C.MV
                                    is_alu_reg_reg <= 1;
                                    decoded_rd <= mem_rdata_latched[11:7];
                                    decoded_rs1 <= 0;
                                    decoded_rs2 <= mem_rdata_latched[6:2];
                                end
                                if (mem_rdata_latched[12] != 0 && mem_rdata_latched[11:7] != 0 && mem_rdata_latched[6:2] == 0) begin // C.JALR
                                    instr_jalr <= 1;
                                    decoded_rd <= 1;
                                    decoded_rs1 <= mem_rdata_latched[11:7];
                                end
                                if (mem_rdata_latched[12] != 0 && mem_rdata_latched[6:2] != 0) begin // C.ADD
                                    is_alu_reg_reg <= 1;
                                    decoded_rd <= mem_rdata_latched[11:7];
                                    decoded_rs1 <= mem_rdata_latched[11:7];
                                    decoded_rs2 <= mem_rdata_latched[6:2];
                                end
                            end
                            3'b110: begin // C.SWSP
                                is_sb_sh_sw <= 1;
                                decoded_rs1 <= 2;
                                decoded_rs2 <= mem_rdata_latched[6:2];
                            end
                        endcase
                    end
                endcase
            end
        end

        if (decoder_trigger && !decoder_pseudo_trigger) begin
            instr_beq   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b000;
            instr_bne   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b001;
            instr_blt   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b100;
            instr_bge   <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b101;
            instr_bltu  <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b110;
            instr_bgeu  <= is_beq_bne_blt_bge_bltu_bgeu && mem_rdata_q[14:12] == 3'b111;

            instr_lb    <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b000;
            instr_lh    <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b001;
            instr_lw    <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b010;
            instr_lbu   <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b100;
            instr_lhu   <= is_lb_lh_lw_lbu_lhu && mem_rdata_q[14:12] == 3'b101;

            instr_sb    <= is_sb_sh_sw && mem_rdata_q[14:12] == 3'b000;
            instr_sh    <= is_sb_sh_sw && mem_rdata_q[14:12] == 3'b001;
            instr_sw    <= is_sb_sh_sw && mem_rdata_q[14:12] == 3'b010;

            instr_addi  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b000;
            instr_slti  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b010;
            instr_sltiu <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b011;
            instr_xori  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b100;
            instr_ori   <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b110;
            instr_andi  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b111;

            instr_slli  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000;
            instr_srli  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000;
            instr_srai  <= is_alu_reg_imm && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000;

            instr_add   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b000 && mem_rdata_q[31:25] == 7'b0000000;
            instr_sub   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b000 && mem_rdata_q[31:25] == 7'b0100000;
            instr_sll   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000;
            instr_slt   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b010 && mem_rdata_q[31:25] == 7'b0000000;
            instr_sltu  <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b011 && mem_rdata_q[31:25] == 7'b0000000;
            instr_xor   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b100 && mem_rdata_q[31:25] == 7'b0000000;
            instr_srl   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000;
            instr_sra   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000;
            instr_or    <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b110 && mem_rdata_q[31:25] == 7'b0000000;
            instr_and   <= is_alu_reg_reg && mem_rdata_q[14:12] == 3'b111 && mem_rdata_q[31:25] == 7'b0000000;

            instr_rdcycle  <= ((mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11000000000000000010) ||
                               (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11000000000100000010));
            instr_rdcycleh <= ((mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11001000000000000010) ||
                               (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11001000000100000010));
            instr_rdinstr  <=  (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11000000001000000010);
            instr_rdinstrh <=  (mem_rdata_q[6:0] == 7'b1110011 && mem_rdata_q[31:12] == 'b11001000001000000010);

            instr_ecall_ebreak <= ((mem_rdata_q[6:0] == 7'b1110011 && !mem_rdata_q[31:21] && !mem_rdata_q[19:7]) ||
                                   (mem_rdata_q[15:0] == 16'h9002));
            instr_fence <= (mem_rdata_q[6:0] == 7'b0001111 && !mem_rdata_q[14:12]);

            is_slli_srli_srai <= is_alu_reg_imm && |{
                                  mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000,
                                  mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000,
                                  mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000
                              };

            is_jalr_addi_slti_sltiu_xori_ori_andi <= instr_jalr || is_alu_reg_imm && |{
                                                      mem_rdata_q[14:12] == 3'b000,
                                                      mem_rdata_q[14:12] == 3'b010,
                                                      mem_rdata_q[14:12] == 3'b011,
                                                      mem_rdata_q[14:12] == 3'b100,
                                                      mem_rdata_q[14:12] == 3'b110,
                                                      mem_rdata_q[14:12] == 3'b111
                                                  };

            is_sll_srl_sra <= is_alu_reg_reg && |{
                               mem_rdata_q[14:12] == 3'b001 && mem_rdata_q[31:25] == 7'b0000000,
                               mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0000000,
                               mem_rdata_q[14:12] == 3'b101 && mem_rdata_q[31:25] == 7'b0100000
                           };

            is_lui_auipc_jal_jalr_addi_add_sub <= 0;
            is_compare <= 0;

            (* parallel_case *)
            case (1'b1)
                instr_jal:
                    decoded_imm <= decoded_imm_j;
                |{instr_lui, instr_auipc}:
                    decoded_imm <= mem_rdata_q[31:12] << 12;
                |{instr_jalr, is_lb_lh_lw_lbu_lhu, is_alu_reg_imm}:
                    decoded_imm <= $signed(mem_rdata_q[31:20]);
                is_beq_bne_blt_bge_bltu_bgeu:
                    decoded_imm <= $signed({mem_rdata_q[31], mem_rdata_q[7], mem_rdata_q[30:25], mem_rdata_q[11:8], 1'b0});
                is_sb_sh_sw:
                    decoded_imm <= $signed({mem_rdata_q[31:25], mem_rdata_q[11:7]});
                default:
                    decoded_imm <= 1'bx;
            endcase
        end

        if (!resetn) begin
            is_beq_bne_blt_bge_bltu_bgeu <= 0;
            is_compare <= 0;

            instr_beq   <= 0;
            instr_bne   <= 0;
            instr_blt   <= 0;
            instr_bge   <= 0;
            instr_bltu  <= 0;
            instr_bgeu  <= 0;

            instr_addi  <= 0;
            instr_slti  <= 0;
            instr_sltiu <= 0;
            instr_xori  <= 0;
            instr_ori   <= 0;
            instr_andi  <= 0;

            instr_add   <= 0;
            instr_sub   <= 0;
            instr_sll   <= 0;
            instr_slt   <= 0;
            instr_sltu  <= 0;
            instr_xor   <= 0;
            instr_srl   <= 0;
            instr_sra   <= 0;
            instr_or    <= 0;
            instr_and   <= 0;

            instr_fence <= 0;
        end
    end


    // Main State Machine

    localparam cpu_state_trap   = 8'b10000000;
    localparam cpu_state_fetch  = 8'b01000000;
    localparam cpu_state_ld_rs1 = 8'b00100000;
    localparam cpu_state_ld_rs2 = 8'b00010000;
    localparam cpu_state_exec   = 8'b00001000;
    localparam cpu_state_shift  = 8'b00000100;
    localparam cpu_state_stmem  = 8'b00000010;
    localparam cpu_state_ldmem  = 8'b00000001;

    reg [7:0] cpu_state;

    reg set_mem_do_rinst;
    reg set_mem_do_rdata;
    reg set_mem_do_wdata;

    reg latched_store;
    reg latched_stalu;
    reg latched_branch;
    reg latched_compr;
    reg latched_trace;
    reg latched_is_lu;
    reg latched_is_lh;
    reg latched_is_lb;
    reg [regindex_bits-1:0] latched_rd;

    reg [31:0] current_pc;
    assign next_pc = latched_store && latched_branch ? reg_out & ~1 : reg_next_pc;

    reg [3:0] pcpi_timeout_counter;
    reg pcpi_timeout;

    reg [31:0] alu_out, alu_out_q;
    reg alu_out_0, alu_out_0_q;
    reg alu_wait, alu_wait_2;

    reg [31:0] alu_add_sub;
    reg [31:0] alu_shl, alu_shr;
    reg alu_eq, alu_ltu, alu_lts;

    always @(posedge clk) begin
        alu_add_sub <= instr_sub ? reg_op1 - reg_op2 : reg_op1 + reg_op2;
        alu_eq <= reg_op1 == reg_op2;
        alu_lts <= $signed(reg_op1) < $signed(reg_op2);
        alu_ltu <= reg_op1 < reg_op2;
        alu_shl <= reg_op1 << reg_op2[4:0];
        alu_shr <= $signed({instr_sra || instr_srai ? reg_op1[31] : 1'b0, reg_op1}) >>> reg_op2[4:0];
    end

    always @* begin
        alu_out_0 = 'bx;
        (* parallel_case, full_case *)
        case (1'b1)
            instr_beq:
                alu_out_0 = alu_eq;
            instr_bne:
                alu_out_0 = !alu_eq;
            instr_bge:
                alu_out_0 = !alu_lts;
            instr_bgeu:
                alu_out_0 = !alu_ltu;
        endcase

        alu_out = 'bx;
        (* parallel_case, full_case *)
        case (1'b1)
            is_lui_auipc_jal_jalr_addi_add_sub:
                alu_out = alu_add_sub;
            is_compare:
                alu_out = alu_out_0;
            instr_xori || instr_xor:
                alu_out = reg_op1 ^ reg_op2;
            instr_ori || instr_or:
                alu_out = reg_op1 | reg_op2;
            instr_andi || instr_and:
                alu_out = reg_op1 & reg_op2;
            (instr_sll || instr_slli):
                alu_out = alu_shl;
            (instr_srl || instr_srli || instr_sra || instr_srai):
                alu_out = alu_shr;
        endcase
    end

    reg clear_prefetched_high_word_q;
    always @(posedge clk) clear_prefetched_high_word_q <= clear_prefetched_high_word;

    always @* begin
        clear_prefetched_high_word = clear_prefetched_high_word_q;
        if (!prefetched_high_word)
            clear_prefetched_high_word = 0;
        if (latched_branch || !resetn)
            clear_prefetched_high_word = 1;
    end

    reg cpuregs_write;
    reg [31:0] cpuregs_wrdata;
    reg [31:0] cpuregs_rs1;
    reg [31:0] cpuregs_rs2;
    reg [regindex_bits-1:0] decoded_rs;

    always @* begin
        cpuregs_write = 0;
        cpuregs_wrdata = 'bx;

        if (cpu_state == cpu_state_fetch) begin
            (* parallel_case *)
            case (1'b1)
                latched_branch: begin
                    cpuregs_wrdata = reg_pc + (latched_compr ? 2 : 4);
                    cpuregs_write = 1;
                end
                latched_store && !latched_branch: begin
                    cpuregs_wrdata = latched_stalu ? alu_out_q : reg_out;
                    cpuregs_write = 1;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (resetn && cpuregs_write && latched_rd)
            cpuregs[latched_rd] <= cpuregs_wrdata;
    end

    always @* begin
        decoded_rs = 'bx;
        cpuregs_rs1 = decoded_rs1 ? cpuregs[decoded_rs1] : 0;
        cpuregs_rs2 = decoded_rs2 ? cpuregs[decoded_rs2] : 0;
    end

    always @(posedge clk) begin
        trap <= 0;
        reg_sh <= 'bx;
        reg_out <= 'bx;
        set_mem_do_rinst = 0;
        set_mem_do_rdata = 0;
        set_mem_do_wdata = 0;

        alu_out_0_q <= alu_out_0;
        alu_out_q <= alu_out;

        alu_wait <= 0;
        alu_wait_2 <= 0;

        if (resetn && !pcpi_int_wait) begin
            if (pcpi_timeout_counter)
                pcpi_timeout_counter <= pcpi_timeout_counter - 1;
        end else
            pcpi_timeout_counter <= ~0;
        pcpi_timeout <= !pcpi_timeout_counter;

        count_cycle <= resetn ? count_cycle + 1 : 0;

        decoder_trigger <= mem_do_rinst && mem_done;
        decoder_trigger_q <= decoder_trigger;
        decoder_pseudo_trigger <= 0;
        decoder_pseudo_trigger_q <= decoder_pseudo_trigger;

        if (!resetn) begin
            reg_pc <= 0;
            reg_next_pc <= 0;
            count_instr <= 0;
            latched_store <= 0;
            latched_stalu <= 0;
            latched_branch <= 0;
            latched_trace <= 0;
            latched_is_lu <= 0;
            latched_is_lh <= 0;
            latched_is_lb <= 0;
            pcpi_timeout <= 0;
            if (~STACKADDR) begin
                latched_store <= 1;
                latched_rd <= 2;
                reg_out <= STACKADDR;
            end
            cpu_state <= cpu_state_fetch;
        end else
            (* parallel_case, full_case *)
        case (cpu_state)
            cpu_state_trap: begin
                trap <= 1;
            end

            cpu_state_fetch: begin
                mem_do_rinst <= !decoder_trigger;
                mem_wordsize <= 0;

                current_pc = reg_next_pc;

                (* parallel_case *)
                case (1'b1)
                    latched_branch: begin
                        current_pc = latched_store ? (latched_stalu ? alu_out_q : reg_out) & ~1 : reg_next_pc;
                    end
                endcase

                reg_pc <= current_pc;
                reg_next_pc <= current_pc;

                latched_store <= 0;
                latched_stalu <= 0;
                latched_branch <= 0;
                latched_is_lu <= 0;
                latched_is_lh <= 0;
                latched_is_lb <= 0;
                latched_rd <= decoded_rd;
                latched_compr <= compressed_instr;

                if (decoder_trigger) begin
                    reg_next_pc <= current_pc + (compressed_instr ? 2 : 4);
                    count_instr <= count_instr + 1;
                    if (instr_jal) begin
                        mem_do_rinst <= 1;
                        reg_next_pc <= current_pc + decoded_imm_j;
                        latched_branch <= 1;
                    end else begin
                        mem_do_rinst <= 0;
                        mem_do_prefetch <= !instr_jalr;
                        cpu_state <= cpu_state_ld_rs1;
                    end
                end
            end

            cpu_state_ld_rs1: begin
                reg_op1 <= 'bx;
                reg_op2 <= 'bx;

                (* parallel_case *)
                case (1'b1)
                    instr_trap: begin
                        reg_op1 <= cpuregs_rs1;
                        reg_sh <= cpuregs_rs2;
                        reg_op2 <= cpuregs_rs2;
                        if (pcpi_int_ready) begin
                            mem_do_rinst <= 1;
                            reg_out <= pcpi_int_rd;
                            latched_store <= pcpi_int_wr;
                            cpu_state <= cpu_state_fetch;
                        end else
                            if (pcpi_timeout || instr_ecall_ebreak) begin
                                cpu_state <= cpu_state_trap;
                            end
                    end
                    is_rdcycle_rdcycleh_rdinstr_rdinstrh: begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            instr_rdcycle:
                                reg_out <= count_cycle[31:0];
                            instr_rdcycleh:
                                reg_out <= count_cycle[63:32];
                            instr_rdinstr:
                                reg_out <= count_instr[31:0];
                            instr_rdinstrh:
                                reg_out <= count_instr[63:32];
                        endcase
                        latched_store <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                    is_lui_auipc_jal: begin
                        reg_op1 <= instr_lui ? 0 : reg_pc;
                        reg_op2 <= decoded_imm;
                        alu_wait <= 1;
                        cpu_state <= cpu_state_exec;
                    end
                    is_lb_lh_lw_lbu_lhu && !instr_trap: begin
                        reg_op1 <= cpuregs_rs1;
                        cpu_state <= cpu_state_ldmem;
                        mem_do_rinst <= 1;
                    end
                    is_jalr_addi_slti_sltiu_xori_ori_andi, is_slli_srli_srai: begin
                        reg_op1 <= cpuregs_rs1;
                        reg_op2 <= is_slli_srli_srai ? decoded_rs2 : decoded_imm;
                        alu_wait <= 1;
                        cpu_state <= cpu_state_exec;
                    end
                    default: begin
                        reg_op1 <= cpuregs_rs1;
                        reg_sh <= cpuregs_rs2;
                        reg_op2 <= cpuregs_rs2;
                        (* parallel_case *)
                        case (1'b1)
                            is_sb_sh_sw: begin
                                cpu_state <= cpu_state_stmem;
                                mem_do_rinst <= 1;
                            end
                            default: begin
                                alu_wait_2 <= is_beq_bne_blt_bge_bltu_bgeu;
                                alu_wait <= 1;
                                cpu_state <= cpu_state_exec;
                            end
                        endcase
                    end
                endcase
            end

            cpu_state_ld_rs2: begin
                reg_sh <= cpuregs_rs2;
                reg_op2 <= cpuregs_rs2;

                (* parallel_case *)
                case (1'b1)
                    instr_trap: begin
                        if (pcpi_int_ready) begin
                            mem_do_rinst <= 1;
                            reg_out <= pcpi_int_rd;
                            latched_store <= pcpi_int_wr;
                            cpu_state <= cpu_state_fetch;
                        end else
                            if (pcpi_timeout || instr_ecall_ebreak) begin
                                cpu_state <= cpu_state_trap;
                            end
                    end
                    is_sb_sh_sw: begin
                        cpu_state <= cpu_state_stmem;
                        mem_do_rinst <= 1;
                    end
                    default: begin
                        alu_wait_2 <= is_beq_bne_blt_bge_bltu_bgeu;
                        alu_wait <= 1;
                        cpu_state <= cpu_state_exec;
                    end
                endcase
            end

            cpu_state_exec: begin
                reg_out <= reg_pc + decoded_imm;
                if (alu_wait || alu_wait_2) begin
                    mem_do_rinst <= mem_do_prefetch && !alu_wait_2;
                    alu_wait <= alu_wait_2;
                end else
                    if (is_beq_bne_blt_bge_bltu_bgeu) begin
                        latched_rd <= 0;
                        latched_store <= alu_out_0_q;
                        latched_branch <= alu_out_0_q;
                        if (mem_done)
                            cpu_state <= cpu_state_fetch;
                        if (alu_out_0_q) begin
                            decoder_trigger <= 0;
                            set_mem_do_rinst = 1;
                        end
                    end else begin
                        latched_branch <= instr_jalr;
                        latched_store <= 1;
                        latched_stalu <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
            end

            cpu_state_shift: begin
                latched_store <= 1;
                if (reg_sh == 0) begin
                    reg_out <= reg_op1;
                    mem_do_rinst <= mem_do_prefetch;
                    cpu_state <= cpu_state_fetch;
                end else if (reg_sh >= 4) begin
                    (* parallel_case, full_case *)
                    case (1'b1)
                        instr_slli || instr_sll: reg_op1 <= reg_op1 << 4;
                        instr_srli || instr_srl: reg_op1 <= reg_op1 >> 4;
                        instr_srai || instr_sra: reg_op1 <= $signed(reg_op1) >>> 4;
                    endcase
                    reg_sh <= reg_sh - 4;
                end else begin
                    (* parallel_case, full_case *)
                    case (1'b1)
                        instr_slli || instr_sll: reg_op1 <= reg_op1 << 1;
                        instr_srli || instr_srl: reg_op1 <= reg_op1 >> 1;
                        instr_srai || instr_sra: reg_op1 <= $signed(reg_op1) >>> 1;
                    endcase
                    reg_sh <= reg_sh - 1;
                end
            end

            cpu_state_stmem: begin
                if (!mem_do_prefetch || mem_done) begin
                    if (!mem_do_wdata) begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            instr_sb: mem_wordsize <= 2;
                            instr_sh: mem_wordsize <= 1;
                            instr_sw: mem_wordsize <= 0;
                        endcase
                        reg_op1 <= reg_op1 + decoded_imm;
                        set_mem_do_wdata = 1;
                    end
                    if (!mem_do_prefetch && mem_done) begin
                        cpu_state <= cpu_state_fetch;
                        decoder_trigger <= 1;
                        decoder_pseudo_trigger <= 1;
                    end
                end
            end

            cpu_state_ldmem: begin
                latched_store <= 1;
                if (!mem_do_prefetch || mem_done) begin
                    if (!mem_do_rdata) begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            instr_lb || instr_lbu: mem_wordsize <= 2;
                            instr_lh || instr_lhu: mem_wordsize <= 1;
                            instr_lw: mem_wordsize <= 0;
                        endcase
                        latched_is_lu <= is_lbu_lhu_lw;
                        latched_is_lh <= instr_lh;
                        latched_is_lb <= instr_lb;
                        reg_op1 <= reg_op1 + decoded_imm;
                        set_mem_do_rdata = 1;
                    end
                    if (!mem_do_prefetch && mem_done) begin
                        (* parallel_case, full_case *)
                        case (1'b1)
                            latched_is_lu: reg_out <= mem_rdata_word;
                            latched_is_lh: reg_out <= $signed(mem_rdata_word[15:0]);
                            latched_is_lb: reg_out <= $signed(mem_rdata_word[7:0]);
                        endcase
                        decoder_trigger <= 1;
                        decoder_pseudo_trigger <= 1;
                        cpu_state <= cpu_state_fetch;
                    end
                end
            end
        endcase

        if (resetn && (mem_do_rdata || mem_do_wdata)) begin
            if (mem_wordsize == 0 && reg_op1[1:0] != 0) begin
                      cpu_state <= cpu_state_trap;
            end
            if (mem_wordsize == 1 && reg_op1[0] != 0) begin
                      cpu_state <= cpu_state_trap;
            end
        end
        if (resetn && mem_do_rinst && reg_pc[0]) begin
                  cpu_state <= cpu_state_trap;
        end

        if (!resetn || mem_done) begin
            mem_do_prefetch <= 0;
            mem_do_rinst <= 0;
            mem_do_rdata <= 0;
            mem_do_wdata <= 0;
        end

        if (set_mem_do_rinst)
            mem_do_rinst <= 1;
        if (set_mem_do_rdata)
            mem_do_rdata <= 1;
        if (set_mem_do_wdata)
            mem_do_wdata <= 1;

        current_pc = 'bx;
    end
endmodule

// This is a simple example implementation of PICORV32_REGS.
// Use the PICORV32_REGS mechanism if you want to use custom
// memory resources to implement the processor register file.
// Note that your implementation must match the requirements of
// the PicoRV32 configuration. (e.g. QREGS, etc)
module picorv32_regs (
        input clk, wen,
        input [5:0] waddr,
        input [5:0] raddr1,
        input [5:0] raddr2,
        input [31:0] wdata,
        output [31:0] rdata1,
        output [31:0] rdata2
    );
    reg [31:0] regs [0:30];

    always @(posedge clk)
        if (wen) regs[~waddr[4:0]] <= wdata;

    assign rdata1 = regs[~raddr1[4:0]];
    assign rdata2 = regs[~raddr2[4:0]];
endmodule


/***************************************************************
 * picorv32_pcpi_mul
 ***************************************************************/

module picorv32_pcpi_mul #(
        parameter STEPS_AT_ONCE = 1,
        parameter CARRY_CHAIN = 4
    ) (
        input clk, resetn,

        input             pcpi_valid,
        input      [31:0] pcpi_insn,
        input      [31:0] pcpi_rs1,
        input      [31:0] pcpi_rs2,
        output reg        pcpi_wr,
        output reg [31:0] pcpi_rd,
        output reg        pcpi_wait,
        output reg        pcpi_ready
    );
    reg instr_mul, instr_mulh, instr_mulhsu, instr_mulhu;
    wire instr_any_mul = |{instr_mul, instr_mulh, instr_mulhsu, instr_mulhu};
    wire instr_any_mulh = |{instr_mulh, instr_mulhsu, instr_mulhu};
    wire instr_rs1_signed = |{instr_mulh, instr_mulhsu};
    wire instr_rs2_signed = |{instr_mulh};

    reg pcpi_wait_q;
    wire mul_start = pcpi_wait && !pcpi_wait_q;

    always @(posedge clk) begin
        instr_mul <= 0;
        instr_mulh <= 0;
        instr_mulhsu <= 0;
        instr_mulhu <= 0;

        if (resetn && pcpi_valid && pcpi_insn[6:0] == 7'b0110011 && pcpi_insn[31:25] == 7'b0000001) begin
            case (pcpi_insn[14:12])
                3'b000: instr_mul <= 1;
                3'b001: instr_mulh <= 1;
                3'b010: instr_mulhsu <= 1;
                3'b011: instr_mulhu <= 1;
            endcase
        end

        pcpi_wait <= instr_any_mul;
        pcpi_wait_q <= pcpi_wait;
    end

    reg [63:0] rs1, rs2, rd, rdx;
    reg [63:0] next_rs1, next_rs2, this_rs2;
    reg [63:0] next_rd, next_rdx, next_rdt;
    reg [6:0] mul_counter;
    reg mul_waiting;
    reg mul_finish;
    integer i, j;

    // carry save accumulator
    always @* begin
        next_rd = rd;
        next_rdx = rdx;
        next_rs1 = rs1;
        next_rs2 = rs2;

        for (i = 0; i < STEPS_AT_ONCE; i=i+1) begin
            this_rs2 = next_rs1[0] ? next_rs2 : 0;
            if (CARRY_CHAIN == 0) begin
                next_rdt = next_rd ^ next_rdx ^ this_rs2;
                next_rdx = ((next_rd & next_rdx) | (next_rd & this_rs2) | (next_rdx & this_rs2)) << 1;
                next_rd = next_rdt;
            end else begin
                next_rdt = 0;
                for (j = 0; j < 64; j = j + CARRY_CHAIN)
                    {next_rdt[j+CARRY_CHAIN-1], next_rd[j +: CARRY_CHAIN]} =
                    next_rd[j +: CARRY_CHAIN] + next_rdx[j +: CARRY_CHAIN] + this_rs2[j +: CARRY_CHAIN];
                next_rdx = next_rdt << 1;
            end
            next_rs1 = next_rs1 >> 1;
            next_rs2 = next_rs2 << 1;
        end
    end

    always @(posedge clk) begin
        mul_finish <= 0;
        if (!resetn) begin
            mul_waiting <= 1;
        end else
            if (mul_waiting) begin
                if (instr_rs1_signed)
                    rs1 <= $signed(pcpi_rs1);
                else
                    rs1 <= $unsigned(pcpi_rs1);

                if (instr_rs2_signed)
                    rs2 <= $signed(pcpi_rs2);
                else
                    rs2 <= $unsigned(pcpi_rs2);

                rd <= 0;
                rdx <= 0;
                mul_counter <= (instr_any_mulh ? 63 - STEPS_AT_ONCE : 31 - STEPS_AT_ONCE);
                mul_waiting <= !mul_start;
            end else begin
                rd <= next_rd;
                rdx <= next_rdx;
                rs1 <= next_rs1;
                rs2 <= next_rs2;

                mul_counter <= mul_counter - STEPS_AT_ONCE;
                if (mul_counter[6]) begin
                    mul_finish <= 1;
                    mul_waiting <= 1;
                end
            end
    end

    always @(posedge clk) begin
        pcpi_wr <= 0;
        pcpi_ready <= 0;
        if (mul_finish && resetn) begin
            pcpi_wr <= 1;
            pcpi_ready <= 1;
            pcpi_rd <= instr_any_mulh ? rd >> 32 : rd;
        end
    end
endmodule

module picorv32_pcpi_fast_mul #(
        parameter EXTRA_MUL_FFS = 0,
        parameter EXTRA_INSN_FFS = 0,
        parameter MUL_CLKGATE = 0
    ) (
        input clk, resetn,

        input             pcpi_valid,
        input      [31:0] pcpi_insn,
        input      [31:0] pcpi_rs1,
        input      [31:0] pcpi_rs2,
        output            pcpi_wr,
        output     [31:0] pcpi_rd,
        output            pcpi_wait,
        output            pcpi_ready
    );
    reg instr_mul, instr_mulh, instr_mulhsu, instr_mulhu;
    wire instr_any_mul = |{instr_mul, instr_mulh, instr_mulhsu, instr_mulhu};
    wire instr_any_mulh = |{instr_mulh, instr_mulhsu, instr_mulhu};
    wire instr_rs1_signed = |{instr_mulh, instr_mulhsu};
    wire instr_rs2_signed = |{instr_mulh};

    reg shift_out;
    reg [3:0] active;
    reg [32:0] rs1, rs2, rs1_q, rs2_q;
    reg [63:0] rd, rd_q;

    wire pcpi_insn_valid = pcpi_valid && pcpi_insn[6:0] == 7'b0110011 && pcpi_insn[31:25] == 7'b0000001;
    reg pcpi_insn_valid_q;

    always @* begin
        instr_mul = 0;
        instr_mulh = 0;
        instr_mulhsu = 0;
        instr_mulhu = 0;

        if (resetn && (EXTRA_INSN_FFS ? pcpi_insn_valid_q : pcpi_insn_valid)) begin
            case (pcpi_insn[14:12])
                3'b000: instr_mul = 1;
                3'b001: instr_mulh = 1;
                3'b010: instr_mulhsu = 1;
                3'b011: instr_mulhu = 1;
            endcase
        end
    end

    always @(posedge clk) begin
        pcpi_insn_valid_q <= pcpi_insn_valid;
        if (!MUL_CLKGATE || active[0]) begin
            rs1_q <= rs1;
            rs2_q <= rs2;
        end
        if (!MUL_CLKGATE || active[1]) begin
            rd <= $signed(EXTRA_MUL_FFS ? rs1_q : rs1) * $signed(EXTRA_MUL_FFS ? rs2_q : rs2);
        end
        if (!MUL_CLKGATE || active[2]) begin
            rd_q <= rd;
        end
    end

    always @(posedge clk) begin
        if (instr_any_mul && !(EXTRA_MUL_FFS ? active[3:0] : active[1:0])) begin
            if (instr_rs1_signed)
                rs1 <= $signed(pcpi_rs1);
            else
                rs1 <= $unsigned(pcpi_rs1);

            if (instr_rs2_signed)
                rs2 <= $signed(pcpi_rs2);
            else
                rs2 <= $unsigned(pcpi_rs2);
            active[0] <= 1;
        end else begin
            active[0] <= 0;
        end

        active[3:1] <= active;
        shift_out <= instr_any_mulh;

        if (!resetn)
            active <= 0;
    end

    assign pcpi_wr = active[EXTRA_MUL_FFS ? 3 : 1];
    assign pcpi_wait = 0;
    assign pcpi_ready = active[EXTRA_MUL_FFS ? 3 : 1];
    assign pcpi_rd = shift_out ? (EXTRA_MUL_FFS ? rd_q : rd) >> 32 : (EXTRA_MUL_FFS ? rd_q : rd);
endmodule


/***************************************************************
 * picorv32_pcpi_div
 ***************************************************************/

module picorv32_pcpi_div (
        input clk, resetn,

        input             pcpi_valid,
        input      [31:0] pcpi_insn,
        input      [31:0] pcpi_rs1,
        input      [31:0] pcpi_rs2,
        output reg        pcpi_wr,
        output reg [31:0] pcpi_rd,
        output reg        pcpi_wait,
        output reg        pcpi_ready
    );
    reg instr_div, instr_divu, instr_rem, instr_remu;
    wire instr_any_div_rem = |{instr_div, instr_divu, instr_rem, instr_remu};

    reg pcpi_wait_q;
    wire start = pcpi_wait && !pcpi_wait_q;

    always @(posedge clk) begin
        instr_div <= 0;
        instr_divu <= 0;
        instr_rem <= 0;
        instr_remu <= 0;

        if (resetn && pcpi_valid && !pcpi_ready && pcpi_insn[6:0] == 7'b0110011 && pcpi_insn[31:25] == 7'b0000001) begin
            case (pcpi_insn[14:12])
                3'b100: instr_div <= 1;
                3'b101: instr_divu <= 1;
                3'b110: instr_rem <= 1;
                3'b111: instr_remu <= 1;
            endcase
        end

        pcpi_wait <= instr_any_div_rem && resetn;
        pcpi_wait_q <= pcpi_wait && resetn;
    end

    reg [31:0] dividend;
    reg [62:0] divisor;
    reg [31:0] quotient;
    reg [31:0] quotient_msk;
    reg running;
    reg outsign;

    always @(posedge clk) begin
        pcpi_ready <= 0;
        pcpi_wr <= 0;
        pcpi_rd <= 'bx;

        if (!resetn) begin
            running <= 0;
        end else
            if (start) begin
                running <= 1;
                dividend <= (instr_div || instr_rem) && pcpi_rs1[31] ? -pcpi_rs1 : pcpi_rs1;
                divisor <= ((instr_div || instr_rem) && pcpi_rs2[31] ? -pcpi_rs2 : pcpi_rs2) << 31;
                outsign <= (instr_div && (pcpi_rs1[31] != pcpi_rs2[31]) && |pcpi_rs2) || (instr_rem && pcpi_rs1[31]);
                quotient <= 0;
                quotient_msk <= 1 << 31;
            end else
                if (!quotient_msk && running) begin
                    running <= 0;
                    pcpi_ready <= 1;
                    pcpi_wr <= 1;
                    if (instr_div || instr_divu)
                        pcpi_rd <= outsign ? -quotient : quotient;
                    else
                        pcpi_rd <= outsign ? -dividend : dividend;
                end else begin
                    if (divisor <= dividend) begin
                        dividend <= dividend - divisor;
                        quotient <= quotient | quotient_msk;
                    end
                    divisor <= divisor >> 1;
                    quotient_msk <= quotient_msk >> 1;
                end
    end
endmodule
