`timescale 1ns / 1ps

// ASIC top: SPI in, I/Q DAC out.
//
//   SPI ──► spi_slave_regs ──► qpsk_bits, START, REPEAT, GAIN
//                                   │
//   zc / qpsk / map_to_subcarriers ─┴► parallel_to_serial ► ifft_64
//                                                              │ 64 samples/symbol
//                                                              ▼
//                                          frame_mem (4 x 64 x 32-bit, ~8 Kbit SRAM)
//                                                              │ CP added on read
//                                                              ▼
//                                              dac_i / dac_q / dac_clk / tx_en
//
// The transmit chain is the same set of verified blocks that full.v uses. The
// difference is the back end: instead of add_cp and a 10,240-bit frame register,
// the IFFT output is written to a 256-word memory and the CP is produced by the
// read address sequence (samples 48..63, then 0..63, of each symbol).
//
// The frame is generated completely before playback starts. The chain needs
// about 267 clk cycles per symbol but a symbol is only 80 DAC samples long, so
// streaming straight from the IFFT would leave gaps in the DAC output. Playing
// from memory gives 320 back-to-back samples at a constant rate.
//
// Operation:
//   1. Write QPSK0..QPSK15 (and GAIN) over SPI.
//   2. Write CTRL.START = 1. The frame is generated (~1,070 clk cycles).
//   3. The 320 samples are played out, one every DAC_DIV clk cycles, with
//      tx_en high. frame_sync is high during sample 0.
//   4. tx_en drops and the DAC is held at zero. With CTRL.REPEAT = 1 the
//      stored frame is replayed back-to-back until REPEAT is cleared; playback
//      then stops at the end of the current frame.
//
// START is accepted when idle or playing (playing aborts and regenerates). It
// is ignored while a frame is being generated, so the FFT always sees whole
// 64-sample symbols.
//
// DAC timing: dac_i/dac_q change on the clk edge where dac_clk falls, and
// dac_clk rises at least one clk cycle later, so the DAC latches on the rising
// edge of dac_clk with >= 1 clk cycle of setup and hold.
module ofdm_asic_top #(
    parameter DAC_W             = 12,  // DAC data bits per rail, 1..16
    parameter DAC_DIV           = 2,   // clk cycles per DAC sample, >= 2
    parameter DAC_OFFSET_BINARY = 0    // 0: two's complement, 1: offset binary
)(
    input  wire             clk,
    input  wire             rst_n,         // async assert, sync release internally

    input  wire             spi_sclk,
    input  wire             spi_cs_n,
    input  wire             spi_mosi,
    output wire             spi_miso,
    output wire             spi_miso_oe,

    output reg  [DAC_W-1:0] dac_i,
    output reg  [DAC_W-1:0] dac_q,
    output reg              dac_clk,
    output reg              tx_en,         // high while frame samples are on the DAC
    output reg              frame_sync     // high during sample 0 of each frame
);

    generate
        if (DAC_DIV < 2 || DAC_W < 1 || DAC_W > 16) begin : bad_param
            // Deliberately undefined module: elaboration fails on bad parameters.
            DAC_DIV_must_be_at_least_2_and_DAC_W_1_to_16 u_bad_param ();
        end
    endgenerate

    localparam SC    = 64;
    localparam GUARD = 10;
    localparam DC    = 32;

    // ---- reset synchroniser --------------------------------------------
    // rst_meta only feeds rst_q's D input; rst_q is the only flop used as an
    // async reset. Keeping them as separate regs (not one 2-bit vector) means
    // no single signal is flopped both synchronously and asynchronously.
    reg rst_meta;
    reg rst_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_meta <= 1'b1;
            rst_q    <= 1'b1;
        end else begin
            rst_meta <= 1'b0;
            rst_q    <= rst_meta;
        end
    end
    wire rst = rst_q;

    // ---- SPI register file ----------------------------------------------
    wire [127:0] qpsk_bits;
    wire         repeat_en;
    wire [2:0]   gain;
    wire         start;

    localparam [1:0] S_IDLE = 2'd0, S_MAP = 2'd1, S_COLLECT = 2'd2, S_PLAY = 2'd3;
    reg [1:0] state;
    reg       frame_valid;

    spi_slave_regs u_spi (
        .clk         (clk),
        .rst         (rst),
        .spi_sclk    (spi_sclk),
        .spi_cs_n    (spi_cs_n),
        .spi_mosi    (spi_mosi),
        .spi_miso    (spi_miso),
        .spi_miso_oe (spi_miso_oe),
        .qpsk_bits   (qpsk_bits),
        .repeat_en   (repeat_en),
        .gain        (gain),
        .start       (start),
        .gen_busy    (state == S_MAP || state == S_COLLECT),
        .tx_active   (state == S_PLAY),
        .frame_valid (frame_valid)
    );

    // ---- transmit chain (same blocks as full.v) --------------------------
    wire [SC*16-1:0] zc_re;
    wire [SC*16-1:0] zc_im;

    zc_sequence_generator #(
        .SUB_COUNT  (SC),
        .GUARD_COUNT(GUARD),
        .DC_INDEX   (DC),
        .DATA_WIDTH (16)
    ) u_zc (
        .zc_seq_re(zc_re),
        .zc_seq_im(zc_im)
    );

    // Snapshot of qpsk_bits taken at START, so SPI writes during generation
    // cannot change the payload half-way through a frame.
    reg  [127:0]     qpsk_work;
    wire [1023:0]    qpsk_i;
    wire [1023:0]    qpsk_q;

    qpsk_signal_128 u_qpsk (
        .clk(clk),
        .in (qpsk_work),
        .I  (qpsk_i),
        .Q  (qpsk_q)
    );

    wire [SC*32-1:0] zc_cplx;
    wire [SC*32-1:0] qpsk_cplx;
    genvar g;
    generate
        for (g = 0; g < SC; g = g + 1) begin : pack
            assign zc_cplx[g*32 +: 32]   = {zc_re[g*16 +: 16], zc_im[g*16 +: 16]};
            assign qpsk_cplx[g*32 +: 32] = {qpsk_i[g*16 +: 16], qpsk_q[g*16 +: 16]};
        end
    endgenerate

    reg  [1:0]       sym_idx;
    wire [SC*32-1:0] subcarriers;

    parameterized_map_to_subcarriers #(
        .SUBCARRIER_COUNT(SC),
        .GUARD_COUNT     (GUARD),
        .DC              (DC),
        .LAST_GUARD_INDEX(GUARD - 1)
    ) u_map (
        .clk        (clk),
        .rst        (rst),
        .sync       (sym_idx == 2'd0),
        .ref_sym    (sym_idx == 2'd1),
        .header     (sym_idx == 2'd2),
        .zc_seq     (zc_cplx),
        .qpsk_symbol(qpsk_cplx),
        .subcarriers(subcarriers)
    );

    reg         sym_start;
    wire        p2s_en;
    wire [15:0] p2s_re;
    wire [15:0] p2s_im;
    /* verilator lint_off UNUSEDSIGNAL */
    wire        p2s_frame_done;   // unused: the IFFT output count ends each symbol
    /* verilator lint_on UNUSEDSIGNAL */

    parallel_to_serial #(.N(SC), .WIDTH(16)) u_p2s (
        .clk          (clk),
        .rst          (rst),
        .start        (sym_start),
        .parallel_data(subcarriers),
        .do_en        (p2s_en),
        .do_re        (p2s_re),
        .do_im        (p2s_im),
        .frame_done   (p2s_frame_done)
    );

    wire        ifft_en;
    wire [15:0] ifft_re;
    wire [15:0] ifft_im;

    ifft_64 #(.WIDTH(16)) u_ifft (
        .clk  (clk),
        .rst  (rst),
        .di_en(p2s_en),
        .di_re(p2s_re),
        .di_im(p2s_im),
        .do_en(ifft_en),
        .do_re(ifft_re),
        .do_im(ifft_im)
    );

    // ---- frame memory ---------------------------------------------------
    // Word {symbol[1:0], sample[5:0]} = {re[15:0], im[15:0]}, natural order.
    // No reset and a registered read port, so it maps onto a 256 x 32 SRAM
    // macro (or a latch array) instead of flip-flops.
    reg  [31:0] frame_mem [0:255];
    reg  [5:0]  wr_cnt;
    wire        mem_we = (state == S_COLLECT) && ifft_en;

    always @(posedge clk) begin
        if (mem_we) frame_mem[{sym_idx, wr_cnt}] <= {ifft_re, ifft_im};
    end

    // Playback position: rd_s = symbol, rd_n = 0..79 within the symbol.
    // rd_n 0..15 is the CP (memory sample 48..63), 16..79 the body (0..63).
    // For 64..79, rd_n[5:0] - 16 wraps mod 64 to 48..63, which is still right.
    reg  [1:0]  rd_s;
    reg  [6:0]  rd_n;
    wire [5:0]  rd_k = (rd_n < 7'd16) ? rd_n[5:0] + 6'd48 : rd_n[5:0] - 6'd16;
    reg  [31:0] rd_data;

    always @(posedge clk) begin
        rd_data <= frame_mem[{rd_s, rd_k}];
    end

    // ---- DAC sample-rate divider -----------------------------------------
    localparam         DW       = (DAC_DIV > 2) ? $clog2(DAC_DIV) : 1;
    localparam [DW-1:0] LAST_DIV = DAC_DIV[DW-1:0] - 1'b1;
    localparam integer HALF_I   = DAC_DIV / 2;
    localparam [DW-1:0] HALF_DIV = HALF_I[DW-1:0];

    reg  [DW-1:0] div_cnt;
    wire          tick    = (div_cnt == LAST_DIV);   // a new sample goes out
    wire [DW-1:0] div_nxt = tick ? {DW{1'b0}} : div_cnt + 1'b1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            div_cnt <= {DW{1'b0}};
            dac_clk <= 1'b0;
        end else begin
            div_cnt <= div_nxt;
            dac_clk <= (div_nxt >= HALF_DIV);
        end
    end

    // Q1.15 sample -> DAC code: saturating gain of 2^g, keep the top DAC_W
    // bits, optionally flip the MSB for offset binary.
    function [DAC_W-1:0] to_dac(input [15:0] x, input [2:0] gsh);
        reg signed [23:0] w;
        /* verilator lint_off UNUSEDSIGNAL */
        reg        [15:0] s;   // s[15-DAC_W:0] are the LSBs truncated away
        /* verilator lint_on UNUSEDSIGNAL */
        begin
            w = $signed({{8{x[15]}}, x}) <<< gsh;
            if (w > 24'sd32767)       s = 16'h7FFF;
            else if (w < -24'sd32768) s = 16'h8000;
            else                      s = w[15:0];
            to_dac = s[15 -: DAC_W];
            if (DAC_OFFSET_BINARY != 0) to_dac[DAC_W-1] = ~to_dac[DAC_W-1];
        end
    endfunction

    // ---- control FSM ------------------------------------------------------
    reg last_out;   // the final sample of a non-repeating frame is on the DAC

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= S_IDLE;
            sym_idx     <= 2'd0;
            wr_cnt      <= 6'd0;
            sym_start   <= 1'b0;
            frame_valid <= 1'b0;
            qpsk_work   <= 128'd0;
            rd_s        <= 2'd0;
            rd_n        <= 7'd0;
            last_out    <= 1'b0;
            tx_en       <= 1'b0;
            frame_sync  <= 1'b0;
            dac_i       <= to_dac(16'h0000, 3'd0);
            dac_q       <= to_dac(16'h0000, 3'd0);
        end else begin
            sym_start <= 1'b0;

            if (start && (state == S_IDLE || state == S_PLAY)) begin
                qpsk_work   <= qpsk_bits;
                sym_idx     <= 2'd0;
                wr_cnt      <= 6'd0;
                frame_valid <= 1'b0;
                rd_s        <= 2'd0;
                rd_n        <= 7'd0;
                last_out    <= 1'b0;
                tx_en       <= 1'b0;
                frame_sync  <= 1'b0;
                dac_i       <= to_dac(16'h0000, 3'd0);
                dac_q       <= to_dac(16'h0000, 3'd0);
                state       <= S_MAP;
            end else begin
                case (state)
                    // sym_idx changed on the previous edge; the mapper registers
                    // the new symbol on this edge, and p2s samples it on the next
                    // one, when it sees sym_start. Same launch timing as full.v.
                    S_MAP: begin
                        sym_start <= 1'b1;
                        state     <= S_COLLECT;
                    end

                    S_COLLECT: begin
                        if (ifft_en) begin
                            wr_cnt <= wr_cnt + 6'd1;
                            if (wr_cnt == 6'd63) begin
                                if (sym_idx == 2'd3) begin
                                    sym_idx     <= 2'd0;
                                    frame_valid <= 1'b1;
                                    state       <= S_PLAY;
                                end else begin
                                    sym_idx <= sym_idx + 2'd1;
                                    state   <= S_MAP;
                                end
                            end
                        end
                    end

                    // rd_s/rd_n are 0 on entry, and rd_data is reloaded every
                    // clk, so the first sample is already waiting at the first
                    // tick. After each tick the next address is set, and with
                    // DAC_DIV >= 2 its data is ready before the following tick.
                    S_PLAY: begin
                        if (tick) begin
                            if (last_out) begin
                                last_out   <= 1'b0;
                                tx_en      <= 1'b0;
                                frame_sync <= 1'b0;
                                dac_i      <= to_dac(16'h0000, 3'd0);
                                dac_q      <= to_dac(16'h0000, 3'd0);
                                state      <= S_IDLE;
                            end else begin
                                dac_i      <= to_dac(rd_data[31:16], gain);
                                dac_q      <= to_dac(rd_data[15:0],  gain);
                                tx_en      <= 1'b1;
                                frame_sync <= (rd_s == 2'd0) && (rd_n == 7'd0);
                                if (rd_n == 7'd79) begin
                                    rd_n <= 7'd0;
                                    rd_s <= rd_s + 2'd1;          // 3 wraps to 0
                                    if (rd_s == 2'd3 && !repeat_en)
                                        last_out <= 1'b1;
                                end else begin
                                    rd_n <= rd_n + 7'd1;
                                end
                            end
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
