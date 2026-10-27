`timescale 1ns / 1ps

// Self-checking testbench for asic/ofdm_asic_top.v.
//
// The golden frame comes from the existing core: ofdm_generate_frame (full.v)
// is run alongside with the same payload and its first ofdm_frame is captured.
// The ASIC top must reproduce those 320 words on its DAC pins.
//
//   dut0: DAC_W = 16, two's complement, DAC_DIV = 2  -> bit-exact vs reference
//   dut1: DAC_W = 12, offset binary,    DAC_DIV = 3  -> top 12 bits, MSB flipped
//
// Both DUTs share SCLK/CS_n/MOSI, so every SPI command reaches both; only
// dut0's MISO is read.
//
// Checks: SPI ID, QPSK burst write + read-back, STATUS, one frame on both
// DUTs, idle DAC level, GAIN, REPEAT (gapless replay, stop at a frame
// boundary) and START during playback (abort and regenerate).

module tb_ofdm_asic_top;

    localparam [127:0] PAYLOAD = 128'hA5C3_0F1E_9B47_D268_3C5A_E1F0_7788_B2D4;
    localparam         N_WORDS = 320;
    localparam         MAXCAP  = 4096;
    localparam         SPI_HALF = 80;     // ns; f_sclk = 6.25 MHz, f_clk / 16

    localparam [6:0] A_CTRL = 7'h00, A_STATUS = 7'h01, A_QPSK0 = 7'h02,
                     A_ID = 7'h12, A_GAIN = 7'h13;

    integer error_count = 0;

    reg clk = 1'b0;
    always #5 clk = ~clk;                // 100 MHz

    reg rst_n = 1'b0;
    reg sclk  = 1'b0;
    reg cs_n  = 1'b1;
    reg mosi  = 1'b0;

    // ---- DUTs -------------------------------------------------------------
    wire        miso0, oe0, miso1, oe1;
    wire [15:0] d0_i, d0_q;
    wire        d0_clk, d0_en, d0_sync;
    wire [11:0] d1_i, d1_q;
    wire        d1_clk, d1_en, d1_sync;

    ofdm_asic_top #(.DAC_W(16), .DAC_DIV(2), .DAC_OFFSET_BINARY(0)) dut0 (
        .clk(clk), .rst_n(rst_n),
        .spi_sclk(sclk), .spi_cs_n(cs_n), .spi_mosi(mosi),
        .spi_miso(miso0), .spi_miso_oe(oe0),
        .dac_i(d0_i), .dac_q(d0_q), .dac_clk(d0_clk),
        .tx_en(d0_en), .frame_sync(d0_sync)
    );

    ofdm_asic_top #(.DAC_W(12), .DAC_DIV(3), .DAC_OFFSET_BINARY(1)) dut1 (
        .clk(clk), .rst_n(rst_n),
        .spi_sclk(sclk), .spi_cs_n(cs_n), .spi_mosi(mosi),
        .spi_miso(miso1), .spi_miso_oe(oe1),
        .dac_i(d1_i), .dac_q(d1_q), .dac_clk(d1_clk),
        .tx_en(d1_en), .frame_sync(d1_sync)
    );

    // ---- reference core -----------------------------------------------------
    reg              ref_rst = 1'b1;
    wire [10239:0]   ref_frame_w;
    wire             ref_done;
    reg  [10239:0]   ref_frame;

    ofdm_generate_frame u_ref (
        .clk       (clk),
        .rst       (ref_rst),
        .qpsk_bits (PAYLOAD),
        .ofdm_frame(ref_frame_w),
        .frame_done(ref_done)
    );

    function [15:0] ref_re(input integer n);
        ref_re = ref_frame[n*32 + 16 +: 16];
    endfunction

    function [15:0] ref_im(input integer n);
        ref_im = ref_frame[n*32 +: 16];
    endfunction

    function [15:0] sat_gain(input [15:0] x, input integer g);
        integer v;
        begin
            v = $signed(x) * (1 << g);
            if (v >  32767) v =  32767;
            if (v < -32768) v = -32768;
            sat_gain = v[15:0];
        end
    endfunction

    function [11:0] to12ob(input [15:0] x);
        to12ob = {~x[15], x[14:4]};
    endfunction

    // ---- SPI master (mode 0) -----------------------------------------------
    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b];
                #SPI_HALF sclk = 1'b1;
                rx[b] = miso0;
                #SPI_HALF sclk = 1'b0;
            end
        end
    endtask

    task spi_begin;
        begin cs_n = 1'b0; #SPI_HALF; end
    endtask

    task spi_end;
        begin #SPI_HALF cs_n = 1'b1; #(2*SPI_HALF); end
    endtask

    task spi_write(input [6:0] a, input [7:0] d);
        reg [7:0] dummy;
        begin
            spi_begin;
            spi_byte({1'b0, a}, dummy);
            spi_byte(d, dummy);
            spi_end;
        end
    endtask

    task spi_read(input [6:0] a, output [7:0] d);
        reg [7:0] dummy;
        begin
            spi_begin;
            spi_byte({1'b1, a}, dummy);
            spi_byte(8'h00, d);
            spi_end;
        end
    endtask

    task check8(input [255:0] what, input [7:0] got, input [7:0] exp);
        begin
            if (got === exp) $display("[PASS] %0s = 0x%02h", what, got);
            else begin
                $display("[FAIL] %0s = 0x%02h, expected 0x%02h", what, got, exp);
                error_count = error_count + 1;
            end
        end
    endtask

    // ---- DAC capture ---------------------------------------------------------
    // Samples are taken on the rising edge of dac_clk, as a real DAC would.
    reg [15:0] cap0_i [0:MAXCAP-1];
    reg [15:0] cap0_q [0:MAXCAP-1];
    reg        cap0_s [0:MAXCAP-1];
    reg [11:0] cap1_i [0:MAXCAP-1];
    reg [11:0] cap1_q [0:MAXCAP-1];
    reg        cap1_s [0:MAXCAP-1];
    integer    cnt0, cnt1;

    task collect0;
        reg going;
        begin
            cnt0  = 0;
            going = 1'b1;
            wait (d0_en === 1'b1);
            while (going) begin
                @(posedge d0_clk);
                if (d0_en === 1'b1 && cnt0 < MAXCAP) begin
                    cap0_i[cnt0] = d0_i;
                    cap0_q[cnt0] = d0_q;
                    cap0_s[cnt0] = d0_sync;
                    cnt0 = cnt0 + 1;
                end else begin
                    going = 1'b0;
                end
            end
        end
    endtask

    task collect1;
        reg going;
        begin
            cnt1  = 0;
            going = 1'b1;
            wait (d1_en === 1'b1);
            while (going) begin
                @(posedge d1_clk);
                if (d1_en === 1'b1 && cnt1 < MAXCAP) begin
                    cap1_i[cnt1] = d1_i;
                    cap1_q[cnt1] = d1_q;
                    cap1_s[cnt1] = d1_sync;
                    cnt1 = cnt1 + 1;
                end else begin
                    going = 1'b0;
                end
            end
        end
    endtask

    // Compare dut0's capture against the reference, frame after frame.
    task check0(input [255:0] what, input integer exp_cnt, input integer g);
        integer n, w, bad;
        begin
            bad = 0;
            if (cnt0 !== exp_cnt) begin
                $display("[FAIL] %0s: %0d samples, expected %0d", what, cnt0, exp_cnt);
                bad = bad + 1;
            end
            for (n = 0; n < cnt0; n = n + 1) begin
                w = n % N_WORDS;
                if (cap0_i[n] !== sat_gain(ref_re(w), g) ||
                    cap0_q[n] !== sat_gain(ref_im(w), g) ||
                    cap0_s[n] !== (w == 0)) begin
                    if (bad < 5)
                        $display("[FAIL] %0s: sample %0d = %h/%h sync %b, expected %h/%h sync %b",
                                 what, n, cap0_i[n], cap0_q[n], cap0_s[n],
                                 sat_gain(ref_re(w), g), sat_gain(ref_im(w), g), (w == 0));
                    bad = bad + 1;
                end
            end
            if (bad == 0) $display("[PASS] %0s: %0d samples bit-exact", what, cnt0);
            error_count = error_count + bad;
        end
    endtask

    // ---- test sequence ---------------------------------------------------------
    reg [7:0] rd;
    reg [7:0] dummy;
    integer   b, n, bad, aborted_at;

    initial begin
        $display("=== tb_ofdm_asic_top ===");

        #100;
        rst_n   = 1'b1;
        ref_rst = 1'b0;

        // Reference frame
        @(posedge ref_done);
        @(negedge clk);
        ref_frame = ref_frame_w;
        $display("Reference frame captured at %0t ps", $time);

        // 1. ID and idle STATUS
        spi_read(A_ID, rd);      check8("ID", rd, 8'hDA);
        spi_read(A_STATUS, rd);  check8("STATUS after reset", rd, 8'h00);

        // 2. QPSK burst write, burst read back
        spi_begin;
        spi_byte({1'b0, A_QPSK0}, dummy);
        for (b = 0; b < 16; b = b + 1) spi_byte(PAYLOAD[b*8 +: 8], dummy);
        spi_end;

        bad = 0;
        spi_begin;
        spi_byte({1'b1, A_QPSK0}, dummy);
        for (b = 0; b < 16; b = b + 1) begin
            spi_byte(8'h00, rd);
            if (rd !== PAYLOAD[b*8 +: 8]) begin
                $display("[FAIL] QPSK%0d read back 0x%02h, expected 0x%02h", b, rd, PAYLOAD[b*8 +: 8]);
                bad = bad + 1;
            end
        end
        spi_end;
        if (bad == 0) $display("[PASS] QPSK burst write / read back");
        error_count = error_count + bad;
        if (dut0.qpsk_bits !== PAYLOAD) begin
            $display("[FAIL] dut0 qpsk_bits = %h", dut0.qpsk_bits);
            error_count = error_count + 1;
        end

        spi_write(A_GAIN, 8'd0);
        spi_read(A_GAIN, rd);    check8("GAIN", rd, 8'h00);

        // 3. One frame on both DUTs
        spi_write(A_CTRL, 8'h01);
        fork
            collect0;
            collect1;
        join
        check0("dut0 single frame", N_WORDS, 0);

        bad = 0;
        if (cnt1 !== N_WORDS) begin
            $display("[FAIL] dut1 single frame: %0d samples, expected %0d", cnt1, N_WORDS);
            bad = bad + 1;
        end
        for (n = 0; n < cnt1; n = n + 1) begin
            if (cap1_i[n] !== to12ob(ref_re(n)) || cap1_q[n] !== to12ob(ref_im(n)) ||
                cap1_s[n] !== (n == 0)) begin
                if (bad < 5)
                    $display("[FAIL] dut1 sample %0d = %h/%h, expected %h/%h",
                             n, cap1_i[n], cap1_q[n], to12ob(ref_re(n)), to12ob(ref_im(n)));
                bad = bad + 1;
            end
        end
        if (bad == 0) $display("[PASS] dut1 single frame: %0d samples, 12-bit offset binary", cnt1);
        error_count = error_count + bad;

        // 4. Idle level and STATUS
        @(negedge clk);
        if (d0_i === 16'h0000 && d0_q === 16'h0000 && d1_i === 12'h800 && d1_q === 12'h800 &&
            d0_en === 1'b0 && d1_en === 1'b0)
            $display("[PASS] idle DAC level (0x0000 two's complement, 0x800 offset binary)");
        else begin
            $display("[FAIL] idle DAC level: d0 %h/%h en %b, d1 %h/%h en %b",
                     d0_i, d0_q, d0_en, d1_i, d1_q, d1_en);
            error_count = error_count + 1;
        end
        spi_read(A_STATUS, rd);  check8("STATUS after frame", rd, 8'h04);

        // 5. GAIN = 2 (x4, saturating)
        spi_write(A_GAIN, 8'd2);
        spi_write(A_CTRL, 8'h01);
        fork
            collect0;
            collect1;
        join
        check0("dut0 GAIN=2", N_WORDS, 2);
        spi_write(A_GAIN, 8'd0);

        // 6. REPEAT: gapless replay, stops at a frame boundary after REPEAT clears
        spi_write(A_CTRL, 8'h03);
        cnt0 = 0;    // the waiter below reads cnt0 before collect0 clears it
        fork
            collect0;
            begin
                wait (cnt0 >= 2*N_WORDS + 40);
                spi_write(A_CTRL, 8'h00);
            end
            collect1;
        join
        if (cnt0 % N_WORDS == 0 && cnt0 >= 3*N_WORDS)
            $display("[PASS] REPEAT: %0d samples = %0d whole frames, no gaps", cnt0, cnt0 / N_WORDS);
        else begin
            $display("[FAIL] REPEAT: %0d samples, not a whole number of >= 3 frames", cnt0);
            error_count = error_count + 1;
        end
        check0("dut0 REPEAT stream", cnt0, 0);

        // 7. START during playback aborts and regenerates
        spi_write(A_CTRL, 8'h01);
        cnt0 = 0;
        fork
            collect0;
            begin
                wait (cnt0 >= 50);
                spi_write(A_CTRL, 8'h01);
            end
            collect1;
        join
        aborted_at = cnt0;
        if (aborted_at > 50 && aborted_at < N_WORDS)
            $display("[PASS] START during playback aborted after %0d samples", aborted_at);
        else begin
            $display("[FAIL] START during playback: %0d samples before stop", aborted_at);
            error_count = error_count + 1;
        end
        fork
            collect0;
            collect1;
        join
        check0("dut0 frame after abort", N_WORDS, 0);

        $display("");
        if (error_count == 0) $display("ALL OFDM ASIC TOP TESTS PASSED");
        else                  $display("OFDM ASIC TOP TESTS FAILED: %0d error(s)", error_count);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("[FAIL] TIMEOUT");
        $display("OFDM ASIC TOP TESTS FAILED: timeout");
        $finish;
    end

endmodule
