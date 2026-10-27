`timescale 1ns / 1ps

// SPI slave (mode 0: CPOL = 0, CPHA = 0) with the ASIC top's 8-bit register file.
//
// Transaction: pull CS_n low, send one command byte {rw, addr[6:0]}
// (rw = 1 read, rw = 0 write), then any number of data bytes. The address
// auto-increments after every data byte, so the 16 QPSK bytes can be written
// or read back in one burst. MSB first on both MOSI and MISO.
//
// Register map:
//   0x00  CTRL    RW  bit0 START  (write 1 to start a frame, self-clearing, reads 0)
//                     bit1 REPEAT (1: replay the stored frame back-to-back)
//   0x01  STATUS  RO  bit0 gen_busy, bit1 tx_active, bit2 frame_valid
//   0x02  QPSK0   RW  qpsk_bits[7:0]
//   ...
//   0x11  QPSK15  RW  qpsk_bits[127:120]
//   0x12  ID      RO  0xDA
//   0x13  GAIN    RW  bits[2:0]: DAC output gain = 2^GAIN, saturating
//
// SCLK, CS_n and MOSI are oversampled by clk through two-flop synchronisers,
// so f_sclk must be at most f_clk / 8. MISO changes a few clk cycles after each
// SCLK rising edge, which leaves the master a full SCLK period before it
// samples on the next rising edge.
module spi_slave_regs (
    input  wire         clk,
    input  wire         rst,
    input  wire         spi_sclk,
    input  wire         spi_cs_n,
    input  wire         spi_mosi,
    output wire         spi_miso,
    output wire         spi_miso_oe,   // pad output enable: high while selected
    output reg  [127:0] qpsk_bits,
    output reg          repeat_en,
    output reg  [2:0]   gain,
    output reg          start,         // one-clk pulse
    input  wire         gen_busy,
    input  wire         tx_active,
    input  wire         frame_valid
);

    localparam [6:0] A_CTRL   = 7'h00;
    localparam [6:0] A_STATUS = 7'h01;
    localparam [6:0] A_QPSK0  = 7'h02;
    localparam [6:0] A_QPSK15 = 7'h11;
    localparam [6:0] A_ID     = 7'h12;
    localparam [6:0] A_GAIN   = 7'h13;
    localparam [7:0] ID       = 8'hDA;

    // ---- synchronisers -------------------------------------------------
    reg [2:0] sclk_q;
    reg [1:0] cs_n_q;
    reg [1:0] mosi_q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sclk_q <= 3'b000;
            cs_n_q <= 2'b11;
            mosi_q <= 2'b00;
        end else begin
            sclk_q <= {sclk_q[1:0], spi_sclk};
            cs_n_q <= {cs_n_q[0], spi_cs_n};
            mosi_q <= {mosi_q[0], spi_mosi};
        end
    end

    // mosi_q[1] has the same two-flop delay as sclk_q[1], so it is the value
    // MOSI held at the SCLK rising edge.
    wire sclk_rise = (sclk_q[2:1] == 2'b01);
    wire selected  = ~cs_n_q[1];
    wire mosi_s    = mosi_q[1];

    // ---- shift logic ---------------------------------------------------
    reg [2:0] bit_cnt;
    reg [6:0] rx_shift;    // first seven bits of the byte in flight
    reg [7:0] tx_shift;
    reg       cmd_phase;   // 1 while the command byte is being received
    reg       rd_mode;
    reg [6:0] addr;

    wire [7:0] rx_byte = {rx_shift, mosi_s};

    // Read data for the byte about to be shifted out: the addressed register
    // right after the command byte, the next register after each data byte.
    wire [6:0] rd_addr = cmd_phase ? rx_byte[6:0] : addr + 7'd1;
    // addr[3:0] - 2 (mod 16) equals addr - 2 across the whole 0x02..0x11 range.
    wire [3:0] rd_qbyte = rd_addr[3:0] - 4'd2;
    wire [3:0] wr_qbyte = addr[3:0]    - 4'd2;

    reg [7:0] rd_data;
    always @(*) begin
        if (rd_addr >= A_QPSK0 && rd_addr <= A_QPSK15)
            rd_data = qpsk_bits[{rd_qbyte, 3'b000} +: 8];
        else case (rd_addr)
            A_CTRL:   rd_data = {6'd0, repeat_en, 1'b0};
            A_STATUS: rd_data = {5'd0, frame_valid, tx_active, gen_busy};
            A_ID:     rd_data = ID;
            A_GAIN:   rd_data = {5'd0, gain};
            default:  rd_data = 8'h00;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_cnt   <= 3'd0;
            rx_shift  <= 7'd0;
            tx_shift  <= 8'd0;
            cmd_phase <= 1'b1;
            rd_mode   <= 1'b0;
            addr      <= 7'd0;
            qpsk_bits <= 128'd0;
            repeat_en <= 1'b0;
            gain      <= 3'd0;
            start     <= 1'b0;
        end else begin
            start <= 1'b0;

            if (!selected) begin
                bit_cnt   <= 3'd0;
                cmd_phase <= 1'b1;
                tx_shift  <= 8'd0;
            end else if (sclk_rise) begin
                bit_cnt  <= bit_cnt + 3'd1;          // wraps 7 -> 0 at each byte
                rx_shift <= rx_byte[6:0];
                tx_shift <= {tx_shift[6:0], 1'b0};

                if (bit_cnt == 3'd7) begin
                    if (cmd_phase) begin
                        cmd_phase <= 1'b0;
                        rd_mode   <= rx_byte[7];
                        addr      <= rx_byte[6:0];
                    end else begin
                        if (!rd_mode) begin
                            if (addr >= A_QPSK0 && addr <= A_QPSK15)
                                qpsk_bits[{wr_qbyte, 3'b000} +: 8] <= rx_byte;
                            else if (addr == A_CTRL) begin
                                start     <= rx_byte[0];
                                repeat_en <= rx_byte[1];
                            end else if (addr == A_GAIN)
                                gain <= rx_byte[2:0];
                        end
                        addr <= addr + 7'd1;
                    end
                    tx_shift <= rd_data;
                end
            end
        end
    end

    assign spi_miso    = tx_shift[7];
    assign spi_miso_oe = selected;

endmodule
