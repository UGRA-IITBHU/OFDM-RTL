`timescale 1ns/1ps
module ofdm_frame_streamer_pp #(
    parameter NUM_SYMBOLS = 80,
    parameter IN_W        = 16,
    parameter OUT_W       = 8,
    parameter QUANT_MODE  = 0,
    parameter OUT_FORMAT  = 0
) (
    input  wire                           clk,
    input  wire                           reset,

    input  wire [NUM_SYMBOLS*2*IN_W-1:0]  data_in,
    input  wire                           data_valid,
    output wire                           data_ready,

    output wire [OUT_W-1:0]               i_out,
    output wire [OUT_W-1:0]               q_out,
    output wire                           sample_valid,
    input  wire                           sample_ready,
    output wire                           frame_start,
    output wire                           frame_last,

    output wire                           done,
    output wire                           busy,
    output wire                           underrun,
    output wire [1:0]                     bank_full
);

    localparam SYM_W   = 2 * IN_W;
    localparam CNT_W   = (NUM_SYMBOLS > 1) ? $clog2(NUM_SYMBOLS) : 1;
    localparam DROP_W  = IN_W - OUT_W;
    localparam RND_SH  = (DROP_W > 0) ? (DROP_W - 1) : 0;

    localparam [CNT_W-1:0] CNT_ZERO = {CNT_W{1'b0}};
    localparam [CNT_W-1:0] CNT_ONE  = 1;
    localparam [CNT_W-1:0] CNT_LAST = NUM_SYMBOLS - 1;
    localparam [IN_W:0]    RND_HALF = {{IN_W{1'b0}}, 1'b1} << RND_SH;
    localparam [OUT_W-1:0] MSB_MASK = (OUT_FORMAT != 0) ? ({{(OUT_W-1){1'b0}}, 1'b1} << (OUT_W - 1))
                                                        : {OUT_W{1'b0}};

    reg [1:0]       full;
    reg             wr_ptr;
    reg             rd_ptr;
    reg [CNT_W-1:0] cnt;
    reg             done_r;
    reg             underrun_r;

    wire            wr_accept   = data_valid && data_ready;
    wire            rd_accept   = sample_valid && sample_ready;
    wire            last_accept = rd_accept && (cnt == CNT_LAST);
    wire            other_full  = rd_ptr ? full[0] : full[1];

    assign data_ready   = ~full[wr_ptr];
    assign sample_valid =  full[rd_ptr];

    always @(posedge clk) begin
        if (reset) begin
            full       <= 2'b00;
            wr_ptr     <= 1'b0;
            rd_ptr     <= 1'b0;
            cnt        <= CNT_ZERO;
            done_r     <= 1'b0;
            underrun_r <= 1'b0;
        end else begin
            done_r     <= last_accept;
            underrun_r <= last_accept && !other_full && !wr_accept;

            if (wr_accept) begin
                full[wr_ptr] <= 1'b1;
                wr_ptr       <= ~wr_ptr;
            end

            if (last_accept) begin
                full[rd_ptr] <= 1'b0;
                rd_ptr       <= ~rd_ptr;
                cnt          <= CNT_ZERO;
            end else if (rd_accept) begin
                cnt <= cnt + CNT_ONE;
            end
        end
    end

    wire [SYM_W-1:0] rdata0, rdata1;

    ofdm_frame_bank #(.NUM_SYMBOLS(NUM_SYMBOLS), .SYM_W(SYM_W), .CNT_W(CNT_W)) bank0 (
        .clk(clk), .we(wr_accept & ~wr_ptr), .wdata(data_in),
        .rd_idx(cnt), .rdata(rdata0)
    );
    ofdm_frame_bank #(.NUM_SYMBOLS(NUM_SYMBOLS), .SYM_W(SYM_W), .CNT_W(CNT_W)) bank1 (
        .clk(clk), .we(wr_accept &  wr_ptr), .wdata(data_in),
        .rd_idx(cnt), .rdata(rdata1)
    );

    function [OUT_W-1:0] quantize;
        input [IN_W-1:0] x;
        reg   [IN_W:0]   sum;
        begin
            if ((QUANT_MODE == 0) || (DROP_W == 0)) begin
                quantize = x[IN_W-1 -: OUT_W];
            end else begin
                sum = {x[IN_W-1], x} + RND_HALF;
                if (sum[IN_W] != sum[IN_W-1])
                    quantize = {1'b0, {(OUT_W-1){1'b1}}};
                else
                    quantize = sum[IN_W-1 -: OUT_W];
            end
        end
    endfunction

    wire [SYM_W-1:0] cur_sym = rd_ptr ? rdata1 : rdata0;
    wire [OUT_W-1:0] i_q     = quantize(cur_sym[SYM_W-1 -: IN_W]);
    wire [OUT_W-1:0] q_q     = quantize(cur_sym[IN_W-1:0]);

    assign i_out = (sample_valid ? i_q : {OUT_W{1'b0}}) ^ MSB_MASK;
    assign q_out = (sample_valid ? q_q : {OUT_W{1'b0}}) ^ MSB_MASK;

    assign frame_start = sample_valid && (cnt == CNT_ZERO);
    assign frame_last  = sample_valid && (cnt == CNT_LAST);
    assign done        = done_r;
    assign underrun    = underrun_r;
    assign busy        = |full;
    assign bank_full   = full;

endmodule

/* verilator lint_off DECLFILENAME */
module ofdm_frame_bank #(
    parameter NUM_SYMBOLS = 80,
    parameter SYM_W       = 32,
    parameter CNT_W       = (NUM_SYMBOLS > 1) ? $clog2(NUM_SYMBOLS) : 1
) (
    input  wire                           clk,
    input  wire                           we,
    input  wire [NUM_SYMBOLS*SYM_W-1:0]   wdata,
    input  wire [CNT_W-1:0]               rd_idx,
    output wire [SYM_W-1:0]               rdata
);

    localparam FRAME_W = NUM_SYMBOLS * SYM_W;
    localparam BASE_W  = $clog2(FRAME_W);
    localparam [31:0]       SYM_W32  = SYM_W;
    localparam [BASE_W-1:0] SYM_STEP = SYM_W32[BASE_W-1:0];

    reg [FRAME_W-1:0] mem;

    always @(posedge clk) begin
        if (we) mem <= wdata;
    end

    wire [BASE_W-1:0] base = {{(BASE_W-CNT_W){1'b0}}, rd_idx} * SYM_STEP;
    assign rdata = mem[base +: SYM_W];

endmodule
/* verilator lint_on DECLFILENAME */
