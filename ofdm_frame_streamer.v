`timescale 1ns/1ps
module ofdm_frame_streamer #(
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
    output wire                           busy
);

    localparam SYM_W   = 2 * IN_W;
    localparam FRAME_W = NUM_SYMBOLS * SYM_W;
    localparam CNT_W   = (NUM_SYMBOLS > 1) ? $clog2(NUM_SYMBOLS) : 1;
    localparam BASE_W  = $clog2(FRAME_W);
    localparam DROP_W  = IN_W - OUT_W;
    localparam RND_SH  = (DROP_W > 0) ? (DROP_W - 1) : 0;

    localparam [CNT_W-1:0]  CNT_ZERO = {CNT_W{1'b0}};
    localparam [CNT_W-1:0]  CNT_ONE  = 1;
    localparam [CNT_W-1:0]  CNT_LAST = NUM_SYMBOLS - 1;
    localparam [BASE_W-1:0] SYM_STEP = SYM_W;
    localparam [IN_W:0]     RND_HALF = {{IN_W{1'b0}}, 1'b1} << RND_SH;
    localparam [OUT_W-1:0]  MSB_MASK = (OUT_FORMAT != 0) ? ({{(OUT_W-1){1'b0}}, 1'b1} << (OUT_W - 1))
                                                         : {OUT_W{1'b0}};

    localparam [1:0] S_IDLE     = 2'd0;
    localparam [1:0] S_LOAD     = 2'd1;
    localparam [1:0] S_TRANSMIT = 2'd2;
    localparam [1:0] S_DONE     = 2'd3;

    reg [1:0]         state;
    reg [CNT_W-1:0]   cnt;
    reg [FRAME_W-1:0] frame_buf;

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

    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE;
            cnt   <= CNT_ZERO;
        end else begin
            case (state)
                S_IDLE: begin
                    if (data_valid) begin
                        frame_buf <= data_in;
                        cnt       <= CNT_ZERO;
                        state     <= S_LOAD;
                    end
                end
                S_LOAD: begin
                    state <= S_TRANSMIT;
                end
                S_TRANSMIT: begin
                    if (sample_ready) begin
                        if (cnt == CNT_LAST) begin
                            cnt   <= CNT_ZERO;
                            state <= S_DONE;
                        end else begin
                            cnt <= cnt + CNT_ONE;
                        end
                    end
                end
                S_DONE: begin
                    state <= S_IDLE;
                end
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

    wire [BASE_W-1:0] sym_base = {{(BASE_W-CNT_W){1'b0}}, cnt} * SYM_STEP;
    wire [SYM_W-1:0]  cur_sym  = frame_buf[sym_base +: SYM_W];

    wire [OUT_W-1:0]  i_q = quantize(cur_sym[SYM_W-1 -: IN_W]);
    wire [OUT_W-1:0]  q_q = quantize(cur_sym[IN_W-1:0]);

    assign i_out = (sample_valid ? i_q : {OUT_W{1'b0}}) ^ MSB_MASK;
    assign q_out = (sample_valid ? q_q : {OUT_W{1'b0}}) ^ MSB_MASK;

    assign data_ready   = (state == S_IDLE);
    assign sample_valid = (state == S_TRANSMIT);
    assign busy         = (state != S_IDLE);
    assign done         = (state == S_DONE);
    assign frame_start  = sample_valid && (cnt == CNT_ZERO);
    assign frame_last   = sample_valid && (cnt == CNT_LAST);

endmodule
