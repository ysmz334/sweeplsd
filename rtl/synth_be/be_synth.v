// Synthesis-only wrapper for timing the BACK-END (event_fifo + backend +
// judge_unit) in isolation at the 720p pixel period (v2b M2 rate decision:
// single clock domain if it closes, pclk/2 behind a dual-rate FIFO if not).
// Registered I/O; not part of any board build.

module be_synth (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        push_in,
    input  wire [14:0] wdata_in,
    output reg         stall_q,
    output reg         rec_valid_q,
    output reg  [7:0]  sig_q          // XOR fold of all record fields (keeps
                                      // the logic alive without ~250 IOBs)
);
    localparam XW = 12;

    reg push;
    reg [14:0] wdata;
    always @(posedge clk) begin
        push <= push_in;
        wdata <= wdata_in;
    end

    wire fifo_empty, fifo_pop, stall;
    wire [14:0] fifo_front;
    event_fifo #(.DW(15), .AW(11)) u_fifo (
        .clk(clk), .rst(rst), .en(en),
        .push(push), .wdata(wdata), .stall(stall),
        .empty(fifo_empty), .front(fifo_front), .pop(fifo_pop));

    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;

    backend #(.XW(XW)) u_be (
        .clk(clk), .rst(rst), .en(en),
        .width(12'd1280), .height(12'd720), .pix_th(18'd15),
        .hyst_on(1'b1), .hyst_strong_min(18'd3),
        .ev_empty(fifo_empty), .ev_kind(fifo_front[13:12]),
        .ev_x(fifo_front[11:0]), .ev_strong(fifo_front[14]), .ev_pop(fifo_pop),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n), .rec_xs(rec_xs), .rec_ys(rec_ys),
        .rec_xss(rec_xss), .rec_yss(rec_yss), .rec_xys(rec_xys),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x)
    );

    wire [7:0] fold =
          rec_sx[7:0] ^ {rec_sx[10:8], 5'd0}
        ^ rec_sy[7:0] ^ {rec_sy[10:8], 5'd0}
        ^ rec_ex[7:0] ^ {rec_ex[10:8], 5'd0}
        ^ rec_ey[7:0] ^ {rec_ey[10:8], 5'd0}
        ^ rec_n[7:0] ^ rec_n[15:8] ^ {6'd0, rec_n[17:16]}
        ^ rec_xs[7:0] ^ rec_xs[15:8] ^ rec_xs[23:16] ^ {2'd0, rec_xs[29:24]}
        ^ rec_ys[7:0] ^ rec_ys[15:8] ^ rec_ys[23:16] ^ {2'd0, rec_ys[29:24]}
        ^ rec_xss[7:0] ^ rec_xss[15:8] ^ rec_xss[23:16] ^ rec_xss[31:24] ^ rec_xss[39:32] ^ {7'd0, rec_xss[40]}
        ^ rec_yss[7:0] ^ rec_yss[15:8] ^ rec_yss[23:16] ^ rec_yss[31:24] ^ rec_yss[39:32] ^ {7'd0, rec_yss[40]}
        ^ rec_xys[7:0] ^ rec_xys[15:8] ^ rec_xys[23:16] ^ rec_xys[31:24] ^ rec_xys[39:32] ^ {7'd0, rec_xys[40]}
        ^ rec_minx[7:0] ^ {rec_minx[10:8], 5'd0}
        ^ rec_minx_y[7:0] ^ {rec_minx_y[10:8], 5'd0}
        ^ rec_maxx[7:0] ^ {rec_maxx[10:8], 5'd0}
        ^ rec_maxx_y[7:0] ^ {rec_maxx_y[10:8], 5'd0}
        ^ rec_miny[7:0] ^ {rec_miny[10:8], 5'd0}
        ^ rec_miny_x[7:0] ^ {rec_miny_x[10:8], 5'd0}
        ^ rec_maxy[7:0] ^ {rec_maxy[10:8], 5'd0}
        ^ rec_maxy_x[7:0] ^ {rec_maxy_x[10:8], 5'd0};

    always @(posedge clk) begin
        stall_q <= stall;
        rec_valid_q <= rec_valid;
        if (rec_valid) sig_q <= sig_q ^ fold;
        if (rst) sig_q <= 8'd0;
    end

endmodule
