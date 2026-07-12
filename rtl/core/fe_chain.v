// Front-end chain (v2b M2): gauss -> gradient -> edge -> feature ->
// event_pack, with the pipeline bookkeeping in ONE place so sweep_core and
// the testbenches cannot drift apart.
//
// Pipelining scheme ("delayed context"): the stages themselves keep their
// original column-lag constants; timing cuts are made by (a) the internal
// vertical/horizontal split inside stage_gauss (+1 tick) and (b) a register
// on the gauss output (+1 tick), and the downstream stages are simply driven
// with a position context (X, Y, x_next) delayed by the same total number of
// ticks. Within their own frame of reference nothing changed, so the event
// stream is bit-identical to the unpipelined chain — it just leaves 2 ticks
// later. The walker's flush margins (6 columns / 7 rows) absorb the shift;
// the frame's EOF still falls well inside the final flush row.
//
// All delay registers are `en`-gated like everything else, so stalls freeze
// the whole pipe coherently and CE_DIV operation stays an exact time
// dilation.

module fe_chain #(
    parameter MAXW_LOG2 = 11,
    parameter XW        = 12
) (
    input  wire            clk,
    input  wire            rst,         // stages + delay registers
    input  wire            ev_rst,      // event packer (per-frame: rst || frame_start)
    input  wire            frame_start, // (d) per-frame histogram clear (stage_edge)
    input  wire            en,
    input  wire [XW-1:0]   X,           // walker position (input side)
    input  wire [XW-1:0]   Y,
    input  wire [XW-1:0]   x_next,
    input  wire [XW-1:0]   width,
    input  wire [XW-1:0]   height,
    input  wire [15:0]     power_th,
    input  wire            strict,
    input  wire            hyst_on,      // (d) hysteresis
    input  wire            hyst_adaptive,
    input  wire [15:0]     hyst_low,
    input  wire [3:0]      edge_border, // (border edge exclusion; 0 = off)
    input  wire [7:0]      px,          // src(X, Y); read when in-image

    output wire            ev_valid,
    output wire [1:0]      ev_kind,
    output wire [XW-1:0]   ev_x,
    output wire            ev_strong,

    // feature-stage taps (tb_frontend_chain checks these; unused elsewhere)
    output wire            f_valid,
    output wire [XW-1:0]   f_x,
    output wire [XW-1:0]   f_y,
    output wire [1:0]      f_code
);

    // ---- frame-geometry register slice ---------------------------------------
    // width/height are quasi-static (they change only with the measured video
    // mode) but they used to fan combinationally from live_core's measurement
    // registers across the die into every stage's position comparators — the
    // worst path of the whole pixel-clock domain (13.2 ns of 13.4, use_w ->
    // stage_edge). A local register stage lets PAR place the source next to
    // the consumers; one cycle of staleness on a quasi-static value is
    // harmless (the fe idles across mode changes).
    reg [XW-1:0] w_q, h_q;
    reg [15:0]   pth_q;      // power_th: constant or a frame-stepped register
                             // (adaptive supply control) — same treatment
    always @(posedge clk) begin
        w_q <= width;
        h_q <= height;
        pth_q <= power_th;
    end

    // ---- stage 1: gaussian (internally 2-staged; output = pos@t-1 - 2) ------
    wire g_v; wire [XW-1:0] g_x, g_y; wire [13:0] g_g;
    stage_gauss #(.MAXW_LOG2(MAXW_LOG2), .XW(XW)) u_gauss (
        .clk(clk), .rst(rst), .en(en), .X(X), .Y(Y), .x_next(x_next),
        .width(w_q), .height(h_q), .px(px),
        .o_valid(g_v), .o_x(g_x), .o_y(g_y), .o_g(g_g));

    // ---- delayed position context + gauss output register --------------------
    // Xd2 = X@t-2: one tick from the gauss-internal split, one from g_*_r.
    reg [XW-1:0] Xd1, Yd1, xnd1;
    reg [XW-1:0] Xd2, Yd2, xnd2;
    reg          g_v_r;
    reg [13:0]   g_g_r;
    always @(posedge clk) begin
        if (en) begin
            Xd1 <= X;    Yd1 <= Y;    xnd1 <= x_next;
            Xd2 <= Xd1;  Yd2 <= Yd1;  xnd2 <= xnd1;
            g_v_r <= g_v;
            g_g_r <= g_g;
        end
        if (rst) begin
            Xd1 <= {XW{1'b0}};  Yd1 <= {XW{1'b0}};  xnd1 <= {XW{1'b0}};
            Xd2 <= {XW{1'b0}};  Yd2 <= {XW{1'b0}};  xnd2 <= {XW{1'b0}};
            g_v_r <= 1'b0;
            g_g_r <= 14'd0;
        end
    end

    // ---- stages 2..4 + event packer, all on the 2-tick-delayed context --------
    wire d_v; wire [XW-1:0] d_x, d_y; wire [15:0] d_p; wire d_d;
    stage_gradient #(.MAXW_LOG2(MAXW_LOG2), .XW(XW)) u_grad (
        .clk(clk), .rst(rst), .en(en), .X(Xd2), .Y(Yd2), .x_next(xnd2),
        .width(w_q), .height(h_q),
        .i_valid(g_v_r), .i_g(g_g_r),
        .o_valid(d_v), .o_x(d_x), .o_y(d_y), .o_power(d_p), .o_dir(d_d));

    wire e_v; wire [XW-1:0] e_x, e_y; wire e_e; wire e_strong;
    stage_edge #(.MAXW_LOG2(MAXW_LOG2), .XW(XW)) u_edge (
        .clk(clk), .rst(rst), .frame_start(frame_start), .en(en),
        .X(Xd2), .Y(Yd2), .x_next(xnd2),
        .width(w_q), .height(h_q), .power_th(pth_q), .strict(strict),
        .hyst_on(hyst_on), .hyst_adaptive(hyst_adaptive), .hyst_low(hyst_low),
        .edge_border(edge_border),
        .i_valid(d_v), .i_power(d_p), .i_dir(d_d),
        .o_valid(e_v), .o_x(e_x), .o_y(e_y), .o_e(e_e), .o_strong(e_strong));

    wire f_strong;
    stage_feature #(.MAXW_LOG2(MAXW_LOG2), .XW(XW)) u_feat (
        .clk(clk), .rst(rst), .en(en), .X(Xd2), .Y(Yd2), .x_next(xnd2),
        .width(w_q), .height(h_q),
        .i_valid(e_v), .i_e(e_e), .i_strong(e_strong),
        .o_valid(f_valid), .o_x(f_x), .o_y(f_y), .o_f(f_code), .o_strong(f_strong));

    event_pack #(.XW(XW)) u_ev (
        .clk(clk), .rst(ev_rst), .en(en),
        .width(w_q), .height(h_q),
        .i_valid(f_valid), .i_x(f_x), .i_y(f_y), .i_f(f_code), .i_strong(f_strong),
        .ev_valid(ev_valid), .ev_kind(ev_kind), .ev_x(ev_x), .ev_strong(ev_strong));

endmodule
