// Synthesis-only wrapper for timing the FRONT-END chain in isolation
// (v2b M2): walker + gauss -> gradient -> edge -> feature -> event_pack,
// registered I/O, no backend/FIFO. Used with synth_fe/run.sh to measure the
// critical path against the 720p pixel period (13.468 ns) after each
// pipelining step. Not part of any board build.

module front_synth (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire [7:0]  px_in,
    output reg         ev_valid_q,
    output reg  [1:0]  ev_kind_q,
    output reg  [11:0] ev_x_q,
    output reg         ev_strong_q
);
    localparam XW = 12;
    wire [XW-1:0] width  = 12'd1280;
    wire [XW-1:0] height = 12'd720;

    reg [7:0] px;
    always @(posedge clk) px <= px_in;

    // free-running walker (no stall in this harness)
    reg [XW-1:0] X, Y;
    wire [XW-1:0] x_next = (X == width + 5) ? {XW{1'b0}} : X + 1'b1;
    always @(posedge clk) begin
        if (rst) begin
            X <= {XW{1'b0}};
            Y <= {XW{1'b0}};
        end else if (en) begin
            if (X == width + 5) begin
                X <= {XW{1'b0}};
                Y <= (Y == height + 6) ? {XW{1'b0}} : Y + 1'b1;
            end else begin
                X <= X + 1'b1;
            end
        end
    end

    wire ev_v; wire [1:0] ev_k; wire [XW-1:0] ev_x; wire ev_s;
    fe_chain #(.MAXW_LOG2(11), .XW(XW)) u_fe (
        .clk(clk), .rst(rst), .ev_rst(rst), .frame_start(1'b0), .en(en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(width), .height(height),
        .power_th(16'd256), .strict(1'b1),           // v2c board setting (constant,
        .hyst_on(1'b1), .hyst_adaptive(1'b1), .hyst_low(16'd120),  // folds like 1'b1)
        .edge_border(4'd3),
        .px(px),
        .ev_valid(ev_v), .ev_kind(ev_k), .ev_x(ev_x), .ev_strong(ev_s),
        .f_valid(), .f_x(), .f_y(), .f_code());

    always @(posedge clk) begin
        ev_valid_q <= ev_v;
        ev_kind_q <= ev_k;
        ev_x_q <= ev_x;
        ev_strong_q <= ev_s;
    end

endmodule
