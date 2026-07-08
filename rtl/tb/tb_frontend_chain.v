// Integration testbench: the full RTL front-end chain
//   gray -> stage_gauss -> stage_gradient -> stage_edge -> stage_feature
//        -> event_pack
// driven only by source pixels, checked against BOTH the golden feature grid
// and the golden event stream (dump_vectors). See run_tb.sh.

`timescale 1ns / 1ps

`ifndef STRICT
`define STRICT 0
`endif
`ifndef HYST_ON
`define HYST_ON 0
`endif
`ifndef HYST_ADAPT
`define HYST_ADAPT 0
`endif
`ifndef HYST_LOW
`define HYST_LOW 120
`endif

module tb_frontend_chain;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam [15:0] PTH = `POWER_TH;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg en = 1'b0;
    reg [XW-1:0] X = 0, Y = 0;
    wire [XW-1:0] x_next = (X == W + 5) ? {XW{1'b0}} : X + 1'b1;

    reg [7:0]  gray [0:W*H-1];
    reg [1:0]  gold_f [0:W*H-1];
    reg [15:0] gold_ev [0:2*W*H+H];   // packed kind(4) x(12) per line
    integer n_gold_ev;

    wire [7:0] px = (X < W && Y < H) ? gray[Y * W + X] : 8'd0;

    wire f_v; wire [XW-1:0] f_x, f_y; wire [1:0] f_f;
    wire ev_v; wire [1:0] ev_k; wire [XW-1:0] ev_x; wire ev_s;
    fe_chain #(.MAXW_LOG2(11), .XW(XW)) u_fe (
        .clk(clk), .rst(rst), .ev_rst(rst), .frame_start(1'b0), .en(en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .power_th(PTH), .strict(`STRICT != 0),
        .hyst_on(`HYST_ON != 0), .hyst_adaptive(`HYST_ADAPT != 0),
        .hyst_low(16'd`HYST_LOW),
        .px(px),
        .ev_valid(ev_v), .ev_kind(ev_k), .ev_x(ev_x), .ev_strong(ev_s),
        .f_valid(f_v), .f_x(f_x), .f_y(f_y), .f_code(f_f));

    always #5 clk = ~clk;

    integer errors = 0;
    integer checked = 0;
    integer ev_idx = 0;
    integer eof_seen = 0;

    always @(posedge clk) begin
        if (!rst && en) begin
            if (f_v) begin
                checked = checked + 1;
                if (f_f !== gold_f[f_y * W + f_x]) begin
                    if (errors == 0)
                        $display("FAIL: feature mismatch at (%0d,%0d): rtl %0d gold %0d",
                                 f_x, f_y, f_f, gold_f[f_y * W + f_x]);
                    errors = errors + 1;
                end
            end
            if (ev_v) begin
                if ({1'b0, ev_s, ev_k, ev_x} !== gold_ev[ev_idx]) begin
                    if (errors == 0)
                        $display("FAIL: event %0d: rtl s=%0d k=%0d x=%0d gold %04x",
                                 ev_idx, ev_s, ev_k, ev_x, gold_ev[ev_idx]);
                    errors = errors + 1;
                end
                ev_idx = ev_idx + 1;
                if (ev_k == 2'd0) eof_seen = 1;
            end
            if (X == W + 5) begin
                X <= 0;
                Y <= Y + 1'b1;
            end else begin
                X <= X + 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (Y == H + 6 && X == 8) begin  // leave room for the deferred EOR/EOF
            if (errors == 0 && checked == W * H && eof_seen && ev_idx == n_gold_ev)
                $display("PASS: %0d feature samples + %0d events bit-exact",
                         checked, ev_idx);
            else
                $display("FAIL: %0d errors, %0d/%0d samples, %0d/%0d events, eof=%0d",
                         errors, checked, W * H, ev_idx, n_gold_ev, eof_seen);
            $finish;
        end
    end

    integer i;
    initial begin
        $readmemh({`VEC, "_gray.hex"}, gray);
        $readmemh({`VEC, "_feat.hex"}, gold_f);
        for (i = 0; i < 2 * W * H + H; i = i + 1) gold_ev[i] = 16'hxxxx;
        $readmemh({`VEC, "_events.hex"}, gold_ev);
        n_gold_ev = 0;
        for (i = 0; i < 2 * W * H + H; i = i + 1)
            if (gold_ev[i] !== 16'hxxxx) n_gold_ev = n_gold_ev + 1;
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        en <= 1'b1;
    end

endmodule
