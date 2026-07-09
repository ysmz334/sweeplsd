// Full-core integration testbench: source pixels in -> front-end chain ->
// event FIFO -> labelling back-end -> segment records, compared against the
// golden records (which the host finalisation turns into detect()'s exact
// segments — hls/host/finalize.hpp). The FIFO's stall gates the front-end
// walker, exercising the elastic backpressure path. See run_tb.sh.

`timescale 1ns / 1ps

`ifndef CE_DIV
`define CE_DIV 1
`endif
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
`ifndef EDGE_BORDER
`define EDGE_BORDER 3
`endif
`ifndef HYST_MIN
`define HYST_MIN 3
`endif
`ifndef BORDER
`define BORDER 0
`endif
`ifndef MPS_2SQ
`define MPS_2SQ 0
`endif

module tb_core_chain;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam integer MAXREC = 8192;
    localparam [15:0] PTH = `POWER_TH;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg run = 1'b0;

    // ---- clock enable (1-in-CE_DIV; see tb_sweep_core.v) ---------------------
    reg [3:0] ce_cnt = 4'd0;
    always @(posedge clk) ce_cnt <= (ce_cnt == `CE_DIV - 1) ? 4'd0 : ce_cnt + 4'd1;
    wire ce = (ce_cnt == 4'd0);

    // ---- front-end walker, gated by FIFO backpressure -----------------------
    wire stall;
    wire en = run && !stall && ce;
    reg [XW-1:0] X = 0, Y = 0;
    wire [XW-1:0] x_next = (X == W + 5) ? {XW{1'b0}} : X + 1'b1;
    wire walking = (Y <= H + 6);

    reg [7:0] gray [0:W*H-1];
    wire [7:0] px = (X < W && Y < H) ? gray[Y * W + X] : 8'd0;

    always @(posedge clk) begin
        if (en && walking) begin
            if (X == W + 5) begin
                X <= 0;
                Y <= Y + 1'b1;
            end else begin
                X <= X + 1'b1;
            end
        end
    end

    wire fe_en = en && walking;

    wire ev_v; wire [1:0] ev_k; wire [XW-1:0] ev_x_o; wire ev_s;
    fe_chain #(.MAXW_LOG2(11), .XW(XW)) u_fe (
        .clk(clk), .rst(rst), .ev_rst(rst), .frame_start(1'b0), .en(fe_en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .power_th(PTH), .strict(`STRICT != 0),
        .hyst_on(`HYST_ON != 0), .hyst_adaptive(`HYST_ADAPT != 0),
        .hyst_low(16'd`HYST_LOW), .edge_border(4'd`EDGE_BORDER),
        .px(px),
        .ev_valid(ev_v), .ev_kind(ev_k), .ev_x(ev_x_o), .ev_strong(ev_s),
        .f_valid(), .f_x(), .f_y(), .f_code());

    // ---- FIFO ------------------------------------------------------------------
    wire fifo_empty, fifo_pop;
    wire [14:0] fifo_front;
    event_fifo #(.DW(15), .AW(11)) u_fifo (
        .clk(clk), .rst(rst), .en(ce),
        .push(ev_v), .wdata({ev_s, ev_k, ev_x_o}), .stall(stall),
        .empty(fifo_empty), .front(fifo_front), .pop(fifo_pop));

    // ---- back-end ------------------------------------------------------------------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;
    wire [87:0] rec_bb = {rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
                          rec_miny, rec_miny_x, rec_maxy, rec_maxy_x};

    backend #(.XW(XW)) u_be (
        .clk(clk), .rst(rst), .en(ce),
        .width(W[XW-1:0]), .height(H[XW-1:0]), .pix_th(18'd`PIX_TH),
        .hyst_on(`HYST_ON != 0), .hyst_strong_min(18'd`HYST_MIN),
        .border(4'd`BORDER), .mps_2sq(5'd`MPS_2SQ),
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

    always #5 clk = ~clk;

    // ---- golden records + comparison ------------------------------------------------
    reg [10:0] g_sx [0:MAXREC-1];
    reg [10:0] g_sy [0:MAXREC-1];
    reg [10:0] g_ex [0:MAXREC-1];
    reg [10:0] g_ey [0:MAXREC-1];
    reg [17:0] g_n  [0:MAXREC-1];
    reg [29:0] g_xs [0:MAXREC-1];
    reg [29:0] g_ys [0:MAXREC-1];
    reg [40:0] g_xss [0:MAXREC-1];
    reg [40:0] g_yss [0:MAXREC-1];
    reg [40:0] g_xys [0:MAXREC-1];
    reg [87:0] g_bb [0:MAXREC-1];
    integer g_cnt;

    integer errors = 0;
    integer ri = 0;
    reg frame_done = 1'b0;

    // rec_valid is a back-end register: under CE_DIV > 1 it stays high across
    // the en-off cycles, so sample it on en cycles only.
    always @(posedge clk) begin
        if (!rst && rec_valid && ce) begin
            if (rec_n == 18'd0) begin
                frame_done <= 1'b1;
            end else begin
                if (ri < g_cnt) begin
                    if (rec_sx !== g_sx[ri] || rec_sy !== g_sy[ri] ||
                        rec_ex !== g_ex[ri] || rec_ey !== g_ey[ri] ||
                        rec_n !== g_n[ri] ||
                        rec_xs !== g_xs[ri] || rec_ys !== g_ys[ri] ||
                        rec_xss !== g_xss[ri] || rec_yss !== g_yss[ri] ||
                        rec_xys !== g_xys[ri] || rec_bb !== g_bb[ri]) begin
                        if (errors == 0)
                            $display("FAIL: record %0d differs", ri);
                        errors = errors + 1;
                    end
                end else begin
                    if (errors == 0)
                        $display("FAIL: extra record %0d (gold has %0d)", ri, g_cnt);
                    errors = errors + 1;
                end
                ri = ri + 1;
            end
        end
    end

    integer cyc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (frame_done) begin
            if (errors == 0 && ri == g_cnt)
                $display("PASS: %0d records bit-exact through the full core", ri);
            else
                $display("FAIL: %0d errors, %0d/%0d records", errors, ri, g_cnt);
            $finish;
        end
        if (cyc > 20000000) begin
            $display("FAIL: watchdog timeout");
            $finish;
        end
    end

    integer fd, r, i;
    reg [10:0] v_sx, v_sy, v_ex, v_ey;
    reg [17:0] v_n;
    reg [29:0] v_xs, v_ys;
    reg [40:0] v_xss, v_yss, v_xys;
    reg [10:0] v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7;
    initial begin
        $readmemh({`VEC, "_gray.hex"}, gray);
        g_cnt = 0;
        fd = $fopen({`VEC, "_records.hex"}, "r");
        if (fd != 0) begin
            r = 18;
            while (r == 18) begin
                r = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h",
                            v_sx, v_sy, v_ex, v_ey, v_n, v_xs, v_ys,
                            v_xss, v_yss, v_xys,
                            v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7);
                if (r == 18) begin
                    g_sx[g_cnt] = v_sx; g_sy[g_cnt] = v_sy;
                    g_ex[g_cnt] = v_ex; g_ey[g_cnt] = v_ey;
                    g_n[g_cnt] = v_n;
                    g_xs[g_cnt] = v_xs; g_ys[g_cnt] = v_ys;
                    g_xss[g_cnt] = v_xss; g_yss[g_cnt] = v_yss;
                    g_xys[g_cnt] = v_xys;
                    g_bb[g_cnt] = {v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7};
                    g_cnt = g_cnt + 1;
                end
            end
            $fclose(fd);
        end
        repeat (6) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);
        run <= 1'b1;
    end

endmodule
