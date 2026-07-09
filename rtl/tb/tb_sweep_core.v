// Parity testbench for the assembled portable core (sweep_core): pixels in
// through the px_valid/px_ready handshake, records compared against golden.
// Runs TWO frames back-to-back to prove the per-frame re-initialisation.
// See run_tb.sh.
//
// -DCE_DIV=N drives the core's clock enable at 1-in-N (default 1 = full
// rate); the record stream must be bit-identical at any N (boards/atlys v2a
// runs the core at N = 2).

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

module tb_sweep_core;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam integer MAXREC = 8192;
    localparam [15:0] PTH = `POWER_TH;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg frame_start = 1'b0;

    // clock-enable generator: 1-in-CE_DIV
    reg [3:0] ce_cnt = 4'd0;
    always @(posedge clk) ce_cnt <= (ce_cnt == `CE_DIV - 1) ? 4'd0 : ce_cnt + 4'd1;
    wire ce = (ce_cnt == 4'd0);

    reg [7:0] gray [0:W*H-1];
    integer p_i;
    reg feeding;
    wire px_valid = feeding && (p_i < W * H);
    wire [7:0] px = gray[p_i];
    wire px_ready, busy;

    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;
    wire [87:0] rec_bb = {rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
                          rec_miny, rec_miny_x, rec_maxy, rec_maxy_x};

    sweep_core #(.XW(XW)) dut (
        .clk(clk), .rst(rst), .en(ce),
        .drop_mode(1'b0), .ev_dropped(),
        .frame_start(frame_start),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .power_th(PTH), .strict(`STRICT != 0),
        .hyst_on(`HYST_ON != 0), .hyst_adaptive(`HYST_ADAPT != 0),
        .hyst_low(16'd`HYST_LOW), .hyst_strong_min(18'd`HYST_MIN),
        .edge_border(4'd`EDGE_BORDER),
        .pix_th(18'd`PIX_TH),
        .border(4'd`BORDER), .mps_2sq(5'd`MPS_2SQ),
        .px_valid(px_valid), .px(px), .px_ready(px_ready), .busy(busy),
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

    always @(posedge clk) if (px_valid && px_ready) p_i <= p_i + 1;

    // golden records
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
    integer frames_done = 0;

    always @(posedge clk) begin
        if (!rst && rec_valid) begin
            if (rec_n == 18'd0) begin
                if (ri != g_cnt) begin
                    $display("FAIL: frame %0d ended with %0d/%0d records",
                             frames_done, ri, g_cnt);
                    errors = errors + 1;
                end
                frames_done = frames_done + 1;
                ri = 0;
            end else begin
                if (ri < g_cnt) begin
                    if (rec_sx !== g_sx[ri] || rec_sy !== g_sy[ri] ||
                        rec_ex !== g_ex[ri] || rec_ey !== g_ey[ri] ||
                        rec_n !== g_n[ri] ||
                        rec_xs !== g_xs[ri] || rec_ys !== g_ys[ri] ||
                        rec_xss !== g_xss[ri] || rec_yss !== g_yss[ri] ||
                        rec_xys !== g_xys[ri] || rec_bb !== g_bb[ri]) begin
                        if (errors == 0)
                            $display("FAIL: frame %0d record %0d differs",
                                     frames_done, ri);
                        errors = errors + 1;
                    end
                end else begin
                    errors = errors + 1;
                end
                ri = ri + 1;
            end
        end
    end

    integer cyc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (frames_done == 2) begin
            if (errors == 0)
                $display("PASS: 2 frames x %0d records bit-exact via sweep_core", g_cnt);
            else
                $display("FAIL: %0d errors", errors);
            $finish;
        end
        if (cyc > 40000000) begin
            $display("FAIL: watchdog timeout (frames=%0d ri=%0d)", frames_done, ri);
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

        feeding = 1'b0;
        p_i = 0;
        repeat (6) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);

        // frame 1
        frame_start <= 1'b1; @(posedge clk); frame_start <= 1'b0;
        // wait for the back-end init sweep, then feed
        repeat (2100 * `CE_DIV) @(posedge clk);
        feeding <= 1'b1;
        p_i = 0;
        while (frames_done < 1) @(posedge clk);
        feeding <= 1'b0;

        // frame 2 (same image; proves per-frame re-init)
        repeat (16) @(posedge clk);
        frame_start <= 1'b1; @(posedge clk); frame_start <= 1'b0;
        repeat (2100 * `CE_DIV) @(posedge clk);
        p_i = 0;
        feeding <= 1'b1;
    end

endmodule
