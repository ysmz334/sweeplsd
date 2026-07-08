// Parity testbench for the v2b M3 live-video glue (live_core): a scaled-down
// 720p-style video timing generator (active WxH plus blanking, positive
// vsync) streams the golden gray image as de/gray, and the records of two
// consecutive frames must match the golden records — proving the DE-slaved
// walker feed, the vsync frame handshake and the eof_seen drain gate.
// Blanking is sized like 720p relative to the flush margins: hblank > 6+2
// columns, vblank(front+sync) lines * htotal > walker flush + drain.
// See run_tb.sh.

`timescale 1ns / 1ps

`ifndef STRICT
`define STRICT 0
`endif

module tb_live;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer HBLANK = 24;
    localparam integer VBLANK = 24;                  // total blank lines
    localparam integer VFP = 8, VSW = 8;             // vsync lines VFP..VFP+VSW-1
    localparam integer HTOTAL = W + HBLANK;
    localparam integer VTOTAL = H + VBLANK;
    localparam integer MAXREC = 8192;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    // ---- video timing generator -------------------------------------------------
    reg [11:0] hc, vc;
    always @(posedge clk) begin
        if (rst) begin
            hc <= 12'd0;
            // start inside the vertical blank, before the sync pulse
            vc <= H[11:0] + 12'd1;
        end else if (hc == HTOTAL - 1) begin
            hc <= 12'd0;
            vc <= (vc == VTOTAL - 1) ? 12'd0 : vc + 12'd1;
        end else begin
            hc <= hc + 12'd1;
        end
    end
    wire de_c = (hc < W) && (vc < H);
    wire vs_c = (vc >= H + VFP) && (vc < H + VFP + VSW);

    reg [7:0] gray [0:W*H-1];
    reg de, vsync;
    reg [7:0] pix;
    always @(posedge clk) begin
        de <= de_c && !rst;
        vsync <= vs_c && !rst;
        pix <= de_c ? gray[vc * W + hc] : 8'd0;
    end

    // ---- DUT ------------------------------------------------------------------------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;
    wire [87:0] rec_bb = {rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
                          rec_miny, rec_miny_x, rec_maxy, rec_maxy_x};
    wire        busy, drop_latch, evdrop_latch, frame_start;

    live_core #(.XW(12)) dut (
        .clk(clk), .rst(rst),
        .de(de), .vsync(vsync), .gray(pix),
        .strict(`STRICT != 0),
        .res_shift(),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x),
        .busy(busy), .drop_latch(drop_latch),
        .evdrop_latch(evdrop_latch),
        .frame_start(frame_start)
    );

    // ---- golden records (endpoint contacts only — the overlay's inputs) ---------
    reg [10:0] g_sx [0:MAXREC-1];
    reg [10:0] g_sy [0:MAXREC-1];
    reg [10:0] g_ex [0:MAXREC-1];
    reg [10:0] g_ey [0:MAXREC-1];
    reg [17:0] g_n  [0:MAXREC-1];
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
                        rec_n !== g_n[ri] || rec_bb !== g_bb[ri]) begin
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
        if (!rst && (drop_latch || evdrop_latch)) begin
            $display("FAIL: drop latch asserted (misalign=%0d evdrop=%0d)",
                     drop_latch, evdrop_latch);
            errors = errors + 1000;
        end
    end

    integer cyc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (frames_done == 2 || errors >= 1000) begin
            if (errors == 0)
                $display("PASS: 2 live frames x %0d records bit-exact via live_core", g_cnt);
            else
                $display("FAIL: %0d errors", errors);
            $finish;
        end
        if (cyc > 40000000) begin
            $display("FAIL: watchdog timeout (frames=%0d ri=%0d busy=%0d)",
                     frames_done, ri, busy);
            $finish;
        end
    end

    integer fd, r;
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
                    g_bb[g_cnt] = {v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7};
                    g_cnt = g_cnt + 1;
                end
            end
            $fclose(fd);
        end
        repeat (8) @(posedge clk);
        rst <= 1'b0;
    end

endmodule
