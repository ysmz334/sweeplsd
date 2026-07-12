// DENSE multi-frame live-path wedge hunter (diagnostic).
// Streams a DENSE scene through the full live path (video timing -> live_core
// -> DE-slaved walker -> fe chain -> event_pack -> boundary register -> FIFO
// with drop_mode -> concurrent-ingest backend -> records), alternating the
// scene every frame (pixel inversion) to model scene changes. Golden records
// are NOT compared (drops are expected at this density); the oracle is frame
// LIVENESS: every frame must emit its end-record (rec_n==0) and the live
// handshake must keep restarting. A per-frame watchdog dumps the internal
// state (walker busy / live eof_seen / backend state / ingest_y / proc_y /
// FIFO count) when a frame wedges — the live freeze reproduced in sim.

`timescale 1ns / 1ps

`ifndef STRICT
`define STRICT 1
`endif
`ifndef NFRAMES
`define NFRAMES 6
`endif
`ifndef HB
`define HB 24
`endif
`ifndef VB
`define VB 24
`endif

module tb_live_dense;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer HBLANK = `HB;
    localparam integer VBLANK = `VB;
    localparam integer VFP = 8, VSW = 8;
    localparam integer HTOTAL = W + HBLANK;
    localparam integer VTOTAL = H + VBLANK;
    localparam integer FRAME_CYC = HTOTAL * VTOTAL;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    // ---- video timing generator ------------------------------------------
    reg [11:0] hc, vc;
    reg [7:0]  frame_no;              // increments at each vc wrap
    always @(posedge clk) begin
        if (rst) begin
            hc <= 12'd0;
            vc <= H[11:0] + 12'd1;
            frame_no <= 8'd0;
        end else if (hc == HTOTAL - 1) begin
            hc <= 12'd0;
            if (vc == VTOTAL - 1) begin
                vc <= 12'd0;
                frame_no <= frame_no + 8'd1;   // scene change point
            end else vc <= vc + 12'd1;
        end else begin
            hc <= hc + 12'd1;
        end
    end
    wire de_c = (hc < W) && (vc < H);
    wire vs_c = (vc >= H + VFP) && (vc < H + VFP + VSW);

    reg [7:0] gray [0:W*H-1];
    reg [7:0] gray2 [0:W*H-1];
    reg de, vsync;
    reg [7:0] pix;
    // scene alternates every frame between two DIFFERENT images (density AND
    // power-histogram change, like a real scene cut)
    wire [7:0] praw  = gray[vc * W + hc];
    wire [7:0] praw2 = gray2[vc * W + hc];
    always @(posedge clk) begin
        de <= de_c && !rst;
        vsync <= vs_c && !rst;
        pix <= de_c ? (frame_no[0] ? praw2 : praw) : 8'd0;
    end

    // ---- DUT ---------------------------------------------------------------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;
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

    // ---- overlay in the loop (the one component never simmed) --------------
    // sof = display frame start ~ vsync fall (same as atlys_rx_top wiring)
    reg vs_q2;
    always @(posedge clk) vs_q2 <= vsync;
    wire sof = !vsync && vs_q2;
    wire ov;
    reg [11:0] ddx, ddy;   // display raster (same timing generator counts)
    always @(posedge clk) begin
        ddx <= hc; ddy <= vc;
    end
    overlay_mask #(.HW(640), .VH(360)) u_ov (
        .clk(clk), .rst(rst),
        .res_shift(W > 1280),
        .rec_valid(rec_valid), .rec_last(rec_n == 18'd0),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .sof(sof), .dx(ddx), .dy(ddy), .ov(ov)
    );
    // dump the FRONT mask as a PGM right after each swap (peek hierarchically)
    integer dumpn = 0;
    integer fpgm, mx, my;
    initial fpgm = $fopen("ovmask_dump.txt", "w");
    task dump_mask;
        integer v;
        begin
            $fwrite(fpgm, "FRAME %0d\n", dumpn);
            for (my = 0; my < 360; my = my + 1) begin
                for (mx = 0; mx < 640; mx = mx + 1) begin
                    v = u_ov.front_sel ? u_ov.mask1[my*640+mx]
                                       : u_ov.mask0[my*640+mx];
                    $fwrite(fpgm, "%0d", v ? 1 : 0);
                end
                $fwrite(fpgm, "\n");
            end
            dumpn = dumpn + 1;
        end
    endtask
    reg sof_d;
    integer ov_ends = 0;
    always @(posedge clk) begin
        sof_d <= sof;
        if (!rst && rec_valid && rec_n == 18'd0) ov_ends = ov_ends + 1;
        if (sof_d && !rst && ov_ends > 0 && dumpn < 6) dump_mask;
    end

    // ---- liveness bookkeeping ---------------------------------------------
    integer recs_this_pass = 0;
    integer maxrow = 0;
    integer q1=0,q2=0,q3=0,q4=0;
    integer passes_done = 0;
    integer starts_seen = 0;
    integer drops_total = 0;
    integer last_end_cyc = 0;
    integer cyc = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (frame_start) begin
                starts_seen = starts_seen + 1;
                $display("[cyc %0d] frame_start #%0d (video frame_no=%0d)",
                         cyc, starts_seen, frame_no);
            end
            if (dut.u_core.ev_dropped) drops_total = drops_total + 1;
            if (rec_valid) begin
                if (rec_n == 18'd0) begin
                    passes_done = passes_done + 1;
                    $display("[cyc %0d] pass %0d END: %0d records, max_row %0d, quarts %0d/%0d/%0d/%0d, drops so far %0d",
                             cyc, passes_done, recs_this_pass, maxrow, q1, q2, q3, q4, drops_total);
                    q1=0;q2=0;q3=0;q4=0;
                    recs_this_pass = 0;
                    maxrow = 0;
                    last_end_cyc = cyc;
                end else begin
                    recs_this_pass = recs_this_pass + 1;
                    if (rec_ey > maxrow) maxrow = rec_ey;
                    if (rec_sy > maxrow) maxrow = rec_sy;
                    if (rec_ey < (H/4)) q1 = q1 + 1;
                    else if (rec_ey < (H/2)) q2 = q2 + 1;
                    else if (rec_ey < (3*H/4)) q3 = q3 + 1;
                    else q4 = q4 + 1;
                end
            end
        end
    end

    // pass-level watchdog: if no pass END for 3 frame times, dump and die
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (passes_done >= `NFRAMES) begin
            $display("PASS-LIVE: %0d passes completed, %0d starts, %0d drops total",
                     passes_done, starts_seen, drops_total);
            $finish;
        end
        if (!rst && (cyc - last_end_cyc) > 3 * FRAME_CYC + 100000) begin
            $display("WEDGE: no pass END for 3+ frames (passes=%0d starts=%0d)",
                     passes_done, starts_seen);
            $display("  live: busy=%0d eof_seen=%0d skip_cnt=%0d",
                     busy, dut.eof_seen, dut.skip_cnt);
            $display("  backend: state=%0d ingest_y=%0d proc_y=%0d eof_seen=%0d",
                     dut.u_core.u_be.state, dut.u_core.u_be.ingest_y,
                     dut.u_core.u_be.proc_y, dut.u_core.u_be.eof_seen);
            $display("  fifo: count=%0d  fe: ev_v=%0d evr_v=%0d",
                     dut.u_core.u_fifo.count, dut.u_core.ev_v, dut.u_core.evr_v);
            $display("  video: frame_no=%0d vc=%0d", frame_no, vc);
            $finish;
        end
        if (cyc > 60000000) begin
            $display("FAIL: global watchdog (passes=%0d)", passes_done);
            $finish;
        end
    end

    initial begin
        $readmemh({`VEC, "_gray.hex"}, gray);
`ifdef VEC2
        $readmemh({`VEC2, "_gray.hex"}, gray2);
`else
        $readmemh({`VEC, "_gray.hex"}, gray2);
`endif
        repeat (8) @(posedge clk);
        rst <= 1'b0;
    end

endmodule
