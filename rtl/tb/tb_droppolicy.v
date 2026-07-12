// DROP-POLICY A/B probe (diagnostic).
// Real saturation dynamics: shrink the FIFO's data capacity via defparam
// (RESERVE up, e.g. 1900 -> 148 data slots) so a dense FullHD vector
// genuinely fills and drains it, then compare shedding policies:
//   -DDHYST=0    legacy per-event thinning  (expect: records below the
//                saturation row wiped — the board bottom-loss)
//   -DDHYST=100  hysteretic shedding        (expect: bottom rows recover,
//                thinned but populated)
// Reports per pass: records, row quartiles, drops, min fl_count, and the
// bottom-half orientation split (verticals are the fragile case: keep-bands
// shorter than a row still cut them).

`timescale 1ns / 1ps

`ifndef STRICT
`define STRICT 1
`endif
`ifndef NFRAMES
`define NFRAMES 2
`endif
`ifndef HB
`define HB 280
`endif
`ifndef VB
`define VB 45
`endif
`ifndef RSV
`define RSV 1900
`endif
`ifndef DHYST
`define DHYST 0
`endif

module tb_droppolicy;
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
    always @(posedge clk) begin
        if (rst) begin
            hc <= 12'd0;
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
    wire [7:0] praw = gray[vc * W + hc];
    always @(posedge clk) begin
        de <= de_c && !rst;
        vsync <= vs_c && !rst;
        pix <= de_c ? praw : 8'd0;
    end

    // ---- DUT (FIFO capacity shrunk / policy selected via defparam) ---------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire        busy, drop_latch, evdrop_latch, frame_start;

    live_core #(.XW(12)) dut (
        .clk(clk), .rst(rst),
        .de(de), .vsync(vsync), .gray(pix),
        .strict(`STRICT != 0),
        .res_shift(),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n),
        .rec_minx(), .rec_minx_y(), .rec_maxx(), .rec_maxx_y(),
        .rec_miny(), .rec_miny_x(), .rec_maxy(), .rec_maxy_x(),
        .busy(busy), .drop_latch(drop_latch),
        .evdrop_latch(evdrop_latch),
        .frame_start(frame_start),
        .dbg_be_state(), .dbg_be_cond(),
        .dbg_push(), .dbg_push_eor(), .dbg_pop(),
        .dbg_jwd_fire(), .dbg_fl_zero(),
        .dbg_use_w(), .dbg_use_h()
    );
    defparam dut.u_core.u_fifo.RESERVE = `RSV;
    defparam dut.u_core.u_fifo.DROP_HYST = `DHYST;

    // ---- bookkeeping ---------------------------------------------------------
    integer recs_this_pass = 0;
    integer maxrow = 0;
    integer q1=0,q2=0,q3=0,q4=0;
    integer bh_v=0, bh_h=0;      // bottom-half orientation split
    integer dy_abs, dx_abs;
    integer passes_done = 0;
    integer drops_pass = 0;
    integer flmin = 2048;
    integer last_end_cyc = 0;
    integer cyc = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (dut.u_core.ev_dropped) drops_pass = drops_pass + 1;
            if (dut.u_core.u_be.state != 6'd0 &&
                dut.u_core.u_be.fl_count < flmin)
                flmin = dut.u_core.u_be.fl_count;
            if (rec_valid) begin
                if (rec_n == 18'd0) begin
                    passes_done = passes_done + 1;
                    $display("pass %0d: %0d records, max_row %0d, quarts %0d/%0d/%0d/%0d, bh_v %0d bh_h %0d, drops %0d, min_fl %0d, pth %0d",
                             passes_done, recs_this_pass, maxrow,
                             q1, q2, q3, q4, bh_v, bh_h, drops_pass, flmin, dut.dbg_pth);
                    q1=0;q2=0;q3=0;q4=0;
                    bh_v=0;bh_h=0;
                    recs_this_pass = 0;
                    maxrow = 0;
                    drops_pass = 0;
                    flmin = 2048;
                    last_end_cyc = cyc;
                end else begin
                    recs_this_pass = recs_this_pass + 1;
                    if (rec_ey > maxrow) maxrow = rec_ey;
                    if (rec_sy > maxrow) maxrow = rec_sy;
                    if (rec_ey < (H/4)) q1 = q1 + 1;
                    else if (rec_ey < (H/2)) q2 = q2 + 1;
                    else if (rec_ey < (3*H/4)) q3 = q3 + 1;
                    else q4 = q4 + 1;
                    if (rec_ey >= (H/2)) begin
                        dy_abs = (rec_ey > rec_sy) ? rec_ey - rec_sy
                                                   : rec_sy - rec_ey;
                        dx_abs = (rec_ex > rec_sx) ? rec_ex - rec_sx
                                                   : rec_sx - rec_ex;
                        if (dy_abs >= dx_abs) bh_v = bh_v + 1;
                        else bh_h = bh_h + 1;
                    end
                end
            end
        end
    end

    always @(posedge clk) begin
        cyc = cyc + 1;
        if (passes_done >= `NFRAMES) begin
            $display("PASS: %0d passes completed", passes_done);
            $finish;
        end
        if (!rst && (cyc - last_end_cyc) > 3 * FRAME_CYC + 100000) begin
            $display("WEDGE: no pass END for 3+ frames (passes=%0d)", passes_done);
            $finish;
        end
        if (cyc > 60000000) begin
            $display("FAIL: global watchdog (passes=%0d)", passes_done);
            $finish;
        end
    end

    initial begin
        $readmemh({`VEC, "_gray.hex"}, gray);
        repeat (8) @(posedge clk);
        rst <= 1'b0;
    end

endmodule
