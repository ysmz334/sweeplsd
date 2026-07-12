// DROP-WIPE mechanism probe (diagnostic).
// Question (board bottom-loss bug): does sustained FIFO saturation below a
// row WIPE all records there (random per-event drops fragment every run to
// ~(1-p)/p px < pix_th), rather than merely "thin" them?
//
// Method: run the full live path on a dense FullHD vector. Frames 0-1 run
// clean (control). From frame 2 on, force the FIFO's afull high with a
// pseudo-random duty (DROPP: 0 = 50%, 1 = 25%, 2 = 12.5%) whenever the video
// row is past INJROW — emulating "saturation from row INJROW on" exactly as
// drop_mode sheds load on the board. Per pass we report record count,
// max end-row, row quartiles, drops, and the labeller's min fl_count
// (exhaustion rival theory: expect it never reaches 0).
//
// Confirmed prediction = quartiles below INJROW collapse to ~0 in the
// injected passes while control passes match the clean run.

`timescale 1ns / 1ps

`ifndef STRICT
`define STRICT 1
`endif
`ifndef NFRAMES
`define NFRAMES 4
`endif
`ifndef HB
`define HB 280
`endif
`ifndef VB
`define VB 45
`endif
`ifndef INJROW
`define INJROW 300
`endif
`ifndef DROPP
`define DROPP 0
`endif

module tb_dropwipe;
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
    reg [7:0]  frame_no;
    always @(posedge clk) begin
        if (rst) begin
            hc <= 12'd0;
            vc <= H[11:0] + 12'd1;
            frame_no <= 8'd0;
        end else if (hc == HTOTAL - 1) begin
            hc <= 12'd0;
            if (vc == VTOTAL - 1) begin
                vc <= 12'd0;
                frame_no <= frame_no + 8'd1;
            end else vc <= vc + 12'd1;
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

    // ---- DUT ---------------------------------------------------------------
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

    // ---- saturation injection ----------------------------------------------
    reg [15:0] lfsr = 16'hACE1;
    wire lbit = (`DROPP == 0) ? lfsr[0]
              : (`DROPP == 1) ? (lfsr[1:0] == 2'b00)
                              : (lfsr[2:0] == 3'b000);
    wire inject = !rst && (frame_no >= 8'd2) && (vc > `INJROW) && (vc < H);
    always @(posedge clk) begin
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        if (inject && lbit) force dut.u_core.u_fifo.afull = 1'b1;
        else                release dut.u_core.u_fifo.afull;
    end

    // ---- bookkeeping ---------------------------------------------------------
    integer recs_this_pass = 0;
    integer maxrow = 0;
    integer q1=0,q2=0,q3=0,q4=0;
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
                    $display("pass %0d: %0d records, max_row %0d, quarts %0d/%0d/%0d/%0d, drops %0d, min_fl %0d",
                             passes_done, recs_this_pass, maxrow,
                             q1, q2, q3, q4, drops_pass, flmin);
                    q1=0;q2=0;q3=0;q4=0;
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
