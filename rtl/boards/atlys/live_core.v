// v2b M3 glue: feed sweep_core from a LIVE video stream (de/vsync/gray on
// the recovered pixel clock, en = 1 — front-end and back-end both close
// timing at 74.25 MHz, see rtl/synth_fe / rtl/synth_be).
//
// The walker is slaved to DE: in-image positions consume exactly one pixel
// per asserted de (px_valid = de), and the flush columns/rows advance during
// blanking. Both target modes leave ample margin for the 6-column / 7-row
// flush (720p60: 370 cols / 30 lines; 1080p30: 280 cols / 45 lines), so a
// pass finishes inside the vertical blank and EVERY frame is processed
// (60 Hz / 30 Hz detection). Frame geometry is MEASURED from de/vsync and a
// pass starts only after two consecutive frames measure identically, so the
// same bitstream follows 720p60, 1080p30 or any mode within the line-buffer
// budget (width <= 1920).
//
// Frame handshake: a pass starts on the TRAILING edge of vsync (for 720p's
// positive sync that is front porch + sync = 10 lines after the last active
// line — enough for the walker flush ~9k clk AND the back-end drain), and
// only once the previous pass has emitted its end-of-frame record
// (eof_seen). If a pass wedges (e.g. the source resolution does not match
// WIDTH/HEIGHT and the walker starves), three skipped vsyncs force a
// restart — the video pass-through is never affected, only the overlay.
//
// The input cannot be stalled, so a FIFO-backpressure stall during DE loses
// pixels and shears the rest of the pass; `drop_latch` reports that (also
// covers walker/DE misalignment) until the next frame start. Recovery is
// automatic at the next frame start.

module live_core #(
    parameter XW = 12
) (
    input  wire        clk,           // recovered pixel clock
    input  wire        rst,
    input  wire        de,            // data enable (aligned with gray)
    input  wire        vsync,         // positive-pulse vertical sync
    input  wire [7:0]  gray,
    input  wire        strict,        // improved-mode NMS tie-break (v2c a)

    output wire        res_shift,     // overlay scale: 0 = /2, 1 = /4 (FullHD)

    output wire        rec_valid,
    output wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey,
    output wire [17:0] rec_n,         // 0 = end of frame
    // (f) bbox extreme points of the record (for the overlay endpoint choice)
    output wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
    output wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x,
    output wire        busy,
    output reg         drop_latch,    // pixel lost / misaligned this pass
    output reg         evdrop_latch,  // events discarded (density ceiling hit)
    output reg         frame_start,   // exposed for debug counters
    output wire [5:0]  dbg_be_state,  // back-end FSM state (wedge diagnosis)
    output wire [2:0]  dbg_be_cond,
    output wire        dbg_push,
    output wire        dbg_push_eor,
    output wire        dbg_pop,
    output wire        dbg_jwd_fire,
    output wire        dbg_fl_zero,
    output wire        dbg_jstall,
    output wire        dbg_jf,
    output wire        dbg_evdrop,   // per-event drop pulse (telemetry)
    output wire [15:0] dbg_pth,      // (kept for port stability; constant)
    output wire [11:0] dbg_use_w,    // measured frame geometry (wedge diagnosis)
    output wire [11:0] dbg_use_h
);

    // register the stream once (timing isolation; pair stays aligned)
    reg        de_r;
    reg [7:0]  gray_r;
    reg        vs_d;
    always @(posedge clk) begin
        de_r <= de && !rst;
        gray_r <= gray;
        vs_d <= vsync && !rst;
    end
    wire vs_fall = !vsync && vs_d;

    // ---- frame geometry measurement (v2b: auto 720p60 / 1080p30 / ...) --------
    // Count DE pixels per line and DE lines per frame; a pass may only start
    // when the just-finished frame measured EXACTLY like the previous one
    // (and in range), so a source mode switch never feeds the core stale
    // dimensions. use_w/use_h update only at a stable vsync — never during a
    // pass (passes start at the same edge).
    reg        de_p;
    reg [11:0] cnt_x, cnt_y, line_w;
    reg        geom_bad;
    reg [11:0] meas_w, meas_h;
    reg [11:0] use_w, use_h;
    wire stable_now = !geom_bad && cnt_y != 12'd0 &&
                      line_w == meas_w && cnt_y == meas_h &&
                      line_w >= 12'd16 && line_w <= 12'd1920 &&
                      cnt_y >= 12'd16 && cnt_y <= 12'd1080;
    always @(posedge clk) begin
        de_p <= de_r;
        if (de_r) cnt_x <= cnt_x + 12'd1;
        if (de_p && !de_r) begin                   // end of an active line
            if (cnt_y == 12'd0) line_w <= cnt_x;
            else if (cnt_x != line_w) geom_bad <= 1'b1;
            cnt_y <= cnt_y + 12'd1;
            cnt_x <= 12'd0;
        end
        if (vs_fall) begin
            if (stable_now) begin
                use_w <= line_w;
                use_h <= cnt_y;
            end
            meas_w <= line_w;
            meas_h <= cnt_y;
            cnt_x <= 12'd0;
            cnt_y <= 12'd0;
            line_w <= 12'd0;
            geom_bad <= 1'b0;
        end
        if (rst) begin
            de_p <= 1'b0;
            cnt_x <= 12'd0; cnt_y <= 12'd0; line_w <= 12'd0;
            geom_bad <= 1'b0;
            meas_w <= 12'd0; meas_h <= 12'd0;
            use_w <= 12'd0; use_h <= 12'd0;
        end
    end
    // registered: use_w spans the die into the overlay's coordinate muxes —
    // quasi-static, so a settling cycle is harmless (was a worst-slack path)
    reg res_shift_q;
    always @(posedge clk) res_shift_q <= (use_w > 12'd1280);
    assign res_shift = res_shift_q;
    assign dbg_use_w = use_w;
    assign dbg_use_h = use_h;

    // ---- pass start control ---------------------------------------------------
    reg eof_seen;
    reg [1:0] skip_cnt;   // skipped-vsync counter (force a restart after 3)
    wire force_start = vs_fall && stable_now && (skip_cnt >= 2'd2);
    always @(posedge clk) begin
        frame_start <= (vs_fall && stable_now && !busy && eof_seen) || force_start;
        if (rec_valid && rec_n == 18'd0) eof_seen <= 1'b1;
        if (vs_fall) begin
            if (busy || !eof_seen) begin
                if (skip_cnt != 2'd2) skip_cnt <= skip_cnt + 2'd1;   // saturate
            end else begin
                skip_cnt <= 2'd0;
            end
        end
        if (frame_start) begin
            eof_seen <= 1'b0;
            skip_cnt <= 2'd0;
        end
        if (rst) begin
            frame_start <= 1'b0;
            eof_seen <= 1'b1;      // nothing to drain before the first pass
            skip_cnt <= 2'd0;
        end
    end

    // NOTE (2026-07-12): a multi-frame adaptive threshold controller
    // (AIMD on power_th / hyst_low, diag builds v14-v16) lived here while
    // hunting the live bottom-loss. It treated the symptom, confused the
    // demo (thresholds changing over seconds have no SW counterpart), and
    // the loss persisted even with full supply authority — the evidence
    // now points at a board-only back-end drain anomaly. Removed at the
    // author's request; thresholds are fixed to the SW defaults again.
    assign dbg_pth = 16'd256;

    // ---- detector core ----------------------------------------------------------
    wire px_ready;
    wire ev_dropped;
    assign dbg_evdrop = ev_dropped;
    wire [29:0] rec_xs, rec_ys;      // moments unused by the overlay demo
    wire [40:0] rec_xss, rec_yss, rec_xys;
    sweep_core #(.XW(XW)) u_core (
        .clk(clk), .rst(rst), .en(1'b1),
        .drop_mode(1'b1), .ev_dropped(ev_dropped),   // live: drop, never stall
        .frame_start(frame_start),
        .width(use_w), .height(use_h),
        .power_th(16'd256), .strict(strict), .pix_th(18'd15),
        // (d) hysteresis follows the improved toggle: adaptive low threshold,
        // clamp-low 120, and a >= 3 strong-pixel gate (matches --improved).
        .hyst_on(strict), .hyst_adaptive(1'b1),
        .hyst_low(16'd120), .hyst_strong_min(18'd3),
        // (i) border margin 3 + (h) max_perp_spread=1 (2*mps^2 = 2) also follow
        // the improved toggle (matches --improved / Params::improved()).
        .border(strict ? 4'd3 : 4'd0), .mps_2sq(strict ? 5'd2 : 5'd0),
        // border edge exclusion: always on (core faithful behaviour, both modes)
        .edge_border(4'd3),
        .px_valid(de_r), .px(gray_r), .px_ready(px_ready),
        .px_x(), .px_y(), .px_addr_x(), .px_addr_y(),
        .busy(busy),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n), .rec_xs(rec_xs), .rec_ys(rec_ys),
        .rec_xss(rec_xss), .rec_yss(rec_yss), .rec_xys(rec_xys),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x),
        .dbg_be_state(dbg_be_state),
        .dbg_be_cond(dbg_be_cond),
        .dbg_push(dbg_push), .dbg_push_eor(dbg_push_eor), .dbg_pop(dbg_pop),
        .dbg_jwd_fire(dbg_jwd_fire),
        .dbg_fl_zero(dbg_fl_zero),
        .dbg_jstall(dbg_jstall),
        .dbg_jf(dbg_jf)
    );

    // A DE pixel the walker could not take = misalignment (with drop_mode
    // the FIFO never stalls the walker, so this should stay 0 whenever the
    // source resolution matches WIDTH x HEIGHT).
    always @(posedge clk) begin
        if (de_r && busy && !px_ready) drop_latch <= 1'b1;
        if (ev_dropped) evdrop_latch <= 1'b1;
        if (frame_start || rst) begin
            drop_latch <= 1'b0;
            evdrop_latch <= 1'b0;
        end
    end

endmodule
