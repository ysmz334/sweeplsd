// (d) Adaptive hysteresis low-threshold histogram — SweepLSD v2c.
//
// Fixed-point twin of sweeplsd::kernels::AdaptiveLowTh (../../src/kernels.hpp)
// and the HLS AdaptiveLowTh (../../hls/src/frontend.cpp). A decayed 64-bin
// power histogram whose low threshold is ~2x the 80th-percentile power. Integer
// only: counts x256, decay v-=v>>8, percentile via cum*5 >= total*4 (== 0.8).
//
// TWO-ROW LAG (k=2). The threshold used for NMS row m is lowTh(H_{m-2}). At the
// first column of each NMS row (row_start) we:
//   (a) publish o_low_th = the result of the scan that finished last row
//       (= lowTh(H_{m-2})),
//   (b) snapshot the live histogram (= H_{m-1}) and (re)start a fresh 64-bin
//       percentile scan over that snapshot, and
//   (c) decay the live histogram.
// Row m's power samples are then folded into the live histogram during the row
// (fold_en, subsampled every 4th column, exactly as the software). Because the
// scan runs over a SNAPSHOT it is immune to the concurrent folds, and the two-
// row lag gives it a full row (>= 64 columns) to complete before its result is
// published. REQUIRES width >= 64 (all board inputs are 640/1280/1920); narrow
// unit-test images are not exercised in adaptive mode. See rtl/DESIGN.md.
//
// The comparator lesson from step 1 holds: o_low_th is a plain register, so the
// downstream NMS compare `pc_c >= o_low_th` is register-vs-register with no
// serial arithmetic ahead of it.

module hyst_hist (
    input  wire        clk,
    input  wire        rst,
    input  wire        frame_start, // per-frame clear (histogram is global state
                                    //   and cannot self-clear via the flush rows;
                                    //   each frame must cold-start like SW detect())
    input  wire        en,
    input  wire        row_start,   // 1-tick pulse at the first column of NMS row m
    input  wire        fold_en,     // fold i_pow this tick (subsampled center row)
    input  wire [15:0] i_pow,       // center-row power to fold
    input  wire        hyst_on,     // (d) hysteresis enabled
    input  wire        adaptive,    // adaptive (vs fixed) low threshold
    input  wire [15:0] hyst_low,    // clamp low / user_low
    input  wire [15:0] high,        // clamp high (= power_th)
    output reg  [15:0] o_low_th     // NMS threshold for the current row (registered)
);
    localparam integer NB = 64;
    localparam [26:0]  KUNIT = 27'd256;

    reg [26:0] bins [0:NB-1];
    reg [26:0] snap [0:NB-1];
    reg [31:0] total, snap_total;

    // percentile scan state (runs over `snap`)
    reg        scanning;
    reg [6:0]  idx;          // 0..64
    reg [33:0] cum;
    reg        found;
    reg [5:0]  found_b;
    reg [15:0] th_scan;      // lowTh(snapshot), valid once the scan finishes

    // (fold pipeline) the RMW `bins[bfold] += KUNIT` is split across two cycles:
    // cycle 1 registers the index, cycle 2 does a LOCAL one-hot self-add (each
    // bin reads only itself). This keeps the 64×27 gather mux out of the pixel-
    // rate path — that gather was the front-end's timing failure at 74.25 MHz.
    // The 4-column fold subsample and the inter-row flush gap absorb the extra
    // cycle, so the histogram is bit-identical to the single-cycle RMW.
    reg        f_pend;
    reg [5:0]  f_idx;
    reg        fs_q;         // locally re-registered frame_start (fanout cut)
    reg        rs_q;         // locally re-registered row_start: the raw pulse
                             //   comes off the position comparators and used to
                             //   gate the 64x27-bit decay/snapshot writes — the
                             //   worst remaining pixel-domain path (Yd2->bins).
                             //   Everything below keys off rs_q, one cycle
                             //   later; the publish still lands columns before
                             //   the first NMS compare of the row (window >= 3)
                             //   and the scan budget is unaffected (>= 64 cols).

    reg        fe_q;         // fold_en, delayed with rs_q (uniform +1 shift of
    reg [15:0] pow_q;        //   ALL inputs = exact time translation)

    wire        active = hyst_on & adaptive;

    // fold bin index: clamp (power >> 6) to NB-1
    wire [9:0]  braw  = pow_q[15:6];
    wire [5:0]  bfold = (|braw[9:6]) ? 6'd63 : braw[5:0];

    // TWO-STAGE scan (timing): the 64:1 snapshot gather mux used to chain into
    // the x5 shift-add and the percentile compare in a single cycle — the
    // worst remaining pixel-clock path (15 levels, 13.22 ns of 13.4). Stage 1
    // registers the bin readout (snap_q); stage 2 accumulates and tests. The
    // scan takes one extra cycle overall — far inside the two-row-lag budget —
    // and processes the identical (bin, idx) sequence, so the published
    // threshold is bit-identical.
    reg  [26:0] snap_q;
    reg  [5:0]  sidx_q;
    reg         sq_v;

    // running inclusion test for the stage-1-registered scan bin
    wire [33:0] cum_incl = cum + {7'd0, snap_q};
    wire [37:0] cum5     = {cum_incl, 2'b00} + cum_incl;   // cum_incl * 5
    wire [33:0] want4    = {snap_total, 2'b00};            // snap_total * 4
    wire        hit      = (cum5 >= {4'd0, want4});

    // threshold from the found bin: 2*(b*64+32) = b*128 + 64, clamped
    wire [15:0] th_p       = {found_b, 7'd0} + 16'd64;
    wire [15:0] th_clamped = (snap_total == 32'd0) ? hyst_low :
                             (th_p < hyst_low)     ? hyst_low :
                             (th_p > high)         ? high : th_p;

    integer b;
    always @(posedge clk) begin
        // ---- en-paced part: the percentile scan (small registers only) ------
        if (en) begin
            if (scanning && !rs_q) begin
                // stage 1: registered snapshot readout (gather mux ends here)
                if (idx != NB) begin
                    snap_q <= snap[idx[5:0]];
                    sidx_q <= idx[5:0];
                    sq_v   <= 1'b1;
                    idx    <= idx + 7'd1;
                end else begin
                    sq_v <= 1'b0;
                    if (!sq_v) begin
                        th_scan  <= th_clamped;  // pipeline drained: found_b final
                        scanning <= 1'b0;
                    end
                end
                // stage 2: accumulate + percentile test on the registered bin
                if (sq_v) begin
                    if (!found && hit) begin
                        found   <= 1'b1;
                        found_b <= sidx_q;
                    end
                    cum <= cum_incl;
                end
            end
        end

        // ---- raw-clock one-shot pipeline for the WIDE writes ----------------
        // en is woven into the SET conditions only, so rs_q / fe_q / f_pend
        // pulse for exactly one raw cycle per qualifying en cycle and the
        // 64x27-bit bins/snap write-enables source from PLAIN REGISTERS. The
        // raw enable used to be en(=walker step_ok, a live position compare):
        // the use_w -> in_img -> step_ok cone rode a fanout-386 CE net into
        // every bins flop — the last near-zero-slack path of the pixel domain.
        // With en==1 (the board) the schedule is identical; under CE_DIV the
        // writes land inside the en gap and every en-observed state matches
        // (the CE_DIV gates verify this).
        rs_q  <= en && row_start;
        fe_q  <= en && fold_en && active && !row_start;
        pow_q <= i_pow;
        f_pend <= fe_q;
        if (fe_q) f_idx <= bfold;

        // fold write-back: local one-hot self-add of the registered index.
        // Never coincides with the rs_q decay (the last fold of a row drains
        // during the flush before row_start), so the two writers of `bins`
        // stay exclusive in time.
        if (f_pend)
            for (b = 0; b < NB; b = b + 1)
                if (b == f_idx) bins[b] <= bins[b] + KUNIT;
        if (fe_q) total <= total + KUNIT;

        if (rs_q) begin
            // (a) publish the previous scan; select fixed/off thresholds too
            o_low_th <= hyst_on ? (adaptive ? th_scan : hyst_low) : high;
            sq_v <= 1'b0;                        // never leak into a fresh scan
            if (active) begin
                // (b) snapshot H_{m-1} (pre-decay) and (re)start the scan
                for (b = 0; b < NB; b = b + 1) snap[b] <= bins[b];
                snap_total <= total;
                scanning <= 1'b1;
                idx      <= 7'd0;
                cum      <= 34'd0;
                found    <= 1'b0;
                found_b  <= 6'd63;
                // (c) decay the live histogram
                for (b = 0; b < NB; b = b + 1) bins[b] <= bins[b] - (bins[b] >> 8);
                total <= total - (total >> 8);
            end
        end
        // frame_start is re-registered locally: it clears ~3.5k histogram FFs
        // and the raw net (sourced in live_core) was itself a worst-slack path.
        // One cycle of clear latency is harmless — the first active row is
        // thousands of cycles after frame start.
        fs_q <= frame_start;
        if (rst || fs_q) begin
            for (b = 0; b < NB; b = b + 1) begin
                bins[b] <= 27'd0;
                snap[b] <= 27'd0;
            end
            total <= 32'd0; snap_total <= 32'd0;
            scanning <= 1'b0; idx <= 7'd0; cum <= 34'd0;
            found <= 1'b0; found_b <= 6'd63;
            th_scan <= 16'd0; o_low_th <= 16'd0;
            f_pend <= 1'b0; f_idx <= 6'd0;
            sq_v <= 1'b0; rs_q <= 1'b0; fe_q <= 1'b0;
        end
    end

endmodule
