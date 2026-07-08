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

    wire        active = hyst_on & adaptive;

    // fold bin index: clamp (power >> 6) to NB-1
    wire [9:0]  braw  = i_pow[15:6];
    wire [5:0]  bfold = (|braw[9:6]) ? 6'd63 : braw[5:0];

    // running inclusion test for the current scan bin
    wire [33:0] cum_incl = cum + {7'd0, snap[idx[5:0]]};
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
        if (en) begin
            f_pend <= 1'b0;
            // fold write-back (cycle 2): local one-hot self-add of the index
            // registered last cycle. Never coincides with a row_start decay —
            // the last fold of a row drains during the flush before row_start —
            // so the two writers of `bins` are exclusive in time.
            if (f_pend)
                for (b = 0; b < NB; b = b + 1)
                    if (b == f_idx) bins[b] <= bins[b] + KUNIT;

            if (row_start) begin
                // (a) publish the previous scan; select fixed/off thresholds too
                o_low_th <= hyst_on ? (adaptive ? th_scan : hyst_low) : high;
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
            end else if (active && fold_en) begin
                // fold cycle 1: register the target bin; the self-add fires next
                // cycle (see the f_pend block above). total is a plain scalar
                // accumulator (no gather), so fold it here directly.
                f_idx  <= bfold;
                f_pend <= 1'b1;
                total  <= total + KUNIT;
            end

            // percentile scan advances one snapshot bin per tick (independent of
            // the fold above; scan reads `snap`, fold writes `bins`). Guarded by
            // !row_start so a fresh start (width >= 64 => scan already done) never
            // collides with the advance on `idx`.
            if (scanning && !row_start) begin
                if (idx == NB) begin
                    th_scan  <= th_clamped;   // found_b now final (last set at idx 63)
                    scanning <= 1'b0;
                end else begin
                    if (!found && hit) begin
                        found   <= 1'b1;
                        found_b <= idx[5:0];
                    end
                    cum <= cum_incl;
                    idx <= idx + 7'd1;
                end
            end
        end
        if (rst || frame_start) begin
            for (b = 0; b < NB; b = b + 1) begin
                bins[b] <= 27'd0;
                snap[b] <= 27'd0;
            end
            total <= 32'd0; snap_total <= 32'd0;
            scanning <= 1'b0; idx <= 7'd0; cum <= 34'd0;
            found <= 1'b0; found_b <= 6'd63;
            th_scan <= 16'd0; o_low_th <= 16'd0;
            f_pend <= 1'b0; f_idx <= 6'd0;
        end
    end

endmodule
