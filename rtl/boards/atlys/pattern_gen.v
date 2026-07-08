// Deterministic test scene, a pure function of (x, y) — no frame buffer, so
// the same generator instance can serve the display path and (a second
// instance) the detector's walker at their own paces. Content chosen to
// exercise the detector: straight bright bars at assorted slopes (should be
// detected) plus a circle ring (should NOT yield long segments — the
// aspect-ratio judgment rejects curves), on a dark background.
//
// Combinational core with one output register; the caller absorbs the
// 1-cycle latency.

// REGISTERED=0: combinational output. 1: one output register (1-tick
// latency — the detector's px_addr prefetch contract). 2: two-stage pipeline
// (latency 2) for full-pixel-rate consumers: the x/y-derived arithmetic
// (line distances, ring d^2 — the DSP-mult depth) is registered first, the
// range compares and colour mux second; the single-cycle version's 9-level
// ~14 ns path does not fit a 13.3 ns 720p pixel period.
// `en` clock-enables the output register(s) so a clock-enabled consumer (the
// CE'd detector core) sees pattern(addr@tick k-1) at en-tick k — the same
// one-tick latency as the full-rate case. Tie en = 1 otherwise.
module pattern_gen #(
    parameter REGISTERED = 1
) (
    input  wire        clk,
    input  wire        en,
    input  wire [11:0] x,
    input  wire [11:0] y,
    output wire [7:0]  gray
);

    // signed workspace
    wire signed [13:0] sx = {2'd0, x};
    wire signed [13:0] sy = {2'd0, y};

    // Scene coordinates sized for the 1280x720 demo mode (the 640x480 v1
    // scene scaled by 1.5; ring tolerance scaled with r so the ring keeps
    // its thickness). The scene splits into part 1 (coordinate arithmetic +
    // range windows — all the DSP-mult depth) and part 2 (the shallow
    // distance-window compares + colour mux), so REGISTERED == 2 can insert
    // its pipeline register between the parts without duplicating logic.

    // ---- part 1: line/ring distances and validity ranges ---------------------
    // horizontal bar y=90, x in [90, 840] / vertical bar x=120, y in [150, 630]
    wire b_h = (y >= 89) && (y <= 91) && (x >= 90) && (x <= 840);
    wire b_v = (x >= 119) && (x <= 121) && (y >= 150) && (y <= 630);
    // 45 degrees: x - y = 150, x in [210, 690]
    wire signed [13:0] d45 = sx - sy - 14'sd150;
    wire r_45 = (x >= 210) && (x <= 690);
    // slope -2/3: 2x + 3y = 2250, x in [150, 900]
    wire signed [15:0] d23 = {sx, 1'b0} + sy * 14'sd3 - 16'sd2250;
    wire r_23 = (x >= 150) && (x <= 900);
    // slope 5/2: 5x - 2y = 2100, y in [90, 660]
    wire signed [16:0] d52 = sx * 14'sd5 - {sy, 1'b0} - 17'sd2100;
    wire r_52 = (y >= 90) && (y <= 660);
    // circle ring, centre (720, 450), r = 135: |d2 - r^2| <= 540
    wire signed [13:0] cdx = sx - 14'sd720;
    wire signed [13:0] cdy = sy - 14'sd450;
    wire signed [27:0] c_d2 = cdx * cdx + cdy * cdy - 28'sd18225;

    // ---- part 2: distance windows + colour ------------------------------------
    function [7:0] shade(
        input bh, input bv,
        input signed [13:0] f45, input v45,
        input signed [15:0] f23, input v23,
        input signed [16:0] f52, input v52,
        input signed [27:0] fc2);
        reg bright;
        begin
            bright = bh | bv
                   | ((f45 >= -14'sd2) && (f45 <= 14'sd2) && v45)
                   | ((f23 >= -16'sd6) && (f23 <= 16'sd6) && v23)
                   | ((f52 >= -17'sd9) && (f52 <= 17'sd9) && v52)
                   | ((fc2 >= -28'sd540) && (fc2 <= 28'sd540));
            shade = bright ? 8'd220 : 8'd30;
        end
    endfunction

    wire [7:0] gray_c = shade(b_h, b_v, d45, r_45, d23, r_23, d52, r_52, c_d2);

    generate
        if (REGISTERED == 2) begin : g_pipe
            reg p_h, p_v, p_45, p_23, p_52;
            reg signed [13:0] d45_r;
            reg signed [15:0] d23_r;
            reg signed [16:0] d52_r;
            reg signed [27:0] c_d2_r;
            reg [7:0] gray_r;
            always @(posedge clk) if (en) begin
                p_h <= b_h; p_v <= b_v;
                p_45 <= r_45; p_23 <= r_23; p_52 <= r_52;
                d45_r <= d45; d23_r <= d23; d52_r <= d52; c_d2_r <= c_d2;
                gray_r <= shade(p_h, p_v, d45_r, p_45, d23_r, p_23,
                                d52_r, p_52, c_d2_r);
            end
            assign gray = gray_r;
        end else if (REGISTERED == 1) begin : g_reg
            reg [7:0] gray_r;
            always @(posedge clk) if (en) gray_r <= gray_c;
            assign gray = gray_r;
        end else begin : g_comb
            assign gray = gray_c;
        end
    endgenerate

endmodule
