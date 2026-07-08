// Stage 1a — 5x5 separable gaussian, {16,64,96,64,16}, single >>10 rescale in
// the horizontal pass (thesis §3.2.1.1). Port of hls/src/frontend.cpp
// hlsGaussian; bit-exact against sweeplsd::gaussianBlur (rtl/tb/tb_gauss.v).
//
// Lockstep-walker interface (see rtl/DESIGN.md): the surrounding walker
// advances one position (X, Y) per enabled clock over a raster larger than
// the image; this stage consumes the source pixel when (X, Y) is inside the
// image and emits the gaussian value of (X-2, Y-2) with zeroed borders
// (rows/cols < 2 or >= dim-2), exactly like the software.
//
// Line buffers are inferred dual-port block RAM with registered (synchronous)
// reads, so the column to be used at walk position X is prefetched while the
// walker sits at X-1 (`x_next` addressing). One read + one write per buffer
// per cycle.
//
// v2b M2: the stage is internally TWO-staged — the vertical sum `vs` is
// registered before the horizontal pass (the single-cycle BRAM-read ->
// vertical -> horizontal chain does not fit a 13.5 ns 720p pixel period).
// The input side (line-buffer write/prefetch) still runs on (X, x_next); the
// output side (window, border gating, o_x/o_y) runs on an internal one-tick
// delayed copy of the position, so the output contract becomes "sample of
// (X@t-1 - 2, Y@t-1 - 2) at tick t". Consumers must therefore be driven
// with a matching delayed position context — fe_chain.v does exactly that.

module stage_gauss #(
    parameter MAXW_LOG2 = 11,          // line-buffer depth = 2^MAXW_LOG2
    parameter XW        = 12           // walk-coordinate width
) (
    input  wire            clk,
    input  wire            rst,        // synchronous, clears buffer validity
    input  wire            en,         // walker advance
    input  wire [XW-1:0]   X,          // current walk position
    input  wire [XW-1:0]   Y,
    input  wire [XW-1:0]   x_next,     // walk X one tick ahead (BRAM prefetch)
    input  wire [XW-1:0]   width,      // image size (<= 2^MAXW_LOG2)
    input  wire [XW-1:0]   height,
    input  wire [7:0]      px,         // src(X, Y); only read when in-image

    output wire            o_valid,    // gaussian sample (o_x, o_y) this tick
    output wire [XW-1:0]   o_x,
    output wire [XW-1:0]   o_y,
    output wire [13:0]     o_g
);

    // Source-row line buffers: lb3 = row Y-1 ... lb0 = row Y-4.
    reg [7:0] lb0 [0:(1<<MAXW_LOG2)-1];
    reg [7:0] lb1 [0:(1<<MAXW_LOG2)-1];
    reg [7:0] lb2 [0:(1<<MAXW_LOG2)-1];
    reg [7:0] lb3 [0:(1<<MAXW_LOG2)-1];
    reg [7:0] lb0_q, lb1_q, lb2_q, lb3_q;   // prefetched column X

    wire in_img = (X < width) && (Y < height);
    wire [7:0] cur = in_img ? px : 8'd0;

    // Vertical 5-tap at row Y-2, column X:
    //   16*(row[Y-4] + row[Y]) + 64*(row[Y-3] + row[Y-1]) + 96*row[Y-2]
    // Written as shifts+adds: a literal `*` makes XST build a DSP48 cascade
    // here (PCIN hops alone ~10 ns — the bulk of the front-end critical
    // path); carry-chain adders are several times faster for these widths.
    wire [8:0]  vs04 = {1'b0, lb0_q} + {1'b0, cur};
    wire [8:0]  vs13 = {1'b0, lb1_q} + {1'b0, lb3_q};
    wire [15:0] vs = {3'd0, vs04, 4'd0}                 // 16*(lb0+cur)
                   + {1'd0, vs13, 6'd0}                 // 64*(lb1+lb3)
                   + {2'd0, lb2_q, 6'd0}                // 96*lb2 = 64*lb2
                   + {3'd0, lb2_q, 5'd0};               //         + 32*lb2
                                                        // total <= 65280

    // ---- pipeline register: vertical sum + the position it belongs to ---------
    reg [15:0]   vs_r;
    reg [XW-1:0] Xi, Yi;                 // position of the vs_r column

    // Horizontal 5-tap window over vertical sums: after the shift below,
    // vwin[4-i] holds the sum of column Xi-i (vwin[0] = Xi-4, vs_r = Xi).
    reg [15:0] vwin0, vwin1, vwin2, vwin3;
    wire [16:0] hs04 = {1'b0, vwin0} + {1'b0, vs_r};
    wire [16:0] hs13 = {1'b0, vwin1} + {1'b0, vwin3};
    wire [24:0] hsum = {4'd0, hs04, 4'd0}               // 16*(vwin0+vs_r)
                     + {2'd0, hs13, 6'd0}               // 64*(vwin1+vwin3)
                     + {3'd0, vwin2, 6'd0}              // 96*vwin2
                     + {4'd0, vwin2, 5'd0};             // <= ~1.67e7

    // Output position (Xi-2, Yi-2) and border gating (one tick behind X, Y).
    wire [XW-1:0] r  = Yi - 2;
    wire [XW-1:0] xc = Xi - 2;
    wire in_out = (Yi >= 2) && (Xi >= 2) && (r < height) && (xc < width);
    wire border = (r < 2) || (r >= height - 2) || (xc < 2) || (xc >= width - 2);

    // Position-only validity — deliberately NOT gated with `en`: o_valid
    // feeds the next stage's data muxes, and an en term would drag the
    // (fast, every-cycle) clock-enable signal into the long combinational
    // datapath, breaking the v2a multicycle constraint (all sequential
    // consumers are en-gated anyway, so non-en-cycle values are discarded).
    assign o_valid = in_out;
    assign o_x = xc;
    assign o_y = r;
    assign o_g = border ? 14'd0 : hsum[23:10];          // >>10, <= 16320

    always @(posedge clk) begin
        if (en) begin
            // Row shift + prefetch. Writes land at column X (in-image cols
            // only); the read of column x_next serves the next tick.
            if (X < width) begin
                lb0[X[MAXW_LOG2-1:0]] <= lb1_q;
                lb1[X[MAXW_LOG2-1:0]] <= lb2_q;
                lb2[X[MAXW_LOG2-1:0]] <= lb3_q;
                lb3[X[MAXW_LOG2-1:0]] <= cur;
            end
            lb0_q <= lb0[x_next[MAXW_LOG2-1:0]];
            lb1_q <= lb1[x_next[MAXW_LOG2-1:0]];
            lb2_q <= lb2[x_next[MAXW_LOG2-1:0]];
            lb3_q <= lb3[x_next[MAXW_LOG2-1:0]];

            // Pipeline register + horizontal window shift (fed by vs_r, one
            // tick behind the vertical pass).
            vs_r <= vs;
            Xi <= X;
            Yi <= Y;
            vwin0 <= vwin1;
            vwin1 <= vwin2;
            vwin2 <= vwin3;
            vwin3 <= vs_r;
        end
        if (rst) begin
            lb0_q <= 8'd0; lb1_q <= 8'd0; lb2_q <= 8'd0; lb3_q <= 8'd0;
            vs_r <= 16'd0;
            Xi <= {XW{1'b0}};
            Yi <= {XW{1'b0}};
            vwin0 <= 16'd0; vwin1 <= 16'd0; vwin2 <= 16'd0; vwin3 <= 16'd0;
        end
    end

endmodule
