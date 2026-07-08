// Stage 1b — 2x2 gradient on gaussian rows y, y+1 -> power (|dx|+|dy|+1)/2
// and H/V direction (thesis §3.2.1.2-4). Port of hls/src/frontend.cpp
// hlsGradient; bit-exact against sweeplsd::computeGradient (rtl/tb).
//
// Lockstep-walker interface (rtl/DESIGN.md): the stage sees the global walk
// position (X, Y) and consumes the gaussian sample of (X-2, Y-2) produced by
// stage_gauss in the same tick; it emits the gradient of (X-3, Y-3). The 2x2
// window needs one gaussian line buffer (row iy-1) plus one-column-delayed
// registers; out-of-image taps are zero (right column X=w, bottom row y=h),
// matching the software's zero stand-ins.

module stage_gradient #(
    parameter MAXW_LOG2 = 11,
    parameter XW        = 12
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            en,
    input  wire [XW-1:0]   X,
    input  wire [XW-1:0]   Y,
    input  wire [XW-1:0]   x_next,
    input  wire [XW-1:0]   width,
    input  wire [XW-1:0]   height,

    input  wire            i_valid,     // gaussian sample (X-2, Y-2) valid
    input  wire [13:0]     i_g,

    output wire            o_valid,     // gradient sample (o_x, o_y) this tick
    output wire [XW-1:0]   o_x,
    output wire [XW-1:0]   o_y,
    output wire [15:0]     o_power,
    output wire            o_dir        // 1 = vertical edge (|dx| dominant)
);

    // Input-sample coordinates (gaussian lattice).
    wire [XW-1:0] ix = X - 2;           // only meaningful when X >= 2
    wire [XW-1:0] ix_next = x_next - 2;

    // Gaussian line buffer: row iy-1.
    reg [13:0] lbg [0:(1<<MAXW_LOG2)-1];
    reg [13:0] lbg_q;                   // prefetched column ix

    wire [13:0] g11 = i_valid ? i_g : 14'd0;                    // g(iy,   ix)
    wire [13:0] g10 = (X >= 2 && ix < width) ? lbg_q : 14'd0;   // g(iy-1, ix)

    reg [13:0] g00;                     // g(iy-1, ix-1)  (previous tick's g10)
    reg [13:0] g01;                     // g(iy,   ix-1)  (previous tick's g11)

    // 2x2 window at output position (ox, oy) = (X-3, Y-3):
    //   g00 g10      dx = |(g10+g11) - (g00+g01)|
    //   g01 g11      dy = |(g01+g11) - (g00+g10)|
    wire [15:0] right2 = {2'd0, g10} + {2'd0, g11};
    wire [15:0] left2  = {2'd0, g00} + {2'd0, g01};
    wire [15:0] bot2   = {2'd0, g01} + {2'd0, g11};
    wire [15:0] top2   = {2'd0, g00} + {2'd0, g10};
    wire [15:0] dx = (right2 >= left2) ? (right2 - left2) : (left2 - right2);
    wire [15:0] dy = (bot2 >= top2) ? (bot2 - top2) : (top2 - bot2);
    wire [16:0] psum = {1'b0, dx} + {1'b0, dy} + 17'd1;

    wire [XW-1:0] ox = X - 3;
    wire [XW-1:0] oy = Y - 3;
    // position-only, not en-gated (see stage_gauss.v)
    assign o_valid = (X >= 3) && (Y >= 3) && (ox < width) && (oy < height);
    assign o_x = ox;
    assign o_y = oy;
    assign o_power = psum[16:1];        // (dx + dy + 1) >> 1, <= 32640
    assign o_dir = (dx > dy);

    always @(posedge clk) begin
        if (en) begin
            if (i_valid) lbg[ix[MAXW_LOG2-1:0]] <= i_g;
            lbg_q <= lbg[ix_next[MAXW_LOG2-1:0]];
            g00 <= g10;
            g01 <= g11;
        end
        if (rst) begin
            lbg_q <= 14'd0;
            g00 <= 14'd0;
            g01 <= 14'd0;
        end
    end

endmodule
