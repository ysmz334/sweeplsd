// Stage 1c — threshold + non-maximum suppression (thesis §3.2.1.5-7). Port of
// hls/src/frontend.cpp hlsEdge; bit-exact against sweeplsd::extractEdges
// (baseline: strict = 0). See rtl/DESIGN.md for the walker interface.
//
// Consumes the gradient sample of (X-3, Y-3); emits the edge bit of
// (X-4, Y-4). Competitors: vertical edges compete left/right on the centre
// power row, horizontal edges up/down (rows r-1 / r+1). Needs two power line
// buffers (rows iy-2, iy-1) and one direction line buffer (row iy-1); the
// out-of-image taps read zero through position gating (no buffer clearing
// needed — the bottom flush rows rewrite both power buffers with zeros before
// the next frame).

module stage_edge #(
    parameter MAXW_LOG2 = 11,
    parameter XW        = 12
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            frame_start, // (d) per-frame histogram clear
    input  wire            en,
    input  wire [XW-1:0]   X,
    input  wire [XW-1:0]   Y,
    input  wire [XW-1:0]   x_next,
    input  wire [XW-1:0]   width,
    input  wire [XW-1:0]   height,
    input  wire [15:0]     power_th,    // (d) the HIGH threshold (strong bit)
    input  wire            strict,      // improved-mode tie-break (baseline 0)
    input  wire            hyst_on,     // (d) hysteresis: NMS uses the LOW threshold
    input  wire            hyst_adaptive,
    input  wire [15:0]     hyst_low,    // fixed low threshold / adaptive clamp-low
    input  wire [3:0]      edge_border, // outer ring not a valid edge (0 = off)

    input  wire            i_valid,     // gradient sample (X-3, Y-3) valid
    input  wire [15:0]     i_power,
    input  wire            i_dir,

    output wire            o_valid,     // edge bit (o_x, o_y) this tick
    output wire [XW-1:0]   o_x,
    output wire [XW-1:0]   o_y,
    output wire            o_e,
    output wire            o_strong     // (d) power(o_x,o_y) >= power_th (HIGH)
);

    wire [XW-1:0] ix = X - 3;
    wire [XW-1:0] ix_next = x_next - 3;
    wire in_col = (X >= 3) && (ix < width);

    // Line buffers: lpa = power row iy-2, lpc = power row iy-1, ldir = dir
    // row iy-1.
    reg [15:0] lpa [0:(1<<MAXW_LOG2)-1];
    reg [15:0] lpc [0:(1<<MAXW_LOG2)-1];
    reg        ldir [0:(1<<MAXW_LOG2)-1];
    reg [15:0] lpa_q, lpc_q;
    reg        ldir_q;

    wire [15:0] pw = i_valid ? i_power : 16'd0;
    wire        dr = i_valid ? i_dir : 1'b0;
    wire [15:0] pa_x     = in_col ? lpa_q : 16'd0;   // pa row, column ix
    wire [15:0] pc_right = in_col ? lpc_q : 16'd0;   // pc row, column ix
    wire        dir_x    = in_col ? ldir_q : 1'b0;

    // Column-delay registers -> taps at output column ox = X-4.
    reg [15:0] pc_m;     // pc[ox-1]
    reg [15:0] pc_c;     // pc[ox]
    reg [15:0] pa_d;     // pa[ox]
    reg [15:0] pb_d;     // pb[ox] (incoming row iy, one column behind)
    reg        dir_d;    // dir[ox]

    wire [XW-1:0] ox = X - 4;
    wire [XW-1:0] oy = Y - 4;

    // pa row is oy-1: for the first output row it is outside the image.
    wire [15:0] pa_eff = (oy >= 1) ? pa_d : 16'd0;

    wire [15:0] Pm = dir_d ? pc_m : pa_eff;
    wire [15:0] Pp = dir_d ? pc_right : pb_d;

    // (d) adaptive/fixed LOW threshold for NMS. o_low_th is a plain register
    // (updated once per row at row_start), so the compare below stays register-
    // vs-register — no serial arithmetic ahead of the front-end comparators.
    wire [15:0] low_th;
    // row_start pulses on the first tick of each NMS row m = oy (oy = Y-4, so the
    // real rows are Y = 4 .. height+3); fold_en subsamples the centre row (row
    // oy = pc_right) every 4th column, matching the software's power[x] for x%4==0.
    reg  [XW-1:0] y_prev;
    wire row_start = (Y != y_prev) && (Y >= 4) && (Y < height + 4);
    wire fold_en   = in_col && (ix[1:0] == 2'b00) && (Y >= 4) && (Y < height + 4);

    hyst_hist u_hist (
        .clk(clk), .rst(rst), .frame_start(frame_start), .en(en),
        .row_start(row_start), .fold_en(fold_en), .i_pow(pc_right),
        .hyst_on(hyst_on), .adaptive(hyst_adaptive),
        .hyst_low(hyst_low), .high(power_th),
        .o_low_th(low_th)
    );

    // c >= Pm + strict, written as two PARALLEL comparators muxed by strict
    // (c >= Pm+1 == c > Pm for 16-bit unsigned). A literal `Pm + strict`
    // adder would sit in SERIES before the compare, inside the front-end's
    // critical chain (e_bit -> feature window -> endpoint adder tree -> event
    // FIFO) — that broke 74.25 MHz timing on the v2c build. With strict tied
    // constant at the top, synthesis folds the mux to a single comparator,
    // the same depth as the baseline.
    wire ge_pm = strict ? (pc_c > Pm) : (pc_c >= Pm);
    wire e_bit = (pc_c >= low_th) && ge_pm && (pc_c >= Pp);

    // Border edge exclusion (Params::edge_border_margin; kernels::zeroEdgeBorderRow):
    // the outer ring is not a valid edge. border_zone is a pure function of the
    // OUTPUT position (ox, oy) and width/height — it depends on none of the NMS
    // datapath, so it is ready long before e_bit and only adds a single AND at
    // the output (it must NOT go in series inside e_bit; see the strict-adder
    // timing note above). Zeroing the edge here (not later) is what makes the
    // downstream 5x5 feature window of the pixels JUST inside the border see the
    // ring cells as zero — matching the software bit-for-bit.
    wire [XW-1:0] eb = {{(XW-4){1'b0}}, edge_border};
    wire border_zone = (edge_border != 4'd0) &&
                       ((ox < eb) || (ox >= width - eb) ||
                        (oy < eb) || (oy >= height - eb));

    // position-only, not en-gated (see stage_gauss.v)
    assign o_valid = (X >= 4) && (Y >= 4) && (ox < width) && (oy < height);
    assign o_x = ox;
    assign o_y = oy;
    assign o_e = e_bit && !border_zone;
    assign o_strong = (pc_c >= power_th);

    always @(posedge clk) begin
        if (en) begin
            if (in_col) begin
                lpa[ix[MAXW_LOG2-1:0]] <= lpc_q;   // row shift: pa <- pc
                lpc[ix[MAXW_LOG2-1:0]] <= pw;
                ldir[ix[MAXW_LOG2-1:0]] <= dr;
            end
            lpa_q <= lpa[ix_next[MAXW_LOG2-1:0]];
            lpc_q <= lpc[ix_next[MAXW_LOG2-1:0]];
            ldir_q <= ldir[ix_next[MAXW_LOG2-1:0]];

            pc_m <= pc_c;
            pc_c <= pc_right;
            pa_d <= pa_x;
            pb_d <= pw;
            dir_d <= dir_x;
            y_prev <= Y;
        end
        if (rst) begin
            lpa_q <= 16'd0; lpc_q <= 16'd0; ldir_q <= 1'b0;
            pc_m <= 16'd0; pc_c <= 16'd0; pa_d <= 16'd0; pb_d <= 16'd0;
            dir_d <= 1'b0;
            y_prev <= {XW{1'b0}};
        end
    end

endmodule
