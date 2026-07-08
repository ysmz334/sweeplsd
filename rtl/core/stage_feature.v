// Stage 2 — endpoint-candidate classification over a 5x5 edge window
// (thesis §3.2.2). Port of hls/src/frontend.cpp hlsFeature; bit-exact against
// sweeplsd::extractEndpointCandidates (rtl/tb/tb_feature.v).
//
// Consumes the edge bit of (X-4, Y-4); emits the feature code of (X-6, Y-6):
// 0 = none, 1 = interior, 2 = endpoint candidate. Four 1-bit line buffers
// hold edge rows iy-4 .. iy-1; the 5x5 window is a shift-register bank whose
// out-of-image taps are forced to zero by position gating (columns via
// `in_col`, top rows via the Y gates below; bottom/right wash through as
// zeros because writes continue during the flush region).

module stage_feature #(
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

    input  wire            i_valid,     // edge bit (X-4, Y-4) valid
    input  wire            i_e,
    input  wire            i_strong,    // (d) strong bit of the same pixel

    output wire            o_valid,     // feature code (o_x, o_y) this tick
    output wire [XW-1:0]   o_x,
    output wire [XW-1:0]   o_y,
    output wire [1:0]      o_f,
    output wire            o_strong     // (d) strong bit of (o_x, o_y)
);

    wire [XW-1:0] ix = X - 4;
    wire [XW-1:0] ix_next = x_next - 4;
    wire in_col = (X >= 4) && (ix < width);

    // Edge-row line buffers: lb3 = row iy-1 ... lb0 = row iy-4.
    reg lb0 [0:(1<<MAXW_LOG2)-1];
    reg lb1 [0:(1<<MAXW_LOG2)-1];
    reg lb2 [0:(1<<MAXW_LOG2)-1];
    reg lb3 [0:(1<<MAXW_LOG2)-1];
    reg lb0_q, lb1_q, lb2_q, lb3_q;

    wire e_in = (i_valid && in_col) ? i_e : 1'b0;

    // (d) strong bit rides a parallel 2-buffer + shift-register network that is
    // an exact copy of the edge network's path to the WINDOW CENTRE (row oy,
    // column ox), so o_strong is the strong bit of (o_x, o_y). Only the centre
    // row is reconstructed (lb3s feeds lb2s feeds t2s feeds win_s_r2); the rows
    // below/above the centre are irrelevant to the centre tap.
    reg s_lb2 [0:(1<<MAXW_LOG2)-1];
    reg s_lb3 [0:(1<<MAXW_LOG2)-1];
    reg s_lb2_q, s_lb3_q;
    wire s_in = (i_valid && in_col) ? i_strong : 1'b0;

    // Column taps for rows iy-4 .. iy at column ix. Top-of-image gating: the
    // tap row must be >= 0 (first frame the buffers hold junk above row 0).
    wire t0 = (in_col && Y >= 8) ? lb0_q : 1'b0;   // row oy-2
    wire t1 = (in_col && Y >= 7) ? lb1_q : 1'b0;   // row oy-1
    wire t2 = (in_col && Y >= 6) ? lb2_q : 1'b0;   // row oy
    wire t3 = (in_col && Y >= 5) ? lb3_q : 1'b0;   // row oy+1
    wire t4 = e_in;                                // row oy+2

    // 5x5 window. The combinational view cw_r* includes this tick's taps:
    // bit [4] = column ix (newest) ... bit [0] = column ix-4, so bit [2] is
    // the output column ox = ix-2 and bit [c] is column offset c-2.
    reg [4:0] win_r0, win_r1, win_r2, win_r3, win_r4;
    wire [4:0] cw_r0 = {t0, win_r0[4:1]};
    wire [4:0] cw_r1 = {t1, win_r1[4:1]};
    wire [4:0] cw_r2 = {t2, win_r2[4:1]};
    wire [4:0] cw_r3 = {t3, win_r3[4:1]};
    wire [4:0] cw_r4 = {t4, win_r4[4:1]};

    // (d) centre-row strong shift register, mirroring win_r2 / cw_r2 exactly.
    reg  [4:0] win_s_r2;
    wire       t2s = (in_col && Y >= 6) ? s_lb2_q : 1'b0;   // strong, row oy
    wire [4:0] cw_s_r2 = {t2s, win_s_r2[4:1]};

    wire is_end;
    endpoint_core core (
        .r0(cw_r0), .r1(cw_r1), .r2(cw_r2), .r3(cw_r3), .r4(cw_r4),
        .is_end(is_end)
    );

    wire centre = cw_r2[2];
    wire [XW-1:0] ox = X - 6;
    wire [XW-1:0] oy = Y - 6;

    // position-only, not en-gated (see stage_gauss.v)
    assign o_valid = (X >= 6) && (Y >= 6) && (ox < width) && (oy < height);
    assign o_x = ox;
    assign o_y = oy;
    assign o_f = centre ? (is_end ? 2'd2 : 2'd1) : 2'd0;
    assign o_strong = cw_s_r2[2];

    always @(posedge clk) begin
        if (en) begin
            if (in_col) begin
                lb0[ix[MAXW_LOG2-1:0]] <= lb1_q;
                lb1[ix[MAXW_LOG2-1:0]] <= lb2_q;
                lb2[ix[MAXW_LOG2-1:0]] <= lb3_q;
                lb3[ix[MAXW_LOG2-1:0]] <= e_in;
                s_lb2[ix[MAXW_LOG2-1:0]] <= s_lb3_q;   // (d) strong: oy <- oy+1
                s_lb3[ix[MAXW_LOG2-1:0]] <= s_in;      //             oy+1 <- in
            end
            lb0_q <= lb0[ix_next[MAXW_LOG2-1:0]];
            lb1_q <= lb1[ix_next[MAXW_LOG2-1:0]];
            lb2_q <= lb2[ix_next[MAXW_LOG2-1:0]];
            lb3_q <= lb3[ix_next[MAXW_LOG2-1:0]];
            s_lb2_q <= s_lb2[ix_next[MAXW_LOG2-1:0]];
            s_lb3_q <= s_lb3[ix_next[MAXW_LOG2-1:0]];

            win_r0 <= cw_r0;
            win_r1 <= cw_r1;
            win_r2 <= cw_r2;
            win_r3 <= cw_r3;
            win_r4 <= cw_r4;
            win_s_r2 <= cw_s_r2;
        end
        if (rst) begin
            lb0_q <= 1'b0; lb1_q <= 1'b0; lb2_q <= 1'b0; lb3_q <= 1'b0;
            win_r0 <= 5'd0; win_r1 <= 5'd0; win_r2 <= 5'd0;
            win_r3 <= 5'd0; win_r4 <= 5'd0;
            s_lb2_q <= 1'b0; s_lb3_q <= 1'b0; win_s_r2 <= 5'd0;
        end
    end

endmodule
