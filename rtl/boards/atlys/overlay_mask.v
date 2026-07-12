// Segment overlay: half-resolution 1-bit ping-pong masks in BRAM plus a
// Bresenham drawer (rtl/DESIGN.md). Records of frame N are drawn (at raw
// integer endpoint precision, halved) into the back mask while the display
// reads the front mask; the buffers swap at the first display start-of-frame
// after the detector's end-of-frame record, then the new back mask is
// cleared before drawing resumes. No external memory.

module overlay_mask #(
    parameter HW = 640,               // mask width  (display/2)
    parameter VH = 360                // mask height
) (
    input  wire        clk,
    input  wire        rst,

    // record stream from sweep_core (n == 0 marks end of frame)
    input  wire        res_shift,     // 0: coords/2 (<=1280 wide), 1: /4 (FullHD)
    input  wire        rec_valid,
    input  wire        rec_last,      // rec_n == 0
    input  wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey,

    // display query (same clock); ov valid 2 cycles after (dx, dy)
    input  wire        sof,           // display start-of-frame (commit swap)
    input  wire [11:0] dx,
    input  wire [11:0] dy,
    output reg         ov
);

    localparam integer NB = HW * VH;
    localparam integer AW = 18;       // >= log2(640*360)

    reg mask0 [0:NB-1];
    reg mask1 [0:NB-1];
    reg front_sel;                    // which mask the display reads

    // ---- record FIFO (absorbs records while the drawer clears) --------------
    // 1024 deep (was 256): records arrive in ROW ORDER, so an overflow drops
    // the LATEST records = the BOTTOM segments — and the frame-end TERM
    // sweep resolves every still-open label in one burst (hundreds of
    // records in a few hundred cycles) exactly when the queue is least
    // empty. 256 measurably truncated dense frames from the bottom; 1024
    // covers the corpus-worst pass (2,617 records total, TERM burst < 1k)
    // at the cost of one more BRAM.
    reg [43:0] rfifo [0:1023];
    reg [10:0] rwp, rrp;
    wire rf_empty = (rwp == rrp);
    wire [10:0] rf_count = rwp - rrp;
    always @(posedge clk) begin
        if (rec_valid && !rec_last && rf_count < 11'd1016) begin
            rfifo[rwp[9:0]] <= {rec_sx, rec_sy, rec_ex, rec_ey};
            rwp <= rwp + 11'd1;
        end
        if (rst) rwp <= 11'd0;
    end

    // ---- swap control ----------------------------------------------------------
    reg swap_pending;
    wire do_swap = sof && swap_pending;

    // ---- drawer FSM: clear back mask, then draw queued segments ----------------
    localparam [1:0] D_CLEAR = 2'd0, D_IDLE = 2'd1, D_SETUP = 2'd2, D_STEP = 2'd3;
    reg [1:0] dstate;
    reg [AW-1:0] clr_a;
    // Bresenham state (halved coordinates)
    reg [9:0] bx, by, bx1, by1;
    reg signed [11:0] berr;
    reg [9:0] adx, ady_a;
    reg sxp, syp;
    reg [43:0] rec_cur;

    // (j) half-pixel lattice shift (v2c): the 2x2 gradient samples at pixel
    // CORNERS, so record coordinates sit 0.5 px up-left of the true edge
    // (baseline and improved alike — same correction canonical LSD applies).
    // Draw each endpoint at the mask cell nearest the TRUE position:
    // round((c+0.5)/2) = (c+1)>>1, round((c+0.5)/4) = (c+2)>>2, clamped to
    // the last visible cell (the rounding can overshoot it by one at the
    // right/bottom edge, which at /2 scale would wrap into the next row).
    function [9:0] cmap;
        input [10:0] c;       // raw record coordinate
        input        rs;      // res_shift
        input [9:0]  lim;     // last visible mask cell
        reg   [11:0] a;
        reg   [9:0]  s;
        begin
            a = {1'b0, c} + (rs ? 12'd2 : 12'd1);
            s = rs ? a[11:2] : a[11:1];
            cmap = (s > lim) ? lim : s;
        end
    endfunction
    // At /4 scale only the 480x270 corner is visible (FullHD display / 4).
    wire [9:0] lim_x = res_shift ? 10'd479 : HW[9:0] - 10'd1;
    wire [9:0] lim_y = res_shift ? 10'd269 : VH[9:0] - 10'd1;
    wire [9:0] h_sx = cmap(rec_cur[43:33], res_shift, lim_x);
    wire [9:0] h_sy = cmap(rec_cur[32:22], res_shift, lim_y);
    wire [9:0] h_ex = cmap(rec_cur[21:11], res_shift, lim_x);
    wire [9:0] h_ey = cmap(rec_cur[10:0],  res_shift, lim_y);

    // drawer write port (into the BACK mask)
    wire dw_en = (dstate == D_CLEAR) || (dstate == D_STEP);
    wire [AW-1:0] dw_addr = (dstate == D_CLEAR) ? clr_a
                                                : (by * HW[9:0] + {8'd0, bx});
    wire dw_data = (dstate != D_CLEAR);

    always @(posedge clk) begin
        if (do_swap) begin
            front_sel <= ~front_sel;
            swap_pending <= 1'b0;
            clr_a <= {AW{1'b0}};
            dstate <= D_CLEAR;
        end else begin
            case (dstate)
                D_CLEAR: begin
                    clr_a <= clr_a + 1'b1;
                    if (clr_a == NB - 1) dstate <= D_IDLE;
                end
                // Drain even while swap_pending: between the end-record and
                // the sof swap the detector is idle (frame_start fires on the
                // same vsync edge as sof), so everything queued belongs to
                // the CURRENT back mask — the old !swap_pending gate just
                // left the backlog undrawn until after the swap, smearing it
                // one frame late into the wrong mask.
                D_IDLE: if (!rf_empty) begin
                    rec_cur <= rfifo[rrp[9:0]];
                    rrp <= rrp + 11'd1;
                    dstate <= D_SETUP;
                end
                D_SETUP: begin
                    bx <= h_sx;
                    by <= h_sy;
                    bx1 <= h_ex;
                    by1 <= h_ey;
                    adx <= (h_ex > h_sx) ? (h_ex - h_sx) : (h_sx - h_ex);
                    ady_a <= (h_ey > h_sy) ? (h_ey - h_sy) : (h_sy - h_ey);
                    sxp <= (h_ex > h_sx);
                    syp <= (h_ey > h_sy);
                    berr <= $signed({2'd0, (h_ex > h_sx) ? (h_ex - h_sx) : (h_sx - h_ex)})
                          - $signed({2'd0, (h_ey > h_sy) ? (h_ey - h_sy) : (h_sy - h_ey)});
                    dstate <= D_STEP;
                end
                D_STEP: begin
                    // plot happens via dw_en this cycle; then advance
                    // (textbook integer Bresenham: e2 = 2*err; both axes may
                    // step in the same iteration on diagonals)
                    if (bx == bx1 && by == by1) begin
                        dstate <= D_IDLE;
                    end else begin
                        berr <= berr
                              + (((berr <<< 1) >= -$signed({2'd0, ady_a})) ?
                                 -$signed({2'd0, ady_a}) : 12'sd0)
                              + (((berr <<< 1) <= $signed({2'd0, adx})) ?
                                 $signed({2'd0, adx}) : 12'sd0);
                        if ((berr <<< 1) >= -$signed({2'd0, ady_a}))
                            bx <= sxp ? bx + 10'd1 : bx - 10'd1;
                        if ((berr <<< 1) <= $signed({2'd0, adx}))
                            by <= syp ? by + 10'd1 : by - 10'd1;
                    end
                end
            endcase
        end
        if (rec_valid && rec_last) swap_pending <= 1'b1;
        if (rst) begin
            dstate <= D_IDLE;
            swap_pending <= 1'b0;
            front_sel <= 1'b0;
            rrp <= 11'd0;
        end
    end

    // ---- mask memories: drawer writes back, display reads front -----------------
    // At /4 scale (FullHD) only the 480x270 corner of the mask is used; the
    // clear sweep always covers the whole buffer, so leftovers cannot show.
    wire [9:0] rdx = res_shift ? {1'd0, dx[10:2]} : dx[10:1];
    wire [8:0] rdy = res_shift ? dy[10:2] : dy[9:1];
    wire [AW-1:0] rd_addr = rdy * HW[9:0] + {8'd0, rdx};
    reg q0, q1;
    always @(posedge clk) begin
        if (dw_en && front_sel) mask0[dw_addr] <= dw_data;       // back = 0
        if (dw_en && !front_sel) mask1[dw_addr] <= dw_data;      // back = 1
        q0 <= mask0[rd_addr];
        q1 <= mask1[rd_addr];
        ov <= front_sel ? q1 : q0;
    end

endmodule
