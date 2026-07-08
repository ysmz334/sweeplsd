// SweepLSD portable core: pixel stream in -> segment records out
// (rtl/DESIGN.md). Contains the lockstep walker, the four II=1 front-end
// stages, the event packer, the elastic FIFO and the labelling back-end.
// No vendor primitives — everything infers (BRAM line buffers, one 36x36
// multiplier in the judge unit).
//
// Pixel handshake: assert px_valid with each in-image pixel in raster order;
// px_ready indicates the walker consumed it. The walker advances through its
// flush columns/rows without pixels, so a video source with >= 6 columns of
// horizontal and >= 7 rows of vertical blanking never stalls mid-frame at
// pixel clock. `stall` (FIFO backpressure) pauses the walker: pausable
// sources (test-pattern generators, memory readers) simply honour px_ready;
// free-running video sources need the upstream FIFO sized for their content
// (see boards/atlys).
//
// After the last record of a frame the core emits one record with n == 0 and
// returns to idle, ready for the next frame (frame_start restarts the
// walker and re-initialises the back-end tables).
//
// `en` is a global clock enable: every sequential element in the core (and
// the shared judge multiplier) advances only on en cycles, so en = 1 is the
// original full-rate behaviour and a 1-in-N en gives an exact 1/N-rate "time
// dilation" of the same computation (used by boards whose pixel clock is
// faster than the front-end's combinational chain — see boards/atlys v2a,
// which pairs a divide-by-2 en with a x2 multicycle constraint). frame_start
// and rst are sampled every clock, independent of en; rec_valid is masked
// with en so it is a single-clock pulse in either mode.

module sweep_core #(
    parameter XW = 12
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            en,            // global clock enable (1 = full rate)
    input  wire            drop_mode,     // 1 = live source: never stall the
                                          //   walker; overload drops data
                                          //   events instead (event_fifo.v)
    output wire            ev_dropped,    // pulse per discarded event

    input  wire            frame_start,   // pulse: begin a frame
    input  wire [XW-1:0]   width,
    input  wire [XW-1:0]   height,
    input  wire [15:0]     power_th,
    input  wire            strict,        // improved-mode NMS tie-break (v2c a)
    input  wire            hyst_on,       // (d) hysteresis
    input  wire            hyst_adaptive,
    input  wire [15:0]     hyst_low,
    input  wire [17:0]     hyst_strong_min,
    input  wire [17:0]     pix_th,
    input  wire [3:0]      border,        // (i) border margin (0 = off)
    input  wire [4:0]      mps_2sq,       // (h) 2*max_perp_spread^2 (0 = off)

    input  wire            px_valid,
    input  wire [7:0]      px,
    output wire            px_ready,
    output wire [XW-1:0]   px_x,          // walker position: which pixel is
    output wire [XW-1:0]   px_y,          //   being requested (function sources)
    output wire [XW-1:0]   px_addr_x,     // position whose pixel must be on px
    output wire [XW-1:0]   px_addr_y,     //   NEXT cycle (registered sources)
    output wire            busy,          // frame in flight

    output wire            rec_valid,
    output wire [10:0]     rec_sx, rec_sy, rec_ex, rec_ey,
    output wire [17:0]     rec_n,         // 0 = end of frame
    output wire [29:0]     rec_xs, rec_ys,
    output wire [40:0]     rec_xss, rec_yss, rec_xys,
    // (f) bbox extreme points of the label (see backend.v)
    output wire [10:0]     rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
    output wire [10:0]     rec_miny, rec_miny_x, rec_maxy, rec_maxy_x
);

    // ---- walker -------------------------------------------------------------
    reg  [XW-1:0] X, Y;
    reg           running;
    wire          stall;
    wire in_img = (X < width) && (Y < height);
    wire step_ok = en && running && !stall && (!in_img || px_valid);  // = en && step_pred
    wire [XW-1:0] x_next = (X == width + 5) ? {XW{1'b0}} : X + 1'b1;
    wire last_pos = (X == width + 5) && (Y == height + 6);

    assign px_ready = en && running && !stall && in_img;
    assign px_x = X;
    assign px_y = Y;
    // One-cycle-ahead pixel address for registered function sources (e.g. a
    // pattern generator): where the walker will sit next cycle if it steps,
    // or where it stays if it doesn't — either way, pattern(px_addr_*)
    // registered once equals the pixel needed at the walker's next position.
    // The prediction deliberately omits `en`: the source register must be
    // en-gated too (see pattern_gen.v), so only en-cycle values are latched,
    // and keeping the every-cycle enable out of the address avoids a
    // single-cycle path through the source's combinational logic (v2a).
    wire step_pred = running && !stall && (!in_img || px_valid);
    assign px_addr_x = !running ? {XW{1'b0}} : (step_pred ? x_next : X);
    assign px_addr_y = !running ? {XW{1'b0}}
                                : ((step_pred && X == width + 5) ? (Y + 1'b1) : Y);
    assign busy = running;

    always @(posedge clk) begin
        if (frame_start) begin
            X <= {XW{1'b0}};
            Y <= {XW{1'b0}};
            running <= 1'b1;
        end else if (step_ok) begin
            if (X == width + 5) begin
                X <= {XW{1'b0}};
                Y <= Y + 1'b1;
            end else begin
                X <= X + 1'b1;
            end
            if (last_pos) running <= 1'b0;
        end
        if (rst) running <= 1'b0;
    end

    wire fe_en = step_ok;

    // ---- front-end chain (pipelined; see fe_chain.v) ------------------------------
    wire ev_v; wire [1:0] ev_k; wire [XW-1:0] ev_x; wire ev_strong;
    fe_chain #(.MAXW_LOG2(11), .XW(XW)) u_fe (
        .clk(clk), .rst(rst), .ev_rst(rst || frame_start),
        .frame_start(frame_start), .en(fe_en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(width), .height(height),
        .power_th(power_th), .strict(strict),
        .hyst_on(hyst_on), .hyst_adaptive(hyst_adaptive), .hyst_low(hyst_low),
        .px(px),
        .ev_valid(ev_v), .ev_kind(ev_k), .ev_x(ev_x), .ev_strong(ev_strong),
        .f_valid(), .f_x(), .f_y(), .f_code());

    // ---- elastic buffer ------------------------------------------------------------
    wire fifo_empty, fifo_pop;
    wire [14:0] fifo_front;
    event_fifo #(.DW(15), .AW(11)) u_fifo (
        .clk(clk), .rst(rst || frame_start), .en(en),
        .drop_mode(drop_mode), .dropped(ev_dropped),
        .push(ev_v), .wdata({ev_strong, ev_k, ev_x}), .stall(stall),
        .empty(fifo_empty), .front(fifo_front), .pop(fifo_pop));

    // ---- labelling back-end ------------------------------------------------------------
    // rec_valid is held across the en-off cycles by the back-end register;
    // masking with en turns it into a one-clk pulse for full-rate consumers.
    wire be_rec_valid;
    assign rec_valid = be_rec_valid && en;
    backend #(.XW(XW)) u_be (
        .clk(clk), .rst(rst || frame_start), .en(en),
        .width(width), .height(height), .pix_th(pix_th),
        .hyst_on(hyst_on), .hyst_strong_min(hyst_strong_min),
        .border(border), .mps_2sq(mps_2sq),
        .ev_empty(fifo_empty), .ev_kind(fifo_front[13:12]),
        .ev_x(fifo_front[11:0]), .ev_strong(fifo_front[14]), .ev_pop(fifo_pop),
        .rec_valid(be_rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n), .rec_xs(rec_xs), .rec_ys(rec_ys),
        .rec_xss(rec_xss), .rec_yss(rec_yss), .rec_xys(rec_xys),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x)
    );

endmodule
