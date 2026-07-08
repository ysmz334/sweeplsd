// SweepLSD live demo, phase-2 v2a (rtl/DESIGN.md): an internally generated
// 1280x720 test scene is displayed over HDMI OUT (J2) while a second instance
// of the same scene function feeds the detector core; the segments found by
// detector pass N are overlaid (green, half-res mask) on the following
// frames. No external memory anywhere — detector state and overlay masks
// live entirely in BRAM.
//
//   v2a timing closure: the pixel clock is 75.0 MHz (750 MHz VCO / 10 —
//   1650x750 total => 60.6 Hz vertical, +1 % over CEA 720p60, which displays
//   accept; exact 74.25 MHz is not reachable from the 100 MHz oscillator in
//   one PLL). The lockstep front-end computes gauss->gradient->NMS->edge
//   combinationally between line-buffer BRAMs (~25 ns), which no longer fits
//   one 13.3 ns pixel period, so the detector core (plus its scene-function
//   instance) runs on a dedicated HALF-RATE clock clk_core = 37.5 MHz
//   (CLKOUT2 of the same PLL, phase-aligned 2:1) — the auto-derived 26.6 ns
//   period constraint then covers every core path with no hand-written
//   multicycle groups. (A clock-enable + FROM:TO multicycle variant was
//   tried first and verified in simulation — the core's `en` port remains —
//   but ngdbuild silently dropped part of the wildcard TNM group (gauss/grad
//   line-buffer BRAMs, ~300 backend FFs), so the constraint could not be
//   trusted; a real clock cannot be mis-grouped.) The pix<->core crossings
//   (sof handshake in, records out) are synchronous 2:1 paths, fully checked
//   by static timing. A detector pass takes ~25 ms (1.5 frames): segment
//   updates land every 2 frames (~30 Hz) while the display always runs at
//   full rate. The serdes DDR clock rises to 375 MHz (750 Mbps TMDS) — same
//   ODDR2 scheme, just faster.
//   LEDs: segment count of the last detector pass (saturating 8 bits).

module atlys_top (
    input  wire       clk100,
    output wire [7:0] led,
    output wire       tmds_tx_clk_p,
    output wire       tmds_tx_clk_n,
    output wire [2:0] tmds_tx_p,
    output wire [2:0] tmds_tx_n
);

    // ---- clocking ------------------------------------------------------------
    wire clk100_b, pll_fb, pix_u, x5_u, core_u, clk_pix, clk_x5, clk_core, locked;
    IBUFG ibuf_clk (.I(clk100), .O(clk100_b));
    PLL_BASE #(
        .CLKIN_PERIOD(10.0),
        .DIVCLK_DIVIDE(2),        // PFD 50 MHz
        .CLKFBOUT_MULT(15),       // VCO 750 MHz
        .CLKOUT0_DIVIDE(2),       // 375 MHz serdes (DDR, 750 Mbps TMDS)
        .CLKOUT1_DIVIDE(10),      // 75 MHz pixel (1280x720@60.6)
        .CLKOUT2_DIVIDE(20),      // 37.5 MHz detector core (phase-aligned 1:2)
        .CLK_FEEDBACK("CLKFBOUT")
    ) pll (
        .CLKIN(clk100_b),
        .CLKFBIN(pll_fb),
        .CLKFBOUT(pll_fb),
        .CLKOUT0(x5_u),
        .CLKOUT1(pix_u),
        .CLKOUT2(core_u),
        .CLKOUT3(), .CLKOUT4(), .CLKOUT5(),
        .LOCKED(locked),
        .RST(1'b0)
    );
    BUFG bufg_x5 (.I(x5_u), .O(clk_x5));
    BUFG bufg_px (.I(pix_u), .O(clk_pix));
    BUFG bufg_co (.I(core_u), .O(clk_core));

    // power-on reset, held until the PLL locks (one per domain)
    reg [7:0] por = 8'hFF;
    always @(posedge clk_pix) por <= locked ? {por[6:0], 1'b0} : 8'hFF;
    wire rst = por[7];
    reg [7:0] por_c = 8'hFF;
    always @(posedge clk_core) por_c <= locked ? {por_c[6:0], 1'b0} : 8'hFF;
    wire rst_core = por_c[7];

    // ---- display timing + scene ------------------------------------------------
    wire [11:0] vx, vy;
    wire de0, hs0, vs0, sof;
    vtgen #(
        .H_ACTIVE(1280), .H_FP(110), .H_SYNC(40), .H_BP(220),
        .V_ACTIVE(720), .V_FP(5), .V_SYNC(5), .V_BP(20),
        .H_POL(1), .V_POL(1)                    // 720p: positive syncs
    ) u_vt (.clk(clk_pix), .rst(rst), .x(vx), .y(vy),
            .de(de0), .hs(hs0), .vs(vs0), .sof(sof));

    wire [7:0] gray_d;                    // valid 2 cycles after (vx, vy)
    pattern_gen #(.REGISTERED(2)) u_pat_d (.clk(clk_pix), .en(1'b1),
                                           .x(vx), .y(vy), .gray(gray_d));

    // ---- detector (clk_core domain, 37.5 MHz) -------------------------------------
    // The scene function is registered (timing) and addressed one cycle ahead
    // via the walker's px_addr_* prefetch interface. Everything below up to
    // the record outputs runs on clk_core; en is tied 1 (the core's clock
    // enable is for CE-style integrations — here the half rate IS the clock).
    wire [11:0] px_ax, px_ay;
    wire [7:0] det_px;
    pattern_gen #(.REGISTERED(1)) u_pat_c (.clk(clk_core), .en(1'b1),
                                           .x(px_ax), .y(px_ay), .gray(det_px));

    wire core_busy, rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;

    // Start handshake pix -> core: stretch the 1-pix-clk sof pulse to 2
    // cycles, then rising-edge-detect it in the core domain (the clocks are
    // phase-aligned 2:1 from one PLL, so these are ordinary synchronous
    // paths, fully covered by static timing).
    reg sof_d;
    always @(posedge clk_pix) sof_d <= sof && !rst;
    wire sof2 = sof || sof_d;

    reg sof2_c, frame_start;
    always @(posedge clk_core) begin
        sof2_c <= sof2 && !rst_core;
        frame_start <= sof2 && !sof2_c && !core_busy && !rst_core;
    end

    sweep_core #(.XW(12)) u_core (
        .clk(clk_core), .rst(rst_core), .en(1'b1),
        .drop_mode(1'b0), .ev_dropped(),      // pausable source: lossless stall
        .frame_start(frame_start),
        .width(12'd1280), .height(12'd720),
        .power_th(16'd256), .strict(1'b0), .pix_th(18'd15),  // v2a stays baseline
        .hyst_on(1'b0), .hyst_adaptive(1'b0),                // (d) hysteresis off
        .hyst_low(16'd120), .hyst_strong_min(18'd3),
        .px_valid(1'b1), .px(det_px), .px_ready(),
        .px_x(), .px_y(),
        .px_addr_x(px_ax), .px_addr_y(px_ay), .busy(core_busy),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n), .rec_xs(rec_xs), .rec_ys(rec_ys),
        .rec_xss(rec_xss), .rec_yss(rec_yss), .rec_xys(rec_xys)
    );

    // ---- overlay -------------------------------------------------------------------
    // Record crossing core -> pix: rec_valid is one core cycle (= 2 pix
    // cycles) high; a pix-domain rising-edge detect turns it into a single
    // strobe (records are >= several core cycles apart, so none can be
    // missed or double-counted). rec_* data stays stable both pix cycles.
    reg rec_v_d;
    always @(posedge clk_pix) rec_v_d <= rec_valid && !rst;
    wire rec_stb = rec_valid && !rec_v_d;

    wire ov;                              // valid 2 cycles after (vx, vy)
    overlay_mask #(.HW(640), .VH(360)) u_ov (
        .clk(clk_pix), .rst(rst), .res_shift(1'b0),
        .rec_valid(rec_stb), .rec_last(rec_n == 18'd0),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .sof(sof), .dx(vx), .dy(vy), .ov(ov)
    );

    // ---- video pipeline alignment (2 stages) -------------------------------------
    // gray_d (2-stage pattern pipeline) and ov (overlay read + register) are
    // both already at stage 2; only de/hs/vs need delaying to match.
    reg de1, hs1, vs1, de2, hs2, vs2;
    always @(posedge clk_pix) begin
        de1 <= de0; hs1 <= hs0; vs1 <= vs0;
        de2 <= de1; hs2 <= hs1; vs2 <= vs1;
    end

    wire [7:0] r_out = ov ? 8'd32 : gray_d;
    wire [7:0] g_out = ov ? 8'd255 : gray_d;
    wire [7:0] b_out = ov ? 8'd32 : gray_d;

    dvid_out u_dvi (
        .clk_pix(clk_pix), .clk_x5(clk_x5), .rst(rst),
        .red(r_out), .green(g_out), .blue(b_out),
        .de(de2), .hs(hs2), .vs(vs2),
        .tmds_tx_clk_p(tmds_tx_clk_p), .tmds_tx_clk_n(tmds_tx_clk_n),
        .tmds_tx_p(tmds_tx_p), .tmds_tx_n(tmds_tx_n)
    );

    // ---- status LEDs: last frame's segment count (saturating) ---------------------
    reg [7:0] seg_cnt, seg_led;
    always @(posedge clk_pix) begin
        if (rec_stb) begin
            if (rec_n == 18'd0) begin
                seg_led <= seg_cnt;
                seg_cnt <= 8'd0;
            end else if (seg_cnt != 8'hFF) begin
                seg_cnt <= seg_cnt + 8'd1;
            end
        end
        if (rst) begin
            seg_cnt <= 8'd0;
            seg_led <= 8'd0;
        end
    end
    assign led = seg_led;

endmodule
