// SweepLSD phase-2 v2b milestone 3: LIVE HDMI line-segment overlay.
// (Milestone 1 pass-through skeleton + rgb2gray -> live_core(sweep_core) ->
// overlay, everything in the single recovered-pixel-clock domain — both the
// pipelined front-end (88.5 MHz) and the back-end (91.7 MHz) close timing at
// 74.25 MHz, so en = 1 and there is no clock crossing anywhere.)
// J3 (HDMI IN, unbuffered — no JP5 needed) -> XAPP495 dvi_decoder -> XAPP495
// dvi_encoder_top -> J2 (HDMI OUT). The FPGA serves EDID on J3's DDC lines
// (M16/M18) from edid_rom_720p.vhd — preferred timing 1280x720@60, DVI-only
// (no CEA block) so sources stay in DVI signalling, which is what the
// XAPP495 decoder understands. Lesson from the first M1 attempt: WITHOUT the
// FPGA EDID the source fails to detect a sink — under dvi_demo (M0) the
// source was in fact reading the FPGA's ROM, not the JP6/JP7-bridged monitor
// EEPROM — so the ROM must stay. The pass-through itself is
// resolution-agnostic within the RX PLL range (pclk ~40-80 MHz).
//
// Everything runs on the RECOVERED clock — no frame buffer, no on-board
// oscillator involvement (clk100 unused). This is the skeleton that v2b
// milestone 3 extends with rgb2gray + sweep_core + overlay between the
// decoder's RGB and the encoder's inputs.
//
// LEDs: 7 = RX PLL locked (TMDS clock present)
//       6/5/4 = red/green/blue channel phase-aligned (all on = good signal)
//       5 = DE seen, 4 = misalignment latch (should stay OFF)
//       3 = event-drop latch (content denser than the labelling engine —
//           overlay thins locally, video unaffected), 2..0 = segment count

module atlys_rx_top (
    input  wire       clk100,         // on-board 100 MHz (EDID I2C sampling)
    input  wire       rstbtn_n,       // red push button (active low)

    input  wire [3:0] rx_tmds_p,      // J3: [3]=clk [2]=red [1]=green [0]=blue
    input  wire [3:0] rx_tmds_n,
    input  wire       rx_scl,         // J3 DDC (M16)
    inout  wire       rx_sda,         // J3 DDC (M18)

    output wire [3:0] tx_tmds_p,      // J2, same channel order
    output wire [3:0] tx_tmds_n,

    output wire [7:0] led,
    output wire       uart_tx         // USB-UART telemetry (diag; B16)
);

    // ---- EDID (I2C slave at 0x50, data in edid_rom_720p.vhd) -------------------
    edid_rom u_edid (
        .clk       (clk100),
        .sclk_raw  (rx_scl),
        .sdat_raw  (rx_sda),
        .edid_debug()
    );

    // ---- HDMI RX (XAPP495) ----------------------------------------------------
    wire rx_reset, rx_pclk, rx_pclkx2, rx_pclkx10;
    wire rx_pllclk0, rx_pllclk1, rx_pllclk2, rx_pll_lckd;
    wire rx_tmdsclk, rx_serdesstrobe;
    wire rx_hsync, rx_vsync, rx_de, rx_psalgnerr;
    wire [7:0] rx_red, rx_green, rx_blue;
    wire [29:0] rx_sdata;
    wire rx_blue_vld, rx_green_vld, rx_red_vld;
    wire rx_blue_rdy, rx_green_rdy, rx_red_rdy;

    dvi_decoder u_rx (
        .tmdsclk_p   (rx_tmds_p[3]),
        .tmdsclk_n   (rx_tmds_n[3]),
        .blue_p      (rx_tmds_p[0]),
        .green_p     (rx_tmds_p[1]),
        .red_p       (rx_tmds_p[2]),
        .blue_n      (rx_tmds_n[0]),
        .green_n     (rx_tmds_n[1]),
        .red_n       (rx_tmds_n[2]),
        .exrst       (~rstbtn_n),

        .reset       (rx_reset),
        .pclk        (rx_pclk),
        .pclkx2      (rx_pclkx2),
        .pclkx10     (rx_pclkx10),
        .pllclk0     (rx_pllclk0),
        .pllclk1     (rx_pllclk1),
        .pllclk2     (rx_pllclk2),
        .pll_lckd    (rx_pll_lckd),
        .tmdsclk     (rx_tmdsclk),
        .serdesstrobe(rx_serdesstrobe),
        .hsync       (rx_hsync),
        .vsync       (rx_vsync),
        .de          (rx_de),
        .blue_vld    (rx_blue_vld),
        .green_vld   (rx_green_vld),
        .red_vld     (rx_red_vld),
        .blue_rdy    (rx_blue_rdy),
        .green_rdy   (rx_green_rdy),
        .red_rdy     (rx_red_rdy),
        .psalgnerr   (rx_psalgnerr),
        .sdout       (rx_sdata),
        .red         (rx_red),
        .green       (rx_green),
        .blue        (rx_blue)
    );

    // ---- TX clocking: dedicated PLL fed by the recovered pixel clock -----------
    // (dvi_demo's tx0 path, single-input: BUFG instead of the BUFGMUX)
    wire tx_pclk;
    BUFG tx_bufg_pclk (.I(rx_pllclk1), .O(tx_pclk));

    wire tx_clkfbout, tx_clkfbin, tx_plllckd;
    wire tx_pllclk0, tx_pllclk2;
    PLL_BASE #(
        .CLKIN_PERIOD(10),
        .CLKFBOUT_MULT(10),       // VCO = 10x pclk
        .CLKOUT0_DIVIDE(1),       // 10x  (serdes, via BUFPLL)
        .CLKOUT1_DIVIDE(10),      // 1x   (unused)
        .CLKOUT2_DIVIDE(5),       // 2x
        .COMPENSATION("SOURCE_SYNCHRONOUS")
    ) pll_tx (
        .CLKFBOUT(tx_clkfbout),
        .CLKOUT0(tx_pllclk0),
        .CLKOUT1(),
        .CLKOUT2(tx_pllclk2),
        .CLKOUT3(), .CLKOUT4(), .CLKOUT5(),
        .LOCKED(tx_plllckd),
        .CLKFBIN(tx_clkfbin),
        .CLKIN(tx_pclk),
        .RST(rx_reset)
    );
    BUFG tx_clkfb_buf (.I(tx_clkfbout), .O(tx_clkfbin));

    wire tx_pclkx2;
    BUFG tx_pclkx2_buf (.I(tx_pllclk2), .O(tx_pclkx2));

    wire tx_pclkx10, tx_serdesstrobe, tx_bufpll_lock;
    BUFPLL #(.DIVIDE(5)) tx_ioclk_buf (
        .PLLIN(tx_pllclk0), .GCLK(tx_pclkx2), .LOCKED(tx_plllckd),
        .IOCLK(tx_pclkx10), .SERDESSTROBE(tx_serdesstrobe), .LOCK(tx_bufpll_lock));

    wire tx_reset = ~tx_bufpll_lock;

    // ---- detector: rgb2gray -> live_core (single pclk domain, en = 1) -------------
    // BT.601 integer luma, registered once: (77R + 150G + 29B) >> 8.
    reg [7:0] gray;
    reg gray_de;                          // de aligned with gray
    always @(posedge rx_pclk) begin
        gray <= ({8'd0, rx_red} * 8'd77
               + {8'd0, rx_green} * 8'd150
               + {8'd0, rx_blue} * 8'd29) >> 8;
        gray_de <= rx_de && !rx_reset;
    end

    wire        rec_valid, core_busy, drop_latch, evdrop_latch, core_fs;
    wire        res_shift;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;
    wire [5:0]  dbg_be_state;
    wire [2:0]  dbg_be_cond;
    wire        dbg_push, dbg_push_eor, dbg_pop, dbg_jwd_fire, dbg_fl_zero;
    wire        dbg_jstall, dbg_jf, dbg_evdrop;
    wire [15:0] dbg_pth;
    wire [11:0] dbg_use_w, dbg_use_h;
    live_core #(.XW(12)) u_live (
        .clk(rx_pclk), .rst(rx_reset),
        .de(gray_de), .vsync(rx_vsync), .gray(gray),
        .strict(1'b1),                 // v2c (a): strict NMS tie-break on
        .res_shift(res_shift),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x),
        .busy(core_busy), .drop_latch(drop_latch),
        .evdrop_latch(evdrop_latch),
        .frame_start(core_fs),
        .dbg_be_state(dbg_be_state),
        .dbg_be_cond(dbg_be_cond),
        .dbg_push(dbg_push), .dbg_push_eor(dbg_push_eor), .dbg_pop(dbg_pop),
        .dbg_jwd_fire(dbg_jwd_fire),
        .dbg_fl_zero(dbg_fl_zero),
        .dbg_jstall(dbg_jstall),
        .dbg_jf(dbg_jf), .dbg_evdrop(dbg_evdrop),
        .dbg_pth(dbg_pth),
        .dbg_use_w(dbg_use_w), .dbg_use_h(dbg_use_h)
    );

    // v2c (f): draw between the bbox extreme points instead of the first/last
    // endpoint contacts — a label with >2 contacts otherwise renders
    // truncated. The exact choice (projection extremes on the fitted axis)
    // needs floats and lives in host finalisation; the board approximates it
    // by the bbox's dominant span, identical for clean segments.
    wire [10:0] bb_w = rec_maxx - rec_minx;
    wire [10:0] bb_h = rec_maxy - rec_miny;
    wire        bb_horiz = (bb_w >= bb_h);
    wire [10:0] ov_sx = bb_horiz ? rec_minx   : rec_miny_x;
    wire [10:0] ov_sy = bb_horiz ? rec_minx_y : rec_miny;
    wire [10:0] ov_ex = bb_horiz ? rec_maxx   : rec_maxy_x;
    wire [10:0] ov_ey = bb_horiz ? rec_maxx_y : rec_maxy;

    // ---- overlay --------------------------------------------------------------------
    // Display position counters for the pass-through stream (dx, dy valid
    // with rx_de) and a vsync-edge pulse as the mask-swap point.
    reg [11:0] dx, dy;
    reg de_d1, vs_d1;
    wire vs_fall = !rx_vsync && vs_d1;
    always @(posedge rx_pclk) begin
        de_d1 <= rx_de && !rx_reset;
        vs_d1 <= rx_vsync && !rx_reset;
        if (rx_de) dx <= dx + 12'd1;
        else dx <= 12'd0;
        if (de_d1 && !rx_de) dy <= dy + 12'd1;    // end of active line
        if (vs_fall || rx_reset) dy <= 12'd0;
    end

    wire ov;                              // valid 2 cycles after (dx, dy)
    overlay_mask #(.HW(640), .VH(360)) u_ov (
        .clk(rx_pclk), .rst(rx_reset), .res_shift(res_shift),
        .rec_valid(rec_valid), .rec_last(rec_n == 18'd0),
        .rec_sx(ov_sx), .rec_sy(ov_sy), .rec_ex(ov_ex), .rec_ey(ov_ey),
        .sof(vs_fall), .dx(dx), .dy(dy), .ov(ov)
    );

    // Align the pass-through video with ov (2 cycles) and mix the overlay in.
    reg [7:0] r_d1, g_d1, b_d1, r_d2, g_d2, b_d2;
    reg hs_d1, hs_d2, vs_v1, vs_v2, de_v1, de_v2;
    always @(posedge rx_pclk) begin
        r_d1 <= rx_red;  g_d1 <= rx_green;  b_d1 <= rx_blue;
        r_d2 <= r_d1;    g_d2 <= g_d1;      b_d2 <= b_d1;
        hs_d1 <= rx_hsync;  hs_d2 <= hs_d1;
        vs_v1 <= rx_vsync;  vs_v2 <= vs_v1;
        de_v1 <= rx_de;     de_v2 <= de_v1;
    end
    wire [7:0] mix_r = ov ? 8'd32  : r_d2;
    wire [7:0] mix_g = ov ? 8'd255 : g_d2;
    wire [7:0] mix_b = ov ? 8'd32  : b_d2;

    // ---- HDMI TX (XAPP495) ------------------------------------------------------
    dvi_encoder_top u_tx (
        .pclk        (tx_pclk),
        .pclkx2      (tx_pclkx2),
        .pclkx10     (tx_pclkx10),
        .serdesstrobe(tx_serdesstrobe),
        .rstin       (tx_reset),
        .blue_din    (mix_b),
        .green_din   (mix_g),
        .red_din     (mix_r),
        .hsync       (hs_d2),
        .vsync       (vs_v2),
        .de          (de_v2),
        .TMDS        (tx_tmds_p),
        .TMDSB       (tx_tmds_n)
    );

    // ---- status LEDs -------------------------------------------------------------
    reg de_seen;
    reg [7:0] seg_cnt, seg_led;
    always @(posedge rx_pclk) begin
        if (rx_de) de_seen <= 1'b1;
        if (rec_valid) begin
            if (rec_n == 18'd0) begin
                seg_led <= seg_cnt;
                seg_cnt <= 8'd0;
            end else if (seg_cnt != 8'hFF) begin
                seg_cnt <= seg_cnt + 8'd1;
            end
        end
        if (rx_reset) begin
            de_seen <= 1'b0;
            seg_cnt <= 8'd0;
            seg_led <= 8'd0;
        end
    end

    // DIAGNOSTIC LED MAP v6 (temporary):
    //   7 = RX PLL lock
    //   6 = END-record heartbeat (flicker/dim = passes completing)
    //   5 = DEATH latched (a pass stopped popping without its END record;
    //       held until the next END — should now stay OFF)
    //   4 = judge WATCHDOG fired this pass (sticky, cleared at pass start;
    //       ON = the judge is still glitching and the watchdog is papering
    //       over it at 4095 cycles per rescue — must stay OFF)
    //   3 = event-drop latch (density ceiling; thinning, informational)
    //   2..0 = segment count LSBs of the last completed pass
    reg end_hb;
    reg [19:0] act_pop_cnt;
    wire act_pop = (act_pop_cnt != 20'd0);
    reg act_pop_d;
    reg pass_open;          // a pass started and has not emitted its END record
    reg death_latched;
    reg jwd_latch;
    always @(posedge rx_pclk) begin
        act_pop_cnt <= dbg_pop ? 20'hFFFFF :
                       (act_pop_cnt != 20'd0 ? act_pop_cnt - 20'd1 : 20'd0);
        act_pop_d <= act_pop;
        if (core_fs) begin
            pass_open <= 1'b1;
            jwd_latch <= 1'b0;
        end
        if (dbg_jwd_fire) jwd_latch <= 1'b1;
        if (rec_valid && rec_n == 18'd0) begin
            end_hb <= ~end_hb;
            pass_open <= 1'b0;
            death_latched <= 1'b0;     // healthy pass completed: rearm
        end
        if (act_pop_d && !act_pop && pass_open && !death_latched)
            death_latched <= 1'b1;
        if (rx_reset) begin
            end_hb <= 1'b0;
            act_pop_cnt <= 20'd0;
            act_pop_d <= 1'b0;
            pass_open <= 1'b0;
            death_latched <= 1'b0;
            jwd_latch <= 1'b0;
        end
    end
    // v9: judge-loss MAGNITUDE — LED4..0 = watchdog fires PER PASS, latched at
    // each end-record (saturating at 31). A stable readable number:
    //   0        = no losses
    //   1..5     = losses trivial (~0.1% of a frame in stalls) -> the bottom
    //              loss is NOT judge-stall-driven
    //   31 (all) = thousands per frame -> the labeller is stall-starved
    //   7 = lock   6 = END heartbeat   5 = event-drop latch
    // v18: DRAIN-THEFT measurement. v17 proved supply == sim (both 1077 and
    // 1032 land in the exact sim log2 bucket) while the board still wipes
    // the bottom -- the back-end's effective drain must be a fraction of the
    // sim's. Judge stalls are the prime suspect (LED4 of v17: jwd fires on
    // exactly the loss images). This build (a) shortens the watchdog rescue
    // 511->63 cy / 2 tries, (b) counts the cycles the labeller waits on the
    // judge BEYOND legitimate latency (jwd > 16). Latched at each END:
    //   4    = any data event dropped this pass
    //   3..0 = log2(judge-stall cycles this pass) - 8, 0 = < 256 cy
    // Interpretation on IMGP1077: explaining the wipe needs a >= 2^20-cycle
    // theft (code 12+); code <= 9 (< 130k cy, 5%) = stalls do NOT dominate
    // and the drain thief is elsewhere (state corruption class).
    // Per-pass counters (pixel domain) + UART snapshot (see uart_telemetry.v).
    reg [23:0] jscnt, pcnt, dcnt, jdcnt, wcnt, rcnt;
    reg jf_q, drop_pass;
    reg [4:0] diag_disp;
    reg [23:0] sn_push, sn_drop, sn_jdisp, sn_jstall, sn_jwd, sn_rec;
    reg sn_tgl;
    integer pb;
    reg [3:0] pmsb;
    always @(*) begin
        pmsb = 4'd0;
        for (pb = 8; pb <= 23; pb = pb + 1)
            if (jscnt[pb]) pmsb = pb - 8;
    end
    always @(posedge rx_pclk) begin
        jf_q <= dbg_jf;
        if (dbg_jstall && jscnt != 24'hFFFFFF) jscnt <= jscnt + 24'd1;
        if (dbg_push && !dbg_push_eor && pcnt != 24'hFFFFFF) pcnt <= pcnt + 24'd1;
        if (dbg_evdrop && dcnt != 24'hFFFFFF) dcnt <= dcnt + 24'd1;
        if (dbg_jf && !jf_q && jdcnt != 24'hFFFFFF) jdcnt <= jdcnt + 24'd1;
        if (dbg_jwd_fire && wcnt != 24'hFFFFFF) wcnt <= wcnt + 24'd1;
        if (evdrop_latch) drop_pass <= 1'b1;
        if (rec_valid && rec_n != 18'd0 && rcnt != 24'hFFFFFF) rcnt <= rcnt + 24'd1;
        if (rec_valid && rec_n == 18'd0) begin
            diag_disp <= {drop_pass, pmsb};
            sn_push <= pcnt;  sn_drop <= dcnt;  sn_jdisp <= jdcnt;
            sn_jstall <= jscnt;  sn_jwd <= wcnt;  sn_rec <= rcnt;
            sn_tgl <= ~sn_tgl;
            jscnt <= 24'd0;  pcnt <= 24'd0;  dcnt <= 24'd0;
            jdcnt <= 24'd0;  wcnt <= 24'd0;  rcnt <= 24'd0;
            drop_pass <= 1'b0;
        end
        if (rx_reset) begin
            jscnt <= 24'd0;  pcnt <= 24'd0;  dcnt <= 24'd0;
            jdcnt <= 24'd0;  wcnt <= 24'd0;  rcnt <= 24'd0;
            jf_q <= 1'b0;
            drop_pass <= 1'b0;
            diag_disp <= 5'd0;
            sn_tgl <= 1'b0;
        end
    end
    assign led = {rx_pll_lckd, end_hb, drop_latch, diag_disp};

    uart_telemetry u_uart (
        .clk100(clk100), .rst(1'b0),
        .snap_toggle(sn_tgl),
        .c_push(sn_push), .c_drop(sn_drop), .c_jdisp(sn_jdisp),
        .c_jstall(sn_jstall), .c_jwd(sn_jwd), .c_rec(sn_rec),
        .uart_tx(uart_tx)
    );

endmodule
