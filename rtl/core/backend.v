// Event-driven labelling back-end (thesis §3.2.3-3.2.4) — the RTL port of
// hls/src/backend.cpp (itself bit-exact against sweeplsd::detect() over the
// full corpus). Consumes the sparse event stream, maintains the 1024-label
// table with free-list recycling, and emits one segment record per accepted
// segment through judge_unit; a final record with n == 0 terminates the
// stream.
//
// Sequential FSM, ~12-20 cycles per interior-pixel event plus ~110 per
// emitted segment (events are 1-5 % of pixels; the upstream event FIFO makes
// the pipeline elastic). All BRAM reads follow the addr@T -> data-reg@T+1 ->
// consume@T+2 discipline (explicit wait states).
//
// The C model's two label rows collapse to ONE tag-validated row buffer plus
// two carry registers: when the previously processed pixel of this row was
// x-1, its centre label (w_sav) and the previous-row cell it overwrote
// (nw_sav) are taken from the registers — the original reference
// implementation's single-row trick, adapted to the sparse scan. Find uses
// single-hop path compression: find() results equal the golden full
// compression, so the output is unchanged, and the scavenger bound
// (last_row <= y-2 unreachable) is compression-independent.

// `en` (global clock enable, see sweep_core.v) gates EVERY sequential element
// here — the FSM, all funnelled write ports and all BRAM read registers. The
// read registers MUST be gated too: they re-latch each clock, so an ungated
// q_* would advance to the next address's data during the en-off cycle and
// break the addr@T -> data@T+1 -> consume@T+2 discipline (e.g. S_MCAPA would
// capture l1's entry instead of l0's). With everything gated, en-ticks are
// exactly the original clock ticks.

module backend #(
    parameter XW = 12
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            en,

    input  wire [XW-1:0]   width,
    input  wire [XW-1:0]   height,
    input  wire [17:0]     pix_th,
    input  wire            hyst_on,          // (d) reject labels with too few
    input  wire [17:0]     hyst_strong_min,  //   strong (power >= high) pixels
    input  wire [3:0]      border,           // (i) skip labelling interior pixels
                                             //   within this many px of the frame
    input  wire [4:0]      mps_2sq,          // (h) 2*max_perp_spread^2 (0 = off)

    // event input (FIFO pop interface: data valid with ev_pop's next cycle
    // handled by the caller presenting ev_kind/ev_x stably until popped)
    input  wire            ev_empty,
    input  wire [1:0]      ev_kind,
    input  wire [XW-1:0]   ev_x,
    input  wire            ev_strong,        // (d) strong bit of the data event
    output wire            ev_pop,

    // accepted-segment record output (always-ready stream)
    output reg             rec_valid,
    output reg  [10:0]     rec_sx, rec_sy, rec_ex, rec_ey,
    output reg  [17:0]     rec_n,          // 0 = end of frame
    output reg  [29:0]     rec_xs, rec_ys,
    output reg  [40:0]     rec_xss, rec_yss, rec_xys,
    // (f) bbox extreme points: the pixel that attained each extreme of the
    // label's bounding box (companion coordinate included)
    output reg  [10:0]     rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
    output reg  [10:0]     rec_miny, rec_miny_x, rec_maxy, rec_maxy_x
);

    localparam [1:0] K_EOF = 2'd0, K_INT = 2'd1, K_END = 2'd2, K_EOR = 2'd3;
    localparam integer NL = 1024;              // labels (id 0 = "none")
    localparam integer NLW = 10;
    localparam [12:0] TAG_NONE = 13'h1FFF;     // "-1": matches no row >= 0

    // ---- label table (one wide BRAM view: shared read address) --------------
    reg [NLW-1:0] t_conn [0:NL-1];
    reg [12:0]    t_lrow [0:NL-1];
    reg [12:0]    t_lx   [0:NL-1];
    reg [17:0]    t_n    [0:NL-1];
    reg [29:0]    t_xs   [0:NL-1];
    reg [29:0]    t_ys   [0:NL-1];
    // (Lever2-B) Sigma x^2 / y^2 / xy stored in 34 bits: the per-label totals
    // are <= 2^32 over the corpus (moment_probe), 4x below the u34 cap. 34-bit
    // deep-1024 arrays infer 2 BRAM18 each instead of 3 for u41 (-3 BRAM18),
    // freeing block RAM for a second labelling engine (2-stripe). The datapath
    // regs (q_/c_/a_/s_/f_/j_, rec_*) stay u41: reads zero-extend 34->41, the
    // accumulate write truncates 41->34 losslessly (value <= 2^32 < 2^34).
    reg [33:0]    t_xss  [0:NL-1];
    reg [33:0]    t_yss  [0:NL-1];
    reg [33:0]    t_xys  [0:NL-1];
    reg [17:0]    t_scnt [0:NL-1];   // (d) strong-pixel count per label
    reg           t_has  [0:NL-1];
    reg [10:0]    t_sx   [0:NL-1];
    reg [10:0]    t_sy   [0:NL-1];
    // (f) bbox extreme points, 7 arrays (max_y needs none: rows arrive in
    // order, so max_y == t_lrow at all times, merges included — the survivor
    // rule keeps the larger last_row). t_myx = x of the FIRST pixel of the
    // label's newest row (NOT t_lx, which is the latest pixel's x). Written
    // only from S_ACC (single write site), like the moment arrays; creation
    // needs no sentinels because it co-occurs with the first accumulate
    // (S_CREATE preloads c_* with this pixel).
    reg [10:0]    t_mnx  [0:NL-1];
    reg [10:0]    t_mnxy [0:NL-1];
    reg [10:0]    t_mxx  [0:NL-1];
    reg [10:0]    t_mxxy [0:NL-1];
    reg [10:0]    t_mny  [0:NL-1];
    reg [10:0]    t_mnyx [0:NL-1];
    reg [10:0]    t_myx  [0:NL-1];

    // Registered single write ports. Any array the FSM writes from more than
    // one state (including the init sweep) looks like a multi-write-port RAM
    // to XST, which then expands the whole array into registers — the f_tag
    // banks alone cost ~80k flip-flops that way. Funnelling every writer
    // through one registered port keeps 1W+1R and infers block RAM. Writes
    // land one cycle after the requesting state; every read of the affected
    // arrays happens several cycles later (checked per array below), and the
    // find chase tolerates a not-yet-compressed pointer (one extra hop).
    reg           conn_we;               // t_conn: compress / create / merge
    reg [NLW-1:0] conn_wa, conn_wd;
    always @(posedge clk) if (en && conn_we) t_conn[conn_wa] <= conn_wd;

    reg           cold_we;               // t_has/t_sx/t_sy: accumulate / contact
    reg [NLW-1:0] cold_wa;
    reg           cold_whas;
    reg [10:0]    cold_wsx, cold_wsy;
    always @(posedge clk) begin
        if (en && cold_we) begin
            t_has[cold_wa] <= cold_whas;
            t_sx[cold_wa] <= cold_wsx;
            t_sy[cold_wa] <= cold_wsy;
        end
    end

    reg [NLW-1:0] l_ra;
    reg [NLW-1:0] q_conn;
    reg [12:0]    q_lrow, q_lx;
    reg [17:0]    q_n;
    reg [29:0]    q_xs, q_ys;
    reg [40:0]    q_xss, q_yss, q_xys;
    reg [17:0]    q_scnt;
    reg           q_has;
    reg [10:0]    q_sx, q_sy;
    reg [10:0]    q_mnx, q_mnxy, q_mxx, q_mxxy, q_mny, q_mnyx, q_myx;
    always @(posedge clk) begin
        if (en) begin
            q_conn <= t_conn[l_ra];
            q_lrow <= t_lrow[l_ra];
            q_lx   <= t_lx[l_ra];
            q_n    <= t_n[l_ra];
            q_xs   <= t_xs[l_ra];
            q_ys   <= t_ys[l_ra];
            q_xss  <= t_xss[l_ra];
            q_yss  <= t_yss[l_ra];
            q_xys  <= t_xys[l_ra];
            q_scnt <= t_scnt[l_ra];
            q_has  <= t_has[l_ra];
            q_sx   <= t_sx[l_ra];
            q_sy   <= t_sy[l_ra];
            q_mnx  <= t_mnx[l_ra];
            q_mnxy <= t_mnxy[l_ra];
            q_mxx  <= t_mxx[l_ra];
            q_mxxy <= t_mxxy[l_ra];
            q_mny  <= t_mny[l_ra];
            q_mnyx <= t_mnyx[l_ra];
            q_myx  <= t_myx[l_ra];
        end
    end

    // ---- free list ring -------------------------------------------------------
    reg [NLW-1:0] fl [0:NL-1];
    reg [NLW-1:0] fl_head;
    reg [NLW:0]   fl_count;
    reg [NLW-1:0] fl_ra;
    reg [NLW-1:0] fl_q;
    always @(posedge clk) if (en) fl_q <= fl[fl_ra];
    reg           fl_we;                 // init sweep / scavenger (funnelled)
    reg [NLW-1:0] fl_wa, fl_wd;
    always @(posedge clk) if (en && fl_we) fl[fl_wa] <= fl_wd;

    // ---- single label row (tag-validated) --------------------------------------
    reg [12:0]    row_tag [0:2047];
    reg [NLW-1:0] row_lab [0:2047];
    reg [10:0]    row_ra;
    reg [12:0]    rowq_tag;
    reg [NLW-1:0] rowq_lab;
    always @(posedge clk) begin
        if (en) begin
            rowq_tag <= row_tag[row_ra];
            rowq_lab <= row_lab[row_ra];
        end
    end
    reg           rt_we;                 // init sweep / accumulate (funnelled)
    reg [10:0]    rt_wa;
    reg [12:0]    rt_wd;
    always @(posedge clk) if (en && rt_we) row_tag[rt_wa] <= rt_wd;

    // ---- feature rows: 3 tag-validated banks ------------------------------------
    reg [12:0] f_tag0 [0:2047];  reg [1:0] f_kind0 [0:2047];
    reg [12:0] f_tag1 [0:2047];  reg [1:0] f_kind1 [0:2047];
    reg [12:0] f_tag2 [0:2047];  reg [1:0] f_kind2 [0:2047];
    reg [10:0] f_ra;
    reg [12:0] fq_tag0, fq_tag1, fq_tag2;
    reg [1:0]  fq_kind0, fq_kind1, fq_kind2;
    always @(posedge clk) begin
        if (en) begin
            fq_tag0 <= f_tag0[f_ra];  fq_kind0 <= f_kind0[f_ra];
            fq_tag1 <= f_tag1[f_ra];  fq_kind1 <= f_kind1[f_ra];
            fq_tag2 <= f_tag2[f_ra];  fq_kind2 <= f_kind2[f_ra];
        end
    end
    reg        ftag_we;                  // init sweep / event ingest (funnelled)
    reg [2:0]  ftag_wb;                  // bank mask (broadcast on init)
    reg [10:0] ftag_wa;
    reg [12:0] ftag_wd;
    always @(posedge clk) begin
        if (en && ftag_we && ftag_wb[0]) f_tag0[ftag_wa] <= ftag_wd;
        if (en && ftag_we && ftag_wb[1]) f_tag1[ftag_wa] <= ftag_wd;
        if (en && ftag_we && ftag_wb[2]) f_tag2[ftag_wa] <= ftag_wd;
    end

    // ---- interior-x lists (ping-pong) --------------------------------------------
    reg [10:0] xl0 [0:2047];
    reg [10:0] xl1 [0:2047];
    reg        xs0 [0:2047];   // (d) strong bit per interior x, parallel to xl*
    reg        xs1 [0:2047];
    reg [11:0] xcnt0, xcnt1;
    reg [10:0] xl_ra;
    reg [10:0] xl0_q, xl1_q;
    reg        xs0_q, xs1_q;
    always @(posedge clk) begin
        if (en) begin
            xl0_q <= xl0[xl_ra];
            xl1_q <= xl1[xl_ra];
            xs0_q <= xs0[xl_ra];
            xs1_q <= xs1[xl_ra];
        end
    end

    // ---- touched lists (3 generations) --------------------------------------------
    reg [NLW-1:0] tl0 [0:NL-1];
    reg [NLW-1:0] tl1 [0:NL-1];
    reg [NLW-1:0] tl2 [0:NL-1];
    reg [NLW:0]   tcnt0, tcnt1, tcnt2;
    reg [NLW-1:0] tl_ra;
    reg [NLW-1:0] tl0_q, tl1_q, tl2_q;
    always @(posedge clk) begin
        if (en) begin
            tl0_q <= tl0[tl_ra];
            tl1_q <= tl1[tl_ra];
            tl2_q <= tl2[tl_ra];
        end
    end

    // ---- judge unit (NON-BLOCKING since v2b M3) --------------------------------------
    // The judge only decides whether a closed segment is RECORDED — labelling
    // does not depend on its verdict, so the FSM fires the request and moves
    // on (~110 cycles per close reclaimed). One request may be in flight
    // (jf_inflight); its payload is latched into the f_* registers at issue
    // (the judge latches its own operand copies, but the endpoint fields are
    // not judge inputs). A second close waits in its issuing state until the
    // first completes, and the end-of-frame record waits for an idle judge,
    // so the RECORD ORDER is exactly the blocking implementation's.
    reg         j_start;
    reg [17:0]  j_n;
    reg [29:0]  j_xs, j_ys;
    reg [40:0]  j_xss, j_yss, j_xys;
    wire        j_busy, j_done, j_accept;
    judge_unit u_judge (
        .clk(clk), .rst(rst), .en(en),
        .start(j_start), .n(j_n), .xs(j_xs), .ys(j_ys),
        .xss(j_xss), .yss(j_yss), .xys(j_xys), .pix_th(pix_th),
        .mps_2sq(mps_2sq),
        .busy(j_busy), .done(j_done), .accept(j_accept)
    );

    reg         jf_inflight;
    reg [10:0]  f_sx, f_sy, f_ex, f_ey;
    reg [17:0]  f_n;
    reg [17:0]  f_scnt;                // (d) strong count of the judged segment
    reg [29:0]  f_xs, f_ys;
    reg [40:0]  f_xss, f_yss, f_xys;
    reg [10:0]  f_minx, f_minx_y, f_maxx, f_maxx_y;
    reg [10:0]  f_miny, f_miny_x, f_maxy, f_maxy_x;
    wire        jfree = !jf_inflight && !j_start;

    // ---- frame / row bookkeeping -------------------------------------------------------
    reg [12:0] ingest_y;
    reg [1:0]  ing_b;                  // feature bank receiving the ingest row
    reg        eof_row;
    reg [12:0] py;                     // row being processed
    reg [1:0]  b_prev, b_cur, b_next;
    reg [1:0]  tl_cur;                 // touched-list generation of row py
    reg [1:0]  tl_scan;                // generation scanned by the scavenger
    reg [21:0] y_sq;
    reg [11:0] xi;
    reg        first_row_done;

    // sparse-scan carry registers
    reg [10:0]    prev_x;
    reg           prev_x_v;
    reg [NLW-1:0] w_sav;
    reg [12:0]    nw_tag_sav;
    reg [NLW-1:0] nw_lab_sav;

    // per-pixel scratch
    reg [10:0] px;
    reg        pstrong;               // (d) strong bit of px
    reg [1:0]  n_aL, n_aC, n_aR, n_cL, n_cR, n_bL, n_bC, n_bR;
    reg [12:0] rq_tag_m1, rq_tag_0, rq_tag_p1;
    reg [NLW-1:0] rq_lab_m1, rq_lab_0, rq_lab_p1;
    reg [NLW-1:0] label0, label1;
    reg [2:0]  gi;                    // gather phase: 0 = read x+1 col, 1 = dispatch
    reg [3:0]  pdone;                 // (gather) neighbours already find-launched
    reg [NLW-1:0] find_id, find_first;
    reg [NLW-1:0] center;
    reg [12:0] c_lrow, c_lx;
    reg [17:0] c_n;
    reg [29:0] c_xs, c_ys;
    reg [40:0] c_xss, c_yss, c_xys;
    reg [17:0] c_scnt;                // (d) strong count of the centre label
    reg        c_has;
    reg [10:0] c_sx, c_sy;
    reg [10:0] c_mnx, c_mnxy, c_mxx, c_mxxy, c_mny, c_mnyx, c_myx;
    reg [NLW-1:0] m_l0, m_l1;
    reg [12:0] a_lrow, a_lx;
    reg [17:0] a_n;
    reg [29:0] a_xs, a_ys;
    reg [40:0] a_xss, a_yss, a_xys;
    reg [17:0] a_scnt;                // (d) merge: l0's strong count
    reg        a_has;
    reg [10:0] a_sx, a_sy;
    reg [10:0] a_mnx, a_mnxy, a_mxx, a_mxxy, a_mny, a_mnyx, a_myx;
    reg [10:0] scav_row_chk;
    reg [NLW-1:0] scav_id, scav_id3;
    reg [NLW:0] si;
    reg        sv1, sv2, sv3, sv4;     // scavenger pipeline valid flags
    reg [11:0] init_a;
    reg [10:0] pxn;                    // prefetched next interior x
    reg        pstrongn;               // (d) strong bit of pxn
    reg        hn;                     // pxn valid (more pixels this row)
    reg        via_fast;               // this pixel took the RB-skip fast path
                                       //   (pxn captured at S_GATH0, not S_RC)
    reg        first_pix;              // no prefetch available yet

    localparam [5:0]
        S_INIT   = 6'd0,
        S_POP    = 6'd1,
        S_EV     = 6'd2,    // retired: POP+EV fused into S_POP (FWFT front)
        S_ROW0   = 6'd3,
        S_RNEXT  = 6'd4,
        S_RW     = 6'd5,
        S_RCAP   = 6'd6,
        S_RD1    = 6'd7,
        S_RD2    = 6'd8,
        S_RD3    = 6'd9,
        S_GATH0  = 6'd10,
        S_GWAIT  = 6'd11,
        S_GATH1  = 6'd12,
        S_CWAIT  = 6'd13,
        S_CREATE = 6'd14,
        S_MRDA   = 6'd15,
        S_MCAPA  = 6'd16,
        S_MCAPB  = 6'd17,
        S_MEXEC  = 6'd18,
        S_ARD    = 6'd19,
        S_ACAP   = 6'd20,
        S_ACC    = 6'd21,
        S_CONT   = 6'd22,
        S_JWAIT  = 6'd23,   // retired (non-blocking judge); code kept unused
        S_SCAVP  = 6'd24,   // II=1 pipelined scavenger (v2b backend opt 2)
        S_RB     = 6'd25,   // short fetch path (prefetch overlap), wait
        S_RC     = 6'd26,   // short fetch path, centre-column capture
        S_RC2    = 6'd27,   // RB-skip fast path: 1-cy wait for the right column
        S_SCAV4  = 6'd28,   // retired
        S_ROWEND = 6'd29,
        S_TERM   = 6'd30,
        S_HALT   = 6'd31;

    reg [5:0] state;

    // POP+EV fused ingest: the event_fifo is first-word-fall-through, so the
    // front (ev_kind/ev_x/ev_strong) is combinationally valid whenever
    // !ev_empty. The event is therefore consumed AND applied in the single
    // S_POP state (no separate latch-then-use S_EV cycle). ev_pop is a
    // COMBINATIONAL output so the FIFO advances the same cycle we apply the
    // event; a registered pop would re-present the same front next cycle and
    // double-apply it. The consumer (event_fifo / tb) gates the pop with `en`,
    // and the whole FSM only advances on `en`, so exactly one event is
    // consumed per en-cycle spent in S_POP.
    assign ev_pop = (state == S_POP) && !ev_empty;

    wire [12:0] py_m1 = py - 13'd1;
    wire [12:0] py_p1 = py + 13'd1;
    wire        row0 = (py == 13'd0);          // no row above the image

    wire touches_end = ((n_aL | n_aC | n_aR | n_cL | n_cR | n_bL | n_bC | n_bR)
                        & 2'd2) != 2'd0;

    // (gather) parallel neighbour dispatch + gi-aware right-column read: the
    // gpres/grem/gselbit/gcand wires are defined after the fcap function (naR_eff
    // needs it). See there.

    // merge combine (a_* = l0's entry, q_* = l1's entry)
    wire keep0 = ($signed(a_lrow) > $signed(q_lrow)) ||
                 (a_lrow == q_lrow && $signed(a_lx) >= $signed(q_lx));
    wire [17:0] s_n   = a_n + q_n;
    wire [29:0] s_xs  = a_xs + q_xs;
    wire [29:0] s_ys  = a_ys + q_ys;
    wire [40:0] s_xss = a_xss + q_xss;
    wire [40:0] s_yss = a_yss + q_yss;
    wire [40:0] s_xys = a_xys + q_xys;
    wire [17:0] s_scnt = a_scnt + q_scnt;   // (d) merged strong count

    // (f) bbox union at merge: loser (ls_*) wins only strictly-better
    // extremes, ties keep the survivor's point — same as the golden merge.
    // max_y_x is always the survivor's (survivor.last_row >= loser.last_row
    // by the survivor rule, so the golden's `>` branch can never fire).
    wire [10:0] sv_mnx  = keep0 ? a_mnx  : q_mnx;
    wire [10:0] sv_mnxy = keep0 ? a_mnxy : q_mnxy;
    wire [10:0] sv_mxx  = keep0 ? a_mxx  : q_mxx;
    wire [10:0] sv_mxxy = keep0 ? a_mxxy : q_mxxy;
    wire [10:0] sv_mny  = keep0 ? a_mny  : q_mny;
    wire [10:0] sv_mnyx = keep0 ? a_mnyx : q_mnyx;
    wire [10:0] sv_myx  = keep0 ? a_myx  : q_myx;
    wire [10:0] ls_mnx  = keep0 ? q_mnx  : a_mnx;
    wire [10:0] ls_mnxy = keep0 ? q_mnxy : a_mnxy;
    wire [10:0] ls_mxx  = keep0 ? q_mxx  : a_mxx;
    wire [10:0] ls_mxxy = keep0 ? q_mxxy : a_mxxy;
    wire [10:0] ls_mny  = keep0 ? q_mny  : a_mny;
    wire [10:0] ls_mnyx = keep0 ? q_mnyx : a_mnyx;
    wire [10:0] u_mnx  = (ls_mnx < sv_mnx) ? ls_mnx  : sv_mnx;
    wire [10:0] u_mnxy = (ls_mnx < sv_mnx) ? ls_mnxy : sv_mnxy;
    wire [10:0] u_mxx  = (ls_mxx > sv_mxx) ? ls_mxx  : sv_mxx;
    wire [10:0] u_mxxy = (ls_mxx > sv_mxx) ? ls_mxxy : sv_mxxy;
    wire [10:0] u_mny  = (ls_mny < sv_mny) ? ls_mny  : sv_mny;
    wire [10:0] u_mnyx = (ls_mny < sv_mny) ? ls_mnyx : sv_mnyx;

    wire [21:0] px_sq = px * px;
    wire [21:0] pxy   = px * py[10:0];

    // (f) bbox update for the current pixel (11-bit compares, parallel to
    // the wide moment adders — nothing sits in series with them)
    wire bb_nx   = (px < c_mnx);
    wire bb_xx   = (px > c_mxx);
    wire bb_ny   = (py[10:0] < c_mny);   // only the creation pixel can fire
    wire bb_row1 = (c_lrow != py);       // first touch of a new bottom row

    // number of entries in the scanned touched-list generation
    wire [NLW:0] scan_cnt = (tl_scan == 2'd0) ? tcnt0 :
                            (tl_scan == 2'd1) ? tcnt1 : tcnt2;
    // interior count of the processed row
    wire [11:0] row_cnt = py[0] ? xcnt1 : xcnt0;

    // feature bank capture helpers (combinational per current read regs)
    wire [1:0] fk_of_bank0 = fq_kind0, fk_of_bank1 = fq_kind1, fk_of_bank2 = fq_kind2;
    wire [12:0] ft_of_bank0 = fq_tag0, ft_of_bank1 = fq_tag1, ft_of_bank2 = fq_tag2;
    function [1:0] fcap(input [1:0] bank, input [12:0] want_tag, input valid);
        reg [12:0] t;
        reg [1:0] k;
        begin
            t = (bank == 2'd0) ? ft_of_bank0 : (bank == 2'd1) ? ft_of_bank1 : ft_of_bank2;
            k = (bank == 2'd0) ? fk_of_bank0 : (bank == 2'd1) ? fk_of_bank1 : fk_of_bank2;
            fcap = (valid && t == want_tag) ? k : 2'd0;
        end
    endfunction

    // (gather) parallel neighbour dispatch. The four already-labelled neighbours
    // in golden order NE, N, NW, W (MSB..LSB); the serial gi=0..5 walk is
    // replaced by a priority pick over the ones still to process, so absent
    // neighbours cost no cycles. The find launched for the picked neighbour, and
    // the label0/label1 accumulation + path-compression in S_GATH1, are
    // unchanged — only the dead cycles between finds are removed (records stay
    // bit-exact; see rtl/DESIGN.md).
    //
    // (Lever 3) The gi=0 GATHER setup no longer costs its own cycle: gpres/gcand
    // dispatch the first neighbour in the SAME cycle they capture the right
    // column. The right-above (NE) neighbour's feature bank / row-label are only
    // COMBINATIONALLY available at gi=0 (fq/rowq still hold px+1 before the port
    // is repurposed for the next-pixel prefetch), so naR_eff/rqlab_p1_eff read
    // them combinationally at gi=0 and from the latched n_aR / rq_lab_p1 from
    // gi>=1. All the other gather inputs (n_aC/n_aL/n_cL, rq_lab_0/rq_lab_m1) are
    // already latched by S_RC / S_RNEXT and valid at gi=0. This feeds a
    // combinational fcap into the gather-dispatch → conn-address path, so it is
    // the one lever with timing risk (verify synth_be closes 74.25 MHz).
    // NOTE: the gi=0 NE capture is INLINED here (not fcap(...)) on purpose.
    // fcap reads the fq_* banks through module wires, not its arguments; called
    // from a *continuous* assignment, iverilog's sensitivity list omits those
    // internal reads, so the wire would hold a stale power-up X even after the
    // banks settle (procedural fcap calls are fine — they re-evaluate at the
    // clock edge). Inlining the tag/kind reads makes the continuous assign
    // depend on fq_* directly. On a tag miss (ne_tag != py_m1) the kind is
    // masked to 0, so an uninitialised-bank X cannot leak. Synthesis-identical
    // to the fcap form.
    wire        ne_valid = (px + 11'd1 < width[10:0]) && !row0;
    wire [12:0] ne_tag  = (b_prev == 2'd0) ? ft_of_bank0 :
                          (b_prev == 2'd1) ? ft_of_bank1 : ft_of_bank2;
    wire [1:0]  ne_kind = (b_prev == 2'd0) ? fk_of_bank0 :
                          (b_prev == 2'd1) ? fk_of_bank1 : fk_of_bank2;
    wire [1:0]  naR_g0  = (ne_valid && ne_tag == py_m1) ? ne_kind : 2'd0;
    wire [1:0] naR_eff = (gi == 3'd0) ? naR_g0 : n_aR;
    wire [NLW-1:0] rqlab_p1_eff = (gi == 3'd0) ? rowq_lab : rq_lab_p1;
    wire [3:0] gpres = {naR_eff == K_INT, n_aC == K_INT, n_aL == K_INT, n_cL == K_INT};
    wire [3:0] grem  = gpres & ~pdone;              // present & not yet launched
    wire [3:0] gselbit = grem[3] ? 4'b1000 :        // NE first
                         grem[2] ? 4'b0100 :        // then N
                         grem[1] ? 4'b0010 :        // then NW
                                   4'b0001;         // then W
    wire [NLW-1:0] gcand = grem[3] ? rqlab_p1_eff :
                           grem[2] ? rq_lab_0  :
                           grem[1] ? rq_lab_m1 : w_sav;

    always @(posedge clk) begin
        if (en) begin
        j_start <= 1'b0;
        rec_valid <= 1'b0;
        conn_we <= 1'b0;
        ftag_we <= 1'b0;
        rt_we <= 1'b0;
        fl_we <= 1'b0;
        cold_we <= 1'b0;

        case (state)
            // ---- per-frame init: invalidate tags, free list = 1..1023 ----
            S_INIT: begin
                rt_we <= 1'b1;
                rt_wa <= init_a[10:0];
                rt_wd <= TAG_NONE;
                ftag_we <= 1'b1;
                ftag_wb <= 3'b111;
                ftag_wa <= init_a[10:0];
                ftag_wd <= TAG_NONE;
                if (init_a < NL - 1) begin
                    fl_we <= 1'b1;
                    fl_wa <= init_a[NLW-1:0];
                    fl_wd <= init_a[NLW-1:0] + 10'd1;
                end
                init_a <= init_a + 12'd1;
                if (init_a == 12'd2047) begin
                    fl_head <= 10'd0;
                    fl_count <= 11'd1023;
                    ingest_y <= 13'd0;
                    ing_b <= 2'd0;
                    xcnt0 <= 12'd0;
                    xcnt1 <= 12'd0;
                    tcnt0 <= 11'd0;
                    tcnt1 <= 11'd0;
                    tcnt2 <= 11'd0;
                    first_row_done <= 1'b0;
                    eof_row <= 1'b0;
                    state <= S_POP;
                end
            end

            // ---- event ingestion (POP+EV fused; see the ev_pop assign) ----
            // The FWFT front is applied directly this cycle; state stays S_POP
            // for data events so the next event is consumed on the next en edge.
            S_POP: if (!ev_empty) begin
                case (ev_kind)
                    K_INT, K_END: begin
                        ftag_we <= 1'b1;
                        ftag_wb <= (ing_b == 2'd0) ? 3'b001 :
                                   (ing_b == 2'd1) ? 3'b010 : 3'b100;
                        ftag_wa <= ev_x[10:0];
                        ftag_wd <= ingest_y;
                        case (ing_b)
                            2'd0: f_kind0[ev_x[10:0]] <= ev_kind;
                            2'd1: f_kind1[ev_x[10:0]] <= ev_kind;
                            default: f_kind2[ev_x[10:0]] <= ev_kind;
                        endcase
                        if (ev_kind == K_INT) begin
                            if (ingest_y[0]) begin
                                xl1[xcnt1[10:0]] <= ev_x[10:0];
                                xs1[xcnt1[10:0]] <= ev_strong;
                                xcnt1 <= xcnt1 + 12'd1;
                            end else begin
                                xl0[xcnt0[10:0]] <= ev_x[10:0];
                                xs0[xcnt0[10:0]] <= ev_strong;
                                xcnt0 <= xcnt0 + 12'd1;
                            end
                        end
                    end
                    K_EOR: begin
                        if (ingest_y != 13'd0) begin
                            eof_row <= 1'b0;
                            state <= S_ROW0;
                        end else begin
                            ingest_y <= 13'd1;
                            ing_b <= 2'd1;
                        end
                    end
                    default: begin  // K_EOF
                        if (ingest_y != 13'd0) begin
                            eof_row <= 1'b1;
                            state <= S_ROW0;
                        end else begin
                            state <= S_TERM;
                        end
                    end
                endcase
            end

            // ---- row setup ----
            S_ROW0: begin
                py <= ingest_y - 13'd1;
                b_next <= ing_b;
                b_cur <= (ing_b == 2'd0) ? 2'd2 : ing_b - 2'd1;
                b_prev <= (ing_b == 2'd0) ? 2'd1 : (ing_b == 2'd1) ? 2'd2 : 2'd0;
                tl_cur <= first_row_done ?
                          ((tl_cur == 2'd2) ? 2'd0 : tl_cur + 2'd1) : 2'd0;
                first_row_done <= 1'b1;
                y_sq <= (ingest_y[10:0] - 11'd1) * (ingest_y[10:0] - 11'd1);
                xi <= 12'd0;
                prev_x_v <= 1'b0;
                first_pix <= 1'b1;
                hn <= 1'b0;
                via_fast <= 1'b0;
                state <= S_RNEXT;
            end

            // Fetch scheduling (v2b backend opt 2): the first pixel of a row
            // takes the full 7-cycle path (RW..RD3); for every further pixel
            // the list entry and its left column were PREFETCHED during the
            // previous pixel's gather/resolve (xl at RNEXT/RCAP, f/row at
            // GATH0), so the short path RNEXT->RB->RC needs only 3 cycles
            // before the gather. Adjacent-pixel read-after-write hazards are
            // covered by the same nw_sav/w_sav carry registers as the
            // original single-row trick.
            S_RNEXT: begin
                if (first_pix ? (row_cnt == 12'd0) : !hn) begin
                    if (py >= 13'd2) begin
                        si <= 11'd0;
                        sv1 <= 1'b0; sv2 <= 1'b0; sv3 <= 1'b0; sv4 <= 1'b0;
                        scav_row_chk <= py[10:0] - 11'd2;
                        tl_scan <= (tl_cur == 2'd2) ? 2'd0 : tl_cur + 2'd1;
                        state <= S_SCAVP;
                    end else begin
                        state <= S_ROWEND;
                    end
                end else if (first_pix) begin
                    xl_ra <= xi[10:0];
                    xi <= xi + 12'd1;
                    via_fast <= 1'b0;
                    state <= S_RW;
                end else if (prev_x_v && prev_x == px - 11'd1) begin
                    // (RB-skip fast path) Adjacent run: the centre column (px) was
                    // read as the PREVIOUS pixel's right column and still sits in
                    // n_aR/n_bR/rq_*_p1 (they are only overwritten at THIS pixel's
                    // S_GATH0). Carry it instead of re-reading, so the fetch needs
                    // no S_RB bubble: issue the right column (px+1) now and take a
                    // single wait cycle (S_RC2) for it. This is the RB third of the
                    // 3-cy fetch floor removed on adjacent pixels (profiler FETCH
                    // decomposition). The left column is in fq/rowq as usual
                    // (prefetched at the previous S_GATH0). Carry == a fresh read at
                    // column px: n_aR/n_bR use the same bank/tag/address, and
                    // row_lab[px] is untouched between the previous S_GATH0 and here
                    // (the previous pixel writes row_lab[px-1]). Same gate as the W
                    // fast-path: `prev_x == px-1` is false after any label-exhaustion
                    // skip (px would jump by >=2), so the carried registers always
                    // belong to px-1.
                    n_aL <= fcap(b_prev, py_m1, px != 11'd0 && !row0);
                    n_cL <= fcap(b_cur, py, px != 11'd0);
                    n_bL <= fcap(b_next, py_p1, px != 11'd0);
                    rq_tag_m1 <= nw_tag_sav;       // adjacency => NW/W from carry
                    rq_lab_m1 <= nw_lab_sav;
                    n_aC <= n_aR;                  // carry centre column (== px) from
                    n_bC <= n_bR;                  // the previous pixel's right column
                    rq_tag_0 <= rq_tag_p1;
                    rq_lab_0 <= rq_lab_p1;
                    nw_tag_sav <= rq_tag_p1;       // this pixel's N cell -> next NW
                    nw_lab_sav <= rq_lab_p1;
                    f_ra <= px + 11'd1;            // issue the right column (px+1) now
                    row_ra <= px + 11'd1;
                    if (xi != row_cnt) begin
                        xl_ra <= xi[10:0];
                        xi <= xi + 12'd1;
                        hn <= 1'b1;
                    end else hn <= 1'b0;
                    gi <= 3'd0; pdone <= 4'd0; label0 <= 10'd0; label1 <= 10'd0;
                    first_pix <= 1'b0;
                    via_fast <= 1'b1;
                    state <= S_RC2;
                end else begin
                    // short path: fq/rowq already hold column px-1
                    n_aL <= fcap(b_prev, py_m1, px != 11'd0 && !row0);
                    n_cL <= fcap(b_cur, py, px != 11'd0);
                    n_bL <= fcap(b_next, py_p1, px != 11'd0);
                    rq_tag_m1 <= (px != 11'd0) ? rowq_tag : TAG_NONE;
                    rq_lab_m1 <= rowq_lab;
                    f_ra <= px;
                    row_ra <= px;
                    if (xi != row_cnt) begin
                        xl_ra <= xi[10:0];
                        xi <= xi + 12'd1;
                        hn <= 1'b1;
                    end else hn <= 1'b0;
                    via_fast <= 1'b0;
                    state <= S_RB;
                end
            end

            S_RW: state <= S_RCAP;      // xl*_q valid next cycle

            S_RCAP: begin
                px <= py[0] ? xl1_q : xl0_q;
                pstrong <= py[0] ? xs1_q : xs0_q;
                f_ra <= (py[0] ? xl1_q : xl0_q) - 11'd1;
                row_ra <= (py[0] ? xl1_q : xl0_q) - 11'd1;
                if (xi != row_cnt) begin
                    xl_ra <= xi[10:0];   // prefetch the next list entry
                    xi <= xi + 12'd1;
                    hn <= 1'b1;
                end else hn <= 1'b0;
                state <= S_RD1;
            end

            S_RD1: begin                // (RAM latches column x-1 this edge)
                f_ra <= px;
                row_ra <= px;
                state <= S_RD2;
            end

            S_RD2: begin                // column x-1 data valid now
                n_aL <= fcap(b_prev, py_m1, px != 11'd0 && !row0);
                n_cL <= fcap(b_cur, py, px != 11'd0);
                n_bL <= fcap(b_next, py_p1, px != 11'd0);
                if (prev_x_v && prev_x == px - 11'd1) begin
                    rq_tag_m1 <= nw_tag_sav;
                    rq_lab_m1 <= nw_lab_sav;
                end else begin
                    rq_tag_m1 <= (px != 11'd0) ? rowq_tag : TAG_NONE;
                    rq_lab_m1 <= rowq_lab;
                end
                f_ra <= px + 11'd1;
                row_ra <= px + 11'd1;
                if (hn) begin pxn <= py[0] ? xl1_q : xl0_q; pstrongn <= py[0] ? xs1_q : xs0_q; end
                state <= S_RD3;
            end

            S_RB: begin                 // (RAM latches column px this edge)
                f_ra <= px + 11'd1;
                row_ra <= px + 11'd1;
                state <= S_RC;
            end

            // (RB-skip fast path) 1-cycle wait: the right column (px+1) addressed
            // in S_RNEXT is in flight (addr@T -> data@T+2 == S_GATH0). The centre
            // column was carried in S_RNEXT, so nothing to capture here.
            S_RC2: state <= S_GATH0;

            S_RD3, S_RC: begin          // column x data valid now
                n_aC <= fcap(b_prev, py_m1, !row0);
                n_bC <= fcap(b_next, py_p1, 1'b1);
                rq_tag_0 <= rowq_tag;
                rq_lab_0 <= rowq_lab;
                nw_tag_sav <= rowq_tag;   // this pixel's N cell, saved for the
                nw_lab_sav <= rowq_lab;   // next pixel's NW if adjacent
                if (state == S_RC && hn) begin
                    pxn <= py[0] ? xl1_q : xl0_q;
                    pstrongn <= py[0] ? xs1_q : xs0_q;
                end
                first_pix <= 1'b0;
                gi <= 3'd0;
                pdone <= 4'd0;
                label0 <= 10'd0;
                label1 <= 10'd0;
                state <= S_GATH0;
            end

            // ---- gather + find (NE, N, NW, W) ----
            S_GATH0: begin
                // gi=0 captures the right column (px+1) neighbours and repurposes
                // the read port to prefetch the next pixel's left column. The
                // dispatch/resolve chain below then runs in the SAME cycle
                // (gpres/gcand are gi-aware, reading the right column
                // combinationally at gi=0) — the old setup-only cycle is folded
                // away (Lever 3). n_aR/n_cR/n_bR/rq_lab_p1 are still latched here
                // for the later gather iterations and the resolve.
                if (gi == 3'd0) begin   // column x+1 data valid on entry
                    n_aR <= fcap(b_prev, py_m1, (px + 11'd1 < width[10:0]) && !row0);
                    n_cR <= fcap(b_cur, py, px + 11'd1 < width[10:0]);
                    n_bR <= fcap(b_next, py_p1, px + 11'd1 < width[10:0]);
                    rq_tag_p1 <= (px + 11'd1 < width[10:0]) ? rowq_tag : TAG_NONE;
                    rq_lab_p1 <= rowq_lab;
                    if (hn) begin        // prefetch the next pixel's left column
                        if (via_fast) begin
                            // The RB-skip fast path has no S_RC, so pxn/pstrongn
                            // are captured here instead; xl*_q is valid now (set in
                            // S_RNEXT, addr@T -> data@T+2 == this S_GATH0 cycle).
                            pxn <= py[0] ? xl1_q : xl0_q;
                            pstrongn <= py[0] ? xs1_q : xs0_q;
                            f_ra <= (py[0] ? xl1_q : xl0_q) - 11'd1;
                            row_ra <= (py[0] ? xl1_q : xl0_q) - 11'd1;
                        end else begin   // normal/long: pxn already captured (S_RC/S_RD2)
                            f_ra <= pxn - 11'd1;   // column; fq/rowq then hold it
                            row_ra <= pxn - 11'd1; // untouched through the resolve
                        end
                    end
                    gi <= 3'd1;
                end

                if (grem == 4'd0) begin
                    // all present neighbours processed -> resolve
                    if (label0 == 10'd0) begin
                        if (fl_count == 11'd0) begin
                            // label pool exhausted (pathological density):
                            // skip the pixel — neighbours treat it as
                            // background; row/carry state untouched.
                            state <= S_RNEXT;
                        end else begin
                            fl_ra <= fl_head;
                            state <= S_CWAIT;
                        end
                    end else if (label1 == 10'd0) begin
                        center <= label0;
                        if (prev_x_v && prev_x == px - 11'd1 && label0 == w_sav) begin
                            // run continuation (the common case along a
                            // segment): c_* mirrors t_*[label0] exactly —
                            // skip the 2-cycle table re-read.
                            state <= S_ACC;
                        end else begin
                            l_ra <= label0;
                            state <= S_ARD;
                        end
                    end else begin
                        m_l0 <= label0;
                        m_l1 <= label1;
                        l_ra <= label0;
                        state <= S_MRDA;
                    end
                end else if (gselbit == 4'b0001 &&
                             prev_x_v && prev_x == px - 11'd1) begin
                    // (W fast-path) The W neighbour's label is the carry
                    // register w_sav, and w_sav is provably a ROOT: `center` is
                    // only ever set from a find result / fresh label / merge
                    // survivor (all roots) and `w_sav <= center`; single-hop
                    // path-compression never rewrites conn[root], and no merge
                    // runs between the previous pixel's accumulate and this
                    // gather. So find(w_sav) would return w_sav immediately.
                    // Fold w_sav into label0/label1 exactly as S_GATH1's root
                    // branch does (no compression write: find_first would be 0),
                    // skipping the 2-cycle S_GWAIT/S_GATH1 read. Gated on the
                    // run-continuation (prev_x == px-1) so w_sav definitely
                    // belongs to px-1 even in the label-exhaustion skip path,
                    // where prev_x is intentionally left stale (gcand == w_sav
                    // whenever gselbit selects W). Records stay bit-exact.
                    if (label0 == 10'd0)
                        label0 <= w_sav;
                    else if (w_sav != label0 && label1 == 10'd0)
                        label1 <= w_sav;
                    pdone <= pdone | 4'b0001;
                    state <= S_GATH0;
                end else begin
                    // launch the find for the highest-priority present neighbour
                    // still to process; mark it done so the next dispatch skips it
                    find_id <= gcand;
                    l_ra <= gcand;
                    find_first <= 10'd0;
                    pdone <= pdone | gselbit;
                    state <= S_GWAIT;
                end
            end

            S_GWAIT: state <= S_GATH1;  // q_conn(find_id) valid next cycle

            S_GATH1: begin
                if (q_conn == 10'd0) begin
                    if (find_first != 10'd0 && find_first != find_id) begin
                        conn_we <= 1'b1;                 // single-hop compress
                        conn_wa <= find_first;
                        conn_wd <= find_id;
                    end
                    if (label0 == 10'd0)
                        label0 <= find_id;
                    else if (find_id != label0 && label1 == 10'd0)
                        label1 <= find_id;
                    state <= S_GATH0;
                end else begin
                    if (find_first == 10'd0) find_first <= find_id;
                    find_id <= q_conn;
                    l_ra <= q_conn;
                    state <= S_GWAIT;
                end
            end

            // ---- create ----
            S_CWAIT: state <= S_CREATE;  // fl_q valid next cycle

            S_CREATE: begin
                center <= fl_q;
                fl_head <= fl_head + 10'd1;
                fl_count <= fl_count - 11'd1;
                c_lrow <= TAG_NONE;
                c_lx <= TAG_NONE;
                c_n <= 18'd0;
                c_xs <= 30'd0; c_ys <= 30'd0;
                c_xss <= 41'd0; c_yss <= 41'd0; c_xys <= 41'd0;
                c_scnt <= 18'd0;   // (d) S_ACC adds this pixel's strong bit
                c_has <= 1'b0; c_sx <= 11'd0; c_sy <= 11'd0;
                // (f) creation co-occurs with the first accumulate: preload
                // the bbox with this pixel; S_ACC's strict compares are then
                // no-ops for it (matches the golden sentinel + first update).
                c_mnx <= px; c_mnxy <= py[10:0];
                c_mxx <= px; c_mxxy <= py[10:0];
                c_mny <= py[10:0]; c_mnyx <= px;
                c_myx <= px;
                conn_we <= 1'b1;                         // fresh label: root
                conn_wa <= fl_q;
                conn_wd <= 10'd0;
                state <= S_ACC;
            end

            // ---- adopt ----
            S_ARD: state <= S_ACAP;      // q_* of centre valid next cycle
            S_ACAP: begin
                c_lrow <= q_lrow; c_lx <= q_lx;
                c_n <= q_n;
                c_xs <= q_xs; c_ys <= q_ys;
                c_xss <= q_xss; c_yss <= q_yss; c_xys <= q_xys;
                c_scnt <= q_scnt;
                c_has <= q_has; c_sx <= q_sx; c_sy <= q_sy;
                c_mnx <= q_mnx; c_mnxy <= q_mnxy;
                c_mxx <= q_mxx; c_mxxy <= q_mxxy;
                c_mny <= q_mny; c_mnyx <= q_mnyx;
                c_myx <= q_myx;
                state <= S_ACC;
            end

            // ---- merge ----
            S_MRDA: begin                // q_* of l0 latching; switch address
                l_ra <= m_l1;
                state <= S_MCAPA;
            end
            S_MCAPA: begin               // q_* = l0's entry
                a_lrow <= q_lrow; a_lx <= q_lx;
                a_n <= q_n;
                a_xs <= q_xs; a_ys <= q_ys;
                a_xss <= q_xss; a_yss <= q_yss; a_xys <= q_xys;
                a_scnt <= q_scnt;
                a_has <= q_has; a_sx <= q_sx; a_sy <= q_sy;
                a_mnx <= q_mnx; a_mnxy <= q_mnxy;
                a_mxx <= q_mxx; a_mxxy <= q_mxxy;
                a_mny <= q_mny; a_mnyx <= q_mnyx;
                a_myx <= q_myx;
                state <= S_MCAPB;
            end
            S_MCAPB: state <= S_MEXEC;   // q_* = l1's entry from here on

            S_MEXEC: if (!(a_has && q_has) || jfree) begin
                // (when a close is needed and the judge is occupied, hold
                // here — a_*/q_*/inputs are all stable)
                center <= keep0 ? m_l0 : m_l1;
                conn_we <= 1'b1;                         // loser -> survivor
                conn_wa <= keep0 ? m_l1 : m_l0;
                conn_wd <= keep0 ? m_l0 : m_l1;
                c_n <= s_n;
                c_xs <= s_xs; c_ys <= s_ys;
                c_xss <= s_xss; c_yss <= s_yss; c_xys <= s_xys;
                c_scnt <= s_scnt;
                c_lrow <= keep0 ? a_lrow : q_lrow;
                c_lx <= keep0 ? a_lx : q_lx;
                c_mnx <= u_mnx; c_mnxy <= u_mnxy;
                c_mxx <= u_mxx; c_mxxy <= u_mxxy;
                c_mny <= u_mny; c_mnyx <= u_mnyx;
                c_myx <= sv_myx;
                if (a_has && q_has) begin
                    // both halves started: close on the combined moments,
                    // contacts in (l0, l1) argument order; survivor keeps its
                    // own start. The record's bbox is the merged union
                    // WITHOUT the current pixel (golden: merge closes before
                    // accumulate); its max_y = the survivor's last_row.
                    c_has <= 1'b1;
                    c_sx <= keep0 ? a_sx : q_sx;
                    c_sy <= keep0 ? a_sy : q_sy;
                    j_n <= s_n; j_xs <= s_xs; j_ys <= s_ys;
                    j_xss <= s_xss; j_yss <= s_yss; j_xys <= s_xys;
                    f_n <= s_n; f_xs <= s_xs; f_ys <= s_ys;
                    f_xss <= s_xss; f_yss <= s_yss; f_xys <= s_xys;
                    f_scnt <= s_scnt;   // (d) gate on the merged strong count
                    f_sx <= a_sx; f_sy <= a_sy;
                    f_ex <= q_sx; f_ey <= q_sy;
                    f_minx <= u_mnx; f_minx_y <= u_mnxy;
                    f_maxx <= u_mxx; f_maxx_y <= u_mxxy;
                    f_miny <= u_mny; f_miny_x <= u_mnyx;
                    f_maxy <= keep0 ? a_lrow[10:0] : q_lrow[10:0];
                    f_maxy_x <= sv_myx;
                    j_start <= 1'b1;
                    jf_inflight <= 1'b1;
                    state <= S_ACC;
                end else if (a_has || q_has) begin
                    c_has <= 1'b1;      // exactly one half started
                    c_sx <= a_has ? a_sx : q_sx;
                    c_sy <= a_has ? a_sy : q_sy;
                    state <= S_ACC;
                end else begin
                    c_has <= 1'b0;
                    c_sx <= 11'd0; c_sy <= 11'd0;
                    state <= S_ACC;
                end
            end

            // ---- accumulate + writes ----
            S_ACC: begin
                if (c_lrow != py) begin
                    case (tl_cur)
                        2'd0: begin tl0[tcnt0[NLW-1:0]] <= center; tcnt0 <= tcnt0 + 11'd1; end
                        2'd1: begin tl1[tcnt1[NLW-1:0]] <= center; tcnt1 <= tcnt1 + 11'd1; end
                        default: begin tl2[tcnt2[NLW-1:0]] <= center; tcnt2 <= tcnt2 + 11'd1; end
                    endcase
                end
                t_lrow[center] <= py;
                t_lx[center] <= {2'd0, px};
                c_lrow <= py;                  // keep c_* an exact mirror of
                c_lx <= {2'd0, px};            //   t_*[center] (fast path)
                // (f) bbox update, written to the table AND the c_* mirror
                t_mnx[center]  <= bb_nx ? px : c_mnx;
                t_mnxy[center] <= bb_nx ? py[10:0] : c_mnxy;
                t_mxx[center]  <= bb_xx ? px : c_mxx;
                t_mxxy[center] <= bb_xx ? py[10:0] : c_mxxy;
                t_mny[center]  <= bb_ny ? py[10:0] : c_mny;
                t_mnyx[center] <= bb_ny ? px : c_mnyx;
                t_myx[center]  <= bb_row1 ? px : c_myx;
                c_mnx  <= bb_nx ? px : c_mnx;
                c_mnxy <= bb_nx ? py[10:0] : c_mnxy;
                c_mxx  <= bb_xx ? px : c_mxx;
                c_mxxy <= bb_xx ? py[10:0] : c_mxxy;
                c_mny  <= bb_ny ? py[10:0] : c_mny;
                c_mnyx <= bb_ny ? px : c_mnyx;
                c_myx  <= bb_row1 ? px : c_myx;
                t_n[center] <= c_n + 18'd1;
                t_xs[center] <= c_xs + {19'd0, px};
                t_ys[center] <= c_ys + {17'd0, py};
                t_xss[center] <= c_xss + {19'd0, px_sq};
                t_yss[center] <= c_yss + {19'd0, y_sq};
                t_xys[center] <= c_xys + {19'd0, pxy};
                t_scnt[center] <= c_scnt + {17'd0, pstrong};   // (d)
                cold_we <= 1'b1;
                cold_wa <= center;
                cold_whas <= c_has;
                cold_wsx <= c_sx;
                cold_wsy <= c_sy;
                c_n <= c_n + 18'd1;
                c_xs <= c_xs + {19'd0, px};
                c_ys <= c_ys + {17'd0, py};
                c_xss <= c_xss + {19'd0, px_sq};
                c_yss <= c_yss + {19'd0, y_sq};
                c_xys <= c_xys + {19'd0, pxy};
                c_scnt <= c_scnt + {17'd0, pstrong};   // (d) mirror t_scnt
                rt_we <= 1'b1;
                rt_wa <= px;
                rt_wd <= py;
                row_lab[px] <= center;
                w_sav <= center;
                prev_x <= px;
                prev_x_v <= 1'b1;
                state <= S_CONT;
            end

            S_CONT: begin
                // (px <= pxn only on the EXIT branches: the judge-occupied
                // hold must keep px intact for the request payload)
                if (touches_end) begin
                    if (c_has) begin
                        if (jfree) begin
                            j_n <= c_n; j_xs <= c_xs; j_ys <= c_ys;
                            j_xss <= c_xss; j_yss <= c_yss; j_xys <= c_xys;
                            f_n <= c_n; f_xs <= c_xs; f_ys <= c_ys;
                            f_xss <= c_xss; f_yss <= c_yss; f_xys <= c_xys;
                            f_scnt <= c_scnt;   // (d) c_scnt already includes this pixel
                            f_sx <= c_sx; f_sy <= c_sy;
                            f_ex <= px; f_ey <= py[10:0];
                            // (f) contact close: the c_* mirror already
                            // includes this pixel (S_ACC ran); max_y == py.
                            f_minx <= c_mnx; f_minx_y <= c_mnxy;
                            f_maxx <= c_mxx; f_maxx_y <= c_mxxy;
                            f_miny <= c_mny; f_miny_x <= c_mnyx;
                            f_maxy <= py[10:0]; f_maxy_x <= c_myx;
                            j_start <= 1'b1;
                            jf_inflight <= 1'b1;
                            px <= pxn; pstrong <= pstrongn;
                            state <= S_RNEXT;
                        end
                        // else: judge occupied — hold here (inputs stable)
                    end else begin
                        cold_we <= 1'b1;
                        cold_wa <= center;
                        cold_whas <= 1'b1;
                        cold_wsx <= px;
                        cold_wsy <= py[10:0];
                        c_has <= 1'b1;             // mirror t_* (fast path)
                        c_sx <= px;
                        c_sy <= py[10:0];
                        px <= pxn; pstrong <= pstrongn;
                        state <= S_RNEXT;
                    end
                end else begin
                    px <= pxn; pstrong <= pstrongn;
                    state <= S_RNEXT;
                end
            end

            // ---- scavenger: free labels whose last activity row is py-2 ----
            // II=1 5-stage pipeline (issue / tl latency / capture id + issue
            // table read / table latency / check & free): n entries in n+4
            // cycles instead of 5n — this is what made dense vertical-stripe
            // content (every stripe label touched every row) drown the
            // back-end. NOTE the BRAM discipline: data is valid TWO cycles
            // after the address (addr@T -> data@T+2) — capturing one cycle
            // early frees live labels and corrupts frames (caught by the
            // FullHD regression).
            S_SCAVP: begin
                if (si != scan_cnt) begin              // stage 0: issue
                    tl_ra <= si[NLW-1:0];
                    si <= si + 11'd1;
                    sv1 <= 1'b1;
                end else sv1 <= 1'b0;
                sv2 <= sv1;                            // stage 1: tl latency
                if (sv2) begin                         // stage 2: tl_q valid
                    scav_id <= (tl_scan == 2'd0) ? tl0_q :
                               (tl_scan == 2'd1) ? tl1_q : tl2_q;
                    l_ra <= (tl_scan == 2'd0) ? tl0_q :
                            (tl_scan == 2'd1) ? tl1_q : tl2_q;
                end
                sv3 <= sv2;                            // stage 3: table latency
                if (sv3) scav_id3 <= scav_id;
                sv4 <= sv3;
                if (sv4 && q_lrow == {2'd0, scav_row_chk}) begin  // stage 4
                    fl_we <= 1'b1;
                    fl_wa <= (fl_head + fl_count[NLW-1:0]) & 10'h3FF;
                    fl_wd <= scav_id3;
                    fl_count <= fl_count + 11'd1;
                end
                if (si == scan_cnt && !sv1 && !sv2 && !sv3 && !sv4) begin
                    case (tl_scan)
                        2'd0: tcnt0 <= 11'd0;
                        2'd1: tcnt1 <= 11'd0;
                        default: tcnt2 <= 11'd0;
                    endcase
                    state <= S_ROWEND;
                end
            end

            S_ROWEND: begin
                if (py[0]) xcnt1 <= 12'd0;
                else xcnt0 <= 12'd0;
                if (eof_row) state <= S_TERM;
                else begin
                    ingest_y <= ingest_y + 13'd1;
                    ing_b <= (ing_b == 2'd2) ? 2'd0 : ing_b + 2'd1;
                    state <= S_POP;
                end
            end

            S_TERM: if (jfree) begin       // last in-flight record first
                rec_valid <= 1'b1;
                rec_n <= 18'd0;
                rec_sx <= 11'd0; rec_sy <= 11'd0; rec_ex <= 11'd0; rec_ey <= 11'd0;
                rec_xs <= 30'd0; rec_ys <= 30'd0;
                rec_xss <= 41'd0; rec_yss <= 41'd0; rec_xys <= 41'd0;
                rec_minx <= 11'd0; rec_minx_y <= 11'd0;
                rec_maxx <= 11'd0; rec_maxx_y <= 11'd0;
                rec_miny <= 11'd0; rec_miny_x <= 11'd0;
                rec_maxy <= 11'd0; rec_maxy_x <= 11'd0;
                state <= S_HALT;
            end

            S_HALT: state <= S_HALT;

            default: state <= S_HALT;
        endcase

        // in-flight judge completion (overrides the default rec_valid clear;
        // never coincides with S_TERM's emission — S_TERM waits for jfree)
        if (j_done) begin
            jf_inflight <= 1'b0;
            // (d) hysteresis gate: reject if too few strong pixels. Independent
            // of the pix/aspect judge (all criteria are ANDed), so applying it
            // at emission matches the C model's early return in judgeAndEmit.
            // (i) border margin: also reject if the finalised bounding box reaches
            // within `border` px of the frame (same integer bbox test as the C
            // model judgeAndEmit; f_maxy == the record's max_y == last_row).
            if (j_accept && !(hyst_on && f_scnt < hyst_strong_min)
                && !(border != 4'd0 &&
                     (f_minx < {7'd0, border} ||
                      f_maxx >= width[10:0] - {7'd0, border} ||
                      f_miny < {7'd0, border} ||
                      f_maxy >= height[10:0] - {7'd0, border}))) begin
                rec_valid <= 1'b1;
                rec_sx <= f_sx; rec_sy <= f_sy;
                rec_ex <= f_ex; rec_ey <= f_ey;
                rec_n <= f_n;
                rec_xs <= f_xs; rec_ys <= f_ys;
                rec_xss <= f_xss; rec_yss <= f_yss; rec_xys <= f_xys;
                rec_minx <= f_minx; rec_minx_y <= f_minx_y;
                rec_maxx <= f_maxx; rec_maxx_y <= f_maxx_y;
                rec_miny <= f_miny; rec_miny_x <= f_miny_x;
                rec_maxy <= f_maxy; rec_maxy_x <= f_maxy_x;
            end
        end
        end

        if (rst) begin
            state <= S_INIT;
            init_a <= 12'd0;
            j_start <= 1'b0;
            jf_inflight <= 1'b0;
            rec_valid <= 1'b0;
        end
    end

endmodule
