// PER-ROW drain-cycle profiling testbench (diagnostic).
//
// Purpose: analyse WHERE the backend spends cycles during a BURST. tb_backend_prof.v
// gives the frame-total state histogram; tb_backend_burst.v gives per-row FIFO
// occupancy/drops. Neither shows the per-row *drain cost* broken down by state, which
// is what decides whether a row overflows: on the board the backend gets exactly
// (W + HBLANK) pixel clocks per row period to drain that row's work. If a row's
// intrinsic drain cost exceeds that budget the FIFO backlog grows -> burst.
//
// This TB runs at CE_DIV=1 with UNLIMITED supply (event array, no FIFO, never stalls),
// so the measured per-row cycle count is the PURE intrinsic drain cost of that row
// (the "drain capability" side of supply-vs-drain). Cycles are attributed to the
// current drain row = number of EOR events popped so far (same row index space the
// producer-side burst.csv uses). For each row it accumulates the state-group cycle
// breakdown and the interior-event count, then dumps {VEC}_rowprof.csv:
//   row,interior,total,ingest,fetch,gather,acc,cont,adopt,scav,mergecreate,rowsetup
// Records are also checked bit-exact against the golden stream (sanity that the
// instrumentation did not perturb the FSM).
//
// Note on the y-lag: the backend processes physical row py while it has already
// ingested row py+1's events (single-row-buffer trick). Attribution by EOR-pop count
// therefore places "ingest of row k + process of row ~k-1" in bucket k; the <=2 row
// phase offset is immaterial for the multi-row burst-region analysis (dense regions
// span ~100 rows) and the per-row total is exact.

`timescale 1ns / 1ps

`ifndef CE_DIV
`define CE_DIV 1
`endif
`ifndef HYST_ON
`define HYST_ON 0
`endif
`ifndef HYST_MIN
`define HYST_MIN 3
`endif
`ifndef BORDER
`define BORDER 0
`endif
`ifndef MPS_2SQ
`define MPS_2SQ 0
`endif

module tb_backend_rowprof;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam integer MAXEV = 2 * W * H + H + 4;
    localparam integer MAXREC = 8192;
    localparam integer MAXROW = 1279;

    reg clk = 1'b0;
    reg rst = 1'b1;

    reg [3:0] ce_cnt = 4'd0;
    always @(posedge clk) ce_cnt <= (ce_cnt == `CE_DIV - 1) ? 4'd0 : ce_cnt + 4'd1;
    wire ce = (ce_cnt == 4'd0);

    // ---- event source (golden stream, unlimited supply) --------------------
    reg [15:0] ev_mem [0:MAXEV-1];
    integer ev_n;
    integer ev_i;
    wire ev_empty = (ev_i >= ev_n);
    wire [1:0] ev_kind = ev_mem[ev_i][13:12];
    wire [XW-1:0] ev_x = ev_mem[ev_i][XW-1:0];
    wire ev_strong = ev_mem[ev_i][14];
    wire ev_pop;

    localparam [1:0] K_EOF = 2'd0, K_INT = 2'd1, K_END = 2'd2, K_EOR = 2'd3;

    // current drain row = # of EOR events popped so far
    integer cur_row;
    always @(posedge clk) begin
        if (rst) begin
            ev_i <= 0; cur_row <= 0;
        end else if (ev_pop && ce) begin
            if (ev_kind == K_EOR && cur_row < MAXROW) cur_row <= cur_row + 1;
            ev_i <= ev_i + 1;
        end
    end

    // ---- golden records -----------------------------------------------------
    reg [10:0] g_sx [0:MAXREC-1];
    reg [10:0] g_sy [0:MAXREC-1];
    reg [10:0] g_ex [0:MAXREC-1];
    reg [10:0] g_ey [0:MAXREC-1];
    reg [17:0] g_n  [0:MAXREC-1];
    reg [29:0] g_xs [0:MAXREC-1];
    reg [29:0] g_ys [0:MAXREC-1];
    reg [40:0] g_xss [0:MAXREC-1];
    reg [40:0] g_yss [0:MAXREC-1];
    reg [40:0] g_xys [0:MAXREC-1];
    reg [87:0] g_bb [0:MAXREC-1];
    integer g_cnt;

    // ---- DUT ----------------------------------------------------------------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;
    wire [87:0] rec_bb = {rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
                          rec_miny, rec_miny_x, rec_maxy, rec_maxy_x};

    backend #(.XW(XW)) dut (
        .clk(clk), .rst(rst), .en(ce),
        .width(W[XW-1:0]), .height(H[XW-1:0]), .pix_th(18'd`PIX_TH),
        .hyst_on(`HYST_ON != 0), .hyst_strong_min(18'd`HYST_MIN),
        .border(4'd`BORDER), .mps_2sq(5'd`MPS_2SQ),
        .ev_empty(ev_empty), .ev_kind(ev_kind), .ev_x(ev_x),
        .ev_strong(ev_strong), .ev_pop(ev_pop),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n), .rec_xs(rec_xs), .rec_ys(rec_ys),
        .rec_xss(rec_xss), .rec_yss(rec_yss), .rec_xys(rec_xys),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x)
    );

    always #5 clk = ~clk;

    integer errors = 0;
    integer ri = 0;
    reg frame_done = 1'b0;

    always @(posedge clk) begin
        if (!rst && rec_valid && ce) begin
            if (rec_n == 18'd0) begin
                frame_done <= 1'b1;
            end else begin
                if (ri < g_cnt) begin
                    if (rec_sx !== g_sx[ri] || rec_sy !== g_sy[ri] ||
                        rec_ex !== g_ex[ri] || rec_ey !== g_ey[ri] ||
                        rec_n !== g_n[ri] ||
                        rec_xs !== g_xs[ri] || rec_ys !== g_ys[ri] ||
                        rec_xss !== g_xss[ri] || rec_yss !== g_yss[ri] ||
                        rec_xys !== g_xys[ri] || rec_bb !== g_bb[ri]) begin
                        errors = errors + 1;
                    end
                end else errors = errors + 1;
                ri = ri + 1;
            end
        end
    end

    // ---- per-row grouped cycle accumulators --------------------------------
    integer row_tot   [0:MAXROW];
    integer row_int   [0:MAXROW];
    integer row_ing   [0:MAXROW];   // S_POP (ingest)
    integer row_fetch [0:MAXROW];   // RNEXT,RW,RCAP,RD1-3,RB,RC,RC2
    integer row_gath  [0:MAXROW];   // GATH0,GWAIT,GATH1
    integer row_acc   [0:MAXROW];   // ACC
    integer row_cont  [0:MAXROW];   // CONT
    integer row_adopt [0:MAXROW];   // ARD,ACAP
    integer row_scav  [0:MAXROW];   // SCAVP
    integer row_mc    [0:MAXROW];   // CWAIT,CREATE,MRDA,MCAPA,MCAPB,MEXEC
    integer row_rse   [0:MAXROW];   // ROW0,ROWEND
    integer active_cyc;

    integer s;
    always @(posedge clk) begin
        if (!rst && !frame_done && ce && cur_row <= MAXROW) begin
            active_cyc = active_cyc + 1;
            row_tot[cur_row] = row_tot[cur_row] + 1;
            s = dut.state;
            case (s)
                1:                        row_ing[cur_row]   = row_ing[cur_row]   + 1;
                4,5,6,7,8,9,23,25,26,27:  row_fetch[cur_row] = row_fetch[cur_row] + 1;
                10,11,12:                 row_gath[cur_row]  = row_gath[cur_row]  + 1;
                21:                       row_acc[cur_row]   = row_acc[cur_row]   + 1;
                22:                       row_cont[cur_row]  = row_cont[cur_row]  + 1;
                19,20:                    row_adopt[cur_row] = row_adopt[cur_row] + 1;
                24:                       row_scav[cur_row]  = row_scav[cur_row]  + 1;
                13,14,15,16,17,18:        row_mc[cur_row]    = row_mc[cur_row]    + 1;
                3,29:                     row_rse[cur_row]   = row_rse[cur_row]   + 1;
                default: ;  // S_INIT / S_TERM / S_HALT
            endcase
        end
        // interior events supplied for this row
        if (!rst && !frame_done && ev_pop && ce &&
            (ev_kind == K_INT || ev_kind == K_END) && cur_row <= MAXROW)
            row_int[cur_row] = row_int[cur_row] + 1;
    end

    // watchdog + finish + report
    integer cyc = 0;
    integer n_int, n_end, n_eor;
    integer rr, fcsv;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (frame_done) begin
            $display("RESULT records=%0d errors=%0d", ri, errors);
            $display("EVENTS int=%0d end=%0d eor=%0d", n_int, n_end, n_eor);
            $display("ACTIVE_CYC %0d rows=%0d", active_cyc, cur_row);
            fcsv = $fopen({`VEC, "_rowprof.csv"}, "w");
            if (fcsv != 0) begin
                $fdisplay(fcsv, "row,interior,total,ingest,fetch,gather,acc,cont,adopt,scav,mergecreate,rowsetup");
                for (rr = 0; rr <= cur_row && rr <= MAXROW; rr = rr + 1)
                    $fdisplay(fcsv, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                              rr, row_int[rr], row_tot[rr], row_ing[rr], row_fetch[rr],
                              row_gath[rr], row_acc[rr], row_cont[rr], row_adopt[rr],
                              row_scav[rr], row_mc[rr], row_rse[rr]);
                $fclose(fcsv);
            end
            $finish;
        end
        if (cyc > 40000000) begin
            $display("FAIL watchdog state=%0d ev %0d/%0d", dut.state, ev_i, ev_n);
            $finish;
        end
    end

    integer fd, r;
    reg [10:0] v_sx, v_sy, v_ex, v_ey;
    reg [17:0] v_n;
    reg [29:0] v_xs, v_ys;
    reg [40:0] v_xss, v_yss, v_xys;
    reg [10:0] v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7;
    integer i;
    initial begin
        for (i = 0; i <= MAXROW; i = i + 1) begin
            row_tot[i]=0; row_int[i]=0; row_ing[i]=0; row_fetch[i]=0; row_gath[i]=0;
            row_acc[i]=0; row_cont[i]=0; row_adopt[i]=0; row_scav[i]=0; row_mc[i]=0;
            row_rse[i]=0;
        end
        active_cyc = 0;
        n_int = 0; n_end = 0; n_eor = 0;
        for (i = 0; i < MAXEV; i = i + 1) ev_mem[i] = 16'hxxxx;
        $readmemh({`VEC, "_events.hex"}, ev_mem);
        ev_n = 0;
        for (i = 0; i < MAXEV; i = i + 1)
            if (ev_mem[i] !== 16'hxxxx) begin
                ev_n = ev_n + 1;
                case (ev_mem[i][13:12])
                    2'd1: n_int = n_int + 1;
                    2'd2: n_end = n_end + 1;
                    2'd3: n_eor = n_eor + 1;
                    default: ;
                endcase
            end

        g_cnt = 0;
        fd = $fopen({`VEC, "_records.hex"}, "r");
        if (fd != 0) begin
            r = 18;
            while (r == 18) begin
                r = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h",
                            v_sx, v_sy, v_ex, v_ey, v_n, v_xs, v_ys,
                            v_xss, v_yss, v_xys,
                            v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7);
                if (r == 18) begin
                    g_sx[g_cnt] = v_sx; g_sy[g_cnt] = v_sy;
                    g_ex[g_cnt] = v_ex; g_ey[g_cnt] = v_ey;
                    g_n[g_cnt] = v_n;
                    g_xs[g_cnt] = v_xs; g_ys[g_cnt] = v_ys;
                    g_xss[g_cnt] = v_xss; g_yss[g_cnt] = v_yss;
                    g_xys[g_cnt] = v_xys;
                    g_bb[g_cnt] = {v_b0, v_b1, v_b2, v_b3, v_b4, v_b5, v_b6, v_b7};
                    g_cnt = g_cnt + 1;
                end
            end
            $fclose(fd);
        end

        repeat (6) @(posedge clk);
        rst <= 1'b0;
    end

endmodule
