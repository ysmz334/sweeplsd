// BURST testbench (diagnostic): real backend.v + real
// event_fifo.v (drop_mode=1), driven by the golden event stream replayed at
// REAL HDMI pixel timing (active pixels 1/clk at their column, EOR at end of
// the active line, then HBLANK idle clocks, then the next row). This is the
// ground-truth burst model that fifo_dropsim.cpp only approximates (dropsim
// lumps the per-interior PROCESS cost at the EOR marker; here the actual FSM
// interleaves fetch/gather/accumulate per pixel). Logs FIFO occupancy over
// time to see whether dense rows overflow the 2048-deep FIFO as a BURST even
// though the average backend load fits the frame budget.
//
// Shared pixel-clock domain (v2b M3): 1 global cycle == 1 pixel clock, en=1.
// Reconstructs each event's produce cycle incrementally from its column x and
// the EOR row boundaries (+HBLANK). drop_mode=1 so overload drops DATA events.
// Prints: peak occupancy, total dropped events, records emitted (vs golden
// count), rows with drops; dumps a per-row CSV (row,interior,peak,drop) for
// the burst profile.

`timescale 1ns / 1ps

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
`ifndef HBLANK
`define HBLANK 280
`endif
`ifndef INIT_WAIT
`define INIT_WAIT 2100
`endif
`ifndef DEPTH_AW
`define DEPTH_AW 11
`endif
`ifndef RSV
`define RSV 1152
`endif
`ifndef DHYST
`define DHYST 448
`endif
`ifndef OUT
`define OUT `VEC
`endif

module tb_backend_burst;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam integer MAXEV = 2 * W * H + H + 4;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;
    wire en = 1'b1;                 // full rate == pixel rate (shared domain)

    // ---- event source memory (golden stream) -------------------------------
    reg [15:0] ev_mem [0:MAXEV-1];
    reg        dropmask [0:MAXEV-1];   // 1 == this event index was dropped by the FIFO
    integer ev_n;

    // ---- timed producer -> FIFO --------------------------------------------
    integer push_i;
    reg     started;
    integer gcyc;                  // global cycle since producer start
    // reconstruction state
    reg [63:0] row_base;           // cycle of column 0 of the current row
    reg [63:0] last_cyc;           // produce cycle of the last pushed event
    integer prod_row;              // row currently being produced (EOR count)

    wire [1:0]  pk  = ev_mem[push_i][13:12];
    wire [11:0] pxx = ev_mem[push_i][XW-1:0];
    wire is_data_p  = (pk == 2'd1) || (pk == 2'd2);
    wire [63:0] base_c = is_data_p ? (row_base + pxx) : (row_base + W);
    wire [63:0] next_cyc = (base_c > last_cyc) ? base_c : (last_cyc + 64'd1);
    wire due = started && (push_i < ev_n) && ({32'd0, gcyc} >= next_cyc);

    // FIFO
    wire        fifo_empty, fifo_pop, fifo_dropped, fifo_stall;
    wire [14:0] fifo_front;
    event_fifo #(.DW(15), .AW(`DEPTH_AW), .RESERVE(`RSV), .DROP_HYST(`DHYST)) u_fifo (
        .clk(clk), .rst(rst), .en(en),
        .drop_mode(1'b1), .push(due), .wdata(ev_mem[push_i]),
        .stall(fifo_stall), .dropped(fifo_dropped),
        .empty(fifo_empty), .front(fifo_front), .pop(fifo_pop));

    // ---- backend DUT --------------------------------------------------------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;

    backend
`ifndef GL
        #(.XW(XW))
`endif
    dut (
        .clk(clk), .rst(rst), .en(en),
        .width(W[XW-1:0]), .height(H[XW-1:0]), .pix_th(18'd`PIX_TH),
        .hyst_on(`HYST_ON != 0), .hyst_strong_min(18'd`HYST_MIN),
        .border(4'd`BORDER), .mps_2sq(5'd`MPS_2SQ),
        .ev_empty(fifo_empty), .ev_kind(fifo_front[13:12]),
        .ev_x(fifo_front[XW-1:0]), .ev_strong(fifo_front[14]), .ev_pop(fifo_pop),
        .rec_valid(rec_valid),
        .rec_sx(rec_sx), .rec_sy(rec_sy), .rec_ex(rec_ex), .rec_ey(rec_ey),
        .rec_n(rec_n), .rec_xs(rec_xs), .rec_ys(rec_ys),
        .rec_xss(rec_xss), .rec_yss(rec_yss), .rec_xys(rec_xys),
        .rec_minx(rec_minx), .rec_minx_y(rec_minx_y),
        .rec_maxx(rec_maxx), .rec_maxx_y(rec_maxx_y),
        .rec_miny(rec_miny), .rec_miny_x(rec_miny_x),
        .rec_maxy(rec_maxy), .rec_maxy_x(rec_maxy_x)
    );

    // ---- producer + reconstruction advance ---------------------------------
    always @(posedge clk) begin
        if (rst) begin
            push_i <= 0; started <= 1'b0; gcyc <= 0;
            row_base <= 64'd0; last_cyc <= 64'hFFFFFFFFFFFFFFFF; prod_row <= 0;
        end else begin
            if (!started) begin
                gcyc <= gcyc + 1;
                if (gcyc >= `INIT_WAIT) begin
                    started <= 1'b1; gcyc <= 0; row_base <= 64'd0;
                    last_cyc <= 64'hFFFFFFFFFFFFFFFF;
                end
            end else begin
                gcyc <= gcyc + 1;
                if (due) begin
                    last_cyc <= next_cyc;
                    if (pk == 2'd3) begin
                        row_base <= row_base + W + `HBLANK;
                        prod_row <= prod_row + 1;
                    end
                    push_i <= push_i + 1;
                end
            end
        end
    end

`ifndef GL
    // ---- MONITOR: judge-wait profile (board-LED comparison) -----------------
    // jstall = cycles with jwd > 16 (same definition as the board's v18 LED);
    // jmax = worst single-dispatch wait; jdisp = dispatches.
    integer jstall_cyc, jmax, jdisp;
    reg jf_q;
    always @(posedge clk) begin
        if (rst) begin jstall_cyc = 0; jmax = 0; jdisp = 0; jf_q = 0; end
        else begin
            if (dut.jf_inflight && !dut.j_done && dut.jwd > 9'd16)
                jstall_cyc = jstall_cyc + 1;
            if (dut.jwd > jmax) jmax = dut.jwd;
            jf_q <= dut.jf_inflight;
            if (dut.jf_inflight && !jf_q) jdisp = jdisp + 1;
        end
    end

    // ---- MONITOR: per-dispatch operand/latency dump (netlist-bug hunt) ------
    integer fdisp, dn, dstart;
    initial begin
        fdisp = $fopen({`OUT, "_disp.txt"}, "w");
        dn = 0;
    end
    always @(posedge clk) begin
        if (!rst) begin
            if (dut.jf_inflight && !jf_q) begin
                dn = dn + 1;
                dstart = gcyc;
                if (dn <= 300 && fdisp != 0)
                    $fdisplay(fdisp, "disp %0d cyc %0d n=%0d xs=%0d ys=%0d xss=%0d yss=%0d xys=%0d mps=%0d",
                              dn, gcyc, dut.j_n, dut.j_xs, dut.j_ys,
                              dut.j_xss, dut.j_yss, dut.j_xys, dut.u_judge.mps_2sq);
            end
            if (!dut.jf_inflight && jf_q && dn <= 300 && fdisp != 0)
                $fdisplay(fdisp, "disp %0d done_at %0d (lat %0d)", dn, gcyc, gcyc - dstart);
        end
    end

`endif
    // ---- MONITOR: markers resident in the FIFO + hard ring overflow ---------
    integer mark_in, mark_max;
    reg ovfl;
    wire push_eff = due && !(u_fifo.drop);
    wire mk_push = push_eff && (pk == 2'd3 || pk == 2'd0);
    wire mk_pop  = fifo_pop && (fifo_front[13:12] == 2'd3 || fifo_front[13:12] == 2'd0);
    always @(posedge clk) begin
        if (rst) begin mark_in = 0; mark_max = 0; ovfl = 1'b0; end
        else begin
            if (mk_push && !mk_pop) mark_in = mark_in + 1;
            else if (mk_pop && !mk_push) mark_in = mark_in - 1;
            if (mark_in > mark_max) mark_max = mark_in;
            if (u_fifo.count > (1<<`DEPTH_AW) && !ovfl) begin
                ovfl = 1'b1;
                $display("HARD-OVERFLOW count=%0d at prod_row=%0d markers=%0d",
                         u_fifo.count, prod_row, mark_in);
            end
        end
    end

    // ---- occupancy / drop / record logging ---------------------------------
    integer peak_occ;
    integer total_drop;
    integer rec_emitted;
    reg     frame_done;
    integer row_int  [0:1200];     // interior events produced per row
    integer row_peak [0:1200];     // peak FIFO occupancy per (producing) row
    integer row_drop [0:1200];     // dropped events per (producing) row
    integer occ;

    always @(posedge clk) begin
        if (!rst && started && !frame_done) begin
            occ = u_fifo.count;
            if (occ > peak_occ) peak_occ = occ;
            if (prod_row <= 1200) begin
                if (occ > row_peak[prod_row]) row_peak[prod_row] = occ;
            end
            if (fifo_dropped) begin
                total_drop = total_drop + 1;
                dropmask[push_i] = 1'b1;   // event ev_mem[push_i] is being dropped this cycle
                if (prod_row <= 1200) row_drop[prod_row] = row_drop[prod_row] + 1;
            end
            if (due && is_data_p && prod_row <= 1200)
                row_int[prod_row] = row_int[prod_row] + 1;
            if (rec_valid) begin
                if (rec_n == 18'd0) frame_done <= 1'b1;
                else rec_emitted = rec_emitted + 1;
            end
        end
    end

    // record dump (sy sx ey ex n per line) for spatial analysis
    integer frec;
    initial frec = $fopen({`OUT, "_recs_out.txt"}, "w");
    always @(posedge clk) begin
        if (!rst && started && !frame_done && rec_valid && rec_n != 18'd0 && frec != 0)
            $fdisplay(frec, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                      rec_sx, rec_sy, rec_ex, rec_ey, rec_n,
                      rec_xs, rec_ys, rec_xss, rec_yss, rec_xys,
                      rec_minx, rec_minx_y, rec_maxx, rec_maxx_y,
                      rec_miny, rec_miny_x, rec_maxy, rec_maxy_x);
    end

    // ---- setup + finish -----------------------------------------------------
    integer i, watchdog, fcsv, rr, rows_with_drop, maxrow, maxrowocc, fmask;
    initial begin
        peak_occ = 0; total_drop = 0; rec_emitted = 0; frame_done = 1'b0;
        for (i = 0; i <= 1200; i = i + 1) begin
            row_int[i] = 0; row_peak[i] = 0; row_drop[i] = 0;
        end
        for (i = 0; i < MAXEV; i = i + 1) begin ev_mem[i] = 16'hxxxx; dropmask[i] = 1'b0; end
        $readmemh({`VEC, "_events.hex"}, ev_mem);
        ev_n = 0;
        for (i = 0; i < MAXEV; i = i + 1)
            if (ev_mem[i] !== 16'hxxxx) ev_n = ev_n + 1;
        $display("BURST-TB events=%0d HBLANK=%0d DEPTH=%0d", ev_n, `HBLANK, (1<<`DEPTH_AW));
        repeat (6) @(posedge clk);
        rst <= 1'b0;
    end

`ifdef EARLYSTOP
    always @(posedge clk) if (!rst && gcyc > `EARLYSTOP) begin
        $display("EARLYSTOP at gcyc %0d rec_emitted=%0d total_drop=%0d",
                 gcyc, rec_emitted, total_drop);
`ifndef GL
        $fclose(fdisp);
`endif
        $finish;
    end
`endif

    // watchdog + report
    always @(posedge clk) begin
        watchdog = watchdog + 1;
        if (frame_done) begin
            rows_with_drop = 0; maxrow = 0; maxrowocc = 0;
            for (rr = 0; rr <= 1200; rr = rr + 1) begin
                if (row_drop[rr] > 0) rows_with_drop = rows_with_drop + 1;
                if (row_drop[rr] > maxrow) maxrow = row_drop[rr];
                if (row_peak[rr] > maxrowocc) maxrowocc = row_peak[rr];
            end
            $display("MARKERS max_in_fifo=%0d ovfl=%0d", mark_max, ovfl);
`ifndef GL
            $display("JUDGE dispatches=%0d jstall_cyc=%0d jmax_wait=%0d",
                     jdisp, jstall_cyc, jmax);
`endif
            $display("RESULT peak_occ=%0d total_drop=%0d rec_emitted=%0d rows_with_drop=%0d max_row_drop=%0d max_row_peak=%0d",
                     peak_occ, total_drop, rec_emitted, rows_with_drop, maxrow, maxrowocc);
            // per-row CSV
            fcsv = $fopen({`OUT, "_burst.csv"}, "w");
            if (fcsv != 0) begin
                $fdisplay(fcsv, "row,interior,peak_occ,drop");
                for (rr = 0; rr < prod_row; rr = rr + 1)
                    $fdisplay(fcsv, "%0d,%0d,%0d,%0d",
                              rr, row_int[rr], row_peak[rr], row_drop[rr]);
                $fclose(fcsv);
            end
            // per-event drop mask (one 0/1 per event, in events.hex order) for the
            // spatial dropviz (fifo_dropviz --dropmask): faithful RTL drop set.
            fmask = $fopen({`OUT, "_dropmask.txt"}, "w");
            if (fmask != 0) begin
                for (rr = 0; rr < ev_n; rr = rr + 1)
                    $fdisplay(fmask, "%0d", dropmask[rr]);
                $fclose(fmask);
            end
            $finish;
        end
        if (watchdog > 30000000) begin
            $display("FAIL watchdog started=%0d push_i=%0d/%0d prod_row=%0d rec=%0d",
                     started, push_i, ev_n, prod_row, rec_emitted);
            $finish;
        end
    end

endmodule
