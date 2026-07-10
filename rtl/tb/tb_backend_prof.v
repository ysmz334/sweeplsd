// PROFILING testbench (temporary, not committed): backend + judge_unit driven
// from the golden event stream, with a per-state occupancy histogram to
// re-derive the cy/interior breakdown after the gather-parallel-skip +
// judge Lever1 optimisations. Run with CE_DIV=1 (fastest supply). Prints the
// grouped state occupancy on finish. Based on tb_backend.v.

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

module tb_backend_prof;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam integer MAXEV = 2 * W * H + H + 4;
    localparam integer MAXREC = 8192;

    reg clk = 1'b0;
    reg rst = 1'b1;

    reg [3:0] ce_cnt = 4'd0;
    always @(posedge clk) ce_cnt <= (ce_cnt == `CE_DIV - 1) ? 4'd0 : ce_cnt + 4'd1;
    wire ce = (ce_cnt == 4'd0);

    // ---- event FIFO model ---------------------------------------------------
    reg [15:0] ev_mem [0:MAXEV-1];
    integer ev_n;
    integer ev_i;
    wire ev_empty = (ev_i >= ev_n);
    wire [1:0] ev_kind = ev_mem[ev_i][13:12];
    wire [XW-1:0] ev_x = ev_mem[ev_i][XW-1:0];
    wire ev_strong = ev_mem[ev_i][14];
    wire ev_pop;

    always @(posedge clk) if (!rst && ev_pop && ce) ev_i <= ev_i + 1;

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

    // ---- per-state occupancy histogram (CE_DIV=1: one FSM step / cycle) -----
    integer st_hist [0:63];
    integer active_cyc;
    integer hk;
    always @(posedge clk) begin
        if (!rst && !frame_done && ce) begin
            st_hist[dut.state] = st_hist[dut.state] + 1;
            active_cyc = active_cyc + 1;
        end
    end

    // watchdog + finish + report
    integer cyc = 0;
    integer n_int, n_end, n_eor;
    // grouped sums
    integer g_init, g_ingest, g_rowsetup, g_fetch, g_gather, g_create,
            g_adopt, g_merge, g_acc, g_cont, g_scav, g_rowend, g_term;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (frame_done) begin
            $display("RESULT records=%0d errors=%0d", ri, errors);
            $display("EVENTS int=%0d end=%0d eor=%0d", n_int, n_end, n_eor);
            $display("ACTIVE_CYC %0d", active_cyc);
            // raw per-state
            for (hk = 0; hk < 32; hk = hk + 1)
                if (st_hist[hk] != 0) $display("STATE %0d = %0d", hk, st_hist[hk]);
            // grouped
            g_init     = st_hist[0];
            g_ingest   = st_hist[1] + st_hist[2];                       // POP,EV
            g_rowsetup = st_hist[3];                                    // ROW0
            g_fetch    = st_hist[4]+st_hist[5]+st_hist[6]+st_hist[7]+
                         st_hist[8]+st_hist[9]+st_hist[23]+st_hist[25]+
                         st_hist[26]+st_hist[27];
                         // RNEXT,RW,RCAP,RD1-3,RX(c),RB,RC,RC2(fast-path wait)
            g_gather   = st_hist[10]+st_hist[11]+st_hist[12];           // GATH0,GWAIT,GATH1
            g_create   = st_hist[13]+st_hist[14];                       // CWAIT,CREATE
            g_adopt    = st_hist[19]+st_hist[20];                       // ARD,ACAP
            g_merge    = st_hist[15]+st_hist[16]+st_hist[17]+st_hist[18];// MRDA,MCAPA,MCAPB,MEXEC
            g_acc      = st_hist[21];                                   // ACC
            g_cont     = st_hist[22];                                   // CONT
            g_scav     = st_hist[24];                                   // SCAVP
            g_rowend   = st_hist[29];                                   // ROWEND
            g_term     = st_hist[30]+st_hist[31];                       // TERM,HALT
            $display("GROUP init=%0d ingest=%0d rowsetup=%0d fetch=%0d gather=%0d create=%0d adopt=%0d merge=%0d acc=%0d cont=%0d scav=%0d rowend=%0d term=%0d",
                     g_init, g_ingest, g_rowsetup, g_fetch, g_gather, g_create,
                     g_adopt, g_merge, g_acc, g_cont, g_scav, g_rowend, g_term);
            $finish;
        end
        if (cyc > 20000000) begin
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
        for (i = 0; i < 64; i = i + 1) st_hist[i] = 0;
        active_cyc = 0;
        n_int = 0; n_end = 0; n_eor = 0;
        ev_i = 0;
        for (i = 0; i < MAXEV; i = i + 1) ev_mem[i] = 16'hxxxx;
        $readmemh({`VEC, "_events.hex"}, ev_mem);
        ev_n = 0;
        for (i = 0; i < MAXEV; i = i + 1)
            if (ev_mem[i] !== 16'hxxxx) begin
                ev_n = ev_n + 1;
                case (ev_mem[i][13:12])
                    2'd1: n_int = n_int + 1;   // K_INT
                    2'd2: n_end = n_end + 1;   // K_END
                    2'd3: n_eor = n_eor + 1;   // K_EOR
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
