// GPRES-pattern histogram testbench (diagnostic).
// Samples the 4-neighbour presence mask gpres = {NE,N,NW,W} once per labelled
// pixel (S_GATH0, gi==0 — single cycle since Lever 3) and prints the 16-bin
// pattern histogram. Used to quantify the CCL decision-tree pruning lever:
// among present neighbours, connected groups provably share one root
// (adjacent pixels of row y-1 were merged when the right one was processed;
// W's NE-neighbour merge links W to N; W's N-neighbour links W to NW), so
// only one find per connected group is needed:
//   W present               -> group {W,NW,N,(NE if N)} = w_sav, 0 finds;
//                              + find(NE) only if NE && !N
//   W absent, N present     -> 1 find (N covers NW and NE)
//   W absent, N absent      -> find(NW) + find(NE) independently
// Golden-record check kept as sanity. Based on tb_backend_prof.v.

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

module tb_backend_gpres;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;
    localparam integer MAXEV = 2 * W * H + H + 4;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;
    wire ce = 1'b1;

    // ---- event source -------------------------------------------------------
    reg [15:0] ev_mem [0:MAXEV-1];
    integer ev_n, ev_i;
    wire ev_empty = (ev_i >= ev_n);
    wire [1:0] ev_kind = ev_mem[ev_i][13:12];
    wire [XW-1:0] ev_x = ev_mem[ev_i][XW-1:0];
    wire ev_strong = ev_mem[ev_i][14];
    wire ev_pop;
    always @(posedge clk) if (!rst && ev_pop) ev_i <= ev_i + 1;

    // ---- DUT ----------------------------------------------------------------
    wire        rec_valid;
    wire [10:0] rec_sx, rec_sy, rec_ex, rec_ey;
    wire [17:0] rec_n;
    wire [29:0] rec_xs, rec_ys;
    wire [40:0] rec_xss, rec_yss, rec_xys;
    wire [10:0] rec_minx, rec_minx_y, rec_maxx, rec_maxx_y;
    wire [10:0] rec_miny, rec_miny_x, rec_maxy, rec_maxy_x;

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

    integer ri = 0;
    reg frame_done = 1'b0;
    always @(posedge clk) begin
        if (!rst && rec_valid) begin
            if (rec_n == 18'd0) frame_done <= 1'b1;
            else ri = ri + 1;
        end
    end

    // ---- gpres pattern histogram (one sample per pixel at gi==0) ------------
    integer hist [0:15];
    integer npix;
    reg [3:0] gp;
    always @(posedge clk) begin
        if (!rst && !frame_done && dut.state == 6'd10 && dut.gi == 3'd0) begin
            gp = dut.gpres;
            hist[gp] = hist[gp] + 1;
            npix = npix + 1;
        end
    end

    // finds today vs pruned, computed from the histogram at the end
    integer k, cyc;
    integer f_today, f_pruned, folds, w_only, nofind_pat;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (frame_done) begin
            $display("RESULT records=%0d pixels=%0d", ri, npix);
            f_today = 0; f_pruned = 0; folds = 0; w_only = 0; nofind_pat = 0;
            for (k = 0; k < 16; k = k + 1) begin
                if (hist[k] != 0)
                    $display("PAT %b (NE,N,NW,W) = %0d", k[3:0], hist[k]);
                // today: one find per present neighbour among NE(3),N(2),NW(1)
                f_today = f_today + hist[k] * (k[3] + k[2] + k[1]);
                // pruned decision tree
                if (k[0])       f_pruned = f_pruned + hist[k] * ((k[3] && !k[2]) ? 1 : 0);
                else if (k[2])  f_pruned = f_pruned + hist[k];
                else            f_pruned = f_pruned + hist[k] * (k[3] + k[1]);
                if (k[0]) folds = folds + hist[k];
                if (k == 4'b0001) w_only = w_only + hist[k];
                // patterns needing no find at all under pruning
                if ((k[0] && !(k[3] && !k[2])) || k == 0) nofind_pat = nofind_pat + hist[k];
            end
            $display("FINDS today=%0d pruned=%0d folds(W)=%0d w_only=%0d nofind_pixels=%0d",
                     f_today, f_pruned, folds, w_only, nofind_pat);
            $finish;
        end
        if (cyc > 20000000) begin
            $display("FAIL watchdog state=%0d ev %0d/%0d", dut.state, ev_i, ev_n);
            $finish;
        end
    end

    integer i;
    initial begin
        for (i = 0; i < 16; i = i + 1) hist[i] = 0;
        npix = 0; cyc = 0; ev_i = 0;
        for (i = 0; i < MAXEV; i = i + 1) ev_mem[i] = 16'hxxxx;
        $readmemh({`VEC, "_events.hex"}, ev_mem);
        ev_n = 0;
        for (i = 0; i < MAXEV; i = i + 1)
            if (ev_mem[i] !== 16'hxxxx) ev_n = ev_n + 1;
        repeat (6) @(posedge clk);
        rst <= 1'b0;
    end

endmodule
