// judge_unit liveness fuzzer (diagnostic).
// The live board loses judge requests systematically (watchdog fires at a
// visible rate; without the watchdog the pass freezes in the S_CONT judge
// hold). The FSM reads as control-flow-bounded, so hunt for (a) operand
// values that break completion (live moments can exceed the corpus envelope:
// u34-wrapped sums, n near 2^18, Cauchy-Schwarz violations from wrapped
// fields => v_ma underflow) and (b) protocol races (start pulses landing at
// every offset around done/busy edges). PASS criterion: every start gets
// exactly one done within 1000 cycles, and no spurious done.

`timescale 1ns / 1ps

module tb_judge_fuzz;
    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg         start;
    reg  [17:0] n;
    reg  [29:0] xs, ys;
    reg  [40:0] xss, yss, xys;
    wire        busy, done, accept;

    judge_unit dut (
        .clk(clk), .rst(rst), .en(1'b1),
        .start(start), .n(n), .xs(xs), .ys(ys),
        .xss(xss), .yss(yss), .xys(xys),
        .pix_th(18'd15), .mps_2sq(5'd2),
        .busy(busy), .done(done), .accept(accept)
    );

    integer seed = 42;
    integer cases = 0, failures = 0, spurious = 0;
    integer wait_cy;
    integer gap;
    reg [31:0] r0, r1, r2;

    task run_case(input [17:0] tn, input [29:0] txs, input [29:0] tys,
                  input [40:0] txss, input [40:0] tyss, input [40:0] txys,
                  input integer post_gap);
        begin
            // drive on negedge: half a cycle of setup, no active-region race
            @(negedge clk);
            n = tn; xs = txs; ys = tys; xss = txss; yss = tyss; xys = txys;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait_cy = 0;
            while (!done && wait_cy < 1000) begin
                @(posedge clk);
                wait_cy = wait_cy + 1;
            end
            if (!done) begin
                failures = failures + 1;
                if (failures <= 10)
                    $display("LIVENESS FAIL #%0d: n=%0d xs=%0d ys=%0d xss=%0d yss=%0d xys=%0d (case %0d)",
                             failures, tn, txs, tys, txss, tyss, txys, cases);
            end
            // gap cycles before the next start (0 = back-to-back at done)
            repeat (post_gap) @(posedge clk);
            cases = cases + 1;
        end
    endtask

    // spurious-done monitor (done outside a pending request window is a bug)
    integer inflight = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (start) inflight = inflight + 1;
            if (done) begin
                if (inflight == 0) spurious = spurious + 1;
                else inflight = inflight - 1;
            end
        end
    end

    integer i;
    initial begin
        start = 0; n = 0; xs = 0; ys = 0; xss = 0; yss = 0; xys = 0;
        repeat (6) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- directed extremes --------------------------------------------
        run_case(18'd0, 0, 0, 0, 0, 0, 2);                       // n=0 early reject
        run_case(18'd14, 30'h3FFFFFFF, 30'h3FFFFFFF,
                 41'h1FFFFFFFFFF, 41'h1FFFFFFFFFF, 41'h1FFFFFFFFFF, 2); // reject, max fields
        run_case(18'h3FFFF, 30'h3FFFFFFF, 30'h3FFFFFFF,
                 41'h1FFFFFFFFFF, 41'h1FFFFFFFFFF, 41'h1FFFFFFFFFF, 2); // all-max
        run_case(18'h3FFFF, 0, 0, 0, 0, 0, 2);                   // huge n, zero sums
        run_case(18'd16, 30'h3FFFFFFF, 30'h3FFFFFFF, 0, 0, 0, 2); // CS violation:
                                                                  // Sx^2 >> N*Sxx (v_ma wraps)
        run_case(18'd100, 30'd100000, 30'd100000,
                 41'd5, 41'd5, 41'h1FFFFFFFFFF, 2);              // mb huge vs tiny ma/mc
        // u34-wrap simulation: squares truncated small, sums big
        run_case(18'd50000, 30'd20000000, 30'd20000000,
                 41'd12345, 41'd678, 41'd42, 2);

        // ---- protocol races: back-to-back starts at every gap --------------
        for (i = 0; i < 12; i = i + 1)
            run_case(18'd20, 30'd2000, 30'd2000, 41'd400000, 41'd400000,
                     41'd400000, i % 4);   // gaps 0..3 right after done
        for (i = 0; i < 12; i = i + 1)
            run_case(18'd3, 0, 0, 0, 0, 0, i % 3); // early-rejects back-to-back

        // ---- random fuzz ----------------------------------------------------
        for (i = 0; i < 200000; i = i + 1) begin
            r0 = $random(seed); r1 = $random(seed); r2 = $random(seed);
            run_case(r0[17:0],
                     {r0[31:16], r1[13:0]}, {r1[31:16], r2[13:0]},
                     {r2[31:24], r0[23:0], r1[8:0]},
                     {r1[23:16], r2[23:0], r0[8:0]},
                     {r0[15:8], r1[23:0], r2[8:0]},
                     r2[1:0]);
        end

        if (failures == 0 && spurious == 0)
            $display("FUZZ PASS: %0d cases, all done pulses arrived, no spurious", cases);
        else
            $display("FUZZ FAIL: %0d cases, %0d liveness failures, %0d spurious dones",
                     cases, failures, spurious);
        $finish;
    end

    integer wd = 0;
    always @(posedge clk) begin
        wd = wd + 1;
        if (wd > 300000000) begin
            $display("FUZZ WATCHDOG at case %0d", cases);
            $finish;
        end
    end

endmodule
