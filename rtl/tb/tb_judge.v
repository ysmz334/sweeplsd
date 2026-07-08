// Self-checking testbench for judge_unit: drives moments accumulated from
// constrained-random integer point clouds (guaranteeing a valid PSD scatter,
// like real labels) plus directed edge cases, and compares the accept
// decision against the exact reference formula evaluated in wide registers.
// Standalone (no vector files):
//   iverilog -g2005 -o t.vvp tb/tb_judge.v core/judge_unit.v && vvp t.vvp

`timescale 1ns / 1ps

`ifndef CE_DIV
`define CE_DIV 1
`endif

module tb_judge;
    reg clk = 1'b0;
    reg rst = 1'b1;

    // clock enable (1-in-CE_DIV; see tb_sweep_core.v)
    reg [3:0] ce_cnt = 4'd0;
    always @(posedge clk) ce_cnt <= (ce_cnt == `CE_DIV - 1) ? 4'd0 : ce_cnt + 4'd1;
    wire ce = (ce_cnt == 4'd0);

    reg         start = 1'b0;
    reg  [17:0] n;
    reg  [29:0] xs, ys;
    reg  [40:0] xss, yss, xys;
    reg  [17:0] pix_th = 18'd15;

    wire busy, done, accept;

    judge_unit dut (
        .clk(clk), .rst(rst), .en(ce),
        .start(start), .n(n), .xs(xs), .ys(ys),
        .xss(xss), .yss(yss), .xys(xys), .pix_th(pix_th),
        .mps_2sq(5'd0),  // (h) off: this golden covers the baseline aspect test
        .busy(busy), .done(done), .accept(accept)
    );

    always #5 clk = ~clk;

    // Exact reference in wide regs.
    reg [127:0] ma, mb_p, mb_m, mc, T, dabs, mbabs, t2, r2, lhs, rhs;
    reg ref_accept;
    task compute_ref;
        begin
            ma = n * xss - xs * xs;
            mc = n * yss - ys * ys;
            mb_p = n * xys;
            mb_m = xs * ys;
            mbabs = (mb_p >= mb_m) ? (mb_p - mb_m) : (mb_m - mb_p);
            T = ma + mc;
            dabs = (ma >= mc) ? (ma - mc) : (mc - ma);
            t2 = T * T;
            r2 = dabs * dabs + 4 * mbabs * mbabs;
            lhs = 361 * t2;
            rhs = 441 * r2;
            ref_accept = (n >= pix_th) && (T != 0) && !(lhs > rhs);
        end
    endtask

    integer errors = 0;
    integer tests = 0;

    task run_one;
        begin
            compute_ref;
            @(negedge clk);
            start <= 1'b1;
            repeat (`CE_DIV) @(negedge clk);   // span exactly one en cycle
            start <= 1'b0;
            while (!done) @(posedge clk);
            tests = tests + 1;
            if (accept !== ref_accept) begin
                errors = errors + 1;
                if (errors <= 5)
                    $display("FAIL: n=%0d xs=%0d ys=%0d xss=%0d yss=%0d xys=%0d: rtl %0d ref %0d",
                             n, xs, ys, xss, yss, xys, accept, ref_accept);
            end
            @(posedge clk);
        end
    endtask

    // Accumulate a random point cloud: a noisy line segment (mostly accepts)
    // or a blob (mostly rejects), with coordinates < 2048.
    integer i, k, npts, x0, y0, ddx, ddy, px, py, noise;
    reg [31:0] seed;
    task gen_cloud(input integer line_like);
        begin
            npts = 8 + ($random(seed) & 32'h3f);           // 8..71 points
            n = 0; xs = 0; ys = 0; xss = 0; yss = 0; xys = 0;
            x0 = $random(seed) & 32'h3ff;
            y0 = $random(seed) & 32'h3ff;
            ddx = ($random(seed) % 5);
            ddy = ($random(seed) % 5);
            if (line_like && ddx == 0 && ddy == 0) ddx = 3;
            noise = line_like ? 1 : 200;
            for (i = 0; i < npts; i = i + 1) begin
                if (line_like) begin
                    px = x0 + (i * ddx) / 2 + ($random(seed) % noise);
                    py = y0 + (i * ddy) / 2 + ($random(seed) % noise);
                end else begin
                    px = x0 + ($random(seed) % noise);
                    py = y0 + ($random(seed) % noise);
                end
                if (px < 0) px = 0;
                if (px > 2047) px = 2047;
                if (py < 0) py = 0;
                if (py > 2047) py = 2047;
                n = n + 1;
                xs = xs + px;
                ys = ys + py;
                xss = xss + px * px;
                yss = yss + py * py;
                xys = xys + px * py;
            end
        end
    endtask

    initial begin
        seed = 32'd12345;
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);

        // Directed: degenerate single-position label (T == 0 -> reject).
        n = 18'd20; xs = 20 * 100; ys = 20 * 200;
        xss = 20 * 100 * 100; yss = 20 * 200 * 200; xys = 20 * 100 * 200;
        run_one;
        // Directed: below pixel threshold.
        gen_cloud(1); n = 18'd5; run_one;
        // Directed: perfect horizontal line (accept).
        n = 0; xs = 0; ys = 0; xss = 0; yss = 0; xys = 0;
        for (k = 0; k < 30; k = k + 1) begin
            n = n + 1; xs = xs + (50 + k); ys = ys + 77;
            xss = xss + (50 + k) * (50 + k); yss = yss + 77 * 77;
            xys = xys + (50 + k) * 77;
        end
        run_one;

        for (k = 0; k < 300; k = k + 1) begin
            gen_cloud(k & 1);
            run_one;
        end

        if (errors == 0)
            $display("PASS: %0d judge cases match the exact reference", tests);
        else
            $display("FAIL: %0d/%0d mismatches", errors, tests);
        $finish;
    end

endmodule
