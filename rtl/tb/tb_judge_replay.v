// Replay RTL-dumped judge dispatches into the judge (RTL or gate-level with
// -DGL) to find the dispatch after which the netlist judge stops answering
// (the live/GL "42 records then silence" bug). Reads tb/replay_ops.txt:
// one dispatch per line, hex: n xs ys xss yss xys gap.

`timescale 1ns / 1ps

module tb_judge_replay;
    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg         start;
    reg  [17:0] n;
    reg  [29:0] xs, ys;
    reg  [40:0] xss, yss, xys;
    wire busy, done, accept;

    judge_unit dut (
        .clk(clk), .rst(rst), .en(1'b1),
        .start(start), .n(n), .xs(xs), .ys(ys),
        .xss(xss), .yss(yss), .xys(xys), .pix_th(18'd15),
        .mps_2sq(5'd2),
        .busy(busy), .done(done), .accept(accept)
    );

    reg [255:0] mem_n [0:511];   // parsed fields (oversized regs, one file)
    integer f, cnt, i, r;
    reg [17:0]  a_n   [0:511];
    reg [29:0]  a_xs  [0:511];
    reg [29:0]  a_ys  [0:511];
    reg [40:0]  a_xss [0:511];
    reg [40:0]  a_yss [0:511];
    reg [40:0]  a_xys [0:511];
    reg [31:0]  a_gap [0:511];

    integer tmo, lat;
    initial begin
        f = $fopen("tb/replay_ops.txt", "r");
        cnt = 0;
        while (!$feof(f) && cnt < 512) begin
            r = $fscanf(f, "%h %h %h %h %h %h %h\n",
                        a_n[cnt], a_xs[cnt], a_ys[cnt],
                        a_xss[cnt], a_yss[cnt], a_xys[cnt], a_gap[cnt]);
            if (r == 7) cnt = cnt + 1;
        end
        $display("loaded %0d dispatches", cnt);
        start = 0;
        repeat (8) @(posedge clk);
        rst = 0;
        repeat (4) @(posedge clk);

        for (i = 0; i < cnt; i = i + 1) begin
            // inter-dispatch gap (compressed: min(gap,200))
            repeat ((a_gap[i] > 200) ? 200 : a_gap[i]) @(posedge clk);
            n <= a_n[i]; xs <= a_xs[i]; ys <= a_ys[i];
            xss <= a_xss[i]; yss <= a_yss[i]; xys <= a_xys[i];
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;
            // wait for done with timeout
            tmo = 0; lat = 0;
            while (!done && tmo < 3000) begin
                @(posedge clk);
                tmo = tmo + 1;
            end
            lat = tmo;
            if (!done && tmo >= 3000) begin
                $display("HANG at dispatch %0d (n=%0d xss=%0d yss=%0d xys=%0d)",
                         i + 1, a_n[i], a_xss[i], a_yss[i], a_xys[i]);
                $finish;
            end
            if (lat > 60)
                $display("dispatch %0d lat %0d n=%0d", i + 1, lat, a_n[i]);
            @(posedge clk);
        end
        $display("PASS: all %0d dispatches completed", cnt);
        $finish;
    end

    // global watchdog
    integer wcyc = 0;
    always @(posedge clk) begin
        wcyc = wcyc + 1;
        if (wcyc > 3000000) begin
            $display("FAIL: global watchdog");
            $finish;
        end
    end

endmodule
