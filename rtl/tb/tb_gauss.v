// Parity testbench: stage_gauss vs the golden gaussian vectors
// (rtl/tools/dump_vectors.cpp). Compile with:
//   iverilog -g2005 -DIMG_W=.. -DIMG_H=.. -DVEC='"vectors/<name>"' \
//            tb/tb_gauss.v core/stage_gauss.v
// Fails (exit via $fatal-style banner) on the first mismatching sample.

`timescale 1ns / 1ps

module tb_gauss;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg en = 1'b0;
    reg [XW-1:0] X = 0, Y = 0;
    wire [XW-1:0] x_next = (X == W + 5) ? {XW{1'b0}} : X + 1'b1;

    reg [7:0]  gray [0:W*H-1];
    reg [15:0] gold [0:W*H-1];

    wire [7:0] px = (X < W && Y < H) ? gray[Y * W + X] : 8'd0;

    wire        o_valid;
    wire [XW-1:0] o_x, o_y;
    wire [13:0] o_g;

    stage_gauss #(.MAXW_LOG2(11), .XW(XW)) dut (
        .clk(clk), .rst(rst), .en(en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .px(px),
        .o_valid(o_valid), .o_x(o_x), .o_y(o_y), .o_g(o_g)
    );

    always #5 clk = ~clk;

    integer errors = 0;
    integer checked = 0;

    always @(posedge clk) begin
        if (!rst && en) begin
            if (o_valid) begin
                checked = checked + 1;
                if (o_g !== gold[o_y * W + o_x][13:0]) begin
                    if (errors == 0)
                        $display("FAIL: first mismatch at (%0d,%0d): rtl %0d gold %0d",
                                 o_x, o_y, o_g, gold[o_y * W + o_x]);
                    errors = errors + 1;
                end
            end
            if (X == W + 5) begin
                X <= 0;
                Y <= Y + 1'b1;
            end else begin
                X <= X + 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (Y == H + 6) begin
            if (errors == 0 && checked == W * H)
                $display("PASS: %0d samples bit-exact", checked);
            else
                $display("FAIL: %0d errors, %0d/%0d checked", errors, checked, W * H);
            $finish;
        end
    end

    initial begin
        $readmemh({`VEC, "_gray.hex"}, gray);
        $readmemh({`VEC, "_gauss.hex"}, gold);
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        en <= 1'b1;
    end

endmodule
