// Parity testbench: stage_feature (+ endpoint_core) vs golden feature
// vectors, driven from the golden edge vectors. See run_tb.sh.

`timescale 1ns / 1ps

module tb_feature;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg en = 1'b0;
    reg [XW-1:0] X = 0, Y = 0;
    wire [XW-1:0] x_next = (X == W + 5) ? {XW{1'b0}} : X + 1'b1;

    reg [0:0] edgem [0:W*H-1];
    reg [1:0] gold [0:W*H-1];

    wire iv = (X >= 4) && (Y >= 4) && (X - 4 < W) && (Y - 4 < H);
    wire ie = iv ? edgem[(Y - 4) * W + (X - 4)][0] : 1'b0;

    wire        o_valid;
    wire [XW-1:0] o_x, o_y;
    wire [1:0]  o_f;
    wire        o_strong;

    stage_feature #(.MAXW_LOG2(11), .XW(XW)) dut (
        .clk(clk), .rst(rst), .en(en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .i_valid(iv), .i_e(ie), .i_strong(1'b0),
        .o_valid(o_valid), .o_x(o_x), .o_y(o_y), .o_f(o_f), .o_strong(o_strong)
    );

    always #5 clk = ~clk;

    integer errors = 0;
    integer checked = 0;

    always @(posedge clk) begin
        if (!rst && en) begin
            if (o_valid) begin
                checked = checked + 1;
                if (o_f !== gold[o_y * W + o_x]) begin
                    if (errors == 0)
                        $display("FAIL: first mismatch at (%0d,%0d): rtl %0d gold %0d",
                                 o_x, o_y, o_f, gold[o_y * W + o_x]);
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
        $readmemh({`VEC, "_edge.hex"}, edgem);
        $readmemh({`VEC, "_feat.hex"}, gold);
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        en <= 1'b1;
    end

endmodule
