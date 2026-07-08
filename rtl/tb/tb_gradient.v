// Parity testbench: stage_gradient vs golden power/dir vectors, driven from
// the golden gaussian vectors. See run_tb.sh.

`timescale 1ns / 1ps

module tb_gradient;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg en = 1'b0;
    reg [XW-1:0] X = 0, Y = 0;
    wire [XW-1:0] x_next = (X == W + 5) ? {XW{1'b0}} : X + 1'b1;

    reg [15:0] gauss [0:W*H-1];
    reg [15:0] gold_p [0:W*H-1];
    reg [0:0]  gold_d [0:W*H-1];

    // Producer emulation: gaussian sample of (X-2, Y-2).
    wire iv = (X >= 2) && (Y >= 2) && (X - 2 < W) && (Y - 2 < H);
    wire [13:0] ig = iv ? gauss[(Y - 2) * W + (X - 2)][13:0] : 14'd0;

    wire        o_valid;
    wire [XW-1:0] o_x, o_y;
    wire [15:0] o_power;
    wire        o_dir;

    stage_gradient #(.MAXW_LOG2(11), .XW(XW)) dut (
        .clk(clk), .rst(rst), .en(en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .i_valid(iv), .i_g(ig),
        .o_valid(o_valid), .o_x(o_x), .o_y(o_y),
        .o_power(o_power), .o_dir(o_dir)
    );

    always #5 clk = ~clk;

    integer errors = 0;
    integer checked = 0;

    always @(posedge clk) begin
        if (!rst && en) begin
            if (o_valid) begin
                checked = checked + 1;
                if (o_power !== gold_p[o_y * W + o_x] ||
                    o_dir !== gold_d[o_y * W + o_x][0]) begin
                    if (errors == 0)
                        $display("FAIL: first mismatch at (%0d,%0d): rtl p=%0d d=%0d gold p=%0d d=%0d",
                                 o_x, o_y, o_power, o_dir,
                                 gold_p[o_y * W + o_x], gold_d[o_y * W + o_x]);
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
        $readmemh({`VEC, "_gauss.hex"}, gauss);
        $readmemh({`VEC, "_power.hex"}, gold_p);
        $readmemh({`VEC, "_dir.hex"}, gold_d);
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        en <= 1'b1;
    end

endmodule
