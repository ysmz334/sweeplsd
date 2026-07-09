// Parity testbench: stage_edge vs golden edge vectors, driven from the golden
// gradient vectors. See run_tb.sh (POWER_TH and STRICT come from the meta
// file; STRICT=1 selects the improved-mode NMS tie-break vectors).

`timescale 1ns / 1ps

`ifndef STRICT
`define STRICT 0
`endif
`ifndef HYST_ON
`define HYST_ON 0
`endif
`ifndef HYST_ADAPT
`define HYST_ADAPT 0
`endif
`ifndef HYST_LOW
`define HYST_LOW 120
`endif
`ifndef EDGE_BORDER
`define EDGE_BORDER 3
`endif

module tb_edge;
    localparam integer W = `IMG_W;
    localparam integer H = `IMG_H;
    localparam integer XW = 12;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg en = 1'b0;
    reg [XW-1:0] X = 0, Y = 0;
    wire [XW-1:0] x_next = (X == W + 5) ? {XW{1'b0}} : X + 1'b1;

    localparam [15:0] PTH = `POWER_TH;

    reg [15:0] power [0:W*H-1];
    reg [0:0]  dir [0:W*H-1];
    reg [0:0]  gold [0:W*H-1];

    wire iv = (X >= 3) && (Y >= 3) && (X - 3 < W) && (Y - 3 < H);
    wire [15:0] ip = iv ? power[(Y - 3) * W + (X - 3)] : 16'd0;
    wire        id = iv ? dir[(Y - 3) * W + (X - 3)][0] : 1'b0;

    wire        o_valid;
    wire [XW-1:0] o_x, o_y;
    wire        o_e;
    wire        o_strong;

    stage_edge #(.MAXW_LOG2(11), .XW(XW)) dut (
        .clk(clk), .rst(rst), .frame_start(1'b0), .en(en),
        .X(X), .Y(Y), .x_next(x_next),
        .width(W[XW-1:0]), .height(H[XW-1:0]),
        .power_th(PTH), .strict(`STRICT != 0),
        .hyst_on(`HYST_ON != 0), .hyst_adaptive(`HYST_ADAPT != 0),
        .hyst_low(16'd`HYST_LOW), .edge_border(4'd`EDGE_BORDER),
        .i_valid(iv), .i_power(ip), .i_dir(id),
        .o_valid(o_valid), .o_x(o_x), .o_y(o_y), .o_e(o_e), .o_strong(o_strong)
    );

    always #5 clk = ~clk;

    integer errors = 0;
    integer checked = 0;

    always @(posedge clk) begin
        if (!rst && en) begin
            if (o_valid) begin
                checked = checked + 1;
                if (o_e !== gold[o_y * W + o_x][0]) begin
                    if (errors == 0)
                        $display("FAIL: first mismatch at (%0d,%0d): rtl %0d gold %0d",
                                 o_x, o_y, o_e, gold[o_y * W + o_x]);
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
        $readmemh({`VEC, "_power.hex"}, power);
        $readmemh({`VEC, "_dir.hex"}, dir);
        $readmemh({`VEC, "_edge.hex"}, gold);
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        en <= 1'b1;
    end

endmodule
