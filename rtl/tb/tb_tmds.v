// Self-checking testbench for tmds_encoder: every encoded data symbol must
// decode back to the input byte (DVI 1.0 decode rule), control symbols must
// match the four spec codes, and the running DC disparity over a long random
// stream must stay bounded (|running ones - zeros| <= 10).
//   iverilog -g2005 -o t.vvp tb/tb_tmds.v boards/atlys/tmds_encoder.v && vvp t.vvp

`timescale 1ns / 1ps

module tb_tmds;
    reg clk = 0;
    reg de = 0;
    reg [7:0] d = 0;
    reg [1:0] c = 0;
    wire [9:0] q;

    tmds_encoder dut (.clk(clk), .de(de), .d(d), .c(c), .q(q));

    always #5 clk = ~clk;

    reg [7:0] d_d2;         // q lags the input by one clock
    reg de_d2;
    reg [1:0] c_d2;
    always @(posedge clk) begin
        d_d2 <= d;
        de_d2 <= de;
        c_d2 <= c;
    end

    // DVI decode of q
    function [7:0] decode(input [9:0] s);
        reg [7:0] v;
        integer i;
        begin
            v = s[9] ? ~s[7:0] : s[7:0];
            decode[0] = v[0];
            for (i = 1; i < 8; i = i + 1)
                decode[i] = s[8] ? (v[i] ^ v[i-1]) : ~(v[i] ^ v[i-1]);
        end
    endfunction

    integer disparity = 0;
    integer i, errors = 0, tests = 0;
    integer ones;
    reg [31:0] seed = 32'd777;

    // DC-balance accounting over EVERY symbol (control resets the encoder's
    // disparity counter, so mirror that); checked against the +-10 bound.
    reg started = 0;
    always @(posedge clk) begin
        if (started) begin
            if (!de_d2) disparity = 0;
            else begin
                disparity = disparity +
                    (2 * (q[0]+q[1]+q[2]+q[3]+q[4]+q[5]+q[6]+q[7]+q[8]+q[9]) - 10);
                if (disparity > 10 || disparity < -10) begin
                    if (errors == 0)
                        $display("FAIL: disparity %0d out of bounds", disparity);
                    errors = errors + 1;
                end
            end
        end
    end

    task check_q;
        begin
            tests = tests + 1;
            if (de_d2) begin
                if (decode(q) !== d_d2) begin
                    if (errors == 0)
                        $display("FAIL: %02x encoded %010b decodes to %02x",
                                 d_d2, q, decode(q));
                    errors = errors + 1;
                end
            end else begin
                if (q !== (c_d2 == 2'b00 ? 10'b1101010100 :
                           c_d2 == 2'b01 ? 10'b0010101011 :
                           c_d2 == 2'b10 ? 10'b0101010100 : 10'b1010101011)) begin
                    if (errors == 0) $display("FAIL: ctrl %b -> %010b", c_d2, q);
                    errors = errors + 1;
                end
            end
        end
    endtask

    initial begin
        @(posedge clk); @(posedge clk); @(posedge clk);
        started = 1;
        // all byte values, then random stream with DE gaps
        de <= 1;
        for (i = 0; i < 256; i = i + 1) begin
            d <= i[7:0];
            @(posedge clk);
            if (i >= 2) check_q;
        end
        for (i = 0; i < 20000; i = i + 1) begin
            de <= ($random(seed) % 100) < 92;   // blanking mixed in
            d <= $random(seed) & 8'hff;
            c <= $random(seed) & 2'b11;
            @(posedge clk);
            check_q;
        end
        if (errors == 0)
            $display("PASS: %0d symbols decode-exact, DC balanced", tests);
        else
            $display("FAIL: %0d errors", errors);
        $finish;
    end

endmodule
