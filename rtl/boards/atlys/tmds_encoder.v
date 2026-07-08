// TMDS 8b/10b encoder (DVI 1.0 spec figure 3-5): transition-minimised XOR/
// XNOR stage followed by DC-balancing disparity control. One symbol per
// pixel clock.

module tmds_encoder (
    input  wire       clk,
    input  wire       de,
    input  wire [7:0] d,      // pixel data
    input  wire [1:0] c,      // {c1, c0} control (hsync/vsync on channel 0)
    output reg  [9:0] q
);

    // ones count of d
    wire [3:0] n1d = {3'd0, d[0]} + {3'd0, d[1]} + {3'd0, d[2]} + {3'd0, d[3]} +
                     {3'd0, d[4]} + {3'd0, d[5]} + {3'd0, d[6]} + {3'd0, d[7]};
    wire use_xnor = (n1d > 4'd4) || (n1d == 4'd4 && d[0] == 1'b0);

    wire [8:0] q_m;
    assign q_m[0] = d[0];
    assign q_m[1] = use_xnor ? ~(q_m[0] ^ d[1]) : (q_m[0] ^ d[1]);
    assign q_m[2] = use_xnor ? ~(q_m[1] ^ d[2]) : (q_m[1] ^ d[2]);
    assign q_m[3] = use_xnor ? ~(q_m[2] ^ d[3]) : (q_m[2] ^ d[3]);
    assign q_m[4] = use_xnor ? ~(q_m[3] ^ d[4]) : (q_m[3] ^ d[4]);
    assign q_m[5] = use_xnor ? ~(q_m[4] ^ d[5]) : (q_m[4] ^ d[5]);
    assign q_m[6] = use_xnor ? ~(q_m[5] ^ d[6]) : (q_m[5] ^ d[6]);
    assign q_m[7] = use_xnor ? ~(q_m[6] ^ d[7]) : (q_m[6] ^ d[7]);
    assign q_m[8] = ~use_xnor;

    wire [3:0] n1qm = {3'd0, q_m[0]} + {3'd0, q_m[1]} + {3'd0, q_m[2]} +
                      {3'd0, q_m[3]} + {3'd0, q_m[4]} + {3'd0, q_m[5]} +
                      {3'd0, q_m[6]} + {3'd0, q_m[7]};
    wire [3:0] n0qm = 4'd8 - n1qm;

    reg signed [4:0] cnt;     // running disparity

    always @(posedge clk) begin
        if (!de) begin
            cnt <= 5'sd0;
            case (c)
                2'b00: q <= 10'b1101010100;
                2'b01: q <= 10'b0010101011;
                2'b10: q <= 10'b0101010100;
                default: q <= 10'b1010101011;
            endcase
        end else begin
            if (cnt == 0 || n1qm == n0qm) begin
                q <= {~q_m[8], q_m[8], q_m[8] ? q_m[7:0] : ~q_m[7:0]};
                if (q_m[8])
                    cnt <= cnt + $signed({1'b0, n1qm}) - $signed({1'b0, n0qm});
                else
                    cnt <= cnt + $signed({1'b0, n0qm}) - $signed({1'b0, n1qm});
            end else if ((cnt > 0 && n1qm > n0qm) || (cnt < 0 && n0qm > n1qm)) begin
                q <= {1'b1, q_m[8], ~q_m[7:0]};
                cnt <= cnt + {3'd0, q_m[8], 1'b0}
                           + $signed({1'b0, n0qm}) - $signed({1'b0, n1qm});
            end else begin
                q <= {1'b0, q_m[8], q_m[7:0]};
                cnt <= cnt - {3'd0, ~q_m[8], 1'b0}
                           + $signed({1'b0, n1qm}) - $signed({1'b0, n0qm});
            end
        end
    end

endmodule
