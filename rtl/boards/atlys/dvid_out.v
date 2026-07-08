// DVI-D / HDMI output for Spartan-6: three TMDS encoders + 10:1 serialisers
// built from ODDR2 at 5x the pixel clock (2 bits per 5x cycle, LSB first),
// plus the pixel-rate clock channel. clk_pix and clk_x5 must come from the
// same PLL. Board pins are driven through OBUFDS (TMDS_33).
//
// Xilinx primitives (ODDR2/OBUFDS): board build only, not for iverilog.

module dvid_out (
    input  wire       clk_pix,
    input  wire       clk_x5,
    input  wire       rst,

    input  wire [7:0] red,
    input  wire [7:0] green,
    input  wire [7:0] blue,
    input  wire       de,
    input  wire       hs,
    input  wire       vs,

    output wire       tmds_tx_clk_p,
    output wire       tmds_tx_clk_n,
    output wire [2:0] tmds_tx_p,
    output wire [2:0] tmds_tx_n
);

    // ---- encode (pixel domain) ---------------------------------------------
    wire [9:0] sym_b, sym_g, sym_r;
    tmds_encoder enc_b (.clk(clk_pix), .de(de), .d(blue),  .c({vs, hs}), .q(sym_b));
    tmds_encoder enc_g (.clk(clk_pix), .de(de), .d(green), .c(2'b00),    .q(sym_g));
    tmds_encoder enc_r (.clk(clk_pix), .de(de), .d(red),   .c(2'b00),    .q(sym_r));

    // pixel-domain latch + phase toggle for the 5x domain
    reg [9:0] lat_b, lat_g, lat_r;
    reg       tog;
    always @(posedge clk_pix) begin
        lat_b <= sym_b;
        lat_g <= sym_g;
        lat_r <= sym_r;
        tog <= ~tog;
        if (rst) tog <= 1'b0;
    end

    // ---- serialise (5x domain, 2 bits per cycle, LSB first) -----------------
    reg tog_x5, tog_x5_d;
    reg [9:0] sh_b, sh_g, sh_r, sh_c;
    always @(posedge clk_x5) begin
        tog_x5 <= tog;
        tog_x5_d <= tog_x5;
        if (tog_x5_d != tog_x5) begin      // pixel boundary: load
            sh_b <= lat_b;
            sh_g <= lat_g;
            sh_r <= lat_r;
            sh_c <= 10'b0000011111;        // clock channel pattern
        end else begin
            sh_b <= {2'b00, sh_b[9:2]};
            sh_g <= {2'b00, sh_g[9:2]};
            sh_r <= {2'b00, sh_r[9:2]};
            sh_c <= {2'b00, sh_c[9:2]};
        end
    end

    // Final per-lane output registers: fanout-1 FFs with no shift feedback,
    // so the placer can drop them right next to each IOB (the shift
    // registers themselves are pinned to fabric LUTs by their feedback and
    // kept landing ~25 rows from the OLOGICs => ~-0.4 ns at 375 MHz). All
    // four lanes — including the clock lane — are delayed by the same one
    // x5 cycle, so the inter-lane alignment is unchanged.
    reg [1:0] q_b, q_g, q_r, q_c;
    always @(posedge clk_x5) begin
        q_b <= sh_b[1:0];
        q_g <= sh_g[1:0];
        q_r <= sh_r[1:0];
        q_c <= sh_c[1:0];
    end

    // ---- ODDR2 + OBUFDS per lane ------------------------------------------------
    wire [3:0] ser_q;

    genvar ch;
    generate
        for (ch = 0; ch < 4; ch = ch + 1) begin : lane
            wire [1:0] q2 = (ch == 0) ? q_b : (ch == 1) ? q_g :
                            (ch == 2) ? q_r : q_c;
            ODDR2 #(
                .DDR_ALIGNMENT("C0"),
                .INIT(1'b0),
                .SRTYPE("ASYNC")
            ) oddr (
                .Q(ser_q[ch]),
                .C0(clk_x5),
                .C1(~clk_x5),
                .CE(1'b1),
                .D0(q2[0]),                // C0 rising edge: first bit
                .D1(q2[1]),                // C1 rising edge: second bit
                .R(1'b0),
                .S(1'b0)
            );
        end
    endgenerate

    OBUFDS #(.IOSTANDARD("TMDS_33")) buf_b (.I(ser_q[0]), .O(tmds_tx_p[0]), .OB(tmds_tx_n[0]));
    OBUFDS #(.IOSTANDARD("TMDS_33")) buf_g (.I(ser_q[1]), .O(tmds_tx_p[1]), .OB(tmds_tx_n[1]));
    OBUFDS #(.IOSTANDARD("TMDS_33")) buf_r (.I(ser_q[2]), .O(tmds_tx_p[2]), .OB(tmds_tx_n[2]));
    OBUFDS #(.IOSTANDARD("TMDS_33")) buf_c (.I(ser_q[3]), .O(tmds_tx_clk_p), .OB(tmds_tx_clk_n));

endmodule
