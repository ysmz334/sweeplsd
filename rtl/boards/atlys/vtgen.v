// Video timing generator, default 1280x720p60 (CEA-861: 1650x750 total,
// +hsync +vsync). Runs on the pixel clock.

module vtgen #(
    parameter H_ACTIVE = 1280,
    parameter H_FP     = 110,
    parameter H_SYNC   = 40,
    parameter H_BP     = 220,
    parameter V_ACTIVE = 720,
    parameter V_FP     = 5,
    parameter V_SYNC   = 5,
    parameter V_BP     = 20,
    parameter H_POL    = 1,     // 1 = positive sync pulse (720p), 0 = negative (VGA)
    parameter V_POL    = 1
) (
    input  wire        clk,
    input  wire        rst,
    output reg  [11:0] x,       // active-area position (valid with de)
    output reg  [11:0] y,
    output reg         de,
    output reg         hs,
    output reg         vs,
    output reg         sof      // 1-clk pulse at the start of vertical sync
);

    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;

    reg [11:0] hc, vc;
    wire h_pulse = (hc >= H_ACTIVE + H_FP) && (hc < H_ACTIVE + H_FP + H_SYNC);
    wire v_pulse = (vc >= V_ACTIVE + V_FP) && (vc < V_ACTIVE + V_FP + V_SYNC);

    always @(posedge clk) begin
        if (rst) begin
            hc <= 12'd0;
            vc <= 12'd0;
        end else begin
            if (hc == H_TOTAL - 1) begin
                hc <= 12'd0;
                vc <= (vc == V_TOTAL - 1) ? 12'd0 : vc + 12'd1;
            end else begin
                hc <= hc + 12'd1;
            end
        end
        x <= hc;
        y <= vc;
        de <= (hc < H_ACTIVE) && (vc < V_ACTIVE);
        hs <= (H_POL != 0) ? h_pulse : ~h_pulse;   // line level incl. polarity
        vs <= (V_POL != 0) ? v_pulse : ~v_pulse;
        sof <= (hc == 12'd0) && (vc == V_ACTIVE + V_FP);
    end

endmodule
