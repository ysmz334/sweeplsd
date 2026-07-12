// TX-only UART telemetry (diagnostic; Atlys USB-UART, FPGA TX = B16).
// Once per detector pass a snapshot of six 24-bit counters is serialized as
// an ASCII line:  "P xxxxxx D xxxxxx J xxxxxx S xxxxxx W xxxxxx R xxxxxx\n"
// (pushes, drops, judge dispatches, judge stall cycles, watchdog fires,
// records) — ~52 chars, 4.5 ms at 115200, well inside a 33 ms frame.
//
// CDC: the counters live in the pixel-clock domain; a frozen snapshot is
// latched there at each END record together with a toggle flag. This side
// (clk100) double-synchronizes the toggle and serializes the snapshot,
// which stays stable for a whole frame — a standard quasi-static handshake.

module uart_telemetry (
    input  wire        clk100,
    input  wire        rst,

    input  wire        snap_toggle,   // pixel-domain: flips at each END record
    input  wire [23:0] c_push,
    input  wire [23:0] c_drop,
    input  wire [23:0] c_jdisp,
    input  wire [23:0] c_jstall,
    input  wire [23:0] c_jwd,
    input  wire [23:0] c_rec,

    output reg         uart_tx
);

    localparam integer DIV = 868;     // 100 MHz / 115200

    // ---- toggle synchronizer -------------------------------------------------
    reg t0, t1, t2;
    always @(posedge clk100) begin
        t0 <= snap_toggle;
        t1 <= t0;
        t2 <= t1;
    end
    wire kick = (t1 != t2);

    // ---- line assembly ---------------------------------------------------------
    // 6 fields x (1 letter + 1 space + 6 hex + 1 space) ... simpler: build a
    // fixed 56-byte buffer on kick.
    function [7:0] hexc;
        input [3:0] v;
        hexc = (v < 10) ? (8'h30 + {4'd0, v}) : (8'h37 + {4'd0, v});
    endfunction

    reg [7:0] line [0:55];
    reg [5:0] llen;
    integer k;
    task set_field;
        input integer base;
        input [7:0] tag;
        input [23:0] val;
        begin
            line[base]   = tag;
            line[base+1] = hexc(val[23:20]);
            line[base+2] = hexc(val[19:16]);
            line[base+3] = hexc(val[15:12]);
            line[base+4] = hexc(val[11:8]);
            line[base+5] = hexc(val[7:4]);
            line[base+6] = hexc(val[3:0]);
            line[base+7] = 8'h20;
        end
    endtask

    // ---- serializer -----------------------------------------------------------
    reg        busy;
    reg [5:0]  ci;        // char index
    reg [3:0]  bi;        // bit index (start, 8 data, stop)
    reg [9:0]  baud;
    reg [9:0]  sh;        // {stop, data[7:0], start}

    always @(posedge clk100) begin
        if (rst) begin
            busy <= 1'b0;
            uart_tx <= 1'b1;
            baud <= 10'd0;
            ci <= 6'd0;
            bi <= 4'd0;
        end else if (!busy) begin
            uart_tx <= 1'b1;
            if (kick) begin
                set_field( 0, "P", c_push);
                set_field( 8, "D", c_drop);
                set_field(16, "J", c_jdisp);
                set_field(24, "S", c_jstall);
                set_field(32, "W", c_jwd);
                set_field(40, "R", c_rec);
                line[48] = 8'h0D;
                line[49] = 8'h0A;
                llen <= 6'd50;
                ci <= 6'd0;
                bi <= 4'd0;
                baud <= 10'd0;
                sh <= {1'b1, 8'h00, 1'b0};   // loaded per char below
                busy <= 1'b1;
            end
        end else begin
            if (baud == DIV - 1) begin
                baud <= 10'd0;
                if (bi == 4'd0) begin
                    sh <= {1'b1, line[ci], 1'b0};
                    uart_tx <= 1'b0;           // start bit
                    bi <= 4'd1;
                end else if (bi <= 4'd9) begin
                    uart_tx <= sh[bi];
                    bi <= bi + 4'd1;
                end else begin
                    // stop bit period done; next char or finish
                    uart_tx <= 1'b1;
                    bi <= 4'd0;
                    if (ci == llen - 1) busy <= 1'b0;
                    else ci <= ci + 6'd1;
                end
            end else begin
                baud <= baud + 10'd1;
            end
        end
    end

endmodule
