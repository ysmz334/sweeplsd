// Dense feature samples -> sparse event stream (thesis §3.2.3 input side).
// Port of hls/src/frontend.cpp hlsEvents. One event per feature pixel
// (kind 1 = interior, 2 = endpoint) plus one end-of-row marker per image row
// and a final end-of-frame marker.
//
// A data event and its row's end-of-row marker cannot collide: the feature
// stage emits column width-1 on the last walk tick of a row, so the marker is
// deferred to the following tick (the walker's early columns of the next row
// produce no feature samples — the slot is always free).

module event_pack #(
    parameter XW = 12
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          en,
    input  wire [XW-1:0] width,
    input  wire [XW-1:0] height,

    input  wire          i_valid,      // feature sample (i_x, i_y)
    input  wire [XW-1:0] i_x,
    input  wire [XW-1:0] i_y,
    input  wire [1:0]    i_f,
    input  wire          i_strong,     // (d) strong bit of (i_x, i_y)

    output wire          ev_valid,
    output wire [1:0]    ev_kind,      // 1 interior / 2 endpoint / 3 EOR / 0 EOF
    output wire [XW-1:0] ev_x,
    output wire          ev_strong     // (d) strong bit (data events; 0 on markers)
);

    localparam [1:0] KIND_EOF = 2'd0, KIND_EOR = 2'd3;

    reg eor_pending = 1'b0;   // row i_y just completed
    reg eof_pending = 1'b0;   // ... and it was the last row
    reg done = 1'b0;          // EOF sent

    wire data_ev = i_valid && (i_f != 2'd0);

    assign ev_valid = en && !done && (data_ev || eor_pending || eof_pending);
    assign ev_kind = data_ev ? i_f : (eor_pending ? KIND_EOR : KIND_EOF);
    assign ev_x = data_ev ? i_x : {XW{1'b0}};
    assign ev_strong = data_ev ? i_strong : 1'b0;

    always @(posedge clk) begin
        if (en) begin
            if (i_valid && i_x == width - 1) begin
                eor_pending <= 1'b1;
                if (i_y == height - 1) eof_pending <= 1'b1;
            end else if (eor_pending) begin
                eor_pending <= 1'b0;    // EOR emitted this tick
            end else if (eof_pending) begin
                eof_pending <= 1'b0;    // EOF emitted this tick
                done <= 1'b1;
            end
        end
        if (rst) begin
            eor_pending <= 1'b0;
            eof_pending <= 1'b0;
            done <= 1'b0;
        end
    end

endmodule
