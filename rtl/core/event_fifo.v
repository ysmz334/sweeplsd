// Elastic buffer between the pixel-rate front-end and the event-rate
// labelling back-end (rtl/DESIGN.md). First-word-fall-through: the front
// entry is visible whenever not empty; a pop advances it on the next edge.
//
// Two overflow policies:
//  - drop_mode = 0 (frame sources that can pause): `stall` tells the walker
//    to stop before the FIFO can overflow — lossless.
//  - drop_mode = 1 (LIVE sources that cannot be paused, v2b M3): stall is
//    never asserted; when the FIFO is nearly full, DATA events (interior /
//    endpoint) are discarded (`dropped` pulses) while the EOR/EOF row
//    markers always use the 8-slot reserve — losing a marker would shear
//    the back-end's row bookkeeping, losing a data event merely thins the
//    labelling locally. Self-limiting: dropped events never enter the
//    interior lists, so overload shrinks the back-end's row work until it
//    catches up.
//
// Simulation-simple register-array implementation; for the board build this
// becomes a BRAM FIFO with an output register (same interface).

module event_fifo #(
    parameter DW = 15,                  // {strong[14], kind[13:12], x[11:0]}
    parameter AW = 11                   // depth 2^AW
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          en,            // global clock enable (sweep_core.v)

    input  wire          drop_mode,    // 1 = live source (see header)
    input  wire          push,
    input  wire [DW-1:0] wdata,
    output wire          stall,        // nearly full: stop the producer
    output wire          dropped,      // pulse: data event discarded (drop_mode)

    output wire          empty,
    output wire [DW-1:0] front,
    input  wire          pop
);

    localparam integer DEPTH = 1 << AW;

    reg [DW-1:0] mem [0:DEPTH-1];
    reg [AW:0] wp, rp;

    wire [AW:0] count = wp - rp;
    assign empty = (count == 0);
    wire afull = (count >= DEPTH - 8);
    assign stall = drop_mode ? 1'b0 : afull;
    // kinds: 1 = interior, 2 = endpoint (data); 3 = EOR, 0 = EOF (markers).
    // Kind sits at [13:12] (strong is the extra top bit [14]), so read it there.
    wire [1:0] wkind = wdata[13:12];
    wire is_data = (wkind == 2'd1) || (wkind == 2'd2);
    wire drop = drop_mode && push && afull && is_data;
    assign dropped = drop && en;
    assign front = mem[rp[AW-1:0]];

    always @(posedge clk) begin
        if (en) begin
            if (push && !drop) begin
                mem[wp[AW-1:0]] <= wdata;
                wp <= wp + 1'b1;
            end
            if (pop && !empty) rp <= rp + 1'b1;
        end
        if (rst) begin
            wp <= {AW+1{1'b0}};
            rp <= {AW+1{1'b0}};
        end
    end

endmodule
