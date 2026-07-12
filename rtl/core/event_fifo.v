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
//    markers always use the RESERVE slots — losing a marker would shear
//    the back-end's row bookkeeping, losing a data event merely thins the
//    labelling locally. Self-limiting: dropped events never enter the
//    interior lists, so overload shrinks the back-end's row work until it
//    catches up.
//
// RESERVE sizing (c2 concurrent ingest): the reserve must hold every marker
// that can be resident at once = how many rows the labeller can lag behind
// the video. The serial-ingest back-end resumed popping every row, so 8
// sufficed; the concurrent-ingest back-end blocks ALL pops while its ingest
// row runs 3 rows ahead of labelling, so on a dense band one EOR accumulates
// per lagged row (measured: 9 resident markers on the worst corpus frame —
// past the old reserve; live scenes go further). 64 covers a 64-row lag,
// far beyond anything drop-thinning lets persist. As a last line of defence
// a hard-full push is dropped WHATEVER its kind: a sheared row is recovered
// by the frame restart, silent ring-pointer corruption is not.
//
// Simulation-simple register-array implementation; for the board build this
// becomes a BRAM FIFO with an output register (same interface).

module event_fifo #(
    parameter DW = 15,                  // {strong[14], kind[13:12], x[11:0]}
    parameter AW = 11,                  // depth 2^AW
    // Marker headroom (see comment at the old localparam site below) and the
    // shedding hysteresis window; parameters so testbenches can shrink the
    // data capacity and exercise real saturation dynamics.
    parameter RESERVE = 1152,
    parameter DROP_HYST = 448           // 0 = legacy per-event thinning
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

    // Marker headroom (RESERVE): EVERY row marker of a maximally-lagged frame
    // must fit, or overload starts eating EOR markers and the labeller
    // silently loses whole rows (observed live at 720p60 on dense content:
    // segments only in the top quarter of the frame — the bottom rows' EORs
    // were hard-dropped). Markers resident <= frame rows + EOF = 1081 for
    // 1080p, so 1152 makes marker loss structurally impossible. Data
    // capacity 2048-1152 = 896.

    wire [AW:0] count = wp - rp;
    assign empty = (count == 0);
    wire afull = (count >= DEPTH - RESERVE);
    wire full_hard = (count >= DEPTH);  // ring about to overwrite: drop anything
    assign stall = drop_mode ? 1'b0 : afull;
    // kinds: 1 = interior, 2 = endpoint (data); 3 = EOR, 0 = EOF (markers).
    // Kind sits at [13:12] (strong is the extra top bit [14]), so read it there.
    wire [1:0] wkind = wdata[13:12];
    wire is_data = (wkind == 2'd1) || (wkind == 2'd2);
    // Shedding policy (drop_mode): per-event thinning at afull looks graceful
    // but is catastrophic for LINE detection — random drops at rate p cut
    // every run into ~(1-p)/p px fragments, all under pix_th=16 once p
    // exceeds ~6%, so a saturated stretch of frame yields ZERO segments (the
    // live "bottom loss"). Instead shed with hysteresis: once afull trips,
    // drop data DOWN TO the low watermark, then pass everything until afull
    // trips again. Same average loss, but the kept stretches are contiguous
    // (hundreds of events), so runs inside them stay intact and still form
    // segments — overload thins the result instead of wiping it.
    reg shedding;
    wire relow = (count <= DEPTH - RESERVE - DROP_HYST);
    always @(posedge clk) begin
        if (en) begin
            if (afull) shedding <= 1'b1;
            else if (relow) shedding <= 1'b0;
        end
        if (rst) shedding <= 1'b0;
    end
    wire shed = (DROP_HYST == 0) ? afull : (shedding || afull);
    wire drop = push && ((drop_mode && shed && is_data) || full_hard);
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
