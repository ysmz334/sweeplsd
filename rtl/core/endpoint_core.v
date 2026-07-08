// Endpoint-candidate classification of one 5x5 edge window (thesis §3.2.2,
// fig. 3.11): thin the outer ring against the inner ring, count survivors,
// and report "endpoint candidate" (is_end = 1) unless the centre is a
// straight line passing through (exactly two survivors on opposite arcs).
//
// Pure combinational port of sweeplsd::kernels::endpointCore (via
// hls/src/frontend.cpp) — the branch-free boolean algebra maps 1:1 onto LUT
// logic. Row r0 = window row -2 (top) ... r4 = +2; bit [c] = column c-2.

module endpoint_core (
    input  wire [4:0] r0,
    input  wire [4:0] r1,
    input  wire [4:0] r2,
    input  wire [4:0] r3,
    input  wire [4:0] r4,
    output wire       is_end
);

    // Inner ring (a..h) and outer ring (i..x2) — the software's tap names.
    wire a  = r2[1], b  = r1[1], c  = r1[2], d  = r1[3];
    wire eR = r2[3], fR = r3[3], gR = r3[2], hR = r3[1];

    wire i0 = r2[0], j0 = r1[0], k0 = r0[0], l0 = r0[1];
    wire m0 = r0[2], n0 = r0[3], o0 = r0[4], p0 = r1[4];
    wire q0 = r2[4], s0 = r4[4], t0 = r4[3];
    wire u0 = r4[2], v0 = r4[1], w0 = r4[0], x2 = r3[0];
    wire rr = r3[4];

    wire ab = a | b, bc = b | c, cd = c | d, de = d | eR;
    wire ef = eR | fR, fg = fR | gR, gh = gR | hR, ha = hR | a;
    wire hab = ha | b, bcd = bc | d, def = de | fR, fgh = fg | hR;

    wire i1 = i0 & hab, m1 = m0 & bcd, q1 = q0 & def, u1 = u0 & fgh;

    wire hai = ha | i1, abi = ab | i1, bcm = bc | m1, cdm = cd | m1;
    wire deq = de | q1, efq = ef | q1, fgu = fg | u1, ghu = gh | u1;

    wire j1 = j0 & abi, l1 = l0 & bcm, n1 = n0 & cdm, p1 = p0 & deq;
    wire r1s = rr & efq, t1 = t0 & fgu, v1 = v0 & ghu, x1 = x2 & hai;

    wire xj = x1 | j1, ln = l1 | n1, pr = p1 | r1s, tv = t1 | v1;
    wire i2 = i1 & ~xj, m2 = m1 & ~ln, q2 = q1 & ~pr, u2 = u1 & ~tv;

    wire jl = j1 & l1, np = n1 & p1, rt = r1s & t1, vx = v1 & x1;
    wire bjl = b | jl, dnp = d | np, frt = fR | rt, hvx = hR | vx;

    wire k1 = k0 & bjl, o1 = o0 & dnp, s1 = s0 & frt, w1 = w0 & hvx;
    wire j2 = j1 & ~k1, l2 = l1 & ~k1, n2 = n1 & ~o1, p2 = p1 & ~o1;
    wire r2s = r1s & ~s1, t2 = t1 & ~s1, v2 = v1 & ~w1, x2s = x1 & ~w1;

    wire [4:0] count =
        {4'd0, i2} + {4'd0, j2} + {4'd0, k1} + {4'd0, l2} +
        {4'd0, m2} + {4'd0, n2} + {4'd0, o1} + {4'd0, p2} +
        {4'd0, q2} + {4'd0, r2s} + {4'd0, s1} + {4'd0, t2} +
        {4'd0, u2} + {4'd0, v2} + {4'd0, w1} + {4'd0, x2s};

    // Ring thermometer masks (i contributes 0): the XOR's popcount is the
    // shorter arc distance between the two survivors.
    wire [7:0] xm = ({8{j2}}  & 8'h01) ^ ({8{k1}}  & 8'h03) ^ ({8{l2}} & 8'h07) ^
                    ({8{m2}}  & 8'h0f) ^ ({8{n2}}  & 8'h1f) ^ ({8{o1}} & 8'h3f) ^
                    ({8{p2}}  & 8'h7f) ^ ({8{q2}}  & 8'hff) ^ ({8{r2s}} & 8'hfe) ^
                    ({8{s1}}  & 8'hfc) ^ ({8{t2}}  & 8'hf8) ^ ({8{u2}} & 8'hf0) ^
                    ({8{v2}}  & 8'he0) ^ ({8{w1}}  & 8'hc0) ^ ({8{x2s}} & 8'h80);

    wire [3:0] pcnt = {3'd0, xm[0]} + {3'd0, xm[1]} + {3'd0, xm[2]} + {3'd0, xm[3]} +
                      {3'd0, xm[4]} + {3'd0, xm[5]} + {3'd0, xm[6]} + {3'd0, xm[7]};

    wire straight = (count == 5'd2) && (pcnt > 4'd6);
    assign is_end = ~straight;

endmodule
