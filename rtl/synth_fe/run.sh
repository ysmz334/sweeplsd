#!/bin/sh
# Front-end timing trial (v2b M2): XST -> ngdbuild -> map -> trce at the
# 720p pixel period. Prints the worst path summary. Run from anywhere.
set -e
cd "$(dirname "$0")"

# Xilinx ISE 14.7 — override via XILINX / ISE_BIN / XILINXD_LICENSE_FILE if
# your install differs (see boards/atlys/build_rx.sh for details).
export XILINX="${XILINX:-E:\Xilinx\14.7\ISE_DS\ISE}"
export XILINXD_LICENSE_FILE="${XILINXD_LICENSE_FILE:-$USERPROFILE\.Xilinx}"
ISE_BIN="${ISE_BIN:-/e/Xilinx/14.7/ISE_DS/ISE/bin/nt64}"
export PATH="$ISE_BIN:${ISE_BIN%/bin/nt64}/lib/nt64:$PATH"

mkdir -p build
cd build

cat > fe.prj <<EOF
verilog work "../../core/fe_chain.v"
verilog work "../../core/stage_gauss.v"
verilog work "../../core/stage_gradient.v"
verilog work "../../core/stage_edge.v"
verilog work "../../core/hyst_hist.v"
verilog work "../../core/stage_feature.v"
verilog work "../../core/endpoint_core.v"
verilog work "../../core/event_pack.v"
verilog work "../front_synth.v"
EOF

# -fsm_extract NO matches the board builds (see boards/atlys/build_rx.sh),
# so front-end figures stay representative of the shipped netlist.
cat > fe.xst <<EOF
run -ifn fe.prj -ifmt mixed -top front_synth -ofn fe.ngc -ofmt NGC
-p xc6slx45-3-csg324 -opt_mode Speed -opt_level 1 -fsm_extract NO
EOF

cat > fe.ucf <<EOF
NET "clk" TNM_NET = "tn_clk";
TIMESPEC "TS_clk" = PERIOD "tn_clk" 13.468 ns HIGH 50%;
EOF

xst -ifn fe.xst < /dev/null > xst.log 2>&1
ngdbuild -uc fe.ucf -p xc6slx45-3-csg324 -aul fe.ngc fe.ngd > ngdbuild.log 2>&1
map -p xc6slx45-3-csg324 -w -o fe_map.ncd fe.ngd fe.pcf > map.log 2>&1
trce -v 3 fe_map.ncd fe.pcf -o fe.twr > trce.log 2>&1 || true
echo "== worst paths (post-map estimate) =="
grep -E 'Timing constraint|failing endpoints|Minimum period|^Slack|Source:|Destination:|Data Path Delay' fe.twr | head -20
