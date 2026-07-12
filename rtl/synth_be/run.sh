#!/bin/sh
# Back-end timing trial at the 720p pixel period (see be_synth.v).
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

cat > be.prj <<EOF
verilog work "../../core/event_fifo.v"
verilog work "../../core/backend.v"
verilog work "../../core/judge_unit.v"
verilog work "../be_synth.v"
EOF

# -fsm_extract NO matches the board builds (XST FSM re-encoding
# mis-synthesizes the back-end FSM — see boards/atlys/build_rx.sh); keeping
# it here makes these timing figures representative of the shipped netlist.
cat > be.xst <<EOF
run -ifn be.prj -ifmt mixed -top be_synth -ofn be.ngc -ofmt NGC
-p xc6slx45-3-csg324 -opt_mode Speed -opt_level 1 -fsm_extract NO
EOF

cat > be.ucf <<EOF
NET "clk" TNM_NET = "tn_clk";
TIMESPEC "TS_clk" = PERIOD "tn_clk" 13.468 ns HIGH 50%;
EOF

xst -ifn be.xst < /dev/null > xst.log 2>&1
ngdbuild -uc be.ucf -p xc6slx45-3-csg324 -aul be.ngc be.ngd > ngdbuild.log 2>&1
map -p xc6slx45-3-csg324 -w -o be_map.ncd be.ngd be.pcf > map.log 2>&1
trce -v 3 be_map.ncd be.pcf -o be.twr > trce.log 2>&1 || true
echo "== worst paths (post-map estimate) =="
grep -E 'Timing constraint|failing endpoints|Minimum period|^Slack|Source:|Destination:|Data Path Delay' be.twr | head -20
