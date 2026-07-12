#!/bin/sh
# ISE flow for the v2b HDMI pass-through top (atlys_rx_top): XST -> ngdbuild
# -> map -> par -> bitgen. Run from anywhere (git-bash); output =
# rtl/boards/atlys/build_rx/top_rx.bit.
set -e
cd "$(dirname "$0")"

# Xilinx ISE 14.7 (Spartan-6 needs ISE, not Vivado/Vitis). Defaults are one
# machine's install — override via the environment if yours differs:
#   XILINX               Windows-style ISE root (read by the ISE tools)
#   ISE_BIN              POSIX-style path to ISE bin/nt64 (for PATH)
#   XILINXD_LICENSE_FILE WebPACK license file or directory
export XILINX="${XILINX:-E:\Xilinx\14.7\ISE_DS\ISE}"
export XILINXD_LICENSE_FILE="${XILINXD_LICENSE_FILE:-$USERPROFILE\.Xilinx}"
ISE_BIN="${ISE_BIN:-/e/Xilinx/14.7/ISE_DS/ISE/bin/nt64}"
export PATH="$ISE_BIN:${ISE_BIN%/bin/nt64}/lib/nt64:$PATH"

mkdir -p build_rx
cd build_rx

cat > top_rx.prj <<EOF
verilog work "../../../core/stage_gauss.v"
verilog work "../../../core/stage_gradient.v"
verilog work "../../../core/stage_edge.v"
verilog work "../../../core/hyst_hist.v"
verilog work "../../../core/stage_feature.v"
verilog work "../../../core/endpoint_core.v"
verilog work "../../../core/event_pack.v"
verilog work "../../../core/event_fifo.v"
verilog work "../../../core/fe_chain.v"
verilog work "../../../core/backend.v"
verilog work "../../../core/judge_unit.v"
verilog work "../../../core/sweep_core.v"
verilog work "../live_core.v"
verilog work "../uart_telemetry.v"
verilog work "../overlay_mask.v"
verilog work "../xapp495/rx/serdes_1_to_5_diff_data.v"
verilog work "../xapp495/rx/phsaligner.v"
verilog work "../xapp495/rx/chnlbond.v"
verilog work "../xapp495/rx/decode.v"
verilog work "../xapp495/rx/dvi_decoder.v"
verilog work "../xapp495/tx/encode.v"
verilog work "../xapp495/tx/serdes_n_to_1.v"
verilog work "../xapp495/tx/convert_30to15_fifo.v"
verilog work "../xapp495/tx/dvi_encoder.v"
verilog work "../xapp495/tx/dvi_encoder_top.v"
verilog work "../xapp495/common/DRAM16XN.v"
vhdl work "../edid_rom_720p.vhd"
verilog work "../atlys_rx_top.v"
EOF

cat > top_rx.xst <<EOF
run -ifn top_rx.prj -ifmt mixed -top atlys_rx_top -ofn top_rx.ngc -ofmt NGC
-p xc6slx45-3-csg324 -opt_mode Speed -opt_level 1 -fsm_extract NO
EOF

# -fsm_extract NO: XST's FSM re-encoding mis-synthesizes the back-end FSM —
# the netlist deviates from RTL under load (judge dispatches stop completing
# after ~40 closes; gate-level sim reproduces the live board's bottom-loss
# exactly, and -fsm_extract NO restores RTL-identical behaviour). Do NOT
# re-enable without re-running the gate-level frame regression.
echo "== xst =="
xst -ifn top_rx.xst < /dev/null
echo "== ngdbuild =="
ngdbuild -uc ../atlys_rx.ucf -p xc6slx45-3-csg324 top_rx.ngc top_rx.ngd
echo "== map =="
# -t 2: placer cost table 2 closes timing at Score 0. Cost table 1 (default)
# leaves the adaptive-hysteresis percentile-scan path (u_hist snap->found_b, 12
# levels) ~0.57ns short once the v2d (h)/(i) judge/emit logic shares the pixel
# clock domain — a placement variance, not a logic path (par -t is disabled on
# Spartan-6, so the cost table is explored via map). Tables 2 and 4 both give 0.
map -p xc6slx45-3-csg324 -w -t 2 -o top_rx_map.ncd top_rx.ngd top_rx.pcf
echo "== par =="
par -w -ol high top_rx_map.ncd top_rx.ncd top_rx.pcf
echo "== bitgen =="
# UnusedPin:Pullnone: the default pulldowns land on the DDC nets shared with
# J2 (D9/C9) and would load the I2C bus.
bitgen -w -g StartUpClk:CClk -g UnusedPin:Pullnone top_rx.ncd top_rx.bit top_rx.pcf
echo "== DONE: build_rx/top_rx.bit =="
