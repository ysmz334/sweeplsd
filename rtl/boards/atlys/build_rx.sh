#!/bin/sh
# ISE flow for the v2b HDMI pass-through top (atlys_rx_top): XST -> ngdbuild
# -> map -> par -> bitgen. Run from anywhere (git-bash); output =
# rtl/boards/atlys/build_rx/top_rx.bit.
set -e
cd "$(dirname "$0")"

export XILINX='E:\Xilinx\14.7\ISE_DS\ISE'
export XILINXD_LICENSE_FILE='C:\Users\MUTSU\.Xilinx'
export PATH="/e/Xilinx/14.7/ISE_DS/ISE/bin/nt64:/e/Xilinx/14.7/ISE_DS/ISE/lib/nt64:$PATH"

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
-p xc6slx45-3-csg324 -opt_mode Speed -opt_level 1
EOF

echo "== xst =="
xst -ifn top_rx.xst < /dev/null
echo "== ngdbuild =="
ngdbuild -uc ../atlys_rx.ucf -p xc6slx45-3-csg324 top_rx.ngc top_rx.ngd
echo "== map =="
map -p xc6slx45-3-csg324 -w -o top_rx_map.ncd top_rx.ngd top_rx.pcf
echo "== par =="
par -w -ol high top_rx_map.ncd top_rx.ncd top_rx.pcf
echo "== bitgen =="
# UnusedPin:Pullnone: the default pulldowns land on the DDC nets shared with
# J2 (D9/C9) and would load the I2C bus.
bitgen -w -g StartUpClk:CClk -g UnusedPin:Pullnone top_rx.ncd top_rx.bit top_rx.pcf
echo "== DONE: build_rx/top_rx.bit =="
