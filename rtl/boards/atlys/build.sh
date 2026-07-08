#!/bin/sh
# Full ISE flow for the Atlys SweepLSD demo: XST -> ngdbuild -> map -> par ->
# bitgen. Run from rtl/boards/atlys (git-bash). Needs XILINX env handled here.
set -e
cd "$(dirname "$0")"

export XILINX='E:\Xilinx\14.7\ISE_DS\ISE'
export XILINXD_LICENSE_FILE='C:\Users\MUTSU\.Xilinx'
export PATH="/e/Xilinx/14.7/ISE_DS/ISE/bin/nt64:/e/Xilinx/14.7/ISE_DS/ISE/lib/nt64:$PATH"

mkdir -p build
cd build

cat > top.prj <<EOF
verilog work "../../../core/stage_gauss.v"
verilog work "../../../core/stage_gradient.v"
verilog work "../../../core/stage_edge.v"
verilog work "../../../core/hyst_hist.v"
verilog work "../../../core/stage_feature.v"
verilog work "../../../core/endpoint_core.v"
verilog work "../../../core/event_pack.v"
verilog work "../../../core/event_fifo.v"
verilog work "../../../core/backend.v"
verilog work "../../../core/judge_unit.v"
verilog work "../../../core/sweep_core.v"
verilog work "../tmds_encoder.v"
verilog work "../vtgen.v"
verilog work "../pattern_gen.v"
verilog work "../dvid_out.v"
verilog work "../overlay_mask.v"
verilog work "../atlys_top.v"
EOF

cat > top.xst <<EOF
run -ifn top.prj -ifmt mixed -top atlys_top -ofn top.ngc -ofmt NGC
-p xc6slx45-3-csg324 -opt_mode Speed -opt_level 1
EOF

echo "== xst =="
xst -ifn top.xst < /dev/null
echo "== ngdbuild =="
ngdbuild -uc ../atlys.ucf -p xc6slx45-3-csg324 top.ngc top.ngd
echo "== map =="
map -p xc6slx45-3-csg324 -w -o top_map.ncd top.ngd top.pcf
echo "== par =="
par -w -ol high top_map.ncd top.ncd top.pcf
echo "== bitgen =="
bitgen -w -g StartUpClk:CClk top.ncd top.bit top.pcf
echo "== DONE: build/top.bit =="
