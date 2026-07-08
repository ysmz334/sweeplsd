# Vitis HLS flow for the SweepLSD core.
#
#   classic tool:   vitis_hls -f scripts/run_hls.tcl [-tclargs cosim]
#   unified (2023.2+, e.g. 2026.1):
#                   vitis-run --mode hls --tcl scripts/run_hls.tcl
#                   (set ::env(SWEEPLSD_HLS_COSIM) 1 to add RTL co-simulation)
#
# Run from the hls/ directory. Reports land in proj_sweeplsd_hls/sol1/syn/report.

set do_cosim [expr {([info exists argv] && [llength $argv] > 0 && [lindex $argv 0] eq "cosim")
                    || [info exists ::env(SWEEPLSD_HLS_COSIM)]}]

open_project -reset proj_sweeplsd_hls
# The full core (II=1 front-end + event-driven labelling back-end), via the
# global-scope wrapper (set_top cannot name functions inside a namespace).
set_top sweeplsd_core_top

add_files src/frontend.cpp -cflags "-Isrc -Icompat -std=c++17"
add_files src/backend.cpp -cflags "-Isrc -Icompat -std=c++17"
add_files -tb tb/tb_frontend.cpp \
    -cflags "-Isrc -Icompat -Ihost -I../src -I../include -std=c++17"
# The testbench links the software golden model:
add_files -tb ../src/gaussian.cpp   -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/gradient.cpp   -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/edge.cpp       -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/feature.cpp    -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/labeling.cpp   -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/sweeplsd.cpp   -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/sweeplsd_onepass.cpp -cflags "-I../src -I../include -std=c++17"
add_files -tb ../src/io.cpp \
    -cflags "-I../src -I../include -I../third_party/stb -std=c++17"

open_solution -reset sol1
# Artix-7: same resource class as the thesis-era Spartan-6 LX45 (phase-2
# target is hand Verilog on that device; see ../DESIGN.md).
set_part xc7a35tcpg236-1
# 100 MHz -> >= 1.6x the FullHD@30 pixel rate at II=1.
create_clock -period 10
# The tool default (27% of the period) is far more conservative than a plain
# 7-series pipeline needs and alone pushed the report 1.31 ns negative.
set_clock_uncertainty 1.0

csim_design
csynth_design
if {$do_cosim} {
    # Full frames would RTL-simulate for hours; the tb honours this env var
    # by running only its small frames (borders/labelling still covered).
    set ::env(SWEEPLSD_TB_SMALL) 1
    cosim_design -rtl verilog
}

exit
