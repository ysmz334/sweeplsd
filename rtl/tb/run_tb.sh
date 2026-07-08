#!/bin/sh
# Run one RTL parity testbench over every golden vector set in tb/vectors/.
#   usage: run_tb.sh <tb_name> [extra core .v files ...]
#   e.g.:  run_tb.sh tb_gauss core/stage_gauss.v
#   CE_DIV=2 run_tb.sh ...   runs the core at a 1-in-2 clock enable (v2a mode)
#   VECDIR=tb/vectors_big run_tb.sh ...   runs another vector directory
#     (vectors_big includes the FullHD regression — slow but the gate)
# The 5th meta field (strict, improved-mode NMS tie-break) is passed to the
# testbench as -DSTRICT; *_imp vector sets exercise the improved mode.
# Requires iverilog/vvp on PATH (OSS CAD Suite). Exits non-zero on any FAIL.

set -e
cd "$(dirname "$0")/.."   # rtl/

# OSS CAD Suite needs both bin/ and lib/ on PATH (the iverilog driver spawns
# sub-tools from lib/). Honour an existing install; default to the local one.
OSS=${OSS_CAD_SUITE:-/e/dev/claude/tools/oss-cad-suite}
PATH="$OSS/bin:$OSS/lib:$PATH"
export PATH

TB="$1"; shift
VECDIR=${VECDIR:-tb/vectors}
FAIL=0
for meta in "$VECDIR"/*_meta.txt; do
    name=$(basename "$meta" _meta.txt)
    read -r w h power_th pix_th strict hyst_on hyst_adapt hyst_low hyst_min _rest < "$meta"
    strict=${strict:-0}
    hyst_on=${hyst_on:-0}; hyst_adapt=${hyst_adapt:-0}
    hyst_low=${hyst_low:-120}; hyst_min=${hyst_min:-3}
    iverilog -g2005 -o "tb/${TB}_${name}.vvp" \
        -DIMG_W="$w" -DIMG_H="$h" -DPOWER_TH="$power_th" -DPIX_TH="$pix_th" \
        -DSTRICT="$strict" \
        -DHYST_ON="$hyst_on" -DHYST_ADAPT="$hyst_adapt" \
        -DHYST_LOW="$hyst_low" -DHYST_MIN="$hyst_min" \
        -DCE_DIV="${CE_DIV:-1}" \
        -DVEC="\"${VECDIR}/${name}\"" \
        "tb/${TB}.v" "$@"
    out=$(vvp "tb/${TB}_${name}.vvp")
    rm -f "tb/${TB}_${name}.vvp"
    echo "[$TB/$name] $out"
    case "$out" in *PASS*) ;; *) FAIL=1 ;; esac
done
exit $FAIL
