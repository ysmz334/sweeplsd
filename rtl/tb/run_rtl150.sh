#!/bin/sh
# Run the RTL burst replay (shipping config: 2048 FIFO, 1080p30 pacing,
# improved mode) over every vprof image and dump full-field records for the
# 3-level comparison renders. 8-way parallel; skips images already done.
#   usage: run_rtl150.sh <outdir>
set -e
cd "$(dirname "$0")/.."   # rtl/

OSS=${OSS_CAD_SUITE:-/e/dev/claude/tools/oss-cad-suite}
PATH="$OSS/bin:$OSS/lib:$PATH"
export PATH

OUT=${1:-tb/rtl150}
mkdir -p "$OUT"

JOBS=8
running=0
for ev in tb/vprof/IMGP*_imp_events.hex; do
    name=$(basename "$ev" _imp_events.hex)
    [ -f "$OUT/${name}_recs_out.txt" ] && continue
    (
        iverilog -g2005 -o "$OUT/${name}.vvp" \
            -DIMG_W=1920 -DIMG_H=1080 -DPIX_TH=15 \
            -DHYST_ON=1 -DHYST_MIN=3 -DBORDER=3 -DMPS_2SQ=2 \
            "-DVEC=\"tb/vprof/${name}_imp\"" "-DOUT=\"$OUT/${name}\"" \
            tb/tb_burst_mon.v core/event_fifo.v core/backend.v core/judge_unit.v \
            2> "$OUT/${name}.log" &&
        vvp "$OUT/${name}.vvp" >> "$OUT/${name}.log" 2>&1
        rm -f "$OUT/${name}.vvp"
        echo "done $name"
    ) &
    running=$((running + 1))
    if [ $running -ge $JOBS ]; then
        wait -n 2>/dev/null || wait
        running=$((running - 1))
    fi
done
wait
echo "ALL DONE: $(ls "$OUT"/*_recs_out.txt | wc -l) images"
