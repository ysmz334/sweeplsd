# SweepLSD on the Digilent Atlys (Spartan-6 XC6SLX45)

Live **HDMI in → detect → segment overlay → HDMI out** demo. The detector is the
portable Verilog core in [`../../core/`](../../core); everything in this folder is
board glue (video timing, TMDS output, EDID, overlay, top levels).

See [`../../DESIGN.md`](../../DESIGN.md) for the architecture and the
verification / timing-closure story.

## Licensing of the sources here

Everything in this repository is MIT (see the top-level `LICENSE`) **except**:

- **`edid_rom_720p.vhd`** — Mike Field, MIT-licensed (© 2015 Michael Alan Field).
  Redistributed here with its header intact; MIT-compatible.
- **`xapp495/`** — the Xilinx **XAPP495** DVI reference design (TMDS SerDes,
  DVI encoder/decoder). It is covered by the Xilinx application-note license
  (use only in a design for a Xilinx device) and is **NOT redistributable**, so
  it is **git-ignored** and absent from a fresh clone. You must obtain it
  yourself to build the board bitstream (see below). Only the *live* HDMI-input
  top (`atlys_rx_top.v`, `build_rx.sh`) needs it; the internally-generated-scene
  top (`atlys_top.v`) drives HDMI out through the in-repo `dvid_out.v` /
  `tmds_encoder.v` (clean-room from the DVI 1.0 spec) and needs no XAPP495.

## Getting XAPP495 (only for the live HDMI-in build)

XAPP495 "Implementing a TMDS Video Interface in the Spartan-6 FPGA" ships a
reference-design zip (`xapp495.zip`) from the Xilinx/AMD application-note page.
Download it and copy these 11 files into `xapp495/` here, preserving the layout
`build_rx.sh` expects:

```
rtl/boards/atlys/xapp495/
├── rx/
│   ├── serdes_1_to_5_diff_data.v
│   ├── phsaligner.v
│   ├── chnlbond.v
│   ├── decode.v
│   └── dvi_decoder.v
├── tx/
│   ├── encode.v
│   ├── serdes_n_to_1.v
│   ├── convert_30to15_fifo.v
│   ├── dvi_encoder.v
│   └── dvi_encoder_top.v
└── common/
    └── DRAM16XN.v
```

(The Digilent Atlys HDMI demo distributes the same files; either source works as
long as the module names match the instantiations in `atlys_rx_top.v`.)

## Building

Requires Xilinx ISE 14.7 (Spartan-6 is not a Vitis target). With the ISE
environment on `PATH`, from this directory:

```sh
sh build_rx.sh     # live HDMI-in top → build_rx/top_rx.bit  (needs xapp495/)
sh build.sh        # internal-scene top → build/top.bit       (no xapp495 needed)
```

`build_rx.sh` uses placer cost table `map -t 2`, which closes timing at Score 0
(the default table 1 leaves the adaptive-hysteresis path ~0.5 ns short — a
placement variance, not a logic path; `par -t` is disabled on Spartan-6, so the
cost table is explored through `map`).

Both scripts synthesize with **`-fsm_extract NO`**, and that option is
**load-bearing**: XST's FSM re-encoding mis-synthesizes the detector's
back-end FSM — the build passes timing (Score 0) and RTL simulation is clean,
yet the gate-level behaviour diverges under load (the live "bottom loss"
failure). Keep the option if you adapt these scripts to another ISE flow; the
diagnosis is in [`../../DESIGN.md`](../../DESIGN.md) ("XST FSM-extraction
mis-synthesis").

The default tool paths at the top of the scripts point at one machine's ISE
install; override `XILINX`, `ISE_BIN` and `XILINXD_LICENSE_FILE` in the
environment if yours differ.

Program with Digilent Adept:

```sh
djtgcfg prog -d Atlys -i 0 -f build_rx/top_rx.bit
```

Connect the HDMI source to **J3** (HDMI IN) and a monitor to **J2** (HDMI OUT).
The FPGA serves a 720p-preferred DVI EDID on J3, so the source outputs 720p (or
1080p30 — geometry is auto-measured). Detected segments are overlaid in green.
