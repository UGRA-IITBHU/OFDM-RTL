# OFDM RTL

A 64-point OFDM transmitter in synthesizable Verilog-2001, plus a Zynq bring-up
path that reads its payload from an SD card and writes the generated frame back
out. The RTL is verified against the C++ reference in `ofdm_hls.cc.cpp`.

## Signal chain

```
qpsk_signal_128 -> map_to_subcarriers -> parallel_to_serial
                -> ifft_64 -> serial_to_parallel -> add_cp -> ofdm_frame
```

- 64 subcarriers, 32 bits each (`{re[15:0], im[15:0]}`, both Q1.15).
- 10 guard subcarriers on each side, DC at index 32.
- 4 symbols of 64 samples, each with a 16-sample cyclic prefix: 320 words total.
- `GUARD_COUNT = 10`, `LAST_GUARD_INDEX = 9`. **Set these together** — the
  latter also sets the pilot phase `(i - LAST_GUARD_INDEX) % 6`.
- Subcarrier content per symbol: 0 = Zernkor Chu sync, 1 = all-ones reference,
  2 = header, 3 = QPSK payload with pilots overriding at the pilot phase.

## Layout

| Path | Contents |
| --- | --- |
| `full.v` | top level `ofdm_generate_frame`; `` `include `` aggregator plus chain wiring |
| `src/` | all RTL, including the vendored r22sdf FFT core |
| `testbenches/` | self-checking testbenches |
| `fpga/` | AXI-Lite wrapper, Vitis bare-metal app, host checker |
| `asic/` | ASIC top: SPI slave input, I/Q DAC output |

Module names often do **not** match filenames — `src/zc_seq.v` defines
`zc_sequence_generator`, `src/add_cp.v` defines `parameterized_add_cp`, and so
on. Grep for `module`, not for filenames.

## Simulation

Everything elaborates as plain **Verilog-2001**; `-g2012` is not required and
not needed by Vivado. `-I src` is required because `src/ifft_64.v` `` `include ``s
bare filenames.

```sh
# lint the whole tree
iverilog -Wall -t null -I src full.v src/*.v testbenches/*.v

# strict lint: must be warning-free on both tops
verilator --lint-only -Wall -Isrc --top-module ofdm_generate_frame full.v
verilator --lint-only -Wall -Isrc --top-module ofdm_axi_lite fpga/ofdm_axi_lite.v full.v

# run a testbench (list the DUT explicitly; there is no auto-discovery)
iverilog -I src -o /tmp/sim.vvp src/zc_seq.v testbenches/tb_zc_seq.v && vvp /tmp/sim.vvp
iverilog -I src -o /tmp/sim.vvp full.v   testbenches/tb_full.v      && vvp /tmp/sim.vvp
iverilog -I src -o /tmp/sim.vvp full.v   testbenches/tb_ofdm_frame.v && vvp /tmp/sim.vvp
iverilog -I src -o /tmp/sim.vvp full.v fpga/ofdm_axi_lite.v \
                                  testbenches/tb_ofdm_axi_lite.v   && vvp /tmp/sim.vvp
```

Note the Verilator flag is `-Isrc` with no space — `-I src` is an Icarus spelling
and Verilator rejects it.

Both `verilator --lint-only -Wall` runs are warning-free, and CI enforces it. The
handful of warnings that *are* intentional are waived inline with
`/* verilator lint_off RULE */` directly above the code they cover, each with a
comment saying why. There are no global `-Wno-` flags, so a genuinely new
warning fails the build. Current waivers:

| Rule | Where | Why |
| --- | --- | --- |
| `WIDTHTRUNC` | `Butterfly.v`, `Multiply.v`, `SdfUnit.v` | Vendored r22sdf; the fixed-point `>>>` scaling narrows on purpose |
| `DECLFILENAME` | `zc_seq.v`, `map_to_subcarriers.v`, `add_cp.v`, `full.v`, `FFT64.v`, `Twiddle64.v` | Module name intentionally differs from the filename used by the build commands |
| `UNUSEDSIGNAL` | `full.v` | `parallel_to_serial.frame_done` is driven but unread (see below) |
| `UNUSEDSIGNAL` | `ifft_64.v` | `br_overflow` is not propagated to the port list (see Known limitations) |
| `UNUSEDSIGNAL` | `ofdm_axi_lite.v` | `s_axi_wstrb` and the `[1:0]` byte lane of the AXI addresses are intentionally ignored |

Two Verilator gotchas if you extend this: `-file "glob"` inside a `lint_off`
metacomment is a **syntax error** (so vendored-file waivers must live in the
vendored file), and a comment whose first word is `verilator` is parsed as a
pragma directive, not prose.

Testbenches print `[PASS]`/`[FAIL]` lines and an `error_count` summary, then
always exit 0 — **read the output, do not trust `$?`**.

Accuracy against the floating-point model:

- IFFT: max **2 LSB** (radix-2² per-stage truncation).
- Full 320-word frame: max **2 LSB** (`tb_full.v`, live real-arithmetic golden).
- `fpga/check_frame.py` against the RTL: max **1.6 LSB**. This one is now
  cross-checked against the bit-exact frame the RTL produces (the 320-word
  table embedded in `tb_ofdm_axi_lite.v`) for payload
  `A5C3_0F1E_9B47_D268_3C5A_E1F0_7788_B2D4`: **1.5 LSB** worst case, CP exact
  on all four symbols. So the host checker is a trustworthy pass/fail gate —
  a hardware `FAIL` means a real hardware problem, not model drift.

`testbenches/tb_full.v` computes its golden IFFT live and drives a pseudo-random
payload; `testbenches/tb_ofdm_frame.v` compares against a generated table of
absolute values. They are independent on purpose — agreement between them is
what makes the end-to-end result meaningful.

## ASIC top (SPI in, DAC out)

`asic/ofdm_asic_top.v` wraps the same transmit chain for a chip: payload and
control over a mode-0 SPI slave (`asic/spi_slave_regs.v`), output on parallel
I/Q DAC pins (`dac_i`, `dac_q`, `dac_clk`, `tx_en`, `frame_sync`). The frame is
built in a 256 x 32-bit memory and played out gaplessly with the cyclic prefix
added on read. Pins, SPI register map and DAC timing are in
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md#9-asic-top--spi-in-dac-out-asic).

```sh
iverilog -I src -o /tmp/sim.vvp full.v asic/spi_slave_regs.v asic/ofdm_asic_top.v \
                                  testbenches/tb_ofdm_asic_top.v && vvp /tmp/sim.vvp
verilator --lint-only -Wall -Isrc --top-module ofdm_asic_top asic/ofdm_asic_top.v \
    asic/spi_slave_regs.v src/zc_seq.v src/qpsk_signal.v src/qpsk_signal_128.v \
    src/map_to_subcarriers.v src/parallel_to_serial.v src/ifft_64.v
```

## FPGA bring-up (Zynq-7000 / ZedBoard)

The design is a plain AXI4-Lite slave. There is no streaming interface and no
*PL-side* DDR traffic: the core generates one frame, is held in reset, and
software reads the 320 words back over AXI. Frame storage is **flip-flops, not
BRAM** (~10,240 FFs plus a 320:1 × 32-bit read mux), so watch utilisation and
post-synthesis timing on that mux.

### Vivado — scripted (recommended)

`fpga/build_bd.tcl` builds the entire block design, bitstream and XSA in one
batch run. It has been run end to end on Vivado 2025.1 with the ZedBoard board
files (`avnet.com:zedboard:part0:1.4`, device `xc7z020clg484-1`):

```sh
vivado -mode batch -source fpga/build_bd.tcl
```

What it does, and why:

- Discovers the Board Manager repository and pins the board part, so the PS
  clock, DDR and MIO come from the board preset instead of guesswork.
- Adds `full.v`, `fpga/ofdm_axi_lite.v` and `src/*.v` as design sources.
- Instantiates PS7, adds `ofdm_axi_lite` as a **module reference**, and lets
  `apply_bd_automation` wire `s_axi`, an AXI interconnect and `proc_sys_reset`.
  `s_axi_aresetn` is a declared `RST` interface, so automation wires clock and
  reset itself — a manual `connect_bd_net` on the reset fails with "already
  connected".
- Assigns the AXI slave at **`0x43C00000`** and prints the authoritative
  `OFDM_BASE` at the end. Zynq-7000 GP0 is hardwired to
  `0x4000_0000`–`0x7FFF_FFFF`, so the address must be pinned on the **slave**
  segment; there is nothing to pin on the master side.
- Synthesises, implements, generates the bitstream, writes
  `timing_summary.rpt` / `utilization.rpt` / `drc.rpt`, and exports
  `ofdm_system.xsa` for Vitis.

Post-route timing at 50 MHz (FCLK_CLK0): **WNS +3.598 ns, TNS 0, WHS +0.030 ns,
THS 0**, zero DRC errors, bitstream written. The design is routing-bound on the
frame write enable — the FFT is nowhere on the critical path, so DSP pipelining
buys nothing here.

**DDR is on**, configured by the board preset. The core generates one frame and
software reads it over AXI, so there is no *PL-side* DDR traffic, but the PS
still needs DDR for the FSBL to boot and for the app to have memory. On
Zynq-7000 the PS DDR pins are dedicated PS MIO owned by the preset; adding a
PL DDR memory part here would be meaningless.

### Vivado — manual (GUI)

1. Create a project targeting your Zynq device, apply a board preset (ZedBoard),
   and enable `FCLK_CLK0` (50 MHz is assumed by the `FREQ_HZ` attribute in
   `fpga/ofdm_axi_lite.v` — change it if you use a different clock).
2. Block design: add `Zynq PS7` with **Enable FCLK_CLK0** and **DDR on**.
3. Run Block Automation to export `FCLK_CLK0` and `FCLK_RESET0_N`.
4. Add `fpga/ofdm_axi_lite.v` as a module. It should appear with an
   `s_axi` AXI4-Lite interface — the `XIL_INTERFACENAME` attributes make this
   explicit rather than relying on Vivado's port-name inference.
5. Connect `s_axi_aclk` ← `FCLK_CLK0`, `s_axi_aresetn` ← `FCLK_RESET0_N`
   **directly** — `FCLK_RESET0_N` is already active-low, like
   `s_axi_aresetn`, so no inverter is needed. Let Connection Automation wire
   the AXI-Lite port.
6. Add an XDC constraining `s_axi_aclk` to your `FCLK_CLK0` period (20 ns for
   50 MHz), otherwise timing analysis reports the clock unconstrained.
7. In the Address Editor, pin the **slave** segment inside GP0
   (`0x4000_0000`–`0x7FFF_FFFF`); a master-side assignment will not stick.
8. Validate, then Generate Bitstream.

### Vitis (standalone)

1. Create a standalone (bare-metal) application, using the exported hardware
   platform `ofdm_bd/ofdm_system.xsa` (the scripted build writes it there and
   includes the bitstream).
2. In the BSP settings, enable the **`xilffs`** FatFs library so `ff.h` exists.
3. Check the drive string in the BSP's `ffs.c` matches what `main.c` passes to
   `f_mount` (`"0:/"`). On some Vitis versions this needs editing.
4. Copy `fpga/main.c` into the application `src/`.
5. Confirm `OFDM_BASE` in `main.c` is `0x43C00000U` — that is what the current
   build assigns, so it should already match. Change it only if the value the
   script prints differs.
6. Build, program the FPGA, and run.

The app reads `ID` (`0x0FDA0001`) before anything else and fails fast with a
clear message if it does not match, so a wrong base address or a stale bitstream
is diagnosed immediately instead of showing up later as a silent frame
mismatch. It also writes `CTRL = 0` before every run, which is required — `done`
is cleared by `!run`, not by a reset of its own.

### SD card flow

`main.c` reads a 16-byte payload from `0:/qpsk.bin` and writes the frame to
`0:/frame.bin`, both opened with an explicit drive prefix so they do not depend
on the current drive. Progress goes out UART.

Use a **small card (2–16 GB) formatted FAT32**. Windows ships cards over 32 GB
as exFAT, and FatFs here has no exFAT driver, so such a card will never mount.
Windows also declines to offer FAT32 for cards larger than 32 GB — use Rufus
with `Large FAT32`, or a smaller card.

```sh
python3 fpga/check_frame.py gen qpsk.bin 1        # make a payload
# ... run on board, copy frame.bin back ...
python3 fpga/check_frame.py check qpsk.bin frame.bin
```

The `qpsk.bin` you check against must be the exact file that was on the card.
Regenerating it with a different seed makes the comparison meaningless.

`check_frame.py` needs numpy (`python3 -m pip install numpy`) — the import is
top-level, so even `gen` fails without it. `check_frame.py check` exits non-zero
on failure and prints a per-symbol LSB error and a cyclic-prefix exactness check.

File formats, all little-endian:

- `qpsk.bin` — 16 bytes = `qpsk_bits[127:0]`, byte 0 holds bits 7:0.
- `frame.bin` — 320 × `u32`, each `{re[31:16], im[15:0]}` signed Q1.15,
  symbol `s` at words `80*s .. 80*s+79` (16 CP then 64 body).

### Finding the UART

`xil_printf` goes out PS UART1 → the ZedBoard's USB-serial bridge. It never
appears in the XSDB console, so a silent XSDB window says nothing about the app.

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name | Format-Table -AutoSize
```

Ignore `Standard Serial over Bluetooth link` ports — those are Windows virtual
ports, unrelated to the board. The one that disappears when you unplug the
ZedBoard is the UART; on a typical machine that is `USB Serial Device (COM5)`.
Open it at **115200, 8-N-1, no flow control**.

### Driving the peripheral from XSDB

Useful when you need to separate "the PL datapath is broken" from "software
can't read files" — these are very different problems and the SD card is a
common source of noise. Launch a standalone debugger so the ELF is not
re-downloaded on every run:

```powershell
& "C:\Xilinx\2025.1\Vitis\bin\xsdb.bat"
```
```
connect
mrd 0x43C00018          # ID -> 0FDA0001 proves bitstream loaded + AXI mapped
mwr 0x43C00000 0x00000000
mwr 0x43C00008 0x7788B2D4
mwr 0x43C0000C 0x3C5AE1F0
mwr 0x43C00010 0x9B47D268
mwr 0x43C00014 0xA5C30F1E
mwr 0x43C00000 0x00000001
mrd 0x43C00004          # STATUS -> 00000001 = done, 00000002 = busy
mrd 0x43C000400         # word   0 -> 00F507A2
mrd 0x43C000540         # word  80 -> FE000000
mrd 0x43C0007C0         # word 240 -> 05A8FC58
mrd 0x43C0008FC         # word 319 -> F840FA67
```

The expected values are for payload `A5C3_0F1E_9B47_D268_3C5A_E1F0_7788_B2D4`;
split your own payload the same way — `qpsk.bin`'s 16 bytes little-endian map
onto `QPSK0..3`, whose bit 0 is bit 0 of `qpsk_bits`.

If `mrd` returns `0`, the bitstream is not loaded. Note that a debugger restart
does **not** disturb the PL — the bitstream persists — but a power cycle does,
and FCLK_CLK0 stops until `ps7_init` runs again, so re-run the app (or the
FSBL) before poking registers after a reset.

### FatFs errors

`main.c` prints the `FRESULT` for every failure. The codes that actually come up:

| Code | Name | Usual cause |
| --- | --- | --- |
| `1` | `DISK_ERR` | Card present but not readable — bad card, dirty contacts, or not seated |
| `3` | `NOT_READY` | Card never detected; SDIO not enabled, or card in the wrong slot |
| `12` | `NOT_ENABLED` | FatFs was built without that drive — check the drive number in the BSP's `ff.c` |
| `13` | `NO_FILESYSTEM` | Mounted, but no FAT — almost always an exFAT or FAT16 card |

If you get `NOT_READY` or `NO_FILESYSTEM` on a card you believe is FAT32, check
whether the PS SD peripheral is enabled at all: search the platform's
`ps7_cortexa9_0/include/xparameters.h` for `SDIO` / `XSDPS`. `fpga/build_bd.tcl`
does **not** set `PCW_SDIO_PERIPHERAL_ENABLE` explicitly — it relies on the board
preset applying it, so if the preset was skipped the SD controller is off.

### Register map

Byte offsets from the base address:

| Offset | Name | Access | Meaning |
| --- | --- | --- | --- |
| `0x000` | `CTRL` | RW | bit0 = run. 0 holds the core in reset and clears `done`; 1 runs until the first frame completes, then freezes it |
| `0x004` | `STATUS` | RO | bit0 = done, bit1 = busy |
| `0x008..0x014` | `QPSK0..3` | RW | `qpsk_bits` |
| `0x018` | `ID` | RO | `0x0FDA0001` |
| `0x400 + 4n` | `FRAME[n]` | RO | `n = 0..319`, word = `{re, im}` |

Sequence: write `CTRL=0`, write `QPSK0..3`, write `CTRL=1`, poll `STATUS.done`,
then read the 320 words. To re-run, write `CTRL=0` again first — that is what
clears `done`.

## Known limitations

- Frame storage is flip-flops, not block RAM.
- The AXI write channel requires `AWVALID` and `WVALID` together. Legal AXI, but
  it would deadlock a master that waits for `AWREADY` before asserting
  `WVALID`; the Xilinx interconnect sends both, so this is fine in practice.
- `wstrb` is ignored; use full 32-bit writes.
- Negating `16'h8000` is a no-op in `ifft_64.v`, so a full-scale −1.0 input is
  off by one LSB.
- `src/uart_rx.v` is unrelated to the OFDM chain and has no testbench.