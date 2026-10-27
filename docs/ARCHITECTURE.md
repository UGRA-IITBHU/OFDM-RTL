# OFDM Transmitter — Architecture

This document describes the RTL transmit chain end to end: what goes in, what
each stage does, how the data is laid out, when things happen, and what comes
out. It closes with a design review against the C++ reference
(`ofdm_hls.cc.cpp`).

For build/lint commands and FPGA bring-up steps see `README.md`.

---

## 1. What the design does

Given **128 payload bits**, the core produces **one OFDM frame of 4 symbols**,
each symbol being a 64-point IFFT output with a 16-sample cyclic prefix:

```
  128 bits ──► [ OFDM core ] ──► 4 symbols × 80 complex samples = 320 words
```

| Symbol | Name      | Subcarrier content (non-guard, non-DC positions)        |
|--------|-----------|---------------------------------------------------------|
| 0      | Sync      | Zadoff-Chu sequence (43-entry LUT)                      |
| 1      | Reference | all `1.0 + j0`                                          |
| 2      | Header    | all `1.0 + j0` (pilots are also `1.0`, so all ones)     |
| 3      | Payload   | QPSK from the 128 input bits, pilots overriding         |

## 2. Top-level parameters

Defined on `ofdm_generate_frame` in `full.v`.

| Parameter          | Default | Meaning                                                   |
|--------------------|---------|-----------------------------------------------------------|
| `SUBCARRIER_COUNT` | 64      | IFFT size / subcarriers per symbol                        |
| `GUARD_COUNT`      | 10      | null subcarriers at each band edge                        |
| `DC_INDEX`         | 32      | null DC subcarrier                                        |
| `DATA_WIDTH`       | 16      | bits per real or imaginary component (Q1.15)              |
| `N_SYMBOLS`        | 4       | symbols per frame                                         |
| `SYMBOL_LEN`       | 80      | samples per symbol including CP                           |
| `CP_LENGTH`        | 16      | cyclic prefix length                                      |

`LAST_GUARD_INDEX` is derived as `GUARD_COUNT - 1` (= 9) and passed to the
mapper. It sets the pilot phase, so it must always move with `GUARD_COUNT`.

Several widths are **not** parameterised and assume the defaults: the QPSK
mapper is fixed at 64 symbols / 128 bits, and the IFFT core is fixed at 64
points.

### Subcarrier map (defaults)

```
index:   0 ........ 9 | 10 ............ 31 | 32 | 33 ............ 53 | 54 ....... 63
         guard (null) |       active       | DC |       active       | guard (null)
```

- **Null:** `i < 10`, `i == 32`, `i >= 54` → 21 subcarriers.
- **Active:** 43 subcarriers (indices 10–31, 33–53).
- **Pilots:** active `i` with `(i - 9) % 6 == 0` → **15, 21, 27, 33, 39, 45, 51** (7 pilots).
- **Data:** the remaining **36** active subcarriers carry payload in symbol 3.

## 3. Data formats

### Complex sample — 32 bits

```
 31            16 15             0
┌────────────────┬────────────────┐
│   re  (Q1.15)  │   im  (Q1.15)  │
└────────────────┴────────────────┘
```

Q1.15: signed 16-bit, value = integer / 32768, range [−1, +0.99997].
Useful constants:

| Value         | Hex       |
|---------------|-----------|
| `+1.0` (max)  | `16'h7FFF` |
| `+1/√2`       | `16'h5A82` |
| `−1/√2`       | `16'hA57E` |
| complex one   | `32'h7FFF_0000` |
| complex zero  | `32'h0000_0000` |

### Vector of samples

Every multi-sample bus (`subcarriers`, `zc_seq_cplx`, `ifft_parallel`,
`ofdm_symbol_cp`, `ofdm_frame`) is a flat packed vector with **sample 0 in the
LSBs**: sample `k` lives at `bus[k*32 +: 32]`. Ports are packed (not unpacked
arrays) so the code stays Verilog-2001 and Vivado-compatible.

## 4. Inputs and outputs of the core

```verilog
module ofdm_generate_frame (
    input  wire          clk,
    input  wire          rst,          // async, active-high
    input  wire [127:0]  qpsk_bits,    // payload, must be held stable
    output reg  [10239:0] ofdm_frame,  // 320 x 32-bit words
    output reg           frame_done    // 1-cycle pulse per completed frame
);
```

### Input: `qpsk_bits[127:0]`

Two bits per subcarrier. Subcarrier `k` (0–63) uses:

| Bit                | Drives | `0` → | `1` → |
|--------------------|--------|-------|-------|
| `qpsk_bits[2k]`    | I (re) | `+1/√2` | `−1/√2` |
| `qpsk_bits[2k+1]`  | Q (im) | `+1/√2` | `−1/√2` |

Only the bits of the 36 **data** subcarriers reach the output. The bit pairs of
the 21 null and 7 pilot subcarriers are mapped by the QPSK stage and then
discarded by the mapper, so 72 of the 128 bits are effective. This matches the
C++ reference.

`qpsk_bits` is sampled continuously; hold it stable from reset release until
`frame_done`.

### Output: `ofdm_frame[10239:0]`

```
word n  = ofdm_frame[n*32 +: 32]     n = 0 .. 319
symbol s occupies words 80*s .. 80*s + 79

within a symbol (w = 0..79):
  w =  0..15  : CP   = IFFT sample (48 + w)      ← copy of the last 16 samples
  w = 16..79  : body = IFFT sample (w - 16)
```

i.e. `frame[s*80 + w]` follows the C++ `add_to_frame` layout exactly.

### Output: `frame_done`

A **one-cycle pulse** on the clock edge that writes symbol 3 into `ofdm_frame`.
The core then immediately starts the next frame and will overwrite
`ofdm_frame` symbol-by-symbol (the first overwrite lands ~260 cycles later).
A consumer must capture the frame, or stop the core, on `frame_done` — the AXI
wrapper does the latter (§7).

## 5. Block diagram

```
                      ┌──────────────────────┐
                      │ zc_sequence_generator│  combinational LUT
                      │   (src/zc_seq.v)     │──── zc_seq_cplx [2047:0] ─────┐
                      └──────────────────────┘                               │
                                                                             ▼
 qpsk_bits ┌──────────────────┐ qpsk_symbol_cplx ┌──────────────────────────────────┐
 [127:0] ─►│ qpsk_signal_128  │────[2047:0]─────►│ parameterized_map_to_subcarriers │
           │ 64× qpsk_signal  │   (1 cycle reg)  │   (src/map_to_subcarriers.v)     │
           └──────────────────┘                  │   sync / ref_sym / header ◄──────┼── sym_idx (FSM)
                                                 └───────────────┬──────────────────┘
                                                    subcarriers  │ [2047:0]  (1 cycle reg)
                                                                 ▼
                                                 ┌───────────────────────────────┐
                                    symbol_start►│ parallel_to_serial            │ 64 samples, 1/clk
                                                 └───────────────┬───────────────┘
                                                   do_en/re/im   ▼
                                                 ┌───────────────────────────────┐
                                                 │ ifft_64                       │
                                                 │  conj → FFT(r22sdf) → conj    │
                                                 │  → bit_reversal_buffer        │
                                                 └───────────────┬───────────────┘
                                                   do_en/re/im   ▼  natural order
                                                 ┌───────────────────────────────┐
                                                 │ serial_to_parallel            │──► frame_done
                                                 └───────────────┬───────────────┘        │
                                                 ifft_parallel   ▼ [2047:0]               │ +1 reg
                                                 ┌───────────────────────────────┐        ▼
                                                 │ parameterized_add_cp          │   symbol_done
                                                 └───────────────┬───────────────┘        │
                                                ofdm_symbol_cp   ▼ [2559:0]               │
                                                 ┌───────────────────────────────┐        │
                                                 │ frame FSM (full.v)            │◄───────┘
                                                 │ ofdm_frame[sym_idx] <= cp     │──► ofdm_frame, frame_done
                                                 └───────────────────────────────┘
```

## 6. Stage-by-stage

### 6.1 `zc_sequence_generator` — `src/zc_seq.v`

- **Type:** purely combinational (generate-time constants).
- **Out:** `zc_seq_re`, `zc_seq_im` — 64 × 16 bits each. `full.v` interleaves
  them into `zc_seq_cplx` (`{re, im}` per sample).
- Null subcarriers get 0. Active subcarrier `i` reads LUT entry
  `i - GUARD_COUNT - (i > DC_INDEX)`, so indices 10–31 → LUT 0–21 and
  33–53 → LUT 22–42. The LUT is the C++ `zc_lut` in Q1.15.

### 6.2 `qpsk_signal_128` / `qpsk_signal` — `src/qpsk_signal_128.v`, `src/qpsk_signal.v`

- 64 parallel instances of a 2-bit → I/Q mapper. **Registered: 1 cycle latency.**
- Mapping table in §4. Outputs `I[1023:0]`, `Q[1023:0]`, symbol `k` at `[16k +: 16]`.
- `src/qpsk.v` is an independent second implementation of the same mapping;
  it is **not** used by `full.v`.

### 6.3 `parameterized_map_to_subcarriers` — `src/map_to_subcarriers.v`

- **Registered: 1 cycle latency.** Async reset clears all subcarriers.
- Inputs `sync`, `ref_sym`, `header` are decoded from `sym_idx` in `full.v`
  (`sym_idx == 0 / 1 / 2`). Symbol 3 has all three low → payload.
- Per subcarrier, first match wins (same order as the C++):

  1. null (guard / DC) → `0`
  2. `sync` → `zc_seq[i]`
  3. `ref_sym` → `1.0`
  4. pilot `(i - LAST_GUARD_INDEX) % 6 == 0` → `1.0`
  5. `header` → `1.0`
  6. otherwise → `qpsk_symbol[i]`

### 6.4 `parallel_to_serial` — `src/parallel_to_serial.v`

- On a one-cycle `start` pulse, streams the 64 samples of `parallel_data`
  out at one per clock, sample 0 first, with `do_en` high for exactly 64
  cycles. `frame_done` (unused at top level) pulses after sample 63.

### 6.5 `ifft_64` — `src/ifft_64.v`

Computes `IFFT(x) = conj(FFT(conj(x))) / N` using a forward-only core:

1. `conj` the input (negate `im`).
2. **`FFT`** (`src/FFT64.v`, vendored r22sdf radix-2² single-path delay
   feedback). Streaming, one sample/clock in and out, scales by 1/64
   internally, output in bit-reversed order.
3. `conj` the output.
4. **`bit_reversal_buffer`** — a 4-bank frame FIFO that writes sample `j` to
   slot `bit_reverse(j)` and reads sequentially, producing natural order.
   Has a sticky `overflow` flag (not exported).

Latency ≈ 137 cycles from first input sample to first output sample; 64
samples out with `do_en` high. Scaling is 1/64 total, which equals the C++
`setSch(0x2A)` schedule.

### 6.6 `serial_to_parallel` — `src/serial_to_parallel.v`

- Collects 64 `do_en` samples into `parallel_data[2047:0]`, sample 0 in the
  LSBs, and pulses `frame_done` on the edge that stores sample 63.

### 6.7 `parameterized_add_cp` — `src/add_cp.v`

- **Registered: 1 cycle latency.** Every clock:
  `out[0..15] = in[48..63]`, `out[16..79] = in[0..63]`.

### 6.8 Frame FSM — `full.v`

Two states, `IDLE` and `RUN`, plus a symbol counter `sym_idx` (0–3).

- `IDLE` → clears `sym_idx`, goes to `RUN` (one cycle).
- `RUN` → pulses `symbol_start` once for symbol 0, then once after each
  `symbol_done`.
- On `symbol_done` (= `s2p.frame_done` delayed one cycle so `add_cp` has
  registered): writes `ofdm_symbol_cp` into slot `sym_idx`, increments
  `sym_idx`. After symbol 3: pulses `frame_done`, returns to `IDLE`, and
  the next frame begins.

## 7. Timing

Symbols go through the chain **one at a time** — the next symbol is not
launched until the previous one has been written to the frame.

```
 edge E   : symbol_done; ofdm_frame[k] written; sym_idx ← k+1
 E+1      : mapper registers symbol k+1 into subcarriers;
            FSM raises symbol_start
 E+2      : p2s samples subcarriers[0], streams 64 samples (E+2 .. E+65)
 +~137    : first IFFT output sample
 +64      : s2p collects all 64; s2p.frame_done
 +1       : add_cp registers; symbol_done → next E
```

Per symbol: p2s 64 + IFFT ~135 + s2p 64 + add_cp/FSM ~4 ≈ **~267 cycles**.
Per frame: ≈ **1,070 cycles** (~21 µs at 50 MHz).

The two "launch" cycles (E+1, E+2) matter: `symbol_start` must arrive after
the mapper has registered the new symbol, otherwise p2s would read the
previous symbol's sample 0. This is why the mapper is reset only by `rst`
and never by `symbol_start`.

## 8. FPGA wrapper — `fpga/ofdm_axi_lite.v`

AXI4-Lite slave around the core, for a Zynq PS master.

| Offset        | Name     | Access | Contents                                   |
|---------------|----------|--------|--------------------------------------------|
| `0x000`       | CTRL     | RW     | bit0 `run`                                 |
| `0x004`       | STATUS   | RO     | bit0 `done`, bit1 `busy`                   |
| `0x008–0x014` | QPSK0..3 | RW     | `qpsk_bits[31:0]` … `[127:96]`             |
| `0x018`       | ID       | RO     | `0x0FDA0001`                               |
| `0x400–0x8FC` | FRAME    | RO     | word `n` at `0x400 + 4n`, n = 0..319       |

Software sequence:

```
write CTRL  = 0             // reset core, clear done
write QPSK0..QPSK3          // payload
write CTRL  = 1             // run
poll  STATUS.done == 1
read  FRAME[0..319]
```

When `done` sets, the wrapper holds the core in reset; because `ofdm_frame`
has no reset of its own, the captured frame stays readable. A re-run
**requires** `CTRL = 0` first.

## 9. ASIC top — SPI in, DAC out (`asic/`)

For a chip, the core's 128-bit input and 10,240-bit output cannot go to pins.
`asic/ofdm_asic_top.v` puts the same verified transmit chain behind an SPI
slave for input and an I/Q DAC interface for output.

```
 SPI ──► spi_slave_regs ──► QPSK0..15, CTRL, GAIN
                                │
  zc / qpsk / map ──► p2s ──► ifft_64 ──► frame_mem (256 × 32 bit) ──► DAC pins
                                          written natural order        CP added on read
```

`add_cp`, `serial_to_parallel` and the 10,240-bit `ofdm_frame` register are
**not** used. The IFFT output goes into a 256-word memory (4 symbols × 64
samples, ~8 Kbit, no reset, registered read, so it maps to an SRAM macro or
latch array), and the cyclic prefix is produced by the read order: samples
48–63, then 0–63, of each symbol.

The whole frame is generated before playback starts. The chain needs ~267
clk cycles per symbol but a symbol is only 80 DAC samples, so streaming
straight from the IFFT would leave gaps in the analog waveform. Playing from
memory gives 320 back-to-back samples at a constant rate.

### Parameters

| Parameter           | Default | Meaning                                        |
|---------------------|---------|------------------------------------------------|
| `DAC_W`             | 12      | DAC bits per rail (I and Q), 1–16              |
| `DAC_DIV`           | 2       | clk cycles per DAC sample, ≥ 2                 |
| `DAC_OFFSET_BINARY` | 0       | 0: two's complement, 1: offset binary          |

Out-of-range values fail elaboration deliberately.

### Pins

| Pin            | Dir | Description |
|----------------|-----|-------------|
| `clk`          | in  | system clock |
| `rst_n`        | in  | active-low reset; asserts asynchronously, released synchronously inside |
| `spi_sclk`     | in  | SPI clock, mode 0, ≤ f_clk / 8 |
| `spi_cs_n`     | in  | SPI chip select, active low |
| `spi_mosi`     | in  | SPI data in, MSB first |
| `spi_miso`     | out | SPI data out, MSB first |
| `spi_miso_oe`  | out | pad output enable for MISO, high while selected (not a pin) |
| `dac_i[DAC_W-1:0]` | out | I (real) DAC code |
| `dac_q[DAC_W-1:0]` | out | Q (imag) DAC code |
| `dac_clk`      | out | DAC latch clock, f_clk / `DAC_DIV` |
| `tx_en`        | out | high while frame samples are on the DAC (RF front-end enable) |
| `frame_sync`   | out | high during sample 0 of each frame |

With `DAC_W = 12`: 5 inputs + 28 outputs = **33 pads**, plus power.

### SPI protocol

Mode 0 (CPOL = 0, CPHA = 0), MSB first. One transaction:

```
CS_n low │ command byte {rw, addr[6:0]} │ data byte │ data byte │ … │ CS_n high
           rw = 1 read, 0 write           addr        addr+1
```

The address auto-increments after every data byte, so the 16 QPSK bytes go
in one burst. On a read, MISO carries the register value during each data
byte; MOSI is ignored.

| Addr          | Name     | Access | Contents |
|---------------|----------|--------|----------|
| `0x00`        | CTRL     | RW     | bit0 START (write 1, self-clearing, reads 0); bit1 REPEAT |
| `0x01`        | STATUS   | RO     | bit0 gen_busy, bit1 tx_active, bit2 frame_valid |
| `0x02`–`0x11` | QPSK0–15 | RW     | QPSKb = `qpsk_bits[8b+7 : 8b]` |
| `0x12`        | ID       | RO     | `0xDA` |
| `0x13`        | GAIN     | RW     | bits[2:0]: output gain 2^GAIN, saturating |

Bit mapping inside `qpsk_bits` is the same as §4: subcarrier `k` uses
`qpsk_bits[2k]` for I and `qpsk_bits[2k+1]` for Q.

### Sending a frame

```
SPI write 0x02.. : 16 payload bytes (burst)
SPI write 0x13   : GAIN (optional)
SPI write 0x00   : 0x01  (START)        or 0x03 (START + REPEAT)
                   ~1,070 clk cycles: frame generated, STATUS.gen_busy = 1
                   320 samples on the DAC, tx_en = 1, frame_sync on sample 0
                   DAC returns to zero, tx_en = 0, STATUS = 0x04
```

- **REPEAT = 1** replays the stored frame back-to-back with no gap. Clearing
  REPEAT stops playback at the end of the current frame.
- **START while playing** aborts playback and regenerates (e.g. new payload).
  START while a frame is being generated is ignored, so the FFT always sees
  whole symbols.
- The payload is snapshotted at START, so SPI writes during generation do not
  corrupt the frame in flight.

### DAC output

```
clk        ┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐        DAC_DIV = 2
dac_clk    ──┐ ┌─┐ ┌─┐ ┌─┐ ┌──      DAC latches on the rising edge
             └─┘ └─┘ └─┘ └─┘
dac_i/q    ══X═══X═══X═══X═══      changes on the edge where dac_clk falls
             s0  s1  s2  s3
```

Data changes on the clk edge where `dac_clk` falls, and `dac_clk` rises at
least one clk cycle later, so the DAC gets ≥ 1 clk cycle of setup and hold.
Sample rate = f_clk / `DAC_DIV`; e.g. 100 MHz clk, `DAC_DIV = 2` → 50 MS/s,
and a frame takes 6.4 µs on air.

Sample conversion, per rail: Q1.15 value × 2^GAIN, saturated to 16 bits,
top `DAC_W` bits kept (truncation), MSB inverted if offset binary. Idle
level is code 0 (two's complement) or mid-scale `100…0` (offset binary).

Because the IFFT scales by 1/64, a typical frame uses only a small part of
the Q1.15 range at GAIN = 0. Raising GAIN by 1 doubles the amplitude (6 dB)
and recovers one bit of DAC resolution, at the cost of clipping the rare
high peaks.

## 10. Verification

| Testbench                      | DUT                       | Check type                               |
|--------------------------------|---------------------------|------------------------------------------|
| `tb_zc_seq.v`                  | ZC generator              | inlined LUT copy, exact                  |
| `tb_QPSK_signal.v`, `tb_qpsk.v`| QPSK                      | none (elaboration only)                  |
| `tb_subcarrier.v`              | mapper                    | re-implemented model, exact              |
| `tb_add_cp.v`                  | add_cp                    | exact                                    |
| `tb_bit_reversal_buffer.v`     | reorder buffer            | exact, incl. back-to-back frames         |
| `tb_ifft_64.v`                 | ifft_64                   | generated golden table, ±tolerance       |
| `tb_ofdm_frame.v`              | full core                 | generated golden table, ±tolerance       |
| `tb_full.v`                    | full core                 | live `real`-arithmetic IFFT model        |
| `tb_ofdm_axi_lite.v`           | AXI wrapper + core        | 320-word bit-exact table, re-run         |
| `tb_ofdm_asic_top.v`           | ASIC top (SPI + DAC)      | DAC stream bit-exact vs `ofdm_generate_frame` run alongside; 12-bit offset-binary variant; GAIN, REPEAT, abort |

Also: `fpga/check_frame.py` (numpy model) passed on ZedBoard hardware at
≤1.6 LSB error per symbol.

---

## 11. Design review (2026-10-27)

**Method:** read every RTL file in the chain and the AXI wrapper, compared
stage by stage with `ofdm_hls.cc.cpp`, and ran the full lint and every
testbench.

**Results:** `iverilog -Wall` clean (two dangling-`rst` warnings, below);
`verilator --lint-only -Wall` warning-free on both tops; all 10
testbenches pass.

### Verdict

The RTL is **functionally correct** as a 64-point OFDM transmitter. No
functional bugs were found in the RTL; the findings below are test-coverage,
cleanup and documentation items.

Checked and confirmed matching the C++:

- Null map (guards 0–9 / 54–63, DC 32) and the ZC LUT index formula,
  including the skip over DC.
- Pilot set `(i - 9) % 6 == 0` and the `else if` priority order. Negative
  `i - 9` cannot produce a false pilot because those indices are already
  null.
- QPSK bit order (`bit 2k → I`, `bit 2k+1 → Q`, `1 → negative`) and the
  `1/√2` constant (`0x5A82`, truncated like `ap_fixed`).
- `{re, im}` packing is consistent across every stage boundary.
- IFFT scaling (1/64) equals `setSch(0x2A)`.
- CP and frame layout equal `add_cp` / `add_to_frame`.
- FSM handshake: mapper output is valid one cycle before p2s samples it;
  `add_cp` output is valid when the frame slot is written.
- AXI: write/read handshakes, register decode, frame index window, and the
  freeze-on-done mechanism.

### Findings

| # | Severity | Where | Finding |
|---|----------|-------|---------|
| 1 | Low | `testbenches/tb_subcarrier.v`, `tb_add_cp.v` | `rst` port left unconnected (`iverilog -Wall` reports it). It floats to `z`, so the reset branch of both modules is never exercised. Connect and pulse `rst`. |
| 2 | Low | `full.v` lines 12, 18 | `src/qpsk.v` and `src/uart_rx.v` are `` `include``d but never instantiated. Dead includes; `qpsk.v` is also a duplicate of `qpsk_signal.v`. |
| 3 | Info | `full.v` | Symbols are processed strictly one at a time (~267 cycles each), so p2s / IFFT / s2p sit idle ~75% of the time. Fine for a run-once frame generator; would need overlapping symbol launches for continuous streaming. |
| 4 | Info | `full.v` | `ofdm_generate_frame` loops forever and overwrites `ofdm_frame`; `frame_done` is a single-cycle pulse. Any user other than the AXI wrapper must latch on it. |
| 5 | Info | `full.v` | Only one payload symbol per frame and 72 of 128 input bits are used; symbol 2 ("header") carries no information. Both match the C++ and are protocol choices, not bugs. |
| 6 | Cosmetic | `full.v` lines 23, 206, 248 | Latency comments disagree (267 vs 264), and "add_cp registered on this same edge" is really the previous edge. Behaviour is correct. |

Known limitations already recorded in `AGENTS.md` and still true: the
bit-reversal `overflow` flag is not exported; `-di_im` cannot negate
`16'h8000` (unreachable with the current inputs); `ofdm_frame` synthesises
as flip-flops rather than BRAM.
