# Tang Nano 20K Logic Analyzer

A 16-channel, 100 MS/s logic analyzer for the
[Sipeed Tang Nano 20K](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html).
It speaks the SUMP / Openbench Logic Sniffer protocol, so it works with
[PulseView](https://sigrok.org/wiki/PulseView) and `sigrok-cli` with no
extra software. Samples are stored in the SDRAM built into the GW2AR-18
package.

| | |
|---|---|
| Channels | 16 (two groups of 8), 3.3 V LVCMOS inputs. **Not 5 V tolerant.** |
| Sample rate | 10 Hz – 100 MHz (100 MHz / n), signals up to about 25 MHz |
| Memory | 8 MB SDRAM: **4,194,288 samples** with 16 channels, **8,388,576 samples** with only channels 0–7 or only 8–15 enabled |
| Trigger | 4-stage parallel trigger (each stage matches selected channels against 0/1), pre-trigger buffer from 0 to 100% |
| Test mode | PulseView option *Pattern: Internal* captures a counter, so the analyzer can be tested with nothing connected |
| Host link | USB, through the board's BL616 USB-serial bridge (the same cable used for programming), 921600 baud |

## Pinout

![Tang Nano 20K pinout](https://wiki.sipeed.com/hardware/zh/tang/tang-nano-20k/assets/nano_20k/tang_nano_20k_pinlabel.png)

*Pinout image: [Sipeed wiki](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html). The grey numbers are the FPGA pin numbers.*

The pin numbers are the FPGA pin numbers (grey in the image). "Header" gives
the position on the board's pin headers, counted from the USB-C end (1) to
the HDMI end (20), with the board oriented as in the image.

| Channel | Pin | Header | | Channel | Pin | Header |
|---|---|---|---|---|---|---|
| 0 | 25 | left 10  | | 8  | 42 | right 5  |
| 1 | 26 | left 11  | | 9  | 48 | right 10 |
| 2 | 27 | left 8   | | 10 | 49 | right 12 |
| 3 | 28 | left 9   | | 11 | 73 | left 1   |
| 4 | 29 | left 12  | | 12 | 74 | left 2   |
| 5 | 30 | left 13  | | 13 | 75 | left 3   |
| 6 | 31 | left 14  | | 14 | 76 | right 3  |
| 7 | 41 | right 6  | | 15 | 77 | left 5   |

Ground is on left 20 and right 2 and 15. Connect GND to the device you are
measuring. Inputs have weak pull-downs, so unconnected channels read 0. To
remove them, set `PULL_MODE=NONE` in [tangnano20k.cst](tangnano20k.cst).

Several of these pins are also wired to the board's LCD connector (see the
"Onboard component" labels in the image). Don't use them for measuring while
an LCD is attached. The pins next to them labelled LED0–LED5 (15–20) drive
the onboard LEDs and are not inputs of the analyzer.

The onboard LEDs (`led[0]`…`led[5]` = FPGA pins 15…20):

| LED | Meaning |
|---|---|
| LED0 | armed, waiting for the trigger |
| LED1 | triggered, recording the post-trigger part |
| LED2 | uploading samples to the PC |
| LED3 | data received from the PC |
| LED4 | **error**: SDRAM read calibration failed, or samples were lost while capturing |
| LED5 | heartbeat (bitstream running) |

## Build and load

Requires the open-source Gowin flow from
[oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build) (yosys,
nextpnr-himbaechel, gowin_pack, openFPGALoader). The Makefile looks for it in
`~/oss-cad-suite`; override that with `make OSS_CAD=/path`.

`gowin_pack` (apycula) runs faster with its optional helper packages. The
Makefile hides the warnings it prints when they are missing; to install them
instead:

```sh
~/oss-cad-suite/py3bin/python3 -m pip install numpy msgspec fastcrc
```

```sh
make            # -> build/la.fs
make flash      # write to the onboard flash (persistent)
make load       # or: load into SRAM only (lost at power off)
```

`make load` is quicker for trying out a build, but the bitstream is lost when
the board loses power, and the board then boots whatever is in its flash
again. To check which version is running, look at the device line printed by
`sigrok-cli … --show` (`FPGA version 2.40`, …) or the device name in
PulseView. If it is not the version you expect, run `make flash`.

## Use with PulseView

1. *Connect to Device* → driver **Openbench Logic Sniffer & SUMP compatibles (ols)**.
2. Interface: **Serial Port**, choose the board's **second** serial port.
   On Linux that is usually `/dev/ttyUSB1`; `/dev/ttyUSB0` is the JTAG interface.
   Baud rate **921600**.
3. *Scan for devices* should find `Tang Nano 20K LA … with 16 channels`.

If the board is unplugged while a program still has its port open, Linux
gives it a new name when it comes back (e.g. `/dev/ttyUSB2` instead of
`/dev/ttyUSB1`). Pick the new name, or close the connection in PulseView
before unplugging. `ls /dev/ttyUSB*` shows the current names.

From the command line:

```sh
sigrok-cli -d ols:conn=/dev/ttyUSB1:serialcomm=921600/8n1 --show
sigrok-cli -d ols:conn=/dev/ttyUSB1:serialcomm=921600/8n1 --config samplerate=10M \
           --samples 100000 -t 0=0 -O vcd -o capture.vcd
```

### Sample rate and signal speed

The highest sample rate is 100 MS/s, on all 16 channels and for the full
memory. libsigrok derives the rates from a 100 MHz base clock, so the
available rates are 100 MHz divided by a whole number: 100, 50, 33.3, 25, 20
MHz and so on, down to 10 Hz.

PulseView's sample-rate list goes up to 200 MHz. That list is fixed in
libsigrok 0.5.x for all devices of this driver type and ignores what the
device supports. When a rate is actually set, libsigrok checks the 100 MHz
limit the analyzer reports and rejects anything above it ("invalid
samplerate" in sigrok-cli; tested on the board with 150 and 200 MHz). So
choose 100 MHz or less.

At 100 MS/s a sample is taken every 10 ns, so each edge is placed to within
±10 ns. Digital signals up to about 25 MHz (4 samples per period) are shown
reliably; protocol decoders work well up to about 10–20 MHz. At these speeds
the wiring to the header matters too: keep wires short and use a ground wire
next to the signal.

### Capture length and when a capture stops

A capture always records a fixed number of samples, set next to PulseView's
*Run* button. The largest values offered are the memory limits from the
table above.

* **No trigger:** recording starts immediately and stops by itself once the
  requested number of samples has been recorded; then the samples are
  uploaded.
* **With a trigger:** the analyzer records continuously into its memory (a
  ring buffer) until the trigger condition occurs, then records the
  post-trigger part (the pre-trigger share is PulseView's trigger position
  setting) and stops. If the condition never occurs it keeps waiting. *Stop*
  in PulseView then aborts the capture without data, because libsigrok resets
  the device instead of asking for what has been recorded so far.

The analyzer cannot stream continuously; the length is always limited by its
memory and by the upload speed below.

### Upload speed

Samples go to the PC over the serial link: 16 channels take 2 bytes per
sample, a single 8-channel group takes 1. The link runs at 921600 baud, but
the board's BL616 USB bridge loses data when it receives a continuous stream
at that rate (measured: 4–6% of the bytes lost). The analyzer therefore
leaves 3 idle bit-times after each byte it sends, which brings the upload to
about 69 kB/s with no loss. 100k samples on 16 channels take about 3 s.
Commands from the PC are not slowed down.

**Large captures:** libsigrok 0.5.x (used by sigrok-cli and PulseView 0.4.2)
handles every received byte separately and cannot keep up with a long
upload: 1M samples on 16 channels (2 MB) arrived complete in testing, but a
whole-memory capture lost data, and once a 100k capture lost data while the
PC was busy with something else. A program that reads the port in large
blocks receives everything (tested with the full memory). Captures up to
about 1M samples are normally safe; for more, try `make TX_GAP=10`, which
trades speed for more margin. If sigrok-cli reports "Device only sent N
samples", data was lost during the upload; repeat the capture.

To capture only channels 0–7 (or 8–15), disable the other group in
PulseView; this halves the upload time and doubles the maximum capture
length.

Both settings can be changed at build time. Select the same baud rate in
PulseView, or use `serialcomm=<baud>/8n1` with sigrok-cli:

```sh
make clean && make BAUD=115200 TX_GAP=0 && make flash BAUD=115200 TX_GAP=0
```

Pass the same `BAUD`/`TX_GAP` to every `make` call (including `load` and
`flash`); a change in either rebuilds the bitstream. The defaults are
`BAUD=921600` and `TX_GAP=3`.

### Notes and limitations

* The sample clock is 100.29 MHz, not exactly 100 MHz, because 100 MHz cannot
  be generated from the 27 MHz crystal within the PLL's limits. All timings
  shown in PulseView are therefore about 0.3% long. This doesn't matter for
  decoding UART, SPI, I2C and similar protocols.
* Not implemented: 200 MHz "demux" mode, RLE compression, external clock,
  serial trigger mode, and per-stage trigger delays. PulseView offers RLE and
  external clock, but leave them off. 200 MHz would be possible with the
  FPGA's double-data-rate inputs, probably for 8 channels only.
* The libsigrok driver offers only level triggers (0/1) for this device, not
  edge triggers. The hardware supports multi-stage sequences such as "0
  then 1", but whether you can set those up depends on the front end.
* Triggering works with both libsigrok 0.5.x (the version in most Linux
  distributions and the PulseView 0.4.2 release) and current libsigrok, which
  program triggers differently. In both cases the trigger marker lands on the
  matching sample.

## How it works

```
probes ─► 2-FF sync ─► divider ─► trigger ─► byte stream, 4 bytes/word ─► FIFO ─► SDRAM ring buffer
                                                                               │
PC ◄── UART ◄── SUMP command parser / readout (newest sample first) ◄──────────┘
```

| File | Purpose |
|---|---|
| [rtl/top.v](rtl/top.v) | Board top level: PLL, reset, LEDs, SDRAM pins |
| [rtl/pll.v](rtl/pll.v) | 27 MHz → 100.29 MHz system clock + 180° SDRAM clock |
| [rtl/la_system.v](rtl/la_system.v) | Input synchronisers, UART, SUMP core, SDRAM controller |
| [rtl/sump_core.v](rtl/sump_core.v) | SUMP protocol, trigger, sample packing, readout, metadata |
| [rtl/sdram_ctrl.v](rtl/sdram_ctrl.v) | Streaming SDRAM controller with read-capture calibration |
| [rtl/sync_fifo.v](rtl/sync_fifo.v), [rtl/uart_rx.v](rtl/uart_rx.v), [rtl/uart_tx.v](rtl/uart_tx.v) | Building blocks |

**SDRAM.** The 64 Mbit SDRAM (2M × 32 bit) runs from the same 100 MHz clock
as the sampler. Samples form a byte stream (1 or 2 bytes per sample) that is
packed 4 bytes per word. While capturing, the controller keeps a row open and
writes one word per clock as long as the FIFO has data, which gives about 95M
words per second. The worst case, 16 channels at 100 MS/s, needs 50M words
per second. The SDRAM pins are bound by name (`O_sdram_*`, `IO_sdram_dq`) by
nextpnr, so they are not in the `.cst` file.

**Read calibration.** At 100 MHz the window in which read data from the SDRAM
is valid is only a few nanoseconds wide, and the DQ pins sit on both sides of
the chip with different delays. At power-up the controller writes pairs of
complementary test words, reads them back as 2-word bursts, and samples each
data line at four points (rising and falling clock edges at two latencies).
Before each read it drives the complement onto the bus, so a capture that is
too early or too late is detected. Each bit then uses the first point that
read every test word correctly. `--show` prints the result as
`Ancillary version sdram:` followed by the point (0–3) used for DQ31 … DQ0,
or `-` for a line with no working point. In that case LED4 lights. On the
test board DQ30/31, whose pins are far from the other data pins, need a
later point than the rest (`sdram:2211…1`).

**Writes.** The controller keeps driving the data bus for the whole time a
row is open for writing, not only in write cycles. On the board, switching
the output enable on for each write was too slow on some data lines (DQ16/17,
DQ30/31), and the SDRAM stored the old bus value.

## Testing

```sh
make sim          # iverilog testbench with an SDRAM timing model
make cosim-test   # the design in Verilator behind a pseudo terminal, tested with sigrok-cli
```

* `make sim` drives the design over its UART the way libsigrok does. It
  checks ID, metadata, captures in all channel-group modes, dividers, 1- and
  2-stage triggers in both libsigrok styles, captures across SDRAM rows and
  refreshes, abort and finish-now. The SDRAM model also flags protocol
  violations (tRCD, tRP, tRFC, accesses to closed rows, and so on). Run
  `make sim SIM_DEFS="-DSD_DELAY=4.0 -DSD_SKEW=3.0"` to simulate a different
  SDRAM output delay and extra delay on DQ16–23.
* `make cosim-test` runs the real `sigrok-cli` against the simulated board,
  so it also checks compatibility with the libsigrok driver PulseView uses.
  It checks captured data and the trigger position, including a trigger that
  fires after the 4M-sample ring buffer has wrapped. A small `LD_PRELOAD`
  shim ([sim/pty/ptyshim.c](sim/pty/ptyshim.c)) lets libserialport open a
  pseudo terminal; it is used only for this test.

With the board connected, [tools/hw_test.sh](tools/hw_test.sh) tests the
real hardware using the internal test pattern (no probes needed). It checks
captures in all channel modes at 100 MS/s and the trigger position, and with
`--full` also a capture of the whole memory:

```sh
make load
tools/hw_test.sh /dev/ttyUSB1 921600 --full     # port, baud rate, optional --full
```

All tests should pass. Because of the libsigrok limit described under
*Upload speed*, the `--full` capture may fail through sigrok-cli even when
the hardware is fine.

**Simulation does not cover synthesis.** Two of the bugs found on the board
came from how the design was synthesized and placed, not from its logic:
yosys put the FIFO into a block RAM (now forced into LUT RAM), and the
output-enable timing of some SDRAM data lines. Run the hardware test after
changing the gateware.

**Don't leave a bitstream loaded that transmits on its own while nothing
reads the port.** During testing the BL616 bridge stopped passing UART data
twice in that situation (e.g. a diagnostic that printed every second, left
running during a long build) and only recovered after unplugging the board.
The analyzer itself only sends when asked.

## License

Copyright 2026 Phil Schatzmann. Licensed under the
[Apache License, Version 2.0](LICENSE); each source file carries an
`SPDX-License-Identifier: Apache-2.0` header.

The pinout image in the [Pinout](#pinout) section is Sipeed's and is only
linked from their wiki, not part of this repository.
