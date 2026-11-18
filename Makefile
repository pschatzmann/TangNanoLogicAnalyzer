# Tang Nano 20K SUMP logic analyzer - open source flow
# (yosys -> nextpnr-himbaechel -> gowin_pack -> openFPGALoader)
#
#   make            build the bitstream (build/la.fs)
#   make load       load into SRAM (lost at power off)
#   make flash      write to the onboard SPI flash (persistent)
#   make sim        run the iverilog testbench
#   make cosim-test end-to-end test with sigrok-cli against a Verilator model
#   make BAUD=115200 ...  use another UART speed (must match PulseView)

OSS_CAD ?= $(HOME)/oss-cad-suite
export PATH := $(OSS_CAD)/bin:$(PATH)

DEVICE  = GW2AR-LV18QN88C8/I7
FAMILY  = GW2A-18C
BAUD   ?= 921600
TX_GAP ?= 3
CST     = tangnano20k.cst
BUILD   = build

RTL = rtl/top.v rtl/pll.v rtl/la_system.v rtl/sump_core.v \
      rtl/uart_rx.v rtl/uart_tx.v rtl/sdram_ctrl.v rtl/sync_fifo.v
CORE_RTL = rtl/la_system.v rtl/sump_core.v rtl/uart_rx.v rtl/uart_tx.v rtl/sdram_ctrl.v rtl/sync_fifo.v

all: $(BUILD)/la.fs

$(BUILD)/la.json: $(RTL) Makefile $(BUILD)/options | $(BUILD)
	yosys -q -l $(BUILD)/yosys.log -p "read_verilog -DBAUD=$(BAUD) -DTX_GAP=$(TX_GAP) $(RTL); \
	  synth_gowin -top top; simplemap t:\$$buf; opt_clean; \
	  tee -o $(BUILD)/utilization.txt stat; write_json $@"

$(BUILD)/la_pnr.json: $(BUILD)/la.json $(CST)
	nextpnr-himbaechel --json $< --write $@ --device $(DEVICE) \
	  --vopt family=$(FAMILY) --vopt cst=$(CST) --freq 100 \
	  --log $(BUILD)/nextpnr.log --report $(BUILD)/report.json

# apycula warns when its optional speed-ups (numpy, msgspec, fastcrc) are
# missing; harmless, so hide it. Better: install them into oss-cad-suite's
# Python (see README).
$(BUILD)/la.fs: $(BUILD)/la_pnr.json
	PYTHONWARNINGS="ignore::UserWarning" gowin_pack -d $(FAMILY) -o $@ $<

$(BUILD):
	mkdir -p $@

# rebuild when BAUD / TX_GAP change
$(BUILD)/options: FORCE | $(BUILD)
	@echo "BAUD=$(BAUD) TX_GAP=$(TX_GAP)" | cmp -s - $@ || echo "BAUD=$(BAUD) TX_GAP=$(TX_GAP)" > $@

FORCE:

load: $(BUILD)/la.fs
	openFPGALoader -b tangnano20k $<

flash: $(BUILD)/la.fs
	openFPGALoader -b tangnano20k -f $<

# make sim SIM_DEFS=-DSD_DELAY=3.0  simulates a different SDRAM output delay (ns)
sim:
	mkdir -p $(BUILD)
	iverilog -g2012 -Wall -Wno-timescale -Wno-sensitivity-entire-array $(SIM_DEFS) -o $(BUILD)/tb.vvp sim/tb_la.v sim/sdram_model.v $(CORE_RTL)
	cd $(BUILD) && vvp -n tb.vvp

COSIM = $(BUILD)/cosim
$(COSIM)/obj/Vvtop: sim/pty/vtop.v sim/pty/main.cpp sim/sdram_model.v $(CORE_RTL)
	mkdir -p $(COSIM)
	verilator --cc --exe --build --timing -O3 -Wno-fatal -Wno-lint -Wno-style \
	  --top-module vtop -Mdir $(COSIM)/obj -j 8 \
	  sim/pty/vtop.v sim/sdram_model.v $(CORE_RTL) $(abspath sim/pty/main.cpp)

$(COSIM)/ptyshim.so: sim/pty/ptyshim.c
	mkdir -p $(COSIM)
	$(CC) -shared -fPIC -O2 -o $@ $< -ldl

cosim-test: $(COSIM)/obj/Vvtop $(COSIM)/ptyshim.so
	sim/pty/run_sigrok_test.sh $(BUILD)

clean:
	rm -rf $(BUILD)

.PHONY: all load flash sim cosim-test clean
