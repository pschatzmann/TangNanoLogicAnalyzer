// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Phil Schatzmann
// Tang Nano 20K - 16 channel SUMP logic analyzer for PulseView / sigrok,
// with 8 MB of sample memory in the embedded SDRAM.
//
// Host link: the onboard BL616 USB-UART bridge (second serial port of the
// board's USB connection), 8N1, BAUD below (set with `make BAUD=...`).
//
// LEDs (active low):
//   LED0  armed, waiting for trigger
//   LED1  triggered, capturing post-trigger samples
//   LED2  uploading samples to the host
//   LED3  serial data from host
//   LED4  error: SDRAM read calibration failed, or FIFO overflow
//   LED5  heartbeat (bitstream running, PLL locked)
`ifndef BAUD
`define BAUD 921600
`endif
// Idle bit times after each byte sent to the PC: the BL616 USB bridge loses
// data when it gets a continuous stream at high baud rates.
`ifndef TX_GAP
`define TX_GAP 3
`endif

module top (
    input  wire        clk_27m,
    input  wire [15:0] probe,
    input  wire        uart_rx,
    output wire        uart_tx,
    output wire [5:0]  led,

    // Embedded SDRAM - these exact port names are what nextpnr's Gowin
    // backend binds to the in-package SDRAM; they are not in the .cst file.
    output wire        O_sdram_clk,
    output wire        O_sdram_cke,
    output wire        O_sdram_cs_n,
    output wire        O_sdram_cas_n,
    output wire        O_sdram_ras_n,
    output wire        O_sdram_wen_n,
    output wire [3:0]  O_sdram_dqm,
    output wire [10:0] O_sdram_addr,
    output wire [1:0]  O_sdram_ba,
    inout  wire [31:0] IO_sdram_dq
);
    localparam integer CLK_HZ = 100_285_714;

    wire clk, clk_p, lock;
    pll u_pll (.clkin(clk_27m), .clkout(clk), .clkoutp(clk_p), .lock(lock));

    // Reset held until the PLL has locked
    reg [3:0] rst_sr = 4'hF;
    always @(posedge clk or negedge lock)
        if (!lock) rst_sr <= 4'hF;
        else       rst_sr <= {rst_sr[2:0], 1'b0};
    wire rst = rst_sr[3];

    assign O_sdram_clk  = clk_p;
    assign O_sdram_cke  = 1'b1;
    assign O_sdram_cs_n = 1'b0;
    assign O_sdram_dqm  = 4'b0000;

    wire armed, capturing, uploading, error, rx_act;

    la_system #(.CLK_HZ(CLK_HZ), .BAUD(`BAUD), .TX_GAP_BITS(`TX_GAP)) u_la (
        .clk(clk), .rst(rst),
        .probes_raw(probe),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .sd_ras_n(O_sdram_ras_n), .sd_cas_n(O_sdram_cas_n),
        .sd_we_n(O_sdram_wen_n), .sd_addr(O_sdram_addr), .sd_ba(O_sdram_ba),
        .sd_dq(IO_sdram_dq),
        .st_armed(armed), .st_capturing(capturing),
        .st_uploading(uploading), .st_error(error), .st_rx_activity(rx_act));

    // Stretch short events so they are visible
    reg [22:0] rx_stretch;
    always @(posedge clk)
        if (rx_act) rx_stretch <= 23'h7FFFFF;
        else if (rx_stretch != 0) rx_stretch <= rx_stretch - 1'b1;

    reg [26:0] hb;
    always @(posedge clk) hb <= hb + 1'b1;

    assign led[0] = ~armed;
    assign led[1] = ~capturing;
    assign led[2] = ~uploading;
    assign led[3] = ~(rx_stretch != 0);
    assign led[4] = ~error;
    assign led[5] = ~hb[26];
endmodule
