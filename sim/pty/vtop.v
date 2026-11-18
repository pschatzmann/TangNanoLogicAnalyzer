`timescale 1ns/1ps
// Top level for the pty co-simulation: la_system + SDRAM model, UART and probes from C++.
module vtop (
    input  wire        uart_rx,
    output wire        uart_tx,
    input  wire [15:0] probes,
    output reg         clk
);
    initial clk = 1'b0;
    always #5 clk = ~clk;              // 100 MHz

    reg rst = 1'b1;
    initial #200 rst = 1'b0;

    wire        sd_ras_n, sd_cas_n, sd_we_n;
    wire [10:0] sd_addr;
    wire [1:0]  sd_ba;
    wire [31:0] sd_dq;
    wire a, c, u, e, r;

    la_system #(.CLK_HZ(100_000_000), .BAUD(12_500_000)) dut (
        .clk(clk), .rst(rst), .probes_raw(probes),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .sd_ras_n(sd_ras_n), .sd_cas_n(sd_cas_n), .sd_we_n(sd_we_n),
        .sd_addr(sd_addr), .sd_ba(sd_ba), .sd_dq(sd_dq),
        .st_armed(a), .st_capturing(c), .st_uploading(u), .st_error(e),
        .st_rx_activity(r));

    sdram_model #(.TAC(5.4), .TOH(2.5), .DELAY(1.0)) sdram (
        .clk(~clk), .cke(1'b1), .cs_n(1'b0),
        .ras_n(sd_ras_n), .cas_n(sd_cas_n), .we_n(sd_we_n),
        .ba(sd_ba), .a(sd_addr), .dqm(4'b0000), .dq(sd_dq));

    reg ready_shown = 0;
    always @(posedge clk)
        if (dut.mem_ready && !ready_shown) begin
            ready_shown <= 1;
            $display("[sim] SDRAM ready, cal_sel=%0d cal_fail=%0d", dut.mem_cal_sel, dut.mem_cal_fail);
        end
    always @(posedge clk)
        if (e) begin $display("[sim] ERROR flag set"); $finish; end
endmodule
