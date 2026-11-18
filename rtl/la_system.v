// Everything except the PLL: input synchronisers, UART, SUMP core and the
// SDRAM controller.
module la_system #(
    parameter integer CLK_HZ = 100_285_714,
    parameter integer BAUD   = 115200,
    parameter integer TX_GAP_BITS = 0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] probes_raw,
    input  wire        uart_rx,
    output wire        uart_tx,

    // SDRAM (clock and chip select/CKE are handled by the top level)
    output wire        sd_ras_n,
    output wire        sd_cas_n,
    output wire        sd_we_n,
    output wire [10:0] sd_addr,
    output wire [1:0]  sd_ba,
    inout  wire [31:0] sd_dq,

    output wire        st_armed,
    output wire        st_capturing,
    output wire        st_uploading,
    output wire        st_error,       // SDRAM calibration failed / FIFO overflow
    output wire        st_rx_activity
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;

    // Two-flop synchronisers
    reg [15:0] probes_m, probes_s;
    reg [1:0]  rx_sync;
    always @(posedge clk) begin
        probes_m <= probes_raw;
        probes_s <= probes_m;
        rx_sync  <= {rx_sync[0], uart_rx};
    end

    wire [7:0] rx_data, tx_data;
    wire       rx_valid, tx_valid, tx_ready;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rx(rx_sync[1]), .data(rx_data), .valid(rx_valid));

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT), .GAP_BITS(TX_GAP_BITS)) u_tx (
        .clk(clk), .rst(rst), .data(tx_data), .valid(tx_valid), .ready(tx_ready),
        .tx(uart_tx));

    wire        mem_ready, mem_cal_fail, mem_idle, mem_wr_reset;
    wire [63:0] mem_cal_sel;
    wire [31:0] mem_cal_bitok;
    wire [31:0] mem_fifo_data, mem_rd_data;
    wire        mem_fifo_empty, mem_fifo_pop;
    wire        mem_rd_req, mem_rd_ack;
    wire [20:0] mem_rd_addr;
    wire        overflow;

    sump_core u_core (
        .clk(clk), .rst(rst), .probes(probes_s),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .mem_ready(mem_ready), .mem_cal_fail(mem_cal_fail),
        .mem_cal_sel(mem_cal_sel), .mem_cal_bitok(mem_cal_bitok),
        .mem_idle(mem_idle),
        .mem_wr_reset(mem_wr_reset),
        .mem_fifo_data(mem_fifo_data), .mem_fifo_empty(mem_fifo_empty),
        .mem_fifo_pop(mem_fifo_pop),
        .mem_rd_req(mem_rd_req), .mem_rd_addr(mem_rd_addr),
        .mem_rd_ack(mem_rd_ack), .mem_rd_data(mem_rd_data),
        .st_armed(st_armed), .st_capturing(st_capturing),
        .st_uploading(st_uploading), .st_overflow(overflow));

    sdram_ctrl #(.CLK_HZ(CLK_HZ)) u_sdram (
        .clk(clk), .rst(rst),
        .sd_ras_n(sd_ras_n), .sd_cas_n(sd_cas_n), .sd_we_n(sd_we_n),
        .sd_addr(sd_addr), .sd_ba(sd_ba), .sd_dq(sd_dq),
        .ready(mem_ready), .cal_fail(mem_cal_fail), .cal_sel(mem_cal_sel),
        .cal_bit_ok(mem_cal_bitok),
        .idle(mem_idle),
        .wr_reset(mem_wr_reset),
        .fifo_data(mem_fifo_data), .fifo_empty(mem_fifo_empty),
        .fifo_pop(mem_fifo_pop),
        .rd_req(mem_rd_req), .rd_addr(mem_rd_addr),
        .rd_ack(mem_rd_ack), .rd_data(mem_rd_data));

    assign st_error       = mem_cal_fail | overflow;
    assign st_rx_activity = !rx_sync[1];
endmodule
