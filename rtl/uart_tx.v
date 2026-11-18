// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Phil Schatzmann
// 8N1 UART transmitter with valid/ready handshake. GAP_BITS adds idle bit
// times after each stop bit (paces the data for slow USB bridges).
module uart_tx #(
    parameter integer CLKS_PER_BIT = 868,
    parameter integer GAP_BITS     = 0
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       valid,
    output wire       ready,
    output reg        tx
);
    localparam integer CW = $clog2(CLKS_PER_BIT + 1);

    reg          busy;
    reg [CW-1:0] cnt;
    reg [5:0]    bitn;
    reg [9:0]    shift;     // {stop, data[7:0], start}

    assign ready = !busy;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            tx   <= 1'b1;
        end else if (!busy) begin
            tx <= 1'b1;
            if (valid) begin
                busy  <= 1'b1;
                shift <= {1'b1, data, 1'b0};
                bitn  <= 6'd0;
                cnt   <= 0;
            end
        end else if (cnt == 0) begin
            if (bitn == 6'd10 + GAP_BITS) begin
                busy <= 1'b0;
            end else begin
                tx    <= shift[0];
                shift <= {1'b1, shift[9:1]};
                bitn  <= bitn + 1'b1;
                cnt   <= CLKS_PER_BIT - 1;
            end
        end else
            cnt <= cnt - 1'b1;
    end
endmodule
