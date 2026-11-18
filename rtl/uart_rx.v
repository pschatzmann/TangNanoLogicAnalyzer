// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Phil Schatzmann
// 8N1 UART receiver. rx must already be synchronised to clk.
module uart_rx #(
    parameter integer CLKS_PER_BIT = 868
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid      // one-cycle strobe
);
    localparam integer CW = $clog2(CLKS_PER_BIT * 3 / 2 + 1);

    reg [1:0]    state;
    reg [CW-1:0] cnt;
    reg [2:0]    bitn;
    reg [7:0]    shift;

    localparam S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
        end else begin
            case (state)
            S_IDLE:
                if (!rx) begin
                    state <= S_START;
                    cnt   <= CLKS_PER_BIT / 2 - 1;     // to middle of start bit
                end
            S_START:
                if (cnt == 0) begin
                    if (!rx) begin
                        state <= S_DATA;
                        cnt   <= CLKS_PER_BIT - 1;
                        bitn  <= 3'd0;
                    end else
                        state <= S_IDLE;               // glitch
                end else
                    cnt <= cnt - 1'b1;
            S_DATA:
                if (cnt == 0) begin
                    shift <= {rx, shift[7:1]};          // LSB first
                    cnt   <= CLKS_PER_BIT - 1;
                    if (bitn == 3'd7) state <= S_STOP;
                    bitn  <= bitn + 1'b1;
                end else
                    cnt <= cnt - 1'b1;
            S_STOP:
                if (cnt == 0) begin
                    // Accept the byte even with a bad stop bit; return to idle
                    // as soon as the line is high so the next start bit is seen.
                    data  <= shift;
                    valid <= rx;
                    state <= S_IDLE;
                end else
                    cnt <= cnt - 1'b1;
            endcase
        end
    end
endmodule
