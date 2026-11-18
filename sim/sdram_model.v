// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Phil Schatzmann
`timescale 1ns/1ps
// Behavioural model of a 2M x 32 SDR SDRAM (4 banks x 2048 rows x 256 cols)
// for simulation. Burst length 1 only. Checks basic protocol rules and
// models output timing (data valid TAC after the clock edge before the CAS
// latency edge, held until TOH after it) plus an extra board/IO delay.
module sdram_model #(
    parameter real TAC   = 5.4,
    parameter real TOH   = 2.5,
    parameter real DELAY = 1.0,     // extra output delay towards the FPGA
    parameter [31:0] STUCK1 = 32'h0, // data bits that always read as 1 (hardware fault)
    parameter real SKEW_HI = 0.0    // extra delay for DQ[23:16] (pin skew)
) (
    input  wire        clk,
    input  wire        cke,
    input  wire        cs_n,
    input  wire        ras_n,
    input  wire        cas_n,
    input  wire        we_n,
    input  wire [1:0]  ba,
    input  wire [10:0] a,
    input  wire [3:0]  dqm,
    inout  wire [31:0] dq
);
    reg [31:0] mem [0:(1<<21)-1];
    reg [10:0] row_open [0:3];
    reg        active   [0:3];
    integer    t_act    [0:3];   // cycle of last ACT per bank
    integer    t_pre    [0:3];   // cycle of last PRE per bank
    integer    t_ref;
    integer    cyc;
    integer    errors;
    integer    writes, reads, refreshes;
    reg [2:0]  cl;
    reg [2:0]  bl;           // burst length code (0: 1, 1: 2)
    reg        single_wr;    // A9: single location writes
    reg        mode_set;

    reg [31:0] dq_drv;
    reg        dq_en;
    assign #(DELAY)           dq[15:0]  = dq_en ? dq_drv[15:0]  : 16'bz;
    assign #(DELAY + SKEW_HI) dq[23:16] = dq_en ? dq_drv[23:16] : 8'bz;
    assign #(DELAY)           dq[31:24] = dq_en ? dq_drv[31:24] : 8'bz;

    // read pipeline: data scheduled for output
    reg [31:0] rq_data [0:7];
    reg        rq_v    [0:7];

    integer b, n;
    initial begin
        cyc = 0; errors = 0; writes = 0; reads = 0; refreshes = 0;
        t_ref = -100; mode_set = 0; cl = 3; dq_en = 0;
        for (b = 0; b < 4; b = b + 1) begin
            active[b] = 0; t_act[b] = -100; t_pre[b] = -100;
        end
        for (n = 0; n < 8; n = n + 1) rq_v[n] = 0;
    end

    task err(input [8*64-1:0] msg);
        begin
            errors = errors + 1;
            if (errors < 20) $display("%t SDRAM ERROR: %0s", $time, msg);
        end
    endtask

    wire [3:0] cmd = {cs_n, ras_n, cas_n, we_n};

    always @(posedge clk) begin
        cyc = cyc + 1;

        // ---- output pipeline: slot 1 = data for the next edge ----
        if (rq_v[1]) begin
            dq_drv <= #(TAC) rq_data[1];
            dq_en  <= #(TAC) 1'b1;
        end
        if (rq_v[0]) begin
            // after TOH the old data is gone: bus floats, or changes to the
            // next burst word (driven TAC after this edge)
            if (!rq_v[1]) dq_en <= #(TOH) 1'b0;
            dq_drv <= #(TOH) 32'hxxxx_xxxx;
        end
        for (n = 0; n < 7; n = n + 1) begin
            rq_v[n]    = rq_v[n+1];
            rq_data[n] = rq_data[n+1];
        end
        rq_v[7] = 0;

        if (cke) case (cmd)
        4'b0011: begin // ACTIVATE
            if (active[ba]) err("ACT on active bank");
            if (cyc - t_pre[ba] < 2) err("tRP violated (ACT)");
            if (cyc - t_act[ba] < 6) err("tRC violated (ACT)");
            if (cyc - t_ref < 7) err("tRFC violated (ACT)");
            active[ba]   = 1;
            row_open[ba] = a;
            t_act[ba]    = cyc;
        end
        4'b0101: begin // READ
            if (!mode_set) err("READ before mode register set");
            if (!active[ba]) err("READ on idle bank");
            if (cyc - t_act[ba] < 2) err("tRCD violated (READ)");
            // data must be valid at the edge cl cycles from now: driven
            // TAC after edge cl-1, held until TOH after edge cl
            rq_data[cl-1] = mem[{row_open[ba], ba, a[7:0]}] | STUCK1;
            rq_v[cl-1]    = 1;
            if (bl == 3'd1) begin       // second word of a 2-word burst
                rq_data[cl] = mem[{row_open[ba], ba, a[7:1], ~a[0]}] | STUCK1;
                rq_v[cl]    = 1;
            end
            reads = reads + 1;
            if (a[10]) begin active[ba] = 0; t_pre[ba] = cyc + 2; end
        end
        4'b0100: begin // WRITE
            if (!mode_set) err("WRITE before mode register set");
            if (!active[ba]) err("WRITE on idle bank");
            if (cyc - t_act[ba] < 2) err("tRCD violated (WRITE)");
            if (^dq === 1'bx) err("WRITE data contains X/Z");
            if (dqm != 4'b0000) err("unexpected DQM");
            mem[{row_open[ba], ba, a[7:0]}] = dq;
            writes = writes + 1;
            if (a[10]) begin active[ba] = 0; t_pre[ba] = cyc + 2; end
        end
        4'b0010: begin // PRECHARGE
            for (b = 0; b < 4; b = b + 1)
                if (a[10] || ba == b) begin
                    if (active[b] && cyc - t_act[b] < 5) err("tRAS violated");
                    active[b] = 0;
                    t_pre[b]  = cyc;
                end
        end
        4'b0001: begin // AUTO REFRESH
            for (b = 0; b < 4; b = b + 1) begin
                if (active[b]) err("REFRESH with active bank");
                if (cyc - t_pre[b] < 2) err("tRP violated (REF)");
            end
            if (cyc - t_ref < 7) err("tRFC violated (REF)");
            t_ref = cyc;
            refreshes = refreshes + 1;
        end
        4'b0000: begin // MODE REGISTER SET
            for (b = 0; b < 4; b = b + 1)
                if (active[b]) err("MRS with active bank");
            cl = a[6:4];
            bl = a[2:0];
            single_wr = a[9];
            if (bl > 3'd1) err("burst length not 1 or 2");
            if (bl != 3'd0 && !single_wr) err("write bursts not supported by model");
            mode_set = 1;
        end
        default: ;
        endcase
    end
endmodule
