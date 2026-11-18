`timescale 1ns/1ps
// Testbench: la_system + SDRAM model, driven over the UART like libsigrok's
// "ols" driver does. Run with `make sim`.
module tb_la;
    localparam integer CLK_HZ = 100_000_000;
    localparam integer BAUD   = 10_000_000;           // fast UART for simulation
    localparam real    BIT_NS = 1.0e9 / BAUD;

    reg clk = 0;
    always #5 clk = ~clk;                             // 100 MHz
    reg rst = 1;

    // Probes: free running counter (changes every clock), or forced
    reg [15:0] cyc = 0;
    always @(posedge clk) cyc <= cyc + 1'b1;
    reg        probes_force = 0;
    reg [15:0] probes_val = 0;
    wire [15:0] probes = probes_force ? probes_val : cyc;

    reg  uart_rx = 1;
    wire uart_tx;

    wire        sd_ras_n, sd_cas_n, sd_we_n;
    wire [10:0] sd_addr;
    wire [1:0]  sd_ba;
    wire [31:0] sd_dq;
    wire st_armed, st_capturing, st_uploading, st_error, st_rx;

    la_system #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) dut (
        .clk(clk), .rst(rst), .probes_raw(probes),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .sd_ras_n(sd_ras_n), .sd_cas_n(sd_cas_n), .sd_we_n(sd_we_n),
        .sd_addr(sd_addr), .sd_ba(sd_ba), .sd_dq(sd_dq),
        .st_armed(st_armed), .st_capturing(st_capturing),
        .st_uploading(st_uploading), .st_error(st_error), .st_rx_activity(st_rx));

    // SDRAM clock: 180 degrees shifted, like the PLL's CLKOUTP
`ifndef SD_DELAY
`define SD_DELAY 1.0
`endif
`ifndef SD_SKEW
`define SD_SKEW 0.0
`endif
    sdram_model #(.TAC(5.4), .TOH(2.5), .DELAY(`SD_DELAY), .SKEW_HI(`SD_SKEW)) sdram (
        .clk(~clk), .cke(1'b1), .cs_n(1'b0),
        .ras_n(sd_ras_n), .cas_n(sd_cas_n), .we_n(sd_we_n),
        .ba(sd_ba), .a(sd_addr), .dqm(4'b0000), .dq(sd_dq));

    // ------------------------------------------------------------------
    // UART host model
    // ------------------------------------------------------------------
    reg [7:0] rxbuf [0:1<<20];
    integer   rxcnt = 0;

    // receiver
    integer bi;
    reg [7:0] rb;
    always begin
        @(negedge uart_tx);
        #(BIT_NS * 1.5);
        for (bi = 0; bi < 8; bi = bi + 1) begin
            rb[bi] = uart_tx;
            #(BIT_NS);
        end
        if (uart_tx !== 1'b1) $display("%t UART framing error", $time);
        rxbuf[rxcnt] = rb;
        rxcnt = rxcnt + 1;
    end

    task send_byte(input [7:0] b);
        integer k;
        begin
            uart_rx = 0; #(BIT_NS);
            for (k = 0; k < 8; k = k + 1) begin uart_rx = b[k]; #(BIT_NS); end
            uart_rx = 1; #(BIT_NS);
        end
    endtask

    task cmd_short(input [7:0] op);
        send_byte(op);
    endtask

    task cmd_long(input [7:0] op, input [31:0] arg);   // little endian, as libsigrok
        begin
            send_byte(op);
            send_byte(arg[7:0]);  send_byte(arg[15:8]);
            send_byte(arg[23:16]); send_byte(arg[31:24]);
        end
    endtask

    task sump_reset;
        integer k;
        for (k = 0; k < 5; k = k + 1) cmd_short(8'h00);
    endtask

    // Wait until n bytes received after `start`, then a bit more to catch extras
    task wait_bytes(input integer start, input integer n, input integer timeout_us);
        integer t;
        begin
            t = 0;
            while (rxcnt - start < n && t < timeout_us * 10) begin #100; t = t + 1; end
            #(BIT_NS * 30);
        end
    endtask

    integer errors = 0;
    task fail(input [8*80-1:0] msg);
        begin
            errors = errors + 1;
            $display("FAIL: %0s", msg);
        end
    endtask

    // ------------------------------------------------------------------
    // Capture helper: configure like libsigrok and run.
    //   ngroups  1 or 2 (group mask: bit0 = channels 0-7, bit1 = 8-15)
    //   returns samples in chronological order in samp[]
    // ------------------------------------------------------------------
    reg [15:0] samp [0:1<<20];
    integer    nsamp;

    task capture(input [1:0] grpmask, input integer readcount, input integer delaycount,
                 input [23:0] divider, input test_mode, input use_long,
                 input integer timeout_us);
        integer start, nbytes, ng, s, b, idx;
        reg [31:0] flags;
        begin
            ng     = grpmask[0] + grpmask[1];
            flags  = (test_mode ? 32'h800 : 0) | 32'h2 /* noise filter */
                   | ((~grpmask & 2'b11) << 2) | 32'h30;
            cmd_long(8'h80, {8'h00, divider});
            if (use_long) begin
                cmd_long(8'h84, readcount - 1);
                cmd_long(8'h83, delaycount - 1);
            end else
                cmd_long(8'h81, {16'(delaycount - 1), 16'(readcount - 1)});
            cmd_long(8'h82, flags);
            start = rxcnt;
            cmd_short(8'h01);
            nbytes = readcount * 4 * ng;
            wait_bytes(start, nbytes, timeout_us);
            if (rxcnt - start != nbytes) begin
                $display("  got %0d bytes, expected %0d", rxcnt - start, nbytes);
                fail("capture byte count");
            end
            nsamp = readcount * 4;
            // device sends newest first; reverse into chronological order
            for (s = 0; s < nsamp; s = s + 1) begin
                idx = start + s * ng;
                samp[nsamp - 1 - s] = 0;
                b = 0;
                if (grpmask[0]) begin samp[nsamp-1-s][7:0]  = rxbuf[idx + b]; b = b + 1; end
                if (grpmask[1]) begin samp[nsamp-1-s][15:8] = rxbuf[idx + b]; b = b + 1; end
            end
        end
    endtask

    task set_stage(input integer st, input [15:0] mask, input [15:0] value,
                   input integer level, input start);
        begin
            cmd_long(8'hC0 + st * 4, mask);
            cmd_long(8'hC1 + st * 4, value);
            cmd_long(8'hC2 + st * 4, {4'b0, start, 3'b0, 6'b0, 2'(level), 16'h0});
        end
    endtask

    // Check that chronological samples count up by `step` (masked per group)
    task check_counting(input [15:0] gmask, input integer step, input [8*40-1:0] name);
        integer s, bad;
        begin
            bad = 0;
            for (s = 1; s < nsamp; s = s + 1)
                if ((^samp[s] === 1'bx) ||
                    ((samp[s] - samp[s-1]) & gmask) !== (step & gmask) &&
                    !(gmask == 16'hFF00 && (samp[s] & 16'hFF00) == (samp[s-1] & 16'hFF00))) begin
                    if (bad < 5)
                        $display("  %0s: sample %0d = %h, previous %h", name, s, samp[s], samp[s-1]);
                    bad = bad + 1;
                end
            if (bad) fail(name);
            else $display("ok:   %0s (%0d samples, last %h)", name, nsamp, samp[nsamp-1]);
        end
    endtask

    integer i, start, t_at;
    reg [8*32-1:0] str;

    initial begin
        $timeformat(-6, 2, " us", 10);
        repeat (20) @(posedge clk);
        rst = 0;

        // wait for SDRAM init + calibration
        wait (dut.mem_ready);
        $display("SDRAM ready at %t, cal_fail=%0d", $time, dut.mem_cal_fail);
        if (dut.mem_cal_fail) fail("SDRAM calibration");

        // ---------------- ID ----------------
        sump_reset;
        start = rxcnt;
        cmd_short(8'h02);
        wait_bytes(start, 4, 100);
        if (rxcnt - start == 4 && rxbuf[start] == "1" && rxbuf[start+1] == "A" &&
            rxbuf[start+2] == "L" && rxbuf[start+3] == "S")
            $display("ok:   ID 1ALS");
        else fail("ID");

        // ---------------- metadata ----------------
        start = rxcnt;
        cmd_short(8'h04);
        wait_bytes(start, 85, 200);
        if (rxcnt - start != 85) begin
            $display("  metadata: %0d bytes", rxcnt - start);
            fail("metadata length");
        end
        begin : parse_meta
            integer p, mem_bytes, nprobes, rate;
            reg [7:0] key;
            p = start;
            while (p < rxcnt && rxbuf[p] != 0) begin
                case (rxbuf[p] >> 5)
                0: begin
                    key = rxbuf[p];
                    str = "";
                    p = p + 1;
                    while (rxbuf[p] != 0) begin str = {str[8*31-1:0], rxbuf[p]}; p = p + 1; end
                    $display("      meta %02x: \"%0s\"", key, str);
                    p = p + 1;
                end
                1: begin
                    i = {rxbuf[p+1], rxbuf[p+2], rxbuf[p+3], rxbuf[p+4]};
                    $display("      meta %02x: %0d", rxbuf[p], i);
                    if (rxbuf[p] == 8'h20) nprobes = i;
                    if (rxbuf[p] == 8'h21) mem_bytes = i;
                    if (rxbuf[p] == 8'h23) rate = i;
                    p = p + 5;
                end
                default: begin fail("metadata type"); p = rxcnt; end
                endcase
            end
            if (nprobes != 16 || mem_bytes != 8388576 || rate != 100000000)
                fail("metadata values");
            else $display("ok:   metadata");
        end

        // ---------------- test pattern, no trigger, 0x81 ----------------
        set_stage(0, 0, 0, 0, 1);
        capture(2'b11, 8, 8, 0, 1, 0, 1000);
        check_counting(16'hFFFF, 1, "test pattern 16ch short cmd");
        if (samp[nsamp-1] !== 32) fail("newest sample value");

        // ---------------- probes, divider 3 -> step 4 ----------------
        capture(2'b11, 16, 16, 3, 0, 1, 1000);
        check_counting(16'hFFFF, 4, "probes divider 3");

        // ---------------- one group (channels 0-7) ----------------
        capture(2'b01, 10, 10, 0, 1, 1, 1000);   // 40 samples, partial word
        check_counting(16'h00FF, 1, "group 0 only");
        capture(2'b01, 11, 11, 1, 0, 1, 1000);
        check_counting(16'h00FF, 2, "group 0 only, probes divider 1");

        // ---------------- one group (channels 8-15) ----------------
        capture(2'b10, 150, 150, 0, 1, 1, 2000); // 600 samples
        check_counting(16'hFF00, 256, "group 1 only");
        if (samp[nsamp-1] !== 16'h0200) fail("group 1 newest value");

        // ---------------- single stage trigger on probes ----------------
        set_stage(0, 16'hFFFF, 16'h1234, 0, 1);
        capture(2'b11, 16, 8, 0, 0, 1, 3000);
        t_at = (16 - 8) * 4 - 1;
        check_counting(16'hFFFF, 1, "trigger probes");
        if (samp[t_at] !== 16'h1234) begin
            $display("  sample at trigger position %0d = %h", t_at, samp[t_at]);
            fail("trigger position");
        end else $display("ok:   trigger at expected position %0d", t_at);

        // ---------------- two stage trigger (test pattern 1000, 1001) ------
        set_stage(0, 16'hFFFF, 16'd1000, 0, 0);
        set_stage(1, 16'hFFFF, 16'd1001, 1, 1);
        capture(2'b11, 16, 8, 0, 1, 1, 3000);
        t_at = (16 - 8) * 4 - 2;
        if (samp[t_at] !== 16'd1001) begin
            $display("  sample at trigger position %0d = %0d", t_at, samp[t_at]);
            fail("2-stage trigger position");
        end else $display("ok:   2-stage trigger at expected position %0d", t_at);
        sump_reset;
        set_stage(1, 0, 0, 0, 0);

        // ---- libsigrok 0.5.x style: real stages without start bit,
        //      followed by an unconditional start stage ----
        sump_reset;
        set_stage(0, 16'hFFFF, 16'h4321, 0, 0);
        set_stage(1, 16'h0000, 16'h0000, 1, 1);
        capture(2'b11, 16, 8, 0, 0, 1, 3000);
        t_at = (16 - 8) * 4 - 1;
        if (samp[t_at] !== 16'h4321) begin
            $display("  sample at trigger position %0d = %h", t_at, samp[t_at]);
            fail("0.5.x style 1-stage trigger position");
        end else $display("ok:   0.5.x style 1-stage trigger at expected position %0d", t_at);
        sump_reset;
        set_stage(0, 16'hFFFF, 16'd2000, 0, 0);
        set_stage(1, 16'hFFFF, 16'd2001, 1, 0);
        set_stage(2, 16'h0000, 16'h0000, 2, 1);
        capture(2'b11, 16, 8, 0, 1, 1, 3000);
        t_at = (16 - 8) * 4 - 2;
        if (samp[t_at] !== 16'd2001) begin
            $display("  sample at trigger position %0d = %0d", t_at, samp[t_at]);
            fail("0.5.x style 2-stage trigger position");
        end else $display("ok:   0.5.x style 2-stage trigger at expected position %0d", t_at);
        sump_reset;
        set_stage(1, 0, 0, 0, 0);
        set_stage(2, 0, 0, 0, 0);

        // ---------------- large capture: rows, refresh, pre-trigger -------
        set_stage(0, 16'hFFFF, 16'd5000, 0, 1);
        capture(2'b11, 1024, 256, 0, 1, 1, 20000);
        t_at = (1024 - 256) * 4 - 1;
        check_counting(16'hFFFF, 1, "large capture 4096 samples");
        if (samp[t_at] !== 16'd5000) begin
            $display("  sample at trigger position %0d = %0d", t_at, samp[t_at]);
            fail("large capture trigger position");
        end

        // ---------------- abort a capture that never triggers ----------
        probes_force = 1;
        probes_val   = 16'h0000;
        set_stage(0, 16'hFFFF, 16'h5555, 0, 1);
        cmd_long(8'h82, 32'h32);
        cmd_short(8'h01);
        #20000;
        if (!st_armed) fail("not armed");
        sump_reset;
        #1000;
        if (st_armed) fail("reset did not abort");
        start = rxcnt;
        cmd_short(8'h02);
        wait_bytes(start, 4, 100);
        if (rxcnt - start == 4 && rxbuf[start] == "1") $display("ok:   abort + ID");
        else fail("ID after abort");
        probes_force = 0;

        // ---------------- finish now (0x05) forces the trigger -----------
        set_stage(0, 16'hFFFF, 16'h5555, 0, 1);
        probes_force = 1;
        cmd_long(8'h84, 3);
        cmd_long(8'h83, 1);
        cmd_long(8'h82, 32'h32);
        start = rxcnt;
        cmd_short(8'h01);
        #5000;
        cmd_short(8'h05);
        wait_bytes(start, 32, 1000);
        if (rxcnt - start == 32) $display("ok:   finish-now");
        else fail("finish-now");
        probes_force = 0;

        if (st_error) fail("error LED (FIFO overflow / calibration)");
        if (sdram.errors) fail("SDRAM protocol errors");
        $display("SDRAM model: %0d writes, %0d reads, %0d refreshes, %0d errors",
                 sdram.writes, sdram.reads, sdram.refreshes, sdram.errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        else $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
