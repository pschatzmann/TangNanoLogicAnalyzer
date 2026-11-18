// SUMP / Openbench Logic Sniffer compatible capture core (16 channels) with
// the sample buffer in SDRAM (2M words).
//
// Works with libsigrok's "ols" driver (PulseView, sigrok-cli).
//
// Memory use: samples form a byte stream (1 byte per sample with one channel
// group enabled, 2 bytes with both, low byte first) packed 4 bytes per
// 32-bit SDRAM word (2M words).
//   * both channel groups enabled           -> 4M samples
//   * only one channel group (0-7 or 8-15)  -> 8M samples
//   The metadata reports ~8 MB, and libsigrok divides that by the number of
//   enabled groups, so it never asks for more than fits. The words are
//   streamed through a FIFO into a ring buffer in SDRAM.
//
// Supported commands
//   0x00 reset, 0x01 arm, 0x02 ID ("1ALS"), 0x04 metadata, 0x05 finish now
//   (forces the trigger), 0x11/0x13 XON/XOFF (ignored),
//   0x80 divider, 0x81 read/delay count (16 bit each), 0x82 flags,
//   0x83 delay count (32 bit), 0x84 read count (32 bit),
//   0xC0-0xCF trigger mask/value/config for stages 0-3 (parallel mode).
// Flags used: bit 2/3 disable channel group 0/1, bit 11 internal test mode
// (a counter that increments every sample). Demux, noise filter, external
// clock, RLE and serial triggers are not implemented (flags ignored).
//
// The sample clock is clk / (divider + 1); clk is (nominally) 100 MHz.
// Samples are returned newest first, as the original SUMP design does.
module sump_core #(
    parameter [8*16-1:0] DEVICE_NAME = "Tang Nano 20K LA",
    parameter [8*4-1:0]  FW_VERSION  = "2.40",
    parameter integer    MAX_RATE_HZ = 100_000_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] probes,      // synchronised inputs

    // host link
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready,

    // SDRAM controller
    input  wire        mem_ready,
    input  wire        mem_cal_fail,
    input  wire [63:0] mem_cal_sel,     // per DQ bit: capture point 0..3
    input  wire [31:0] mem_cal_bitok,
    input  wire        mem_idle,
    output reg         mem_wr_reset,
    output wire [31:0] mem_fifo_data,
    output wire        mem_fifo_empty,
    input  wire        mem_fifo_pop,
    output reg         mem_rd_req,
    output reg  [20:0] mem_rd_addr,
    input  wire        mem_rd_ack,
    input  wire [31:0] mem_rd_data,

    output wire        st_armed,     // waiting for trigger
    output wire        st_capturing, // triggered, filling post-trigger buffer
    output wire        st_uploading, // sending samples to the host
    output wire        st_overflow   // FIFO overflow in the last capture
);
    // ------------------------------------------------------------------
    // Command parser: short commands are one byte (bit 7 clear), long
    // commands one byte + 4 argument bytes, least significant byte first.
    // ------------------------------------------------------------------
    reg  [2:0]  cmd_cnt;
    reg  [7:0]  cmd_op;
    reg  [31:0] cmd_arg;
    reg         short_stb, long_stb;

    always @(posedge clk) begin
        short_stb <= 1'b0;
        long_stb  <= 1'b0;
        if (rst) begin
            cmd_cnt <= 3'd0;
        end else if (rx_valid) begin
            if (cmd_cnt == 3'd0) begin
                cmd_op <= rx_data;
                if (rx_data[7]) cmd_cnt <= 3'd1;
                else            short_stb <= 1'b1;
            end else begin
                cmd_arg <= {rx_data, cmd_arg[31:8]};
                if (cmd_cnt == 3'd4) begin
                    cmd_cnt  <= 3'd0;
                    long_stb <= 1'b1;
                end else
                    cmd_cnt <= cmd_cnt + 1'b1;
            end
        end
    end

    wire cmd_reset  = short_stb && cmd_op == 8'h00;
    wire cmd_arm    = short_stb && cmd_op == 8'h01;
    wire cmd_id     = short_stb && cmd_op == 8'h02;
    wire cmd_meta   = short_stb && cmd_op == 8'h04;
    wire cmd_finish = short_stb && cmd_op == 8'h05;

    // ------------------------------------------------------------------
    // Configuration registers
    // ------------------------------------------------------------------
    localparam integer CW = 21;        // count fields: up to 2M (x4 = 8M samples)
    reg [23:0]   divider;
    reg [CW-1:0] read_field;           // samples to read       = (read_field  + 1) * 4
    reg [CW-1:0] delay_field;          // samples after trigger = (delay_field + 1) * 4
    reg [15:0]   flags;
    reg [15:0]   trg_mask  [0:3];
    reg [15:0]   trg_value [0:3];
    reg [1:0]    trg_level [0:3];
    reg          trg_start [0:3];

    function [CW-1:0] clamp_cnt(input [31:0] v);
        clamp_cnt = (|v[31:CW]) ? {CW{1'b1}} : v[CW-1:0];
    endfunction

    integer i, j, k;
    always @(posedge clk) begin
        if (rst) begin
            divider     <= 24'd0;
            read_field  <= 0;
            delay_field <= 0;
            flags       <= 16'd0;
            for (i = 0; i < 4; i = i + 1) begin
                trg_mask[i]  <= 16'd0;
                trg_value[i] <= 16'd0;
                trg_level[i] <= 2'd0;
                trg_start[i] <= 1'b0;
            end
        end else if (long_stb) begin
            casez (cmd_op)
            8'h80: divider     <= cmd_arg[23:0];
            8'h81: begin
                   read_field  <= {{(CW-16){1'b0}}, cmd_arg[15:0]};
                   delay_field <= {{(CW-16){1'b0}}, cmd_arg[31:16]};
                   end
            8'h82: flags       <= cmd_arg[15:0];
            8'h83: delay_field <= clamp_cnt(cmd_arg);
            8'h84: read_field  <= clamp_cnt(cmd_arg);
            8'b1100_??00: trg_mask [cmd_op[3:2]] <= cmd_arg[15:0];
            8'b1100_??01: trg_value[cmd_op[3:2]] <= cmd_arg[15:0];
            8'b1100_??10: begin
                trg_level[cmd_op[3:2]] <= cmd_arg[17:16];
                trg_start[cmd_op[3:2]] <= cmd_arg[27];
            end
            default: ;
            endcase
        end
    end

    // Channel group handling
    wire grp0_en   = !flags[2];
    wire grp1_en   = !flags[3];
    wire one_grp   = grp0_en ^ grp1_en;    // 8-bit samples, 4 per word
    wire use_hi    = grp1_en && !grp0_en;  // the single group is channels 8-15
    wire test_mode = flags[11];

    // A stage takes part in triggering if it was configured with a mask or
    // as a start stage (unconfigured stages would otherwise always match).
    wire [3:0] stage_used;
    genvar g;
    generate for (g = 0; g < 4; g = g + 1) begin : g_used
        assign stage_used[g] = (|trg_mask[g]) || trg_start[g];
    end endgenerate

    // ------------------------------------------------------------------
    // FIFO towards the SDRAM
    // ------------------------------------------------------------------
    reg         fifo_clr;
    reg         fifo_push;
    reg  [31:0] fifo_din;

    sync_fifo #(.DW(32), .AW(6)) u_fifo (
        .clk(clk), .clr(fifo_clr | rst),
        .din(fifo_din), .push(fifo_push),
        .dout(mem_fifo_data), .pop(mem_fifo_pop), .empty(mem_fifo_empty),
        .overflow(st_overflow));

    // ------------------------------------------------------------------
    // Main state machine
    // ------------------------------------------------------------------
    localparam [2:0] S_IDLE   = 3'd0,
                     S_ARMED  = 3'd1,   // sampling, waiting for trigger
                     S_POST   = 3'd2,   // sampling post-trigger part
                     S_FLUSH  = 3'd3,   // waiting for the FIFO to drain
                     S_READ   = 3'd4,   // uploading samples
                     S_ROM    = 3'd5,   // sending ID / metadata
                     S_FLUSH0 = 3'd6;   // writing a partially filled word
    reg [2:0] state;

    assign st_armed     = state == S_ARMED;
    assign st_capturing = state == S_POST || state == S_FLUSH0 || state == S_FLUSH;
    assign st_uploading = state == S_READ;

    wire sampling = state == S_ARMED || state == S_POST;

    // Sample clock divider
    reg  [23:0] div_cnt;
    reg         tick;
    always @(posedge clk) begin
        tick <= 1'b0;
        if (!sampling) begin
            div_cnt <= 24'd0;
        end else if (div_cnt == 24'd0) begin
            div_cnt <= divider;
            tick    <= 1'b1;
        end else
            div_cnt <= div_cnt - 1'b1;
    end

    // Sample pipeline: s1 = sample, s2 = sample + per-stage match flags
    reg  [15:0] test_cnt;
    reg  [15:0] s1, s2;
    reg         v1, v2;
    reg  [3:0]  m2;
    always @(posedge clk) begin
        v1 <= tick && sampling;
        v2 <= v1 && sampling;
        if (!sampling) test_cnt <= 16'd0;
        else if (tick) test_cnt <= test_cnt + 1'b1;
        if (tick) s1 <= test_mode ? test_cnt : probes;
        s2 <= s1;
        for (j = 0; j < 4; j = j + 1)
            m2[j] <= ((s1 ^ trg_value[j]) & trg_mask[j]) == 16'd0;
    end

    // Counts (in samples)
    wire [24:0] read_samples  = ({4'b0, read_field}  + 25'd1) << 2;  // R
    wire [24:0] delay_samples = ({4'b0, delay_field} + 25'd1) << 2;  // post

    reg  [24:0] pre_needed;   // samples to collect before the trigger may fire
    reg  [24:0] wr_count;     // samples written since arm (saturating)
    reg         pre_ok;
    reg  [1:0]  level;
    reg  [24:0] post_rem;
    reg         force_trg;

    // Byte packer: 4 bytes per word. wword = word being filled (also the
    // SDRAM word address, as the controller writes words in order from 0),
    // wslot = next free byte in it (always even with 2-byte samples).
    reg  [31:0] pack;
    reg  [20:0] wword;
    reg  [1:0]  wslot;
    reg  [31:0] pk_word;      // word including this sample
    wire        pk_full = one_grp ? wslot == 2'd3 : wslot[1];
    wire [1:0]  wslot_n = one_grp ? wslot + 2'd1 : wslot + 2'd2;
    wire [7:0]  sb0 = (one_grp && use_hi) ? s2[15:8] : s2[7:0];
    wire [7:0]  sb1 = s2[15:8];
    always @(*) begin
        pk_word = pack;
        if (one_grp) pk_word[8*wslot +: 8]   = sb0;
        else         pk_word[8*wslot +: 16]  = {sb1, sb0};
    end
    wire [31:0] pack_n = pk_full ? 32'd0 : pk_word;

    // Levels that hold an unconditional start stage (mask 0). libsigrok
    // 0.5.x programs its trigger stages without the start bit and appends
    // such a stage; reaching it fires the trigger. Firing directly on the
    // sample that advances into it keeps the trigger on the matching sample.
    reg [3:0] uncond_start;
    always @(*) begin
        uncond_start = 4'b0000;
        for (k = 0; k < 4; k = k + 1)
            if (trg_start[k] && trg_mask[k] == 16'd0)
                uncond_start[trg_level[k]] = 1'b1;
    end

    // Trigger evaluation on the sample in s2 (registered match flags)
    reg       fire;
    reg       advance;
    always @(*) begin
        fire    = 1'b0;
        advance = 1'b0;
        for (k = 0; k < 4; k = k + 1) begin
            if (stage_used[k] && trg_level[k] == level && m2[k]) begin
                if (trg_start[k] || (level != 2'd3 && uncond_start[level + 2'd1]))
                    fire    = 1'b1;
                else
                    advance = 1'b1;
            end
        end
    end

    // Readout: walks the byte stream backwards from the newest byte, with a
    // one-word cache.
    reg  [24:0] rd_rem;       // samples left to send
    reg  [20:0] rp_word;      // position of the next byte to read
    reg  [1:0]  rp_slot;
    reg  [2:0]  rd_step;
    reg  [31:0] cw_data;      // cached word
    reg  [20:0] cw_addr;
    reg         cw_valid;
    reg         have_hi;
    reg         partial;      // a partially filled word is still to be written
    reg  [7:0]  rd_lo, rd_hi;
    wire [7:0]  rd_byte = cw_data[8*rp_slot +: 8];

    // ------------------------------------------------------------------
    // ID / metadata ROM
    // ------------------------------------------------------------------
    reg  [6:0]  rom_ptr, rom_end;
    reg  [7:0]  rom_byte;

    localparam [31:0] META_PROBES = 32'd16;
    // bytes: 2M words x 4, minus 8 words. A capture that filled the whole
    // ring would overwrite its oldest bytes with the final, partially
    // filled word; with this margin libsigrok never asks for that much.
    localparam [31:0] META_MEMORY = 32'd8388576;
    localparam [31:0] META_RATE   = MAX_RATE_HZ;
    localparam [6:0]  ROM_ID      = 7'd0;           // "1ALS"
    localparam [6:0]  ROM_META    = 7'd4;
    localparam [6:0]  ROM_END     = 7'd88;          // terminating 0x00

    // SDRAM read calibration result, shown as ancillary version
    // "sdram:<32 chars>" - the capture point used for DQ bits 31..0
    function [7:0] cal_char(input integer bit_n);
        cal_char = !mem_ready            ? "?" :
                   !mem_cal_bitok[bit_n] ? "-" :
                   8'h30 + {6'd0, mem_cal_sel[2*bit_n +: 2]};
    endfunction

    always @(*) begin
        case (rom_ptr)
        // ID
        7'd0:  rom_byte = "1";
        7'd1:  rom_byte = "A";
        7'd2:  rom_byte = "L";
        7'd3:  rom_byte = "S";
        // 0x01 device name
        7'd4:  rom_byte = 8'h01;
        7'd5:  rom_byte = DEVICE_NAME[127:120];
        7'd6:  rom_byte = DEVICE_NAME[119:112];
        7'd7:  rom_byte = DEVICE_NAME[111:104];
        7'd8:  rom_byte = DEVICE_NAME[103:96];
        7'd9:  rom_byte = DEVICE_NAME[95:88];
        7'd10: rom_byte = DEVICE_NAME[87:80];
        7'd11: rom_byte = DEVICE_NAME[79:72];
        7'd12: rom_byte = DEVICE_NAME[71:64];
        7'd13: rom_byte = DEVICE_NAME[63:56];
        7'd14: rom_byte = DEVICE_NAME[55:48];
        7'd15: rom_byte = DEVICE_NAME[47:40];
        7'd16: rom_byte = DEVICE_NAME[39:32];
        7'd17: rom_byte = DEVICE_NAME[31:24];
        7'd18: rom_byte = DEVICE_NAME[23:16];
        7'd19: rom_byte = DEVICE_NAME[15:8];
        7'd20: rom_byte = DEVICE_NAME[7:0];
        7'd21: rom_byte = 8'h00;
        // 0x02 firmware version
        7'd22: rom_byte = 8'h02;
        7'd23: rom_byte = FW_VERSION[31:24];
        7'd24: rom_byte = FW_VERSION[23:16];
        7'd25: rom_byte = FW_VERSION[15:8];
        7'd26: rom_byte = FW_VERSION[7:0];
        7'd27: rom_byte = 8'h00;
        // 0x03 ancillary version: SDRAM calibration, capture point per
        // DQ bit 31..0 ('-' = none works, '?' = not calibrated yet)
        7'd28: rom_byte = 8'h03;
        7'd29: rom_byte = "s";
        7'd30: rom_byte = "d";
        7'd31: rom_byte = "r";
        7'd32: rom_byte = "a";
        7'd33: rom_byte = "m";
        7'd34: rom_byte = ":";
        7'd35: rom_byte = cal_char(31);
        7'd36: rom_byte = cal_char(30);
        7'd37: rom_byte = cal_char(29);
        7'd38: rom_byte = cal_char(28);
        7'd39: rom_byte = cal_char(27);
        7'd40: rom_byte = cal_char(26);
        7'd41: rom_byte = cal_char(25);
        7'd42: rom_byte = cal_char(24);
        7'd43: rom_byte = cal_char(23);
        7'd44: rom_byte = cal_char(22);
        7'd45: rom_byte = cal_char(21);
        7'd46: rom_byte = cal_char(20);
        7'd47: rom_byte = cal_char(19);
        7'd48: rom_byte = cal_char(18);
        7'd49: rom_byte = cal_char(17);
        7'd50: rom_byte = cal_char(16);
        7'd51: rom_byte = cal_char(15);
        7'd52: rom_byte = cal_char(14);
        7'd53: rom_byte = cal_char(13);
        7'd54: rom_byte = cal_char(12);
        7'd55: rom_byte = cal_char(11);
        7'd56: rom_byte = cal_char(10);
        7'd57: rom_byte = cal_char(9);
        7'd58: rom_byte = cal_char(8);
        7'd59: rom_byte = cal_char(7);
        7'd60: rom_byte = cal_char(6);
        7'd61: rom_byte = cal_char(5);
        7'd62: rom_byte = cal_char(4);
        7'd63: rom_byte = cal_char(3);
        7'd64: rom_byte = cal_char(2);
        7'd65: rom_byte = cal_char(1);
        7'd66: rom_byte = cal_char(0);
        7'd67: rom_byte = 8'h00;
        // 0x20 number of probes (32 bit, big endian)
        7'd68: rom_byte = 8'h20;
        7'd69: rom_byte = META_PROBES[31:24];
        7'd70: rom_byte = META_PROBES[23:16];
        7'd71: rom_byte = META_PROBES[15:8];
        7'd72: rom_byte = META_PROBES[7:0];
        // 0x21 sample memory in bytes
        7'd73: rom_byte = 8'h21;
        7'd74: rom_byte = META_MEMORY[31:24];
        7'd75: rom_byte = META_MEMORY[23:16];
        7'd76: rom_byte = META_MEMORY[15:8];
        7'd77: rom_byte = META_MEMORY[7:0];
        // 0x23 max sample rate
        7'd78: rom_byte = 8'h23;
        7'd79: rom_byte = META_RATE[31:24];
        7'd80: rom_byte = META_RATE[23:16];
        7'd81: rom_byte = META_RATE[15:8];
        7'd82: rom_byte = META_RATE[7:0];
        // 0x24 protocol version 2
        7'd83: rom_byte = 8'h24;
        7'd84: rom_byte = 8'h00;
        7'd85: rom_byte = 8'h00;
        7'd86: rom_byte = 8'h00;
        7'd87: rom_byte = 8'h02;
        // 7'd88: end of metadata
        default: rom_byte = 8'h00;
        endcase
    end

    always @(posedge clk) begin
        fifo_push    <= 1'b0;
        fifo_clr     <= 1'b0;
        mem_wr_reset <= 1'b0;

        if (rst) begin
            state      <= S_IDLE;
            tx_valid   <= 1'b0;
            force_trg  <= 1'b0;
            mem_rd_req <= 1'b0;
        end else if (cmd_reset) begin
            // Abort whatever is going on (a byte already handed to the UART
            // still completes; data already in the FIFO is still written).
            state      <= S_IDLE;
            tx_valid   <= 1'b0;
            mem_rd_req <= 1'b0;
        end else begin
            case (state)
            // ----------------------------------------------------------
            S_IDLE: begin
                if (cmd_arm && mem_ready) begin
                    state        <= S_ARMED;
                    level        <= 2'd0;
                    wword        <= 21'd0;
                    wslot        <= 2'd0;
                    wr_count     <= 25'd0;
                    pre_ok       <= 1'b0;
                    force_trg    <= 1'b0;
                    pack         <= 24'd0;
                    fifo_clr     <= 1'b1;
                    mem_wr_reset <= 1'b1;
                    pre_needed   <= (read_samples > delay_samples)
                                    ? read_samples - delay_samples : 25'd0;
                end else if (cmd_id) begin
                    state   <= S_ROM;
                    rom_ptr <= ROM_ID;
                    rom_end <= ROM_META - 1'b1;
                end else if (cmd_meta) begin
                    state   <= S_ROM;
                    rom_ptr <= ROM_META;
                    rom_end <= ROM_END;
                end
            end
            // ----------------------------------------------------------
            S_ROM: begin
                if (!tx_valid) begin
                    tx_data  <= rom_byte;
                    tx_valid <= 1'b1;
                end else if (tx_ready) begin
                    tx_valid <= 1'b0;
                    if (rom_ptr == rom_end) state <= S_IDLE;
                    rom_ptr <= rom_ptr + 1'b1;
                end
            end
            // ----------------------------------------------------------
            S_ARMED, S_POST: begin
                if (cmd_finish) force_trg <= 1'b1;
                pre_ok <= wr_count >= pre_needed;

                if (v2) begin
                    // Store the sample
                    pack  <= pack_n;
                    wslot <= wslot_n;
                    if (pk_full) begin
                        fifo_din  <= pk_word;
                        fifo_push <= 1'b1;
                        wword     <= wword + 1'b1;
                    end
                    if (wr_count != {25{1'b1}}) wr_count <= wr_count + 1'b1;

                    if (state == S_ARMED) begin
                        if (pre_ok && (fire || force_trg)) begin
                            // libsigrok places the trigger marker
                            // <number of stages> samples before the
                            // post-trigger part (OLS pipeline delay), so
                            // extend the post-trigger part by level
                            // (= stages - 1) to put the marker on the
                            // matching sample.
                            state    <= S_POST;
                            post_rem <= delay_samples + (force_trg ? 25'd0 : {23'd0, level});
                        end else if (pre_ok && advance) begin
                            level <= level + 1'b1;
                        end
                    end else begin
                        post_rem <= post_rem - 1'b1;
                        if (post_rem == 25'd1) begin
                            // last sample; a partially filled word is
                            // written in S_FLUSH0
                            state    <= S_FLUSH0;
                            partial  <= wslot_n != 2'd0;
                            rd_rem   <= read_samples;
                            rd_step  <= 3'd0;
                            cw_valid <= 1'b0;
                            have_hi  <= 1'b0;
                        end
                    end
                end
            end
            // ----------------------------------------------------------
            S_FLUSH0: begin
                if (partial) begin
                    fifo_din  <= pack;
                    fifo_push <= 1'b1;
                end
                // newest byte = the one before the next free position
                if (wslot == 2'd0) begin
                    rp_word <= wword - 1'b1;
                    rp_slot <= 2'd3;
                end else begin
                    rp_word <= wword;
                    rp_slot <= wslot - 1'b1;
                end
                state <= S_FLUSH;
            end
            // ----------------------------------------------------------
            S_FLUSH: begin
                // fifo_push of the last word happened in the previous
                // cycle, so the FIFO is not empty before it was written.
                if (mem_fifo_empty && mem_idle && !fifo_push)
                    state <= S_READ;
            end
            // ----------------------------------------------------------
            S_READ: begin
                case (rd_step)
                3'd0: begin    // make sure the word holding the byte is cached
                    if (cw_valid && cw_addr == rp_word)
                        rd_step <= 3'd2;
                    else begin
                        mem_rd_req  <= 1'b1;
                        mem_rd_addr <= rp_word;
                        rd_step     <= 3'd1;
                    end
                end
                3'd1: begin
                    if (mem_rd_ack) begin
                        mem_rd_req <= 1'b0;
                        cw_data    <= mem_rd_data;
                        cw_addr    <= rp_word;
                        cw_valid   <= 1'b1;
                        rd_step    <= 3'd2;
                    end
                end
                3'd2: begin    // take the byte, step back one byte
                    if (rp_slot == 2'd0) begin
                        rp_slot <= 2'd3;
                        rp_word <= rp_word - 1'b1;
                    end else
                        rp_slot <= rp_slot - 1'b1;
                    if (one_grp) begin
                        rd_lo   <= rd_byte;
                        rd_step <= 3'd3;
                    end else if (!have_hi) begin
                        rd_hi   <= rd_byte;     // stored after the low byte
                        have_hi <= 1'b1;
                        rd_step <= 3'd0;
                    end else begin
                        rd_lo   <= rd_byte;
                        have_hi <= 1'b0;
                        rd_step <= 3'd3;
                    end
                end
                3'd3: begin    // first byte (channel group 0, or the only group)
                    if (!tx_valid) begin
                        tx_data  <= rd_lo;
                        tx_valid <= 1'b1;
                    end else if (tx_ready) begin
                        tx_valid <= 1'b0;
                        rd_step  <= one_grp ? 3'd5 : 3'd4;
                    end
                end
                3'd4: begin    // second byte (channel group 1)
                    if (!tx_valid) begin
                        tx_data  <= rd_hi;
                        tx_valid <= 1'b1;
                    end else if (tx_ready) begin
                        tx_valid <= 1'b0;
                        rd_step  <= 3'd5;
                    end
                end
                default: begin // next sample (going backwards in time)
                    rd_rem  <= rd_rem - 1'b1;
                    rd_step <= 3'd0;
                    if (rd_rem == 25'd1) state <= S_IDLE;
                end
                endcase
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
