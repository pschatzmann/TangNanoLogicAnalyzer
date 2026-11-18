// Streaming SDRAM controller for the GW2AR-18's embedded 64 Mbit SDR SDRAM
// (2M x 32: 4 banks x 2048 rows x 256 columns), clocked at ~100 MHz.
//
// * Write stream: words are taken from a first-word-fall-through FIFO and
//   written to consecutive addresses (wrapping at 2M words). A row stays open
//   and one WRITE (burst length 1) is issued per clock while data is
//   available, so the sustained rate is ~95M words/s.
// * Read port: single word reads (used for the slow upload to the host).
// * Auto refresh every ~7.8 us.
// * Read capture calibration, per data bit: the DQ pins sit on both sides
//   of the chip, so their delays differ. After initialisation, pairs of
//   complementary words are written and read back as 2-word bursts (mode
//   register: burst read, single write) while the bus was driven with the
//   complement just before - so a capture that is too early or too late sees
//   wrong data. Each bit is captured at four candidate points (rising/
//   falling edge, CL and CL+1 cycles); per bit the first candidate that read
//   every word correctly is used from then on.
// * DQ30/31 sit far from the other DQ pins (IOR44) and need a later
//   capture point than the rest on the Tang Nano 20K.
//
// Word address mapping: {row[10:0], bank[1:0], column[7:0]}.
module sdram_ctrl #(
    parameter integer CLK_HZ = 100_285_714,
    parameter integer CL     = 3,           // CAS latency (2 or 3)
    parameter [31:0]  DATA_MASK = 32'hFFFF_FFFF  // DQ bits carrying data (must calibrate)
) (
    input  wire        clk,
    input  wire        rst,

    // SDRAM pins (SDRAM clock = clk shifted by 180 degrees, generated outside)
    output reg         sd_ras_n,
    output reg         sd_cas_n,
    output reg         sd_we_n,
    output reg  [10:0] sd_addr,
    output reg  [1:0]  sd_ba,
    inout  wire [31:0] sd_dq,

    // status
    output reg         ready,       // initialised and calibrated
    output reg         cal_fail,
    output reg  [63:0] cal_sel,     // per bit 0..31: selected capture point
    output reg  [31:0] cal_bit_ok,  // per bit: a working capture point exists
    output wire        idle,        // no access in progress, all rows closed

    // write stream
    input  wire        wr_reset,    // restart writing at word 0
    input  wire [31:0] fifo_data,
    input  wire        fifo_empty,
    output wire        fifo_pop,

    // read port
    input  wire        rd_req,      // hold until rd_ack
    input  wire [20:0] rd_addr,
    output reg         rd_ack,      // one cycle, rd_data valid
    output reg  [31:0] rd_data
);
    // ------------------------------------------------------------------
    // Timing (cycles at ~10 ns, rounded up generously)
    // ------------------------------------------------------------------
    localparam integer T_RP   = 3;    // precharge -> activate/refresh
    localparam integer T_RCD  = 3;    // activate -> read/write
    localparam integer T_WR   = 3;    // last write data -> precharge
    localparam integer T_RFC  = 8;    // refresh -> anything
    localparam integer T_MRD  = 3;
    localparam integer T_INIT = CLK_HZ / 5000 + 100;   // 200 us
    localparam integer T_REFI = CLK_HZ / 1_000_000 * 78 / 10; // 7.8 us
    localparam integer WR_IDLE_CLOSE = 32;  // close a row after this many idle cycles
    localparam integer CAL_PAIRS = 16;     // word pairs P(i), ~P(i)

    // Commands {ras_n, cas_n, we_n}
    localparam [2:0] C_NOP = 3'b111, C_ACT = 3'b011, C_RD = 3'b101,
                     C_WR  = 3'b100, C_PRE = 3'b010, C_REF = 3'b001,
                     C_MRS = 3'b000;

    // ------------------------------------------------------------------
    // DQ: tristate output, and the four read capture candidates
    // ------------------------------------------------------------------
    reg  [31:0] dq_out;
    reg         dq_oe;
    assign sd_dq = dq_oe ? dq_out : 32'bz;

    reg  [31:0] dq_p, dq_n;           // sampled on rising / falling edge
    always @(posedge clk) dq_p <= sd_dq;
    always @(negedge clk) dq_n <= sd_dq;

    // ------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------
    localparam [3:0] S_INIT    = 4'd0,
                     S_IPRE    = 4'd1,
                     S_IREF    = 4'd2,
                     S_IMRS    = 4'd3,
                     S_WAIT    = 4'd4,
                     S_IDLE    = 4'd5,
                     S_WRITE   = 4'd6,
                     S_WCLOSE  = 4'd7,
                     S_RD      = 4'd8,
                     S_RDWAIT  = 4'd9,
                     S_CALWR   = 4'd10,
                     S_CALNEXT = 4'd11;
    reg [3:0]  state, ret_state;
    reg [15:0] wait_cnt;
    reg [3:0]  init_refs;
    reg        init_done;             // mode register set, refresh timer runs

    reg [20:0] waddr;                 // next word to write
    reg [12:0] open_rb;               // {row, bank} currently open for writing
    reg [5:0]  wr_idle;
    reg [3:0]  wr_tail;               // cycles since last write (for T_WR)

    reg [9:0]  refi_cnt;
    reg        ref_req;

    reg        rd_cal;                // current read is a calibration read
    reg [20:0] rd_a;
    reg [3:0]  rd_cnt;
    reg [31:0] cand [0:3];
    reg [31:0] cal_ok [0:3];          // per candidate: bits still passing
    reg [5:0]  cal_i;

    function [31:0] cal_pattern(input [3:0] i);
        cal_pattern = {4{i, ~i}} ^ (32'h0000_0001 << i) ^ (32'h8000_0000 >> (i + 4'd1));
    endfunction

    assign idle = state == S_IDLE;

    wire wr_row_change = waddr[20:8] != open_rb;
    wire wr_stop       = ref_req || wr_row_change ||
                         (fifo_empty && wr_idle == WR_IDLE_CLOSE);
    assign fifo_pop    = state == S_WRITE && !fifo_empty && !wr_stop;

    // Refresh interval timer
    always @(posedge clk) begin
        if (rst || !init_done) begin
            refi_cnt <= 0;
            ref_req  <= 1'b0;
        end else begin
            if (refi_cnt == T_REFI - 1) begin
                refi_cnt <= 0;
                ref_req  <= 1'b1;
            end else
                refi_cnt <= refi_cnt + 1'b1;
            if (state == S_IDLE && ref_req) ref_req <= 1'b0;  // being serviced
        end
    end

    integer k, b;
    reg  [31:0] cal_ok_next [0:3];
    wire [31:0] cal_exp = cal_pattern(cal_i[3:0]);
    always @(*)
        for (k = 0; k < 4; k = k + 1)
            for (b = 0; b < 32; b = b + 1)
                cal_ok_next[k][b] = cal_ok[k][b] && cand[k][b] === cal_exp[b];
    always @(posedge clk) begin
        {sd_ras_n, sd_cas_n, sd_we_n} <= C_NOP;
        dq_oe  <= 1'b0;
        rd_ack <= 1'b0;

        if (rst) begin
            state    <= S_INIT;
            wait_cnt <= T_INIT;
            ready    <= 1'b0;
            cal_fail <= 1'b0;
            cal_sel  <= 64'd0;
            cal_bit_ok <= 32'd0;
            init_done <= 1'b0;
            waddr    <= 21'd0;
        end else begin
            case (state)
            // ---------------- initialisation -------------------------
            S_INIT:
                if (wait_cnt == 0) state <= S_IPRE;
                else wait_cnt <= wait_cnt - 1'b1;
            S_IPRE: begin
                {sd_ras_n, sd_cas_n, sd_we_n} <= C_PRE;
                sd_addr[10] <= 1'b1;                     // all banks
                init_refs   <= 4'd8;
                wait_cnt    <= T_RP - 1;
                state       <= S_WAIT;
                ret_state   <= S_IREF;
            end
            S_IREF: begin
                {sd_ras_n, sd_cas_n, sd_we_n} <= C_REF;
                init_refs <= init_refs - 1'b1;
                wait_cnt  <= T_RFC - 1;
                state     <= S_WAIT;
                ret_state <= (init_refs == 1) ? S_IMRS : S_IREF;
            end
            S_IMRS: begin
                {sd_ras_n, sd_cas_n, sd_we_n} <= C_MRS;
                sd_ba   <= 2'b00;
                // burst read length 2, single location writes, CAS latency CL
                sd_addr <= {1'b0, 1'b1, 2'b00, CL[2:0], 1'b0, 3'b001};
                wait_cnt  <= T_MRD - 1;
                state     <= S_WAIT;
                ret_state <= S_CALWR;
                cal_i     <= 0;
                init_done <= 1'b1;
            end
            // ---------------- calibration ----------------------------
            // Write 2*CAL_PAIRS words at the start of bank 0 / row 0: word
            // 2i = P(i), word 2i+1 = ~P(i), ...
            S_CALWR: begin
                if (cal_i == 0) begin
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_ACT;
                    sd_ba   <= 2'd0;
                    sd_addr <= 11'd0;
                    cal_i   <= 1;
                    wait_cnt  <= T_RCD - 1;
                    state     <= S_WAIT;
                    ret_state <= S_CALWR;
                end else if (cal_i <= 2 * CAL_PAIRS) begin
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_WR;
                    sd_addr <= {6'd0, cal_i[4:0] - 5'd1};
                    dq_out  <= (cal_i[0] ? 32'd0 : 32'hFFFF_FFFF)   // odd word index: ~P
                               ^ cal_pattern(cal_i[4:1] - {3'd0, ~cal_i[0]});
                    dq_oe   <= 1'b1;
                    cal_i   <= cal_i + 1'b1;
                end else begin
                    // ... close the row, then read them back
                    wait_cnt  <= T_WR - 1;
                    state     <= S_WAIT;
                    ret_state <= S_WCLOSE;
                    cal_i     <= 0;
                    for (k = 0; k < 4; k = k + 1) cal_ok[k] <= 32'hFFFF_FFFF;
                end
            end
            S_CALNEXT: begin
                for (k = 0; k < 4; k = k + 1) cal_ok[k] <= cal_ok_next[k];
                cal_i  <= cal_i + 1'b1;
                state  <= S_IDLE;
                if (cal_i == CAL_PAIRS - 1) begin
                    ready <= 1'b1;
                    for (b = 0; b < 32; b = b + 1) begin
                        cal_sel[2*b +: 2] <= cal_ok_next[0][b] ? 2'd0 :
                                             cal_ok_next[1][b] ? 2'd1 :
                                             cal_ok_next[2][b] ? 2'd2 : 2'd3;
                        cal_bit_ok[b] <= cal_ok_next[0][b] | cal_ok_next[1][b] |
                                         cal_ok_next[2][b] | cal_ok_next[3][b];
                    end
                    cal_fail <= ((cal_ok_next[0] | cal_ok_next[1] |
                                  cal_ok_next[2] | cal_ok_next[3]) & DATA_MASK) != DATA_MASK;
                end
            end
            // ---------------- normal operation -----------------------
            S_IDLE: begin
                if (ref_req) begin
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_REF;
                    wait_cnt  <= T_RFC - 1;
                    state     <= S_WAIT;
                    ret_state <= S_IDLE;
                end else if (!ready) begin
                    // calibration read of words 2*cal_i, 2*cal_i+1
                    rd_cal <= 1'b1;
                    rd_a   <= {16'd0, cal_i[3:0], 1'b0};
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_ACT;
                    sd_ba   <= 2'd0;
                    sd_addr <= 11'd0;
                    wait_cnt  <= T_RCD - 1;
                    state     <= S_WAIT;
                    ret_state <= S_RD;
                end else if (!fifo_empty) begin
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_ACT;
                    sd_ba   <= waddr[9:8];
                    sd_addr <= waddr[20:10];
                    open_rb <= waddr[20:8];
                    wr_idle <= 0;
                    wait_cnt  <= T_RCD - 1;
                    state     <= S_WAIT;
                    ret_state <= S_WRITE;
                end else if (rd_req && !rd_ack) begin
                    rd_cal <= 1'b0;
                    rd_a   <= rd_addr;
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_ACT;
                    sd_ba   <= rd_addr[9:8];
                    sd_addr <= rd_addr[20:10];
                    wait_cnt  <= T_RCD - 1;
                    state     <= S_WAIT;
                    ret_state <= S_RD;
                end
            end
            S_WRITE: begin
                if (fifo_pop) begin
                    {sd_ras_n, sd_cas_n, sd_we_n} <= C_WR;
                    sd_ba   <= waddr[9:8];
                    sd_addr <= {3'b000, waddr[7:0]};     // A10=0: no auto precharge
                    dq_out  <= fifo_data;
                    dq_oe   <= 1'b1;
                    waddr   <= waddr + 1'b1;
                    wr_idle <= 0;
                end else if (wr_stop) begin
                    wait_cnt  <= T_WR - 1;
                    state     <= S_WAIT;
                    ret_state <= S_WCLOSE;
                end else
                    wr_idle <= wr_idle + 1'b1;
            end
            S_WCLOSE: begin
                {sd_ras_n, sd_cas_n, sd_we_n} <= C_PRE;
                sd_addr[10] <= 1'b1;                     // all banks
                wait_cnt  <= T_RP - 1;
                state     <= S_WAIT;
                ret_state <= S_IDLE;
            end
            S_RD: begin
                {sd_ras_n, sd_cas_n, sd_we_n} <= C_RD;
                sd_ba   <= rd_a[9:8];
                sd_addr <= {1'b1, 2'b00, rd_a[7:0]};     // A10=1: auto precharge
                rd_cnt  <= 4'd1;
                state   <= S_RDWAIT;
                if (rd_cal) begin
                    // pre-charge the bus with the complement of the expected
                    // data, so a capture before the data is valid fails
                    dq_out <= ~cal_exp;
                    dq_oe  <= 1'b1;
                end
            end
            S_RDWAIT: begin
                // rd_cnt = rising edges since the READ command was issued
                rd_cnt <= rd_cnt + 1'b1;
                if (rd_cnt == CL + 1) begin
                    cand[0] <= dq_p;      // rising edge CL
                    cand[1] <= dq_n;      // falling edge CL + 1/2
                end
                if (rd_cnt == CL + 2) begin
                    cand[2] <= dq_p;      // rising edge CL + 1
                    cand[3] <= dq_n;      // falling edge CL + 1 1/2
                end
                if (rd_cnt == CL + 3) begin
                    rd_data <= 32'd0;
                    for (b = 0; b < 32; b = b + 1)
                        rd_data[b] <= cand[cal_sel[2*b +: 2]][b];
                end
                // auto precharge done and T_RC satisfied well before this
                if (rd_cnt == CL + 3 + T_RP) begin
                    if (rd_cal) state <= S_CALNEXT;
                    else begin
                        rd_ack <= 1'b1;
                        state  <= S_IDLE;
                    end
                end
            end
            S_WAIT:
                if (wait_cnt == 0) state <= ret_state;
                else wait_cnt <= wait_cnt - 1'b1;
            default: state <= S_IDLE;
            endcase
            if (wr_reset) waddr <= 21'd0;

            // Keep driving DQ for the whole time a row is open for writing
            // (from the ACTIVATE until the PRECHARGE), not only in WRITE
            // cycles: on the real chip the output enable of some DQ pins
            // (DQ16/17) turns on too late for a WRITE that follows an idle
            // cycle, and the SDRAM stored the old bus value. The SDRAM
            // never drives DQ in this phase.
            if (state == S_WRITE ||
                (state == S_WAIT && (ret_state == S_WRITE || ret_state == S_WCLOSE ||
                                     ret_state == S_CALWR)) ||
                (state == S_CALWR && cal_i != 0 && cal_i <= 2 * CAL_PAIRS))
                dq_oe <= 1'b1;
        end
    end
endmodule
