// Small synchronous FIFO, first-word-fall-through (asynchronous read, so it
// maps to LUT RAM). Absorbs SDRAM row changes and refreshes.
module sync_fifo #(
    parameter integer DW = 32,
    parameter integer AW = 6
) (
    input  wire          clk,
    input  wire          clr,
    input  wire [DW-1:0] din,
    input  wire          push,
    output wire [DW-1:0] dout,
    input  wire          pop,
    output wire          empty,
    output reg           overflow   // sticky until clr
);
    // Must not become block RAM: yosys maps this async-read memory to a
    // Gowin SDP block (adding a read register), which on the real chip
    // returned stale words when a word was popped right after its push.
    (* ram_style = "distributed" *)
    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW:0]   wp, rp;

    wire full = (wp - rp) == (1 << AW);
    assign empty = wp == rp;
    assign dout  = mem[rp[AW-1:0]];

    always @(posedge clk) begin
        if (push && !full) mem[wp[AW-1:0]] <= din;
        if (clr) begin
            wp <= 0;
            rp <= 0;
            overflow <= 1'b0;
        end else begin
            if (push) begin
                if (!full) wp <= wp + 1'b1;
                else       overflow <= 1'b1;
            end
            if (pop && !empty) rp <= rp + 1'b1;
        end
    end
endmodule
