// Generated synchronous 2048x16 glyph ROM; same one-cycle latency.
// M14: the read register holds the raw memory word (as info_font does), and
// the "glyph >= 128 is blank" select is applied after it. With the select in
// front of the register TD could not use the block-RAM output register and
// built the ROM from ~490 LUTs; the output is unchanged cycle for cycle.
module glyph_rom(input wire clk, input wire [7:0] glyph,
    input wire [3:0] row, output wire [15:0] bits);
    reg [15:0] memory [0:2047];
    initial $readmemh("heritage_glyph.hex", memory);
    reg [15:0] raw;
    reg blank;
    always @(posedge clk) begin
        raw <= memory[{glyph[6:0], row}];
        blank <= glyph[7];
    end
    assign bits = blank ? 16'd0 : raw;
endmodule
