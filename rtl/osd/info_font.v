// One-bit synchronous 8x8 ASCII glyph ROM, one block RAM instead of LUT multiplexers.
// Address: ASCII[6:0], glyph row[2:0], glyph column[2:0]. Padded cells are zero.
module info_font(input wire clk,input wire [6:0] ascii,
    input wire [2:0] row,col,output reg pixel);
    reg glyph_rom [0:8191];
    initial $readmemh("info_font.hex",glyph_rom);
    always @(posedge clk)pixel<=glyph_rom[{ascii,row,col}];
endmodule
