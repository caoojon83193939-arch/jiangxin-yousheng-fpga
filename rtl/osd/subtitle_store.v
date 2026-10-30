// M11 two-bank card subtitle memory. Written in the SD clock domain by
// subtitle_loader, read in the pixel clock domain by osd_overlay. Unreset
// synchronous ports infer dual-clock block RAM (same style as pcm_bank_buffer).
//   text : {bank, position[5:0]}           -> glyph index[5:0]
//   glyph: {bank, glyph[5:0], row[3:0]}    -> 16 pixels, bit 15 = left
module subtitle_store(
    input wire wr_clk,
    input wire wr_en, wr_text, wr_bank,
    input wire [9:0] wr_addr,
    input wire [15:0] wr_data,
    input wire rd_clk,
    input wire [6:0] text_addr,
    output reg [5:0] text_q,
    input wire [10:0] glyph_addr,
    output reg [15:0] glyph_q
);
    reg [5:0] text_mem [0:127];
    reg [15:0] glyph_mem [0:2047];
    always @(posedge wr_clk) begin
        if (wr_en && wr_text) text_mem[{wr_bank, wr_addr[5:0]}] <= wr_data[5:0];
        if (wr_en && !wr_text) glyph_mem[{wr_bank, wr_addr}] <= wr_data;
    end
    always @(posedge rd_clk) begin
        text_q <= text_mem[text_addr];
        glyph_q <= glyph_mem[glyph_addr];
    end
endmodule
