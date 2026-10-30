// M13: loudest of the 16 spectrum bands for the status display. One band is
// compared per pixel clock after each new level set, so no 16-way compare
// tree is added to the pixel domain. levels = 16 x 6-bit, band 0 in [5:0].
module spectrum_peak(
    input wire clk, rst,
    input wire [95:0] levels,
    input wire levels_valid,
    output reg [3:0] band,
    output reg [5:0] level
);
    reg [95:0] work;
    reg [4:0] index;           // 16 = idle
    reg [3:0] best_band;
    reg [5:0] best_level;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            work <= 0; index <= 16; best_band <= 0; best_level <= 0; band <= 0; level <= 0;
        end else if (levels_valid) begin
            work <= levels; index <= 0; best_band <= 0; best_level <= 0;
        end else if (!index[4]) begin
            if (work[5:0] > best_level) begin best_level <= work[5:0]; best_band <= index[3:0]; end
            work <= work >> 6;
            index <= index + 1'b1;
            if (index == 15) begin
                band <= work[5:0] > best_level ? 4'd15 : best_band;
                level <= work[5:0] > best_level ? work[5:0] : best_level;
            end
        end
    end
endmodule
