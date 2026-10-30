// pixel_adjust.v
// Per-pixel contrast then brightness, 2-cycle pipeline, saturating.
//   contrast : level 0 = x1.0, 1 = x1.25 around mid-grey 128
//   brightness: 4 levels, gain = {154,205,256,307}/256  (0.6, 0.8, 1.0, 1.2)
module pixel_adjust(
    input  wire        clk,
    input  wire        rst,
    input  wire        contrast_hi,
    input  wire [1:0]  bright_lvl,
    input  wire [23:0] rgb_i,
    input  wire        hs_i, vs_i, de_i,
    output reg  [23:0] rgb_o,
    output reg         hs_o, vs_o, de_o
);
    function [8:0] gain_lut;
        input [1:0] l;
        case (l)
            2'd0: gain_lut = 9'd154;
            2'd1: gain_lut = 9'd205;
            2'd2: gain_lut = 9'd256;
            default: gain_lut = 9'd307;
        endcase
    endfunction

    // stage 1: contrast  c = (p-128)*1.25 + 128 = p*5/4 - 32
    function [7:0] contrast_f;
        input [7:0] p;
        input       hi;
        reg [10:0] t;
        begin
            if (!hi) contrast_f = p;
            else begin
                t = {3'b0, p} * 11'd5;          // p*5
                t = t >> 2;                      // /4
                if (t < 11'd32) contrast_f = 8'd0;
                else if (t - 11'd32 > 11'd255) contrast_f = 8'd255;
                else contrast_f = t[7:0] - 8'd32;
            end
        end
    endfunction

    // stage 2: brightness
    function [7:0] bright_f;
        input [7:0] p;
        input [8:0] g;
        reg [16:0] m;
        begin
            m = p * g;                            // 8x9 = 17 bits, /256
            bright_f = (m[16:8] > 9'd255) ? 8'd255 : m[15:8];
        end
    endfunction

    reg [23:0] s1;
    reg        hs1, vs1, de1;
    wire [8:0] g = gain_lut(bright_lvl);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s1 <= 24'd0; hs1 <= 1'b0; vs1 <= 1'b0; de1 <= 1'b0;
            rgb_o <= 24'd0; hs_o <= 1'b0; vs_o <= 1'b0; de_o <= 1'b0;
        end else begin
            s1  <= {contrast_f(rgb_i[23:16], contrast_hi),
                    contrast_f(rgb_i[15:8],  contrast_hi),
                    contrast_f(rgb_i[7:0],   contrast_hi)};
            hs1 <= hs_i; vs1 <= vs_i; de1 <= de_i;
            rgb_o <= {bright_f(s1[23:16], g), bright_f(s1[15:8], g), bright_f(s1[7:0], g)};
            hs_o <= hs1; vs_o <= vs1; de_o <= de1;
        end
    end
endmodule
