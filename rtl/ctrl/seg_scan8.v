// M13: all eight digits of the HX4S20C display (two 4-digit modules).
// seg_sel[0] = SEL0 (F16, leftmost) .. seg_sel[7] = SEL7 (D11), selected low
// exactly as the official six-digit seg_scan drives SEL2..SEL7.
// digits = {d0 (left), .., d7}, each {dp,g,f,e,d,c,b,a}, active low.
// M14: enable=0 deselects every digit (display dark, no segment current).
module seg_scan8 #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SCAN_HZ = 200            // full refresh rate
)(
    input wire clk, rst,
    input wire enable,
    input wire [63:0] digits,
    output reg [7:0] seg_sel,
    output reg [7:0] seg_data
);
    localparam integer SLOT_CYC = CLK_HZ / (SCAN_HZ * 8);
    reg [$clog2(SLOT_CYC)-1:0] timer;
    reg [2:0] slot;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            timer <= 0; slot <= 0; seg_sel <= 8'hFF; seg_data <= 8'hFF;
        end else begin
            if (timer == SLOT_CYC - 1) begin
                timer <= 0;
                slot <= slot + 1'b1;
            end else timer <= timer + 1'b1;
            seg_sel <= enable ? ~(8'd1 << slot) : 8'hFF;
            seg_data <= enable ? digits[63 - 8 * slot -: 8] : 8'hFF;
        end
    end
endmodule
