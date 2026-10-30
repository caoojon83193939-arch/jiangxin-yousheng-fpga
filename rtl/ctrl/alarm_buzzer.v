// M15 alarm buzzer (board buzzer on H11, PNP driver, low = on, passive
// transducer: needs a square wave; hardware switch SW5 can cut it off).
// Same two-tone pattern as the HDMI alarm (alarm_tone_sel: F1/F2 swapped
// every HALF_MS from the alarm start), clocked from the same emergency
// level, so the buzzer and the HDMI siren change pitch together. Off (high)
// outside an alarm.
module alarm_buzzer #(
    parameter integer CLK_HZ = 25_000_000,
    parameter integer F1_HZ = 800,
    parameter integer F2_HZ = 1000,
    parameter integer HALF_MS = 250
)(
    input wire clk, rst, emergency,
    output reg buzzer_n
);
    localparam integer HALF_CYC = (CLK_HZ / 1000) * HALF_MS;
    localparam integer T1 = CLK_HZ / (2 * F1_HZ);
    localparam integer T2 = CLK_HZ / (2 * F2_HZ);
    reg [$clog2(HALF_CYC)-1:0] seg;
    reg hi;
    reg [$clog2(T1)-1:0] wave;
    always @(posedge clk or posedge rst) begin
        if (rst) begin seg <= 0; hi <= 0; wave <= 0; buzzer_n <= 1; end
        else if (!emergency) begin seg <= 0; hi <= 0; wave <= 0; buzzer_n <= 1; end
        else begin
            if (seg == HALF_CYC - 1) begin seg <= 0; hi <= ~hi; end
            else seg <= seg + 1'b1;
            if (wave >= (hi ? T2 : T1) - 1) begin wave <= 0; buzzer_n <= ~buzzer_n; end
            else wave <= wave + 1'b1;
        end
    end
endmodule
