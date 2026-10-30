// M15 run-time clock from reset (pixel-clock domain): a millisecond count
// for the UART log time stamps and BCD hh:mm:ss for the maintenance header
// and the staff display. Hours run 00..99 and wrap (over four days).
// The BCD digits are counted directly, so no binary-to-decimal conversion.
module uptime_clock #(
    parameter integer CLK_HZ = 25_000_000
)(
    input wire clk, rst,
    output reg [31:0] ms,
    output reg ms_tick,
    output reg [23:0] hms          // {h1,h0,m1,m0,s1,s0}
);
    localparam integer MS_CYC = CLK_HZ / 1000;
    reg [$clog2(MS_CYC)-1:0] prescale;
    reg [9:0] ms_in_s;
    always @(posedge clk or posedge rst) begin
        if (rst) begin prescale <= 0; ms <= 0; ms_tick <= 0; ms_in_s <= 0; hms <= 0; end
        else begin
            ms_tick <= 0;
            if (prescale == MS_CYC - 1) begin
                prescale <= 0; ms <= ms + 1'b1; ms_tick <= 1;
                if (ms_in_s == 999) begin
                    ms_in_s <= 0;
                    if (hms[3:0] != 9) hms[3:0] <= hms[3:0] + 1'b1;
                    else begin
                        hms[3:0] <= 0;
                        if (hms[7:4] != 5) hms[7:4] <= hms[7:4] + 1'b1;
                        else begin
                            hms[7:4] <= 0;
                            if (hms[11:8] != 9) hms[11:8] <= hms[11:8] + 1'b1;
                            else begin
                                hms[11:8] <= 0;
                                if (hms[15:12] != 5) hms[15:12] <= hms[15:12] + 1'b1;
                                else begin
                                    hms[15:12] <= 0;
                                    if (hms[19:16] != 9) hms[19:16] <= hms[19:16] + 1'b1;
                                    else begin
                                        hms[19:16] <= 0;
                                        hms[23:20] <= hms[23:20] == 9 ? 4'd0 : hms[23:20] + 1'b1;
                                    end
                                end
                            end
                        end
                    end
                end else ms_in_s <= ms_in_s + 1'b1;
            end else prescale <= prescale + 1'b1;
        end
    end
endmodule
