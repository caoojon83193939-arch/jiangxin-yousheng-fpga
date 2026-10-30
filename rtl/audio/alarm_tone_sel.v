// alarm_tone_sel.v
// Chooses the DDS phase increment for audio_tone_pcm:
//   normal    : 0 (module default tone)
//   emergency : alternate F1 / F2 every HALF_MS milliseconds (two-tone alarm)
module alarm_tone_sel #(
    parameter integer CLK_HZ  = 25_000_000,
    parameter integer FS_HZ   = 48_000,
    parameter integer F1_HZ   = 800,
    parameter integer F2_HZ   = 1000,
    parameter integer HALF_MS = 250
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        emergency,
    output reg  [31:0] phase_inc
);
    localparam [63:0] I1 = ((64'd1 << 32) * F1_HZ) / FS_HZ;
    localparam [63:0] I2 = ((64'd1 << 32) * F2_HZ) / FS_HZ;
    localparam integer HALF_CYC = (CLK_HZ / 1000) * HALF_MS;

    reg [31:0] cnt;
    reg        hi;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt <= 32'd0; hi <= 1'b0; phase_inc <= 32'd0;
        end else if (!emergency) begin
            cnt <= 32'd0; hi <= 1'b0; phase_inc <= 32'd0;
        end else begin
            if (cnt >= HALF_CYC - 1) begin cnt <= 32'd0; hi <= ~hi; end
            else cnt <= cnt + 32'd1;
            phase_inc <= hi ? I2[31:0] : I1[31:0];
        end
    end
endmodule
