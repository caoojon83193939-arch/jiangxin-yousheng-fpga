// audio_tone_pcm.v
// 48 kHz PCM test-tone source in the pixel-clock domain (no extra PLL).
// A fractional divider produces the sample tick (CLK_HZ/FS on average, so the
// ACR CTS measured by audio_arc_calculate settles at exactly CLK_HZ*N/(128*FS)),
// a 32-bit DDS phase accumulator selects a quarter-wave sine LUT.
// Output format matches hdmi_1_4b_transmitter_core: valid pulse + 24-bit L/R.
module audio_tone_pcm #(
    parameter integer CLK_HZ  = 25_000_000,
    parameter integer FS_HZ   = 48_000,
    parameter integer TONE_HZ = 1000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        mute,
    input  wire [2:0]  volume,        // 0 = -42 dB ... 7 = 0 dB (6 dB steps)
    input  wire [31:0] phase_inc_ovr, // 0 = use TONE_HZ, else DDS increment override
    output reg         sample_valid,
    output reg  [23:0] sample_left,
    output reg  [23:0] sample_right
);
    // DDS increment for TONE_HZ at FS_HZ: TONE * 2^32 / FS (64-bit intermediate)
    localparam [63:0] INC64 = ((64'd1 << 32) * TONE_HZ) / FS_HZ;
    localparam [31:0] INC_DEFAULT = INC64[31:0];

    function [15:0] qsin;   // quarter-wave, 64 entries, 0..32767
        input [5:0] i;
        reg [15:0] q;
        begin
            case (i)
            6'd0: q = 16'd402;
            6'd1: q = 16'd1206;
            6'd2: q = 16'd2009;
            6'd3: q = 16'd2811;
            6'd4: q = 16'd3612;
            6'd5: q = 16'd4410;
            6'd6: q = 16'd5205;
            6'd7: q = 16'd5998;
            6'd8: q = 16'd6786;
            6'd9: q = 16'd7571;
            6'd10: q = 16'd8351;
            6'd11: q = 16'd9126;
            6'd12: q = 16'd9896;
            6'd13: q = 16'd10659;
            6'd14: q = 16'd11417;
            6'd15: q = 16'd12167;
            6'd16: q = 16'd12910;
            6'd17: q = 16'd13645;
            6'd18: q = 16'd14372;
            6'd19: q = 16'd15090;
            6'd20: q = 16'd15800;
            6'd21: q = 16'd16499;
            6'd22: q = 16'd17189;
            6'd23: q = 16'd17869;
            6'd24: q = 16'd18537;
            6'd25: q = 16'd19195;
            6'd26: q = 16'd19841;
            6'd27: q = 16'd20475;
            6'd28: q = 16'd21096;
            6'd29: q = 16'd21705;
            6'd30: q = 16'd22301;
            6'd31: q = 16'd22884;
            6'd32: q = 16'd23452;
            6'd33: q = 16'd24007;
            6'd34: q = 16'd24547;
            6'd35: q = 16'd25072;
            6'd36: q = 16'd25582;
            6'd37: q = 16'd26077;
            6'd38: q = 16'd26556;
            6'd39: q = 16'd27019;
            6'd40: q = 16'd27466;
            6'd41: q = 16'd27896;
            6'd42: q = 16'd28310;
            6'd43: q = 16'd28706;
            6'd44: q = 16'd29085;
            6'd45: q = 16'd29447;
            6'd46: q = 16'd29791;
            6'd47: q = 16'd30117;
            6'd48: q = 16'd30424;
            6'd49: q = 16'd30714;
            6'd50: q = 16'd30985;
            6'd51: q = 16'd31237;
            6'd52: q = 16'd31470;
            6'd53: q = 16'd31685;
            6'd54: q = 16'd31880;
            6'd55: q = 16'd32057;
            6'd56: q = 16'd32213;
            6'd57: q = 16'd32351;
            6'd58: q = 16'd32469;
            6'd59: q = 16'd32567;
            6'd60: q = 16'd32646;
            6'd61: q = 16'd32705;
            6'd62: q = 16'd32745;
            6'd63: q = 16'd32765;
            default: q = 16'd0;
            endcase
            qsin = q;
        end
    endfunction

    // ---- sample tick: fractional divider ----
    reg [31:0] acc;
    reg        tick;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            acc  <= 32'd0;
            tick <= 1'b0;
        end else begin
            if (acc + FS_HZ >= CLK_HZ) begin
                acc  <= acc + FS_HZ - CLK_HZ;
                tick <= 1'b1;
            end else begin
                acc  <= acc + FS_HZ;
                tick <= 1'b0;
            end
        end
    end

    // ---- DDS ----
    reg  [31:0] phase;
    wire [31:0] inc = (phase_inc_ovr != 32'd0) ? phase_inc_ovr : INC_DEFAULT;
    wire [1:0]  quad = phase[31:30];
    wire [5:0]  idx  = quad[0] ? ~phase[29:24] : phase[29:24];   // mirror in 2nd/4th quadrant
    wire [15:0] mag  = qsin(idx);
    wire signed [16:0] s17 = quad[1] ? -$signed({1'b0, mag}) : $signed({1'b0, mag});

    // volume: shift right by (7 - volume); full scale at volume 7 is +-2^22 (half of 24-bit range)
    wire signed [23:0] s24_full = {s17, 7'b0};
    wire signed [23:0] s24 = mute ? 24'sd0 : (s24_full >>> (3'd7 - volume));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase        <= 32'd0;
            sample_valid <= 1'b0;
            sample_left  <= 24'd0;
            sample_right <= 24'd0;
        end else begin
            sample_valid <= tick;
            if (tick) begin
                phase        <= phase + inc;
                sample_left  <= s24;
                sample_right <= s24;
            end
        end
    end
endmodule
