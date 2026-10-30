// One uninterrupted sample clock for HDMI/ACR, independently of card health.
// Source priority: emergency alarm > card WAV (music/narration) > test tone.
// The 1 kHz test tone (-18 dB) plays only when the reader reports that no WAV
// is playable (tone_fallback); it shares the alarm DDS, so both have the same
// 48 kHz cadence. SW1 mute silences WAV and test tone, never the alarm.
// M13: track_ready (reader has a track selected, synchronized) separates a
// real underrun from the intended silence between tour narrations: once the
// buffer has drained with no track selected, the next track starts afresh.
// M15: volume 1..8 (3 dB steps: 8 = 0 dB ... 1 = -21 dB) scales WAV and the
// test tone in the same clock as the source mux (sample timing unchanged;
// volume 8 is bit-identical to M14b); the alarm is never scaled.
// raw_left/raw_right are the same samples before the volume, for the
// spectrum analyser, so the bars follow the programme rather than the hall
// volume. One 16x9 multiplier per channel (25 MHz, plenty of slack).
module guide_audio_output #(
    parameter integer CLK_HZ = 25_000_000,
    parameter integer FS_HZ = 48_000
)(
    input wire clk, rst, emergency, mute, tone_fallback, track_ready,
    input wire [3:0] volume,
    input wire pcm_empty,
    input wire [31:0] pcm_data, // {right[15:0],left[15:0]}
    output wire pcm_pop,
    output reg sample_valid,
    output reg [23:0] sample_left, sample_right,
    output reg [23:0] raw_left, raw_right,
    output reg underrun,
    output wire music_active,
    output wire tone_active
);
    wire alarm_valid;
    wire [23:0] alarm_l, alarm_r;
    wire [31:0] alarm_inc;
    alarm_tone_sel #(.CLK_HZ(CLK_HZ),.FS_HZ(FS_HZ)) alarm_sel(
        .clk(clk),.rst(rst),.emergency(emergency),.phase_inc(alarm_inc));
    // phase_inc is 0 outside emergency -> the DDS default 1 kHz test tone.
    audio_tone_pcm #(.CLK_HZ(CLK_HZ),.FS_HZ(FS_HZ)) alarm_source(
        .clk(clk),.rst(rst),.mute(1'b0),.volume(emergency ? 3'd7 : 3'd4),
        .phase_inc_ovr(alarm_inc),.sample_valid(alarm_valid),
        .sample_left(alarm_l),.sample_right(alarm_r));
    reg started;
    wire tone_mode = tone_fallback && pcm_empty;
    assign pcm_pop = alarm_valid && !emergency && !pcm_empty;
    assign music_active = started && !emergency && !mute && !pcm_empty;
    assign tone_active = !emergency && !mute && tone_mode;
    // Source mux (what M14b registered as the sample).
    reg [23:0] src_l, src_r;
    always @(*) begin
        if (emergency) begin src_l = alarm_l; src_r = alarm_r; end
        else if (!pcm_empty) begin
            src_l = mute ? 24'd0 : {pcm_data[15:0],8'd0};
            src_r = mute ? 24'd0 : {pcm_data[31:16],8'd0};
        end else if (tone_fallback) begin
            src_l = mute ? 24'd0 : alarm_l;
            src_r = mute ? 24'd0 : alarm_r;
        end else begin src_l = 0; src_r = 0; end
    end
    function [8:0] gain_of;
        input [3:0] v;
        case (v)
            4'd7: gain_of = 9'd181; 4'd6: gain_of = 9'd128; 4'd5: gain_of = 9'd91; 4'd4: gain_of = 9'd64;
            4'd3: gain_of = 9'd45;  4'd2: gain_of = 9'd32;  4'd1: gain_of = 9'd23;
            default: gain_of = 9'd256;
        endcase
    endfunction
    wire [8:0] gain = gain_of(volume);
    wire signed [25:0] prod_l = $signed(src_l[23:8]) * $signed({1'b0, gain});
    wire signed [25:0] prod_r = $signed(src_r[23:8]) * $signed({1'b0, gain});
    wire full = emergency || gain == 9'd256;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sample_valid <= 0; sample_left <= 0; sample_right <= 0;
            raw_left <= 0; raw_right <= 0;
            underrun <= 0; started <= 0;
        end else begin
            sample_valid <= alarm_valid;
            if (alarm_valid) begin
                raw_left <= src_l; raw_right <= src_r;
                sample_left <= full ? src_l : {prod_l[23:8], 8'd0};
                sample_right <= full ? src_r : {prod_r[23:8], 8'd0};
                if (!emergency) begin
                    if (!pcm_empty) started <= 1;
                    else if (!tone_fallback) begin
                        // A missing card/WAV is reported by tone_fallback, not as underrun.
                        if (started && track_ready) underrun <= 1;
                        else if (!track_ready) started <= 0;
                    end
                end
            end
        end
    end
endmodule
