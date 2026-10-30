// M14 power management, pixel-clock domain (the clock that keeps running).
//   Closing-hours standby: KEY2 and KEY3 held together for STANDBY_MS enters
//     standby (not during an alarm). In standby the top level holds the HDMI
//     serialiser in reset (no TMDS clock -> the monitor goes to sleep),
//     pauses card reading and turns off the seven-segment display and LEDs.
//     Any key press wakes; the photograph stays black for WAKE_DARK_MS so the
//     monitor can re-synchronise before the exhibit opens through the iris.
//   Key masking: the keys that entered or left standby are ignored until all
//     keys are released, so neither the combination nor the wake press acts
//     as a page turn, a brightness step, a tour toggle or an alarm.
//   Staff display: in the exhibition view the seven-segment display goes dark
//     SEG_IDLE_MS after the last key or setting change; the maintenance view
//     keeps it on. Any activity lights it again (the key still acts).
module power_ctrl #(
    parameter integer CLK_HZ = 25_000_000,
    parameter integer STANDBY_MS = 2000,
    parameter integer WAKE_DARK_MS = 2000,
    parameter integer SEG_IDLE_MS = 60000
)(
    input wire clk, rst,
    input wire key2_held, key3_held, key4_held,
    input wire any_press,                 // press edge of KEY2/KEY3/KEY4
    input wire activity,                  // any key event or setting change
    input wire emergency, maint,
    output reg standby,
    output wire dark,                     // standby or monitor re-sync time
    output wire key_block,                // ignore key events
    output wire seg_on
);
    localparam integer MS_CYC = CLK_HZ / 1000;
    localparam integer TW = $clog2(SEG_IDLE_MS + 1);
    reg [$clog2(MS_CYC)-1:0] prescale;
    wire ms_tick = prescale == MS_CYC - 1;
    reg [TW-1:0] combo_ms, dark_ms, idle_ms;
    reg mask;
    wire all_released = !key2_held && !key3_held && !key4_held;
    assign dark = standby || dark_ms != 0;
    assign key_block = standby || mask;
    assign seg_on = !standby && (maint || idle_ms < SEG_IDLE_MS);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prescale <= 0; combo_ms <= 0; dark_ms <= 0; idle_ms <= 0;
            standby <= 0; mask <= 0;
        end else begin
            prescale <= ms_tick ? 0 : prescale + 1'b1;
            if (standby) begin
                combo_ms <= 0;
                if (any_press) begin
                    standby <= 0; mask <= 1; dark_ms <= WAKE_DARK_MS; idle_ms <= 0;
                end
            end else begin
                if (mask && all_released) mask <= 0;
                if (ms_tick && dark_ms != 0) dark_ms <= dark_ms - 1'b1;
                if (!key2_held || !key3_held || emergency || mask) combo_ms <= 0;
                else if (ms_tick) begin
                    if (combo_ms >= STANDBY_MS - 1) begin
                        standby <= 1; mask <= 1; combo_ms <= 0; dark_ms <= 0;
                    end else combo_ms <= combo_ms + 1'b1;
                end
                if (activity || emergency) idle_ms <= 0;
                else if (ms_tick && idle_ms < SEG_IDLE_MS) idle_ms <= idle_ms + 1'b1;
            end
        end
    end
endmodule
