// M15 key mapping (pixel-clock domain), moved out of top.v so it can be
// tested on its own.
//   Exhibition view (maint=0, visitors)     Maintenance view (maint=1, staff)
//   KEY2 short : next page                  KEY2 short : volume +
//   KEY3 short : previous page              KEY3 short : volume -
//   KEY3 long  : brightness step            KEY3 long  : brightness step
//   KEY4 short : tour on / off              KEY4 short : tour off/content/30 s/60 s
//   KEY4 long  : alarm                      KEY4 long  : alarm
//   alarm: KEY2 press clears it (both views)
// KEY2 page turn / volume act on the short *release* (as KEY3 always did),
// so holding KEY2 for the KEY2+KEY3 standby combination no longer turns a
// page first. The KEY2 press that clears an alarm is consumed: its release
// does not also turn a page. KEY3 long held together with KEY2 is the
// standby combination, not a brightness step. key_block (standby / wake
// mask from power_ctrl) suppresses everything.
// log_valid/log_arg report each accepted action for the UART log:
//   1 next  2 prev  3 tour toggle  4 dwell step  5 brightness  6 vol+  7 vol-
//   8 alarm on  9 alarm off
module key_actions(
    input wire clk, rst,
    input wire k2_press, k2_short, k2_held,
    input wire k3_short, k3_long,
    input wire k4_short, k4_long,
    input wire maint, key_block, tour_on,
    output reg emergency,
    output reg [1:0] bright,
    output reg [3:0] volume,            // 1..8, 8 = full scale
    output reg [1:0] dwell_sel,
    output reg next_t, prev_t, auto_t,
    output reg log_valid,
    output reg [3:0] log_arg
);
    reg k2_consumed;
    wire key_ok = !emergency && !key_block;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            emergency <= 0; bright <= 2'd2; volume <= 4'd8; dwell_sel <= 0;
            next_t <= 0; prev_t <= 0; auto_t <= 0; k2_consumed <= 0;
            log_valid <= 0; log_arg <= 0;
        end else begin
            log_valid <= 0;
            if (!k2_held) k2_consumed <= 0;
            // Alarm first: it overrides every other action.
            if (k4_long && !key_block && !emergency) begin
                emergency <= 1; log_valid <= 1; log_arg <= 4'd8;
            end else if (emergency && k2_press && !key_block) begin
                emergency <= 0; k2_consumed <= 1; log_valid <= 1; log_arg <= 4'd9;
            end else if (k3_long && !key_block && !k2_held) begin
                bright <= bright + 2'd1; log_valid <= 1; log_arg <= 4'd5;
            end else if (k2_short && key_ok && !k2_consumed) begin
                if (maint) begin
                    if (volume != 4'd8) volume <= volume + 1'b1;
                    log_valid <= 1; log_arg <= 4'd6;
                end else begin
                    next_t <= ~next_t; log_valid <= 1; log_arg <= 4'd1;
                end
            end else if (k3_short && key_ok) begin
                if (maint) begin
                    if (volume != 4'd1) volume <= volume - 1'b1;
                    log_valid <= 1; log_arg <= 4'd7;
                end else begin
                    prev_t <= ~prev_t; log_valid <= 1; log_arg <= 4'd2;
                end
            end else if (k4_short && key_ok) begin
                // Maintenance: off -> content -> >=30 s -> >=60 s -> off.
                // tour_on lags by a frame, far shorter than two key presses.
                if (!maint) begin auto_t <= ~auto_t; log_valid <= 1; log_arg <= 4'd3; end
                else if (!tour_on) begin auto_t <= ~auto_t; dwell_sel <= 0; log_valid <= 1; log_arg <= 4'd3; end
                else if (dwell_sel != 2'd2) begin dwell_sel <= dwell_sel + 1'b1; log_valid <= 1; log_arg <= 4'd4; end
                else begin auto_t <= ~auto_t; dwell_sel <= 0; log_valid <= 1; log_arg <= 4'd3; end
            end
        end
    end
endmodule
