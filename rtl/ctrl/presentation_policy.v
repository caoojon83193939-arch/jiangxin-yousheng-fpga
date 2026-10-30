// M13 presentation policy: the exhibition view only presents, comparison and
// diagnostics belong to the maintenance view (SW2 up).
//   * Scaling: exhibition always bilinear; maintenance follows SW4 (live A/B).
//     Any change of the effective mode (SW4 in maintenance, or SW2 itself)
//     reloads the displayed exhibit once (reload_t toggles).
//   * Test tone (no playable WAV / no card): only in the maintenance view;
//     the exhibition view stays silent and shows NO IMAGE instead.
// Switch bounce: 2-FF synchronisers, then a 40 ms hold-off after each
// accepted change; the start-up value is taken after the synchronisers have
// settled and never counts as a change.
module presentation_policy #(
    parameter integer CLK_HZ = 25_000_000,
    parameter integer HOLD_MS = 40
)(
    input wire clk, rst,
    input wire sw_maint, sw_smooth,       // raw switches
    output reg maint,                     // debounced SW2
    output reg smooth,                    // effective scaling mode
    output reg reload_t                   // toggles on each effective change
);
    localparam integer HOLD = CLK_HZ / 1000 * HOLD_MS;
    (* async_reg = "true" *) reg [1:0] maint_meta, smooth_meta;
    reg [$clog2(HOLD+1)-1:0] maint_timer, smooth_timer;
    reg [3:0] settle;
    wire started = settle == 0;
    wire target = maint ? smooth_meta[1] : 1'b1;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            maint_meta <= 0; smooth_meta <= 0; maint <= 0; smooth <= 1; reload_t <= 0;
            maint_timer <= 0; smooth_timer <= 0; settle <= 4'd8;
        end else begin
            maint_meta <= {maint_meta[0], sw_maint};
            smooth_meta <= {smooth_meta[0], sw_smooth};
            if (maint_timer != 0) maint_timer <= maint_timer - 1'b1;
            if (smooth_timer != 0) smooth_timer <= smooth_timer - 1'b1;
            if (!started) begin
                settle <= settle - 1'b1;
                maint <= maint_meta[1];
                smooth <= maint_meta[1] ? smooth_meta[1] : 1'b1;
            end else begin
                if (maint_meta[1] != maint && maint_timer == 0) begin
                    maint <= maint_meta[1]; maint_timer <= HOLD;
                end
                if (target != smooth && smooth_timer == 0) begin
                    smooth <= target; reload_t <= ~reload_t; smooth_timer <= HOLD;
                end
            end
        end
    end
endmodule
