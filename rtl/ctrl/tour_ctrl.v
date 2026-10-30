// M13 guided-tour pacing, in the 50 MHz board-clock domain so the nearly
// full SD domain only receives two toggles. A page advances when
//   * it has been shown for at least MIN_DWELL_MS, and
//   * its caption has scrolled through once (pass_toggle from the OSD), and
//   * its narration has finished (guide_done), or no narration is playing,
// or unconditionally after MAX_DWELL_MS (e.g. the caption never completes).
// A narration or caption therefore decides its own page time: editing the
// card content changes the tour rhythm without re-synthesis.
// M14: dwell_sel sets the minimum time per page (the configurable tour
// period): 0 = content (MIN_DWELL_MS), 1 = at least 30 s, 2 = at least 60 s.
// The page still waits for its narration and caption, so a long setting
// never cuts a sentence; MAX_DWELL_MS bounds every setting.
// While the tour is off (KEY4) and nobody touches a key for IDLE_RESUME_MS,
// the tour resumes, so a terminal left on a page returns to its loop.
// All *_async inputs are registered SD/pixel-domain signals; each is only a
// level or a toggle, synchronised here with two flip-flops.
module tour_ctrl #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer MIN_DWELL_MS = 6000,
    parameter integer MAX_DWELL_MS = 90000,
    parameter integer RESEND_MS = 10000,
    parameter integer WRAP_GUARD_MS = 2000,
    parameter integer IDLE_RESUME_MS = 60000
)(
    input wire clk, rst,
    input wire emergency,              // local (this clock domain)
    input wire user_activity,          // local pulse: any key event
    input wire auto_en_async, display_valid_async, guide_done_async, narr_async,
    input wire [1:0] role_async,
    input wire [1:0] dwell_sel,        // local
    input wire pass_toggle_async,
    output reg advance_toggle,
    output reg auto_on_toggle,
    output wire touring                // for the status display
);
    localparam integer MS_CYC = CLK_HZ / 1000;
    localparam integer MSW = $clog2(MS_CYC);
    localparam integer TW = $clog2(MAX_DWELL_MS + IDLE_RESUME_MS + 1);
    (* async_reg = "true" *) reg [5:0] meta;
    reg [5:0] level;
    (* async_reg = "true" *) reg pass_meta;
    reg [1:0] pass_sync;
    always @(posedge clk or posedge rst) begin
        if (rst) begin meta <= 0; level <= 0; pass_meta <= 0; pass_sync <= 0; end
        else begin
            meta <= {auto_en_async, display_valid_async, guide_done_async, narr_async, role_async};
            level <= meta;
            pass_meta <= pass_toggle_async;
            pass_sync <= {pass_sync[0], pass_meta};
        end
    end
    wire auto_en = level[5], shown = level[4], guide_done = level[3], narr = level[2];
    wire [1:0] role = level[1:0];
    wire pass = pass_sync[1] ^ pass_sync[0];
    assign touring = auto_en;

    reg [MSW-1:0] prescale;
    wire ms_tick = prescale == MS_CYC - 1;
    reg [TW-1:0] dwell, idle;
    reg [2:0] page_prev;
    reg pass_seen, sent;
    wire page_changed = page_prev != {shown, role};
    wire audio_done = guide_done || !narr;
    wire [TW-1:0] min_dwell = dwell_sel == 2'd2 ? 60000 : dwell_sel == 2'd1 ? 30000 : MIN_DWELL_MS;
    wire due = dwell >= MAX_DWELL_MS ||
               (dwell >= min_dwell && pass_seen && audio_done);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prescale <= 0; dwell <= 0; idle <= 0; page_prev <= 0;
            pass_seen <= 0; sent <= 0; advance_toggle <= 0; auto_on_toggle <= 0;
        end else begin
            prescale <= ms_tick ? {MSW{1'b0}} : prescale + 1'b1;
            page_prev <= {shown, role};
            // Dwell restarts for every newly shown page, after an alarm and
            // whenever the tour is off.
            if (page_changed || emergency || !auto_en || !shown) begin
                dwell <= 0; pass_seen <= 0; sent <= 0;
            end else begin
                if (ms_tick && dwell < MAX_DWELL_MS) dwell <= dwell + 1'b1;
                // A wrap in the first seconds may still be the previous caption.
                if (pass && dwell >= WRAP_GUARD_MS) pass_seen <= 1;
                if (!sent && due) begin
                    advance_toggle <= ~advance_toggle;
                    sent <= 1;
                    dwell <= MAX_DWELL_MS - RESEND_MS;   // a failed load is retried
                end else if (sent && dwell >= MAX_DWELL_MS) sent <= 0;
            end
            if (user_activity || auto_en || emergency || !shown) idle <= 0;
            else if (ms_tick) begin
                if (idle >= IDLE_RESUME_MS - 1) begin
                    idle <= 0;
                    auto_on_toggle <= ~auto_on_toggle;
                end else idle <= idle + 1'b1;
            end
        end
    end
endmodule
