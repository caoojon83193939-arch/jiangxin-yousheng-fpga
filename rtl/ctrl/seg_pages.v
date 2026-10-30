// M13 eight-digit status display for the staff (the screen is for visitors).
// Pages rotate every PAGE_MS; a setting change jumps to the page showing it.
//   0 status      P n - c  A  s   n shown exhibit, c exhibits found,
//                                 A tour / n manual, s loader state
//                                 (0 no card, 1 idle, 2 scanning, 3 FIFO
//                                 handshake, 4 writing) or error code;
//                                 E replaces P on an error
//                 ALAr    E       during an alarm (fixed)
//   1 resolution  1024 768        source BMP width / height
//   2 scaling     F 62 bIL        fit scale %, bIL bilinear / nn nearest
//   3 picture     b100C125        brightness % / contrast %
//   4 spectrum    SPn 07 45       n music / A alarm / o silent, loudest
//                                 band 01..16 and its level 00..63
//   5 volume      U0L    6        volume level 1..8 (M15)
//   6 run time    U 01.02.03      hours.minutes.seconds since reset (M15)
// A volume change jumps to page 5.
// Pixel-domain values arrive as one snapshot bus with a toggle; the bus only
// changes at the next frame, so it is captured after the toggle is seen.
// SD-domain status levels are display-only and use plain 2-FF synchronisers.
module seg_pages #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer PAGE_MS = 3000
)(
    input wire clk, rst,
    input wire emergency,                    // local
    input wire touring,                      // local (tour controller)
    input wire [3:0] volume,                 // local
    input wire [23:0] uptime,                // local, BCD {h1,h0,m1,m0,s1,s0}
    input wire [76:0] snap_async,
    input wire snap_toggle_async,
    input wire [3:0] state_async,            // loader state or error code
    input wire error_async,
    output reg [63:0] digits                 // d0 (left) .. d7, {dp,g..a} active low
);
    (* async_reg = "true" *) reg snap_meta;
    reg [1:0] snap_sync;
    reg [76:0] snap;
    reg [1:0] seen;            // first snapshot is not a setting change
    (* async_reg = "true" *) reg [4:0] st_meta;
    reg [4:0] st;
    always @(posedge clk or posedge rst) begin
        if (rst) begin snap_meta <= 0; snap_sync <= 0; snap <= 0; st_meta <= 0; st <= 0; seen <= 0; end
        else begin
            snap_meta <= snap_toggle_async;
            snap_sync <= {snap_sync[0], snap_meta};
            if (snap_sync[1] ^ snap_sync[0]) begin snap <= snap_async; seen[0] <= 1'b1; end
            seen[1] <= seen[0];
            st_meta <= {error_async, state_async};
            st <= st_meta;
        end
    end
    // Snapshot layout (see top.v).
    wire [15:0] w = snap[76:61], h = snap[60:45];
    wire [19:0] scale = snap[44:25];
    wire valid = snap[24];
    wire [1:0] bright = snap[23:22];
    wire contrast = snap[21], smooth = snap[20];
    wire [1:0] role = snap[19:18];
    wire [2:0] count = snap[17:15];
    wire muted = snap[14], alarm_audio = snap[13], shown = snap[12];
    wire [3:0] band = snap[11:8];
    wire [5:0] level = snap[7:2];
    wire audible = snap[1];

    localparam [7:0] BL = 8'hFF, DASH = 8'hBF;
    localparam [7:0] C_U = 8'hC1;
    localparam [7:0] C_A = 8'h88, C_b = 8'h83, C_C = 8'hC6, C_E = 8'h86, C_F = 8'h8E,
                     C_I = 8'hCF, C_L = 8'hC7, C_n = 8'hAB, C_o = 8'hA3, C_P = 8'h8C,
                     C_r = 8'hAF, C_S = 8'h92;
    function [7:0] dig;
        input [3:0] v;
        case (v)
            4'd0: dig = 8'hC0; 4'd1: dig = 8'hF9; 4'd2: dig = 8'hA4; 4'd3: dig = 8'hB0;
            4'd4: dig = 8'h99; 4'd5: dig = 8'h92; 4'd6: dig = 8'h82; 4'd7: dig = 8'hF8;
            4'd8: dig = 8'h80; 4'd9: dig = 8'h90; 4'hA: dig = 8'h88; 4'hB: dig = 8'h83;
            4'hC: dig = 8'hC6; 4'hD: dig = 8'hA1; 4'hE: dig = 8'h86; default: dig = 8'h8E;
        endcase
    endfunction
    // Digit with leading-zero blanking (lead = all higher digits are zero).
    function [7:0] ldig;
        input [3:0] v;
        input lead;
        ldig = (lead && v == 0) ? BL : dig(v);
    endfunction

    // Page selection: rotate, or jump to the page of a changed setting.
    localparam integer MS_CYC = CLK_HZ / 1000;
    reg [$clog2(MS_CYC)-1:0] prescale;
    reg [$clog2(PAGE_MS)-1:0] page_ms;
    reg [2:0] page;
    reg [2:0] prev_pic;
    reg prev_smooth;
    reg [3:0] prev_vol;
    reg [2:0] prev_page_key;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prescale <= 0; page_ms <= 0; page <= 0;
            prev_pic <= 0; prev_smooth <= 0; prev_page_key <= 0; prev_vol <= 0;
        end else begin
            prescale <= prescale == MS_CYC - 1 ? 0 : prescale + 1'b1;
            prev_pic <= {bright, contrast};
            prev_smooth <= smooth;
            prev_vol <= volume;
            prev_page_key <= {shown, role};
            if (!seen[1]) ;
            else if (prev_pic != {bright, contrast}) begin page <= 3; page_ms <= 0; end
            else if (prev_smooth != smooth) begin page <= 2; page_ms <= 0; end
            else if (prev_vol != volume) begin page <= 5; page_ms <= 0; end
            else if (prev_page_key != {shown, role}) begin page <= 1; page_ms <= 0; end
            else if (prescale == MS_CYC - 1) begin
                if (page_ms == PAGE_MS - 1) begin
                    page_ms <= 0;
                    page <= page == 6 ? 3'd0 : page + 1'b1;
                end else page_ms <= page_ms + 1'b1;
            end
        end
    end

    // Two-digit decimal of a 6-bit level (0..63).
    wire [3:0] lv_t = level >= 60 ? 6 : level >= 50 ? 5 : level >= 40 ? 4 :
                      level >= 30 ? 3 : level >= 20 ? 2 : level >= 10 ? 1 : 0;
    wire [5:0] lv_o = level - lv_t * 6'd10;
    wire [4:0] band1 = {1'b0, band} + 5'd1;          // 1..16
    wire band_hi = band1 >= 5'd10;
    wire [4:0] band_lo5 = band_hi ? band1 - 5'd10 : band1;
    wire [3:0] band_lo = band_lo5[3:0];
    wire scale_big = scale[19:12] != 0;              // >= 1000 %: shown as ---

    reg [63:0] next;
    always @(*) begin
        next = {8{BL}};
        if (emergency)
            next = {C_A, C_L, C_A, C_r, BL, BL, BL, C_E};
        else case (page)
            3'd0: next = {st[4] ? C_E : C_P, shown ? dig({2'b0, role} + 4'd1) : DASH, DASH,
                          dig({1'b0, count}), BL, touring ? C_A : C_n, BL, dig(st[3:0])};
            3'd1: if (!valid) next = {DASH, DASH, DASH, DASH, BL, DASH, DASH, DASH};
                  else next = {ldig(w[15:12], 1'b1), ldig(w[11:8], w[15:12] == 0),
                               ldig(w[7:4], w[15:8] == 0), dig(w[3:0]),
                               ldig(h[15:12], 1'b1), ldig(h[11:8], h[15:12] == 0),
                               ldig(h[7:4], h[15:8] == 0), dig(h[3:0])};
            3'd2: next = {C_F,
                          !valid || scale_big ? DASH : ldig(scale[11:8], 1'b1),
                          !valid || scale_big ? DASH : ldig(scale[7:4], scale[11:8] == 0),
                          !valid || scale_big ? DASH : dig(scale[3:0]), BL,
                          smooth ? C_b : C_n, smooth ? C_I : C_n, smooth ? C_L : BL};
            3'd3: next = {C_b, bright[1] ? dig(1) : BL,
                          bright == 0 ? dig(6) : bright == 1 ? dig(8) : bright == 2 ? dig(0) : dig(2),
                          dig(0), C_C, dig(1), contrast ? dig(2) : dig(0), contrast ? dig(5) : dig(0)};
            3'd5: next = {C_U, dig(0), C_L, BL, BL, BL, BL, dig(volume)};
            3'd6: next = {C_U, BL, dig(uptime[23:20]), dig(uptime[19:16]) & 8'h7F,
                          dig(uptime[15:12]), dig(uptime[11:8]) & 8'h7F, dig(uptime[7:4]), dig(uptime[3:0])};
            default: next = {C_S, C_P, alarm_audio ? C_A : (muted || !audible) ? C_o : C_n, BL,
                             band_hi ? dig(1) : dig(0), dig(band_lo),
                             dig(lv_t), dig(lv_o[3:0])};
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst) digits <= {8{BL}};
        else digits <= next;
    end
endmodule
