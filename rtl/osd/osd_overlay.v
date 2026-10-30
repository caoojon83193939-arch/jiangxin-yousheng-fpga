// osd_overlay.v
// On-screen display for the multimedia information terminal:
//   * bottom ticker band (BAND_H px): 16x16 glyphs scaled x2, scrolling left,
//     messages 0..2 rotate, message 3 is the emergency text
//   * top-right status "n/c Bk" (image index / count, brightness level)
//   * emergency: red band, red flashing frame, status shows 'C' marker
// Latency: 3 clocks from *_i to *_o (coordinates are generated internally
// from the delayed stream so nothing upstream needs to know).
// M11: when a card subtitle page is committed (card_valid), the normal ticker
// scrolls that page (text and glyphs from subtitle_store, same 1+1 cycle
// read latency as text_rom/glyph_rom) instead of rotating ROM messages 0..2.
// A new page restarts from the right edge. The alarm text and the alarm
// ticker always use the on-chip ROM. card_valid=0 is exactly the M9b ticker.
// M13: pass_toggle flips each time the normal ticker text has scrolled fully
// through once (not in an alarm); the tour controller waits for one pass so
// a visitor can read the whole caption before the page changes.
module osd_overlay #(
    parameter integer H_ACTIVE   = 640,
    parameter integer V_ACTIVE   = 480,
    parameter integer BAND_H     = 48,
    parameter integer SCROLL_PX  = 2,      // pixels per frame
    parameter integer FRAME_W    = 8       // emergency border width
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [23:0] rgb_i,
    input  wire        hs_i, vs_i, de_i,
    // control (already in this clock domain)
    input  wire        osd_en,
    input  wire        emergency,
    input  wire        safety_valid,      // legacy status; must not dismiss the alarm page
    input  wire [1:0]  img_idx,          // 0..3
    input  wire [2:0]  img_count,        // 0..4
    input  wire [1:0]  bright_lvl,
    input  wire        contrast_hi,
    // card subtitle page (pixel clock domain, changes only at falling VS)
    input  wire        card_valid, card_bank,
    input  wire [5:0]  card_len,
    output wire [6:0]  card_text_addr,
    input  wire [5:0]  card_text,
    output wire [10:0] card_glyph_addr,
    input  wire [15:0] card_bits,
    output reg         pass_toggle,
    output reg  [23:0] rgb_o,
    output reg         hs_o, vs_o, de_o
);
    localparam integer BAND_Y0  = V_ACTIVE - BAND_H;
    localparam integer TEXT_Y0  = BAND_Y0 + (BAND_H - 32) / 2;   // 32 px tall text
    localparam integer STAT_X0  = H_ACTIVE - 8 * 16 - 8;         // 8 glyphs, 16 px each
    localparam integer STAT_Y0  = 8;

    // ------------------------------------------------------------------
    // coordinates from the incoming stream
    // ------------------------------------------------------------------
    wire [9:0] x, y;
    wire       frame_tick;
    video_coord #(.H_ACTIVE(H_ACTIVE), .V_ACTIVE(V_ACTIVE)) u_coord(
        .clk(clk), .rst(rst), .vs(vs_i), .de(de_i), .x(x), .y(y), .frame_tick(frame_tick));

    // ------------------------------------------------------------------
    // per-frame state: scroll offset, active message, blink
    // ------------------------------------------------------------------
    reg  [1:0]  msg_sel;           // 0..2 normal rotation, 3 = emergency
    reg  [11:0] scroll;            // pixels scrolled into the text (0 .. total_w)
    reg  [5:0]  blink_cnt;
    reg         blink;
    wire [5:0]  msg_len;
    wire        card_mode = card_valid && !emergency;
    wire [5:0]  len_eff = card_mode ? card_len : msg_len;
    wire [11:0] text_w  = {len_eff, 5'b0};                 // len*32
    wire [11:0] total_w = text_w + H_ACTIVE;               // text enters from the right edge
    reg         emergency_d;
    reg  [1:0]  card_d;             // card page identity of the last frame
    // Only a committed page is identified by its bank; while no card page is
    // shown the bank is irrelevant and must not disturb the ROM ticker.
    wire [1:0]  card_id = {card_valid, card_valid & card_bank};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            msg_sel <= 2'd0; scroll <= 12'd0; blink_cnt <= 6'd0; blink <= 1'b0; emergency_d <= 1'b0;
            card_d <= 2'd0; pass_toggle <= 1'b0;
        end else if (frame_tick) begin
            emergency_d <= emergency;
            card_d <= card_id;
            blink_cnt <= blink_cnt + 6'd1;
            if (blink_cnt == 6'd14) begin blink_cnt <= 6'd0; blink <= ~blink; end   // ~2 Hz
            if (emergency != emergency_d) begin
                msg_sel <= emergency ? 2'd3 : 2'd0;
                scroll  <= emergency ? H_ACTIVE : 12'd0;
            end else if (!emergency && card_d != card_id) begin
                // A new card page (or its withdrawal) starts from the right edge.
                msg_sel <= 2'd0;
                scroll  <= 12'd0;
            end else if (scroll + SCROLL_PX >= total_w) begin
                scroll <= 12'd0;
                if (!emergency) pass_toggle <= ~pass_toggle;
                if (!emergency && !card_mode) msg_sel <= (msg_sel == 2'd2) ? 2'd0 : msg_sel + 2'd1;
            end else begin
                scroll <= scroll + SCROLL_PX;
            end
        end
    end

    // ------------------------------------------------------------------
    // stage 1: where are we, which char
    // ------------------------------------------------------------------
    wire in_band  = (y >= BAND_Y0);
    wire in_text  = (y >= TEXT_Y0) && (y < TEXT_Y0 + 32);
    wire in_stat  = 1'b0 && (y >= STAT_Y0) && (y < STAT_Y0 + 16) && (x >= STAT_X0) && (x < STAT_X0 + 8 * 16);
    wire in_frame = (x < FRAME_W) || (x >= H_ACTIVE - FRAME_W) || (y < FRAME_W) || (y >= V_ACTIVE - FRAME_W);

    // ticker: position of this pixel inside the scrolled text space
    wire [11:0] tpos = {2'b0, x} + scroll;                 // 0 .. 640+total_w
    wire        t_vis = (tpos >= H_ACTIVE) && (tpos < H_ACTIVE + text_w);
    wire [11:0] tp2   = tpos - H_ACTIVE;
    wire [5:0]  t_chr = tp2[10:5];                          // /32
    wire [3:0]  t_col = tp2[4:1];                           // (x%32)/2
    wire [3:0]  t_row = (y - TEXT_Y0) >> 1;

    // status: "n/c Bk C" -> glyphs
    wire [3:0]  s_chr = (x - STAT_X0) >> 4;
    wire [3:0]  s_col = (x - STAT_X0);
    wire [3:0]  s_row = y - STAT_Y0;
    reg  [7:0]  s_glyph;
    always @(*) begin
        case (s_chr)
            4'd0: s_glyph = 8'd1 + {6'b0, img_idx} + 8'd1;   // '1'..'4'
            4'd1: s_glyph = 8'd11;                            // '/'
            4'd2: s_glyph = 8'd1 + {5'b0, img_count};         // '0'..'4'
            4'd3: s_glyph = 8'd0;
            4'd4: s_glyph = 8'd12;                            // 'B'
            4'd5: s_glyph = 8'd1 + {6'b0, bright_lvl} + 8'd1; // '1'..'4'
            4'd6: s_glyph = contrast_hi ? 8'd13 : 8'd0;       // 'C'
            default: s_glyph = 8'd0;
        endcase
    end

    wire [7:0] t_glyph;
    text_rom u_text(.clk(clk), .msg(msg_sel), .pos(t_chr), .glyph(t_glyph), .len(msg_len));
    // Card text read in parallel with text_rom (both one synchronous cycle).
    assign card_text_addr = {card_bank, t_chr};

    // Storage-independent static emergency message, using the same on-chip
    // message/glyph ROMs. Keep the page visible for the entire emergency,
    // including after a BMP loads: raw scan order cannot prove its content.
    wire in_alert=(x>=80 && x<560 && y>=200 && y<232);
    wire [9:0] alert_x=x-10'd80;
    wire [9:0] alert_y=y-10'd200;
    wire [7:0] alert_glyph;
    text_rom u_alert_text(.clk(clk),.msg(2'd3),.pos({1'b0,alert_x[9:5]}),.glyph(alert_glyph),.len());
    reg alert1,alert2;

    reg [23:0] rgb1; reg hs1, vs1, de1;
    reg        band1, text1, stat1, frame1, tvis1, card1;
    reg [3:0]  col1, row1;
    always @(posedge clk) begin
        rgb1 <= rgb_i; hs1 <= hs_i; vs1 <= vs_i; de1 <= de_i;
        band1 <= in_band; text1 <= in_text & t_vis; stat1 <= in_stat; frame1 <= in_frame;
        alert1 <= in_alert;
        card1 <= card_mode && !in_alert && !in_stat;
        col1  <= in_alert ? alert_x[4:1] : in_stat ? s_col : t_col;
        row1  <= in_alert ? alert_y[4:1] : in_stat ? s_row : t_row;
    end
    reg [7:0] s_glyph1;
    always @(posedge clk) s_glyph1 <= s_glyph;

    // ------------------------------------------------------------------
    // stage 2: glyph row fetch
    // ------------------------------------------------------------------
    wire [7:0]  g_sel = alert1 ? alert_glyph : stat1 ? s_glyph1 : t_glyph;
    wire [15:0] g_bits;
    wire [15:0] rom_bits;
    glyph_rom u_glyph(.clk(clk), .glyph(g_sel), .row(row1), .bits(rom_bits));
    assign card_glyph_addr = {card_bank, card_text, row1};
    reg card2;
    assign g_bits = card2 ? card_bits : rom_bits;

    reg [23:0] rgb2; reg hs2, vs2, de2;
    reg        band2, text2, stat2, frame2;
    reg [3:0]  col2;
    always @(posedge clk) begin
        rgb2 <= rgb1; hs2 <= hs1; vs2 <= vs1; de2 <= de1;
        band2 <= band1; text2 <= text1; stat2 <= stat1; frame2 <= frame1; col2 <= col1;
        alert2 <= alert1;
        card2 <= card1;
    end

    // ------------------------------------------------------------------
    // stage 3: compose
    // ------------------------------------------------------------------
    wire pix_on = g_bits[4'd15 - col2];
    // Use a stable dark panel instead of dimming the current image.  This
    // keeps the ticker readable on both bright and dark photographs.
    wire [23:0] band_bg   = emergency ? 24'hA81018 : 24'h211C17;
    wire [23:0] text_fg   = emergency ? 24'hFFFFFF : 24'hF3E9D5;
    wire [23:0] stat_fg   = emergency ? 24'hFF4040 : 24'hFFFF80;
    wire        frame_on  = emergency & blink & frame2;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb_o <= 24'd0; hs_o <= 1'b0; vs_o <= 1'b0; de_o <= 1'b0;
        end else begin
            hs_o <= hs2; vs_o <= vs2; de_o <= de2;
            if ((!osd_en && !emergency) || !de2) rgb_o <= rgb2;
            else if (emergency && alert2 && pix_on) rgb_o <= 24'hFFFFFF;
            else if (frame_on)            rgb_o <= 24'hFF0000;
            else if (stat2 && pix_on)     rgb_o <= stat_fg;
            else if (band2 && text2 && pix_on) rgb_o <= text_fg;
            else if (band2)               rgb_o <= band_bg;
            else if (emergency)           rgb_o <= 24'h28090D;
            else                          rgb_o <= rgb2;
        end
    end
endmodule
