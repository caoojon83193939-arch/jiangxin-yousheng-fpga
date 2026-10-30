// Transparent 16-band mirrored bars. No panel, grid or channel labels.
// Exactly the same 3-edge RGB/sync latency as M6b's visualizer.
// M10: during an alarm the analyzer keeps running on the alarm tone and the
// bars stay visible in red, independent of SW2 and of the TF card: the siren
// is also shown as a visual signal. This block now follows the alarm OSD;
// its region (y 339..418, x 16..623) is disjoint from the header, ticker,
// alarm text and 8 px border, so normal-mode pixels are unchanged.
module spectrum_overlay(
    input wire clk,rst,emergency,overlay_enable,
    input wire [95:0] levels,
    input wire [23:0] rgb_i,
    input wire hs_i,vs_i,de_i,
    output reg [23:0] rgb_o,
    output reg hs_o,vs_o,de_o
);
    reg vs_d;
    reg [95:0] display_levels;
    reg alarm_frame;
    always @(posedge clk or posedge rst) begin
        if(rst)begin vs_d<=0;display_levels<=0;alarm_frame<=0;end
        else begin
            vs_d<=vs_i;
            // Levels and palette change together at the frame boundary.
            if(vs_d && !vs_i)begin display_levels<=levels;alarm_frame<=emergency;end
        end
    end
    wire [9:0] x,y;
    video_coord coord(.clk(clk),.rst(rst),.vs(vs_i),.de(de_i),.x(x),.y(y),.frame_tick());
    reg [23:0] rgb0,rgb1;
    reg hs0,vs0,de0,hs1,vs1,de1;
    reg [9:0] x1,y1;
    reg enable1;
    always @(posedge clk)begin
        rgb0<=rgb_i;hs0<=hs_i;vs0<=vs_i;de0<=de_i;
        rgb1<=rgb0;hs1<=hs0;vs1<=vs0;de1<=de0;x1<=x;y1<=y;
        enable1<=overlay_enable || alarm_frame;
    end
    // 32 slots across 608 pixels; keep division out of the pixel path.
    reg [4:0] slot_pos;
    reg [4:0] slot;
    reg [4:0] pos1;
    reg [6:0] height1;
    wire [3:0] band=slot[4] ? slot[3:0] : ~slot[3:0];
    wire [5:0] band_level=display_levels[band*6+:6];
    wire music_visible=|display_levels;
    always @(posedge clk)begin
        if(x<16 || x>=624 || !de0)begin slot_pos<=0;slot<=0;end
        else if(slot_pos==18)begin slot_pos<=0;slot<=slot+1'b1;end
        else slot_pos<=slot_pos+1'b1;
        pos1<=slot_pos;
        // Double visual height without changing FFT sensitivity or audio gain.
        // Decorative 4px floor only while some music energy remains. This is
        // a layout cue, not a claim that a silent band has nonzero FFT power.
        height1<=music_visible && band_level<2 ? 7'd4 : {band_level,1'b0};
    end
    wire region=x1>=16 && x1<624 && y1>=339 && y1<419;
    wire [9:0] top_y=10'd419-{3'd0,height1};
    wire within_bar=region && height1!=0 && y1>=top_y && pos1>=5 && pos1<=13;
    // Nine-pixel capsule centered in each odd-width slot (mirror symmetric).
    wire end_row=(y1==top_y || y1==418);
    wire near_end=(y1==top_y+1 || y1==417);
    wire stroke=within_bar && (!end_row || (pos1>=7 && pos1<=11)) &&
                 (!near_end || (pos1>=6 && pos1<=12));
    wire [23:0] ink_music=y1<363 ? 24'hD9FFF0 : y1<395 ? 24'h72DDCE : 24'h4297A7;
    wire [23:0] ink_alarm=y1<363 ? 24'hFFE0C8 : y1<395 ? 24'hFF7A52 : 24'hE0283A;
    wire [23:0] ink=alarm_frame ? ink_alarm : ink_music;
    // 75% ink + 25% original, exact outside strokes. Shift/add only.
    function [7:0] blend;
        input [7:0] bg,fg;
        reg [9:0] total;
        begin total={2'b0,bg}+{2'b0,fg}+{1'b0,fg,1'b0};blend=total[9:2];end
    endfunction
    always @(posedge clk or posedge rst)begin
        if(rst)begin rgb_o<=0;hs_o<=0;vs_o<=0;de_o<=0;end
        else begin
            hs_o<=hs1;vs_o<=vs1;de_o<=de1;
            if(!enable1 || !de1 || !stroke)rgb_o<=rgb1;
            else rgb_o<={blend(rgb1[23:16],ink[23:16]),blend(rgb1[15:8],ink[15:8]),blend(rgb1[7:0],ink[7:0])};
        end
    end
endmodule
