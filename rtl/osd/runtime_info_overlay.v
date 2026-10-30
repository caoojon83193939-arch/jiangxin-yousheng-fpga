// Compact gallery header, temporary controls and geometry; three-cycle RGB/sync.
// M10 adds a truthful status field (header columns 16..24), priority chosen by
// the caller: 1 LOADING, 2 NO IMAGE, 3 GUIDE, 4 MUSIC, 5 TEST TONE, 6 MUTED.
// The field never re-opens the parameter line (it is not part of settings).
// M12: parameter line columns 34..36 show the scaling mode, BIL or NN; a mode
// change is a setting and re-opens the line.
// M13 display modes. maint=1 (SW2 up, maintenance): both rows stay on screen.
// maint=0 (exhibition, the product default): visitors see only the poster,
// subtitles and spectrum; the panel appears for NOTICE_FRAMES after a setting
// the staff changed (brightness, contrast, scaling mode, tour on/off), and
// the header row alone while no image is displayed (LOADING / NO IMAGE).
// Page changes and image geometry no longer open the panel.
// M14: header columns 34..36 show the tour's minimum dwell while touring
// (30S / 60S; blank = content-driven); a dwell change is a setting.
// M15 (ROW3=1): the maintenance view adds a third row, y 52..72:
//   UP hh:mm:ss  VOL v/8  RCV nnn
// run time since reset, volume level and the SD auto-recovery count. Rows
// 1-2 are unchanged; ROW3=0 is exactly M14. A volume change is a setting.
module runtime_info_overlay #(parameter integer NOTICE_FRAMES=180, parameter ROW3=0)(
    input wire clk,rst,maint,emergency,valid,
    input wire [51:0] metadata,
    input wire [1:0] image_role,brightness,
    input wire [2:0] image_count,
    input wire contrast,auto_play,
    input wire [2:0] field,
    input wire smooth,
    input wire [1:0] dwell,
    input wire [23:0] uptime,          // BCD {h1,h0,m1,m0,s1,s0}
    input wire [3:0] volume,
    input wire [7:0] recoveries,
    input wire [23:0] rgb_i,
    input wire hs_i,vs_i,de_i,
    output reg [23:0] rgb_o,
    output reg hs_o,vs_o,de_o
);
    wire [9:0] x,y;
    wire frame_tick;
    localparam CW=$clog2(NOTICE_FRAMES+1);
    reg [CW-1:0] notice_left;
    reg [10:0] previous;
    reg primed;
    wire [3:0] vol_setting = ROW3 ? volume : 4'd0;
    wire [10:0] settings={brightness,contrast,smooth,auto_play,dwell,vol_setting};
    always @(posedge clk or posedge rst)begin
        if(rst)begin notice_left<=0;previous<=0;primed<=0;end
        else if(frame_tick)begin
            previous<=settings;primed<=1;
            // The reset/start-up values are not a change made by the staff.
            if(emergency)notice_left<=0;
            else if(primed && previous!=settings)notice_left<=NOTICE_FRAMES;
            else if(notice_left!=0)notice_left<=notice_left-1'b1;
        end
    end
    wire details=maint || notice_left!=0;
    wire header=details || !valid;
    video_coord coords(.clk(clk),.rst(rst),.vs(vs_i),.de(de_i),.x(x),.y(y),.frame_tick(frame_tick));
    reg [23:0] rgb0,rgb1;
    reg hs0,vs0,de0,hs1,vs1,de1;
    reg panel1,text1,line1;
    wire row3=ROW3 && maint;
    wire panel=(x>=16 && x<624 && y>=8 && ((header && y<32) || (details && y>=32 && y<52) || (row3 && y>=52 && y<72)));
    wire second=y>=32;
    wire third=ROW3 && y>=52;
    wire [9:0] tx=x-10'd24;
    wire [9:0] ty=third ? y-10'd52 : second ? y-10'd32 : y-10'd12;
    wire text_area=(x>=24 && x<616 && ((y>=12 && y<28)||(details && y>=32 && y<48)||(row3 && y>=52 && y<68)));
    // Recovery count as three decimal digits (0..255).
    wire [7:0] rc=recoveries;
    wire [3:0] rc_h = rc>=200 ? 4'd2 : rc>=100 ? 4'd1 : 4'd0;
    wire [7:0] rc_r = rc - (rc_h==2 ? 8'd200 : rc_h==1 ? 8'd100 : 8'd0);
    wire [3:0] rc_t = rc_r>=90?4'd9:rc_r>=80?4'd8:rc_r>=70?4'd7:rc_r>=60?4'd6:rc_r>=50?4'd5:
                      rc_r>=40?4'd4:rc_r>=30?4'd3:rc_r>=20?4'd2:rc_r>=10?4'd1:4'd0;
    wire [7:0] rc_o8 = rc_r - {rc_t,3'b0} - {rc_t,1'b0};
    wire [3:0] rc_o = rc_o8[3:0];
    wire [5:0] char_index=tx[9:4];
    reg [7:0] ascii;
    reg [11:0] bcd;
    wire [15:0] w=metadata[51:36],h=metadata[35:20];
    wire [19:0] scale=metadata[19:0];
    function [7:0] digit;
        input [3:0] n;
        begin digit=8'd48+{4'd0,n};end
    endfunction
    // 9-character status words, column 0..8 -> header columns 16..24.
    function [7:0] field_char;
        input [2:0] code;
        input [3:0] col;
        reg [71:0] word;
        begin
            case(code)
                1:word="LOADING  ";2:word="NO IMAGE ";3:word="GUIDE    ";
                4:word="MUSIC    ";5:word="TEST TONE";6:word="MUTED    ";
                default:word="         ";
            endcase
            field_char=word[71-8*col-:8];
        end
    endfunction
    wire [5:0] field_col=char_index-6'd16;
    always @(*)begin
        case(brightness)
            0:bcd=12'h060;1:bcd=12'h080;2:bcd=12'h100;default:bcd=12'h120;
        endcase
        ascii=" ";
        if(third)case(char_index)
            0:ascii="U";1:ascii="P";
            3:ascii=digit(uptime[23:20]);4:ascii=digit(uptime[19:16]);5:ascii=":";
            6:ascii=digit(uptime[15:12]);7:ascii=digit(uptime[11:8]);8:ascii=":";
            9:ascii=digit(uptime[7:4]);10:ascii=digit(uptime[3:0]);
            13:ascii="V";14:ascii="O";15:ascii="L";17:ascii=digit(volume);18:ascii="/";19:ascii="8";
            22:ascii="R";23:ascii="C";24:ascii="V";
            26:ascii=rc_h==0 ? " " : digit(rc_h);
            27:ascii=(rc_h==0 && rc_t==0) ? " " : digit(rc_t);
            28:ascii=digit(rc_o);
            default:ascii=" ";
        endcase
        else if(!second)case(char_index)
            0:ascii="H";1:ascii="E";2:ascii="R";3:ascii="I";
            4:ascii="T";5:ascii="A";6:ascii="G";7:ascii="E";
            10:ascii="0";11:ascii=valid ? digit({2'b0,image_role}+4'd1) : "-";
            12:ascii="/";13:ascii="0";14:ascii=digit({1'b0,image_count});
            16,17,18,19,20,21,22,23,24:ascii=field_char(field,field_col[3:0]);
            28:ascii=auto_play ? "A":"M";29:ascii=auto_play ? "U":"A";
            30:ascii=auto_play ? "T":"N";31:ascii=auto_play ? "O":"U";
            32:ascii=auto_play ? " ":"A";33:ascii=auto_play ? " ":"L";
            34:ascii=auto_play && dwell==2'd1 ? "3" : auto_play && dwell==2'd2 ? "6" : " ";
            35:ascii=auto_play && dwell!=2'd0 ? "0" : " ";
            36:ascii=auto_play && dwell!=2'd0 ? "S" : " ";
            default:ascii=" ";
        endcase
        else case(char_index)
            0:ascii="B";1:ascii=digit(bcd[11:8]);2:ascii=digit(bcd[7:4]);3:ascii=digit(bcd[3:0]);4:ascii="%";
            6:ascii="C";7:ascii="1";8:ascii=contrast?"2":"0";9:ascii=contrast?"5":"0";10:ascii="%";
            12:ascii=!valid?"-":w[15:12]==0?" ":digit(w[15:12]);
            13:ascii=!valid?"-":w[15:8]==0?" ":digit(w[11:8]);
            14:ascii=!valid?"-":w[15:4]==0?" ":digit(w[7:4]);
            15:ascii=valid?digit(w[3:0]):"-";16:ascii="x";
            17:ascii=!valid?"-":h[15:12]==0?" ":digit(h[15:12]);
            18:ascii=!valid?"-":h[15:8]==0?" ":digit(h[11:8]);
            19:ascii=!valid?"-":h[15:4]==0?" ":digit(h[7:4]);
            20:ascii=valid?digit(h[3:0]):"-";
            23:ascii="F";24:ascii="I";25:ascii="T";
            27:ascii=!valid?"-":scale[19:16]==0?" ":digit(scale[19:16]);
            28:ascii=!valid?"-":scale[19:12]==0?" ":digit(scale[15:12]);
            29:ascii=!valid?"-":scale[19:8]==0?" ":digit(scale[11:8]);
            30:ascii=!valid?"-":scale[19:4]==0?" ":digit(scale[7:4]);
            31:ascii=valid?digit(scale[3:0]):"-";32:ascii="%";
            34:ascii=smooth?"B":"N";35:ascii=smooth?"I":"N";36:ascii=smooth?"L":" ";
            default:ascii=" ";
        endcase
    end
    wire pixel1;
    info_font font(.clk(clk),.ascii(ascii[6:0]),.row(ty[3:1]),.col(tx[3:1]),.pixel(pixel1));
    wire [23:0] dimmed={2'b0,rgb1[23:18],2'b0,rgb1[15:10],2'b0,rgb1[7:2]};
    always @(posedge clk or posedge rst)begin
        if(rst)begin
            rgb0<=0;rgb1<=0;rgb_o<=0;hs0<=0;vs0<=0;de0<=0;
            hs1<=0;vs1<=0;de1<=0;hs_o<=0;vs_o<=0;de_o<=0;
            panel1<=0;text1<=0;line1<=0;
        end else begin
            rgb0<=rgb_i;hs0<=hs_i;vs0<=vs_i;de0<=de_i;
            rgb1<=rgb0;hs1<=hs0;vs1<=vs0;de1<=de0;
            panel1<=panel && !emergency;
            text1<=text_area;line1<=second;
            hs_o<=hs1;vs_o<=vs1;de_o<=de1;
            if(!de1 || !panel1 || emergency)rgb_o<=rgb1;
            else if(pixel1 && text1)rgb_o<=line1 ? 24'hF3E9D5 : 24'hA9CFBA;
            else rgb_o<=dimmed;
        end
    end
endmodule
