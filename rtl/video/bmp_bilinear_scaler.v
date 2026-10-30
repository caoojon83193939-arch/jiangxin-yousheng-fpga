// M12 streaming aspect-fit scaler with bilinear interpolation (drop-in for
// bmp_aspect_scaler: same ports plus `smooth`, same fit/letterbox geometry,
// same valid/ready discipline, output in BMP file row order).
//
// Mapping (center aligned, per axis, destination index i in 0..d-1):
//     q = floor(((2i+1)*s - d) * 32 / d)       (0 if negative)
//     idx = q >> 6, frac = q & 63               source position idx + frac/64
//     idx >= s-1  ->  idx = s-1, frac = 0       (edge clamp)
// The mapping is mirror symmetric, so file-order rows of bottom-up BMPs use
// the same formula with the mirrored letterbox margin.
// Weight w (0..64): smooth ? frac : (frac >= 32 ? 64 : 0), i.e. SW4 off gives
// center-aligned nearest neighbour through the same datapath.
// Pixel = lerp(lerp(p00,p01,wx), lerp(p10,p11,wx), wy),
//     lerp(a,b,w) = a + (((b-a)*w + 32) >>> 6)   (exact, stays within [a,b])
//
// Two row RAMs hold source rows by parity: row r lives in ram[r&1], so the
// two rows needed (sy, sy+1) are always in different RAMs and a row is only
// overwritten by row r+2, which is requested only after row r is finished.
// The same divider also computes the fit size (floor(n/d) = floor(32n/d)>>5),
// so there is one 27-bit subtractor in the block.
// Positions come from one 17-step restoring divider; ~30 cycles per active
// output pixel (about 0.1 s per 640x480 frame at 100 MHz), far below the SD
// source rate, so the datapath stays small and shallow.
module bmp_bilinear_scaler #(
    parameter integer OUT_WIDTH=640, OUT_HEIGHT=480, MAX_WIDTH=1024
)(
    input wire clk,rst,abort_load,start,topdown,smooth,
    input wire [10:0] source_width,source_height,
    input wire in_valid,
    input wire [23:0] in_data,
    output wire in_ready,
    output wire out_valid,
    output wire [23:0] out_data,
    input wire out_ready,
    output reg busy,done
);
    localparam integer AW=$clog2(MAX_WIDTH);
    localparam [4:0] IDLE=0,PRODUCTS=1,FIT=2,FIT_DONE=3,GEOMETRY=4,ROW_BEGIN=5,
        DIV=6,YPOS=7,INGEST=8,PIX_BEGIN=9,XPOS=10,FETCH0=11,FETCH1=12,FETCH2=13,
        H0M=14,H0A=15,H1M=16,H1A=17,VM=18,VA=19,SEND=20,DRAIN=21,YCHK=22;
    reg [4:0] state,div_ret;
    reg [10:0] sw,sh,sw_m1,sh_m1,dw,dh,x0,x1,y0,y1,x,y,in_x,in_row;
    reg [20:0] width_cross,height_cross;
    reg fit_width,td,mode_smooth;
    // position accumulators: (2i+1)*s - d, signed
    reg signed [22:0] numx,numy;
    // sequential divider
    reg [26:0] div_r,div_d;
    reg [16:0] div_q;
    reg [4:0] div_n;
    // current output pixel geometry
    reg [10:0] ry0,ry1,sx1;
    reg [6:0] wx,wy;
    // row RAMs and read port
    reg [23:0] ram0[0:MAX_WIDTH-1];
    reg [23:0] ram1[0:MAX_WIDTH-1];
    reg [AW-1:0] rd_addr;
    reg [23:0] q0,q1;
    reg [23:0] pa0,pb0,pb1,ra,la,lb,out_pix;
    reg [6:0] lw;
    reg signed [16:0] prod_r,prod_g,prod_b;

    wire active_row=y>=y0 && y<y1;
    wire active_pixel=active_row && x>=x0 && x<x1;
    assign in_ready=busy && !abort_load && (state==INGEST || state==DRAIN);
    assign out_valid=busy && !abort_load && state==SEND;
    assign out_data=out_pix;

    // divider result -> index / fraction with edge clamp
    wire [10:0] q_idx=div_q[16:6];
    wire [5:0] q_frac=div_q[5:0];
    function [6:0] weight;
        input [5:0] f;
        input s;
        weight=s ? {1'b0,f} : (f[5] ? 7'd64 : 7'd0);
    endfunction

    // lerp datapath: product registered, then add
    function signed [16:0] mul;
        input [7:0] a,b;
        input [6:0] w;
        mul=($signed({1'b0,b})-$signed({1'b0,a}))*$signed({1'b0,w});
    endfunction
    function [7:0] add;
        input [7:0] a;
        input signed [16:0] p;
        reg signed [16:0] t;
        reg signed [9:0] s;
        begin t=(p+17'sd32)>>>6; s=$signed({2'b0,a})+t[9:0]; add=s[7:0]; end
    endfunction
    wire [23:0] lerp_out={add(la[23:16],prod_r),add(la[15:8],prod_g),add(la[7:0],prod_b)};

    // Row mapping for the two captured words.
    wire [23:0] qa=ry0[0] ? q1 : q0;
    wire [23:0] qb=ry1==ry0 ? qa : (ry1[0] ? q1 : q0);

    always @(posedge clk)begin
        if(in_valid && in_ready && state==INGEST && in_row>=ry0)begin
            if(in_row[0])ram1[in_x[AW-1:0]]<=in_data;
            else ram0[in_x[AW-1:0]]<=in_data;
        end
        q0<=ram0[rd_addr];
        q1<=ram1[rd_addr];
    end

    task start_div;
        input signed [22:0] num;
        input [10:0] d;
        input [4:0] ret;
        begin
            div_r<=num[22] ? 27'd0 : {num[21:0],5'b0};
            div_d<={d,16'b0};
            div_q<=0;
            div_n<=num[22] ? 5'd0 : 5'd17;
            div_ret<=ret;
            state<=DIV;
        end
    endtask

    always @(posedge clk or posedge rst)begin
        if(rst)begin
            state<=IDLE;div_ret<=IDLE;busy<=0;done<=0;sw<=0;sh<=0;dw<=0;dh<=0;
            sw_m1<=0;sh_m1<=0;x0<=0;x1<=0;y0<=0;y1<=0;x<=0;y<=0;in_x<=0;in_row<=0;
            width_cross<=0;height_cross<=0;
            fit_width<=0;td<=0;mode_smooth<=0;numx<=0;numy<=0;
            div_r<=0;div_d<=0;div_q<=0;div_n<=0;
            ry0<=0;ry1<=0;sx1<=0;wx<=0;wy<=0;rd_addr<=0;
            pa0<=0;pb0<=0;pb1<=0;ra<=0;la<=0;lb<=0;lw<=0;out_pix<=0;
            prod_r<=0;prod_g<=0;prod_b<=0;
        end else begin
            done<=0;
            case(state)
                IDLE:if(start)begin
                    busy<=1;sw<=source_width;sh<=source_height;td<=topdown;mode_smooth<=smooth;
                    in_x<=0;in_row<=0;x<=0;y<=0;
                    state<=PRODUCTS;
                end
                PRODUCTS:begin
                    width_cross<=sw*OUT_HEIGHT;height_cross<=sh*OUT_WIDTH;
                    sw_m1<=sw-1'b1;sh_m1<=sh-1'b1;
                    state<=FIT;
                end
                // Fit size = floor(cross/side) <= 640, via the shared divider.
                FIT:if(width_cross>=height_cross)begin
                    fit_width<=1;dw<=OUT_WIDTH;start_div({2'b0,height_cross},sw,FIT_DONE);
                end else begin
                    fit_width<=0;dh<=OUT_HEIGHT;start_div({2'b0,width_cross},sh,FIT_DONE);
                end
                FIT_DONE:begin
                    if(fit_width)dh<=div_q[16:5]==0 ? 11'd1 : div_q[15:5];
                    else dw<=div_q[16:5]==0 ? 11'd1 : div_q[15:5];
                    state<=GEOMETRY;
                end
                GEOMETRY:begin
                    x0<=(OUT_WIDTH-dw)/2;x1<=(OUT_WIDTH-dw)/2+dw;
                    // Mirrored letterbox margins for bottom-up files.
                    y0<=td ? (OUT_HEIGHT-dh)/2 : OUT_HEIGHT-dh-(OUT_HEIGHT-dh)/2;
                    y1<=td ? (OUT_HEIGHT-dh)/2+dh : OUT_HEIGHT-(OUT_HEIGHT-dh)/2;
                    numy<=$signed({12'd0,sh})-$signed({12'd0,dh});
                    y<=0;state<=ROW_BEGIN;
                end
                ROW_BEGIN:begin
                    x<=0;
                    numx<=$signed({12'd0,sw})-$signed({12'd0,dw});
                    if(active_row)start_div(numy,dh,YPOS);
                    else state<=PIX_BEGIN;
                end
                DIV:if(div_n!=0)begin
                    if(div_r>=div_d)begin div_r<=div_r-div_d;div_q<={div_q[15:0],1'b1};end
                    else div_q<={div_q[15:0],1'b0};
                    div_d<={1'b0,div_d[26:1]};
                    div_n<=div_n-1'b1;
                end else state<=div_ret;
                YPOS:begin
                    if(q_idx>=sh_m1)begin ry0<=sh_m1;ry1<=sh_m1;wy<=0;end
                    else begin ry0<=q_idx;ry1<=q_idx+1'b1;wy<=weight(q_frac,mode_smooth);end
                    state<=YCHK;
                end
                // INGEST (in_ready high) is entered only when a row is really
                // needed: the BMP source samples in_ready and delivers the
                // pixel one cycle later, so ready must not blink.
                YCHK:state<=in_row>ry1 ? PIX_BEGIN : INGEST;
                // Rows 0..in_row-1 have been received. Leave only on an
                // accepted pixel, never with a pixel in flight.
                INGEST:if(in_valid && in_ready)begin
                    if(in_x==sw_m1)begin
                        in_x<=0;in_row<=in_row+1'b1;
                        if(in_row==ry1)state<=PIX_BEGIN;
                    end else in_x<=in_x+1'b1;
                end
                PIX_BEGIN:if(active_pixel)start_div(numx,dw,XPOS);
                else begin out_pix<=24'd0;state<=SEND;end
                XPOS:begin
                    if(q_idx>=sw_m1)begin
                        sx1<=sw_m1;wx<=0;rd_addr<=sw_m1[AW-1:0];
                    end else begin
                        sx1<=q_idx+1'b1;wx<=weight(q_frac,mode_smooth);
                        rd_addr<=q_idx[AW-1:0];
                    end
                    state<=FETCH0;
                end
                FETCH0:begin rd_addr<=sx1[AW-1:0];state<=FETCH1;end
                FETCH1:begin pa0<=qa;pb0<=qb;state<=FETCH2;end
                FETCH2:begin pb1<=qb;la<=pa0;lb<=qa;lw<=wx;state<=H0M;end
                H0M:begin
                    prod_r<=mul(la[23:16],lb[23:16],lw);prod_g<=mul(la[15:8],lb[15:8],lw);
                    prod_b<=mul(la[7:0],lb[7:0],lw);state<=H0A;
                end
                H0A:begin ra<=lerp_out;la<=pb0;lb<=pb1;lw<=wx;state<=H1M;end
                H1M:begin
                    prod_r<=mul(la[23:16],lb[23:16],lw);prod_g<=mul(la[15:8],lb[15:8],lw);
                    prod_b<=mul(la[7:0],lb[7:0],lw);state<=H1A;
                end
                H1A:begin la<=ra;lb<=lerp_out;lw<=wy;state<=VM;end
                VM:begin
                    prod_r<=mul(la[23:16],lb[23:16],lw);prod_g<=mul(la[15:8],lb[15:8],lw);
                    prod_b<=mul(la[7:0],lb[7:0],lw);state<=VA;
                end
                VA:begin out_pix<=lerp_out;state<=SEND;end
                SEND:if(out_ready)begin
                    if(active_pixel)numx<=numx+$signed({11'd0,sw,1'b0});
                    if(x==OUT_WIDTH-1)begin
                        if(active_row)numy<=numy+$signed({11'd0,sh,1'b0});
                        if(y==OUT_HEIGHT-1)state<=DRAIN;
                        else begin y<=y+1'b1;state<=ROW_BEGIN;end
                    end else begin x<=x+1'b1;state<=PIX_BEGIN;end
                end
                DRAIN:begin
                    if(in_row==sh)begin busy<=0;done<=1;state<=IDLE;end
                    else if(in_valid && in_ready)begin
                        if(in_x==sw_m1)begin in_x<=0;in_row<=in_row+1'b1;end
                        else in_x<=in_x+1'b1;
                    end
                end
                default:state<=IDLE;
            endcase
            if(abort_load)begin state<=IDLE;busy<=0;done<=0;end
        end
    end
endmodule
