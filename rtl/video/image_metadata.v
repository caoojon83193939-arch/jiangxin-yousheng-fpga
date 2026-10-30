// Prepare source geometry / nominal aspect-fit scale before frame completion.
// Serial subtraction and double-dabble avoid division on a 100 MHz path.
// Format: {width BCD[15:0], height BCD[15:0], scale-percent BCD[19:0]}.
// Scale is rounded to nearest integer; actual target dimensions round down.
module image_metadata(
    input wire clk,rst,start,abort_load,load_done,load_error,
    input wire [10:0] source_width,source_height,
    input wire write_bank,display_bank,
    output wire [51:0] display_meta,
    output wire display_meta_valid
);
    localparam IDLE=0,PRODUCT=1,SELECT=2,DIVIDE=3,DECIMAL=4,READY=5;
    reg [2:0] state;
    reg [10:0] width,height,divisor;
    reg [12:0] cross_w,cross_h;
    reg [16:0] remainder;
    reg [15:0] percent;
    reg [15:0] bin_w,bin_h,bin_p;
    reg [15:0] bcd_w,bcd_h;
    reg [19:0] bcd_p;
    reg [4:0] digit_step;
    reg bank;
    reg [51:0] meta0,meta1;
    reg valid0,valid1;
    assign display_meta=display_bank ? meta1 : meta0;
    assign display_meta_valid=display_bank ? valid1 : valid0;
    function [19:0] add3;
        input [19:0] value;
        integer i;
        begin
            add3=value;
            for(i=0;i<5;i=i+1)
                if(value[i*4+:4]>=5)add3[i*4+:4]=value[i*4+:4]+4'd3;
        end
    endfunction
    wire [19:0] adj_w=add3({4'd0,bcd_w});
    wire [19:0] adj_h=add3({4'd0,bcd_h});
    wire [19:0] adj_p=add3(bcd_p);
    always @(posedge clk or posedge rst)begin
        if(rst)begin
            state<=IDLE;width<=0;height<=0;divisor<=1;cross_w<=0;cross_h<=0;
            remainder<=0;percent<=0;bin_w<=0;bin_h<=0;bin_p<=0;
            bcd_w<=0;bcd_h<=0;bcd_p<=0;digit_step<=0;bank<=0;
            meta0<=0;meta1<=0;valid0<=0;valid1<=0;
        end else if(abort_load || load_error)state<=IDLE;
        else if(start)begin
            width<=source_width;height<=source_height;bank<=write_bank;
            state<=PRODUCT;
        end else case(state)
            // Compare 3*width with 4*height: exact 4:3 fit test, no DSPs.
            PRODUCT:begin cross_w<={2'd0,width}+{1'd0,width,1'b0};cross_h<={height,2'b0};state<=SELECT;end
            SELECT:begin
                percent<=0;
                if(width==0 || height==0)state<=IDLE;
                else begin
                    if(cross_w>=cross_h)begin divisor<=width;remainder<=17'd64000+{7'd0,width[10:1]};end
                    else begin divisor<=height;remainder<=17'd48000+{7'd0,height[10:1]};end
                    state<=DIVIDE;
                end
            end
            DIVIDE:if(remainder>={6'd0,divisor})begin
                remainder<=remainder-{6'd0,divisor};percent<=percent+1'b1;
            end else begin
                bin_w<={5'd0,width};bin_h<={5'd0,height};bin_p<=percent;
                bcd_w<=0;bcd_h<=0;bcd_p<=0;digit_step<=0;state<=DECIMAL;
            end
            DECIMAL:begin
                bcd_w<={adj_w[14:0],bin_w[15]};bin_w<={bin_w[14:0],1'b0};
                bcd_h<={adj_h[14:0],bin_h[15]};bin_h<={bin_h[14:0],1'b0};
                bcd_p<={adj_p[18:0],bin_p[15]};bin_p<={bin_p[14:0],1'b0};
                digit_step<=digit_step+1'b1;
                if(digit_step==15)state<=READY;
            end
            READY:if(load_done)begin
                // The inactive bank only becomes visible when media_ctrl
                // also observes SDRAM completion and commits that bank.
                if(bank)begin meta1<={bcd_w,bcd_h,bcd_p};valid1<=1;end
                else begin meta0<={bcd_w,bcd_h,bcd_p};valid0<=1;end
                state<=IDLE;
            end
            default:state<=IDLE;
        endcase
    end
endmodule
