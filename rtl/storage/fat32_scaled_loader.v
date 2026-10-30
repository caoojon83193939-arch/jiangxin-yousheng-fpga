// M12: bmp_bilinear_scaler; `smooth` (SW4) selects bilinear or center nearest,
// latched by the scaler at each image start.
// Preserve the M7 controller contract: one completion means all source bytes
// were accepted AND the complete scaled frame was handed to the write FIFO.
module fat32_scaled_loader #(
    parameter integer IMG_WIDTH=640,IMG_HEIGHT=480,
    parameter integer MAX_WIDTH=1024,MAX_HEIGHT=1024,MAX_DIR_CLUSTERS=128
)(
    input wire clk,rst,sd_init_done,scan_start,
    output wire scan_busy,hit_valid,scan_done,
    output wire [31:0] hit_sector,
    input wire load_start,load_abort,smooth,
    input wire [31:0] load_sector,
    output reg load_busy,load_done,
    output wire load_error,bmp_topdown,write_req,
    input wire write_req_ack,write_ready,
    output wire pix_wr_en,
    output wire [23:0] pix_data,
    output wire sd_sec_read,
    output wire [31:0] sd_sec_read_addr,
    input wire [7:0] sd_sec_read_data,
    input wire sd_sec_read_data_valid,sd_sec_read_end,
    output wire [3:0] state_code,error_code,missing_mask,bad_mask,
    output wire source_start,
    output wire [10:0] source_width,source_height,
    output wire [1:0] error_role
);
    wire raw_busy,raw_done,raw_valid,raw_ready,scaled_valid,scale_busy,scale_done;
    wire [23:0] raw_pixel;
    reg source_finished,scale_finished;
    assign pix_wr_en=scaled_valid && write_ready && !load_abort;
    fat32_bmp_source #(.MAX_WIDTH(MAX_WIDTH),.MAX_HEIGHT(MAX_HEIGHT),.MAX_DIR_CLUSTERS(MAX_DIR_CLUSTERS)) source(
        .clk(clk),.rst(rst),.sd_init_done(sd_init_done),.scan_start(scan_start),
        .scan_busy(scan_busy),.hit_valid(hit_valid),.scan_done(scan_done),.hit_sector(hit_sector),
        .load_start(load_start),.load_abort(load_abort),.load_sector(load_sector),
        .load_busy(raw_busy),.load_done(raw_done),.load_error(load_error),.bmp_topdown(bmp_topdown),
        .write_req(write_req),.write_req_ack(write_req_ack),.pix_ready(raw_ready),
        .source_start(source_start),.source_width(source_width),.source_height(source_height),
        .pix_wr_en(raw_valid),.pix_data(raw_pixel),.sd_sec_read(sd_sec_read),.sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data),.sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),.state_code(state_code),.error_code(error_code),
        .missing_mask(missing_mask),.bad_mask(bad_mask),.error_role(error_role));
    bmp_bilinear_scaler #(.OUT_WIDTH(IMG_WIDTH),.OUT_HEIGHT(IMG_HEIGHT),.MAX_WIDTH(MAX_WIDTH)) scaler(
        .clk(clk),.rst(rst),.abort_load(load_abort || load_error),.start(source_start),.smooth(smooth),
        .topdown(bmp_topdown),.source_width(source_width),.source_height(source_height),
        .in_valid(raw_valid),.in_data(raw_pixel),.in_ready(raw_ready),
        .out_valid(scaled_valid),.out_data(pix_data),.out_ready(write_ready),.busy(scale_busy),.done(scale_done));
    always @(posedge clk or posedge rst)begin
        if(rst)begin load_busy<=0;load_done<=0;source_finished<=0;scale_finished<=0;end
        else begin
            load_done<=0;
            if(load_start)begin load_busy<=1;source_finished<=0;scale_finished<=0;end
            else if(load_busy)begin
                if(raw_done)source_finished<=1;
                if(scale_done)scale_finished<=1;
                if((source_finished || raw_done) && (scale_finished || scale_done))begin
                    load_busy<=0;load_done<=1;
                end
                if(load_error)begin load_busy<=0;load_done<=0;end
            end
            if(load_abort)begin load_busy<=0;load_done<=0;source_finished<=0;scale_finished<=0;end
        end
    end
endmodule
