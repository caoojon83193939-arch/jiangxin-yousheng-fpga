// M14b retiming: the per-byte pixel decisions in PIX_BYTE (32-bit
// file_byte>=pixel_offset, row/column bounds, end-of-row) are registered in
// PIX_FETCH, which always precedes PIX_BYTE and during which file_byte, row
// and col_byte are stable; while PIX_BYTE waits for pix_ready they do not
// change either. Behaviour is identical cycle for cycle.
// Read-only FAT32 short-name BMP catalog. Tokens 0..3 mean IMG01..IMG04,
// never physical LBAs. Entire allocation chains are checked before publishing.
// Abort discards the client transaction; the shared arbiter drains its sector.
module fat32_bmp_source #(
    parameter integer MAX_WIDTH=1024, MAX_HEIGHT=1024,
    parameter integer MAX_DIR_CLUSTERS=128
)(
    input wire clk,rst,sd_init_done,scan_start,
    output reg scan_busy,hit_valid,scan_done,
    output reg [31:0] hit_sector,
    input wire load_start,load_abort,
    input wire [31:0] load_sector,
    output reg load_busy,load_done,load_error,bmp_topdown,write_req,
    input wire write_req_ack,
    input wire pix_ready,
    output reg source_start,
    output wire [10:0] source_width,source_height,
    output reg pix_wr_en,
    output reg [23:0] pix_data,
    output reg sd_sec_read,
    output reg [31:0] sd_sec_read_addr,
    input wire [7:0] sd_sec_read_data,
    input wire sd_sec_read_data_valid,sd_sec_read_end,
    output wire [3:0] state_code,
    output reg [3:0] error_code,missing_mask,bad_mask,
    output reg [1:0] error_role
);
    localparam PIXEL_BYTES=((MAX_WIDTH*3+3)&~3)*MAX_HEIGHT;
    reg [31:0] abs_height;
    reg [12:0] row_bytes,stride;
    reg [23:0] payload_bytes;
    localparam CLUSTER_BITS=$clog2((PIXEL_BYTES+4096+511)/512+1);
    localparam IDLE=0,WAIT_SEC=1,EVAL=2,BOOT_EVAL=3,LAYOUT=4,LIMITS=5,
        ROOT=6,DIR_NEXT=7,CHECK_FILE=8,HEADER_EVAL=9,CHAIN_BEGIN=10,
        FAT_EVAL=11,NEXT_FILE=12,PUBLISH=13,MAP_SHIFT=14,MAP_ADD=15,
        MAP_REQ=16,WR_REQ=17,WR_SETTLE=18,PIX_NEXT=19,FAT_REQ=20,
        LAYOUT2=21,LIMITS2=22,CHAIN_COUNT=23,GEOM_WAIT=24,GEOM_EVAL=25,
        CHECK_WAIT=26,CHECK_META=27,PIX_FETCH=28,PIX_BYTE=29;
    localparam BOOT=0,DIR=1,HEADER=2,FAT=3,PIX=4;
    localparam FAT_DIR=0,FAT_CHECK=1,FAT_DATA=2;
    reg [4:0] state;
    reg [2:0] kind,map_kind;
    reg [1:0] fat_purpose,role;
    reg [2:0] cursor;
    reg [9:0] byte_idx;
    reg [31:0] first[0:3],sizes[0:3];
    reg [3:0] found,valid_files,duplicates;
    reg [31:0] partition,selected_partition,selected_size;
    reg partition_tried,partition_found;
    reg [7:0] part_type;
    reg [31:0] part_start,part_size;
    reg [15:0] bps,reserved,root_entries,fat16,total16,flags,version,signature;
    reg [7:0] fat_count,bpb_spc,spc,sector_in_cluster;
    reg [2:0] spc_shift;
    reg [31:0] total32,fat_size,root_cluster,max_cluster;
    reg [32:0] fat_span,overhead,volume_end,fat_lba,data_lba,partition_end;
    reg [31:0] data_sectors,cluster_count;
    reg [31:0] current_cluster,map_offset,cluster_sector,file_size;
    // Valid files are capped at PIXEL_BYTES+4096, so this counter need not
    // carry all 32 directory-size bits. Compute ceil(size/cluster_bytes) in
    // three stages while the header sector is arriving.
    reg [CLUSTER_BITS-1:0] clusters_left,file_cluster_quotient_r,file_clusters_r;
    reg [31:0] file_last_byte_r;
    reg [15:0] dir_hops;
    reg [87:0] dir_name;
    reg [7:0] dir_attr;
    reg [15:0] dir_hi,dir_lo;
    reg [23:0] dir_size_lo;
    reg dir_end;
    reg [31:0] fat_value;
    reg [8:0] fat_offset;
    reg [15:0] magic,planes,bpp;
    reg [31:0] bmp_size,pixel_offset,dib_size,width,height,compression,image_size;
    reg [32:0] pixel_end;
    reg [31:0] file_byte;
    reg [10:0] row;
    reg [12:0] col_byte;
    reg [8:0] sector_pos;
    reg [7:0] sector_ram[0:511];
    reg [7:0] pixel_byte;
    reg [1:0] phase;
    reg [7:0] blue,green,settle;
    reg bpb_ok_r,header_ok_r,next_ok_r,sector_more_r;
    reg geometry_ok_r,metadata_ok_r,last_cluster_r,more_clusters_r;
    reg [31:0] catalog_first;
    reg [32:0] fat_entries_needed;
    (* keep = "true" *) reg pixel_sector;
    wire local_rst;
    reset_sync u_local_reset(.clk(clk),.async_rst(rst),.rst(local_rst));
    reg [7:0] din;
    reg vin,eos;
    reg pix_in_r,col_in_r,col_last_r;
    always @(posedge clk) if(state==PIX_FETCH) begin
        pix_in_r<=file_byte>=pixel_offset && row<source_height;
        col_in_r<=col_byte<row_bytes;
        col_last_r<=col_byte==stride-1;
    end
    always @(posedge clk or posedge local_rst)
        if(local_rst)begin din<=0;vin<=0;eos<=0;end
        else begin din<=sd_sec_read_data;vin<=sd_sec_read_data_valid;eos<=sd_sec_read_end;end
    assign source_width=width[10:0];
    assign source_height=abs_height[10:0];
    always @(posedge clk) begin
        if(state==WAIT_SEC && pixel_sector && vin) sector_ram[byte_idx[8:0]]<=din;
        pixel_byte<=sector_ram[sector_pos];
    end
    wire [31:0] next_cluster={4'd0,fat_value[27:0]};
    reg eoc_r; // FAT32 EOC 0x0ffffff8..0x0fffffff: upper 25 bits all ones.
    wire bpb_ok=bps==512 && bpb_spc!=0 && (bpb_spc&(bpb_spc-1'b1))==0 &&
        reserved!=0 && (fat_count==1 || fat_count==2) && root_entries==0 &&
        fat16==0 && total16==0 && fat_size!=0 && version==0 &&
        root_cluster>=2 && root_cluster<32'h0ffffff0 &&
        (!flags[7] || flags[3:0]<fat_count);
    wire header_ok=magic==16'h4d42 && dib_size==40 && width>=1 && width<=MAX_WIDTH &&
        abs_height>=1 && abs_height<=MAX_HEIGHT && planes==1 && bpp==24 &&
        compression==0 && pixel_offset>=54 && pixel_offset<=4096 &&
        !pixel_end[32] && bmp_size>=pixel_end[31:0] && bmp_size<=file_size &&
        (image_size==0 || image_size==payload_bytes);
    always @(posedge clk or posedge local_rst)begin
        if(local_rst)begin eoc_r<=0;bpb_ok_r<=0;header_ok_r<=0;next_ok_r<=0;sector_more_r<=0;pixel_end<=0;abs_height<=0;row_bytes<=0;stride<=0;payload_bytes<=0;
            geometry_ok_r<=0;metadata_ok_r<=0;last_cluster_r<=0;more_clusters_r<=0;
            file_last_byte_r<=0;file_cluster_quotient_r<=0;file_clusters_r<=0;end
        else begin
            eoc_r<=&fat_value[27:3];
            bpb_ok_r<=bpb_ok;header_ok_r<=header_ok;
            file_last_byte_r<=file_size-1;
            file_cluster_quotient_r<=file_last_byte_r>>(9+spc_shift);
            file_clusters_r<=file_cluster_quotient_r+1'b1;
            last_cluster_r<=clusters_left==1;more_clusters_r<=clusters_left>1;
            metadata_ok_r<=catalog_first>=2 && catalog_first<=max_cluster &&
                file_size>=54 && file_size<=PIXEL_BYTES+4096;
            geometry_ok_r<=!volume_end[32] && !data_lba[32] && fat_span<total32 && overhead<total32 &&
                cluster_count>=65525 && cluster_count<32'h0fffffee &&
                ({1'b0,fat_size}<<7)>=fat_entries_needed && root_cluster<=max_cluster &&
                (!partition_tried || (!partition_end[32] && total32<=selected_size));
            abs_height<=height[31] ? -height : height;
            row_bytes<=width[10:0]*13'd3;
            stride<=(row_bytes+13'd3)&13'h1ffc;
            payload_bytes<=stride*abs_height[10:0];
            pixel_end<={1'b0,pixel_offset}+payload_bytes;
            next_ok_r<=next_cluster>=2 && next_cluster<=max_cluster &&
                next_cluster<32'h0ffffff0 && next_cluster!=current_cluster;
            sector_more_r<=({1'b0,sector_in_cluster}+9'd1)<{1'b0,spc};
        end
    end
    assign state_code=state[3:0];
    function [2:0] log_spc;
        input [7:0] value;
        begin case(value)
            2:log_spc=1;4:log_spc=2;8:log_spc=3;16:log_spc=4;
            32:log_spc=5;64:log_spc=6;128:log_spc=7;default:log_spc=0;
        endcase end
    endfunction
    // dir_name has settled for twenty clocks before the size's last byte.
    wire named=(dir_name[87:48]==40'h494d473030+40'd1 ||
                dir_name[87:48]==40'h494d473030+40'd2 ||
                dir_name[87:48]==40'h494d473030+40'd3 ||
                dir_name[87:48]==40'h494d473030+40'd4) &&
                dir_name[47:0]==48'h202020424d50;
    wire [1:0] entry_role=dir_name[49:48]-2'd1;
    task status_error;
        input [3:0] code;
        begin if(error_code==0)begin error_code<=code;error_role<=role;end end
    endtask
    task fail_file;
        input [3:0] code;
        begin
            status_error(code);bad_mask[role]<=1;valid_files[role]<=0;
            sd_sec_read<=0;write_req<=0;
            if(scan_busy)state<=NEXT_FILE;
            else begin load_busy<=0;load_error<=1;state<=IDLE;end
        end
    endtask
    task fail_volume;
        input [3:0] code;
        begin status_error(code);scan_busy<=0;scan_done<=1;sd_sec_read<=0;state<=IDLE;end
    endtask
    task request_sector;
        input [2:0] purpose;
        input [31:0] lba;
        begin kind<=purpose;pixel_sector<=purpose==PIX;sd_sec_read_addr<=lba;sd_sec_read<=1;byte_idx<=0;state<=WAIT_SEC;end
    endtask
    task map_cluster;
        input [31:0] cluster;
        input [2:0] purpose;
        begin current_cluster<=cluster;sector_in_cluster<=0;
            map_offset<=cluster-2;map_kind<=purpose;state<=MAP_SHIFT;end
    endtask
    task read_fat;
        input [1:0] purpose;
        begin fat_purpose<=purpose;fat_offset<={current_cluster[6:0],2'b00};state<=FAT_REQ;end
    endtask
    integer i;
    always @(posedge clk or posedge local_rst)begin
        if(local_rst)begin
            state<=IDLE;source_start<=0;sector_pos<=0;kind<=0;map_kind<=0;pixel_sector<=0;catalog_first<=0;fat_entries_needed<=0;fat_purpose<=0;role<=0;cursor<=0;
            scan_busy<=0;scan_done<=0;hit_valid<=0;hit_sector<=0;
            load_busy<=0;load_done<=0;load_error<=0;bmp_topdown<=0;write_req<=0;
            pix_wr_en<=0;pix_data<=0;sd_sec_read<=0;sd_sec_read_addr<=0;
            error_code<=0;error_role<=0;missing_mask<=0;bad_mask<=0;
            byte_idx<=0;found<=0;valid_files<=0;duplicates<=0;
            partition<=0;selected_partition<=0;selected_size<=0;
            partition_tried<=0;partition_found<=0;part_type<=0;part_start<=0;part_size<=0;
            bps<=0;reserved<=0;root_entries<=0;fat16<=0;total16<=0;flags<=0;version<=0;signature<=0;
            fat_count<=0;bpb_spc<=0;spc<=0;spc_shift<=0;sector_in_cluster<=0;
            total32<=0;fat_size<=0;root_cluster<=0;max_cluster<=0;
            partition_end<=0;fat_span<=0;overhead<=0;volume_end<=0;fat_lba<=0;data_lba<=0;
            data_sectors<=0;cluster_count<=0;current_cluster<=0;map_offset<=0;
            cluster_sector<=0;file_size<=0;clusters_left<=0;dir_hops<=0;
            dir_name<=0;dir_attr<=0;dir_hi<=0;dir_lo<=0;dir_size_lo<=0;dir_end<=0;
            fat_value<=0;fat_offset<=0;magic<=0;planes<=0;bpp<=0;bmp_size<=0;pixel_offset<=0;
            dib_size<=0;width<=0;height<=0;compression<=0;image_size<=0;
            file_byte<=0;row<=0;col_byte<=0;phase<=0;blue<=0;green<=0;settle<=0;
            for(i=0;i<4;i=i+1)begin first[i]<=0;sizes[i]<=0;end
        end else begin
            source_start<=0;scan_done<=0;hit_valid<=0;load_done<=0;pix_wr_en<=0;
            if(state==WAIT_SEC && vin)begin
                sd_sec_read<=0;byte_idx<=byte_idx+1'b1;
                if(kind==BOOT)begin
                    case(byte_idx)
                        11:bps[7:0]<=din;12:bps[15:8]<=din;
                        13:bpb_spc<=din;
                        14:reserved[7:0]<=din;15:reserved[15:8]<=din;
                        16:fat_count<=din;
                        17:root_entries[7:0]<=din;18:root_entries[15:8]<=din;
                        19:total16[7:0]<=din;20:total16[15:8]<=din;
                        22:fat16[7:0]<=din;23:fat16[15:8]<=din;
                        32:total32[7:0]<=din;33:total32[15:8]<=din;
                        34:total32[23:16]<=din;35:total32[31:24]<=din;
                        36:fat_size[7:0]<=din;37:fat_size[15:8]<=din;
                        38:fat_size[23:16]<=din;39:fat_size[31:24]<=din;
                        40:flags[7:0]<=din;41:flags[15:8]<=din;
                        42:version[7:0]<=din;43:version[15:8]<=din;
                        44:root_cluster[7:0]<=din;45:root_cluster[15:8]<=din;
                        46:root_cluster[23:16]<=din;47:root_cluster[31:24]<=din;
                        510:signature[7:0]<=din;511:signature[15:8]<=din;
                    endcase
                    if(!partition_tried && byte_idx>=446 && byte_idx<510) begin
                        // MBR entries start at 446, so low nibble is shifted by 14.
                        case((byte_idx-10'd446) & 10'd15)
                            4:part_type<=din;
                            8:part_start[7:0]<=din;9:part_start[15:8]<=din;
                            10:part_start[23:16]<=din;11:part_start[31:24]<=din;
                            12:part_size[7:0]<=din;13:part_size[15:8]<=din;
                            14:part_size[23:16]<=din;
                            15:begin
                                part_size[31:24]<=din;
                                if(!partition_found && (part_type==8'h0b || part_type==8'h0c) &&
                                   part_start!=0 && {din,part_size[23:0]}!=0) begin
                                    partition_found<=1;selected_partition<=part_start;selected_size<={din,part_size[23:0]};
                                end
                            end
                        endcase
                    end
                end
                if(kind==DIR && !dir_end)begin
                    if(byte_idx[4:0]<11)dir_name<={dir_name[79:0],din};
                    case(byte_idx[4:0])
                        0:if(din==0)dir_end<=1;
                        11:dir_attr<=din;
                        20:dir_hi[7:0]<=din;21:dir_hi[15:8]<=din;
                        26:dir_lo[7:0]<=din;27:dir_lo[15:8]<=din;
                        28:dir_size_lo[7:0]<=din;29:dir_size_lo[15:8]<=din;30:dir_size_lo[23:16]<=din;
                        31:if(named && (dir_attr&8'h18)==0 && dir_attr!=8'h0f)begin
                            if(found[entry_role])duplicates[entry_role]<=1;
                            found[entry_role]<=1;first[entry_role]<={dir_hi,dir_lo};
                            sizes[entry_role]<={din,dir_size_lo};
                        end
                    endcase
                end
                if(kind==HEADER)begin
                    case(byte_idx)
                        0:magic[7:0]<=din;
                        1:magic[15:8]<=din;
                        2:bmp_size[7:0]<=din;
                        3:bmp_size[15:8]<=din;
                        4:bmp_size[23:16]<=din;
                        5:bmp_size[31:24]<=din;
                        10:pixel_offset[7:0]<=din;
                        11:pixel_offset[15:8]<=din;
                        12:pixel_offset[23:16]<=din;
                        13:pixel_offset[31:24]<=din;
                        14:dib_size[7:0]<=din;
                        15:dib_size[15:8]<=din;
                        16:dib_size[23:16]<=din;
                        17:dib_size[31:24]<=din;
                        18:width[7:0]<=din;
                        19:width[15:8]<=din;
                        20:width[23:16]<=din;
                        21:width[31:24]<=din;
                        22:height[7:0]<=din;
                        23:height[15:8]<=din;
                        24:height[23:16]<=din;
                        25:height[31:24]<=din;
                        26:planes[7:0]<=din;
                        27:planes[15:8]<=din;
                        28:bpp[7:0]<=din;
                        29:bpp[15:8]<=din;
                        30:compression[7:0]<=din;
                        31:compression[15:8]<=din;
                        32:compression[23:16]<=din;
                        33:compression[31:24]<=din;
                        34:image_size[7:0]<=din;
                        35:image_size[15:8]<=din;
                        36:image_size[23:16]<=din;
                        37:image_size[31:24]<=din;
                    endcase
                end
                if(kind==FAT && byte_idx[8:2]==fat_offset[8:2])begin
                    case(byte_idx[1:0])
                        0:fat_value[7:0]<=din;1:fat_value[15:8]<=din;
                        2:fat_value[23:16]<=din;3:fat_value[31:24]<=din;
                    endcase
                end
            end
            case(state)
                IDLE:begin
                    if(scan_start && sd_init_done)begin
                        scan_busy<=1;found<=0;valid_files<=0;duplicates<=0;
                        error_code<=0;missing_mask<=0;bad_mask<=0;load_error<=0;
                        role<=0;partition<=0;partition_tried<=0;partition_found<=0;
                        request_sector(BOOT,0);
                    end else if(load_start && sd_init_done)begin
                        load_busy<=1;load_error<=0;role<=load_sector[1:0];
                        if(load_sector>3 || !valid_files[load_sector[1:0]])begin
                            load_busy<=0;load_error<=1;
                        end else begin file_size<=sizes[load_sector[1:0]];
                            map_cluster(first[load_sector[1:0]],HEADER);end
                    end
                end
                WAIT_SEC:if(eos)begin sd_sec_read<=0;state<=EVAL;end
                // One cycle separates last-byte capture from registered checks.
                EVAL:begin
                    pixel_sector<=0;
                    if(byte_idx!=512)begin
                        if(scan_busy && (kind==BOOT || kind==DIR || (kind==FAT && fat_purpose==FAT_DIR)))fail_volume(5);
                        else fail_file(5);
                    end else case(kind)
                        BOOT:state<=BOOT_EVAL;DIR:state<=DIR_NEXT;
                        HEADER:state<=HEADER_EVAL;FAT:state<=FAT_EVAL;PIX:begin sector_pos<=0;state<=PIX_FETCH;end
                    endcase
                end
                BOOT_EVAL:begin
                    if(signature!=16'haa55)fail_volume(1);
                    else if(bpb_ok_r)begin
                        spc<=bpb_spc;spc_shift<=log_spc(bpb_spc);
                        fat_span<=fat_count==2 ? {fat_size,1'b0} : {1'b0,fat_size};
                        volume_end<={1'b0,partition}+{1'b0,total32};
                        partition_end<={1'b0,partition}+{1'b0,selected_size};state<=LAYOUT;
                    end else if(!partition_tried && partition_found)begin
                        partition<=selected_partition;partition_tried<=1;
                        request_sector(BOOT,selected_partition);
                    end else fail_volume(1);
                end
                LAYOUT:begin
                    overhead<=fat_span+reserved;
                    fat_lba<={1'b0,partition}+reserved;state<=LAYOUT2;
                end
                LAYOUT2:begin
                    data_lba<={1'b0,partition}+overhead;
                    if(flags[7] && flags[3:0]==1)fat_lba<=fat_lba+fat_size;
                    data_sectors<=total32-overhead[31:0];state<=LIMITS;
                end
                LIMITS:begin
                    cluster_count<=data_sectors>>spc_shift;state<=LIMITS2;
                end
                LIMITS2:begin
                    max_cluster<=cluster_count+1;
                    fat_entries_needed<={1'b0,cluster_count}+2;state<=GEOM_WAIT;
                end
                GEOM_WAIT:state<=GEOM_EVAL;
                GEOM_EVAL:if(!geometry_ok_r)fail_volume(1);else state<=ROOT;
                ROOT:begin dir_hops<=1;dir_end<=0;map_cluster(root_cluster,DIR);end
                DIR_NEXT:begin
                    if(dir_end)begin role<=0;cursor<=0;missing_mask<=~found;state<=CHECK_FILE;end
                    else if(sector_more_r)begin sector_in_cluster<=sector_in_cluster+1'b1;
                        cluster_sector<=cluster_sector+1'b1;request_sector(DIR,cluster_sector+1'b1);end
                    else read_fat(FAT_DIR);
                end
                CHECK_FILE:begin
                    if(!found[role])begin status_error(2);state<=NEXT_FILE;end
                    else if(duplicates[role])fail_file(6);
                    else begin file_size<=sizes[role];catalog_first<=first[role];state<=CHECK_WAIT;end
                end
                CHECK_WAIT:state<=CHECK_META;
                CHECK_META:if(!metadata_ok_r)fail_file(3);else map_cluster(catalog_first,HEADER);
                HEADER_EVAL:begin
                    if(!header_ok_r)fail_file(3);
                    else if(scan_busy)state<=CHAIN_BEGIN;
                    else begin bmp_topdown<=height[31];write_req<=1;state<=WR_REQ;end
                end
                CHAIN_BEGIN:state<=CHAIN_COUNT;
                CHAIN_COUNT:begin
                    clusters_left<=file_clusters_r;
                    current_cluster<=first[role];read_fat(FAT_CHECK);
                end
                FAT_REQ:request_sector(FAT,fat_lba[31:0]+(current_cluster>>7));
                FAT_EVAL:begin
                    case(fat_purpose)
                        FAT_DIR:begin
                            if(eoc_r)begin role<=0;cursor<=0;missing_mask<=~found;state<=CHECK_FILE;end
                            else if(!next_ok_r || dir_hops>=MAX_DIR_CLUSTERS)fail_volume(4);
                            else begin dir_hops<=dir_hops+1'b1;map_cluster(next_cluster,DIR);end
                        end
                        FAT_CHECK:begin
                            if(last_cluster_r)begin
                                if(!eoc_r)fail_file(4);
                                else begin valid_files[role]<=1;state<=NEXT_FILE;end
                            end else if(!next_ok_r)fail_file(4);
                            else begin clusters_left<=clusters_left-1'b1;
                                current_cluster<=next_cluster;
                                fat_offset<={next_cluster[6:0],2'b00};state<=FAT_REQ;end
                        end
                        FAT_DATA:begin
                            if(!next_ok_r || !more_clusters_r)fail_file(4);
                            else begin clusters_left<=clusters_left-1'b1;map_cluster(next_cluster,PIX);end
                        end
                    endcase
                end
                NEXT_FILE:begin
                    if(role==3)begin cursor<=0;state<=PUBLISH;end
                    else begin role<=role+1'b1;state<=CHECK_FILE;end
                end
                PUBLISH:begin
                    if(cursor==4)begin scan_busy<=0;scan_done<=1;state<=IDLE;end
                    else begin
                        if(valid_files[cursor[1:0]])begin hit_valid<=1;hit_sector<={30'd0,cursor[1:0]};end
                        cursor<=cursor+1'b1;
                    end
                end
                MAP_SHIFT:begin map_offset<=map_offset<<spc_shift;state<=MAP_ADD;end
                MAP_ADD:begin cluster_sector<=data_lba[31:0]+map_offset;state<=MAP_REQ;end
                MAP_REQ:request_sector(map_kind,cluster_sector);
                WR_REQ:if(write_req_ack)begin write_req<=0;settle<=0;state<=WR_SETTLE;end
                WR_SETTLE:begin
                    settle<=settle+1'b1;
                    if(settle==127 && !write_req_ack)begin
                        source_start<=1;file_byte<=0;row<=0;col_byte<=0;phase<=0;
                        clusters_left<=file_clusters_r;
                        map_cluster(first[role],PIX);
                    end
                end
                // The sector transport cannot be paused mid-sector. Buffer it,
                // then stop at an RGB pixel boundary while the scaler is busy.
                PIX_FETCH:state<=PIX_BYTE;
                PIX_BYTE:if(!(pix_in_r && col_in_r && phase==2) || pix_ready)begin
                    file_byte<=file_byte+1'b1;
                    if(pix_in_r)begin
                        if(col_in_r)begin
                            case(phase)
                                0:blue<=pixel_byte;
                                1:green<=pixel_byte;
                                2:begin pix_data<={pixel_byte,green,blue};pix_wr_en<=1;end
                            endcase
                            phase<=phase==2 ? 0 : phase+1'b1;
                        end
                        if(col_last_r)begin col_byte<=0;row<=row+1'b1;end
                        else col_byte<=col_byte+1'b1;
                    end
                    if(sector_pos==511)state<=PIX_NEXT;
                    else begin sector_pos<=sector_pos+1'b1;state<=PIX_FETCH;end
                end
                PIX_NEXT:begin
                    if(row==source_height)begin load_busy<=0;load_done<=1;state<=IDLE;end
                    else if(file_byte>=file_size)fail_file(3);
                    else if(sector_more_r)begin sector_in_cluster<=sector_in_cluster+1'b1;
                        cluster_sector<=cluster_sector+1'b1;request_sector(PIX,cluster_sector+1'b1);end
                    else read_fat(FAT_DATA);
                end
            endcase
            // Held abort overrides all parsing, requests and completion pulses.
            if(load_abort)begin
                if(scan_busy)begin scan_done<=1;status_error(5);end
                scan_busy<=0;load_busy<=0;load_done<=0;hit_valid<=0;
                source_start<=0;write_req<=0;pix_wr_en<=0;pixel_sector<=0;sd_sec_read<=0;state<=IDLE;
            end
        end
    end
endmodule
