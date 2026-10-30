// M10 exhibit guide audio. One root-directory pass catalogs MUSIC.WAV and
// NARR01..NARR04.WAV; the track then follows the displayed exhibit:
//   guide_mode (manual browsing) : NARR0n of the displayed page, once, then
//                                  MUSIC; with no MUSIC the narration loops
//   otherwise (auto slideshow)   : MUSIC
// A missing or malformed narration falls back to MUSIC. Tracks switch only
// between sectors (a WAV frame never straddles a sector: 44 and 512 are
// multiples of 4). Streaming field capture avoids a 512-byte sector-buffer
// mux. Supports fragmented files and directory chains, MBR FAT32 primary
// partitions and superfloppy volumes. WAV input is the canonical 44-byte PCM
// format produced by prepare_music.py.
// tone_fallback: no playable WAV at all, a fatal card/volume error, or no card
// initialized within NO_CARD_CYCLES -> the output stage plays a test tone.
// M11: the same directory pass also records SUB01..SUB04.BIN (exactly 2048
// bytes, plain files). Right after the catalog the FSM maps each page's first
// cluster to its first sector LBA with the existing map_cluster datapath
// (SUB_NEXT/SUB_CHECK/SUB_STORE), rejecting clusters out of range or smaller
// than 4 sectors. Every cluster 2..max_cluster lies wholly inside the data
// area, so the page's four sectors are then always on the volume. The loader
// needs no 32-bit arithmetic of its own.
// M13 guided tour: with tour=1 the displayed exhibit's narration is played
// once and then the terminal stays silent until the page changes (the tour
// controller advances when guide_done rises), instead of continuing with
// MUSIC. Pages without a narration still play MUSIC. tour=0 is exactly M11.
module fat32_guide_reader #(
    parameter integer MAX_DIR_CLUSTERS = 128,
    parameter integer NO_CARD_CYCLES = 300_000_000   // 3 s at 100 MHz
)(
    input wire clk, rst, enable, pause,
    input wire guide_mode, page_valid, tour,
    input wire [1:0] page,
    input wire [11:0] free_frames,
    output reg pcm_wr,
    output wire pcm_flush,
    output reg [31:0] pcm_data,
    output reg ready,
    output reg [1:0] track_kind,      // 0 none, 1 MUSIC, 2 narration (while ready)
    output reg guide_done,            // narration of the displayed page heard once
    output reg tone_fallback,
    output reg [7:0] error_code,
    output reg sector_req,
    output reg [31:0] sector_lba,
    input wire [7:0] sector_data,
    input wire sector_valid, sector_end, sector_fault,
    // Subtitle catalog (read-only view, clk domain).
    output wire sub_ready,               // catalog done and page LBAs computed
    output reg [3:0] sub_present,        // usable pages (valid after sub_ready)
    input wire [1:0] sub_page,
    output reg [31:0] sub_lba_q          // registered: first LBA of SUB0(sub_page+1)
);
    // Local reset branch avoids the high-fanout top-level SD reset net.
    wire local_rst;
    reset_sync u_local_reset(.clk(clk),.async_rst(rst),.rst(local_rst));
    // Register the arbiter output as one coherent byte/end/fault bundle. Its
    // one-cycle latency fits the arbiter GAP before the next request selection.
    reg [7:0] sector_data_q;
    reg sector_valid_q,sector_end_q,sector_fault_q;
    always @(posedge clk or posedge local_rst)begin
        if(local_rst)begin sector_data_q<=0;sector_valid_q<=0;sector_end_q<=0;sector_fault_q<=0;end
        else begin
            sector_data_q<=sector_data;sector_valid_q<=sector_valid;
            sector_end_q<=sector_end;sector_fault_q<=sector_fault;
        end
    end
    localparam START=0, WAIT=1, EVAL=2, LAYOUT=3, ROOT=4,
               DIR_NEXT=5, FAT_NEXT=6, AUDIO=7, FAILED=8, LIMITS=9,
               MAP_SHIFT=10, MAP_ADD=11, MAP_REQUEST=12, LAYOUT_CHECK=13,
               PICK=14, PICK_CHECK=15, NO_TRACK=16,
               SUB_NEXT=17, SUB_CHECK=18, SUB_STORE=19;
    localparam BOOT=0, DIR=1, FAT=2, HEADER=3, DATA=4, SUBMAP=5;
    // Catalog slots: 0 = MUSIC, 1..4 = NARR01..NARR04; 7 = no track.
    localparam [2:0] SLOT_MUSIC=0, SLOT_NONE=7;
    reg [4:0] state;
    reg [2:0] kind;
    reg [9:0] byte_idx;
    reg [31:0] partition, volume_end, fat_lba, data_lba, max_cluster;
    reg catalog_done;
    reg [2:0] spc_shift;
    reg partition_tried, partition_found;
    reg [7:0] part_type;
    reg [31:0] part_start, part_size, selected_partition;
    reg [31:0] current_cluster, first_cluster, file_size, remaining;
    reg [7:0] sector_in_cluster, spc;
    reg [31:0] cluster_sector;
    reg [31:0] map_offset;
    reg [2:0] map_kind;
    reg fat_for_dir;
    reg [31:0] fat_value;
    reg [8:0] fat_offset;
    reg [15:0] dir_hops;
    reg [31:0] chain_hops;
    reg [5:0] skip_bytes;
    reg [1:0] pcm_phase;
    reg [23:0] pcm_partial;

    // BPB (little endian); all fields are captured again for the partition VBR.
    reg [15:0] bps, reserved, fat16, total16, flags, version, signature;
    // FAT32 requires BPB_RootEntCnt==0. Retain that predicate directly instead
    // of two separately enabled byte-register banks (TD local CE collision).
    reg root_entries_zero;
    reg [7:0] fat_count, bpb_spc;
    reg [31:0] total32, fat_size, root_cluster;
    reg [31:0] fat_span, data_sectors;
    // Separate BPB address arithmetic from the 100 MHz compare/FSM path.
    reg [32:0] layout_end, layout_used;
    reg [87:0] dir_name;
    reg [7:0] dir_attr;
    reg [15:0] dir_hi, dir_lo;
    reg [31:0] dir_size;
    reg dir_end;
    reg [3:0] name_slot;
    reg [31:0] sub_cluster [0:3];     // first cluster, then first LBA
    reg [2:0] sub_map;                // page being mapped; 4 = all done
    assign sub_ready = catalog_done && sub_map[2];
    reg [31:0] cat_cluster [0:4];
    reg [31:0] cat_size [0:4];
    reg [4:0] cat_present;
    reg [2:0] current;          // slot being played / prepared
    reg [1:0] pick_wait;
    // Register the controller-domain selectors locally (same clock).
    reg guide_q, page_valid_q, tour_q;
    reg [1:0] page_q;
    reg [3:0] select_prev;
    reg [28:0] no_card_timer;
    reg no_card;
    wire [2:0] page_slot = {1'b0,page_q}+3'd1;
    wire narr_ok = guide_q && page_valid_q && cat_present[page_slot];
    wire [2:0] desired = narr_ok && (!guide_done || (!tour_q && !cat_present[SLOT_MUSIC])) ? page_slot :
                         narr_ok && tour_q ? SLOT_NONE :
                         cat_present[SLOT_MUSIC] ? SLOT_MUSIC : SLOT_NONE;
    reg [2:0] desired_r;
    integer n;
    // 0 MUSIC, 1..4 NARR01..04, 8..11 SUB01..04, 15 other.
    function [3:0] slot_of;
        input [87:0] n;
        begin
            case(n)
                "MUSIC   WAV":slot_of=0;
                "NARR01  WAV":slot_of=1;"NARR02  WAV":slot_of=2;
                "NARR03  WAV":slot_of=3;"NARR04  WAV":slot_of=4;
                "SUB01   BIN":slot_of=8;"SUB02   BIN":slot_of=9;
                "SUB03   BIN":slot_of=10;"SUB04   BIN":slot_of=11;
                default:slot_of=15;
            endcase
        end
    endfunction
    always @(posedge clk) sub_lba_q<=sub_cluster[sub_page];
    // RIFF fields, canonical PCM fmt(16) immediately followed by data.
    reg [31:0] riff, riff_size, wave_id, fmt_id, fmt_size, sample_rate;
    reg [31:0] byte_rate, data_id, data_size;
    reg [15:0] format_tag, channels, block_align, bits;
    wire bpb_ok = bps==512 && bpb_spc!=0 &&
                  ((bpb_spc & (bpb_spc-1'b1))==0) && reserved!=0 &&
                  (fat_count==1 || fat_count==2) && root_entries_zero &&
                  fat16==0 && total16==0 && fat_size!=0 && version==0 &&
                  root_cluster>=2 && root_cluster<32'h0ffffff0 &&
                  (!flags[7] || flags[3:0]<fat_count);
    wire wav_ok = riff==32'h46464952 && wave_id==32'h45564157 &&
                  fmt_id==32'h20746d66 && fmt_size==16 && format_tag==1 &&
                  channels==2 && sample_rate==48000 && byte_rate==192000 &&
                  block_align==4 && bits==16 && data_id==32'h61746164 &&
                  data_size!=0 && data_size[1:0]==0 && file_size>=44 &&
                  data_size<=file_size-44 && riff_size==data_size+36;
    // These fields settle during the rest of the 512-byte sector, long before
    // EVAL/DIR_NEXT uses them. Break large compare+FSM paths at 100MHz.
    reg bpb_ok_r,wav_ok_r,sector_more_r;
    always @(posedge clk or posedge local_rst)begin
        if(local_rst)begin bpb_ok_r<=0;wav_ok_r<=0;sector_more_r<=0;end
        else begin bpb_ok_r<=bpb_ok;wav_ok_r<=wav_ok;
            sector_more_r<=({1'b0,sector_in_cluster}+9'd1)<{1'b0,spc};end
    end
    wire [31:0] next_cluster = {4'd0,fat_value[27:0]};
    wire next_valid_c = next_cluster>=2 && next_cluster<=max_cluster &&
                        next_cluster<32'h0ffffff0 && next_cluster!=current_cluster;
    // M13: fat_value is complete mid-sector, at least the FAT state before
    // FAT_NEXT reads these, so the 32-bit compares are registered here.
    reg next_valid, fat_chain_end;
    always @(posedge clk or posedge local_rst)begin
        if(local_rst)begin next_valid<=0;fat_chain_end<=0;end
        else begin next_valid<=next_valid_c;fat_chain_end<=fat_value[27:0]>=28'hffffff8;end
    end
    assign pcm_flush = (state==AUDIO) && (free_frames<128);

    function [2:0] log_spc;
        input [7:0] value;
        begin
            case(value)
                2:log_spc=1;4:log_spc=2;8:log_spc=3;16:log_spc=4;
                32:log_spc=5;64:log_spc=6;128:log_spc=7;default:log_spc=0;
            endcase
        end
    endfunction
    task request_sector;
        input [2:0] purpose;
        input [31:0] lba;
        begin
            kind<=purpose; sector_lba<=lba; sector_req<=1;
            byte_idx<=0; state<=WAIT;
        end
    endtask
    task fail;
        input [7:0] code;
        begin error_code<=code; ready<=0; sector_req<=0; state<=FAILED; end
    endtask
    task read_fat;
        input directory;
        begin
            fat_for_dir<=directory;
            fat_offset<={current_cluster[6:0],2'b00};
            request_sector(FAT, fat_lba+(current_cluster>>7));
        end
    endtask
    task map_cluster;
        input [31:0] cluster;
        input [2:0] purpose;
        begin
            current_cluster<=cluster;sector_in_cluster<=0;
            map_offset<=cluster-2;map_kind<=purpose;state<=MAP_SHIFT;
        end
    endtask

    always @(posedge clk or posedge local_rst) begin
        if(local_rst) begin
            state<=START;kind<=BOOT;byte_idx<=0;sector_req<=0;sector_lba<=0;
            pcm_wr<=0;pcm_data<=0;ready<=0;error_code<=0;
            partition<=0;partition_tried<=0;partition_found<=0;
            part_type<=0;part_start<=0;part_size<=0;selected_partition<=0;
            volume_end<=0;fat_lba<=0;data_lba<=0;max_cluster<=0;
            current_cluster<=0;first_cluster<=0;file_size<=0;remaining<=0;
            sector_in_cluster<=0;spc<=0;spc_shift<=0;cluster_sector<=0;map_offset<=0;map_kind<=0;
            fat_for_dir<=0;fat_value<=0;fat_offset<=0;dir_hops<=0;chain_hops<=0;
            skip_bytes<=0;pcm_phase<=0;pcm_partial<=0;
            bps<=0;reserved<=0;root_entries_zero<=0;fat16<=0;total16<=0;
            flags<=0;version<=0;signature<=0;fat_count<=0;bpb_spc<=0;
            total32<=0;fat_size<=0;root_cluster<=0;fat_span<=0;data_sectors<=0;layout_end<=0;layout_used<=0;
            dir_name<=0;dir_attr<=0;dir_hi<=0;dir_lo<=0;dir_size<=0;
            dir_end<=0;name_slot<=15;sub_present<=0;
            for(n=0;n<5;n=n+1)begin cat_cluster[n]<=0;cat_size[n]<=0;end
            for(n=0;n<4;n=n+1)sub_cluster[n]<=0;
            sub_map<=0;
            cat_present<=0;catalog_done<=0;current<=SLOT_NONE;guide_done<=0;pick_wait<=0;
            guide_q<=0;page_valid_q<=0;page_q<=0;tour_q<=0;select_prev<=0;desired_r<=SLOT_NONE;
            no_card_timer<=0;no_card<=0;tone_fallback<=0;track_kind<=0;
            riff<=0;riff_size<=0;wave_id<=0;fmt_id<=0;fmt_size<=0;
            sample_rate<=0;byte_rate<=0;data_id<=0;data_size<=0;
            format_tag<=0;channels<=0;block_align<=0;bits<=0;
        end else begin
            pcm_wr<=0;
            guide_q<=guide_mode;page_valid_q<=page_valid;page_q<=page;tour_q<=tour;
            select_prev<={guide_q,page_valid_q,page_q};
            desired_r<=desired;
            if(enable)no_card_timer<=0;
            else if(no_card_timer<NO_CARD_CYCLES-1)no_card_timer<=no_card_timer+1'b1;
            no_card<=!enable && no_card_timer>=NO_CARD_CYCLES-1;
            tone_fallback<=state==FAILED || no_card || (catalog_done && cat_present==0);
            track_kind<=!ready ? 2'd0 : current==SLOT_MUSIC ? 2'd1 : 2'd2;
            if(state==WAIT && sector_valid_q) begin
                byte_idx<=byte_idx+1'b1;
                if(kind==BOOT) begin
                    case(byte_idx)
                        11:bps[7:0]<=sector_data_q;12:bps[15:8]<=sector_data_q;
                        13:bpb_spc<=sector_data_q;
                        14:reserved[7:0]<=sector_data_q;15:reserved[15:8]<=sector_data_q;
                        16:fat_count<=sector_data_q;
                        17:root_entries_zero<=sector_data_q==0;
                        18:root_entries_zero<=root_entries_zero && sector_data_q==0;
                        19:total16[7:0]<=sector_data_q;20:total16[15:8]<=sector_data_q;
                        22:fat16[7:0]<=sector_data_q;23:fat16[15:8]<=sector_data_q;
                        32:total32[7:0]<=sector_data_q;33:total32[15:8]<=sector_data_q;
                        34:total32[23:16]<=sector_data_q;35:total32[31:24]<=sector_data_q;
                        36:fat_size[7:0]<=sector_data_q;37:fat_size[15:8]<=sector_data_q;
                        38:fat_size[23:16]<=sector_data_q;39:fat_size[31:24]<=sector_data_q;
                        40:flags[7:0]<=sector_data_q;41:flags[15:8]<=sector_data_q;
                        42:version[7:0]<=sector_data_q;43:version[15:8]<=sector_data_q;
                        44:root_cluster[7:0]<=sector_data_q;45:root_cluster[15:8]<=sector_data_q;
                        46:root_cluster[23:16]<=sector_data_q;47:root_cluster[31:24]<=sector_data_q;
                        510:signature[7:0]<=sector_data_q;511:signature[15:8]<=sector_data_q;
                    endcase
                    if(!partition_tried && byte_idx>=446 && byte_idx<510) begin
                        // MBR entries start at 446, so low nibble is shifted by 14.
                        case((byte_idx-10'd446) & 10'd15)
                            4:part_type<=sector_data_q;
                            8:part_start[7:0]<=sector_data_q;9:part_start[15:8]<=sector_data_q;
                            10:part_start[23:16]<=sector_data_q;11:part_start[31:24]<=sector_data_q;
                            12:part_size[7:0]<=sector_data_q;13:part_size[15:8]<=sector_data_q;
                            14:part_size[23:16]<=sector_data_q;
                            15:begin
                                part_size[31:24]<=sector_data_q;
                                if(!partition_found && (part_type==8'h0b || part_type==8'h0c) &&
                                   part_start!=0 && {sector_data_q,part_size[23:0]}!=0) begin
                                    partition_found<=1;selected_partition<=part_start;
                                end
                            end
                        endcase
                    end
                end
                if(kind==DIR && !dir_end) begin
                    if(byte_idx[4:0]<11) dir_name<={dir_name[79:0],sector_data_q};
                    case(byte_idx[4:0])
                        0:if(sector_data_q==0) dir_end<=1;
                        // Name bytes 0..10 are complete here; register the
                        // five-way compare away from the byte-31 update.
                        11:begin dir_attr<=sector_data_q;name_slot<=slot_of(dir_name);end
                        20:dir_hi[7:0]<=sector_data_q;21:dir_hi[15:8]<=sector_data_q;
                        26:dir_lo[7:0]<=sector_data_q;27:dir_lo[15:8]<=sector_data_q;
                        28:dir_size[7:0]<=sector_data_q;29:dir_size[15:8]<=sector_data_q;
                        30:dir_size[23:16]<=sector_data_q;
                        31:begin
                            dir_size[31:24]<=sector_data_q;
                            // First plain-file match wins; LFN/volume/dir never match.
                            if(name_slot<5 && !cat_present[name_slot[2:0]] &&
                               (dir_attr & 8'h18)==0 && dir_attr!=8'h0f) begin
                                cat_cluster[name_slot[2:0]]<={dir_hi,dir_lo};
                                cat_size[name_slot[2:0]]<={sector_data_q,dir_size[23:0]};
                                cat_present[name_slot[2:0]]<=1;
                            end
                            // Subtitle pages have one fixed size; others are ignored.
                            if(name_slot[3] && !sub_present[name_slot[1:0]] &&
                               (dir_attr & 8'h18)==0 && dir_attr!=8'h0f &&
                               sector_data_q==0 && dir_size[23:0]==24'd2048) begin
                                sub_cluster[name_slot[1:0]]<={dir_hi,dir_lo};
                                sub_present[name_slot[1:0]]<=1;
                            end
                        end
                    endcase
                end
                if(kind==FAT) begin
                    if(byte_idx=={1'b0,fat_offset}) fat_value[7:0]<=sector_data_q;
                    if(byte_idx=={1'b0,fat_offset}+1) fat_value[15:8]<=sector_data_q;
                    if(byte_idx=={1'b0,fat_offset}+2) fat_value[23:16]<=sector_data_q;
                    if(byte_idx=={1'b0,fat_offset}+3) fat_value[31:24]<=sector_data_q;
                end
                if(kind==HEADER && byte_idx<44) begin
                    // Fixed-width shifts implement little-endian field capture.
                    case(byte_idx[9:2])
                        0:riff<={sector_data_q,riff[31:8]};
                        1:riff_size<={sector_data_q,riff_size[31:8]};
                        2:wave_id<={sector_data_q,wave_id[31:8]};
                        3:fmt_id<={sector_data_q,fmt_id[31:8]};
                        4:fmt_size<={sector_data_q,fmt_size[31:8]};
                        5:if(!byte_idx[1]) format_tag<={sector_data_q,format_tag[15:8]};
                          else channels<={sector_data_q,channels[15:8]};
                        6:sample_rate<={sector_data_q,sample_rate[31:8]};
                        7:byte_rate<={sector_data_q,byte_rate[31:8]};
                        8:if(!byte_idx[1]) block_align<={sector_data_q,block_align[15:8]};
                          else bits<={sector_data_q,bits[15:8]};
                        9:data_id<={sector_data_q,data_id[31:8]};
                        10:data_size<={sector_data_q,data_size[31:8]};
                    endcase
                end
                if(kind==DATA) begin
                    if(skip_bytes!=0) skip_bytes<=skip_bytes-1'b1;
                    else if(remaining!=0) begin
                        remaining<=remaining-1'b1;
                        pcm_phase<=pcm_phase+1'b1;
                        pcm_partial<={sector_data_q,pcm_partial[23:8]};
                        if(pcm_phase==3) begin
                            pcm_data<={sector_data_q,pcm_partial};pcm_wr<=1;
                        end
                    end
                end
            end
            case(state)
                START: if(enable) request_sector(BOOT,0);
                WAIT: if(sector_end_q) begin
                    sector_req<=0;
                    if(byte_idx + (sector_valid_q ? 1 : 0)!=512) fail(8'h11);
                    else state<=EVAL;
                end
                EVAL: case(kind)
                    BOOT: begin
                        if(signature!=16'haa55) fail(8'h20);
                        else if(bpb_ok_r) begin
                            spc<=bpb_spc;spc_shift<=log_spc(bpb_spc);
                            fat_span<=fat_count==2 ? fat_size<<1 : fat_size;
                            fat_lba<=partition+{16'd0,reserved};state<=LAYOUT;
                        end else if(!partition_tried && partition_found) begin
                            partition<=selected_partition;partition_tried<=1;
                            request_sector(BOOT,selected_partition);
                        end else fail(8'h21);
                    end
                    DIR: if(dir_end) begin catalog_done<=1;sub_map<=0;state<=SUB_NEXT;end
                    else state<=DIR_NEXT;
                    FAT: state<=FAT_NEXT;
                    // A malformed narration is dropped; the pick falls back to MUSIC.
                    HEADER: if(!wav_ok_r) begin
                        cat_present[current]<=0;pick_wait<=3;state<=PICK;
                    end else begin
                        remaining<=data_size;skip_bytes<=44;pcm_phase<=0;
                        ready<=1;chain_hops<=0;state<=AUDIO;
                    end
                    DATA: if(remaining==0) begin
                        if(current!=SLOT_MUSIC && (cat_present[SLOT_MUSIC] || tour_q)) begin
                            // Narration heard once: background music, or tour silence.
                            guide_done<=1;ready<=0;pick_wait<=3;state<=PICK;
                        end else begin
                            // Loop without exposing sector padding as audio.
                            remaining<=data_size;skip_bytes<=44;chain_hops<=0;
                            map_cluster(first_cluster,DATA);
                        end
                    end else if(sector_more_r) begin
                        sector_in_cluster<=sector_in_cluster+1'b1;
                        cluster_sector<=cluster_sector+1'b1;state<=AUDIO;
                    end else read_fat(1'b0);
                endcase
                LAYOUT: begin
                    layout_end<={1'b0,partition}+{1'b0,total32};
                    layout_used<={1'b0,fat_span}+{17'd0,reserved};
                    state<=LAYOUT_CHECK;
                end
                LAYOUT_CHECK: begin
                    if({1'b0,total32}<=layout_used ||
                       fat_span<fat_size || layout_end[32]) fail(8'h21);
                    else begin
                        data_lba<=fat_lba+fat_span;
                        volume_end<=layout_end[31:0];
                        data_sectors<=total32-layout_used[31:0];
                        // FAT mirroring disabled: honor the selected active FAT.
                        if(flags[7] && flags[3:0]==1) fat_lba<=fat_lba+fat_size;
                        state<=LIMITS;
                    end
                end
                LIMITS: begin
                    max_cluster<=(data_sectors>>spc_shift)+1;state<=ROOT;
                end
                ROOT: begin
                    if(root_cluster>max_cluster || max_cluster>=32'h0ffffff0 ||
                       (max_cluster>>7)>=fat_size) fail(8'h24);
                    else begin
                        dir_hops<=1;map_cluster(root_cluster,DIR);
                    end
                end
                DIR_NEXT: if(sector_more_r) begin
                    sector_in_cluster<=sector_in_cluster+1'b1;
                    cluster_sector<=cluster_sector+1'b1;
                    request_sector(DIR,cluster_sector+1'b1);
                end else if(dir_hops>=MAX_DIR_CLUSTERS) fail(8'h23);
                else read_fat(1'b1);
                // A full root directory has no zero entry: its chain end is the end.
                FAT_NEXT: if(fat_for_dir && fat_chain_end) begin
                    catalog_done<=1;sub_map<=0;state<=SUB_NEXT;
                end else if(!next_valid) fail(fat_for_dir ? 8'h22 : 8'h25);
                else begin
                    if(fat_for_dir) begin
                        dir_hops<=dir_hops+1'b1;
                        map_cluster(next_cluster,DIR);
                    end else begin
                        chain_hops<=chain_hops+1'b1;
                        if(chain_hops>=(file_size>>9)) fail(8'h23);
                        else map_cluster(next_cluster,DATA);
                    end
                end
                // Between sectors: follow the exhibit before fetching more PCM.
                AUDIO: if(!pause && desired_r!=current) begin
                    ready<=0;pick_wait<=3;state<=PICK;
                end else if(!pause && free_frames>=128) begin
                    if(cluster_sector>=volume_end) fail(8'h24);
                    else request_sector(DATA,cluster_sector);
                end
                PICK: if(pick_wait!=0) pick_wait<=pick_wait-1'b1;
                else if(desired_r==SLOT_NONE) begin
                    current<=SLOT_NONE;ready<=0;state<=NO_TRACK;
                end else begin
                    current<=desired_r;
                    first_cluster<=cat_cluster[desired_r];
                    file_size<=cat_size[desired_r];
                    state<=PICK_CHECK;
                end
                PICK_CHECK: if(first_cluster<2 || first_cluster>max_cluster || file_size<44) begin
                    cat_present[current]<=0;pick_wait<=3;state<=PICK;
                end else map_cluster(first_cluster,HEADER);
                // Nothing to play for this page yet (e.g. narration-only card
                // before the first image): wait silently for a selectable slot.
                NO_TRACK: if(desired_r!=SLOT_NONE) begin pick_wait<=1;state<=PICK;end
                // Subtitle page mapping, before any audio is selected.
                SUB_NEXT: if(sub_map[2]) begin pick_wait<=3;state<=PICK;end
                else if(!sub_present[sub_map[1:0]]) sub_map<=sub_map+1'b1;
                else begin first_cluster<=sub_cluster[sub_map[1:0]];state<=SUB_CHECK;end
                SUB_CHECK: if(first_cluster<2 || first_cluster>max_cluster || spc_shift<2) begin
                    sub_present[sub_map[1:0]]<=0;sub_map<=sub_map+1'b1;state<=SUB_NEXT;
                end else map_cluster(first_cluster,SUBMAP);
                SUB_STORE: begin
                    sub_cluster[sub_map[1:0]]<=cluster_sector;
                    sub_map<=sub_map+1'b1;state<=SUB_NEXT;
                end
                MAP_SHIFT: begin map_offset<=map_offset<<spc_shift;state<=MAP_ADD;end
                MAP_ADD: begin cluster_sector<=data_lba+map_offset;state<=MAP_REQUEST;end
                MAP_REQUEST: if(map_kind==DATA) state<=AUDIO;
                             else if(map_kind==SUBMAP) state<=SUB_STORE;
                             else request_sector(map_kind,cluster_sector);
                FAILED: sector_req<=0;
                default: fail(8'h12);
            endcase
            // Another exhibit or mode: the next narration starts afresh
            // (after the case so a same-cycle end of file cannot survive it).
            if(select_prev!={guide_q,page_valid_q,page_q})guide_done<=0;
            // Includes errors on the image side: discard any abandoned request.
            if(sector_fault_q && state!=START && state!=FAILED) fail(8'h10);
        end
    end
endmodule
