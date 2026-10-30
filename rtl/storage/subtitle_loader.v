// M11 card subtitle loader (SD clock domain).
// When the displayed exhibit changes, reads its SUBnn.BIN page (2048 bytes =
// four consecutive sectors of the file's first cluster) into the back bank of
// subtitle_store, validates it while streaming, then publishes the result.
//
// SUBnn.BIN layout (scripts/prepare_subtitles.py):
//   0..3   "SUB1"            4 text length L (1..63)   5 glyph count G (1..60)
//   6..7   16-bit little-endian sum of bytes 8..2047
//   64..127  L glyph indices (< G), rest padding
//   128..2047 G glyphs x 16 rows x 16 bits, big-endian (bit 15 = left pixel)
//
// Bank ownership: the loader only writes bank ~shown_bank. A publish is a
// toggle request whose payload is held until the video side acknowledges,
// and the video side acknowledges only after it has switched at a frame
// boundary. So the bank being scanned out is never written.
// Anything unusable (no file, cluster < 2 KB or out of volume -- both checked
// by the reader --, bad header/sum/index, card fault) publishes "invalid":
// the OSD falls back to the on-chip ticker.
module subtitle_loader(
    input wire clk, rst,
    // catalog from fat32_guide_reader
    input wire sub_ready,
    input wire [3:0] sub_present,
    output reg [1:0] sub_page,
    input wire [31:0] sub_lba_q,          // first LBA of the page (4 sectors)
    // displayed exhibit
    input wire page_valid,
    input wire [1:0] page,
    // arbiter client
    output reg sector_req,
    output reg [31:0] sector_lba,
    input wire [7:0] sector_data,
    input wire sector_valid, sector_end, sector_fault,
    // store write port
    output reg wr_en, wr_text,
    output reg [9:0] wr_addr,
    output reg [15:0] wr_data,
    output wire wr_bank,
    // publish handshake to subtitle_commit (video domain)
    output reg pub_req,
    output reg pub_valid, pub_bank,
    output reg [5:0] pub_len,
    input wire pub_ack,
    output reg [7:0] loads, invalids    // diagnostics: valid pages, invalid/absent publishes
);
    localparam IDLE=0, LOOKUP=1, START=2, READ=5, NEXT=6,
               VALIDATE=7, PUBLISH=8, WAIT_ACK=9;
    reg [3:0] state;
    (* async_reg = "true" *) reg [2:0] ack_sync;
    reg shown_bank;
    assign wr_bank = ~shown_bank;
    // key = what the display needs: {valid page, page}
    reg page_valid_q;
    reg [1:0] page_q;
    wire [2:0] want = {page_valid_q, page_q};
    reg [2:0] published, loading;
    reg ever;                       // anything published yet
    reg restart;

    reg [1:0] sec;
    reg [9:0] byte_idx;
    reg bad;
    reg [5:0] len;
    reg [5:0] count;
    reg [15:0] sum, expect_sum;
    reg [7:0] hi;
    reg [7:0] data_q;
    reg valid_q, end_q, fault_q;
    wire [10:0] pos = {sec, byte_idx[8:0]};
    wire [10:0] glyph_off = pos - 11'd128;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state<=IDLE; ack_sync<=0; shown_bank<=0; page_valid_q<=0; page_q<=0;
            published<=0; loading<=0; ever<=0; restart<=0;
            sec<=0; byte_idx<=0; bad<=0; len<=0; count<=0; sum<=0; expect_sum<=0; hi<=0;
            data_q<=0; valid_q<=0; end_q<=0; fault_q<=0;
            sub_page<=0; sector_req<=0; sector_lba<=0;
            wr_en<=0; wr_text<=0; wr_addr<=0; wr_data<=0;
            pub_req<=0; pub_valid<=0; pub_bank<=0; pub_len<=0; loads<=0; invalids<=0;
        end else begin
            ack_sync<={ack_sync[1:0],pub_ack};
            page_valid_q<=page_valid; page_q<=page;
            data_q<=sector_data; valid_q<=sector_valid; end_q<=sector_end; fault_q<=sector_fault;
            wr_en<=0;
            if (state!=IDLE && state!=PUBLISH && state!=WAIT_ACK && want!=loading) restart<=1;
            case (state)
                IDLE: if (sub_ready && (want!=published || !ever)) begin
                    loading<=want; restart<=0; sub_page<=page_q;
                    if (!page_valid_q || !sub_present[page_q]) begin
                        bad<=1; state<=PUBLISH;
                    end else state<=LOOKUP;
                end
                LOOKUP: state<=START;               // sub_lba_q follows sub_page
                START: begin
                    sector_lba<=sub_lba_q;
                    sec<=0; byte_idx<=0; bad<=0; sum<=0; len<=0; count<=0;
                    state<=READ;
                end
                READ: begin
                    if (!sector_req && !end_q) begin sector_req<=1; byte_idx<=0; end
                    if (valid_q && sector_req) begin
                        byte_idx<=byte_idx+1'b1;
                        if (pos>=8) sum<=sum+{8'd0,data_q};
                        case (pos)
                            0: if (data_q!="S") bad<=1;
                            1: if (data_q!="U") bad<=1;
                            2: if (data_q!="B") bad<=1;
                            3: if (data_q!="1") bad<=1;
                            4: len<=data_q[5:0];
                            5: begin count<=data_q[5:0]; if (data_q>60 || data_q==0) bad<=1; end
                            6: expect_sum[7:0]<=data_q;
                            7: expect_sum[15:8]<=data_q;
                        endcase
                        if (pos==4 && (data_q>63 || data_q==0)) bad<=1;
                        if (pos>=64 && pos<128) begin
                            if (pos[5:0]<len && data_q>={2'b0,count}) bad<=1;
                            wr_en<=1; wr_text<=1; wr_addr<={4'd0,pos[5:0]}; wr_data<={10'd0,data_q[5:0]};
                        end
                        if (pos>=128) begin
                            if (!glyph_off[0]) hi<=data_q;
                            else begin
                                wr_en<=1; wr_text<=0; wr_addr<=glyph_off[10:1]; wr_data<={hi,data_q};
                            end
                        end
                    end
                    if (end_q && sector_req) begin
                        sector_req<=0;
                        if (byte_idx+(valid_q ? 1 : 0)!=512) bad<=1;
                        state<=NEXT;
                    end
                    if (fault_q) begin sector_req<=0; bad<=1; state<=PUBLISH; end
                end
                NEXT: if (restart) state<=IDLE;
                      else if (sec==3) state<=VALIDATE;
                      else begin sec<=sec+1'b1; sector_lba<=sector_lba+32'd1; state<=READ; end
                VALIDATE: begin
                    if (sum!=expect_sum) bad<=1;
                    state<=PUBLISH;
                end
                PUBLISH: if (restart) state<=IDLE;
                else begin
                    pub_valid<=!bad; pub_bank<=wr_bank; pub_len<=len;
                    pub_req<=~pub_req; published<=loading; ever<=1;
                    if (bad) invalids<=invalids+1'b1; else loads<=loads+1'b1;
                    state<=WAIT_ACK;
                end
                WAIT_ACK: if (ack_sync[2]==pub_req) begin
                    if (pub_valid) shown_bank<=pub_bank;
                    state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
