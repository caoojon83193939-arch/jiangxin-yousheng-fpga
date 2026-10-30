// Level requests remain asserted until end; an entire CMD17 belongs to one
// client. GAP lets clients update their address before the next arbitration.
// M11: third client for card subtitles. Fixed priority music > subtitle >
// image: audio must not underrun, a subtitle page is only four sectors, and
// image loads tolerate the short delay. Image-side semantics are unchanged.
module sd_sector_arbiter3 #(
    parameter integer TIMEOUT_CYCLES = 25_000_000
)(
    input wire clk, rst, card_ready,
    input wire image_req, music_req, sub_req,
    input wire [31:0] image_lba, music_lba, sub_lba,
    output wire image_valid, image_end, music_valid, music_end, sub_valid, sub_end,
    output wire image_idle,
    output wire [7:0] client_data,
    output reg fault,
    output reg card_reset,
    output reg card_req,
    output reg [31:0] card_lba,
    input wire card_valid, card_end,
    input wire [7:0] card_data
);
    localparam IDLE=0, ACTIVE=1, GAP=2, RECOVER=3;
    localparam [1:0] OWN_IMAGE=0, OWN_MUSIC=1, OWN_SUB=2;
    reg [1:0] state;
    reg [1:0] owner;
    reg [31:0] timer;
    reg [7:0] reset_count;
    assign client_data = card_data;
    wire active = state==ACTIVE;
    // Cancellation drains the current sector; no old image bytes after idle.
    assign image_idle = !(active && owner==OWN_IMAGE) && state!=RECOVER;
    assign image_valid = active && owner==OWN_IMAGE && card_valid;
    assign music_valid = active && owner==OWN_MUSIC && card_valid;
    assign sub_valid   = active && owner==OWN_SUB && card_valid;
    assign image_end = active && owner==OWN_IMAGE && card_end;
    assign music_end = active && owner==OWN_MUSIC && card_end;
    assign sub_end   = active && owner==OWN_SUB && card_end;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state<=IDLE; owner<=OWN_IMAGE; card_req<=0; card_lba<=0;
            timer<=0; fault<=0; card_reset<=0; reset_count<=0;
        end else begin
            fault<=0;
            case(state)
                IDLE: if(card_ready && (music_req || sub_req || image_req)) begin
                    owner<=music_req ? OWN_MUSIC : sub_req ? OWN_SUB : OWN_IMAGE;
                    card_lba<=music_req ? music_lba : sub_req ? sub_lba : image_lba;
                    card_req<=1; timer<=0; state<=ACTIVE;
                end
                ACTIVE: begin
                    if(card_end) begin card_req<=0; state<=GAP; end
                    else if(timer>=TIMEOUT_CYCLES-1) begin
                        card_req<=0; fault<=1; card_reset<=1;
                        reset_count<=0; state<=RECOVER;
                    end else timer<=timer+1'b1;
                end
                GAP: state<=IDLE;
                RECOVER: begin
                    if(&reset_count) begin card_reset<=0; state<=IDLE; end
                    else reset_count<=reset_count+1'b1;
                end
            endcase
        end
    end
endmodule
