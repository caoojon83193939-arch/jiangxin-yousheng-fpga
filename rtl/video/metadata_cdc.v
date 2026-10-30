// M13: image geometry for the header and the staff display, computed in the
// pixel domain instead of the (full) 100 MHz SD domain.
// SD side (a few registers): latch the source width/height and target bank at
// source_start; publish {bank, width, height} with a toggle when that load
// completes (load_done), drop it on abort/error. The published bundle is held
// until the next completed load, i.e. stable for seconds before it changes.
// Pixel side: the unchanged image_metadata (BCD width/height, fit scale)
// runs on the published bundle and fills that bank's slot. A slot is
// written at most ~2.6 ms (64000 divider steps at 25 MHz, width 1) after
// load_done, while media_ctrl commits the bank only after the SDRAM write and
// the transition grant (>= one frame later), so the displayed bank's
// numbers are always current when the frame is shown.
module metadata_cdc(
    input wire sd_clk, sd_rst,
    input wire start, abort_load, load_error, load_done,
    input wire [10:0] source_width, source_height,
    input wire write_bank,
    input wire video_clk, video_rst,
    input wire display_bank,                  // pixel domain, frame-aligned
    output wire [51:0] display_meta,
    output wire display_meta_valid
);
    reg [10:0] w_q, h_q;
    reg bank_q, armed, pub_t;
    reg [22:0] pub;                           // {bank, width, height}
    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin w_q <= 0; h_q <= 0; bank_q <= 0; armed <= 0; pub_t <= 0; pub <= 0; end
        else if (abort_load || load_error) armed <= 0;
        else if (start) begin
            w_q <= source_width; h_q <= source_height; bank_q <= write_bank; armed <= 1;
        end else if (load_done && armed) begin
            pub <= {bank_q, w_q, h_q}; pub_t <= ~pub_t; armed <= 0;
        end
    end

    (* async_reg = "true" *) reg pub_meta;
    reg [1:0] pub_sync;
    reg [22:0] pub_v;
    reg go;
    always @(posedge video_clk or posedge video_rst) begin
        if (video_rst) begin pub_meta <= 0; pub_sync <= 0; pub_v <= 0; go <= 0; end
        else begin
            pub_meta <= pub_t;
            pub_sync <= {pub_sync[0], pub_meta};
            go <= pub_sync[1] ^ pub_sync[0];
            if (pub_sync[1] ^ pub_sync[0]) pub_v <= pub;   // stable since long before
        end
    end
    // load_done=1: the bundle is only published for a completed load, so the
    // result is stored as soon as the conversion reaches READY.
    image_metadata u_meta(.clk(video_clk),.rst(video_rst),.start(go),
        .abort_load(1'b0),.load_error(1'b0),.load_done(1'b1),
        .source_width(pub_v[21:11]),.source_height(pub_v[10:0]),
        .write_bank(pub_v[22]),.display_bank(display_bank),
        .display_meta(display_meta),.display_meta_valid(display_meta_valid));
endmodule
