// M11 subtitle publish receiver (pixel clock domain). The loader holds
// {valid, bank, len} unchanged from its request toggle until this side
// acknowledges. The payload is taken after a three-stage request synchronizer,
// becomes active at the next falling VS (vertical blank), and only then is
// acknowledged, so the loader never writes the bank that is being displayed.
module subtitle_commit(
    input wire clk, rst, vs,
    input wire pub_req, pub_valid, pub_bank,
    input wire [5:0] pub_len,
    output reg pub_ack,
    output reg card_valid, card_bank,
    output reg [5:0] card_len
);
    (* async_reg = "true" *) reg [2:0] req_sync;
    reg pending, vs_d;
    reg p_valid, p_bank;
    reg [5:0] p_len;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            req_sync<=0; pending<=0; vs_d<=0; pub_ack<=0;
            p_valid<=0; p_bank<=0; p_len<=0;
            card_valid<=0; card_bank<=0; card_len<=0;
        end else begin
            req_sync<={req_sync[1:0],pub_req}; vs_d<=vs;
            if (!pending && req_sync[2]!=pub_ack) begin
                p_valid<=pub_valid; p_bank<=pub_bank; p_len<=pub_len; pending<=1;
            end
            if (pending && vs_d && !vs) begin
                card_valid<=p_valid; card_bank<=p_bank; card_len<=p_len;
                pending<=0; pub_ack<=req_sync[2];
            end
        end
    end
endmodule
