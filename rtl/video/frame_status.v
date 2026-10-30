// Bundled-data request/ack mailbox. Source payload is held until acknowledged;
// it is never synchronized bit-by-bit. Intermediate UI changes may coalesce.
// The receiver keeps a pending snapshot and commits only at falling VS.
module frame_status #(
    parameter WIDTH=64,
    parameter [WIDTH-1:0] RESET_VALUE=0
)(
    input wire src_clk,src_rst,
    input wire [WIDTH-1:0] source,
    input wire video_clk,video_rst,vs,
    output reg [WIDTH-1:0] active
);
    reg [WIDTH-1:0] held,pending;
    reg req,ack,vs_d;
    (* async_reg="true" *) reg [2:0] ack_sync,req_sync;
    always @(posedge src_clk or posedge src_rst)begin
        if(src_rst)begin held<=RESET_VALUE;req<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[1:0],ack};
            if(ack_sync[2]==req && source!=held)begin held<=source;req<=~req;end
        end
    end
    always @(posedge video_clk or posedge video_rst)begin
        if(video_rst)begin
            req_sync<=0;ack<=0;pending<=RESET_VALUE;active<=RESET_VALUE;vs_d<=0;
        end else begin
            req_sync<={req_sync[1:0],req};vs_d<=vs;
            if(req_sync[2]!=ack)begin pending<=held;ack<=req_sync[2];end
            if(vs_d && !vs)active<=pending;
        end
    end
endmodule
