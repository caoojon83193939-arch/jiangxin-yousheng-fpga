// FIFO level/full are already in write_clk. Only clear comes from mem_clk.
// Three settling clocks plus a registered decision isolate the scaler control.
// New transactions wait for the existing writer request/ack/settle handshake;
// abort blocks emission independently, before this status indication propagates.
module frame_write_ready(
    input wire clk,rst,fifo_clear,fifo_full,
    input wire [9:0] fifo_level,
    output reg ready
);
    (* async_reg = "true" *) reg [2:0] clear_sync;
    always @(posedge clk or posedge rst)begin
        if(rst)begin clear_sync<=3'b111;ready<=0;end
        else begin
            clear_sync<={clear_sync[1:0],fifo_clear};
            ready<=!clear_sync[2] && !fifo_full && fifo_level<10'd480;
        end
    end
endmodule
