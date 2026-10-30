// Assert immediately; deassert only after three local rising edges.
module reset_sync(input wire clk,async_rst,output wire rst);
    (* async_reg="true" *) reg [2:0] pipe=3'b111;
    always @(posedge clk or posedge async_rst)begin
        if(async_rst)pipe<=3'b111;else pipe<={pipe[1:0],1'b0};
    end
    assign rst=pipe[2];
endmodule
