// Two-bank PCM mailbox. Only toggles cross clock domains; published RAM stays
// immutable until the reader returns ownership. Each bank holds 1024 stereo frames.
module pcm_bank_buffer #(
    parameter integer BANK_BITS = 10
)(
    input wire wr_clk, rd_clk, rst,
    input wire wr_en, flush,
    input wire [31:0] wr_data,
    output wire [BANK_BITS+1:0] free_frames,
    input wire pop,
    output wire empty,
    output reg [31:0] data,
    output reg overflow
);
    wire wr_rst,rd_rst;
    reset_sync u_wr_rst(.clk(wr_clk),.async_rst(rst),.rst(wr_rst));
    reset_sync u_rd_rst(.clk(rd_clk),.async_rst(rst),.rst(rd_rst));
    localparam integer BANK_SIZE = 1 << BANK_BITS;
    reg [31:0] mem [0:2*BANK_SIZE-1];
    reg wr_bank, rd_bank;
    reg [BANK_BITS-1:0] wp, rp;
    reg [1:0] publish, returned;
    // Bundled metadata: set before publish crosses three synchronizer stages;
    // held unchanged until returned ownership, just like the corresponding RAM.
    reg [BANK_BITS-1:0] last0, last1;
    (* async_reg = "true" *) reg [1:0] ack0, ack1, ack2;
    (* async_reg = "true" *) reg [1:0] pub0, pub1, pub2;
    wire available = publish[wr_bank] == ack2[wr_bank];
    wire other_bank = ~wr_bank;
    wire other_free = publish[other_bank] == ack2[other_bank];
    assign free_frames = available ? (BANK_SIZE - {1'b0,wp}) +
                                  (other_free ? BANK_SIZE : 0) : 0;
    assign empty = pub2[rd_bank] == returned[rd_bank];
    // Unreset synchronous read/write ports infer true dual-port block RAM.
    always @(posedge wr_clk)
        if (wr_en && available && !wr_rst) mem[{wr_bank,wp}] <= wr_data;
    always @(posedge rd_clk) data <= mem[{rd_bank,rp}];
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_bank <= 0; wp <= 0; publish <= 0; last0<=0;last1<=0;
            ack0 <= 0; ack1 <= 0; ack2 <= 0; overflow <= 0;
        end else begin
            ack0 <= returned; ack1 <= ack0; ack2 <= ack1;
            if (wr_en) begin
                if (!available) overflow <= 1;
                else if (&wp) begin
                    if(wr_bank) last1<=wp; else last0<=wp;
                    publish[wr_bank] <= ~publish[wr_bank];
                    wr_bank <= ~wr_bank; wp <= 0;
                end else wp <= wp + 1'b1;
            end else if(flush && available && wp!=0) begin
                if(wr_bank) last1<=wp-1'b1; else last0<=wp-1'b1;
                publish[wr_bank]<=~publish[wr_bank];wr_bank<=~wr_bank;wp<=0;
            end
        end
    end
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_bank <= 0; rp <= 0; returned <= 0;
            pub0 <= 0; pub1 <= 0; pub2 <= 0;
        end else begin
            pub0 <= publish; pub1 <= pub0; pub2 <= pub1;
            if (pop && !empty) begin
                if (rp == (rd_bank ? last1 : last0)) begin
                    returned[rd_bank] <= pub2[rd_bank];
                    rd_bank <= ~rd_bank; rp <= 0;
                end else rp <= rp + 1'b1;
            end
        end
    end
endmodule
