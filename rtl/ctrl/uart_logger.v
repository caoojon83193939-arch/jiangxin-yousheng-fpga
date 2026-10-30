// M15 UART event log (pixel-clock domain), 115200 8N1 on the CH340 (D12).
// One fixed-length text line per event:
//   [000012.345] SHOW  02 \r\n       (24 characters, ~2.1 ms at 115200 baud)
// = seconds.milliseconds since reset, a 5-letter name, a hex argument, and
// '!' instead of the last space if that source fired again before its first
// event could be queued (the first argument is kept).
// Structure (sized for the nearly full device): each source only has a
// pending bit and its argument; one source per clock (lowest index first)
// moves into a 4-entry queue and is time-stamped there. While the queue has
// room that is within N clocks of the event (under 0.5 us at 25 MHz, far
// below the 1 ms resolution); only a burst of more than four events waits
// for a line to finish. One 32-bit stamp per queue entry instead of one per
// source, and a 4:1 instead of a 12:1 read multiplexer.
// Sources (bit index of ev_valid):
//   0 BOOT  1 CARD  2 STATE 3 LOAD  4 SHOW  5 AUDIO 6 KEY   7 ALARM
//   8 STBY  9 RECOV 10 IMERR 11 AUERR
module uart_logger #(
    parameter integer CLK_HZ = 25_000_000,
    parameter integer BAUD = 115200,
    parameter integer N = 12
)(
    input wire clk, rst,
    input wire [31:0] ms,
    input wire [N-1:0] ev_valid,
    input wire [8*N-1:0] ev_arg,
    output reg tx
);
    integer i;
    // ---------------- pending sources ----------------
    reg [N-1:0] pending, lost;
    reg [7:0] arg [0:N-1];
    reg [3:0] pick;
    reg pick_ok;
    always @(*) begin
        pick = 0; pick_ok = 0;
        for (i = N - 1; i >= 0; i = i - 1)
            if (pending[i]) begin pick = i; pick_ok = 1; end
    end
    // ---------------- 4-entry queue ----------------
    reg [44:0] q [0:3];            // {lost, code[3:0], arg[7:0], stamp[31:0]}
    reg [1:0] q_wr, q_rd;
    reg [2:0] q_count;
    wire push = pick_ok && q_count != 4;
    reg pop;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pending <= 0; lost <= 0; q_wr <= 0; q_rd <= 0; q_count <= 0;
            for (i = 0; i < N; i = i + 1) arg[i] <= 0;
            for (i = 0; i < 4; i = i + 1) q[i] <= 0;
        end else begin
            for (i = 0; i < N; i = i + 1) begin
                if (push && pick == i) begin
                    // Moved to the queue; a same-cycle event starts afresh.
                    pending[i] <= ev_valid[i]; lost[i] <= 0;
                    if (ev_valid[i]) arg[i] <= ev_arg[8*i +: 8];
                end else if (ev_valid[i]) begin
                    if (!pending[i]) begin pending[i] <= 1; arg[i] <= ev_arg[8*i +: 8]; end
                    else lost[i] <= 1;
                end
            end
            if (push) begin q[q_wr] <= {lost[pick], pick, arg[pick], ms}; q_wr <= q_wr + 1'b1; end
            if (pop) q_rd <= q_rd + 1'b1;
            q_count <= q_count + (push ? 3'd1 : 3'd0) - (pop ? 3'd1 : 3'd0);
        end
    end
    // ---------------- line formatter ----------------
    localparam integer LAST = 23;
    localparam [1:0] IDLE = 0, CONV = 1, SEND = 2;
    reg [1:0] state;
    reg [3:0] code;
    reg [7:0] cur_arg;
    reg cur_lost;
    reg [31:0] bin;
    reg [35:0] bcd;                // 9 BCD digits of the millisecond count
    reg [4:0] step, pos;
    reg tx_start;
    reg [7:0] tx_data;
    reg busy;
    function [35:0] add3;
        input [35:0] v;
        integer k;
        begin
            add3 = v;
            for (k = 0; k < 9; k = k + 1)
                if (v[4*k +: 4] >= 5) add3[4*k +: 4] = v[4*k +: 4] + 4'd3;
        end
    endfunction
    function [7:0] hexc;
        input [3:0] v;
        hexc = v < 10 ? 8'd48 + v : 8'd55 + v;
    endfunction
    function [39:0] name;
        input [3:0] c;
        case (c)
            0: name = "BOOT "; 1: name = "CARD "; 2: name = "STATE"; 3: name = "LOAD ";
            4: name = "SHOW "; 5: name = "AUDIO"; 6: name = "KEY  "; 7: name = "ALARM";
            8: name = "STBY "; 9: name = "RECOV"; 10: name = "IMERR"; 11: name = "AUERR";
            default: name = "?????";
        endcase
    endfunction
    wire [39:0] nm = name(code);
    wire [35:0] bcd_adj = add3(bcd);
    reg [7:0] ch;
    always @(*) begin
        case (pos)
            0: ch = "[";
            1: ch = 8'd48 + bcd[35:32]; 2: ch = 8'd48 + bcd[31:28]; 3: ch = 8'd48 + bcd[27:24];
            4: ch = 8'd48 + bcd[23:20]; 5: ch = 8'd48 + bcd[19:16]; 6: ch = 8'd48 + bcd[15:12];
            7: ch = ".";
            8: ch = 8'd48 + bcd[11:8]; 9: ch = 8'd48 + bcd[7:4]; 10: ch = 8'd48 + bcd[3:0];
            11: ch = "]"; 12: ch = " ";
            13: ch = nm[39:32]; 14: ch = nm[31:24]; 15: ch = nm[23:16]; 16: ch = nm[15:8]; 17: ch = nm[7:0];
            18: ch = " ";
            19: ch = hexc(cur_arg[7:4]); 20: ch = hexc(cur_arg[3:0]);
            21: ch = cur_lost ? "!" : " ";
            22: ch = 8'd13;
            default: ch = 8'd10;
        endcase
    end
    wire [44:0] head = q[q_rd];
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE; pop <= 0; code <= 0; cur_arg <= 0; cur_lost <= 0;
            bin <= 0; bcd <= 0; step <= 0; pos <= 0; tx_start <= 0; tx_data <= 8'hFF;
        end else begin
            tx_start <= 0;
            pop <= 0;
            case (state)
                IDLE: if (q_count != 0 && !pop) begin
                    {cur_lost, code, cur_arg, bin} <= head;
                    bcd <= 0; step <= 0; pop <= 1; state <= CONV;
                end
                CONV: begin
                    bcd <= {bcd_adj[34:0], bin[31]};
                    bin <= {bin[30:0], 1'b0};
                    if (step == 31) begin pos <= 0; state <= SEND; end
                    step <= step + 1'b1;
                end
                SEND: if (!busy && !tx_start) begin
                    tx_data <= ch; tx_start <= 1;
                    if (pos == LAST) state <= IDLE;
                    else pos <= pos + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end
    // ---------------- 8N1 transmitter ----------------
    localparam integer DIV = (CLK_HZ + BAUD / 2) / BAUD;
    reg [$clog2(DIV)-1:0] baud_cnt;
    reg [3:0] bit_idx;
    reg [9:0] shift;
    always @(posedge clk or posedge rst) begin
        if (rst) begin tx <= 1; baud_cnt <= 0; bit_idx <= 0; shift <= 10'h3FF; busy <= 0; end
        else if (!busy) begin
            tx <= 1;
            if (tx_start) begin shift <= {1'b1, tx_data, 1'b0}; busy <= 1; baud_cnt <= 0; bit_idx <= 0; end
        end else begin
            tx <= shift[0];
            if (baud_cnt == DIV - 1) begin
                baud_cnt <= 0; shift <= {1'b1, shift[9:1]};
                if (bit_idx == 9) busy <= 0; else bit_idx <= bit_idx + 1'b1;
            end else baud_cnt <= baud_cnt + 1'b1;
        end
    end
endmodule
