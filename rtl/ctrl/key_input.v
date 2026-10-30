// key_input.v
// Two-stage synchronizer + debounce + press/long-press event generator.
// Button is active-low on HX4S20C (pressed = 0).
// M14: counters sized to their limits (25 MHz: 19-bit debounce, 25-bit long
// press) instead of 32 bits; behaviour is identical cycle for cycle.
module key_input #(
    parameter integer CLK_FREQ_HZ   = 100_000_000,
    parameter integer DEBOUNCE_MS   = 20,
    parameter integer LONG_PRESS_MS = 1000
)(
    input  wire clk,
    input  wire rst,            // active high
    input  wire key_n,          // raw button, active low
    output reg  press,          // 1-cycle pulse on debounced press edge
    output reg  release_short,  // 1-cycle pulse on release when no long press happened
    output reg  long_press,     // 1-cycle pulse once when held for LONG_PRESS_MS
    output reg  held            // debounced level, 1 = pressed
);
    localparam integer DEB_CYC  = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
    localparam integer LONG_CYC = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;

    reg        sync0, sync1;
    localparam integer DW = $clog2(DEB_CYC);
    localparam integer LW = $clog2(LONG_CYC);
    reg [DW-1:0] deb_cnt;
    reg [LW-1:0] hold_cnt;
    reg        long_marked;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync0 <= 1'b1;
            sync1 <= 1'b1;
        end else begin
            sync0 <= key_n;
            sync1 <= sync0;
        end
    end

    wire pressed_raw = ~sync1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            deb_cnt       <= 0;
            held          <= 1'b0;
            press         <= 1'b0;
            release_short <= 1'b0;
            long_press    <= 1'b0;
            hold_cnt      <= 0;
            long_marked   <= 1'b0;
        end else begin
            press         <= 1'b0;
            release_short <= 1'b0;
            long_press    <= 1'b0;

            // debounce: level must stay different for DEB_CYC cycles
            if (pressed_raw == held) begin
                deb_cnt <= 0;
            end else if (deb_cnt >= DEB_CYC - 1) begin
                deb_cnt <= 0;
                held    <= pressed_raw;
                if (pressed_raw) begin
                    press <= 1'b1;
                end else if (!long_marked) begin
                    release_short <= 1'b1;
                end
            end else begin
                deb_cnt <= deb_cnt + 1'b1;
            end

            // long press detection on debounced level
            if (held) begin
                if (hold_cnt >= LONG_CYC - 1) begin
                    if (!long_marked) begin
                        long_marked <= 1'b1;
                        long_press  <= 1'b1;
                    end
                end else begin
                    hold_cnt <= hold_cnt + 1'b1;
                end
            end else begin
                hold_cnt    <= 0;
                long_marked <= 1'b0;
            end
        end
    end
endmodule
