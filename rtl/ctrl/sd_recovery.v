// M14b SD auto-recovery, pixel-clock domain. Replaces "press KEY1" for the
// three ways the card side can stay down after power-up:
//   1. Card initialisation never completes. The official sd_card_sec_read_write
//      runs the power-up sequence (74 clocks, CMD0) once after reset and then
//      only re-sends the failing command; a card that missed that window
//      never answers. After INIT_MS without sd_init_done the SD SPI stack is
//      reset, which restarts the full sequence (what KEY1 did).
//   2. The image scan failed and the controller sits in NO IMAGE (it only
//      retried on KEY2/KEY3). After NOIMG_MS: SPI stack reset, guide-reader
//      reset, then a retry event to media_ctrl (rescan once the card is up).
//      Also covers a card inserted after power-up.
//   3. The guide reader stopped in FAILED after a card fault while images
//      kept working (silent until KEY1). After READER_MS: reader reset.
// Retries repeat while the condition holds; none happen during standby
// (the card side is deliberately paused). Inputs are 2-FF synchronised
// levels; outputs are reset pulses (synchronised again by reset_sync in the
// SD domain) and one toggle, issued after the pulses have ended.
module sd_recovery #(
    parameter integer CLK_HZ = 25_000_000,
    parameter integer INIT_MS = 3000,
    parameter integer NOIMG_MS = 10000,
    parameter integer READER_MS = 10000,
    parameter integer PULSE_MS = 2
)(
    input wire clk, rst,
    input wire init_done_async, noimg_async, reader_fail_async,
    input wire standby,
    output reg stack_rst,                 // SD SPI stack reset pulse
    output reg reader_rst,                // guide reader reset pulse
    output reg retry_t,                   // media_ctrl: rescan from NO IMAGE
    output reg [7:0] recoveries           // count for the staff display
);
    (* async_reg = "true" *) reg [2:0] meta;
    reg [2:0] lvl;
    always @(posedge clk or posedge rst) begin
        if (rst) begin meta <= 0; lvl <= 0; end
        else begin meta <= {init_done_async, noimg_async, reader_fail_async}; lvl <= meta; end
    end
    wire init_done = lvl[2], noimg = lvl[1], reader_fail = lvl[0];

    localparam integer MS_CYC = CLK_HZ / 1000;
    localparam integer TW = $clog2(NOIMG_MS + 1);
    reg [$clog2(MS_CYC)-1:0] prescale;
    wire ms_tick = prescale == MS_CYC - 1;
    reg [TW-1:0] init_ms, noimg_ms, reader_ms;
    reg [$clog2(PULSE_MS + 1)-1:0] pulse_ms;
    reg pulsing, then_retry;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prescale <= 0; init_ms <= 0; noimg_ms <= 0; reader_ms <= 0; pulse_ms <= 0;
            pulsing <= 0; then_retry <= 0; stack_rst <= 0; reader_rst <= 0; retry_t <= 0;
            recoveries <= 0;
        end else begin
            prescale <= ms_tick ? 0 : prescale + 1'b1;
            if (pulsing) begin
                // Hold the resets for PULSE_MS, then release; timers restart.
                if (ms_tick) begin
                    if (pulse_ms == PULSE_MS - 1) begin
                        pulsing <= 0; stack_rst <= 0; reader_rst <= 0;
                        if (then_retry) retry_t <= ~retry_t;
                        then_retry <= 0;
                    end else pulse_ms <= pulse_ms + 1'b1;
                end
                init_ms <= 0; noimg_ms <= 0; reader_ms <= 0;
            end else if (standby) begin
                init_ms <= 0; noimg_ms <= 0; reader_ms <= 0;
            end else if (ms_tick) begin
                init_ms <= init_done ? 0 : init_ms + 1'b1;
                noimg_ms <= noimg ? noimg_ms + 1'b1 : 0;
                reader_ms <= (reader_fail && !noimg) ? reader_ms + 1'b1 : 0;
                if (noimg && noimg_ms >= NOIMG_MS - 1) begin
                    pulsing <= 1; pulse_ms <= 0; stack_rst <= 1; reader_rst <= 1; then_retry <= 1;
                    if (~&recoveries) recoveries <= recoveries + 1'b1;
                end else if (!init_done && init_ms >= INIT_MS - 1) begin
                    pulsing <= 1; pulse_ms <= 0; stack_rst <= 1;
                    if (~&recoveries) recoveries <= recoveries + 1'b1;
                end else if (reader_fail && !noimg && reader_ms >= READER_MS - 1) begin
                    pulsing <= 1; pulse_ms <= 0; reader_rst <= 1;
                    if (~&recoveries) recoveries <= recoveries + 1'b1;
                end
            end
        end
    end
endmodule
