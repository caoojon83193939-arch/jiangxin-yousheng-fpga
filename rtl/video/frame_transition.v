// Transition through black. All state/alpha changes occur in vertical
// blank (falling VS). The four-phase request/grant handshake crosses to the
// SD controller: request -> black/grant -> buffer commit/request low -> release.
// Keep black for two more frame boundaries after the commit is observed so
// the SDRAM reader has latched the new base and discarded the old FIFO data.
// M10 graded presentation (EXIT_FADE=1): an alarm cuts in at once (the OSD
// alarm page never waits for a fade). Under the full-screen alarm page the
// photograph is held black, so when the page is withdrawn mid-frame nothing
// flashes; the exhibit then returns from black.
// M13 styles. The frame buffer is read once per pixel, so there is no second
// image to slide or cross-dissolve (two 640x480@60 streams need ~147 MB/s,
// the 31.25 MHz x32 SDRAM peaks at 125 MB/s). Instead the old image leaves
// and the new one arrives through black with a shaped, 32-pixel soft edge:
//   0 FADE   uniform gain alpha/32 (identical to M10)             tour, reload
//   1 SCROLL opens from the centre column like a hand scroll, with   visitor
//            two wooden rollers riding on the edges                 navigation
//   2 IRIS   circular aperture from the centre, like a shadow-play  first image,
//            lamp coming up                                         alarm release
// The style is sampled when a transition starts (style_async is quasi-static:
// the controller sets it at load start, long before the swap request).
// Pixel path latency: three clocks, including HS/VS/DE. OSD must follow.
module frame_transition #(
    parameter EXIT_FADE = 0
)(
    input wire clk, rst,
    input wire swap_req_async,
    input wire [1:0] style_async,
    input wire display_valid,
    input wire emergency,
    output reg swap_grant,
    output wire busy,
    output wire [5:0] alpha_debug,
    input wire [23:0] rgb_i,
    input wire hs_i, vs_i, de_i,
    output reg [23:0] rgb_o,
    output reg hs_o, vs_o, de_o
);
    localparam [2:0] IDLE=0, FADE_OUT=1, WAIT_COMMIT=2,
                     SETTLE=3, FADE_IN=4;
    localparam [1:0] FADE=0, SCROLL=1, IRIS=2;
    reg [2:0] state;
    reg [2:0] req_sync;
    reg [1:0] style_meta, style_sync, style;
    reg vs_d;
    reg [5:0] alpha; // progress 0..32; 32 = image fully visible everywhere
    reg settle_frame;
    reg emergency_d;
    wire frame_tick = vs_d && !vs_i;
    assign busy = (state != IDLE) || req_sync[2];
    assign alpha_debug = alpha;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            req_sync <= 0;
            style_meta <= 0; style_sync <= 0;
            vs_d <= 0;
        end else begin
            req_sync <= {req_sync[1:0], swap_req_async};
            style_meta <= style_async; style_sync <= style_meta;
            vs_d <= vs_i;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            alpha <= 0;
            swap_grant <= 0;
            settle_frame <= 0;
            emergency_d <= 0;
            style <= IRIS;
        end else if (frame_tick) begin
            emergency_d <= emergency;
            case (state)
                IDLE: begin
                    swap_grant <= 0;
                    if (req_sync[2]) begin
                        style <= style_sync;
                        if (!display_valid || emergency || alpha == 0) begin
                            alpha <= 0;
                            swap_grant <= 1;
                            state <= WAIT_COMMIT;
                        end else begin
                            alpha <= 31;
                            state <= FADE_OUT;
                        end
                    end else if (EXIT_FADE && emergency) alpha <= 0;
                    else if (EXIT_FADE && emergency_d && display_valid) begin
                        // Alarm released: the page behind it returns softly.
                        alpha <= 0;
                        style <= IRIS;
                        state <= FADE_IN;
                    end else alpha <= display_valid ? 6'd32 : 6'd0;
                end
                FADE_OUT: begin
                    // Aborted load/request: recover the old image, no swap.
                    if (!req_sync[2]) state <= FADE_IN;
                    else if (alpha <= 1 || emergency) begin
                        alpha <= 0;
                        swap_grant <= 1;
                        state <= WAIT_COMMIT;
                    end else alpha <= alpha - 1'b1;
                end
                WAIT_COMMIT: begin
                    alpha <= 0;
                    if (!req_sync[2]) begin
                        swap_grant <= 0;
                        settle_frame <= 0;
                        state <= SETTLE;
                    end
                end
                SETTLE: begin
                    alpha <= 0;
                    if (settle_frame) state <= FADE_IN;
                    else settle_frame <= 1;
                end
                FADE_IN: begin
                    if (alpha >= 31 || emergency) begin
                        alpha <= display_valid ? 6'd32 : 6'd0;
                        state <= IDLE;
                    end else alpha <= alpha + 1'b1;
                end
                default: begin
                    state <= IDLE;
                    alpha <= 0;
                    swap_grant <= 0;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Stage 1: distance of the pixel from the screen centre (320, 240).
    //   SCROLL: |dx|                         0..320, edge at alpha*11
    //   IRIS:   max(M, 13/16 M + 19/32 m, 31/32 M + 9/32 m) of M=max, m=min of
    //           |dx|,|dy|: a 3-segment Euclidean estimate within +-0.9 %,
    //           so the aperture is visibly round. 0..403, edge at alpha*14
    // At alpha=32 both edges clear the farthest pixel by >= 32 (full gain);
    // at alpha=0 the gain is 0 everywhere, so the grant is always on black.
    // ------------------------------------------------------------------
    wire [9:0] x, y;
    video_coord coord(.clk(clk),.rst(rst),.vs(vs_i),.de(de_i),.x(x),.y(y),.frame_tick());
    reg [23:0] rgb0, rgb1;
    reg hs0, vs0, de0, hs1, vs1, de1;
    wire [9:0] dx = x >= 10'd320 ? x - 10'd320 : 10'd320 - x;
    wire [9:0] dy = y >= 10'd240 ? y - 10'd240 : 10'd240 - y;
    wire [9:0] dmax = dx >= dy ? dx : dy;
    wire [9:0] dmin = dx >= dy ? dy : dx;
    wire [9:0] seg1 = dmax - (dmax >> 3) - (dmax >> 4) + (dmin >> 1) + (dmin >> 4) + (dmin >> 5);
    wire [9:0] seg2 = dmax - (dmax >> 5) + (dmin >> 2) + (dmin >> 5);
    wire [9:0] seg12 = seg1 >= seg2 ? seg1 : seg2;
    wire [9:0] round_dist = seg12 >= dmax ? seg12 : dmax;
    reg [9:0] dist1;
    reg [1:0] style1;
    reg [5:0] alpha1;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb0 <= 0; rgb1 <= 0; dist1 <= 0; style1 <= 0; alpha1 <= 0;
            hs0 <= 0; vs0 <= 0; de0 <= 0; hs1 <= 0; vs1 <= 0; de1 <= 0;
        end else begin
            rgb0 <= rgb_i; hs0 <= hs_i; vs0 <= vs_i; de0 <= de_i;
            // rgb0 is aligned with the registered coordinates x, y.
            rgb1 <= rgb0; hs1 <= hs0; vs1 <= vs0; de1 <= de0;
            dist1 <= style == IRIS ? round_dist : dx;
            style1 <= style;
            alpha1 <= alpha;
        end
    end

    // ------------------------------------------------------------------
    // Stage 2: per-pixel gain = clamp(edge - distance, 0, 32), then scale.
    // FADE keeps the M10 uniform gain, so style 0 is M10 plus two clocks.
    // SCROLL rollers: an 8 px wooden bar just outside the soft edge, drawn
    // only while the scroll is partly open.
    // ------------------------------------------------------------------
    wire [9:0] edge_pos = style1 == IRIS ? {alpha1, 3'b0} + {alpha1, 2'b0} + {alpha1, 1'b0}
                                     : {alpha1, 3'b0} + {alpha1, 1'b0} + {4'd0, alpha1};
    wire [10:0] margin = {1'b0, edge_pos} - {1'b0, dist1};
    wire [5:0] shaped = margin[10] ? 6'd0 : (margin[9:0] >= 10'd32 ? 6'd32 : margin[5:0]);
    wire [5:0] gain = style1 == FADE ? alpha1 : shaped;
    wire [10:0] past = {1'b0, dist1} - {1'b0, edge_pos};
    wire roller = style1 == SCROLL && alpha1 != 0 && alpha1 != 32 && !past[10] && past[9:0] < 10'd8;
    wire [23:0] wood = past[2:0] < 3'd2 ? 24'hC89B5E : 24'h6B4526;
    wire [13:0] r_scaled = rgb1[23:16] * gain;
    wire [13:0] g_scaled = rgb1[15:8] * gain;
    wire [13:0] b_scaled = rgb1[7:0] * gain;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb_o <= 0;
            hs_o <= 0; vs_o <= 0; de_o <= 0;
        end else begin
            rgb_o <= roller ? wood : {r_scaled[12:5], g_scaled[12:5], b_scaled[12:5]};
            hs_o <= hs1; vs_o <= vs1; de_o <= de1;
        end
    end
endmodule
