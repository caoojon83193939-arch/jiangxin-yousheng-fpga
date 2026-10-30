// video_coord.v
// Pixel coordinate counters derived from hs/vs/de of the timed video stream.
// x counts active pixels in a line, y counts active lines in a frame.
// frame_tick pulses once per frame (at the first active pixel).
module video_coord #(
    parameter integer H_ACTIVE = 640,
    parameter integer V_ACTIVE = 480
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       vs,
    input  wire       de,
    output reg  [9:0] x,
    output reg  [9:0] y,
    output reg        frame_tick
);
    reg de_d, vs_d, new_frame;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            de_d <= 1'b0; vs_d <= 1'b0; new_frame <= 1'b1;
            x <= 10'd0; y <= 10'd0; frame_tick <= 1'b0;
        end else begin
            de_d <= de;
            vs_d <= vs;
            frame_tick <= 1'b0;
            if (vs != vs_d) new_frame <= 1'b1;          // any vsync edge arms a new frame
            if (de) begin
                if (!de_d) begin                          // first pixel of a line
                    x <= 10'd0;
                    if (new_frame) begin
                        y <= 10'd0;
                        new_frame <= 1'b0;
                        frame_tick <= 1'b1;
                    end else if (y != V_ACTIVE - 1) begin
                        y <= y + 10'd1;
                    end
                end else begin
                    x <= x + 10'd1;
                end
            end
        end
    end
endmodule
