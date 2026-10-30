// M15: service tools (see docs/37, docs/38).
//   * Volume 8 levels (maintenance view: KEY2 short +, KEY3 short -); the
//     alarm is never scaled; the spectrum analyses the pre-volume samples.
//   * KEY2 page turn on the short release (holding KEY2 for standby no
//     longer turns a page); the KEY2 press that clears an alarm is consumed.
//   * Alarm buzzer on H11 (two-tone square wave in step with the HDMI siren).
//   * Run-time clock: maintenance-view third row "UP hh:mm:ss VOL v/8 RCV n"
//     and staff-display pages 5 (volume) / 6 (run time).
//   * UART event log on D12 (115200 8N1) with millisecond time stamps.
//   * Status LEDs follow the staff-display idle (errors and alarms stay lit).
//   * fat32_bmp_source: per-byte pixel decisions registered one state early.
// M14b: SD auto-recovery (sd_recovery): card init watchdog, NO IMAGE retry,
//   guide-reader retry, so a cold start never needs KEY1; wake keeps the
//   picture black 4 s (measured monitor resync ~3 s) before the iris.
// M14: low power and small logic optimisations (see docs/35, docs/36).
//   * Closing-hours standby: KEY2+KEY3 held 2 s. HDMI serialiser held in
//     reset (monitor sleeps), card reading paused, seven-segment and LEDs
//     off (LED1 slow heartbeat). Any key wakes; the exhibit returns through
//     the iris after 2 s of black while the monitor re-synchronises.
//   * Seven-segment display dark after 60 s without activity in the
//     exhibition view; spectrum FFT stops while nothing is audible.
//   * Maintenance view: KEY4 short cycles tour off / content / >=30 s / >=60 s.
//   * Glyph ROM in block RAM; counters sized to their limits.
// M13: product polish (see docs/34).
//   * Guided tour from power-up: every exhibit plays its narration, the page
//     advances when narration and caption have both finished (tour_ctrl);
//     manual browsing returns to the tour after 60 s without a key.
//   * SW2 selects exhibition (down: poster + subtitles + spectrum only, the
//     header pops up for a few seconds after a setting change; always
//     bilinear; silent without a card) or maintenance (up: header rows
//     permanently shown, SW4 nearest/bilinear A/B, 1 kHz diagnostic tone). The old "all OSD off"
//     mode is gone; spectrum and subtitles are part of the product page.
//   * Transitions: scroll for visitor navigation, fade for the tour, iris for
//     the first image and the alarm release.
//   * Eight-digit staff display (seg_pages): status / resolution / scaling /
//     brightness-contrast / spectrum pages.
//   * Keys, SW4 debounce, emergency and brightness state, tour pacing and the
//     staff display moved from the full 100 MHz SD domain to the 25 MHz pixel
//     clock (PLL-compensated, unlike the R7 pad clock, whose fabric skew
//     broke hold on every crossing); the SD domain receives toggles and
//     quasi-static levels through register-only synchronisers.
// M12: image quality. SW4 selects bilinear (on) or center-aligned nearest
//   (off) scaling in bmp_bilinear_scaler; flipping it re-loads the displayed
//   exhibit so both can be compared live. The parameter line shows BIL / NN.
// M11: card subtitles. SUB01..SUB04.BIN on the TF card replace the on-chip
//   ticker for the displayed exhibit (content changes without re-synthesis);
//   no/invalid page -> the M10 ROM ticker. Alarm text stays on-chip.
// M10: exhibit guide audio, test-tone fallback, graded alarm presentation
// and a truthful header status field, on top of the board-verified M9b.
//   Manual browsing plays NARR0n.WAV for the displayed exhibit (then MUSIC);
//   the auto slideshow plays MUSIC.WAV; no playable WAV -> 1 kHz test tone.
//   Alarm: instant on-chip page + red spectrum of the siren; release fades in.
// M9: actual BMP metadata and frame-atomic status/brightness/contrast.
// Inherits M8b drained cancellation, SDRAM alignment and forced emergency OSD
//   video + interaction + HDMI 1.4b audio + OSD ticker + brightness/contrast + emergency mode
// Board: HX4S20C (EG4S20BG256), HDMI_B connector (video + audio).
//   KEY1 = reset                     KEY2 = next image / (emergency) exit
//   KEY3 = previous image            KEY4 short = auto-play on/off, long (1 s) = EMERGENCY
//   SW1 = mute   SW2 = maintenance view   SW3 = contrast boost   SW4 = bilinear
//   Brightness: KEY3 long press cycles 4 levels
//   LED1 = display valid  LED2 = auto-play  LED3 = HDMI locked  LED4 = error / emergency blink
module top(
    input  wire        clk,          // 50 MHz
    input  wire        rst_n,        // KEY1
    input  wire        key_next_n,   // KEY2
    input  wire        key_prev_n,   // KEY3
    input  wire        key_auto_n,   // KEY4
    input  wire        sw_mute,      // SW1
    input  wire        sw_maint,     // SW2: 1 = maintenance view
    input  wire        sw_contrast,  // SW3
    input  wire        sw_smooth,    // SW4: bilinear scaling
    output wire [3:0]  led,
    output wire [7:0]  seg_sel,
    output wire [7:0]  seg_data,
    output wire        uart_tx,      // CH340 RXD (FPGA TXD, D12)
    output wire        buzzer_n,     // board buzzer, low = on (SW5 cuts it)
    // HDMI B (TMDS + DDC)
    output wire        HDMI_CLK_P,
    output wire        HDMI_D2_P,
    output wire        HDMI_D1_P,
    output wire        HDMI_D0_P,
    output wire        HDMI_DDC_SCL,
    inout  wire        HDMI_DDC_SDA,
    // TF card (SPI mode)
    output wire        sd_ncs,
    output wire        sd_dclk,
    output wire        sd_mosi,
    input  wire        sd_miso
);
    localparam MEM_DATA_BITS = 32;
    localparam ADDR_BITS     = 21;
    localparam [23:0] FRAME_PIXELS = 24'd307200;   // 640*480
    localparam [23:0] BUF0_ADDR    = 24'd0;
    localparam [23:0] BUF1_ADDR    = FRAME_PIXELS;
    localparam integer SD_CLK_HZ   = 100_000_000;
    localparam integer PIX_CLK_HZ  = 25_000_000;

    // ------------------------------------------------------------------
    // clocks + power-on reset (PLLs locked 20 ms)
    // ------------------------------------------------------------------
    wire sd_card_clk, ext_mem_clk, ext_mem_clk_sft, video_clk, hdmi_5x_clk;
    wire sys_pll_lock, video_pll_lock;

    sys_pll sys_pll_m0(.refclk(clk), .reset(1'b0), .extlock(sys_pll_lock),
        .clk0_out(sd_card_clk), .clk1_out(ext_mem_clk), .clk2_out(ext_mem_clk_sft));
    video_pll video_pll_m0(.refclk(clk), .reset(1'b0), .extlock(video_pll_lock),
        .clk0_out(video_clk), .clk1_out(hdmi_5x_clk));

    (* async_reg = "true" *) reg [1:0] sys_lock_sync=0,video_lock_sync=0;
    always @(posedge clk) begin
        sys_lock_sync<={sys_lock_sync[0],sys_pll_lock};
        video_lock_sync<={video_lock_sync[0],video_pll_lock};
    end
    reg [19:0] por_cnt     = 20'd0;
    reg        rst_all_reg = 1'b1;
    always @(posedge clk) begin
        if (!rst_n || !sys_lock_sync[1] || !video_lock_sync[1]) begin
            por_cnt <= 20'd0; rst_all_reg <= 1'b1;
        end else if (!rst_all_reg) begin
            rst_all_reg <= 1'b0;
        end else if (sys_lock_sync[1] && video_lock_sync[1]) begin
            if (por_cnt < 20'd1_000_000) begin por_cnt <= por_cnt + 1'b1; rst_all_reg <= 1'b1; end
            else rst_all_reg <= 1'b0;
        end else begin
            por_cnt <= 20'd0; rst_all_reg <= 1'b1;
        end
    end
    wire rst = rst_all_reg;
    wire rst_sd,rst_mem,rst_video;
    reset_sync u_rst_sd(.clk(sd_card_clk),.async_rst(rst),.rst(rst_sd));
    // Auto-recovery resets: asserted from the pixel domain, released in sync.
    wire stack_rst_v, reader_rst_v, rst_stack, rst_reader;
    reset_sync u_rst_stack(.clk(sd_card_clk),.async_rst(rst | stack_rst_v),.rst(rst_stack));
    reset_sync u_rst_reader(.clk(sd_card_clk),.async_rst(rst | reader_rst_v),.rst(rst_reader));
    reset_sync u_rst_mem(.clk(ext_mem_clk),.async_rst(rst),.rst(rst_mem));
    reset_sync u_rst_video(.clk(video_clk),.async_rst(rst),.rst(rst_video));

    // ------------------------------------------------------------------
    // keys, switches and player mode in the 25 MHz pixel-clock domain
    // ------------------------------------------------------------------
    wire ev_next, ev_next_short, ev_prev, ev_prev_long, ev_auto_short, ev_auto_long;
    wire press3, press4, held2, held3, held4;
    key_input #(.CLK_FREQ_HZ(PIX_CLK_HZ)) u_key_next(
        .clk(video_clk), .rst(rst_video), .key_n(key_next_n),
        .press(ev_next), .release_short(ev_next_short), .long_press(), .held(held2));
    key_input #(.CLK_FREQ_HZ(PIX_CLK_HZ)) u_key_prev(
        .clk(video_clk), .rst(rst_video), .key_n(key_prev_n),
        .press(press3), .release_short(ev_prev), .long_press(ev_prev_long), .held(held3));
    key_input #(.CLK_FREQ_HZ(PIX_CLK_HZ)) u_key_auto(
        .clk(video_clk), .rst(rst_video), .key_n(key_auto_n),
        .press(press4), .release_short(ev_auto_short), .long_press(ev_auto_long), .held(held4));

    // Power management: standby, key masking, staff-display idle.
    wire maint_p;
    wire standby, dark, key_block, seg_on, setting_change, tour_on;
    wire user_activity = ev_next | ev_next_short | press3 | press4 | ev_prev | ev_prev_long | ev_auto_short | ev_auto_long;
    wire emergency_b;
    power_ctrl #(.CLK_HZ(PIX_CLK_HZ),.WAKE_DARK_MS(4000)) u_power(.clk(video_clk),.rst(rst_video),
        .key2_held(held2),.key3_held(held3),.key4_held(held4),
        .any_press(ev_next | press3 | press4),.activity(user_activity | setting_change),
        .emergency(emergency_b),.maint(maint_p),
        .standby(standby),.dark(dark),.key_block(key_block),.seg_on(seg_on));

    // SW2/SW4: the exhibition view always shows bilinear and never plays the
    // test tone; SW4 A/B and the test tone exist only in the maintenance view.
    wire smooth_b, reload_t;
    presentation_policy #(.CLK_HZ(PIX_CLK_HZ)) u_policy(.clk(video_clk),.rst(rst_video),
        .sw_maint(sw_maint),.sw_smooth(sw_smooth),.maint(maint_p),.smooth(smooth_b),.reload_t(reload_t));
    // Key mapping (both views, alarm, volume, tour period): key_actions.v.
    wire [1:0] bright_b, dwell_sel;
    wire [3:0] volume;
    wire next_t, prev_t, auto_t, key_log_valid;
    wire [3:0] key_log_arg;
    key_actions u_keys(.clk(video_clk),.rst(rst_video),
        .k2_press(ev_next),.k2_short(ev_next_short),.k2_held(held2),
        .k3_short(ev_prev),.k3_long(ev_prev_long),
        .k4_short(ev_auto_short),.k4_long(ev_auto_long),
        .maint(maint_p),.key_block(key_block),.tour_on(tour_on),
        .emergency(emergency_b),.bright(bright_b),.volume(volume),.dwell_sel(dwell_sel),
        .next_t(next_t),.prev_t(prev_t),.auto_t(auto_t),
        .log_valid(key_log_valid),.log_arg(key_log_arg));

    // Pixel clock -> SD domain: register-only synchronisers.
    wire advance_t, auto_on_t, retry_t;
    (* async_reg = "true" *) reg [6:0] tog_meta;
    reg [13:0] tog_sync;
    (* async_reg = "true" *) reg [4:0] lvl_meta;
    reg [4:0] lvl_sd;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin tog_meta<=0; tog_sync<=0; lvl_meta<=0; lvl_sd<=0; end
        else begin
            tog_meta <= {retry_t, next_t, prev_t, auto_t, reload_t, advance_t, auto_on_t};
            tog_sync <= {tog_sync[6:0], tog_meta};
            lvl_meta <= {standby, emergency_b, bright_b, smooth_b};
            lvl_sd   <= lvl_meta;
        end
    end
    wire [6:0] tog_pulse = tog_sync[13:7] ^ tog_sync[6:0];
    wire ev_retry = tog_pulse[6];
    wire ev_next_ctrl = tog_pulse[5], ev_prev_ctrl = tog_pulse[4], ev_auto_ctrl = tog_pulse[3];
    wire ev_smooth_reload = tog_pulse[2], ev_advance = tog_pulse[1], ev_auto_on = tog_pulse[0];
    wire emergency = lvl_sd[3];
    // Standby pauses the SD side exactly like an alarm (no loads, guide audio
    // paused, page and tour state kept), without the alarm presentation.
    wire quiet = emergency | lvl_sd[4];
    wire [1:0] bright_lvl = lvl_sd[2:1];
    wire smooth_sd = lvl_sd[0];

    // ------------------------------------------------------------------
    // TF card SPI stack (official) + BMP loader + player control
    // ------------------------------------------------------------------
    wire        sd_init_done, sd_sec_read, sd_sec_read_data_valid, sd_sec_read_end;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;
    wire image_req, image_valid, image_end, music_req, music_valid, music_end;
    wire [31:0] image_lba, music_lba;
    wire card_fault, card_reset;
    wire [7:0] client_data;
    wire pcm_wr, pcm_flush, music_ready, pcm_empty, pcm_pop, pcm_overflow, audio_underrun;
    wire music_fault;
    wire music_active;
    wire audio_valid;
    wire [23:0] audio_left, audio_right;
    wire [31:0] pcm_write_data, pcm_read_data;
    wire [11:0] pcm_free;
    wire [7:0] music_error;
    wire image_idle, writer_abort_ack;
    // Controller outputs used by the guide reader (declared before use).
    wire [1:0] img_role;
    wire display_valid, auto_en;
    wire tone_fallback;
    wire [1:0] track_kind;
    wire guide_done;
    wire [1:0] load_kind;
    wire load_abort,load_busy,sd_card_write_req_ack;
    reg [2:0] abort_ack_sync, write_ack_sync;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if(rst_sd) begin abort_ack_sync<=0;write_ack_sync<=0;end
        else begin
            abort_ack_sync<={abort_ack_sync[1:0],writer_abort_ack};
            write_ack_sync<={write_ack_sync[1:0],sd_card_write_req_ack};
        end
    end
    wire pipeline_idle=image_idle && !load_busy && !abort_ack_sync[2];
    wire abort_done=image_idle && !load_busy && abort_ack_sync[2];
    // Card subtitle client (declared here, loader below).
    wire sub_req, sub_valid, sub_end;
    wire [31:0] sub_lba;
    wire sub_ready;
    wire [3:0] sub_present;
    wire [1:0] sub_page;
    wire [31:0] sub_first_lba;
    sd_sector_arbiter3 u_sd_arb(.clk(sd_card_clk),.rst(rst_sd),.card_ready(sd_init_done),
        .image_req(image_req && !load_abort),.image_lba(image_lba),
        .music_req(music_req && !quiet),.music_lba(music_lba),
        .sub_req(sub_req),.sub_lba(sub_lba),
        .image_idle(image_idle), .image_valid(image_valid),.image_end(image_end),
        .music_valid(music_valid),.music_end(music_end),
        .sub_valid(sub_valid),.sub_end(sub_end),.client_data(client_data),
        .fault(card_fault),.card_reset(card_reset),
        .card_req(sd_sec_read),.card_lba(sd_sec_read_addr),
        .card_valid(sd_sec_read_data_valid),.card_end(sd_sec_read_end),.card_data(sd_sec_read_data));
    // Manual browsing = visitor chooses an exhibit -> its narration.
    fat32_guide_reader u_music(.clk(sd_card_clk),.rst(rst_reader),.enable(sd_init_done),
        .pause(quiet),.guide_mode(1'b1),.tour(auto_en),.page_valid(display_valid),.page(img_role),
        .free_frames(pcm_free),.pcm_wr(pcm_wr),.pcm_flush(pcm_flush),.pcm_data(pcm_write_data),
        .ready(music_ready),.track_kind(track_kind),.guide_done(guide_done),.tone_fallback(tone_fallback),
        .error_code(music_error),.sector_req(music_req),.sector_lba(music_lba),
        .sector_data(client_data),.sector_valid(music_valid),.sector_end(music_end),.sector_fault(card_fault),
        .sub_ready(sub_ready),.sub_present(sub_present),.sub_page(sub_page),
        .sub_lba_q(sub_first_lba));
    wire sub_wr_en, sub_wr_text, sub_wr_bank;
    wire [9:0] sub_wr_addr;
    wire [15:0] sub_wr_data;
    wire sub_pub_req, sub_pub_valid, sub_pub_bank, sub_pub_ack;
    wire [5:0] sub_pub_len;
    subtitle_loader u_sub_loader(.clk(sd_card_clk),.rst(rst_sd),
        .sub_ready(sub_ready),.sub_present(sub_present),.sub_page(sub_page),
        .sub_lba_q(sub_first_lba),
        .page_valid(display_valid),.page(img_role),
        .sector_req(sub_req),.sector_lba(sub_lba),.sector_data(client_data),
        .sector_valid(sub_valid),.sector_end(sub_end),.sector_fault(card_fault),
        .wr_en(sub_wr_en),.wr_text(sub_wr_text),.wr_addr(sub_wr_addr),.wr_data(sub_wr_data),
        .wr_bank(sub_wr_bank),
        .pub_req(sub_pub_req),.pub_valid(sub_pub_valid),.pub_bank(sub_pub_bank),
        .pub_len(sub_pub_len),.pub_ack(sub_pub_ack),.loads(),.invalids());
    pcm_bank_buffer u_pcm_buffer(.wr_clk(sd_card_clk),.rd_clk(video_clk),.rst(rst),
        .wr_en(pcm_wr),.flush(pcm_flush),.wr_data(pcm_write_data),.free_frames(pcm_free),
        .pop(pcm_pop),.empty(pcm_empty),.data(pcm_read_data),.overflow(pcm_overflow));

    sd_card_top sd_card_top_m0(
        .clk(sd_card_clk), .rst(rst_stack | card_reset),
        .SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso),
        .sd_init_done(sd_init_done),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .sd_sec_write(1'b0), .sd_sec_write_addr(32'd0), .sd_sec_write_data(8'd0),
        .sd_sec_write_data_req(), .sd_sec_write_end());

    wire        scan_start, scan_busy, hit_valid, scan_done;
    wire [31:0] hit_sector, load_sector;
    wire        load_start, load_done, load_error, bmp_topdown;
    wire        sd_card_write_req, sd_card_write_en;
    wire [23:0] pix_rgb;
    wire scaled_write_ready;
    wire [3:0] loader_state,image_error,missing_images,bad_images;
    wire [1:0] image_error_role;

    wire metadata_start;
    wire [10:0] metadata_width,metadata_height;
    fat32_scaled_loader #(.IMG_WIDTH(640), .IMG_HEIGHT(480)) u_loader(
        .clk(sd_card_clk), .rst(rst_sd), .sd_init_done(sd_init_done),
        .scan_start(scan_start), .scan_busy(scan_busy),
        .hit_valid(hit_valid), .hit_sector(hit_sector), .scan_done(scan_done),
        .load_start(load_start), .load_sector(load_sector), .load_abort(load_abort | card_fault), .smooth(smooth_sd),
        .load_busy(load_busy), .load_done(load_done), .load_error(load_error),
        .bmp_topdown(bmp_topdown),
        .write_req(sd_card_write_req), .write_req_ack(write_ack_sync[2]),
        .write_ready(scaled_write_ready), .pix_wr_en(sd_card_write_en), .pix_data(pix_rgb),
        .sd_sec_read(image_req), .sd_sec_read_addr(image_lba),
        .sd_sec_read_data(client_data), .sd_sec_read_data_valid(image_valid),
        .sd_sec_read_end(image_end),
        .state_code(loader_state),.error_code(image_error),.error_role(image_error_role),
        .missing_mask(missing_images),.bad_mask(bad_images),
        .source_start(metadata_start),.source_width(metadata_width),.source_height(metadata_height));

    wire [31:0] sd_card_write_data = {pix_rgb, 8'b0};

    wire write_finish_mem;
    reg  write_finish_toggle;
    always @(posedge ext_mem_clk or posedge rst_mem) begin
        if (rst_mem) write_finish_toggle <= 1'b0;
        else if (write_finish_mem) write_finish_toggle <= ~write_finish_toggle;
    end

    wire [1:0] write_buf_idx, disp_buf_idx, img_idx;
    wire       err_no_image, err_load;
    wire [2:0] img_count;
    wire [3:0] ctrl_state;
    wire swap_req, swap_grant_v, transition_busy_v;
    reg [2:0] grant_sync, transition_busy_sync;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin
            grant_sync <= 3'b000;
            transition_busy_sync <= 3'b000;
        end else begin
            grant_sync <= {grant_sync[1:0], swap_grant_v};
            transition_busy_sync <= {transition_busy_sync[1:0], transition_busy_v};
        end
    end

    // No card: report NO IMAGE after 15 s instead of 60 s (KEY2/KEY3 retry).
    media_ctrl #(.CLK_FREQ_HZ(SD_CLK_HZ), .LOAD_TIMEOUT_MS(8000),
                 .SCAN_TIMEOUT_MS(15000), .EMERGENCY_TAKEOVER(0),
                 .EXTERNAL_ADVANCE(1), .AUTO_DEFAULT(1),
                 .BUF0_ADDR(BUF0_ADDR), .BUF1_ADDR(BUF1_ADDR)) u_ctrl(
        .clk(sd_card_clk), .rst(rst_sd), .sd_init_done(sd_init_done),
        .ev_next(ev_next_ctrl), .ev_prev(ev_prev_ctrl), .ev_auto_toggle(ev_auto_ctrl),
        .ev_reload(ev_smooth_reload),.ev_advance(ev_advance),.ev_auto_on(ev_auto_on),.ev_retry(ev_retry),.load_kind(load_kind),
        .emergency(quiet), .transition_busy(transition_busy_sync[2]),
        .swap_req(swap_req), .swap_grant(grant_sync[2]),
        .scan_start(scan_start), .scan_busy(scan_busy),
        .hit_valid(hit_valid), .hit_sector(hit_sector), .scan_done(scan_done),
        .load_start(load_start), .load_sector(load_sector), .load_abort(load_abort),
        .load_busy(load_busy), .load_done(load_done), .load_error(load_error),
        .write_finish_toggle(write_finish_toggle),
        .pipeline_idle(pipeline_idle),.abort_done(abort_done),.card_fault(card_fault),
        .write_buf_idx(write_buf_idx), .disp_buf_idx(disp_buf_idx), .display_valid(display_valid),
        .img_count(img_count), .img_idx(img_idx), .img_role(img_role), .auto_en(auto_en),
        .ctrl_state(ctrl_state), .err_no_image(err_no_image), .err_load(err_load));

    // ------------------------------------------------------------------
    // status: LEDs + 7-seg
    // ------------------------------------------------------------------
    wire video_locked;
    reg [24:0] blink_cnt;
    always @(posedge clk) blink_cnt <= blink_cnt + 1'b1;
    // Status LEDs follow the staff display: dark when it is idle, except an
    // error (LED4) or the alarm blink, which always show.
    wire led_err = err_no_image | err_load | music_fault | (image_error!=0);
    assign led = standby ? {3'b000, blink_cnt[24] & blink_cnt[23]} :
                 {emergency_b ? blink_cnt[23] : led_err,
                  seg_on & video_locked, seg_on & auto_en, seg_on & display_valid};

    // SD auto-recovery (pixel domain); its inputs are registered SD levels.
    reg init_done_sd, noimg_sd, reader_fail_sd;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin init_done_sd<=0; noimg_sd<=0; reader_fail_sd<=0; end
        else begin
            init_done_sd <= sd_init_done;
            noimg_sd <= ctrl_state==4'd4;                 // ST_NOIMG
            reader_fail_sd <= music_error!=0;
        end
    end
    wire [7:0] recoveries;
    sd_recovery #(.CLK_HZ(PIX_CLK_HZ)) u_recovery(.clk(video_clk),.rst(rst_video),
        .init_done_async(init_done_sd),.noimg_async(noimg_sd),.reader_fail_async(reader_fail_sd),
        .standby(standby),.stack_rst(stack_rst_v),.reader_rst(reader_rst_v),.retry_t(retry_t),
        .recoveries(recoveries));

    // Staff display: SD-domain state registered here, pixel-domain snapshot
    // below, pages and scanning in the pixel-clock domain.
    reg [3:0] seg_state_sd;
    reg seg_error_sd;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin seg_state_sd<=0; seg_error_sd<=0; end
        else begin
            seg_state_sd <= image_error!=0 ? image_error : loader_state;
            seg_error_sd <= err_no_image | err_load | (image_error!=0) | music_fault;
        end
    end
    wire [63:0] seg_digits;
    wire [76:0] seg_snap;
    wire seg_snap_t;
    // Run-time clock (ms stamps for the UART log, BCD for header/display).
    wire [31:0] up_ms;
    wire [23:0] up_hms;
    uptime_clock #(.CLK_HZ(PIX_CLK_HZ)) u_uptime(.clk(video_clk),.rst(rst_video),
        .ms(up_ms),.ms_tick(),.hms(up_hms));
    seg_pages #(.CLK_HZ(PIX_CLK_HZ)) u_seg_pages(.clk(video_clk),.rst(rst_video),.emergency(emergency_b),.touring(tour_on),
        .volume(volume),.uptime(up_hms),
        .snap_async(seg_snap),.snap_toggle_async(seg_snap_t),
        .state_async(seg_state_sd),.error_async(seg_error_sd),.digits(seg_digits));
    seg_scan8 #(.CLK_HZ(PIX_CLK_HZ)) u_seg_scan(.clk(video_clk),.rst(rst_video),.enable(seg_on),.digits(seg_digits),.seg_sel(seg_sel),.seg_data(seg_data));

    // ------------------------------------------------------------------
    // video timing + frame buffer read (official)
    // ------------------------------------------------------------------
    wire hs_0, vs_0, de_0, hs, vs, de;
    wire        video_read_req, video_read_req_ack, video_read_en;
    (* async_reg = "true" *) reg [2:0] read_ack_sync;
    always @(posedge video_clk or posedge rst_video)
        if(rst_video)read_ack_sync<=0;
        else read_ack_sync<={read_ack_sync[1:0],video_read_req_ack};
    wire [31:0] video_read_data;
    wire [23:0] vout_data;

    video_timing_data video_timing_data_m0(.video_clk(video_clk), .rst(rst_video),
        .read_req(video_read_req), .read_req_ack(read_ack_sync[2]),
        .hs(hs_0), .vs(vs_0), .de(de_0));
    video_delay video_delay_m0(.video_clk(video_clk), .rst(rst_video),
        .read_en(video_read_en), .read_data(video_read_data[31:8]),
        .hs(hs_0), .vs(vs_0), .de(de_0), .hs_r(hs), .vs_r(vs), .de_r(de),
        .vout_data(vout_data));

    // Emergency remains prompt and independent of the ordinary UI mailbox.
    reg [1:0] dv_sync,em_sync;
    reg [1:0] contrast_sd,maint_sd;
    always @(posedge sd_card_clk or posedge rst_sd)begin
        if(rst_sd)begin contrast_sd<=0;maint_sd<=0;end
        else begin contrast_sd<={contrast_sd[0],sw_contrast};maint_sd<={maint_sd[0],maint_p};end
    end
    always @(posedge video_clk or posedge rst_video)begin
        if(rst_video)begin dv_sync<=0;em_sync<=0;end
        else begin dv_sync<={dv_sync[0],display_valid};em_sync<={em_sync[0],emergency};end
    end
    // Header status word, highest priority first; all inputs are real state.
    reg [1:0] mute_sd;
    reg [2:0] field_sd;
    always @(posedge sd_card_clk or posedge rst_sd)begin
        if(rst_sd)begin mute_sd<=0;field_sd<=0;end
        else begin
            mute_sd<={mute_sd[0],sw_mute};
            if(ctrl_state==4'd0 || ctrl_state==4'd1 || load_busy) field_sd<=3'd1;   // LOADING
            else if(err_no_image)            field_sd<=3'd2;                       // NO IMAGE
            else if(mute_sd[1])              field_sd<=3'd6;                       // MUTED
            else if(track_kind==2'd2)        field_sd<=3'd3;                       // GUIDE
            else if(track_kind==2'd1)        field_sd<=3'd4;                       // MUSIC
            else if(tone_fallback && maint_sd[1]) field_sd<=3'd5;                  // TEST TONE
            else                             field_sd<=3'd0;
        end
    end
    // M13: the mailbox carries the displayed bank, not the 52-bit BCD
    // geometry; the geometry is computed per bank in the pixel domain.
    wire [16:0] status_source={smooth_sd,field_sd,1'b0,disp_buf_idx[0],display_valid,
        bright_lvl,contrast_sd[1],maint_sd[1],img_role,img_count,auto_en};
    wire [16:0] status_active;
    frame_status #(.WIDTH(17),.RESET_VALUE(17'h200)) u_status(.src_clk(sd_card_clk),.src_rst(rst_sd),
        .source(status_source),.video_clk(video_clk),.video_rst(rst_video),.vs(vs),.active(status_active));
    wire [2:0] frame_field=status_active[15:13];
    wire frame_smooth=status_active[16];
    wire frame_bank=status_active[11];
    wire [51:0] info_metadata;
    wire bank_meta_valid;
    metadata_cdc u_metadata(.sd_clk(sd_card_clk),.sd_rst(rst_sd),
        .start(metadata_start),.abort_load(load_abort | card_fault),.load_error(load_error),.load_done(load_done),
        .source_width(metadata_width),.source_height(metadata_height),.write_bank(write_buf_idx[0]),
        .video_clk(video_clk),.video_rst(rst_video),.display_bank(frame_bank),
        .display_meta(info_metadata),.display_meta_valid(bank_meta_valid));
    wire info_valid=status_active[10] && bank_meta_valid;
    wire [1:0] frame_bright=status_active[9:8];
    wire frame_contrast=status_active[7],frame_maint=status_active[6];
    wire [1:0] frame_role=status_active[5:4];
    wire [2:0] frame_count=status_active[3:1];
    wire frame_auto=status_active[0];
    wire [23:0] rgb_raw = dv_sync[1] ? vout_data : 24'd0;

    // ------------------------------------------------------------------
    // pixel pipeline: brightness/contrast (2 clk) -> fade (1 clk) -> OSD (3 clk)
    // ------------------------------------------------------------------
    wire [23:0] rgb_adj; wire hs_a, vs_a, de_a;
    pixel_adjust u_adj(.clk(video_clk), .rst(rst_video),
        .contrast_hi(frame_contrast), .bright_lvl(frame_bright),
        .rgb_i(rgb_raw), .hs_i(hs), .vs_i(vs), .de_i(de),
        .rgb_o(rgb_adj), .hs_o(hs_a), .vs_o(vs_a), .de_o(de_a));

    // Fade only the photograph; OSD and alarms are independent of the fade.
    // Alpha changes at falling VS (vertical blank), never in an active line.
    wire [23:0] rgb_fade; wire hs_f, vs_f, de_f;
    frame_transition #(.EXIT_FADE(1)) u_transition(.clk(video_clk), .rst(rst_video),
        .swap_req_async(swap_req), .style_async(load_kind), .display_valid(dv_sync[1]),
        .emergency(em_sync[1] | dark), .swap_grant(swap_grant_v),
        .busy(transition_busy_v), .alpha_debug(),
        .rgb_i(rgb_adj), .hs_i(hs_a), .vs_i(vs_a), .de_i(de_a),
        .rgb_o(rgb_fade), .hs_o(hs_f), .vs_o(vs_f), .de_o(de_f));

    wire [23:0] rgb_info; wire hs_info,vs_info,de_info;
    runtime_info_overlay #(.ROW3(1)) u_info(.clk(video_clk),.rst(rst_video),.maint(frame_maint),
        .uptime(up_hms),.volume(volume),.recoveries(recoveries),
        .emergency(em_sync[1]),.valid(info_valid),.metadata(info_metadata),
        .image_role(frame_role),.image_count(frame_count),.brightness(frame_bright),
        .contrast(frame_contrast),.auto_play(frame_auto),.field(frame_field),.smooth(frame_smooth),.dwell(dwell_sel),
        .rgb_i(rgb_fade),.hs_i(hs_f),.vs_i(vs_f),.de_i(de_f),
        .rgb_o(rgb_info),.hs_o(hs_info),.vs_o(vs_info),.de_o(de_info));
    wire card_valid, card_bank;
    wire [5:0] card_len, card_text;
    wire [6:0] card_text_addr;
    wire [10:0] card_glyph_addr;
    wire [15:0] card_bits;
    subtitle_commit u_sub_commit(.clk(video_clk),.rst(rst_video),.vs(vs),
        .pub_req(sub_pub_req),.pub_valid(sub_pub_valid),.pub_bank(sub_pub_bank),
        .pub_len(sub_pub_len),.pub_ack(sub_pub_ack),
        .card_valid(card_valid),.card_bank(card_bank),.card_len(card_len));
    subtitle_store u_sub_store(.wr_clk(sd_card_clk),.wr_en(sub_wr_en),.wr_text(sub_wr_text),
        .wr_bank(sub_wr_bank),.wr_addr(sub_wr_addr),.wr_data(sub_wr_data),
        .rd_clk(video_clk),.text_addr(card_text_addr),.text_q(card_text),
        .glyph_addr(card_glyph_addr),.glyph_q(card_bits));
    wire [23:0] rgb_alert; wire hs_alert,vs_alert,de_alert;
    wire caption_pass_t;
    osd_overlay #(.H_ACTIVE(640), .V_ACTIVE(480)) u_osd(.clk(video_clk), .rst(rst_video),
        .rgb_i(rgb_info), .hs_i(hs_info), .vs_i(vs_info), .de_i(de_info),
        .osd_en(1'b1), .emergency(em_sync[1]), .pass_toggle(caption_pass_t),
        .safety_valid(info_valid && frame_role==3),
        .img_idx(frame_role), .img_count(frame_count), .bright_lvl(frame_bright), .contrast_hi(frame_contrast),
        .card_valid(card_valid), .card_bank(card_bank), .card_len(card_len),
        .card_text_addr(card_text_addr), .card_text(card_text),
        .card_glyph_addr(card_glyph_addr), .card_bits(card_bits),
        .rgb_o(rgb_alert), .hs_o(hs_alert), .vs_o(vs_alert), .de_o(de_alert));

    // Spectrum last: in an alarm it draws the siren over the alarm page.
    // The analyzer keeps running in emergency (it sees the alarm samples).
    wire [95:0] spectrum_levels;
    wire [23:0] raw_left, raw_right;      // pre-volume samples (u_audio)
    wire spectrum_valid,spectrum_overrun;
    // FFT idles while nothing is audible, once the bars have reached zero.
    wire tone_on;
    wire spec_feed = music_active | tone_on | em_sync[1] | (spectrum_levels != 96'd0);
    spectrum_analyzer u_spectrum(.clk(video_clk),.rst(rst_video),.emergency(1'b0),
        .sample_valid(audio_valid & spec_feed),.sample_left(raw_left),.sample_right(raw_right),
        .levels(spectrum_levels),.levels_valid(spectrum_valid),.overrun(spectrum_overrun));
    reg [1:0] mute_sync;
    always @(posedge video_clk or posedge rst_video) begin
        if(rst_video) mute_sync<=0;
        else mute_sync<={mute_sync[0],sw_mute};
    end
    // Setting changes (SW1/SW2/SW3/SW4 effective values) count as staff
    // activity for the seven-segment idle timer.
    reg [3:0] setting_prev;
    reg setting_seen;
    wire [3:0] setting_now = {mute_sync[1], maint_p, frame_contrast, reload_t};
    assign setting_change = setting_seen && setting_now != setting_prev;
    always @(posedge video_clk or posedge rst_video) begin
        if (rst_video) begin setting_prev <= 0; setting_seen <= 0; end
        else begin setting_prev <= setting_now; setting_seen <= 1; end
    end
    reg [1:0] tone_sync, ready_sync;
    always @(posedge video_clk or posedge rst_video) begin
        if(rst_video) begin tone_sync<=0; ready_sync<=0; end
        else begin tone_sync<={tone_sync[0],tone_fallback}; ready_sync<={ready_sync[0],music_ready}; end
    end
    wire [3:0] peak_band; wire [5:0] peak_level;
    spectrum_peak u_peak(.clk(video_clk),.rst(rst_video),.levels(spectrum_levels),
        .levels_valid(spectrum_valid),.band(peak_band),.level(peak_level));
    // One snapshot per frame for the staff display (layout: seg_pages.v).
    reg [76:0] seg_snap_r;
    reg seg_snap_tr, snap_vs_d;
    always @(posedge video_clk or posedge rst_video) begin
        if (rst_video) begin seg_snap_r<=0; seg_snap_tr<=0; snap_vs_d<=0; end
        else begin
            snap_vs_d <= vs;
            if (snap_vs_d && !vs) begin
                seg_snap_r <= {info_metadata, info_valid, frame_bright, frame_contrast, frame_smooth,
                               frame_role, frame_count, mute_sync[1], em_sync[1], dv_sync[1],
                               peak_band, peak_level, ready_sync[1] | (tone_sync[1] & maint_p), 1'b0};
                seg_snap_tr <= ~seg_snap_tr;
            end
        end
    end
    assign seg_snap = seg_snap_r;
    assign seg_snap_t = seg_snap_tr;
    // Tour pacing (pixel clock). Levels come from registers only.
    tour_ctrl #(.CLK_HZ(PIX_CLK_HZ)) u_tour(.clk(video_clk),.rst(rst_video),.emergency(emergency_b | standby),.user_activity(user_activity),.dwell_sel(dwell_sel),
        .auto_en_async(frame_auto),.display_valid_async(dv_sync[1]),
        .guide_done_async(guide_done),.narr_async(track_kind[1]),
        .role_async(frame_role),.pass_toggle_async(caption_pass_t),
        .advance_toggle(advance_t),.auto_on_toggle(auto_on_t),.touring(tour_on));
    wire [23:0] rgb_osd; wire hs_o, vs_o, de_o;
    spectrum_overlay u_visualizer(.clk(video_clk),.rst(rst_video),.levels(spectrum_levels),
        .emergency(em_sync[1]),.overlay_enable(dv_sync[1]),
        .rgb_i(rgb_alert),.hs_i(hs_alert),.vs_i(vs_alert),.de_i(de_alert),
        .rgb_o(rgb_osd),.hs_o(hs_o),.vs_o(vs_o),.de_o(de_o));

    // ------------------------------------------------------------------
    // M15: alarm buzzer and UART event log (pixel-clock domain)
    // ------------------------------------------------------------------
    alarm_buzzer #(.CLK_HZ(PIX_CLK_HZ)) u_buzzer(.clk(video_clk),.rst(rst_video),
        .emergency(em_sync[1]),.buzzer_n(buzzer_n));
    // SD-domain sources: a load-start toggle with its exhibit token; the
    // other levels are existing SD registers, synchronised here (2 FF).
    reg load_t_sd;
    reg [1:0] load_role_sd;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin load_t_sd <= 0; load_role_sd <= 0; end
        else if (load_start) begin load_t_sd <= ~load_t_sd; load_role_sd <= load_sector[1:0]; end
    end
    (* async_reg = "true" *) reg [22:0] log_meta;
    reg [22:0] log_lvl, log_prev;
    reg log_primed;
    always @(posedge video_clk or posedge rst_video) begin
        if (rst_video) begin log_meta <= 0; log_lvl <= 0; log_prev <= 0; log_primed <= 0; end
        else begin
            log_meta <= {init_done_sd, ctrl_state, load_t_sd, load_role_sd,
                         music_ready, track_kind, image_error, music_error};
            log_lvl <= log_meta;
            log_prev <= log_lvl;
            log_primed <= 1;
        end
    end
    wire lg_card = log_lvl[22], lg_card_p = log_prev[22];
    wire [3:0] lg_state = log_lvl[21:18], lg_state_p = log_prev[21:18];
    wire lg_load_t = log_lvl[17], lg_load_tp = log_prev[17];
    wire [1:0] lg_role = log_lvl[16:15];
    wire [1:0] lg_audio = log_lvl[14] ? log_lvl[13:12] : 2'd0;
    wire [1:0] lg_audio_p = log_prev[14] ? log_prev[13:12] : 2'd0;
    wire [3:0] lg_imerr = log_lvl[11:8], lg_imerr_p = log_prev[11:8];
    wire [7:0] lg_auerr = log_lvl[7:0], lg_auerr_p = log_prev[7:0];
    // Pixel-domain sources.
    reg boot_done;
    reg [2:0] show_prev;
    reg alarm_prev, stby_prev, recov_prev;
    wire [2:0] show_now = {info_valid, frame_role};
    always @(posedge video_clk or posedge rst_video) begin
        if (rst_video) begin boot_done <= 0; show_prev <= 0; alarm_prev <= 0; stby_prev <= 0; recov_prev <= 0; end
        else begin
            boot_done <= 1; show_prev <= show_now; alarm_prev <= emergency_b;
            stby_prev <= standby; recov_prev <= stack_rst_v | reader_rst_v;
        end
    end
    wire recov_now = stack_rst_v | reader_rst_v;
    wire [11:0] log_valid = {
        log_primed && lg_auerr != lg_auerr_p && lg_auerr != 0,      // 11 AUERR
        log_primed && lg_imerr != lg_imerr_p && lg_imerr != 0,      // 10 IMERR
        recov_now && !recov_prev,                                    //  9 RECOV
        standby != stby_prev,                                        //  8 STBY
        emergency_b != alarm_prev,                                   //  7 ALARM
        key_log_valid,                                               //  6 KEY
        log_primed && lg_audio != lg_audio_p,                        //  5 AUDIO
        show_now != show_prev && info_valid,                         //  4 SHOW
        log_primed && lg_load_t != lg_load_tp,                       //  3 LOAD
        log_primed && lg_state != lg_state_p,                        //  2 STATE
        log_primed && lg_card != lg_card_p,                          //  1 CARD
        !boot_done};                                                 //  0 BOOT
    wire [95:0] log_args = {
        lg_auerr, {4'd0, lg_imerr}, {6'd0, reader_rst_v, stack_rst_v}, {7'd0, standby},
        {7'd0, emergency_b}, {4'd0, key_log_arg}, {6'd0, lg_audio}, {6'd0, frame_role},
        {6'd0, lg_role}, {4'd0, lg_state}, {7'd0, lg_card}, 8'd0};
    uart_logger #(.CLK_HZ(PIX_CLK_HZ)) u_log(.clk(video_clk),.rst(rst_video),.ms(up_ms),
        .ev_valid(log_valid),.ev_arg(log_args),.tx(uart_tx));

    // ------------------------------------------------------------------
    // HDMI 1.4b transmitter (official APUG092 core): video AXIS + audio PCM
    // ------------------------------------------------------------------
    wire        axis_user, axis_valid, axis_last, axis_ready;
    wire [23:0] axis_data;
    video_rgb_to_axis_640x480 u_rgb2axis(.I_clk(video_clk), .I_rst(rst_video),
        .I_vs(vs_o), .I_de(de_o), .I_rgb(rgb_osd),
        .O_video_user(axis_user), .O_video_valid(axis_valid),
        .O_video_last(axis_last), .O_video_data(axis_data));

    wire edid_trig;
    startup_pulse #(.CNT_MAX(20'd100000)) u_startup_pulse(.I_clk(video_clk), .I_rst(rst_video), .O_pulse(edid_trig));

    // WAV music pauses at emergency entry; the alarm is independent of SD.
    guide_audio_output #(.CLK_HZ(PIX_CLK_HZ)) u_audio(
        .clk(video_clk),.rst(rst_video),.emergency(em_sync[1]),.mute(mute_sync[1] | standby),
        .tone_fallback(tone_sync[1] & maint_p),.track_ready(ready_sync[1]),
        .pcm_empty(pcm_empty),.pcm_data(pcm_read_data),.pcm_pop(pcm_pop),
        .sample_valid(audio_valid),.sample_left(audio_left),.sample_right(audio_right),
        .raw_left(raw_left),.raw_right(raw_right),.volume(volume),
        .underrun(audio_underrun),.music_active(music_active),.tone_active(tone_on));
    reg [2:0] underrun_sync;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if(rst_sd) underrun_sync<=0;
        else underrun_sync<={underrun_sync[1:0],audio_underrun};
    end
    assign music_fault = (music_error!=0) | pcm_overflow | underrun_sync[2];

    wire        acr_valid;
    wire [19:0] acr_cts, acr_n;
    audio_arc_calculate #(.ACR_N(6144)) u_acr(.I_clk(video_clk), .I_rst(rst_video),
        .I_audio_valid(audio_valid), .O_acr_valid(acr_valid), .O_acr_cts(acr_cts), .O_acr_n(acr_n));

    wire [9:0] tmds_ch0, tmds_ch1, tmds_ch2, tmds_clk;
    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE("EG"), .HTOTAL(800), .HSA(96), .HFP(16), .HBP(48), .HACTIVE(640),
        .VTOTAL(525), .VSA(2), .VFP(10), .VBP(33), .VACTIVE(480),
        .VIDEO_VIC(1), .VIDEO_TPG("Disable"), .VIDEO_FORMAT("RGB"),
        .AUDIO_SAMPLE_RATE("48K"), .IIC_SCL_DIV(250)
    ) u_hdmi_core(
        .I_pixel_clk(video_clk), .I_rst(rst_video),
        .I_edid_read_trig(edid_trig), .O_edid_read_valid(), .O_edid_read_data(),
        .I_axis_s_user(axis_user), .I_axis_s_valid(axis_valid), .I_axis_s_last(axis_last),
        .I_axis_s_data(axis_data), .O_axis_s_ready(axis_ready),
        .I_audio_valid(audio_valid), .I_audio_left_data(audio_left), .I_audio_right_data(audio_right),
        .I_acr_valid(acr_valid), .I_acr_cts(acr_cts), .I_acr_n(acr_n),
        .O_video_locked(video_locked),
        .O_ddc_scl(HDMI_DDC_SCL), .IO_ddc_sda(HDMI_DDC_SDA),
        .O_ch0_tmds_data(tmds_ch0), .O_ch1_tmds_data(tmds_ch1),
        .O_ch2_tmds_data(tmds_ch2), .O_clk_tmds_data(tmds_clk));

    hdmi_phy_wrapper #(.DEVICE("EG")) u_hdmi_phy(
        .I_pixel_clk(video_clk), .I_serial_clk(hdmi_5x_clk), .I_rst(rst_video | standby),
        .I_tmds_channel_0(tmds_ch0), .I_tmds_channel_1(tmds_ch1),
        .I_tmds_channel_2(tmds_ch2), .I_tmds_channel_clk(tmds_clk),
        .O_tmds_ch0_p(HDMI_D0_P), .O_tmds_ch1_p(HDMI_D1_P),
        .O_tmds_ch2_p(HDMI_D2_P), .O_tmds_clk_p(HDMI_CLK_P));

    // ------------------------------------------------------------------
    // SDRAM frame buffer (official) with runtime vertical flip
    // ------------------------------------------------------------------
    wire Sdr_init_done, Sdr_init_ref_vld, Sdr_busy;
    wire App_rd_en, Sdr_rd_en, App_wr_en;
    wire [ADDR_BITS-1:0] App_rd_addr, App_wr_addr;
    wire [MEM_DATA_BITS-1:0] Sdr_rd_dout, App_wr_din;
    wire [3:0] App_wr_dm;

    frame_read_write #(.FRAME_WIDTH(640), .FRAME_HEIGHT(480)) frame_read_write_m0(
        .mem_clk(ext_mem_clk), .rst(rst_mem),
        .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr), .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
        .read_clk(video_clk), .read_req(video_read_req), .read_req_ack(video_read_req_ack), .read_finish(),
        .read_addr_0(BUF0_ADDR[ADDR_BITS-1:0]), .read_addr_1(BUF1_ADDR[ADDR_BITS-1:0]),
        .read_addr_2(BUF0_ADDR[ADDR_BITS-1:0]), .read_addr_3(BUF1_ADDR[ADDR_BITS-1:0]),
        .read_addr_index(disp_buf_idx), .read_len(FRAME_PIXELS[ADDR_BITS-1:0]),
        .read_en(video_read_en), .read_data(video_read_data),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
        .write_rst(rst_sd), .write_ready(scaled_write_ready), .write_clk(sd_card_clk), .write_req(sd_card_write_req), .write_req_ack(sd_card_write_req_ack),
        .write_finish(write_finish_mem),.write_abort(load_abort),.write_abort_ack(writer_abort_ack),
        .write_addr_0(BUF0_ADDR[ADDR_BITS-1:0]), .write_addr_1(BUF1_ADDR[ADDR_BITS-1:0]),
        .write_addr_2(BUF0_ADDR[ADDR_BITS-1:0]), .write_addr_3(BUF1_ADDR[ADDR_BITS-1:0]),
        .write_addr_index(write_buf_idx), .write_len(FRAME_PIXELS[ADDR_BITS-1:0]),
        .write_en(sd_card_write_en), .write_data(sd_card_write_data),
        .write_vflip(~bmp_topdown));

    sdram U3(.Clk(ext_mem_clk), .Clk_sft(ext_mem_clk_sft), .Rst(rst_mem),
        .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_dm(App_wr_dm), .App_wr_din(App_wr_din),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr), .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout));
endmodule
