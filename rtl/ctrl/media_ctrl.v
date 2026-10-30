// M13: EXTERNAL_ADVANCE=1 removes the fixed auto-play counter; the tour
// controller (50 MHz domain) decides the dwell and sends ev_advance; a manual
// page turn pauses the tour and ev_auto_on resumes it after an idle period. AUTO_DEFAULT is the tour
// state after reset (1 = an unattended terminal starts touring). load_kind
// tells the transition which style to use for the load being committed:
// 0 fade (tour / reload), 1 scroll (visitor navigation), 2 iris (first image).
// M14b: ev_retry (from sd_recovery) leaves NO IMAGE like KEY2/KEY3, without
// touching the tour or the page; the card side has just been reset.
// One image transaction at a time, with level-driven emergency target and
// acknowledged abort/drain before buffer reuse. All ports are in clk domain.
// M12: ev_reload re-loads the displayed exhibit (used when SW4 changes the
// scaling mode), through the normal fade/commit path; newer navigation wins.
// M10: EMERGENCY_TAKEOVER=0 keeps the displayed exhibit during an alarm. The
// on-chip alarm page covers the whole raster, so loading IMG04 behind it was
// invisible SD traffic; the exhibit simply fades back in after release.
module media_ctrl #(
    parameter integer CLK_FREQ_HZ=100_000_000,
    parameter integer AUTO_PLAY_MS=3000,
    parameter integer LOAD_TIMEOUT_MS=4000,
    parameter integer SCAN_TIMEOUT_MS=60000,
    parameter integer RETRY_MS=500,
    parameter EMERGENCY_TAKEOVER=1,
    parameter EXTERNAL_ADVANCE=0,
    parameter AUTO_DEFAULT=0,
    parameter [23:0] BUF0_ADDR=0, BUF1_ADDR=307200
)(
    input wire clk,rst,sd_init_done,
    input wire ev_next,ev_prev,ev_auto_toggle,emergency,transition_busy,swap_grant,
    input wire ev_reload,
    input wire ev_advance,ev_auto_on,ev_retry,
    output reg [1:0] load_kind,
    output reg swap_req,
    output reg scan_start,
    input wire scan_busy,hit_valid,
    input wire [31:0] hit_sector,
    input wire scan_done,
    output reg load_start,
    output reg [31:0] load_sector,
    output reg load_abort,
    input wire load_busy,load_done,load_error,write_finish_toggle,
    input wire pipeline_idle,abort_done,card_fault,
    output reg [1:0] write_buf_idx,disp_buf_idx,
    output reg display_valid,
    output reg [2:0] img_count,
    output reg [1:0] img_idx,
    output wire [1:0] img_role,
    output reg auto_en,
    output reg [3:0] ctrl_state,
    output reg err_no_image,err_load
);
    localparam ST_WAIT_SD=0,ST_SCAN=1,ST_FIRST=2,ST_RUN=3,ST_NOIMG=4;
    localparam integer AUTO_CYC=(CLK_FREQ_HZ/1000)*AUTO_PLAY_MS;
    localparam integer TMO_CYC=(CLK_FREQ_HZ/1000)*LOAD_TIMEOUT_MS;
    // 64-bit elaboration prevents the 60s x100MHz constant overflowing int.
    localparam [63:0] SCAN_CYC=(64'd1*CLK_FREQ_HZ/1000)*SCAN_TIMEOUT_MS;
    // 15 s at 100 MHz fits 31 bits; the counter need not be 64 bits wide.
    localparam integer SCAN_W=$clog2(SCAN_CYC+1);
    localparam integer RETRY_CYC=(CLK_FREQ_HZ/1000)*RETRY_MS;
    reg [31:0] img_sector[0:3];
    reg safety_found;
    reg [1:0] safety_idx;
    assign img_role=img_sector[img_idx][1:0];
    reg [1:0] load_idx,pending_buf,resume_idx;
    reg loading,load_started,source_done,wrfin_seen;
    reg launch_pending;
    reg emergency_d,resume_valid,rearm;
    reg manual_pending,manual_prev,reload_pending,advance_pending;
    // M14: counters sized to their limits (were 32 bits).
    localparam integer TMO_W=$clog2(TMO_CYC+1), RETRY_W=$clog2(RETRY_CYC+1);
    localparam integer AUTO_W=EXTERNAL_ADVANCE ? 1 : $clog2(AUTO_CYC+1);
    reg [TMO_W-1:0] timer;
    reg [AUTO_W-1:0] auto_cnt;
    reg [RETRY_W-1:0] retry_cnt;
    reg [SCAN_W-1:0] scan_timer;
    reg [2:0] wrfin_sync;
    wire wrfin_pulse=wrfin_sync[2]^wrfin_sync[1];
    wire restore_pending=resume_valid && !emergency;
    wire takeover_pending=EMERGENCY_TAKEOVER && emergency && safety_found && (!display_valid || img_idx!=safety_idx);
    wire ctrl_idle=ctrl_state==ST_RUN && !loading && !load_abort && !rearm &&
                   !transition_busy && pipeline_idle && sd_init_done && !card_fault && retry_cnt==0;
    wire frame_ready=loading && (source_done||load_done) && (wrfin_seen||wrfin_pulse);
    // Error is sticky in loader; ignore the previous transaction's error
    // until the new loader has acknowledged start by asserting busy.
    wire failed=loading && (card_fault || (load_started && load_error) || timer>=TMO_CYC-1);
    wire commit_now=loading && swap_req && swap_grant && !failed;
    wire auto_due=EXTERNAL_ADVANCE ? advance_pending : (auto_cnt>=AUTO_CYC-1);
    reg request_valid;
    reg [1:0] requested_idx;
    always @(*) begin
        request_valid=0;requested_idx=0;
        if(emergency)begin
            if(EMERGENCY_TAKEOVER)begin request_valid=takeover_pending;requested_idx=safety_idx;end
            // No takeover: still bring up the first exhibit behind the alarm
            // page, so the release is immediate.
            else begin request_valid=!display_valid && img_count!=0;requested_idx=0;end
        end
        else if(restore_pending)begin
            requested_idx=resume_idx;
            request_valid={1'b0,resume_idx}<img_count && (!display_valid || img_idx!=resume_idx);
        end else if(!display_valid && img_count!=0)request_valid=1;
        else if(reload_pending && !manual_pending && !ev_next && !ev_prev)begin
            request_valid=1;requested_idx=img_idx;
        end else if(img_count>1 && (manual_pending||ev_next||ev_prev||(auto_en && auto_due)))begin
            request_valid=1;
            if(ev_prev || (!ev_next && manual_pending && manual_prev))
                requested_idx=img_idx==0 ? img_count[1:0]-1'b1 : img_idx-1'b1;
            else requested_idx=({1'b0,img_idx}+1>=img_count) ? 2'd0 : img_idx+1'b1;
        end
    end
    integer i;
    always @(posedge clk or posedge rst)begin
        if(rst)wrfin_sync<=0;else wrfin_sync<={wrfin_sync[1:0],write_finish_toggle};
    end
    always @(posedge clk or posedge rst)begin
        if(rst)begin
            swap_req<=0;scan_start<=0;load_start<=0;load_sector<=0;load_abort<=0;
            write_buf_idx<=0;disp_buf_idx<=0;display_valid<=0;img_count<=0;img_idx<=0;
            safety_found<=0;safety_idx<=0;auto_en<=AUTO_DEFAULT;load_kind<=2;ctrl_state<=ST_WAIT_SD;err_no_image<=0;err_load<=0;
            launch_pending<=0;load_idx<=0;pending_buf<=0;resume_idx<=0;loading<=0;load_started<=0;
            source_done<=0;wrfin_seen<=0;emergency_d<=0;resume_valid<=0;rearm<=0;
            manual_pending<=0;manual_prev<=0;reload_pending<=0;advance_pending<=0;timer<=0;auto_cnt<=0;retry_cnt<=0;scan_timer<=0;
            for(i=0;i<4;i=i+1)img_sector[i]<=0;
        end else begin
            scan_start<=0;load_start<=0;emergency_d<=emergency;
            // Catalog lookup is a separate cycle from direction/priority selection.
            if(launch_pending && !failed && !load_abort && !rearm)begin
                load_sector<=img_sector[load_idx];load_start<=1;launch_pending<=0;
            end
            if(retry_cnt!=0)retry_cnt<=retry_cnt-1'b1;
            if(!emergency && !resume_valid && ev_auto_toggle)auto_en<=~auto_en;
            // A visitor who turns a page holds the tour on that exhibit; the
            // tour controller resumes it after a quiet period (ev_auto_on).
            else if(EXTERNAL_ADVANCE && !emergency && !resume_valid && (ev_next||ev_prev))auto_en<=0;
            else if(EXTERNAL_ADVANCE && !emergency && !resume_valid && ev_auto_on)auto_en<=1;
            // A dwell that ends during an alarm or while paused is dropped.
            if(!EXTERNAL_ADVANCE || emergency || !auto_en)advance_pending<=0;
            else if(ev_advance)advance_pending<=1;
            if(emergency)manual_pending<=0;
            if(ev_reload)reload_pending<=1;   // kept through an alarm, served after it
            else if(!resume_valid && (ev_next||ev_prev))begin
                manual_pending<=1;manual_prev<=ev_prev; // latest direction wins
            end
            if(emergency && !emergency_d && !resume_valid)begin
                resume_idx<=display_valid ? img_idx : 2'd0;
                resume_valid<=1;
            end
            if(ctrl_idle && !emergency && resume_valid &&
               display_valid && img_idx==resume_idx)resume_valid<=0;
            if(!EXTERNAL_ADVANCE && ctrl_idle && !emergency && !resume_valid && auto_en && display_valid)begin
                if(auto_cnt<AUTO_CYC-1)auto_cnt<=auto_cnt+1'b1;
            end else auto_cnt<=0;

            // Abort is a four-phase handshake. A stuck endpoint never gets
            // bypassed by a timeout: old write ownership must not be reused.
            if(load_abort)begin
                if(abort_done)begin load_abort<=0;rearm<=1;end
            end else if(rearm)begin
                if(!abort_done && pipeline_idle)begin rearm<=0;retry_cnt<=RETRY_CYC;end
            end else if(loading)begin
                if(failed)begin
                    launch_pending<=0;load_abort<=1;loading<=0;swap_req<=0;source_done<=0;wrfin_seen<=0;
                    load_started<=0;err_load<=1;timer<=0;
                end else if(commit_now)begin
                    disp_buf_idx<=pending_buf;img_idx<=load_idx;display_valid<=1;
                    loading<=0;swap_req<=0;source_done<=0;wrfin_seen<=0;timer<=0;err_load<=0;
                end else begin
                    timer<=timer+1'b1;
                    if(load_busy)load_started<=1;
                    if(load_done)source_done<=1;
                    if(wrfin_pulse)wrfin_seen<=1;
                    if(frame_ready)swap_req<=1;
                end
            end

            case(ctrl_state)
                ST_WAIT_SD:begin
                    if(sd_init_done && !load_abort && !rearm && pipeline_idle)begin
                        img_count<=0;safety_found<=0;scan_start<=1;scan_timer<=0;ctrl_state<=ST_SCAN;err_no_image<=0;
                    end else if(scan_timer>=SCAN_CYC-1)begin ctrl_state<=ST_NOIMG;err_no_image<=1;end
                    else scan_timer<=scan_timer+1'b1;
                end
                ST_SCAN:begin
                    scan_timer<=scan_timer+1'b1;
                    if(hit_valid && img_count<4)begin
                        img_sector[img_count]<=hit_sector;img_count<=img_count+1'b1;
                        if(hit_sector==3)begin safety_found<=1;safety_idx<=img_count[1:0];end
                    end
                    if(card_fault || scan_timer>=SCAN_CYC-1)begin
                        load_abort<=1;err_no_image<=1;err_load<=1;ctrl_state<=ST_NOIMG;
                    end else if(scan_done)begin
                        if(img_count==0 && !hit_valid)begin err_no_image<=1;ctrl_state<=ST_NOIMG;end
                        else ctrl_state<=ST_FIRST;
                    end
                end
                ST_FIRST:ctrl_state<=ST_RUN;
                ST_RUN:if(ctrl_idle && request_valid)begin
                    load_idx<=requested_idx;launch_pending<=1;
                    pending_buf<=display_valid ? (disp_buf_idx^2'd1) : 2'd0;
                    write_buf_idx<=display_valid ? (disp_buf_idx^2'd1) : 2'd0;
                    loading<=1;load_started<=0;source_done<=0;wrfin_seen<=0;
                    swap_req<=0;timer<=0;manual_pending<=0;auto_cnt<=0;advance_pending<=0;
                    // Style of this load: first image iris, visitor scroll, else fade.
                    load_kind<=!display_valid ? 2'd2 : (manual_pending||ev_next||ev_prev) ? 2'd1 : 2'd0;
                    // Any new load shows the current mode: the reload is served.
                    if(!ev_reload)reload_pending<=0;
                end
                // Explicit user retry after missing card/scan failure; no
                // unbounded silent rescan loop. KEY1 always resets all blocks.
                ST_NOIMG:if((ev_next||ev_prev||ev_retry) && !load_abort && !rearm)begin
                    ctrl_state<=ST_WAIT_SD;scan_timer<=0;manual_pending<=0;
                end
                default:ctrl_state<=ST_WAIT_SD;
            endcase
        end
    end
endmodule
