// 512-point, stereo-power FFT analyzer. All logic is in video_clk.
// One Hann-windowed block per 1024 actual 48kHz samples (46.875 updates/s).
// Computation fits inside the following 512 samples; it never stalls audio.
module spectrum_analyzer(
    input wire clk,rst,emergency,
    input wire sample_valid,
    input wire [23:0] sample_left,sample_right,
    output reg [95:0] levels,
    output reg levels_valid,
    output reg overrun
);
    localparam WAIT=0,CAP_MUL=1,CAP_L=2,CAP_R=3,
        READ_A=4,READ_B=5,MUL=6,ROTATE=7,WRITE_A=8,WRITE_B=9,
        POWER_READ=10,POWER_MUL=11,POWER_ACC=12,LEVEL=13,PUBLISH=14;
    reg [3:0] state;
    reg [9:0] phase;
    reg [8:0] cap_index;
    reg signed [15:0] cap_left,cap_right;
    reg signed [15:0] window_value;
    reg signed [31:0] window_left,window_right;
    reg signed [17:0] real_mem[0:1023],imag_mem[0:1023];
    reg signed [15:0] hann[0:511];
    reg [31:0] twiddle[0:255];
    initial begin
        $readmemh("spectrum_hann.hex",hann);
        $readmemh("spectrum_twiddle.hex",twiddle);
    end
    function [8:0] reverse9;
        input [8:0] a;
        integer j;
        begin for(j=0;j<9;j=j+1) reverse9[j]=a[8-j]; end
    endfunction
    reg channel;
    reg [3:0] stage;
    reg [8:0] base,offset;
    reg [8:0] half_size,tw_step;
    reg [7:0] tw_addr;
    wire [9:0] addr_a={channel,base+offset};
    wire [9:0] addr_b=addr_a+{1'b0,half_size};
    reg signed [17:0] ar,ai,tr,ti;
    reg signed [15:0] cr,ci;
    reg signed [33:0] p0,p1,p2,p3;
    wire signed [34:0] rot_r={p0[33],p0}-{p1[33],p1};
    wire signed [34:0] rot_i={p2[33],p2}+{p3[33],p3};
    wire signed [18:0] sum_r={ar[17],ar}+{tr[17],tr};
    wire signed [18:0] sum_i={ai[17],ai}+{ti[17],ti};
    wire signed [18:0] diff_r={ar[17],ar}-{tr[17],tr};
    wire signed [18:0] diff_i={ai[17],ai}-{ti[17],ti};
    reg [7:0] power_index;
    reg signed [17:0] ram_r,ram_i;
    wire [9:0] read_addr=state==READ_A ? addr_a : state==READ_B ? addr_b : {channel,1'b0,power_index};
    reg [35:0] square_r,square_i;
    reg [39:0] energy[0:15];
    reg [3:0] level_index;
    reg [5:0] smoothed[0:15];
    reg [95:0] pending_levels;
    function [3:0] band_of;
        input [7:0] k;
        begin
            if(k<2)band_of=0; else if(k<3)band_of=1;
            else if(k<4)band_of=2; else if(k<5)band_of=3;
            else if(k<7)band_of=4; else if(k<9)band_of=5;
            else if(k<12)band_of=6; else if(k<16)band_of=7;
            else if(k<21)band_of=8; else if(k<28)band_of=9;
            else if(k<37)band_of=10; else if(k<49)band_of=11;
            else if(k<64)band_of=12; else if(k<81)band_of=13;
            else if(k<102)band_of=14; else band_of=15;
        end
    endfunction
    function [5:0] height_of;
        input [39:0] power;
        integer bitno,highest,h;
        begin
            highest=0;
            for(bitno=0;bitno<40;bitno=bitno+1) if(power[bitno])highest=bitno;
            h=0;
            if(highest>=7) h=2*(highest-7)+(power>>(highest-1)&1);
            height_of=h>40?6'd40:h;
        end
    endfunction
    wire [5:0] target=height_of(energy[level_index]);
    wire [6:0] rise={1'b0,target}-{1'b0,smoothed[level_index]};
    wire [6:0] fall={1'b0,smoothed[level_index]}-{1'b0,target};
    wire [5:0] next_level=target>smoothed[level_index] ?
        smoothed[level_index]+((rise+1)>>1) :
        smoothed[level_index]-((fall+3)>>2);
    integer n;
    // Explicit single write port. TD does not merge the four conditional
    // array assignments reliably even though their state conditions exclude.
    reg [9:0] write_addr;
    reg signed [17:0] write_real,write_imag;
    wire write_enable=!rst && !emergency &&
        (state==CAP_L || state==CAP_R || state==WRITE_A || state==WRITE_B);
    always @(*) begin
        write_addr=0;write_real=0;write_imag=0;
        case(state)
            CAP_L: begin
                write_addr={1'b0,reverse9(cap_index)};
                write_real=window_left>>>15;
            end
            CAP_R: begin
                write_addr={1'b1,reverse9(cap_index)};
                write_real=window_right>>>15;
            end
            WRITE_A: begin write_addr=addr_a;write_real=sum_r>>>1;write_imag=sum_i>>>1;end
            WRITE_B: begin write_addr=addr_b;write_real=diff_r>>>1;write_imag=diff_i>>>1;end
        endcase
    end
    // RAM has no reset; complete capture overwrites every input before FFT.
    always @(posedge clk) if(write_enable) begin
        real_mem[write_addr]<=write_real;imag_mem[write_addr]<=write_imag;
    end
    always @(posedge clk) begin
        // One physical synchronous read port per RAM; separate conditional
        // reads into ar/br/er would infer three ports and turn RAM into flops.
        ram_r<=real_mem[read_addr];ram_i<=imag_mem[read_addr];
        if(state==READ_B)begin
            ar<=ram_r;ai<=ram_i;
            cr<=twiddle[tw_addr][31:16];ci<=twiddle[tw_addr][15:0];
        end
        if(state==WAIT && sample_valid) window_value<=hann[phase[8:0]];
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=WAIT;phase<=0;levels<=0;levels_valid<=0;overrun<=0;
            cap_index<=0;cap_left<=0;cap_right<=0;window_left<=0;window_right<=0;
            channel<=0;stage<=0;base<=0;offset<=0;half_size<=1;tw_step<=256;tw_addr<=0;
            p0<=0;p1<=0;p2<=0;p3<=0;tr<=0;ti<=0;
            power_index<=1;square_r<=0;square_i<=0;level_index<=0;pending_levels<=0;
            for(n=0;n<16;n=n+1) begin energy[n]<=0;smoothed[n]<=0;end
        end else if(emergency) begin
            state<=WAIT;phase<=0;levels<=0;levels_valid<=0;pending_levels<=0;overrun<=0;
            for(n=0;n<16;n=n+1) begin energy[n]<=0;smoothed[n]<=0;end
        end else begin
            levels_valid<=0;
            if(sample_valid)begin
                phase<=phase+1'b1;
                if(phase<512 && state!=WAIT)overrun<=1;
            end
            case(state)
                WAIT: if(sample_valid && phase<512) begin
                    cap_index<=phase[8:0];cap_left<=sample_left[23:8];cap_right<=sample_right[23:8];state<=CAP_MUL;
                    if(phase==0)for(n=0;n<16;n=n+1)energy[n]<=0;
                end
                CAP_MUL:begin window_left<=cap_left*window_value;window_right<=cap_right*window_value;state<=CAP_L;end
                CAP_L:state<=CAP_R;
                CAP_R:begin
                    state<=WAIT;
                    if(cap_index==511)begin
                        channel<=0;stage<=0;base<=0;offset<=0;half_size<=1;tw_step<=256;tw_addr<=0;state<=READ_A;
                    end
                end
                READ_A:state<=READ_B;
                READ_B:state<=MUL;
                MUL:begin p0<=ram_r*cr;p1<=ram_i*ci;p2<=ram_r*ci;p3<=ram_i*cr;state<=ROTATE;end
                ROTATE:begin tr<=rot_r>>>15;ti<=rot_i>>>15;state<=WRITE_A;end
                WRITE_A:state<=WRITE_B;
                WRITE_B:begin
                    state<=READ_A;
                    if(offset==half_size-1)begin
                        offset<=0;tw_addr<=0;
                        if({1'b0,base}+({1'b0,half_size}<<1)==512)begin
                            base<=0;
                            if(stage==8)begin
                                if(!channel)begin channel<=1;stage<=0;half_size<=1;tw_step<=256;end
                                else begin channel<=0;power_index<=1;state<=POWER_READ;end
                            end else begin stage<=stage+1'b1;half_size<=half_size<<1;tw_step<=tw_step>>1;end
                        end else base<=base+(half_size<<1);
                    end else begin offset<=offset+1'b1;tw_addr<=tw_addr+tw_step;end
                end
                POWER_READ:state<=POWER_MUL;
                POWER_MUL:begin square_r<=ram_r*ram_r;square_i<=ram_i*ram_i;state<=POWER_ACC;end
                POWER_ACC:begin
                    energy[band_of(power_index)]<=energy[band_of(power_index)]+{4'd0,square_r}+{4'd0,square_i};
                    state<=POWER_READ;
                    if(power_index==128)begin
                        if(!channel)begin channel<=1;power_index<=1;end
                        else begin level_index<=0;state<=LEVEL;end
                    end else power_index<=power_index+1'b1;
                end
                LEVEL:begin
                    smoothed[level_index]<=next_level;
                    pending_levels[level_index*6+:6]<=next_level;
                    if(level_index==15)state<=PUBLISH;else level_index<=level_index+1'b1;
                end
                PUBLISH:begin levels<=pending_levels;levels_valid<=1;state<=WAIT;end
                default:state<=WAIT;
            endcase
        end
    end
endmodule
