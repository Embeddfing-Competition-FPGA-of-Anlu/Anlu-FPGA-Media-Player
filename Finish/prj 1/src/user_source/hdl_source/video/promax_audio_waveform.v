`timescale 1ns/1ps

// 第11课 音频波形（稳定版，video_clk 域）
//   原版问题：每帧从任意相位抓 32 个点（每 12 个样本取 1 个），相位每帧都不同，大音量时整条线乱跳
//   改进（示波器做法）：
//     1. 上升沿过零触发 + 迟滞：先等样本低于 -TRIG_TH（布防），再等它回到 >=0 才开始采集，
//        每次抓到的波形起点相位一致，画面基本静止；静音/直流时超时自由触发，不会卡住
//     2. 512 点 x 1 像素，每 2 个样本求平均取 1 点（≈21ms 窗口，同时做简单低通去掉毛刺）
//     3. 相邻两列连线（画两点之间的竖线段），不再是离散的点
//     4. 双缓冲在 BRAM 里，采满一缓冲后在场同步处交换，显示中的缓冲永远完整，不撕裂
//     5. 自动增益：每抓满一缓冲统计峰值，让下一屏峰值落在框高的 1/2~1 之间；
//        变大立即减增益（不冲框），变小每 8 屏才加一档（不忽大忽小）；最大 64 倍，最后再限幅 ±30
//   去掉了原来右侧的橙黄色峰值条（安静时只剩右下角一个小黄块）
//   WAVE_WIDTH 固定 512（读地址 9 位）
module promax_audio_waveform #(
    parameter integer WAVE_X       = 64,
    parameter integer WAVE_Y       = 400,
    parameter integer WAVE_WIDTH   = 512,
    parameter integer WAVE_HEIGHT  = 64,
    parameter integer TRIG_TH      = 1024,   // 迟滞门限（约 -30dB）
    parameter integer TRIG_TIMEOUT = 480     // 10ms 内没触发就自由触发
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_x_pre,       // 早一拍的 pixel_x，用来发 BRAM 读地址
    input  wire        enable,
    input  wire        pcm_tap_valid,
    input  wire signed [15:0] pcm_tap_left,
    output wire [23:0] rgb_out,
    output wire        waveform_active
);
    localparam integer CENTER_Y = WAVE_Y + WAVE_HEIGHT/2;
    localparam [1:0] S_TRIG = 2'd0, S_CAP = 2'd1, S_DONE = 2'd2;

    reg [11:0] line_y;
    reg de_q, vs_q;
    wire frame_start = vs_q && !vsync;

    // ---------- 采集 ----------
    reg  [7:0]  mem [0:1023];               // {bank, 9 位列号} -> 有符号偏移（向上为正）
    reg  [1:0]  st;
    reg         armed, half, wbank, dbank, dvalid;
    reg  [9:0]  wait_cnt;
    reg  [8:0]  wr_idx;
    reg  signed [15:0] acc;
    reg         we;
    reg  [9:0]  wa;
    reg  [7:0]  wd;
    wire signed [16:0] sum  = acc + pcm_tap_left;          // 两样本之和
    reg  [2:0]  gsh;                                       // 右移量 = 5 + gsh（5..11）
    reg  [15:0] pk;                                        // 本缓冲 |sum| 峰值
    reg  [2:0]  gdec;
    wire signed [16:0] offs = sum >>> (5 + gsh);
    wire [16:0] sabs = sum[16] ? -sum : sum;
    // 峰值最高位 -> 目标右移量：peak >> sh 落在 [16,32)
    wire [4:0]  pmsb = pk[15] ? 5'd15 : pk[14] ? 5'd14 : pk[13] ? 5'd13 : pk[12] ? 5'd12 :
                       pk[11] ? 5'd11 : pk[10] ? 5'd10 : pk[9]  ? 5'd9  : 5'd8;
    wire [4:0]  gt5  = pmsb - 5'd9;
    wire [2:0]  gtgt = (pmsb <= 5'd9) ? 3'd0 : (gt5 >= 5'd6) ? 3'd6 : gt5[2:0];
    wire [7:0]  v_clamp = (offs > 17'sd30)  ? 8'd30 :
                          (offs < -17'sd30) ? 8'hE2 : offs[7:0];   // -30 = 8'hE2

    always @(posedge clk) if (we) mem[wa] <= wd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            line_y <= 12'd0;  de_q <= 1'b0;  vs_q <= 1'b1;
            st <= S_TRIG;  armed <= 1'b0;  half <= 1'b0;  wbank <= 1'b0;  dbank <= 1'b1;  dvalid <= 1'b0;
            wait_cnt <= 10'd0;  wr_idx <= 9'd0;  acc <= 16'sd0;
            gsh <= 3'd6;  pk <= 16'd0;  gdec <= 3'd0;
            we <= 1'b0;  wa <= 10'd0;  wd <= 8'd0;
        end else begin
            de_q <= de;
            vs_q <= vsync;
            if (!vsync)
                line_y <= 12'd0;
            else if (de_q && !de)
                line_y <= line_y + 1'b1;

            we <= 1'b0;
            case (st)
                S_TRIG: if (pcm_tap_valid) begin
                    if (wait_cnt != 10'h3ff) wait_cnt <= wait_cnt + 10'd1;
                    if (pcm_tap_left < -TRIG_TH) armed <= 1'b1;
                    if ((armed && pcm_tap_left >= 0) || wait_cnt >= TRIG_TIMEOUT) begin
                        st <= S_CAP;  wr_idx <= 9'd0;  half <= 1'b0;
                    end
                end
                S_CAP: if (pcm_tap_valid) begin
                    if (!half) begin
                        acc <= pcm_tap_left;  half <= 1'b1;
                    end else begin
                        half <= 1'b0;
                        we <= 1'b1;  wa <= {wbank, wr_idx};  wd <= v_clamp;
                        if (sabs[15:0] > pk && !sabs[16]) pk <= sabs[15:0];
                        else if (sabs[16])                pk <= 16'hffff;
                        if (wr_idx == 9'd511) st <= S_DONE;
                        else                  wr_idx <= wr_idx + 9'd1;
                    end
                end
                default: ;                                      // S_DONE：等场同步交换
            endcase

            if (frame_start && st == S_DONE) begin
                dbank <= wbank;  wbank <= ~wbank;  dvalid <= 1'b1;
                pk <= 16'd0;
                if (gtgt > gsh) begin gsh <= gtgt;  gdec <= 3'd0; end
                else if (gtgt < gsh) begin
                    gdec <= gdec + 3'd1;
                    if (gdec == 3'd7) gsh <= gsh - 3'd1;
                end else gdec <= 3'd0;
                st <= S_TRIG;  armed <= 1'b0;  wait_cnt <= 10'd0;
            end
        end
    end

    // ---------- 显示：读当前列和上一列，画两点之间的竖线段 ----------
    wire [9:0] rx_pre = pixel_x_pre - WAVE_X;
    reg  [7:0] rd_q, prev_q;
    always @(posedge clk) begin
        rd_q   <= mem[{dbank, rx_pre[8:0]}];
        prev_q <= rd_q;
    end

    wire [9:0]  rx     = pixel_x - WAVE_X;
    wire        in_box = enable && de && (pixel_x >= WAVE_X) && (rx < WAVE_WIDTH) &&
                         (line_y >= WAVE_Y) && (line_y < WAVE_Y + WAVE_HEIGHT);
    wire signed [8:0]  cur  = $signed({rd_q[7], rd_q});
    wire signed [8:0]  prv  = (rx == 10'd0) ? cur : $signed({prev_q[7], prev_q});
    wire signed [8:0]  hi   = (cur > prv) ? cur : prv;
    wire signed [8:0]  lo   = (cur > prv) ? prv : cur;
    wire signed [12:0] dy   = $signed(CENTER_Y) - $signed({1'b0, line_y});   // 相对中心，向上为正
    wire wave_pixel = in_box && dvalid && (dy >= lo - 9'sd1) && (dy <= hi + 9'sd1);
    wire axis_pixel = in_box && (line_y == CENTER_Y);

    assign waveform_active = wave_pixel || axis_pixel;
    assign rgb_out = wave_pixel ? 24'h20ff90 :
                     axis_pixel ? 24'h406060 : rgb_in;
endmodule
