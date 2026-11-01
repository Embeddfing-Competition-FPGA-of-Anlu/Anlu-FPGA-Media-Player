`timescale 1ns/1ps

// 第9课 图片转场特效（完赛整合版，基于课程 video_fade_transition 改写）
//   单读口方案：SDRAM 一帧只读一张图，所以“旧图淡出到全黑 -> 切 bank -> 新图进入”
//   手动切图：淡出 -> 黑场 -> 淡入（Alpha 渐变）
//   自动轮播：淡出 -> 黑场 -> 擦除揭示，方向在 左->右 / 上->下 / 右->左 间轮换
//   缩略图选中：直接切 bank，新图从中心向四周展开
//   首张图上屏：黑场淡入
//   相对课程版：增加黑场超时（切 bank 失败也能恢复），增加自动/选中两种揭示方式
module video_fade_transition #(
    parameter integer FADE_FRAMES  = 16,      // 256 / FADE_FRAMES 为整数
    parameter integer HOLD_TIMEOUT = 30       // 黑场最多等待帧数
)(
    // 控制域 sd_card_clk
    input  wire       ctrl_clk,
    input  wire       ctrl_rst_n,
    input  wire       switch_req,       // 单拍
    input  wire       switch_auto,
    output wire       fade_go,          // 单拍：已全黑，可以切 bank
    output wire       fade_busy,
    // 像素域 video_clk
    input  wire       pix_clk,
    input  wire       pix_rst_n,
    input  wire       player_active,    // 已帧边界锁存
    input  wire       frame_start,
    input  wire       index_commit,     // 帧边界：显示 bank 已更换
    output wire [7:0] alpha,
    output wire       wipe_active,
    output wire [1:0] wipe_kind,        // 0 左->右 1 上->下 2 右->左 3 中心展开
    output wire [4:0] ramp
);
    localparam integer STEP = 256 / FADE_FRAMES;
    localparam [1:0] S_IDLE = 2'd0, S_OUT = 2'd1, S_HOLD = 2'd2, S_IN = 2'd3;

    reg [1:0] state;                     // 像素域状态（控制域只读它的 busy 位，经两级同步）

    // ---------- 控制域 ----------
    reg req_tgl, auto_hold;
    reg cdc_commit_s1, commit_s2, ack_seen;
    reg cdc_busy_s1, busy_s2;
    reg commit_tgl;                      // 像素域

    always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n) begin
            req_tgl <= 1'b0;  auto_hold <= 1'b0;
            cdc_commit_s1 <= 1'b0;  commit_s2 <= 1'b0;  ack_seen <= 1'b0;
            cdc_busy_s1 <= 1'b0;    busy_s2 <= 1'b0;
        end else begin
            cdc_busy_s1   <= (state != S_IDLE);
            busy_s2       <= cdc_busy_s1;
            cdc_commit_s1 <= commit_tgl;
            commit_s2     <= cdc_commit_s1;
            if (commit_s2 != ack_seen)
                ack_seen <= commit_s2;
            if (switch_req) begin
                req_tgl   <= ~req_tgl;
                auto_hold <= switch_auto;   // 与 req 同时更新，像素域看到 req 翻转时它早已稳定
            end
        end
    end
    assign fade_go   = (commit_s2 != ack_seen);
    assign fade_busy = busy_s2;

    // ---------- 像素域 ----------
    // state 已在上方声明
    reg [4:0] rcnt;
    reg [5:0] hold_cnt;
    reg       cdc_req_s1, req_s2, req_seen;
    reg       pa_q;
    reg       wipe_on;
    reg [1:0] kind, auto_kind;
    wire      req_new = (req_s2 != req_seen);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            state <= S_IDLE;  rcnt <= 5'd0;  hold_cnt <= 6'd0;
            cdc_req_s1 <= 1'b0;  req_s2 <= 1'b0;  req_seen <= 1'b0;
            commit_tgl <= 1'b0;  pa_q <= 1'b0;  wipe_on <= 1'b0;
            kind <= 2'd0;  auto_kind <= 2'd0;
        end else begin
            cdc_req_s1 <= req_tgl;
            req_s2     <= cdc_req_s1;
            pa_q       <= player_active;
            case (state)
                S_IDLE: begin
                    if (req_new) begin
                        req_seen <= req_s2;
                        if (player_active) begin
                            state <= S_OUT;  rcnt <= FADE_FRAMES[4:0];
                        end else
                            commit_tgl <= ~commit_tgl;      // 未上屏：直接放行
                    end else if (index_commit && player_active) begin
                        state <= S_IN;  rcnt <= 5'd0;  wipe_on <= 1'b1;  kind <= 2'd3;
                    end else if (player_active && !pa_q) begin
                        state <= S_IN;  rcnt <= 5'd0;  wipe_on <= 1'b0;
                    end
                end
                S_OUT: if (frame_start) begin
                    if (rcnt == 5'd0) begin
                        state <= S_HOLD;  hold_cnt <= 6'd0;
                        commit_tgl <= ~commit_tgl;
                    end else
                        rcnt <= rcnt - 5'd1;
                end
                S_HOLD: begin
                    if (index_commit || (frame_start && hold_cnt >= HOLD_TIMEOUT)) begin
                        state <= S_IN;  rcnt <= 5'd0;
                        wipe_on <= auto_hold;
                        kind    <= auto_kind;
                        if (auto_hold)
                            auto_kind <= (auto_kind == 2'd2) ? 2'd0 : auto_kind + 2'd1;
                    end else if (frame_start)
                        hold_cnt <= hold_cnt + 6'd1;
                end
                default: if (frame_start) begin     // S_IN
                    if (rcnt >= FADE_FRAMES - 1) begin
                        state <= S_IDLE;  wipe_on <= 1'b0;
                    end else
                        rcnt <= rcnt + 5'd1;
                end
            endcase
            if (state != S_IDLE && req_new)
                req_seen <= req_s2;                 // 转场中的新请求丢弃（导航侧有超时兜底）
        end
    end

    wire [8:0] ramp_a = rcnt * STEP;
    assign alpha = (state == S_IDLE) ? 8'd255 :
                   (state == S_HOLD) ? 8'd0 :
                   (state == S_IN && wipe_on) ? 8'd255 :
                   (state == S_OUT && rcnt == FADE_FRAMES) ? 8'd255 :
                   (ramp_a > 9'd255) ? 8'd255 : ramp_a[7:0];
    assign wipe_active = (state == S_IN) && wipe_on;
    assign wipe_kind   = kind;
    assign ramp        = rcnt;
endmodule
