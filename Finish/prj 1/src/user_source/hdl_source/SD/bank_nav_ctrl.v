`timescale 1ns/1ps

// 第7课 + 第9课 + 第14课：显示 bank 导航与转场门控，sd_card_clk 域
//   四张图都已预载在各自 bank，切图 = 改 disp_idx（读基地址），不再重新读 SD 卡
//   next/prev/自动轮播：先发 switch_req 给转场模块，等画面淡出到全黑（fade_go）再切 bank，
//                       看不到切换瞬间；转场模块不响应时 SWITCH_TIMEOUT_MS 后直接切，不会卡死
//   缩略图选中：直接切 bank，由转场模块在新帧做中心展开揭示
//   未加载完成/加载失败的图自动跳过；只剩一张可用时不切换
module bank_nav_ctrl #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter integer SWITCH_TIMEOUT_MS = 1500,
    parameter         FADE_EN           = 1
)(
    input            clk,
    input            rst,
    input      [3:0] image_complete,
    input      [2:0] found_count,
    input            next_cmd,
    input            prev_cmd,
    input            select_cmd,
    input      [1:0] select_idx,
    input            auto_cmd,          // 自动轮播开/关
    input      [1:0] period_sel,        // 轮播周期 0:2s 1:3s 2:5s 3:10s
    input wire       navigation_hold,
    input            auto_pause,        // 菜单/缩略图界面中暂停轮播计时
    input            fade_go,           // 转场模块：已淡出到全黑，可以切了（单拍）
    input            fade_busy,
    output reg       switch_req,        // 单拍：请求转场
    output reg       switch_auto,       // 本次请求来自自动轮播（转场用擦除揭示）
    output reg [1:0] disp_idx,
    output reg       disp_valid,
    output reg       pic_changed,
    output reg       auto_on
);
    localparam [31:0] SW_TIMEOUT = (CLK_FREQ_HZ / 1000) * SWITCH_TIMEOUT_MS;
    localparam [31:0] P2  = CLK_FREQ_HZ * 2;
    localparam [31:0] P3  = CLK_FREQ_HZ * 3;
    localparam [31:0] P5  = CLK_FREQ_HZ * 5;
    localparam [31:0] P10 = CLK_FREQ_HZ * 10;

    reg        sw_pend;
    reg        sw_dir;              // 0=下一张 1=上一张
    reg [31:0] sw_timer;
    reg [31:0] auto_cnt;

    wire [31:0] auto_cycles = (period_sel == 2'd0) ? P2 : (period_sel == 2'd1) ? P3 :
                              (period_sel == 2'd2) ? P5 : P10;
    wire        auto_tick   = (auto_cnt >= auto_cycles - 32'd1);
    wire [3:0]  others      = image_complete & ~(4'b0001 << disp_idx);
    wire        can_switch  = disp_valid && (others != 4'd0);

    // 按方向找下一个“已加载完成”的 bank（2 位自然回绕；未找到的图位图为 0，自动跳过）
    function [1:0] step_idx;
        input [1:0] cur;
        input       dir;
        input [3:0] mask;
        reg   [1:0] c1, c2, c3;
        begin
            c1 = dir ? cur - 2'd1 : cur + 2'd1;
            c2 = dir ? cur - 2'd2 : cur + 2'd2;
            c3 = dir ? cur - 2'd3 : cur + 2'd3;
            step_idx = mask[c1] ? c1 : mask[c2] ? c2 : mask[c3] ? c3 : cur;
        end
    endfunction

    function [1:0] lowest;
        input [3:0] m;
        lowest = m[0] ? 2'd0 : m[1] ? 2'd1 : m[2] ? 2'd2 : 2'd3;
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            switch_req <= 1'b0;  switch_auto <= 1'b0;  disp_idx <= 2'd0;
            disp_valid <= 1'b0;  pic_changed <= 1'b0;  auto_on <= 1'b0;
            sw_pend <= 1'b0;     sw_dir <= 1'b0;       sw_timer <= 32'd0;
            auto_cnt <= 32'd0;
        end else begin
            switch_req  <= 1'b0;
            pic_changed <= 1'b0;

            if (auto_cmd && found_count > 3'd1)
                auto_on <= ~auto_on;

            // 轮播计时：到点后停在 auto_tick，等转场空闲再发请求
            if (auto_on && can_switch && !sw_pend && !auto_pause && !auto_cmd) begin
                if (!auto_tick) auto_cnt <= auto_cnt + 32'd1;
            end else
                auto_cnt <= 32'd0;

            if (navigation_hold) begin
                sw_pend<=0;sw_timer<=0;
            end else if (!disp_valid) begin
                // 首张图预载完成：立即上屏（转场模块做黑场淡入）
                if (image_complete != 4'd0) begin
                    disp_idx <= lowest(image_complete);
                    disp_valid <= 1'b1;  pic_changed <= 1'b1;
                end
            end else if (sw_pend) begin
                if (fade_go || sw_timer >= SW_TIMEOUT) begin
                    sw_pend <= 1'b0;  sw_timer <= 32'd0;
                    if (can_switch) begin
                        disp_idx <= step_idx(disp_idx, sw_dir, image_complete);
                        pic_changed <= 1'b1;
                    end
                end else
                    sw_timer <= sw_timer + 32'd1;
            end else if (select_cmd) begin
                auto_cnt <= 32'd0;
                if (image_complete[select_idx] && select_idx != disp_idx) begin
                    disp_idx <= select_idx;  pic_changed <= 1'b1;
                end
            end else if ((next_cmd || prev_cmd) && can_switch && !fade_busy) begin
                auto_cnt <= 32'd0;  switch_auto <= 1'b0;
                if (FADE_EN) begin
                    sw_pend <= 1'b1;  sw_timer <= 32'd0;  switch_req <= 1'b1;
                    sw_dir  <= !next_cmd;                  // 同时按下时下一张优先
                end else begin
                    disp_idx <= step_idx(disp_idx, !next_cmd, image_complete);
                    pic_changed <= 1'b1;
                end
            end else if (auto_on && auto_tick && can_switch && !fade_busy && !auto_pause) begin
                auto_cnt <= 32'd0;  switch_auto <= 1'b1;
                if (FADE_EN) begin
                    sw_pend <= 1'b1;  sw_timer <= 32'd0;  switch_req <= 1'b1;  sw_dir <= 1'b0;
                end else begin
                    disp_idx <= step_idx(disp_idx, 1'b0, image_complete);
                    pic_changed <= 1'b1;
                end
            end
        end
    end
endmodule
