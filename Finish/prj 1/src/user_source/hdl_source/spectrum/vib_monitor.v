`timescale 1ns/1ps
// 设备声学 / 振动监测（video_clk 域）
//   WAV 放泵、电机的运行录音（或现场拾音），spectrum_dft64 实时算 64 段频谱；
//   本模块在每帧第 SAMPLE_Y 行顺着频谱读口取一遍 64 根柱高（不占额外 RAM 读口）：
//     level   = 高频段（柱 32~63，约 3kHz 以上）最大柱高 0~80：轴承磨损、气蚀、松动的能量主要在这里
//     peak_hz = 全频段最高柱对应的中心频率（柱号 -> DFT 频点 k，k*93.75Hz 近似为 k*94）
//   level >= lim 持续 HOLD 帧（约 1 秒）弹无声警告，回落后同样经过 HOLD 帧才消失，避免抖动。
//   内部 48kHz 测试音落在低频柱，不会触发振动警告；无声时 level=0
module vib_monitor #(
    parameter integer X0 = 64,                // 与 spectrum_overlay 的 X0 一致
    parameter integer SAMPLE_Y = 4,
    parameter integer HOLD = 60
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       frame_tick,
    input  wire       de,
    input  wire [9:0] x,                      // 与 h 同拍（频谱读地址早一拍发出）
    input  wire [9:0] y,
    input  wire [6:0] h,
    input  wire [6:0] lim,
    output reg  [6:0] level,
    output reg [15:0] peak_hz,
    output reg        peak_ok,                // 频谱里有明显峰值
    output reg        warn
);
    // 与 spectrum_dft64 的 bin_k 表一致
    function [7:0] bin_k;
        input [5:0] i;
        case (i)
            6'd33: bin_k = 8'd36;  6'd34: bin_k = 8'd38;  6'd35: bin_k = 8'd41;  6'd36: bin_k = 8'd43;
            6'd37: bin_k = 8'd46;  6'd38: bin_k = 8'd49;  6'd39: bin_k = 8'd52;  6'd40: bin_k = 8'd55;
            6'd41: bin_k = 8'd59;  6'd42: bin_k = 8'd62;  6'd43: bin_k = 8'd66;  6'd44: bin_k = 8'd70;
            6'd45: bin_k = 8'd75;  6'd46: bin_k = 8'd79;  6'd47: bin_k = 8'd84;  6'd48: bin_k = 8'd89;
            6'd49: bin_k = 8'd95;  6'd50: bin_k = 8'd101; 6'd51: bin_k = 8'd107; 6'd52: bin_k = 8'd113;
            6'd53: bin_k = 8'd120; 6'd54: bin_k = 8'd127; 6'd55: bin_k = 8'd135; 6'd56: bin_k = 8'd143;
            6'd57: bin_k = 8'd151; 6'd58: bin_k = 8'd160; 6'd59: bin_k = 8'd169; 6'd60: bin_k = 8'd179;
            6'd61: bin_k = 8'd190; 6'd62: bin_k = 8'd201; 6'd63: bin_k = 8'd213;
            default: bin_k = {2'd0, i} + 8'd1;  // 柱 0~32：k = 柱号 + 1
        endcase
    endfunction

    wire [9:0] rx = x - X0;
    wire       samp = de && y == SAMPLE_Y && x >= X0 && rx < 10'd512 && rx[2:0] == 3'd0;
    wire [5:0] bar = rx[8:3];
    reg  [6:0] hf_max, all_max;
    reg  [5:0] all_bar;
    reg  [6:0] cnt;
    wire [15:0] k = {8'd0, bin_k(all_bar)};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level <= 0; peak_hz <= 0; peak_ok <= 0; warn <= 0;
            hf_max <= 0; all_max <= 0; all_bar <= 0; cnt <= 0;
        end else if (frame_tick) begin
            level   <= hf_max;
            peak_ok <= all_max >= 7'd8;
            peak_hz <= (k << 6) + (k << 4) + (k << 3) + (k << 2) + (k << 1);   // k*94
            if (hf_max >= lim) begin
                if (cnt < HOLD) cnt <= cnt + 1'b1; else warn <= 1'b1;
            end else begin
                if (cnt != 0) cnt <= cnt - 1'b1; else warn <= 1'b0;
            end
            hf_max <= 0; all_max <= 0; all_bar <= 0;
        end else if (samp) begin
            if (bar[5] && h > hf_max) hf_max <= h;
            if (h > all_max) begin all_max <= h; all_bar <= bar; end
        end
    end
endmodule
