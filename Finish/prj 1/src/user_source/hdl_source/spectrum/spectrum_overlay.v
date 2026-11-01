`timescale 1ns/1ps

// 扩展5：频谱柱状图叠加（video_clk 域，组合输出，与波形层同级）
//   区域 X0..X0+511，底边 Y_BOT，最高 80 像素；64 根柱，每根 6 像素宽 + 2 像素间隔
//   柱高从频谱模块的显示 RAM 读：上一拍用 pixel_x_pre（早一拍的坐标）发地址，本拍拿到数据
//   颜色按高度分段：绿（<40）/ 黄（<64）/ 红；峰值为白色横线；背景为半透明暗底
module spectrum_overlay #(
    parameter integer X0    = 64,
    parameter integer Y_BOT = 440,
    parameter integer H_MAX = 80
)(
    input  wire        de,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_x_pre,   // 早一拍的 pixel_x，用来生成 RAM 读地址
    input  wire [8:0]  pixel_y,
    input  wire        enable,
    input  wire [23:0] rgb_in,
    output wire [5:0]  rd_bar,
    input  wire [6:0]  h,
    input  wire [6:0]  pk,
    output wire [23:0] rgb_out
);
    wire [9:0] rx_pre = pixel_x_pre - X0;
    assign rd_bar = rx_pre[8:3];

    wire [9:0] rx    = pixel_x - X0;
    wire [9:0] ry    = Y_BOT - 1 - {1'b0, pixel_y};          // 0 = 最底行
    wire       in_pn = enable && de && (pixel_x >= X0) && (rx < 10'd512) &&
                       ({1'b0, pixel_y} < Y_BOT) && (ry < H_MAX + 2);
    wire       gap   = (rx[2:0] >= 3'd6);
    wire       body  = in_pn && !gap && (ry < {3'd0, h});
    wire       cap   = in_pn && !gap && (pk != 7'd0) && (ry == {3'd0, pk} || ry == {3'd0, pk} + 10'd1);

    wire [23:0] bar_c = (ry < 10'd40) ? 24'h20e060 :
                        (ry < 10'd64) ? 24'hf0e020 : 24'hff3020;
    wire [23:0] dim   = {2'b0, rgb_in[23:18], 2'b0, rgb_in[15:10], 2'b0, rgb_in[7:2]};   // 1/4 亮度

    assign rgb_out = cap  ? 24'hffffff :
                     body ? bar_c :
                     in_pn ? dim : rgb_in;
endmodule
