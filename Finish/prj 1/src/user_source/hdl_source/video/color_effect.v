`timescale 1ns/1ps

// 扩展：逐像素色彩特效（点运算，组合逻辑，不占拍；只用移位和加法，不用 DSP）
//   0 关  1 灰度  2 怀旧(棕褐)  3 负片  4 色调分离(每通道 4 级)  5 二值化  6 暖色  7 冷色
//   亮度 Y = (77R + 150G + 29B) >> 8，用移位近似：R*(64+8+4+1) G*(128+16+4+2) B*(16+8+4+1)
module color_effect (
    input  wire [2:0]  mode,
    input  wire [23:0] rgb_in,
    output reg  [23:0] rgb_out
);
    wire [7:0] r = rgb_in[23:16];
    wire [7:0] g = rgb_in[15:8];
    wire [7:0] b = rgb_in[7:0];
    wire [15:0] ysum = ({8'd0, r} << 6) + ({8'd0, r} << 3) + ({8'd0, r} << 2) + {8'd0, r} +
                       ({8'd0, g} << 7) + ({8'd0, g} << 4) + ({8'd0, g} << 2) + ({8'd0, g} << 1) +
                       ({8'd0, b} << 4) + ({8'd0, b} << 3) + ({8'd0, b} << 2) + {8'd0, b};
    wire [7:0] y = ysum[15:8];

    // 饱和加减
    function [7:0] sat_add;
        input [7:0] a;
        input [7:0] d;
        reg [8:0] s;
        begin s = a + d; sat_add = s[8] ? 8'd255 : s[7:0]; end
    endfunction
    function [7:0] sat_sub;
        input [7:0] a;
        input [7:0] d;
        begin sat_sub = (a > d) ? a - d : 8'd0; end
    endfunction

    always @* begin
        case (mode)
            3'd1: rgb_out = {y, y, y};
            3'd2: rgb_out = {sat_add(y, 8'd40), sat_add(y, 8'd16), sat_sub(y, 8'd24)};
            3'd3: rgb_out = ~rgb_in;
            3'd4: rgb_out = {r[7:6], r[7:6], r[7:6], r[7:6], g[7:6], g[7:6], g[7:6], g[7:6],
                             b[7:6], b[7:6], b[7:6], b[7:6]};
            3'd5: rgb_out = y[7] ? 24'hffffff : 24'h000000;
            3'd6: rgb_out = {sat_add(r, 8'd32), sat_add(g, 8'd8), sat_sub(b, 8'd32)};
            3'd7: rgb_out = {sat_sub(r, 8'd32), sat_add(g, 8'd8), sat_add(b, 8'd32)};
            default: rgb_out = rgb_in;
        endcase
    end
endmodule
