`timescale 1ns/1ps

// 第13课 亮度对比度调节（完赛整合版，补全课程骨架中的 TODO），video_clk 域，固定 2 拍延迟
//   增益为 Q1.7 定点：128 = 1.0 倍
//   第 1 拍 亮度：y = x * bright_gain / 128，饱和到 255
//   第 2 拍 对比度：z = (y - 128) * contrast_gain / 128 + 128，饱和到 0..255
//   增益在顶层按帧边界锁存，同一帧内不会变化，画面不撕裂
module brightness_contrast_adjust (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [23:0] rgb_in,
    input  wire [7:0]  bright_gain,
    input  wire [7:0]  contrast_gain,
    output reg  [23:0] rgb_out
);
    function [7:0] bright;
        input [7:0] x;
        input [7:0] g;
        reg  [15:0] m;
        begin
            m = x * g;
            bright = (m[15:7] > 9'd255) ? 8'd255 : m[14:7];
        end
    endfunction

    function [7:0] contrast;
        input [7:0] x;
        input [7:0] g;
        reg signed [8:0]  c;
        reg signed [17:0] s;
        reg signed [10:0] z;
        begin
            c = $signed({1'b0, x}) - 9'sd128;
            s = c * $signed({1'b0, g});
            z = (s >>> 7) + 11'sd128;
            contrast = (z < 0) ? 8'd0 : (z > 11'sd255) ? 8'd255 : z[7:0];
        end
    endfunction

    reg [23:0] stage1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage1  <= 24'd0;
            rgb_out <= 24'd0;
        end else begin
            stage1  <= {bright(rgb_in[23:16], bright_gain),
                        bright(rgb_in[15:8],  bright_gain),
                        bright(rgb_in[7:0],   bright_gain)};
            rgb_out <= {contrast(stage1[23:16], contrast_gain),
                        contrast(stage1[15:8],  contrast_gain),
                        contrast(stage1[7:0],   contrast_gain)};
        end
    end
endmodule
