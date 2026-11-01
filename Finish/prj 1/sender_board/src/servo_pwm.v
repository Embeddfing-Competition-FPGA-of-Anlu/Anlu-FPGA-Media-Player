`timescale 1ns/1ps
// 舵机 PWM：周期 20ms（50Hz），高电平 500us + angle * 2000us / 180（0° 0.5ms，90° 1.5ms，180° 2.5ms）
//   新角度在下一个周期开始时生效，不会出现半截脉冲
module servo_pwm #(
    parameter integer CLK_HZ = 50000000
)(
    input  wire clk, rst_n,
    input  wire [7:0] angle,                 // 0~180
    output reg  pwm
);
    localparam integer US = CLK_HZ / 1000000;
    localparam integer PER = CLK_HZ / 50;
    wire [7:0] a = angle > 8'd180 ? 8'd180 : angle;
    wire [21:0] ad = a * 14'd11378;          // angle * 2000/180 * 1024
    wire [11:0] hus = 12'd500 + ad[21:10];
    reg [23:0] cnt, hi;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cnt <= 0; hi <= 1500 * US; pwm <= 0; end
        else begin
            if (cnt == PER - 1) begin cnt <= 0; hi <= hus * US; end else cnt <= cnt + 1'b1;
            pwm <= cnt < hi;
        end
endmodule
