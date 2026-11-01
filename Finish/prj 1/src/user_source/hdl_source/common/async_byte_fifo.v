`timescale 1ns/1ps

// 完赛整合（第10课 WAV）：字节宽异步 FIFO，格雷码指针跨域，存储体推断为 BRAM
//   写侧：sd_card_clk 100MHz（WAV 解析器输出的 PCM 字节）
//   读侧：video_clk 25MHz（48kHz 采样拼装）
// 写侧不做满保护：生产者用 wr_used 判断剩余空间（流读取器每次至少留 512 字节余量才读下一扇区）。
module async_byte_fifo #(
    parameter integer AW = 12                  // 2^12 = 4096 字节，约 21ms 立体声 / 42ms 单声道
)(
    input  wire          wr_clk,
    input  wire          wr_rst_n,
    input  wire          wr_en,
    input  wire [7:0]    wr_data,
    output reg  [AW:0]   wr_used,

    input  wire          rd_clk,
    input  wire          rd_rst_n,
    input  wire          rd_en,
    output reg  [7:0]    rd_data,
    output wire          rd_empty
);
    reg [7:0] mem [0:(1<<AW)-1];

    reg [AW:0] wbin, wgray;
    reg [AW:0] rbin, rgray;
    reg [AW:0] cdc_rgray_s1, rgray_w, rbin_w;  // 读指针 -> 写域
    reg [AW:0] cdc_wgray_s1, wgray_r;          // 写指针 -> 读域

    function [AW:0] g2b;
        input [AW:0] g;
        integer i;
        begin
            g2b[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1)
                g2b[i] = g2b[i + 1] ^ g[i];
        end
    endfunction

    wire [AW:0] wbin_nx = wbin + 1'b1;
    wire [AW:0] rbin_nx = rbin + 1'b1;

    // ---------------- 写域 ----------------
    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wbin[AW-1:0]] <= wr_data;
    end

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wbin         <= {(AW+1){1'b0}};
            wgray        <= {(AW+1){1'b0}};
            cdc_rgray_s1 <= {(AW+1){1'b0}};
            rgray_w      <= {(AW+1){1'b0}};
            rbin_w       <= {(AW+1){1'b0}};
            wr_used      <= {(AW+1){1'b0}};
        end else begin
            cdc_rgray_s1 <= rgray;
            rgray_w      <= cdc_rgray_s1;
            rbin_w       <= g2b(rgray_w);        // 多打一拍，格雷转二进制不进 100MHz 关键路径
            wr_used      <= wbin - rbin_w;       // 保守估计（读指针滞后几拍只会让 used 偏大）
            if (wr_en) begin
                wbin  <= wbin_nx;
                wgray <= wbin_nx ^ (wbin_nx >> 1);
            end
        end
    end

    // ---------------- 读域 ----------------
    assign rd_empty = (rgray == wgray_r);
    wire   rd_fire  = rd_en && !rd_empty;

    always @(posedge rd_clk) begin
        if (rd_fire)
            rd_data <= mem[rbin[AW-1:0]];
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rbin         <= {(AW+1){1'b0}};
            rgray        <= {(AW+1){1'b0}};
            cdc_wgray_s1 <= {(AW+1){1'b0}};
            wgray_r      <= {(AW+1){1'b0}};
        end else begin
            cdc_wgray_s1 <= wgray;
            wgray_r      <= cdc_wgray_s1;
            if (rd_fire) begin
                rbin  <= rbin_nx;
                rgray <= rbin_nx ^ (rbin_nx >> 1);
            end
        end
    end
endmodule
