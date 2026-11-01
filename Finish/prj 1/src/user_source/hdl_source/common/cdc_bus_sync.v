`timescale 1ns/1ps

// 完赛整合：多位“慢变控制总线”跨时钟域（toggle 握手）
//   源域：值变化且上一笔已被确认 -> 整组锁存到 cdc_hold，翻转 req
//   目的域：req 两级同步后检测到翻转 -> 整组采样 cdc_hold（此时已稳定多拍），回 ack
// 适合模式/亮度/焦点/完成位图这类按键级变化的信号，避免多位信号各自 2FF 造成的错拍组合。
module cdc_bus_sync #(
    parameter integer W    = 8,
    parameter integer FREE_RUN = 0,       // 1：不做整宽比较，握手一完成就重新采样（宽总线省 LUT，约 6 拍刷新一次）
    parameter [W-1:0] INIT = {W{1'b0}}
)(
    input  wire         src_clk,
    input  wire         src_rst_n,
    input  wire [W-1:0] src_data,
    input  wire         dst_clk,
    input  wire         dst_rst_n,
    output reg  [W-1:0] dst_data
);
    reg [W-1:0] cdc_hold;
    reg         req_tgl;
    reg         ack_tgl;
    reg         cdc_ack_s1, ack_s2;
    reg         cdc_req_s1, req_s2, req_s3;

    wire busy = (req_tgl != ack_s2);

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            cdc_hold   <= INIT;
            req_tgl    <= 1'b0;
            cdc_ack_s1 <= 1'b0;
            ack_s2     <= 1'b0;
        end else begin
            cdc_ack_s1 <= ack_tgl;
            ack_s2     <= cdc_ack_s1;
            if (!busy && (FREE_RUN != 0 || src_data != cdc_hold)) begin
                cdc_hold <= src_data;
                req_tgl  <= ~req_tgl;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            cdc_req_s1 <= 1'b0;
            req_s2     <= 1'b0;
            req_s3     <= 1'b0;
            ack_tgl    <= 1'b0;
            dst_data   <= INIT;
        end else begin
            cdc_req_s1 <= req_tgl;
            req_s2     <= cdc_req_s1;
            req_s3     <= req_s2;
            if (req_s2 != req_s3) begin
                dst_data <= cdc_hold;
                ack_tgl  <= req_s2;
            end
        end
    end
endmodule
