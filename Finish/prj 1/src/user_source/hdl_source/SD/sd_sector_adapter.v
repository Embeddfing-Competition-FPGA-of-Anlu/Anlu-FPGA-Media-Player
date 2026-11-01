`timescale 1ns/1ps

// 第6课：SD 卡扇区读接口适配层
// 把 FAT32 模块的 sector_* 接口（单拍 req + busy + byte_index + error）
// 转换成官方 sd_card_top 的 sd_sec_* 接口（电平 read，读到 end 后撤销）。
//   sector_busy       ：请求发出到读完成之间为 1（官方没有，这里生成）
//   sector_byte_index ：扇区内字节序号 0~511（官方不输出，这里计数）
//   sector_error      ：官方没有错误码；这里加了超时看门狗，卡死时报 0x10 并结束本次读
module sd_sector_adapter #(
    parameter integer TIMEOUT_CYCLES = 50_000_000   // 500ms @ 100MHz 内没有任何数据就判超时
) (
    input  wire        clk,
    input  wire        rst_n,
    // === FAT32 模块侧（sector 接口）===
    input  wire        sector_req,
    input  wire [31:0] sector_lba,
    output wire        sector_busy,
    output wire        sector_valid,
    output wire [7:0]  sector_byte,
    output wire [8:0]  sector_byte_index,
    output wire        sector_done,
    output wire [7:0]  sector_error,
    output wire        sector_timeout,       // 单拍：本次读超时（上层据此重新初始化 SD 卡）
    // === 官方 sd_card_top 侧（sd_sec 接口）===
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire        sd_sec_read_data_valid,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_end
);
    reg [8:0]  byte_cnt;
    reg        busy_reg;
    reg [31:0] idle_cnt;
    reg        timeout_done;
    reg        timeout_flag;

    wire timeout_hit = busy_reg && (idle_cnt >= TIMEOUT_CYCLES);

    assign sector_busy       = busy_reg;
    assign sector_valid      = busy_reg && sd_sec_read_data_valid;
    assign sector_byte       = sd_sec_read_data;
    assign sector_byte_index = byte_cnt;
    assign sector_done       = (busy_reg && sd_sec_read_end) || timeout_done;
    assign sector_error      = timeout_done ? 8'h10 : 8'h00;
    assign sector_timeout    = timeout_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            byte_cnt         <= 9'd0;
            busy_reg         <= 1'b0;
            idle_cnt         <= 32'd0;
            timeout_done     <= 1'b0;
            timeout_flag     <= 1'b0;
        end else begin
            timeout_done <= 1'b0;
            if (sector_req && !busy_reg) begin
                // 请求到来且空闲：锁存扇区号，拉高读请求
                sd_sec_read      <= 1'b1;
                sd_sec_read_addr <= sector_lba;
                busy_reg         <= 1'b1;
                byte_cnt         <= 9'd0;
                idle_cnt         <= 32'd0;
            end else if (busy_reg && sd_sec_read_end) begin
                // 读完成：撤销请求
                sd_sec_read <= 1'b0;
                busy_reg    <= 1'b0;
                byte_cnt    <= 9'd0;
            end else if (timeout_hit) begin
                // 超时：结束本次读并报错（SD 卡由上层复位重新初始化）
                sd_sec_read  <= 1'b0;
                busy_reg     <= 1'b0;
                byte_cnt     <= 9'd0;
                timeout_done <= 1'b1;
            end else if (busy_reg) begin
                if (sd_sec_read_data_valid) begin
                    byte_cnt <= byte_cnt + 9'd1;
                    idle_cnt <= 32'd0;
                end else begin
                    idle_cnt <= idle_cnt + 32'd1;
                end
            end
        end
    end
endmodule
