`timescale 1ns/1ps

// 简单双口 RAM（一写一读、独立时钟），写法按综合工具 BRAM 推断模板：
// 写口无复位，读口寄存输出（1 拍读延迟）。
module dp_ram_2clk #(
    parameter integer DW    = 16,
    parameter integer AW    = 14,
    parameter integer DEPTH = 9600
)(
    input  wire          wclk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          rclk,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    // 缩略图 RAM 较大：指定放进 BRAM32K（EG4S20 有 16 块，原工程一块没用），把 9K 块留给 FIFO
    (* ram_style = "bram_32k" *) reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge wclk) begin
        if (we)
            mem[waddr] <= wdata;
    end

    always @(posedge rclk) begin
        rdata <= mem[raddr];
    end
endmodule
