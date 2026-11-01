`timescale 1ns/1ps

// 单时钟字节 FIFO（BRAM），读出寄存 1 拍：pop 当拍发出，下一拍 dout 有效
//   count 为已存字节数，供上游做背压（例如超过一半就暂停读下一个扇区）
module byte_fifo #(
    parameter integer AW = 11                     // 2^AW 字节
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          clr,                     // 同步清空
    input  wire          push,
    input  wire [7:0]    din,
    input  wire          pop,                     // 调用方保证 count != 0
    output reg  [7:0]    dout,
    output reg  [AW:0]   count,
    output wire          empty
);
    reg [7:0]    mem [0:(1<<AW)-1];
    reg [AW-1:0] wp, rp;
    wire do_push = push && (count != (1 << AW));
    wire do_pop  = pop && (count != 0);
    assign empty = (count == 0);

    always @(posedge clk) if (do_push) mem[wp] <= din;
    always @(posedge clk) if (do_pop)  dout <= mem[rp];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= 0;  rp <= 0;  count <= 0;
        end else if (clr) begin
            wp <= 0;  rp <= 0;  count <= 0;
        end else begin
            if (do_push) wp <= wp + 1'b1;
            if (do_pop)  rp <= rp + 1'b1;
            count <= count + do_push - do_pop;
        end
    end
endmodule
