`timescale 1ns/1ps

// 第12课 图片缩放与缩略图（完赛整合版）：加载时同步生成缩略图，sd_card_clk 域
//   像素流来自 BMP 解析器（自下而上、每行 640 像素），不额外读 SD 卡、不占 SDRAM 带宽
//   640x480 -> 80x60：水平 8 像素盒式平均（抗混叠），垂直每 8 行取中间一行
//   存储：RGB565，按缩略图行号奇偶分到两块 RAM（显示端一拍同时取上下两行做双线性）
//   地址 = img*2400 + (ty>>1)*80 + tx，全部用移位相加，不用乘法器
module thumb_capture #(parameter DYNAMIC_MODE=0) (
    input wire mode_800,
    input             clk,
    input             rst_n,
    input             load_begin,      // 单拍：开始加载一张图（重试也会重发）
    input       [1:0] load_idx,
    input             top_down,        // 扩展3：像素流自上而下（负高度 BMP）
    input             pix_valid,
    input      [23:0] pix_rgb,         // {R,G,B}
    output reg        we_even,
    output reg        we_odd,
    output reg [13:0] waddr,
    output reg [15:0] wdata
);
    reg [9:0]  col;
    reg [9:0]  frow;                   // 文件行号：0 = 图像最底行
    reg [11:0] acc_r, acc_g, acc_b;
    reg [3:0] subcol,subrow;reg [6:0] thumb_x;reg [5:0] thumb_y;
    wire native_800=DYNAMIC_MODE && mode_800;
    wire [3:0] group_size=native_800?10:8;
    reg [1:0]  img;

    wire [11:0] sum_r = acc_r + pix_rgb[23:16];
    wire [11:0] sum_g = acc_g + pix_rgb[15:8];
    wire [11:0] sum_b = acc_b + pix_rgb[7:0];
    // 自下而上：显示行 = 479 - frow，取显示行 8k+3 即 frow[2:0]==4；自上而下：显示行 = frow
    wire        row_hit = top_down ? subrow==(group_size>>1)-1'b1 : subrow==(group_size>>1);
    wire [5:0]  ty = top_down ? thumb_y : 6'd59-thumb_y;
    wire [6:0] tx=thumb_x;
    wire [4:0]  tyh = ty[5:1];
    // img*2400 = img*(2048+256+64+32)；tyh*80 = tyh*(64+16)
    wire [13:0] img_base = ({12'd0, img} << 11) + ({12'd0, img} << 8) +
                           ({12'd0, img} << 6)  + ({12'd0, img} << 5);
    wire [7:0] avg_r=native_800?sum_r/10:sum_r>>3;
    wire [7:0] avg_g=native_800?sum_g/10:sum_g>>3;
    wire [7:0] avg_b=native_800?sum_b/10:sum_b>>3;
    wire [13:0] row_base = ({9'd0, tyh} << 6) + ({9'd0, tyh} << 4);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            subcol<=0;subrow<=0;thumb_x<=0;thumb_y<=0;
            col <= 10'd0;  frow <= 10'd0;  img <= 2'd0;
            acc_r <= 11'd0; acc_g <= 11'd0; acc_b <= 11'd0;
            we_even <= 1'b0; we_odd <= 1'b0; waddr <= 14'd0; wdata <= 16'd0;
        end else begin
            we_even <= 1'b0;
            we_odd  <= 1'b0;
            if (load_begin) begin
                subcol<=0;subrow<=0;thumb_x<=0;thumb_y<=0;
            col <= 10'd0;  frow <= 10'd0;  img <= load_idx;
                acc_r <= 11'd0; acc_g <= 11'd0; acc_b <= 11'd0;
            end else if (pix_valid && frow < (native_800?600:480)) begin
                if (subcol==group_size-1'b1) begin
                    acc_r <= 0; acc_g <= 0; acc_b <= 0;subcol<=0;thumb_x<=thumb_x+1'b1;
                    if (row_hit) begin
                        waddr <= img_base + row_base + {7'd0, tx};
                        wdata <= {avg_r[7:3],avg_g[7:2],avg_b[7:3]};  // 8 点平均后取 565
                        we_even <= !ty[0];
                        we_odd  <=  ty[0];
                    end
                end else begin
                    subcol<=subcol+1'b1;
                    acc_r <= sum_r; acc_g <= sum_g; acc_b <= sum_b;
                end
                if (col==(native_800?799:639)) begin
                    col  <= 10'd0;thumb_x<=0;subcol<=0;
                    if(subrow==group_size-1'b1)begin subrow<=0;thumb_y<=thumb_y+1'b1;end else subrow<=subrow+1'b1;
                    frow <= frow + 9'd1;
                end else
                    col <= col + 10'd1;
            end
        end
    end
endmodule
