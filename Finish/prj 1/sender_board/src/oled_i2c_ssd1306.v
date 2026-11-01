`timescale 1ns/1ps
// SSD1306 / SH1106 128x64 OLED，I2C 只写驱动，地址 0x3C（写地址 0x78）
//   SCL/SDA 开漏：oe=1 拉低，oe=0 释放（靠上拉）
//   页寻址模式，连续刷新：每页一次“设页地址”事务（3 字节命令）+ 一次“写数据”事务（128 字节），8 页一帧
//   像素按需取：驱动给出 pix_page / pix_col，渲染模块在几拍内给出 pix_data（一字节 = 一列 8 个像素，bit0 在上）
//   400kHz 时一帧约 24ms；每 64 帧重发一次初始化命令（不含关屏），OLED 热插拔或受干扰后能自己恢复
//   上电先等 100ms 再初始化（SSD1306 内部升压需要时间）
module oled_i2c_ssd1306 #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 400000,
    parameter integer COL_OFFSET = 0          // SSD1306 = 0；SH1106（1.3 寸常见）= 2
)(
    input  wire       clk,
    input  wire       rst_n,
    output reg  [2:0] pix_page,
    output reg  [6:0] pix_col,
    input  wire [7:0] pix_data,
    output reg        scl_oe,
    output reg        sda_oe
);
    localparam integer QDIV = ((CLK_HZ + I2C_HZ * 4 - 1) / (I2C_HZ * 4)) < 2 ? 2 : ((CLK_HZ + I2C_HZ * 4 - 1) / (I2C_HZ * 4));   // 向上取整，不超速
    localparam integer PWR_Q = (CLK_HZ / 10) / QDIV;          // 100ms，单位：四分之一位
    localparam integer N_INIT = 24;
    localparam [2:0] S_PWR = 0, S_START = 1, S_BIT = 2, S_STOP = 3, S_GAP = 4;
    localparam [1:0] T_INIT = 0, T_ADDR = 1, T_DATA = 2;
    localparam [7:0] COL_LO = 8'h00 | (COL_OFFSET & 15), COL_HI = 8'h10 | ((COL_OFFSET >> 4) & 15);

    reg [15:0] qcnt;
    wire qtick = qcnt == QDIV - 1;
    reg [2:0]  st;
    reg [1:0]  q;                 // 一位里的四分之一相位
    reg [3:0]  bitn;              // 0~7 数据位，8 = ACK
    reg [7:0]  sh;
    reg [7:0]  bi;                // 当前字节在事务里的序号：0 地址，1 控制字节，2.. 数据
    reg [1:0]  tr;
    reg [5:0]  frame;
    reg [23:0] pwr;

    function [7:0] init_byte; input [7:0] n;
        case (n)
            0: init_byte = 8'hD5;  1: init_byte = 8'h80;   // 时钟分频
            2: init_byte = 8'hA8;  3: init_byte = 8'h3F;   // 64 行
            4: init_byte = 8'hD3;  5: init_byte = 8'h00;   // 显示偏移 0
            6: init_byte = 8'h40;                          // 起始行 0
            7: init_byte = 8'h8D;  8: init_byte = 8'h14;   // 内部升压开
            9: init_byte = 8'h20; 10: init_byte = 8'h02;   // 页寻址模式
           11: init_byte = 8'hA1;                          // 列镜像（模块正放）
           12: init_byte = 8'hC8;                          // 行扫描反向（模块正放）
           13: init_byte = 8'hDA; 14: init_byte = 8'h12;   // COM 引脚配置
           15: init_byte = 8'h81; 16: init_byte = 8'hCF;   // 对比度
           17: init_byte = 8'hD9; 18: init_byte = 8'hF1;   // 预充电
           19: init_byte = 8'hDB; 20: init_byte = 8'h40;   // VCOMH
           21: init_byte = 8'hA4;                          // 显示 RAM 内容
           22: init_byte = 8'hA6;                          // 正常显示（不反色）
           default: init_byte = 8'hAF;                     // 开显示
        endcase
    endfunction

    // 事务里第 n 个字节
    function [7:0] tx_byte; input [1:0] t; input [7:0] n; input [2:0] pg; input [7:0] pd;
        if (n == 0) tx_byte = 8'h78;
        else if (n == 1) tx_byte = (t == T_DATA) ? 8'h40 : 8'h00;
        else case (t)
            T_INIT: tx_byte = init_byte(n - 8'd2);
            T_ADDR: tx_byte = (n == 2) ? (8'hB0 | {5'd0, pg}) : (n == 3) ? COL_LO : COL_HI;
            default: tx_byte = pd;
        endcase
    endfunction

    wire last_byte = (tr == T_INIT) ? (bi == N_INIT + 1) : (tr == T_ADDR) ? (bi == 8'd4) : (bi == 8'd129);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qcnt <= 0; st <= S_PWR; q <= 0; bitn <= 0; sh <= 0; bi <= 0; tr <= T_INIT; frame <= 0; pwr <= 0;
            pix_page <= 0; pix_col <= 0; scl_oe <= 0; sda_oe <= 0;
        end else begin
            qcnt <= qtick ? 16'd0 : qcnt + 1'b1;
            if (qtick) begin
                case (st)
                S_PWR: begin
                    scl_oe <= 0; sda_oe <= 0;
                    if (pwr == PWR_Q) begin st <= S_START; q <= 0; tr <= T_INIT; end else pwr <= pwr + 1'b1;
                end
                S_START: begin                                  // 空闲（双高）-> SDA 落 -> SCL 落
                    q <= q + 1'b1;
                    case (q)
                        2'd0: begin scl_oe <= 0; sda_oe <= 0; end
                        2'd1: sda_oe <= 1;
                        2'd2: scl_oe <= 1;
                        default: begin
                            bi <= 0; sh <= 8'h78; bitn <= 0; st <= S_BIT;
                        end
                    endcase
                end
                S_BIT: begin                                    // q0 SCL 低时改 SDA，q1 SCL 升，q2 保持，q3 SCL 落
                    q <= q + 1'b1;
                    case (q)
                        2'd0: sda_oe <= (bitn == 4'd8) ? 1'b0 : ~sh[7];   // ACK 位释放 SDA
                        2'd1: scl_oe <= 0;
                        2'd2: ;
                        default: begin
                            scl_oe <= 1;
                            if (bitn != 4'd8) begin sh <= {sh[6:0], 1'b0}; bitn <= bitn + 1'b1; end
                            else if (last_byte) st <= S_STOP;
                            else begin
                                // 装下一字节；同时把再下一字节的列号给渲染模块（它有几拍延迟，一字节约 22us 足够）
                                sh <= tx_byte(tr, bi + 8'd1, pix_page, pix_data);
                                if (tr == T_DATA) pix_col <= bi[6:0];        // 下一字节序号 bi+1，再下一字节列 = bi
                                bi <= bi + 1'b1; bitn <= 0;
                            end
                        end
                    endcase
                end
                S_STOP: begin                                   // SCL 低时 SDA 拉低 -> SCL 升 -> SDA 升
                    q <= q + 1'b1;
                    case (q)
                        2'd0: sda_oe <= 1;
                        2'd1: scl_oe <= 0;
                        2'd2: sda_oe <= 0;
                        default: st <= S_GAP;
                    endcase
                end
                default: begin                                  // S_GAP：总线空闲 4 个四分之一位，决定下一事务
                    q <= q + 1'b1;
                    if (q == 2'd3) begin
                        st <= S_START;
                        case (tr)
                            T_INIT: begin tr <= T_ADDR; pix_page <= 0; end
                            T_ADDR: begin tr <= T_DATA; pix_col <= 0; end
                            default: begin
                                if (pix_page == 3'd7) begin
                                    frame <= frame + 1'b1;
                                    tr <= (frame == 6'd63) ? T_INIT : T_ADDR;
                                end else tr <= T_ADDR;
                                pix_page <= pix_page + 1'b1;
                            end
                        endcase
                    end
                end
                endcase
            end
        end
    end
endmodule
