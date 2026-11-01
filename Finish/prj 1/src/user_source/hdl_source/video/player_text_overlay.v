`timescale 1ns/1ps

// 第8课 OSD 字幕叠加 + 第13课 参数显示（完赛整合版，基于课程 promax_text_overlay 改写）
//   参数行：BRT100 CON100 VOL080 P03S A WAV NT HC F1 OK —— 亮度/对比度/音量/轮播周期/自动/音源/
//           节拍切图BT(SW3)/频谱揭幕SR(SW4)/3x3滤波档/显示健康（OK=读FIFO无断供，UF=近 2 秒内出现过断供）
//   字幕：UART 下发的整行（最多 64 字符）；从未下发时显示 设备名 + 日期时间
//           菜单打开时当前可调项反色高亮（深蓝底 + 青色字）
//   滚动字幕：屏幕底部半透明暗带 + 白字，每 2 帧左移 1 像素循环滚动
//   数值都用查表得到 ASCII，不做除法；5x7 点阵放大 2 倍，字符格 12x16
module player_text_overlay #(
    parameter integer PREFETCH_FONT=0,
    parameter integer PARAM_X = 8,
    parameter integer PARAM_Y = 24,
    parameter integer SUB_Y   = 450
)(
    input wire [9:0] pixel_x_pre,
    input wire [8:0] pixel_y_pre,
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    input  wire [9:0]  pixel_x,
    input  wire [8:0]  pixel_y,
    input  wire        param_en,
    input  wire        sub_en,
    input  wire        menu_active,
    input  wire [1:0]  menu_item,
    input  wire [2:0]  bright_lvl,
    input  wire [2:0]  contrast_lvl,
    input  wire [3:0]  vol_lvl,
    input  wire [1:0]  period_sel,
    input  wire        auto_on,
    input  wire        wav_mode,
    input  wire        night_on,
    input  wire        hc_on,
    input  wire [2:0]  filt,
    input  wire        uf_flag,
    input  wire [511:0] sub_vec,
    input  wire [6:0]  sub_len,
    input  wire [2:0]  tool,
    input  wire [47:0] dt,             // BCD {yy,mo,dd,hh,mi,ss}
    output wire [23:0] rgb_out
);
    localparam integer CELL_W = 12;
    localparam integer CELL_H = 16;

    reg       vs_q;
    reg [9:0] scroll;
    reg       sdiv;
    wire      frame_start = vs_q && !vsync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_q <= 1'b1;  scroll <= 10'd0;  sdiv <= 1'b0;
        end else begin
            vs_q <= vsync;
            if (frame_start) begin
                sdiv <= ~sdiv;
                if (sdiv) scroll <= scroll + 10'd1;
            end
        end
    end

    function [23:0] pct3;              // 亮度/对比度档位 -> 百分比
        input [2:0] l;
        case (l)
            3'd0: pct3 = "050";  3'd1: pct3 = "067";  3'd2: pct3 = "083";
            3'd3: pct3 = "100";  3'd4: pct3 = "117";  3'd5: pct3 = "133";
            default: pct3 = "150";
        endcase
    endfunction
    function [23:0] vol3;
        input [3:0] v;
        case (v)
            4'd0: vol3 = "000";  4'd1: vol3 = "010";  4'd2: vol3 = "020";
            4'd3: vol3 = "030";  4'd4: vol3 = "040";  4'd5: vol3 = "050";
            4'd6: vol3 = "060";  4'd7: vol3 = "070";  4'd8: vol3 = "080";
            4'd9: vol3 = "090";  default: vol3 = "100";
        endcase
    endfunction
    function [15:0] prd2;
        input [1:0] p;
        case (p)
            2'd0: prd2 = "02";  2'd1: prd2 = "03";  2'd2: prd2 = "05";  default: prd2 = "10";
        endcase
    endfunction

    wire [23:0] s_b = pct3(bright_lvl);
    wire [23:0] s_c = pct3(contrast_lvl);
    wire [23:0] s_v = vol3(vol_lvl);
    wire [15:0] s_p = prd2(period_sel);

    function [7:0] param_char;
        input [5:0] i;
        case (i)
            6'd0: param_char = "B";  6'd1: param_char = "R";  6'd2: param_char = "T";
            6'd3: param_char = s_b[23:16]; 6'd4: param_char = s_b[15:8]; 6'd5: param_char = s_b[7:0];
            6'd7: param_char = "C";  6'd8: param_char = "O";  6'd9: param_char = "N";
            6'd10: param_char = s_c[23:16]; 6'd11: param_char = s_c[15:8]; 6'd12: param_char = s_c[7:0];
            6'd14: param_char = "V"; 6'd15: param_char = "O"; 6'd16: param_char = "L";
            6'd17: param_char = s_v[23:16]; 6'd18: param_char = s_v[15:8]; 6'd19: param_char = s_v[7:0];
            6'd21: param_char = "P"; 6'd22: param_char = s_p[15:8]; 6'd23: param_char = s_p[7:0];
            6'd24: param_char = "S";
            6'd26: param_char = auto_on ? "A" : "M";
            6'd28: param_char = wav_mode ? "W" : "T";
            6'd29: param_char = wav_mode ? "A" : "O";
            6'd30: param_char = wav_mode ? "V" : "N";
            6'd32: param_char = night_on ? "N" : " ";             // NT = 夜间模式
            6'd33: param_char = night_on ? "T" : " ";
            6'd35: param_char = hc_on ? "S" : " ";
            6'd36: param_char = hc_on ? "R" : " ";
            6'd41: param_char = uf_flag ? "U" : "O";
            6'd42: param_char = uf_flag ? "F" : "K";
            default: param_char = " ";
        endcase
    endfunction

    // 参数字段编号：0 亮度 1 对比度 2 音量 3 周期；其它为 4（不高亮）
    function [2:0] field_of;
        input [5:0] i;
        field_of = (i <= 6'd5) ? 3'd0 : (i >= 6'd7  && i <= 6'd12) ? 3'd1 :
                   (i >= 6'd14 && i <= 6'd19) ? 3'd2 : (i >= 6'd21 && i <= 6'd24) ? 3'd3 : 3'd4;
    endfunction

    // 默认字幕（串口没下发过文字时）：设备名 + 日期时间，例如 "ETCH-01  2026-10-23 08:00:00"
    // 板上没有 RTC：时间从上电默认值开始走，上位机发 !Zyymmddhhmmss 校准
    reg [63:0] tname;
    always @* case(tool) 1:tname="ETCH-01 "; 2:tname="CVD-02  "; 3:tname="CMP-03  "; 4:tname="LITHO-04"; default:tname="FAB HMI "; endcase
    function [7:0] bcdc;
        input [3:0] v;
        bcdc=8'd48+{4'd0,v};
    endfunction
    function [7:0] sub_char;
        input [4:0] i;
        case (i)
            0,1,2,3,4,5,6,7: sub_char = tname[(7-i[2:0])*8 +: 8];
            5'd9:  sub_char = "2";             5'd10: sub_char = "0";
            5'd11: sub_char = bcdc(dt[47:44]); 5'd12: sub_char = bcdc(dt[43:40]); 5'd13: sub_char = "-";
            5'd14: sub_char = bcdc(dt[39:36]); 5'd15: sub_char = bcdc(dt[35:32]); 5'd16: sub_char = "-";
            5'd17: sub_char = bcdc(dt[31:28]); 5'd18: sub_char = bcdc(dt[27:24]);
            5'd20: sub_char = bcdc(dt[23:20]); 5'd21: sub_char = bcdc(dt[19:16]); 5'd22: sub_char = ":";
            5'd23: sub_char = bcdc(dt[15:12]); 5'd24: sub_char = bcdc(dt[11:8]);  5'd25: sub_char = ":";
            5'd26: sub_char = bcdc(dt[7:4]);   5'd27: sub_char = bcdc(dt[3:0]);
            default: sub_char = " ";
        endcase
    endfunction

    function [4:0] glyph_row;
        input [7:0] character;
        input [2:0] row;
        begin
            glyph_row = 5'b00000;
            case (character)
                "0": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10011;3:glyph_row=5'b10101;4:glyph_row=5'b11001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "1": case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b01100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "2": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00001;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "3": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b00001;2:glyph_row=5'b00001;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "4": case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00110;2:glyph_row=5'b01010;3:glyph_row=5'b10010;4:glyph_row=5'b11111;5:glyph_row=5'b00010;6:glyph_row=5'b00010;default:glyph_row=0;endcase
                "5": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "6": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "7": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b01000;default:glyph_row=0;endcase
                "8": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "9": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01111;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "A": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "B": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "C": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b01111;default:glyph_row=0;endcase
                "D": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "E": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "F": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "G": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10000;3:glyph_row=5'b10111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "H": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "I": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "J": case(row) 0:glyph_row=5'b00111;1:glyph_row=5'b00010;2:glyph_row=5'b00010;3:glyph_row=5'b00010;4:glyph_row=5'b10010;5:glyph_row=5'b10010;6:glyph_row=5'b01100;default:glyph_row=0;endcase
                "K": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10010;2:glyph_row=5'b10100;3:glyph_row=5'b11000;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "L": case(row) 0:glyph_row=5'b10000;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "M": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11011;2:glyph_row=5'b10101;3:glyph_row=5'b10101;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "N": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11001;2:glyph_row=5'b10101;3:glyph_row=5'b10011;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "O": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "P": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "Q": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10101;5:glyph_row=5'b10010;6:glyph_row=5'b01101;default:glyph_row=0;endcase
                "R": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "S": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "T": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "U": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "V": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b01010;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "W": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10101;4:glyph_row=5'b10101;5:glyph_row=5'b10101;6:glyph_row=5'b01010;default:glyph_row=0;endcase
                "X": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b01010;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "Y": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "Z": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                ":": case(row) 1:glyph_row=5'b00100;2:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                ".": case(row) 5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                ",": case(row) 5:glyph_row=5'b00100;6:glyph_row=5'b01000;default:glyph_row=0;endcase
                "!": case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                "?": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                "-": case(row) 3:glyph_row=5'b11111;default:glyph_row=0;endcase
                "_": case(row) 6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "/": case(row) 0:glyph_row=5'b00001;1:glyph_row=5'b00010;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                default: glyph_row = 5'b00000;
            endcase
        end
    endfunction

    // ---------------- 参数行 ----------------
    wire [9:0] plx    = pixel_x - PARAM_X;
    wire [8:0] ply    = pixel_y - PARAM_Y;
    wire       p_band = param_en && de && (pixel_x >= PARAM_X) && (plx < 10'd516) &&
                        (pixel_y >= PARAM_Y) && (ply < CELL_H);
    wire [5:0] p_cell = plx / CELL_W;
    wire [3:0] p_cx   = plx - p_cell * CELL_W;
    wire [4:0] p_row;
    wire [9:0] plx_pre=pixel_x_pre-PARAM_X;
    wire [8:0] ply_pre=pixel_y_pre-PARAM_Y;
    wire [5:0] p_cell_pre=plx_pre/CELL_W;
    wire       p_pix  = p_band && (p_cx[3:1] < 3'd5) && (ply[3:1] < 3'd7) && p_row[3'd4 - p_cx[3:1]];
    wire       p_sel  = p_band && menu_active && (field_of(p_cell) == {1'b0, menu_item});

    // ---------------- 滚动字幕 ----------------
    wire [9:0] sx     = pixel_x + scroll;                 // 10 位自然回绕：1024 像素一个循环
    wire [8:0] sly    = pixel_y - SUB_Y;
    wire       s_band = sub_en && de && (pixel_y >= SUB_Y - 2) && (pixel_y < SUB_Y + CELL_H + 2);
    // 字幕长度 -> 像素宽度（len*12 = len*8 + len*4）；sub_len=0 时用默认 32 字符
    wire [9:0] s_ext  = (sub_len == 7'd0) ? 10'd384 : ({3'd0, sub_len} << 3) + ({3'd0, sub_len} << 2);
    wire       s_line = sub_en && de && (pixel_y >= SUB_Y) && (sly < CELL_H) && (sx < s_ext);
    wire [5:0] s_cell = sx / CELL_W;
    wire [3:0] s_cx   = sx - s_cell * CELL_W;
    wire [5:0] s_idx  = s_cell - sub_len[5:0];           // 移位缓冲：第 k 个字符在 64-len+k（mod 64）
    wire [7:0] s_chr  = (sub_len == 7'd0) ? sub_char(s_cell[4:0]) : sub_vec[s_idx*8 +: 8];
    wire [4:0] s_row;
    wire [9:0] sx_pre=pixel_x_pre+scroll;
    wire [8:0] sly_pre=pixel_y_pre-SUB_Y;
    wire [5:0] s_cell_pre=sx_pre/CELL_W;
    wire [5:0] s_idx_pre=s_cell_pre-sub_len[5:0];
    wire [7:0] s_chr_pre=sub_len==0?sub_char(s_cell_pre[4:0]):sub_vec[s_idx_pre*8 +: 8];
    generate if(PREFETCH_FONT)begin:g_font
        (* ram_style="block", syn_ramstyle="block_ram" *) reg [4:0] font_rom[0:1023];
        reg [4:0] p_q,s_q;integer k;
        initial for(k=0;k<1024;k=k+1)font_rom[k]=glyph_row(k/8,k%8);
        wire [7:0] p_chr_pre=param_char(p_cell_pre);
        always @(posedge clk)begin
            p_q<=font_rom[{p_chr_pre[6:0],ply_pre[3:1]}];
            s_q<=font_rom[{s_chr_pre[6:0],sly_pre[3:1]}];
        end
        assign p_row=p_q;assign s_row=s_q;
    end else begin:g_comb_font
        assign p_row=glyph_row(param_char(p_cell),ply[3:1]);
        assign s_row=glyph_row(s_chr,sly[3:1]);
    end endgenerate
    wire       s_pix  = s_line && (s_cx[3:1] < 3'd5) && (sly[3:1] < 3'd7) && s_row[3'd4 - s_cx[3:1]];

    wire [23:0] dim = {1'b0, rgb_in[23:17], 1'b0, rgb_in[15:9], 1'b0, rgb_in[7:1]};

    assign rgb_out = p_pix  ? (p_sel ? 24'h40ffff : 24'hfff080) :
                     p_sel  ? 24'h102060 :
                     s_pix  ? 24'hffffff :
                     s_band ? dim : rgb_in;
endmodule
