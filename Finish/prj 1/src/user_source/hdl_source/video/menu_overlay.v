`timescale 1ns/1ps

// 按键菜单：半透明设置面板（video_clk 域，组合输出，最后一级叠加层）
//   位置 (160,64)，320x336 像素，20 列 x 21 行，每格 16x16（5x7 点阵放大 2 倍）
//   第 0 行标题；第 1~19 行为 19 个菜单项（光标 > + 高亮底）；第 20 行按键提示
//   每行：列 0 光标、列 2~10 名称（9 字符）、列 11 编辑标记 *、列 12~19 当前值（8 字符，右对齐）
//   背景 = 原画面亮度减半 + 深蓝色调（半透明），选中行更亮，编辑中的值为黄色
//   格宽 16 像素，行列号直接取坐标高位，不需要除法
module menu_overlay #(
    parameter integer HDMI_MODE_MENU=0,
    parameter integer PREFETCH_FONT=HDMI_MODE_MENU,
    parameter integer X0 = 160,
    parameter integer Y0 = 64
)(
    input wire clk,
    input wire [9:0] pixel_x_pre,
    input wire [8:0] pixel_y_pre,
    input  wire        de,
    input  wire [9:0]  pixel_x,
    input  wire [8:0]  pixel_y,
    input  wire        enable,
    input  wire [4:0]  item,
    input  wire        edit,
    input  wire [2:0]  bright_lvl, contrast_lvl,
    input  wire [3:0]  vol_lvl,
    input  wire [1:0]  period_sel,
    input  wire        auto_on,
    input  wire [1:0]  viz,
    input  wire        filt,              // 0 白天 1 夜间
    input  wire [2:0]  fx,
    input  wire [1:0]  overlay_mode,
    input  wire [1:0]  pic_idx,
    input  wire [2:0]  found,
    input  wire [9:0]  lim_idx,          // 阈值档位 {vib,tmax,imax,vmax,vmin}
    input  wire        snd_tone,          // 1：强制旋律音
    input  wire        wav_ok,            // 卡上有可用的 MUSIC.WAV
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    localparam [159:0] TITLE  = "   ==== MENU ====   ";
    localparam [159:0] HINT_S = " K1/K3 MOVE  K2 SET ";
    localparam [159:0] HINT_E = " K1 +  K3 -  K2 OK  ";

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
                "K": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10010;2:glyph_row=5'b10100;3:glyph_row=5'b11000;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "L": case(row) 0:glyph_row=5'b10000;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "M": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11011;2:glyph_row=5'b10101;3:glyph_row=5'b10101;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "N": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11001;2:glyph_row=5'b10101;3:glyph_row=5'b10011;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "O": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "P": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "R": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "S": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "T": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "U": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "V": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b01010;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "W": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10101;4:glyph_row=5'b10101;5:glyph_row=5'b10101;6:glyph_row=5'b01010;default:glyph_row=0;endcase
                "Y": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "X": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b01010;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "%": case(row) 0:glyph_row=5'b11001;1:glyph_row=5'b11010;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01011;6:glyph_row=5'b10011;default:glyph_row=0;endcase
                ">": case(row) 0:glyph_row=5'b01000;1:glyph_row=5'b00100;2:glyph_row=5'b00010;3:glyph_row=5'b00001;4:glyph_row=5'b00010;5:glyph_row=5'b00100;6:glyph_row=5'b01000;default:glyph_row=0;endcase
                "+": case(row) 1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b11111;4:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                "*": case(row) 1:glyph_row=5'b10101;2:glyph_row=5'b01110;3:glyph_row=5'b11111;4:glyph_row=5'b01110;5:glyph_row=5'b10101;default:glyph_row=0;endcase
                "=": case(row) 2:glyph_row=5'b11111;4:glyph_row=5'b11111;default:glyph_row=0;endcase
                "-": case(row) 3:glyph_row=5'b11111;default:glyph_row=0;endcase
                "/": case(row) 0:glyph_row=5'b00001;1:glyph_row=5'b00010;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                default: glyph_row = 5'b00000;
            endcase
        end
    endfunction

    // 菜单项名称（9 字符）
    function [71:0] label;
        input [4:0] i;
        case (i)
            5'd0:  label = "BRIGHT   ";  5'd1:  label = "CONTRAST ";  5'd2:  label = "VOLUME   ";
            5'd3:  label = "PERIOD   ";  5'd4:  label = "AUTO PLAY";  5'd5:  label = "VISUAL   ";
            5'd6:  label = "FILTER   ";  5'd7:  label = "EFFECT   ";  5'd8:  label = "OVERLAY  ";
            5'd9:  label = "PICTURE  ";  5'd10: label = "V MIN    ";  5'd11: label = "V MAX    ";
            5'd12: label = "I MAX    ";  5'd13: label = "T MAX    ";  5'd14: label = "VIB LIM  ";
            5'd15: label = "THUMBNAIL";  5'd16: label = "SAVE     ";  5'd17: label = "SOUND    ";
            default: label = "EXIT     ";
        endcase
    endfunction

    function [23:0] pct3;
        input [2:0] l;
        case (l)
            3'd0: pct3 = "050";  3'd1: pct3 = "067";  3'd2: pct3 = "083";  3'd3: pct3 = "100";
            3'd4: pct3 = "117";  3'd5: pct3 = "133";  default: pct3 = "150";
        endcase
    endfunction
    function [23:0] vol3;
        input [3:0] v;
        case (v)
            4'd0: vol3 = "  0";  4'd1: vol3 = " 10";  4'd2: vol3 = " 20";  4'd3: vol3 = " 30";
            4'd4: vol3 = " 40";  4'd5: vol3 = " 50";  4'd6: vol3 = " 60";  4'd7: vol3 = " 70";
            4'd8: vol3 = " 80";  4'd9: vol3 = " 90";  default: vol3 = "100";
        endcase
    endfunction

    // 当前值（8 字符，右对齐；字符串字面量位数不足时高位补 0，0 字符显示为空白）
    // 阈值档位显示（与 board_monitor / media_pixel_pipeline 的档位表一致）
    function [63:0] vmin_t; input [1:0] k; vmin_t = k==0 ? "3000 MV" : k==1 ? "3100 MV" : k==2 ? "3200 MV" : "3250 MV"; endfunction
    function [63:0] vmax_t; input [1:0] k; vmax_t = k==0 ? "3350 MV" : k==1 ? "3400 MV" : k==2 ? "3450 MV" : "3500 MV"; endfunction
    function [63:0] imax_t; input [1:0] k; imax_t = k==0 ? " 50 MA" : k==1 ? " 60 MA" : k==2 ? " 80 MA" : "100 MA"; endfunction
    function [63:0] tmax_t; input [1:0] k; tmax_t = k==0 ? "45 C" : k==1 ? "50 C" : k==2 ? "55 C" : "60 C"; endfunction
    function [63:0] vib_t;  input [1:0] k; vib_t  = k==0 ? "30" : k==1 ? "40" : k==2 ? "50" : "60"; endfunction
    function [63:0] value;
        input [4:0] i;
        reg [7:0] pd, fd;
        begin
            pd = "1" + {6'd0, pic_idx};
            fd = "0" + {5'd0, found};
            case (i)
                5'd0:  value = {pct3(bright_lvl), "%"};
                5'd1:  value = {pct3(contrast_lvl), "%"};
                5'd2:  value = {vol3(vol_lvl), "%"};
                5'd3:  value = (period_sel == 2'd0) ? " 2S" : (period_sel == 2'd1) ? " 3S" :
                               (period_sel == 2'd2) ? " 5S" : "10S";
                5'd4:  value = auto_on ? "ON" : "OFF";
                5'd5:  value = (viz == 2'd0) ? "OFF" : (viz == 2'd1) ? "SPECTRUM" : (viz == 2'd2) ? "WAVE" : "BOTH";
                5'd6:  value = filt ? "NIGHT" : "DAY";
                5'd7:  case (fx)
                           3'd1: value = "GRAY";    3'd2: value = "SEPIA";   3'd3: value = "NEGATIVE";
                           3'd4: value = "POSTER";  3'd5: value = "BINARY";  3'd6: value = "WARM";
                           3'd7: value = "COOL";    default: value = "OFF";
                       endcase
                5'd8:  value = (overlay_mode == 2'd0) ? "FULL" : (overlay_mode == 2'd1) ? "INFO" : "CLEAN";
                5'd9:  value = {"PIC ", pd, "/", fd};
                5'd10: value = vmin_t(lim_idx[1:0]);
                5'd11: value = vmax_t(lim_idx[3:2]);
                5'd12: value = imax_t(lim_idx[5:4]);
                5'd13: value = tmax_t(lim_idx[7:6]);
                5'd14: value = vib_t(lim_idx[9:8]);
                5'd15: value = "OPEN";
                5'd16: value = "FLASH";
                5'd17: value = snd_tone ? "TONE 48K" : wav_ok ? "MUSIC" : "NO WAV";
                default: value = 64'd0;
            endcase
        end
    endfunction

    // ---------- 坐标 -> 行/列/点阵位置 ----------
    wire [9:0] rx = pixel_x - X0;
    wire [9:0] rx_char=(PREFETCH_FONT ? pixel_x_pre : pixel_x)-X0;
    wire [8:0] ry_char=(PREFETCH_FONT ? pixel_y_pre : pixel_y)-Y0;
    wire [8:0] ry = pixel_y - Y0;
    wire in_panel = enable && de && (pixel_x >= X0) && (rx < 10'd320) && (pixel_y >= Y0) && (ry < 9'd336);
    wire [4:0] row = ry_char[8:4];
    wire [4:0] row_now=ry[8:4],col_now=rx[8:4];
    wire sel_now=(row_now!=0 && row_now!=20 && row_now-1'b1==item);
    wire [4:0] col = rx_char[8:4];
    wire [2:0] gc  = rx[3:1];
    wire [2:0] gr  = ry_char[3:1];
    wire [4:0] ri  = row - 5'd1;                            // 菜单项号（第 1~19 行）
    wire       is_item = (row != 5'd0) && (row != 5'd20);
    wire       sel     = is_item && (ri == item);

    wire [71:0]  lbl = label(ri);
    wire [63:0]  val = value(ri);
    wire [159:0] hint = edit ? HINT_E : HINT_S;
    wire [4:0]   tc = 5'd19 - col;                          // 标题/提示行的字符位置（高位是第一个字符）
    wire [3:0]   lc = 4'd10 - col[3:0];                     // 名称字符位置（列 2..10）
    wire [2:0]   vc = 3'd7 - (col[2:0] - 3'd4);             // 值字符位置（列 12..19）

    reg [7:0] chr;
    always @* begin
        chr = " ";
        if (row == 5'd0)                         chr = TITLE[tc*8 +: 8];
        else if (row == 5'd20)                   chr = hint[tc*8 +: 8];
        else if (col == 5'd0)                    chr = sel ? ">" : " ";
        else if (col == 5'd11)                   chr = (sel && edit) ? "*" : " ";
        else if (col >= 5'd2 && col <= 5'd10)    chr = lbl[lc*8 +: 8];
        else if (col >= 5'd12)                   chr = val[vc*8 +: 8];
    end

    wire [4:0] bits;
    generate if(PREFETCH_FONT)begin:g_font
        // Prefetch from the preceding coordinate stage; RGB latency is unchanged.
        (* ram_style="block", syn_ramstyle="block_ram" *) reg [4:0] font_rom[0:1023];
        reg [4:0] font_q;integer k;
        initial for(k=0;k<1024;k=k+1)font_rom[k]=glyph_row(k/8,k%8);
        always @(posedge clk)font_q<=font_rom[{chr[6:0],gr}];
        assign bits=font_q;
    end else begin:g_comb_font
        assign bits=glyph_row(chr,gr);
    end endgenerate
    wire       txt  = in_panel && (gc < 3'd5) && (ry[3:1] < 3'd7) && bits[3'd4 - gc];
    wire       bord = in_panel && (rx == 10'd0 || rx == 10'd319 || ry == 9'd0 || ry == 9'd335);

    // 半透明底：原画面亮度减半 + 色调
    wire [23:0] half = {1'b0, rgb_in[23:17], 1'b0, rgb_in[15:9], 1'b0, rgb_in[7:1]};
    wire [23:0] bg   = sel_now ? half + 24'h103058 :
                       (row_now == 5'd0 || row_now == 5'd20) ? half + 24'h001830 : half + 24'h000828;
    wire [23:0] fg   = (row_now == 5'd0)  ? 24'hffe040 :
                       (row_now == 5'd20) ? 24'hb0b0b0 :
                       (col_now == 5'd0 || col_now == 5'd11) ? 24'hffe040 :
                       (col_now >= 5'd12) ? ((sel_now && edit) ? 24'hffe040 : 24'h60ffff) : 24'hffffff;

    assign rgb_out = !in_panel ? rgb_in :
                     bord ? 24'h40c0ff :
                     txt  ? fg : bg;
endmodule

