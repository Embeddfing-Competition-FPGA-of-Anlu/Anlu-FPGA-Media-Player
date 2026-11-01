`timescale 1ns/1ps
// 产线监控页的现场画面：接收发送板视频串口（3.125Mbps 8N1，每行 0xFF + 行号 0~119 + 160 字节 RGB332），
//   存进 19200 字节 BRAM（拆成 16384 + 2816 两块，避免按 2 的幂取整多占 BRAM），
//   在 HDMI 画面 (X0,Y0) 处 2 倍放大显示 320x240 彩色窗口。外框和标题画在 PIC4 背景图里，这里只填画面。
//   en=0（不在产线监控页 / 菜单 / 缩略图）或连续 ALIVE_FR 帧（约 0.5 秒）没收到完整的行时不显示，露出背景图里的“等待现场画面”。
//   坐标 x_in/y_in 直接用像素流水线的（与 de_in 对齐），本模块不再自己计数。
//   ban=1（警告横幅）时第 13~14 行（y 208~239）让给横幅。
//   接收在 sclk（100MHz，CPB=32）域，显示在 vclk（25MHz）域；输出比输入晚 2 拍，hs/vs/de/rgb 同步延迟。
module cam_pip #(
    parameter integer CPB = 32,
    parameter integer X0 = 304,
    parameter integer Y0 = 96,                       // 必须是偶数
    parameter integer ALIVE_FR = 31,                 // 最大 31
    parameter integer MARK_STATIONS = 0              // fixed camera: A/B/C left/center/right
)(
    input  wire sclk, srst_n, vid_rx,
    input  wire vclk, vrst_n, en, ban,
    input  wire review_alert,
    input  wire [2:0] review_tool,
    input  wire hs_in, vs_in, de_in, input wire [9:0] x_in, y_in, input wire [23:0] rgb_in,
    output reg  hs_out, vs_out, de_out, output reg [23:0] rgb_out
);
    // ---------------- sclk 域：串口接收 ----------------
    reg cdc_vrx_s1, rx_s2, rx, rx_q;
    reg busy, bv; reg [5:0] bc; reg [3:0] bi; reg [7:0] sh, bd;
    always @(posedge sclk or negedge srst_n)
        if (!srst_n) begin cdc_vrx_s1 <= 1; rx_s2 <= 1; rx <= 1; rx_q <= 1; busy <= 0; bv <= 0; bc <= 0; bi <= 0; sh <= 0; bd <= 0; end
        else begin
            cdc_vrx_s1 <= vid_rx; rx_s2 <= cdc_vrx_s1; rx <= rx_s2; rx_q <= rx;
            bv <= 0;
            if (!busy) begin if (rx_q && !rx) begin busy <= 1; bc <= 0; bi <= 0; end end
            else if (bc == ((bi == 0) ? CPB / 2 - 1 : CPB - 1)) begin
                bc <= 0;
                if (bi == 0) begin if (rx) busy <= 0; else bi <= 1; end            // 起始位中点仍为高：毛刺
                else if (bi == 9) begin busy <= 0; if (rx) begin bv <= 1; bd <= sh; end end
                else begin sh <= {rx, sh[7:1]}; bi <= bi + 1'b1; end
            end else bc <= bc + 1'b1;
        end

    // ---------------- sclk 域：行解析 -> BRAM（行头算一次 行号*160，之后逐字节 +1） ----------------
    reg [1:0] ps; reg [7:0] col; reg [14:0] wp, wa; reg we; reg [7:0] wd; reg ltg;
    always @(posedge sclk or negedge srst_n)
        if (!srst_n) begin ps <= 0; col <= 0; wp <= 0; wa <= 0; we <= 0; wd <= 0; ltg <= 0; end
        else begin
            we <= 0;
            if (bv) begin
                if (bd == 8'hff) ps <= 1;                                           // 行头（像素字节不会是 0xFF）
                else case (ps)
                    1: if (bd < 8'd120) begin wp <= {1'b0, bd[6:0], 7'd0} + {3'd0, bd[6:0], 5'd0}; col <= 0; ps <= 2; end
                       else ps <= 0;
                    2: begin
                        we <= 1; wa <= wp; wd <= bd; wp <= wp + 1'b1; col <= col + 1'b1;
                        if (col == 8'd159) begin ps <= 0; ltg <= ~ltg; end            // 收完一行
                    end
                    default: ;
                endcase
            end
        end
    reg [7:0] mem_a [0:16383];
    reg [7:0] mem_b [0:2815];
    always @(posedge sclk) if (we && !wa[14]) mem_a[wa[13:0]] <= wd;
    always @(posedge sclk) if (we && wa[14]) mem_b[wa[11:0]] <= wd;

    // ---------------- vclk 域：坐标、窗口、读 BRAM、叠加 ----------------
    // 有无数据：sclk 每收完一行翻转 ltg；vclk 数“连续多少帧没翻转”
    reg cdc_ltg_s1, ltg_s2, ltg_s3, de_q, vs_q; reg [4:0] fc; reg [14:0] rbase, ra;
    wire alive_v = fc < ALIVE_FR;
    wire in_y = y_in >= Y0 && y_in < Y0 + 240;
    wire in_win = de_in && in_y && x_in >= X0 && x_in < X0 + 320;
    always @(posedge vclk or negedge vrst_n)
        if (!vrst_n) begin cdc_ltg_s1 <= 0; ltg_s2 <= 0; ltg_s3 <= 0; de_q <= 0; vs_q <= 0; fc <= 5'd31; rbase <= 0; ra <= 0; end
        else begin
            cdc_ltg_s1 <= ltg; ltg_s2 <= cdc_ltg_s1; ltg_s3 <= ltg_s2;
            de_q <= de_in; vs_q <= vs_in;
            if (ltg_s3 != ltg_s2) fc <= 0; else if (vs_q && !vs_in && fc != 5'd31) fc <= fc + 1'b1;
            if (de_in && !de_q) begin                                                    // 行开始：y_in 已是本行
                if (y_in == Y0) rbase <= 0; else if (in_y && !y_in[0]) rbase <= rbase + 15'd160;   // 每 2 行换一行源数据
            end
            if (x_in == X0 - 1) ra <= rbase; else if (in_win && x_in[0]) ra <= ra + 1'b1;          // 每 2 像素换一个源像素
        end
    reg [7:0] rda, rdb; reg sel1;
    always @(posedge vclk) begin rda <= mem_a[ra[13:0]]; rdb <= mem_b[ra[11:0]]; end
    wire ban_row = ban && (y_in[9:4] == 6'd13 || y_in[9:4] == 6'd14);
    // Screen-space regions inside the unchanged full camera view. No digital pan/zoom.
    // Tool 1=A, 2=B, 3=C; legacy tool 4 requests whole-view review.
    reg [9:0] box_l, box_r;
    always @* case(review_tool)
        1:begin box_l=X0+4; box_r=X0+104; end
        2:begin box_l=X0+110; box_r=X0+210; end
        3:begin box_l=X0+216; box_r=X0+316; end
        default:begin box_l=X0+4; box_r=X0+316; end
    endcase
    wire mark_en=MARK_STATIONS && review_alert && review_tool>=1 && review_tool<=4;
    wire box_inside=x_in>=box_l && x_in<box_r && y_in>=Y0+8 && y_in<Y0+232;
    wire box_edge=box_inside && (x_in<box_l+2 || x_in>=box_r-2 || y_in<Y0+10 || y_in>=Y0+230);
    wire label_inside=x_in>=box_l+6 && x_in<box_l+62 && y_in>=Y0+14 && y_in<Y0+24;
    wire [9:0] lx=x_in-box_l-6, ly=y_in-Y0-14;
    reg [7:0] letter;
    always @* case(lx[5:3])
        0:letter="C"; 1:letter="H"; 2:letter="E"; 3:letter="C"; 4:letter="K"; 5:letter=" ";
        default:letter=review_tool==1 ? "A" : review_tool==2 ? "B" : review_tool==3 ? "C" : "4";
    endcase
    function [14:0] glyph;
        input [7:0] ch;
        case(ch)
            "C":glyph=15'b111_100_100_100_111;
            "H":glyph=15'b101_101_111_101_101;
            "E":glyph=15'b111_100_110_100_111;
            "K":glyph=15'b101_101_110_101_101;
            "A":glyph=15'b010_101_111_101_101;
            "B":glyph=15'b110_101_110_101_110;
            "4":glyph=15'b101_101_111_001_001;
            default:glyph=0;
        endcase
    endfunction
    wire [14:0] font=glyph(letter);
    wire [3:0] font_bit=4'd14-({1'b0,ly[3:1]}*4'd3+{2'd0,lx[2:1]});
    wire label_ink=lx[2:1]<3 && font[font_bit];
    wire mark_now=mark_en && in_win && alive_v && en && !ban_row;
    reg mark1;
    reg [23:0] mark_rgb1;
    reg hs1, vs1, de1, w1; reg [23:0] rgb1;
    wire [7:0] p = sel1 ? rdb : rda;
    always @(posedge vclk or negedge vrst_n)
        if (!vrst_n) begin hs1 <= 1; vs1 <= 1; de1 <= 0; w1 <= 0; sel1 <= 0; rgb1 <= 0;
                           hs_out <= 1; vs_out <= 1; de_out <= 0; rgb_out <= 0; mark1<=0; mark_rgb1<=0; end
        else begin
            hs1 <= hs_in; vs1 <= vs_in; de1 <= de_in; rgb1 <= rgb_in; sel1 <= ra[14];
            w1 <= in_win && alive_v && en && !ban_row;
            mark1<=mark_now && (box_edge || label_inside);
            mark_rgb1<=box_edge ? 24'hffb020 : label_ink ? 24'hffffe0 : 24'h302010;
            hs_out <= hs1; vs_out <= vs1; de_out <= de1;
            rgb_out <= mark1 ? mark_rgb1 : w1 ? {p[7:5], p[7:5], p[7:6], p[4:2], p[4:2], p[4:3], p[1:0], p[1:0], p[1:0], p[1:0]} : rgb1;
        end
endmodule
