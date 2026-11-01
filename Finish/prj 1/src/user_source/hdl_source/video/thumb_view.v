`timescale 1ns/1ps

// 第12课 + 第14课：2x2 缩略图浏览界面，video_clk 域，固定 3 拍延迟
//   屏幕分 4 个 320x240 格，每格居中放 160x120 缩略图（存储 80x60，2 倍双线性放大）
//   双线性：2 倍放大时插值权重只有 0 / 0.5，垂直用奇偶两块 RAM 一拍取上下两行，
//           水平用上一像素的垂直结果做平均，整个插值只用加法和移位
//   焦点格画黄色边框；未加载完成灰色、加载失败暗红、没有这张图显示背景色
module thumb_view (
    input             clk,
    input             rst_n,
    input             de,
    input       [9:0] x,
    input       [8:0] y,
    input       [1:0] focus,
    input       [3:0] complete,
    input       [3:0] failed,
    input       [2:0] found,
    // 缩略图写口（来自 sd_card_clk 域 thumb_capture）
    input             wclk,
    input             we_even,
    input             we_odd,
    input      [13:0] waddr,
    input      [15:0] wdata,
    output reg [23:0] rgb
);
    localparam [23:0] C_BG     = 24'h101828;
    localparam [23:0] C_BORDER = 24'hffd020;
    localparam [23:0] C_LOAD   = 24'h505050;
    localparam [23:0] C_BAD    = 24'h601818;

    // ---------- 第 0 拍：坐标分解 ----------
    wire [1:0] tcell = {y >= 9'd240, x >= 10'd320};
    wire [8:0] cx   = x[8:0] - (x >= 10'd320 ? 9'd320 : 9'd0);
    wire [7:0] cy   = y[7:0] - (y >= 9'd240 ? 8'd240 : 8'd0);
    wire [8:0] lx   = cx - 9'd80;              // 缩略图在格内的坐标（越界时为大数）
    wire [7:0] ly   = cy - 8'd60;
    wire in_tile    = (cx >= 9'd80) && (cx < 9'd240) && (cy >= 8'd60) && (cy < 8'd180);
    wire in_frame   = (cx >= 9'd74) && (cx < 9'd246) && (cy >= 8'd54) && (cy < 8'd186);
    wire [5:0] r0   = ly[6:1];
    wire       mixv = ly[0] && (r0 != 6'd59);
    wire [5:0] r1   = mixv ? r0 + 6'd1 : r0;
    wire [6:0] cr   = (lx[7:1] + {6'd0, lx[0]} > 7'd79) ? 7'd79 : lx[7:1] + {6'd0, lx[0]};
    wire       swap = r0[0];                   // 1：上一行在奇数 RAM
    wire [4:0] ev_row = swap ? r1[5:1] : r0[5:1];
    wire [4:0] od_row = swap ? r0[5:1] : r1[5:1];
    wire [13:0] img_base = ({12'd0, tcell} << 11) + ({12'd0, tcell} << 8) +
                           ({12'd0, tcell} << 6)  + ({12'd0, tcell} << 5);
    wire [13:0] addr_e = img_base + ({9'd0, ev_row} << 6) + ({9'd0, ev_row} << 4) + {7'd0, cr};
    wire [13:0] addr_o = img_base + ({9'd0, od_row} << 6) + ({9'd0, od_row} << 4) + {7'd0, cr};

    wire [15:0] q_e, q_o;
    dp_ram_2clk #(.DW(16), .AW(14), .DEPTH(9600)) u_ram_even (
        .wclk(wclk), .we(we_even), .waddr(waddr), .wdata(wdata),
        .rclk(clk), .raddr(addr_e), .rdata(q_e));
    dp_ram_2clk #(.DW(16), .AW(14), .DEPTH(9600)) u_ram_odd (
        .wclk(wclk), .we(we_odd), .waddr(waddr), .wdata(wdata),
        .rclk(clk), .raddr(addr_o), .rdata(q_o));

    // 侧带信息随 RAM 读延迟打拍：RAM 地址第 0 拍给出，第 1 拍出数据
    reg [2:0] de_p, tile_p, frame_p, swap_p, mixv_p, mixh_p;
    reg [1:0] cell_p1, cell_p2, cell_p3;

    function [23:0] x565;
        input [15:0] p;
        x565 = {p[15:11], p[15:13], p[10:5], p[10:9], p[4:0], p[4:2]};
    endfunction
    function [23:0] avg;
        input [23:0] a, b;
        reg [8:0] r, g, bl;
        begin
            r  = a[23:16] + b[23:16];
            g  = a[15:8]  + b[15:8];
            bl = a[7:0]   + b[7:0];
            avg = {r[8:1], g[8:1], bl[8:1]};
        end
    endfunction

    // 第 2 拍：垂直插值
    wire [23:0] top = x565(swap_p[0] ? q_o : q_e);
    wire [23:0] bot = x565(swap_p[0] ? q_e : q_o);
    reg  [23:0] vcur, vprev;
    wire [23:0] hmix = mixh_p[1] ? avg(vprev, vcur) : vcur;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_p <= 3'd0; tile_p <= 3'd0; frame_p <= 3'd0;
            swap_p <= 3'd0; mixv_p <= 3'd0; mixh_p <= 3'd0;
            cell_p1 <= 2'd0; cell_p2 <= 2'd0; cell_p3 <= 2'd0;
            vcur <= 24'd0; vprev <= 24'd0; rgb <= 24'd0;
        end else begin
            de_p    <= {de_p[1:0], de};
            tile_p  <= {tile_p[1:0], in_tile};
            frame_p <= {frame_p[1:0], in_frame && (tcell == focus)};
            swap_p  <= {swap_p[1:0], swap};
            mixv_p  <= {mixv_p[1:0], mixv};
            mixh_p  <= {mixh_p[1:0], lx[0]};
            cell_p1 <= tcell;  cell_p2 <= cell_p1;  cell_p3 <= cell_p2;

            vcur  <= mixv_p[0] ? avg(top, bot) : top;
            vprev <= vcur;

            // 第 3 拍输出（总延迟 3 拍）
            if (!de_p[1])
                rgb <= 24'd0;
            else if (tile_p[1]) begin
                if ({1'b0, cell_p2} >= found)        rgb <= C_BG;
                else if (complete[cell_p2])          rgb <= hmix;
                else if (failed[cell_p2])            rgb <= C_BAD;
                else                                 rgb <= C_LOAD;
            end else if (frame_p[1])
                rgb <= C_BORDER;
            else
                rgb <= C_BG;
        end
    end
endmodule
