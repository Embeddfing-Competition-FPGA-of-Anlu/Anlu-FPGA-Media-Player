`timescale 1ns/1ps

// 扩展3：任意分辨率 BMP 适配（sd_card_clk 100MHz 域）
//   输入：BMP 像素区字节流（经 2KB 字节 FIFO），源宽高 1..2048，24/32 位，每行按 4 字节补齐
//   输出：恒为 640x480 像素流（与源文件行序相同），按 target_width 指定的 4:3 视口等比缩放、居中、补黑边
//   算法：最近邻、像素中心对齐；每张图只做 3 次顺序除法（适配尺寸、水平步长、垂直步长），
//         逐像素只做整数 DDA 累加（int += q，frac += 2r，溢出进位），没有逐像素除法
//   放大：同一源行重复输出多行（行缓存复用，不重读 SD）；缩小：不需要的源行读入后直接丢弃
//   背压：写 FIFO（512 深）水位 >= 440 时暂停输出；字节不够时等待，SD 读取由上层按 FIFO 水位暂停
module bmp_scaler #(parameter DYNAMIC_MODE=0) (
    input wire mode_800,
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,            // 单拍：头已校验，参数有效
    input  wire        abort,
    input  wire [11:0] src_w,            // 1..2048
    input  wire [11:0] src_h,            // 1..2048（绝对值）
    input wire [9:0] target_width, // 4:3 viewport, 160..640, multiples of 16
    input  wire        bpp32,
    input  wire        fifo_empty,
    input  wire [7:0]  fifo_dout,        // pop 后下一拍有效
    output wire        fifo_pop,
    input  wire [8:0]  wr_used,
    output reg         pix_valid,
    output reg  [23:0] pix_rgb,          // {R,G,B}
    output wire        busy
);
    localparam [3:0] S_IDLE = 4'd0, S_SETUP = 4'd1, S_W1 = 4'd2, S_W2 = 4'd3, S_W3 = 4'd4,
                     S_INIT = 4'd5, S_ROW = 4'd6, S_LOAD = 4'd7, S_EMIT = 4'd8;
    reg [3:0]  st;
    assign busy = (st != S_IDLE);

    // ---------- 参数 ----------
    wire native_800=DYNAMIC_MODE && mode_800;
    wire [9:0] canvas_w=native_800?800:640,canvas_h=native_800?600:480;
    reg [9:0] viewport_w;reg [9:0] viewport_h;
    reg [11:0] W, H;
    reg        b32, wide;
    reg [13:0] stride;
    reg [9:0]  img_w, off_x;
    reg [9:0]  img_h, off_y;
    reg [11:0] qx, qy;
    reg [11:0] rx2, ry2;                 // 2*余数
    reg [10:0] dx2, dy2;                 // 2*目标尺寸
    // ---------- 除法器（共用） ----------
    reg         dv_start;
    reg  [21:0] dv_num;
    reg  [11:0] dv_den;
    wire        dv_done;
    wire [21:0] dv_q;
    wire [11:0] dv_r;
    seq_div #(.NW(22), .DW(12)) u_div (
        .clk(clk), .rst_n(rst_n), .start(dv_start), .num(dv_num), .den(dv_den),
        .done(dv_done), .q(dv_q), .r(dv_r));

    // ---------- 行缓存 2048x24 ----------
    reg [23:0] lb [0:2047];
    reg [23:0] lb_q;
    reg        lb_we;
    reg [10:0] lb_wa, lb_ra;
    reg [23:0] lb_wd;
    always @(posedge clk) if (lb_we) lb[lb_wa] <= lb_wd;
    always @(posedge clk) lb_q <= lb[lb_ra];

    // ---------- 行状态 ----------
    reg [9:0]  out_row;
    reg [11:0] next_src, lrow, sy, sx;
    reg [11:0] fy, fx;
    reg        have_row, row_black;
    reg [13:0] icnt;
    reg        room;
    reg        rchk, f_done, f_more, f_black, f_reuse;   // 换行判定：第一拍寄存比较结果，第二拍决策                      // icnt < stride，提前一拍算好，比较器不在读使能路径上
    reg [1:0]  bi;
    reg [11:0] col;
    reg [7:0]  pb, pg;
    reg        pop_d;
    reg [9:0]  px;
    reg        e_v, e_blk, e_v2, e_blk2;      // 读地址 -> BRAM 寄存输出 共 2 拍
    reg [8:0]  wr_used_q;

    assign fifo_pop = (st == S_LOAD) && room && !fifo_empty && !abort;

    reg [9:0] img_end_x;
    reg [9:0] img_end_y;
    reg        in_x;                      // px 在图像水平范围内（随 px 一起更新，比较器不在 fx/sx 使能路径上）
    wire       go        = (wr_used_q < 9'd440);
    // DDA 一步：int += q，frac += 2r，frac >= 2D 时进位
    wire [12:0] fx_n = fx + rx2;
    wire [12:0] fy_n = fy + ry2;
    wire fit_wide = ({2'b0, W} + {1'b0, W, 1'b0}) >= {H, 2'b00};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            img_end_x<=0;img_end_y<=0;
            viewport_w<=640;viewport_h<=480;
            st <= S_IDLE;  W <= 0; H <= 0; b32 <= 0; wide <= 0; stride <= 0;
            img_w <= 0; img_h <= 0; off_x <= 0; off_y <= 0; qx <= 0; qy <= 0;
            rx2 <= 0; ry2 <= 0; dx2 <= 0; dy2 <= 0;
            dv_start <= 0; dv_num <= 0; dv_den <= 12'd1;
            lb_we <= 0; lb_wa <= 0; lb_ra <= 0; lb_wd <= 0;
            out_row <= 0; next_src <= 0; lrow <= 0; sy <= 0; sx <= 0; fy <= 0; fx <= 0;
            have_row <= 0; row_black <= 0; icnt <= 0; room <= 0;
            rchk <= 0; f_done <= 0; f_more <= 0; f_black <= 0; f_reuse <= 0; bi <= 0; col <= 0;
            pb <= 0; pg <= 0; pop_d <= 0; px <= 0; in_x <= 0; e_v <= 0; e_blk <= 0; e_v2 <= 0; e_blk2 <= 0; wr_used_q <= 0;
            pix_valid <= 0; pix_rgb <= 0;
        end else begin
            dv_start  <= 1'b0;
            lb_we     <= 1'b0;
            pop_d     <= fifo_pop;
            wr_used_q <= wr_used;
            // 输出级：上一拍发出的读，本拍出像素
            e_v2      <= e_v;
            e_blk2    <= e_blk;
            pix_valid <= e_v2;
            pix_rgb   <= e_blk2 ? 24'd0 : lb_q;
            e_v       <= 1'b0;

            if (abort) begin
                st <= S_IDLE;
            end else case (st)
                S_IDLE: if (start) begin
                    viewport_w<=DYNAMIC_MODE ? canvas_w : ((target_width>=160 && target_width<=640) ? target_width : 640);
                    viewport_h<=DYNAMIC_MODE ? canvas_h : ((target_width>=160 && target_width<=640) ? (target_width>>2)*3 : 480);
                    W <= src_w;  H <= src_h;  b32 <= bpp32;  st <= S_SETUP;
                end
                S_SETUP: begin
                    // W*480 >= H*640  <=>  W*3 >= H*4：宽图按宽适配，否则按高适配
                    stride <= b32 ? {W, 2'b00} : (({2'b0, W} + {1'b0, W, 1'b0} + 14'd3) & 14'h3ffc);
                    wide   <= fit_wide;
                    // Parallel DSP arithmetic costs fewer MSlices than the
                    // shared operand mux and avoids a compare -> DSP path.
                    if (fit_wide) begin
                        dv_num <= H * viewport_w + {11'd0, W[11:1]};
                        dv_den <= W;
                    end else begin
                        dv_num <= W * viewport_h + {11'd0, H[11:1]};
                        dv_den <= H;
                    end
                    dv_start <= 1'b1;  st <= S_W1;
                end
                S_W1: if (dv_done) begin
                    if (wide) begin
                        img_w <= viewport_w;
                        img_h <= (dv_q == 0) ? 10'd1 : (dv_q > viewport_h) ? viewport_h : dv_q[9:0];
                        dv_num <= {10'd0, W};  dv_den <= {2'd0,viewport_w};
                    end else begin
                        img_h <= viewport_h;
                        img_w <= (dv_q == 0) ? 10'd1 : (dv_q > viewport_w) ? viewport_w : dv_q[9:0];
                        dv_num <= {10'd0, W};
                        dv_den <= (dv_q == 0) ? 12'd1 : (dv_q > viewport_w) ? {2'd0,viewport_w} : dv_q[11:0];
                    end
                    dv_start <= 1'b1;  st <= S_W2;
                end
                S_W2: if (dv_done) begin
                    qx <= dv_q[11:0];  rx2 <= {dv_r[10:0], 1'b0};  dx2 <= {img_w, 1'b0};
                    dv_num <= {10'd0, H};  dv_den <= {2'd0, img_h};
                    dv_start <= 1'b1;  st <= S_W3;
                end
                S_W3: if (dv_done) begin
                    qy <= dv_q[11:0];  ry2 <= {dv_r[10:0], 1'b0};  dy2 <= {img_h, 1'b0};
                    // 中心采样起点：q 偶 -> (q/2, r)；q 奇 -> ((q-1)/2, D+r)，分数单位 1/(2D)
                    sy <= {1'b0, dv_q[11:1]};
                    fy <= dv_q[0] ? {2'd0, img_h} + dv_r : dv_r;
                    st <= S_INIT;
                end
                S_INIT: begin
                    img_end_x <= ({1'b0,canvas_w} + {1'b0,img_w}) >> 1;
                    img_end_y <= ({1'b0,canvas_h} + {1'b0,img_h}) >> 1;
                    off_x <= (canvas_w - img_w) >> 1;
                    off_y <= (canvas_h - img_h) >> 1;
                    out_row <= 9'd0;  next_src <= 12'd0;  have_row <= 1'b0;
                    st <= S_ROW;
                end
                S_ROW: if (!rchk) begin
                    rchk    <= 1'b1;
                    f_done  <= out_row == canvas_h;
                    f_more  <= next_src < H;
                    f_black <= {1'b0, out_row} < {1'b0, off_y} || {1'b0, out_row} >= img_end_y;
                    f_reuse <= have_row && (lrow == sy || next_src >= H);
                end else begin
                    rchk <= 1'b0;
                    if (f_done) begin
                        // 输出完毕：把剩下的源行读完（丢弃），然后结束
                        if (f_more) begin
                            icnt <= 0; room <= stride != 0; bi <= 0; col <= 0;  st <= S_LOAD;
                        end else
                            st <= S_IDLE;
                    end else if (f_black) begin
                        row_black <= 1'b1;  px <= 10'd0;  in_x <= (off_x == 10'd0) && (img_end_x != 10'd0);  st <= S_EMIT;
                    end else if (f_reuse) begin   // 源行已读完时复用当前行（兜底）
                        row_black <= 1'b0;  px <= 10'd0;  in_x <= (off_x == 10'd0) && (img_end_x != 10'd0);
                        sx <= {1'b0, qx[11:1]};
                        fx <= qx[0] ? {2'd0, img_w} + {1'b0, rx2[11:1]} : {1'b0, rx2[11:1]};
                        st <= S_EMIT;
                    end else begin
                        icnt <= 0; room <= stride != 0; bi <= 0; col <= 0;  st <= S_LOAD;
                    end
                end
                S_LOAD: begin
                    if (fifo_pop) begin icnt <= icnt + 14'd1; room <= icnt + 14'd1 < stride; end
                    if (pop_d) begin
                        case (bi)
                            2'd0: begin pb <= fifo_dout;  bi <= 2'd1; end
                            2'd1: begin pg <= fifo_dout;  bi <= 2'd2; end
                            2'd2: begin
                                lb_we <= (col < W);  lb_wa <= col[10:0];  lb_wd <= {fifo_dout, pg, pb};
                                col <= col + 12'd1;
                                bi <= b32 ? 2'd3 : 2'd0;
                            end
                            default: bi <= 2'd0;                   // 32 位的 A/X 字节
                        endcase
                        // pop_d consumes the request issued one cycle earlier.
                        // On this edge icnt is exactly the received-byte count + 1.
                        if (icnt == stride) begin
                            lrow <= next_src;  next_src <= next_src + 12'd1;  have_row <= 1'b1;
                            st <= S_ROW;
                        end
                    end
                end
                default: begin                                     // S_EMIT：一行 640 像素
                    if (go) begin
                        e_v   <= 1'b1;
                        e_blk <= row_black || !in_x;
                        lb_ra <= sx[10:0];
                        if (!row_black && in_x) begin
                            if (fx_n >= {2'd0, dx2}) begin fx <= fx_n - {2'd0, dx2};  sx <= sx + qx + 12'd1; end
                            else begin                     fx <= fx_n[11:0];        sx <= sx + qx; end
                        end
                        px <= px + 10'd1;  in_x <= (px + 10'd1 >= off_x) && (px + 10'd1 < img_end_x);
                        if (px == canvas_w-1'b1) begin
                            out_row <= out_row + 9'd1;
                            if (!row_black) begin
                                if (fy_n >= {2'd0, dy2}) begin fy <= fy_n - {2'd0, dy2};  sy <= sy + qy + 12'd1; end
                                else begin                     fy <= fy_n[11:0];        sy <= sy + qy; end
                            end
                            st <= S_ROW;
                        end
                    end
                end
            endcase
        end
    end
endmodule

