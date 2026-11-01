`timescale 1ns/1ps
// OV5640 160x120 彩色采集 + 现场视觉处理 + 视频串口发送 + 上板诊断计数
//   采集（pclk 域）：DVP 上升沿采样，HREF 高 = 行有效。VSYNC 任一跳变都把行计数清零，跳变后的第一行算新帧，
//     VSYNC 是高脉冲、低脉冲还是整帧电平都能用。RGB565 每 2 字节一个像素（先低字节），
//     转成 RGB332（R3 G3 B2）一字节；纯白 0xFF 改成 0xFE，数据里不会出现行头 0xFF
//   现场视觉（vmode）：1 边缘 Sobel  2 3x3 卷积引擎（ksel 选核）  3 帧差运动 + A/B/C 工位分区；
//     卷积核 ksel：0 锐化  1 高斯降噪（这两种把灰度增量加回彩色）  2 拉普拉斯轮廓  3 浮雕
//                  4 两层小 CNN：Sobel 卷积 -> 偏置 32 + ReLU -> 3x3 最大池化（步长 1），输出特征图
//     hitp = 上一帧“命中”像素占比 %（边缘点 / 轮廓点 / 特征点 / 运动点；锐化、降噪、浮雕为 0）
//   工位分区：列 0~52 = A，53~105 = B，106~159 = C（与看板现场画面的 CHECK 框位置一致）；
//     zp0~zp2 = 上一帧各区运动点占比 %，不管当前显示哪种算法都在统计，每采完一帧给一个 ztick
//   缓存：19200 字节 BRAM，每字节 1 个像素。发送端请求一帧 -> 下一次 VSYNC 跳变开始采集 -> 120 行后通知发送端
//   发送（clk 域）：每行 0xFF + 行号(0~119) + 160 字节，8N1，波特率 clk/VID_CPB（50MHz/16 = 3.125Mbps），
//     一帧 19440 字节约 62ms，采一帧发一帧约 7~8 帧/秒
//   诊断（clk 域）：VSYNC 上升沿/秒、两次 VSYNC 上升沿之间的 HREF 行数、每行字节数、PCLK 频率、D0~D7 跳变位
module cam_video #(
    parameter integer CLK_HZ = 50000000,
    parameter integer VID_CPB = 16
)(
    input  wire clk, rst_n, run, s_tick,
    input  wire [2:0] vmode,                 // 0 原图 1 边缘 2 卷积引擎 3 帧差运动（分区）
    input  wire [2:0] ksel,                  // 卷积核 0 锐化 1 降噪 2 轮廓 3 浮雕 4 CNN
    input  wire [2:0] zact,                  // 正在报警的工位：运动模式里该区运动点标红，其余工位标黄
    input  wire pclk, vsync, href, input wire [7:0] d,
    output wire vid_tx,
    output reg  [7:0] fps,                   // 采集逻辑认到的帧/秒
    output reg  [7:0] vps,                   // VSYNC 上升沿/秒
    output reg  [11:0] vlines,               // 每帧 HREF 行数（应为 120）
    output reg  [11:0] hlen,                 // 每行 HREF 高电平的 PCLK 数（应为 320）
    output reg  [11:0] pmhz,                 // PCLK 频率，单位 0.1MHz
    output reg  [7:0] dmask,                 // 1 秒内有跳变的数据位（应为 FF）
    output reg  [7:0] sps,                   // 每秒从视频串口发完的帧数
    output reg  [6:0] hitp,                  // 上一帧命中像素占比 0~100
    output reg  [6:0] zp0, zp1, zp2,         // 上一帧 A/B/C 工位区运动点占比 0~100
    output reg        ztick                  // zp 更新（每采完一帧一个脉冲）
);
    reg req_t;

    // ---------------- pclk 域：采集 + 现场视觉处理 ----------------
    reg [7:0] cd = 0; reg cv = 0, cv_q = 0, ch = 0, ch_q = 0;
    reg [2:0] vm_s1 = 0, vm = 0, za_s1 = 0, za = 0, ks_s1 = 0, ks = 0;    // 来自 clk 域的准静态设置
    always @(posedge pclk) begin
        cd <= d; cv <= vsync; ch <= href; cv_q <= cv; ch_q <= ch;
        vm_s1 <= vmode; vm <= vm_s1; za_s1 <= zact; za <= za_s1; ks_s1 <= ksel; ks <= ks_s1;
    end
    wire ve = cv != cv_q;                                       // VSYNC 跳变（帧头或帧尾）
    wire hs = ch && !ch_q, he = !ch && ch_q;                    // 行开始 / 行结束
    reg [11:0] lc = 0, bc = 0, hl = 0; reg [14:0] base = 0; reg vpend = 0, ftg = 0; reg [9:0] pc = 0;
    reg [2:0] rq_s = 0; reg arm = 0, cap = 0, dtg = 0;
    reg [7:0] hold = 0; reg we = 0; reg [14:0] wa = 0; reg [7:0] wd = 0;
    reg [14:0] hc = 0, hcl = 0;                                 // 本帧 / 上一帧命中像素数
    reg [12:0] zc0 = 0, zc1 = 0, zc2 = 0, zl0 = 0, zl1 = 0, zl2 = 0; reg ztg = 0;   // 各工位区运动点（本帧 / 上一帧）
    wire psel = ch && cap && lc < 12'd120 && bc < 12'd320;
    wire pv = psel && bc[0];                                    // 像素第 2 个字节：整个像素到齐
    wire [7:0] xp = bc[8:1];                                    // 像素列 0~159
    wire [14:0] pa = base + {7'd0, xp};
    // RGB565 先低字节（上板确认）；扩展成 8 位 R/G/B，灰度 = (R + 2G + B) / 4
    wire [15:0] px = {cd, hold};
    wire [7:0] r8 = {px[15:11], px[15:13]}, g8 = {px[10:5], px[10:9]}, b8 = {px[4:0], px[4:2]};
    wire [9:0] gsum = {2'b0, r8} + {1'b0, g8, 1'b0} + {2'b0, b8};
    wire [7:0] gy8 = gsum[9:2];
    wire [3:0] gn = gy8[7:4];
    // RGB332 量化前加 4x4 Bayer 有序抖动：R/G 每级 32、B 每级 64，直接截断会出现明显色带；
    //   抖动后色带变成细网纹，看板 2 倍放大后观感接近更多色阶（不增加带宽）
    reg [3:0] bay;
    always @* case ({lc[1:0], xp[1:0]})
        4'd0: bay = 4'd0;   4'd1: bay = 4'd8;   4'd2: bay = 4'd2;   4'd3: bay = 4'd10;
        4'd4: bay = 4'd12;  4'd5: bay = 4'd4;   4'd6: bay = 4'd14;  4'd7: bay = 4'd6;
        4'd8: bay = 4'd3;   4'd9: bay = 4'd11;  4'd10: bay = 4'd1;  4'd11: bay = 4'd9;
        4'd12: bay = 4'd15; 4'd13: bay = 4'd7;  4'd14: bay = 4'd13; default: bay = 4'd5;
    endcase
    wire [8:0] rd9 = {1'b0, r8} + {4'd0, bay, 1'b0}, gd9 = {1'b0, g8} + {4'd0, bay, 1'b0}, bd9 = {1'b0, b8} + {3'd0, bay, 2'b00};
    wire [7:0] rdd = rd9[8] ? 8'hff : rd9[7:0], gdd = gd9[8] ? 8'hff : gd9[7:0], bdd = bd9[8] ? 8'hff : bd9[7:0];
    wire [7:0] c332 = {rdd[7:5], gdd[7:5], bdd[7:6]};

    // 两行灰度行缓存（3x3 窗口）+ 上一帧 4 位灰度（帧差）：像素第 1 字节读、第 2 字节写，同地址先读后写
    reg [7:0] lb1 [0:159]; reg [7:0] lb2 [0:159]; reg [3:0] pf [0:19199];
    reg [7:0] l1q = 0, l2q = 0; reg [3:0] pfq = 0;
    always @(posedge pclk) begin l1q <= lb1[xp]; if (pv) lb1[xp] <= gy8; end
    always @(posedge pclk) begin l2q <= lb2[xp]; if (pv) lb2[xp] <= l1q; end
    always @(posedge pclk) begin pfq <= pf[pa]; if (pv) pf[pa] <= gn; end

    wire cstart = ve && arm && !cap;                            // 本帧开始采集（与下面 cap <= 1 同拍）
    // 卷积引擎用的彩色行缓存（上一行 RGB565）：锐化/降噪把灰度增量加回窗口中心像素的彩色
    reg [15:0] lbc [0:159]; reg [15:0] lcq = 0;
    always @(posedge pclk) begin lcq <= lbc[xp]; if (pv) lbc[xp] <= px; end
    // RGB332 + Bayer 抖动（与上面原图路径相同的算法）
    function [7:0] q332d; input [7:0] r, g, b; input [3:0] k; reg [8:0] a, c, e;
        begin
            a = {1'b0, r} + {4'd0, k, 1'b0}; c = {1'b0, g} + {4'd0, k, 1'b0}; e = {1'b0, b} + {3'd0, k, 2'b00};
            q332d = {a[8] ? 3'd7 : a[7:5], c[8] ? 3'd7 : c[7:5], e[8] ? 2'd3 : e[7:6]};
            if (q332d == 8'hff) q332d = 8'hfe;
        end
    endfunction

    // 第 1 拍（像素到齐）：移窗口、寄存原图/灰度/帧差/红色判断
    reg [7:0] t0 = 0, t1 = 0, t2 = 0, m0 = 0, m1 = 0, m2 = 0, w0 = 0, w1 = 0, w2 = 0;
    reg [7:0] o332 = 0, gq = 0; reg mot = 0, eok = 0, pv_d = 0; reg [14:0] wa2 = 0;
    reg [3:0] bq = 0; reg [1:0] zq = 0; reg lnq = 0; reg [15:0] cc1 = 0, cc2 = 0; reg cok = 0;
    always @(posedge pclk) begin
        pv_d <= pv;
        if (pv) begin
            t0 <= t1; t1 <= t2; t2 <= l2q;  m0 <= m1; m1 <= m2; m2 <= l1q;  w0 <= w1; w1 <= w2; w2 <= gy8;
            o332 <= (c332 == 8'hff) ? 8'hfe : c332; gq <= gy8;
            mot <= ((gn > pfq) ? gn - pfq : pfq - gn) >= 4'd2;
            cc1 <= cc2; cc2 <= lcq; bq <= bay; cok <= lc >= 12'd2 && xp >= 8'd3;   // cc1 = 窗口中心 (行-1, 列-1) 的彩色
            zq <= (xp < 8'd53) ? 2'd0 : (xp < 8'd106) ? 2'd1 : 2'd2;  lnq <= xp == 8'd53 || xp == 8'd106;
            eok <= lc >= 12'd2 && xp >= 8'd2;
            wa2 <= pa;
        end
    end
    // 第 2 拍：Sobel（窗口右下角是当前像素，结果写在当前像素位置，整体偏 1 像素可忽略）+ 按模式选输出
    wire [9:0] gxp = {2'b0, t2} + {1'b0, m2, 1'b0} + {2'b0, w2}, gxn = {2'b0, t0} + {1'b0, m0, 1'b0} + {2'b0, w0};
    wire [9:0] gyp = {2'b0, w0} + {1'b0, w1, 1'b0} + {2'b0, w2}, gyn = {2'b0, t0} + {1'b0, t1, 1'b0} + {2'b0, t2};
    wire [9:0] ax = (gxp > gxn) ? gxp - gxn : gxn - gxp, ay = (gyp > gyn) ? gyp - gyn : gyn - gyp;
    wire [10:0] mg = {1'b0, ax} + {1'b0, ay};
    wire [7:0] e8 = !eok ? 8'd0 : (mg > 11'd255) ? 8'd255 : mg[7:0];
    function [7:0] g332; input [7:0] v; g332 = (v[7:5] == 3'd7) ? 8'hfe : {v[7:5], v[7:5], v[7:6]}; endfunction
    // ---- 3x3 卷积核（窗口 t=行-2  m=行-1  w=当前行，下标 0/1/2 = 列-2/列-1/当前列，中心 m1） ----
    wire signed [12:0] dsh = {3'd0, m1, 2'b00} - {5'd0, t1} - {5'd0, m0} - {5'd0, m2} - {5'd0, w1};   // 拉普拉斯
    wire [11:0] gsum3 = {4'd0, t0} + {3'd0, t1, 1'b0} + {4'd0, t2} + {3'd0, m0, 1'b0} + {2'd0, m1, 2'b00}
                      + {3'd0, m2, 1'b0} + {4'd0, w0} + {3'd0, w1, 1'b0} + {4'd0, w2};                    // 高斯 1-2-1
    wire signed [12:0] dbl = {5'd0, gsum3[11:4]} - {5'd0, m1};
    wire signed [12:0] dk = (ks == 3'd1) ? dbl : dsh;                    // 加回彩色的灰度增量
    function [7:0] addc; input [7:0] c; input signed [12:0] d; reg signed [13:0] t;
        begin t = $signed({6'd0, c}) + d; addc = t[13] ? 8'd0 : (t > 14'sd255) ? 8'd255 : t[7:0]; end
    endfunction
    wire [7:0] cr = {cc1[15:11], cc1[15:13]}, cgc = {cc1[10:5], cc1[10:9]}, cb = {cc1[4:0], cc1[4:2]};
    wire [12:0] alap = dsh[12] ? -dsh : dsh;
    wire [7:0] lap8 = (alap > 13'd255) ? 8'd255 : alap[7:0];
    wire signed [12:0] emb = 13'sd128 + {5'd0, m2} + {5'd0, w1} + {5'd0, w2} - {5'd0, t0} - {5'd0, t1} - {5'd0, m0};
    wire [7:0] emb8 = emb[12] ? 8'd0 : (emb > 13'sd255) ? 8'd255 : emb[7:0];
    // ---- 小 CNN：第 1 层 f = ReLU(Sobel - 32)，存两行特征图；第 2 层 3x3 最大池化 ----
    //   f 在像素第 2 拍（pv_d）算出；pv_d 读两行特征图的旧值，下一像素的 pv 写回并移入池化窗口
    wire [7:0] f1 = (e8 > 8'd32) ? e8 - 8'd32 : 8'd0;
    reg [7:0] lbf1 [0:159]; reg [7:0] lbf2 [0:159];
    reg [7:0] fr = 0, fq1 = 0, fq2 = 0; reg [7:0] xpd = 0;
    reg [7:0] pa0 = 0, pa1 = 0, pa2 = 0, pb0 = 0, pb1 = 0, pb2 = 0, pc0 = 0, pc1 = 0, pc2 = 0;
    always @(posedge pclk) begin
        if (pv) begin
            lbf1[xpd] <= fr; lbf2[xpd] <= fq1; xpd <= xp;
            pa0 <= pa1; pa1 <= pa2; pa2 <= fq2;  pb0 <= pb1; pb1 <= pb2; pb2 <= fq1;  pc0 <= pc1; pc1 <= pc2; pc2 <= fr;
        end
        if (pv_d) begin fr <= f1; fq1 <= lbf1[xpd]; fq2 <= lbf2[xpd]; end
    end
    function [7:0] mx; input [7:0] a, b; mx = (a > b) ? a : b; endfunction
    wire [7:0] pool = mx(mx(mx(pa0, pa1), mx(pa2, pb0)), mx(mx(pb1, pb2), mx(mx(pc0, pc1), pc2)));
    reg [7:0] kov; reg khit;
    always @* case (ks)
        3'd0, 3'd1: begin kov = eok ? q332d(addc(cr, dk), addc(cgc, dk), addc(cb, dk), bq) : 8'h00; khit = 1'b0; end
        3'd2: begin kov = eok ? g332(lap8) : 8'h00; khit = eok && lap8 >= 8'd32; end
        3'd3: begin kov = eok ? g332(emb8) : 8'h00; khit = 1'b0; end
        default: begin kov = cok ? g332(pool) : 8'h00; khit = cok && pool >= 8'd32; end
    endcase
    reg [7:0] ov; reg hit;
    always @* case (vm)
        3'd1: begin ov = g332(e8); hit = e8 >= 8'd64; end                         // 边缘（白线）
        3'd2: begin ov = kov; hit = khit; end                                     // 卷积引擎
        // 帧差 + 工位分区：分界线浅灰；运动点在报警工位标红、其余工位标黄；其余压暗变灰
        3'd3: begin ov = lnq ? 8'hdb : mot ? (za[zq] ? 8'he0 : 8'hfc) : g332({1'b0, gq[7:1]}); hit = mot; end
        default: begin ov = o332; hit = 1'b0; end                                  // 原图
    endcase

    always @(posedge pclk) begin
        pc <= pc + 1'b1;
        rq_s <= {rq_s[1:0], req_t};
        if (ve) begin lc <= 0; base <= 0; end
        else if (he) begin lc <= lc + 1'b1; base <= base + 15'd160; end
        bc <= !ch ? 12'd0 : (bc == 12'hfff ? bc : bc + 1'b1);
        if (he) hl <= bc;
        if (hs && vpend) begin vpend <= 0; ftg <= ~ftg; end
        if (ve) vpend <= 1;
        // 请求后的下一次 VSYNC 跳变开始采集；够 120 行，或不足 120 行又遇到跳变，就结束
        if (ve) begin
            if (cap && lc != 0) begin cap <= 0; dtg <= ~dtg; end
            else if (arm && !cap) begin cap <= 1; arm <= 0; hcl <= hc; hc <= 0; end
        end else if (cap && he && lc == 12'd119) begin cap <= 0; dtg <= ~dtg; end
        if (rq_s[2] != rq_s[1]) arm <= 1;
        if (psel && !bc[0]) hold <= cd;
        we <= pv_d; wa <= wa2; wd <= ov;
        if (pv_d && hit && hc != 15'h7fff) hc <= hc + 1'b1;
        // 各工位区运动点：与 hcl 同时在下一帧开始时锁存（最后一个像素早已计完）
        if (cstart) begin zl0 <= zc0; zl1 <= zc1; zl2 <= zc2; zc0 <= 0; zc1 <= 0; zc2 <= 0; ztg <= ~ztg; end
        else if (pv_d && mot) case (zq) 2'd0: zc0 <= zc0 + 1'b1; 2'd1: zc1 <= zc1 + 1'b1; default: zc2 <= zc2 + 1'b1; endcase
    end
    reg [7:0] mem [0:19199];
    always @(posedge pclk) if (we) mem[wa] <= wd;
    reg [14:0] ra; reg [7:0] rd;
    always @(posedge clk) rd <= mem[ra];

    reg [14:0] hc_s1, hc_s2; wire [24:0] hpm = hc_s2 * 10'd683;               // /192 = 占 19200 的百分比
    always @(posedge clk) begin hc_s1 <= hcl; hc_s2 <= hc_s1; hitp <= hpm[23:17]; end
    // 工位区占比：A/B 区 53 列 x 120 行 = 6360 点，n*1031>>16 = n*100/6360；C 区 54 列，略超 100 时限幅
    //   zl 在 pclk 域锁存后要一整帧才再变，翻转信号同步 3 拍后读取是稳定的
    function [6:0] zpct; input [12:0] n; reg [23:0] m; begin m = n * 11'd1031; zpct = (m[23:16] > 8'd100) ? 7'd100 : m[22:16]; end endfunction
    reg [2:0] zt_s;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin zt_s <= 0; zp0 <= 0; zp1 <= 0; zp2 <= 0; ztick <= 0; end
        else begin
            zt_s <= {zt_s[1:0], ztg}; ztick <= 1'b0;
            if (zt_s[2] != zt_s[1]) begin zp0 <= zpct(zl0); zp1 <= zpct(zl1); zp2 <= zpct(zl2); ztick <= 1'b1; end
        end

    // ---------------- clk 域：诊断计数 ----------------
    localparam integer WIN = CLK_HZ / 100000 * 1024;            // 窗口内 PCLK/1024 的上升沿数 = PCLK 频率（0.1MHz）
    reg [2:0] dn_s, fr_s, vs_s, hr_s, pc_s; reg [7:0] d_s1, d_s2, d_s3, dacc, fcnt, vcnt;
    reg [11:0] hcnt, hl_s, pcnt; reg [19:0] win;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            dn_s <= 0; fr_s <= 0; vs_s <= 0; hr_s <= 0; pc_s <= 0; d_s1 <= 0; d_s2 <= 0; d_s3 <= 0; dacc <= 0;
            fcnt <= 0; vcnt <= 0; hcnt <= 0; hl_s <= 0; pcnt <= 0; win <= 0;
            fps <= 0; vps <= 0; vlines <= 0; hlen <= 0; pmhz <= 0; dmask <= 0;
        end else begin
            dn_s <= {dn_s[1:0], dtg}; fr_s <= {fr_s[1:0], ftg};
            vs_s <= {vs_s[1:0], vsync}; hr_s <= {hr_s[1:0], href}; pc_s <= {pc_s[1:0], pc[9]};
            d_s1 <= d; d_s2 <= d_s1; d_s3 <= d_s2;
            hl_s <= hl; hlen <= hl_s;                                  // 每行一样，读数稳定
            dacc <= dacc | (d_s2 ^ d_s3);
            if (fr_s[2] != fr_s[1] && fcnt != 8'hff) fcnt <= fcnt + 1'b1;
            if (vs_s[2:1] == 2'b01) begin
                if (vcnt != 8'hff) vcnt <= vcnt + 1'b1;
                vlines <= hcnt; hcnt <= 0;
            end else if (hr_s[2:1] == 2'b01 && hcnt != 12'hfff) hcnt <= hcnt + 1'b1;
            if (s_tick) begin fps <= fcnt; fcnt <= 0; vps <= vcnt; vcnt <= 0; dmask <= dacc; dacc <= 0; end
            if (win == WIN - 1) begin win <= 0; pmhz <= pcnt; pcnt <= 0; end
            else begin win <= win + 1'b1; if (pc_s[2:1] == 2'b01) pcnt <= pcnt + 1'b1; end
        end

    // ---------------- clk 域：请求一帧 -> 等采完 -> 逐行发送 ----------------
    localparam [2:0] V_IDLE = 0, V_WAIT = 1, V_SYNC = 2, V_LN = 3, V_PIX = 4;
    reg [2:0] vst; reg [6:0] ln; reg [7:0] col; reg [7:0] scnt; reg tx_start, tx_wait; reg [7:0] tx_data; wire tx_busy;
    uart_tx_byte #(.CLKS_PER_BIT(VID_CPB)) u_tx (.clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .tx(vid_tx), .busy(tx_busy), .done());
    wire can = !tx_start && !tx_wait && !tx_busy;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin vst <= V_IDLE; scnt <= 0; sps <= 0; req_t <= 0; ln <= 0; col <= 0; ra <= 0; tx_start <= 0; tx_wait <= 0; tx_data <= 8'hff; end
        else begin
            tx_start <= 0;
            if (tx_busy) tx_wait <= 0;
            case (vst)
                V_IDLE: if (run) begin req_t <= ~req_t; vst <= V_WAIT; end
                V_WAIT: if (dn_s[2] != dn_s[1]) begin ln <= 0; ra <= 0; vst <= V_SYNC; end
                V_SYNC: if (can) begin tx_start <= 1; tx_wait <= 1; tx_data <= 8'hff; vst <= V_LN; end
                V_LN:   if (can) begin tx_start <= 1; tx_wait <= 1; tx_data <= {1'b0, ln}; col <= 0; vst <= V_PIX; end
                default: if (can) begin
                    tx_start <= 1; tx_wait <= 1; tx_data <= rd; ra <= ra + 1'b1; col <= col + 1'b1;
                    if (col == 8'd159) begin
                        ln <= ln + 1'b1; vst <= (ln == 7'd119) ? V_IDLE : V_SYNC;
                        if (ln == 7'd119 && scnt != 8'hff) scnt <= scnt + 1'b1;
                    end
                end
            endcase
            if (s_tick) begin sps <= scnt; scnt <= 0; end
        end
endmodule
