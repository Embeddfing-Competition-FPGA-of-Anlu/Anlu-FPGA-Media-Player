`timescale 1ns/1ps
// 发送板摄像头链路仿真：SCCB 从机模型（芯片 ID 0x5640）+ DVP 摄像头模型（160x120 RGB565）+ 视频串口解码逐字节校验
//   原图 -> SW5 诊断 -> SW6 现场视觉 5 种模式逐帧逐字节比对（含命中占比）-> 云台舵机 PWM 脉宽
//   clk 10MHz，pclk 10MHz，仿真里视频串口 5Mbps
module tb_cam;
    reg clk = 0;  always #50 clk = ~clk;
    reg pclk = 0; always #50 pclk = ~pclk;
    reg [7:0] sw = 0;
    wire scl, sda; pullup (scl); pullup (sda);
    reg sda_dr = 0; assign sda = sda_dr ? 1'b0 : 1'bz;
    wire cam_rst_n, cam_pwdn, vtx, ltx; reg vs = 0, hr = 0; reg [7:0] dd = 0;
    wire [3:0] row, seg_sel; wire [7:0] seg_led, led; wire spn, stl;
    reg [4:0] key = 5'h1f;                                 // 键号 0~15，1f = 没按
    wire [3:0] kn = 4'd15 - key[3:0];
    wire [3:0] kcol = (!key[4] && !row[kn[3:2]]) ? ~(4'b0001 << kn[1:0]) : 4'hf;
    sender_top #(.CLK_HZ(10000000), .SEC_MS(20)) dut (.clk(clk), .row(row), .col(kcol), .seg_sel(seg_sel),
        .seg_led(seg_led), .sw(sw), .led(led), .link_tx(ltx), .link_rx(1'b1),
        .cam_scl(scl), .cam_sda(sda), .cam_rst_n(cam_rst_n), .cam_pwdn(cam_pwdn),
        .cam_pclk(pclk), .cam_vsync(vs), .cam_href(hr), .cam_d(dd), .vid_tx(vtx), .servo_pan(spn), .servo_tilt(stl), .oled_scl(), .oled_sda());

    // ---------------- SCCB 从机：地址 0x3C，16 位寄存器地址 ----------------
    reg [7:0] sh, tx_b, wdat; reg [15:0] ra; reg sel = 0, rdm = 0, snd = 0;
    integer bitn = -1, byten = 0, nwr = 0;
    reg [7:0] r4300 = 0, r3809 = 0, r380b = 0, r3008 = 0, r3103 = 0;
    function [7:0] rreg; input [15:0] a;
        case (a) 16'h300a: rreg = 8'h56; 16'h300b: rreg = 8'h40; 16'h3103: rreg = 8'h13; default: rreg = 8'h00; endcase
    endfunction
    always @(negedge sda) if (scl === 1'b1) begin bitn = -1; byten = 0; snd = 0; sda_dr = 0; end       // START
    always @(posedge sda) if (scl === 1'b1) begin                                                       // STOP
        if (sel && !rdm && byten == 4) begin
            nwr = nwr + 1;
            case (ra) 16'h4300: r4300 = wdat; 16'h3809: r3809 = wdat; 16'h380b: r380b = wdat;
                      16'h3008: r3008 = wdat; 16'h3103: r3103 = wdat; default: ; endcase
        end
        byten = 0; snd = 0;
    end
    always @(posedge scl) begin
        if (!snd && bitn >= 0 && bitn < 8) sh = {sh[6:0], sda};
        if (snd && bitn == 8 && sda === 1'b1) snd = 0;                                                  // 主机 NACK
    end
    always @(negedge scl) begin
        bitn = bitn + 1;
        if (bitn == 8) begin
            if (snd) sda_dr = 0;
            else begin
                byten = byten + 1;
                if (byten == 1) begin sel = sh[7:1] == 7'h3c; rdm = sh[0]; if (rdm) tx_b = rreg(ra); end
                else if (byten == 2) ra[15:8] = sh;
                else if (byten == 3) ra[7:0] = sh;
                else if (byten == 4) wdat = sh;
                sda_dr = sel;                                                                           // ACK
            end
        end else if (bitn == 9) begin
            bitn = 0; sda_dr = 0;
            if (rdm && sel && byten == 1) begin snd = 1; byten = 2; end
            if (snd) sda_dr = ~tx_b[7];
        end else if (snd && bitn > 0 && bitn < 8) sda_dr = ~tx_b[7 - bitn];
    end

    // ---------------- 摄像头 DVP：120 行 x 320 字节 RGB565，下降沿换数据 ----------------
    //   VSYNC 每帧随机 3 种形式：0 高脉冲  1 低脉冲  2 整帧高电平（行期间为高）
    //   像素 RGB565 先低字节（上板确认）；(5,7) 为纯白，验证 0xFF 压成 0xFE
    reg movf = 0; integer box = 0;                         // movf=1：画面里加一个每帧右移 8 列的白块（帧差测试）
    function [15:0] pv; input integer l, p; reg [15:0] q;
        begin q = l * 977 + p * 131 + l * p; if (l == 5 && p == 7) q = 16'hffff;
              if (movf && p >= box && p < box + 8) q = 16'hffff;
              pv = q; end
    endfunction
    integer fl, fb, fn = 0, vm = 0, vmseen = 0;
    initial begin
        wait (cam_rst_n === 1'b1 && cam_pwdn === 1'b0);
        forever begin
            box = (fn * 8) % 152;
            vm = (vmseen == 7) ? 0 : {$random} % 3; fn = fn + 1;          // 三种都采到后固定（真摄像头不会换形式）
            @(negedge pclk);
            case (vm)
                0: begin vs = 1; repeat (400) @(negedge pclk); vs = 0; repeat (200) @(negedge pclk); end
                1: begin vs = 0; repeat (400) @(negedge pclk); vs = 1; repeat (200) @(negedge pclk); end
                default: begin vs = 0; repeat (550) @(negedge pclk); vs = 1; repeat (50) @(negedge pclk); end
            endcase
            for (fl = 0; fl < 120; fl = fl + 1) begin
                for (fb = 0; fb < 320; fb = fb + 1) begin
                    @(negedge pclk); hr = 1; dd = fb[0] ? pv(fl, fb / 2) >> 8 : pv(fl, fb / 2) & 8'hff;
                end
                @(negedge pclk); hr = 0; dd = 0; repeat (79) @(negedge pclk);
            end
            if (vm == 2) vs = 0;                                                   // 整帧电平：帧尾回低
            repeat (300) @(negedge pclk);
        end
    end
    // 记录被采集的帧用了哪种 VSYNC：以采集中的第一行为准（采集可能在上一帧的帧尾跳变就开始）
    always @(posedge hr) if (dut.u_cam_vid.cap && dut.u_cam_vid.lc == 0) vmseen = vmseen | (1 << vm);

    // ---------------- 视频串口解码：0xFF + 行号 + 160 字节 RGB332 ----------------
    defparam dut.u_cam_vid.VID_CPB = 2;                 // 仿真加速：视频串口 5Mbps
    localparam integer BT = 2 * 100;
    reg [15:0] tv;
    // RGB332 量化前 4x4 Bayer 抖动（与 cam_video 一致）：R/G 加 2*b、B 加 4*b，饱和到 255 后截断
    function [3:0] bayer; input [3:0] i;
        case (i) 0: bayer = 0; 1: bayer = 8; 2: bayer = 2; 3: bayer = 10; 4: bayer = 12; 5: bayer = 4; 6: bayer = 14; 7: bayer = 6;
                 8: bayer = 3; 9: bayer = 11; 10: bayer = 1; 11: bayer = 9; 12: bayer = 15; 13: bayer = 7; 14: bayer = 13; default: bayer = 5; endcase
    endfunction
    function [7:0] e332; input integer l, p; integer r8, g8, b8, bv;
        begin
            tv = pv(l, p); bv = bayer({l[1:0], p[1:0]});
            r8 = {tv[15:11], tv[15:13]} + 2 * bv;  g8 = {tv[10:5], tv[10:9]} + 2 * bv;  b8 = {tv[4:0], tv[4:2]} + 4 * bv;
            if (r8 > 255) r8 = 255;  if (g8 > 255) g8 = 255;  if (b8 > 255) b8 = 255;
            e332 = {r8[7:5], g8[7:5], b8[7:6]}; if (e332 == 8'hff) e332 = 8'hfe;
        end
    endfunction
    // 现场视觉参考模型（和 cam_video 的定点算法逐位一致）
    function [7:0] gry; input integer l, p; reg [15:0] q; reg [9:0] s;
        begin q = pv(l, p); s = {2'b0, q[15:11], q[15:13]} + {1'b0, q[10:5], q[10:9], 1'b0} + {2'b0, q[4:0], q[4:2]}; gry = s[9:2]; end
    endfunction
    function [7:0] g3; input [7:0] v; g3 = (v[7:5] == 3'd7) ? 8'hfe : {v[7:5], v[7:5], v[7:6]}; endfunction
    function [7:0] sob; input integer l, p; integer a, b, c, d2, ax, ay;
        begin
            if (l < 2 || p < 2) sob = 0;
            else begin
                a = gry(l-2, p) + 2 * gry(l-1, p) + gry(l, p);    b = gry(l-2, p-2) + 2 * gry(l-1, p-2) + gry(l, p-2);
                c = gry(l, p-2) + 2 * gry(l, p-1) + gry(l, p);    d2 = gry(l-2, p-2) + 2 * gry(l-2, p-1) + gry(l-2, p);
                ax = a > b ? a - b : b - a; ay = c > d2 ? c - d2 : d2 - c;
                sob = (ax + ay > 255) ? 255 : ax + ay;
            end
        end
    endfunction
    // ---- 卷积引擎参考模型：窗口 t=行-2 m=行-1 w=当前行，列 -2/-1/0，中心 (行-1, 列-1) ----
    function integer cl8; input integer v; cl8 = v < 0 ? 0 : v > 255 ? 255 : v; endfunction
    function integer dshf; input integer l, p;
        dshf = 4 * gry(l-1, p-1) - gry(l-2, p-1) - gry(l-1, p-2) - gry(l-1, p) - gry(l, p-1);
    endfunction
    function integer dblf; input integer l, p;
        dblf = (gry(l-2, p-2) + 2 * gry(l-2, p-1) + gry(l-2, p) + 2 * gry(l-1, p-2) + 4 * gry(l-1, p-1)
              + 2 * gry(l-1, p) + gry(l, p-2) + 2 * gry(l, p-1) + gry(l, p)) / 16 - gry(l-1, p-1);
    endfunction
    function [7:0] cconv; input integer k, l, p; reg [15:0] q; integer d, r8, g8, b8, bv;
        begin
            if (l < 2 || p < 2) cconv = 0;
            else begin
                q = pv(l-1, p-1); d = (k == 1) ? dblf(l, p) : dshf(l, p); bv = bayer({l[1:0], p[1:0]});
                r8 = cl8({q[15:11], q[15:13]} + d) + 2 * bv;  g8 = cl8({q[10:5], q[10:9]} + d) + 2 * bv;  b8 = cl8({q[4:0], q[4:2]} + d) + 4 * bv;
                if (r8 > 255) r8 = 255;  if (g8 > 255) g8 = 255;  if (b8 > 255) b8 = 255;
                cconv = {r8[7:5], g8[7:5], b8[7:6]}; if (cconv == 8'hff) cconv = 8'hfe;
            end
        end
    endfunction
    function [7:0] lapf; input integer l, p; integer d;
        begin if (l < 2 || p < 2) lapf = 0; else begin d = dshf(l, p); if (d < 0) d = -d; lapf = d > 255 ? 255 : d; end end
    endfunction
    function [7:0] embf; input integer l, p;
        embf = (l < 2 || p < 2) ? 0 : cl8(128 + gry(l-1, p) + gry(l, p-1) + gry(l, p) - gry(l-2, p-2) - gry(l-2, p-1) - gry(l-1, p-2));
    endfunction
    // 小 CNN：f = ReLU(Sobel - 32)，输出 = 行 l-2..l、列 p-3..p-1 上 f 的最大值
    function [7:0] fea; input integer l, p; integer e; begin e = sob(l, p); fea = e > 32 ? e - 32 : 0; end endfunction
    function [7:0] cnnf; input integer l, p; integer r, c; reg [7:0] mxv;
        begin
            mxv = 0;
            if (l >= 2 && p >= 3) for (r = l - 2; r <= l; r = r + 1) for (c = p - 3; c <= p - 1; c = c + 1) if (fea(r, c) > mxv) mxv = fea(r, c);
            cnnf = mxv;
        end
    endfunction
    // m：0 原图 1 边缘 3 帧差分区 10~14 卷积核 0~4
    function [7:0] pexp; input integer m, l, p;
        case (m)
            1: pexp = g3(sob(l, p));
            3: pexp = (p == 53 || p == 106) ? 8'hdb : g3({1'b0, gry(l, p) >> 1});   // 静止：只有工位分界线
            10, 11: pexp = cconv(m - 10, l, p);
            12: pexp = g3(lapf(l, p));
            13: pexp = g3(embf(l, p));
            14: pexp = (l >= 2 && p >= 3) ? g3(cnnf(l, p)) : 8'h00;
            default: pexp = e332(l, p);
        endcase
    endfunction
    integer ecnt [0:15]; integer xl, xp2;
    initial begin
        for (xl = 0; xl < 16; xl = xl + 1) ecnt[xl] = 0;
        for (xl = 0; xl < 120; xl = xl + 1) for (xp2 = 0; xp2 < 160; xp2 = xp2 + 1) begin
            if (sob(xl, xp2) >= 64) ecnt[1] = ecnt[1] + 1;
            if (lapf(xl, xp2) >= 32) ecnt[12] = ecnt[12] + 1;
            if (cnnf(xl, xp2) >= 32) ecnt[14] = ecnt[14] + 1;
        end
        $display("expected hits (of 19200): edge %0d  laplace %0d  cnn %0d", ecnt[1], ecnt[12], ecnt[14]);
    end
    reg chk = 1; integer tm = 0;
    reg [7:0] rb, ex; integer i, st = 0, eln = 0, col = 0, lines_ok = 0, frames_ok = 0, err = 0;
    initial begin
        wait (dut.rst_n === 1'b1);
        forever begin
            @(negedge vtx); #(BT / 2);
            for (i = 0; i < 8; i = i + 1) begin #(BT); rb[i] = vtx; end
            #(BT); if (vtx !== 1'b1) err = err + 1;
            case (st)
                0: if (rb == 8'hff) st = 1; else err = err + 1;
                1: begin
                    if (rb != eln) begin err = err + 1; if (err < 5) $display("line no %0d, expect %0d", rb, eln); end
                    col = 0; st = 2;
                end
                default: begin
                    ex = pexp(tm, eln, col);
                    if (chk && rb != ex) begin err = err + 1; if (err < 5) $display("line %0d byte %0d: %h expect %h", eln, col, rb, ex); end
                    col = col + 1;
                    if (col == 160) begin
                        st = 0; lines_ok = lines_ok + 1; eln = (eln + 1) % 120;
                        if (eln == 0) begin frames_ok = frames_ok + 1; $display("%t frame %0d ok", $time, frames_ok); end
                    end
                end
            endcase
        end
    end

    reg diag_ok; integer mi, mk, f0, e0, hexp, vis_bad = 0;
    task press(input integer k); begin key = k; #50000000; key = 5'h1f; #45000000; end endtask
    time rp, rt, hp = 0, ht = 0, per = 0, lastp = 0;
    always @(posedge spn) begin if (lastp != 0) per = $time - lastp; lastp = $time; rp = $time; end
    always @(negedge spn) hp = $time - rp;
    always @(posedge stl) rt = $time;
    always @(negedge stl) ht = $time - rt;
    integer smax = 0; always @(posedge clk) if (dut.cam_sps > smax) smax = dut.cam_sps;
    initial begin
        wait (dut.cam_st == 2'd1);
        $display("%t init done: %0d writes, 4300=%h 3809=%h 380B=%h 3008=%h 3103=%h", $time, nwr, r4300, r3809, r380b, r3008, r3103);
        wait (frames_ok >= 4 && vmseen == 7);
        sw[5] = 1; #30000000;
        $display("SW5 diag: item=%0d fps=%0d vps=%0d lines=%0d hlen=%0d pclk=%0d.%0dMHz dmask=%h (per 20ms)",
                 dut.item, dut.cam_fps, dut.cam_vps, dut.cam_vlines, dut.cam_hlen, dut.cam_pmhz / 10, dut.cam_pmhz % 10, dut.cam_dmask);
        diag_ok = err == 0 && nwr == 254 && r4300 == 8'h6f && r3809 == 8'ha0 && r380b == 8'h78 && r3008 == 8'h02 && r3103 == 8'h03
            && dut.item == 12 && dut.cam_fps != 0 && dut.cam_vps != 0 && dut.cam_vlines == 120 && dut.cam_hlen == 320
            && dut.cam_pmhz >= 99 && dut.cam_pmhz <= 101 && dut.cam_dmask == 8'hff && dut.cam_id == 16'h5640 && smax >= 1;
        $display("diag %0s, %0d lines checked", diag_ok ? "ok" : "FAIL", lines_ok);
        // ---- SW6 现场视觉：按 0~4 选算法，模式切换后跳过 2 帧，第 3 帧逐字节比对，再核对命中占比 ----
        sw[5] = 0; sw[6] = 1; #30000000;
        // 顺序：边缘 -> 键 2 进卷积（锐化）-> 再按键 2 依次 降噪/轮廓/浮雕/CNN -> 帧差分区 -> 原图
        for (mi = 0; mi < 8; mi = mi + 1) begin
            mk = (mi == 0) ? 1 : (mi <= 5) ? 2 : (mi == 6) ? 3 : 0;
            chk = 0; press(mk); tm = (mi >= 1 && mi <= 5) ? 9 + mi : mk; f0 = frames_ok;
            wait (frames_ok >= f0 + 2); e0 = err; chk = 1;
            wait (frames_ok >= f0 + 3); chk = 0; #15000000;
            hexp = (ecnt[tm] * 683) >> 17;
            if (dut.cam_hitp != hexp) begin vis_bad = vis_bad + 1; $display("  mode %0d hitp %0d expect %0d", tm, dut.cam_hitp, hexp); end
            if (err != e0) vis_bad = vis_bad + 1;
            $display("mode %0d (vmode %0d ksel %0d): frame err %0d, hit %0d%%", tm, dut.vmode, dut.ksel, err - e0, dut.cam_hitp);
        end
        // ---- 帧差：切到运动模式，画面里白块逐帧右移，应检测出运动 ----
        press(3); movf = 1; chk = 0; f0 = frames_ok;
        wait (frames_ok >= f0 + 6); #15000000;
        $display("motion test (moving box): hit %0d%%, zones A %0d%% B %0d%% C %0d%%, intrusion zact=%b",
                 dut.cam_hitp, dut.zp0, dut.zp1, dut.zp2, dut.zact);
        if (dut.cam_hitp < 5) vis_bad = vis_bad + 1;
        if (dut.zact == 0) begin vis_bad = vis_bad + 1; $display("  moving box did not raise a zone intrusion"); end
        movf = 0;
        // ---- 云台：默认 90 度 1.5ms；水平+ 一次 95 度，俯仰- 一次 85 度 ----
        $display("servo default: pan %0dus tilt %0dus period %0dus", hp / 1000, ht / 1000, per / 1000);
        if (hp != 1500000 || ht != 1500000 || per != 20000000) vis_bad = vis_bad + 1;
        press(14); press(10); #45000000;
        $display("servo after keys: pan %0dus (95 deg) tilt %0dus (85 deg), pan=%0d tilt=%0d", hp / 1000, ht / 1000, dut.pan, dut.tilt);
        if (hp != 1555000 || ht != 1444000) vis_bad = vis_bad + 1;
        if (diag_ok && err == 0 && vis_bad == 0) $display("CAM PASS");
        else $display("CAM FAIL err=%0d vis_bad=%0d diag=%0d", err, vis_bad, diag_ok);
        $finish;
    end
    initial begin #2000000000; #2000000000; #2000000000; $display("TIMEOUT st=%0d frames=%0d err=%0d vmseen=%0d", dut.cam_st, frames_ok, err, vmseen); $finish; end
endmodule
