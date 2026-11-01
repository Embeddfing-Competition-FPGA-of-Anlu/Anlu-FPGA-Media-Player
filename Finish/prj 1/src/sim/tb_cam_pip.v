`timescale 1ns/1ps
// 产线监控页画中画仿真：640x480 时序 + 3.125Mbps 视频串口一帧 RGB332，逐像素比对
//   A 收到视频前原样输出；B 彩色窗口；D 横幅时第 13~14 行让给横幅；E 不在本页（en=0）原样；C 停发后原样
module tb_cam_pip;
    reg sclk = 0; always #5 sclk = ~sclk;           // 100MHz
    reg vclk = 0; always #20 vclk = ~vclk;          // 25MHz
    reg rst_n = 0; initial #200 rst_n = 1;
    reg rx = 1, en = 1, ban = 0;
    reg [9:0] h = 0, v = 0;
    always @(posedge vclk) if (h == 799) begin h <= 0; v <= (v == 524) ? 10'd0 : v + 1'b1; end else h <= h + 1'b1;
    wire de_i = h < 640 && v < 480, hs_i = !(h >= 656 && h < 752), vs_i = !(v >= 490 && v < 492);
    wire [23:0] rgb_i = {h[7:0], v[7:0], 8'h5a};
    wire hs_o, vs_o, de_o; wire [23:0] rgb_o;
    cam_pip #(.X0(304), .Y0(96), .ALIVE_FR(8)) dut (.sclk(sclk), .srst_n(rst_n), .vid_rx(rx),
        .vclk(vclk), .vrst_n(rst_n), .en(en), .ban(ban), .review_alert(1'b0), .review_tool(3'd0),
        .hs_in(hs_i), .vs_in(vs_i), .de_in(de_i), .x_in(h), .y_in(v), .rgb_in(rgb_i),
        .hs_out(hs_o), .vs_out(vs_o), .de_out(de_o), .rgb_out(rgb_o));

    function [7:0] pv; input integer l, c; pv = (l * 7 + c * 3) % 255; endfunction     // 不会是 0xFF
    function [23:0] ex; input [7:0] p; ex = {p[7:5], p[7:5], p[7:6], p[4:2], p[4:2], p[4:3], p[1:0], p[1:0], p[1:0], p[1:0]}; endfunction
    task sb(input [7:0] b); integer k; begin rx = 0; #320; for (k = 0; k < 8; k = k + 1) begin rx = b[k]; #320; end rx = 1; #320; end endtask

    // 输出端独立数坐标，按坐标算期望值
    reg dq = 0, vq = 1, chk = 0, pip = 0; integer ox = 0, oy = 0, err = 0, n = 0, nw = 0;
    reg [23:0] e; reg banrow;
    always @(posedge vclk) if (rst_n) begin
        if (de_o && chk) begin
            banrow = ban && oy >= 208 && oy < 240;
            if (pip && en && !banrow && ox >= 304 && ox < 624 && oy >= 96 && oy < 336) begin
                e = ex(pv((oy - 96) / 2, (ox - 304) / 2)); nw = nw + 1;
            end else e = {ox[7:0], oy[7:0], 8'h5a};
            n = n + 1;
            if (rgb_o !== e) begin err = err + 1; if (err < 6) $display("(%0d,%0d) %h expect %h pip=%0d en=%0d ban=%0d", ox, oy, rgb_o, e, pip, en, ban); end
        end
        dq <= de_o; vq <= vs_o;
        if (vs_o != vq) oy <= 0; else if (dq && !de_o) oy <= oy + 1;
        ox <= de_o ? ox + 1 : 0;
    end

    integer l, c, e0;
    task frame_check(input [8*16-1:0] tag, input expect_win);
        begin
            e0 = err; chk = 1; pip = expect_win; n = 0; nw = 0; @(negedge vs_i); chk = 0;
            $display("%0s: %0d px (%0d in window), err %0d", tag, n, nw, err - e0);
        end
    endtask
    initial begin
        wait (rst_n); @(negedge vs_i); @(negedge vs_i);
        frame_check("A before video", 0);
        sb(8'h12); sb(8'h34);                                    // 垃圾字节，验证重新同步
        for (l = 0; l < 120; l = l + 1) begin
            sb(8'hff); sb(l);
            for (c = 0; c < 160; c = c + 1) sb(pv(l, c));
        end
        @(negedge vs_i); @(negedge vs_i);
        frame_check("B color window", 1);
        if (nw != 76800) err = err + 1;
        ban = 1; frame_check("D banner rows", 1); ban = 0;
        if (nw != 76800 - 320 * 32) err = err + 1;
        en = 0; frame_check("E other page", 1); en = 1;
        if (nw != 0) err = err + 1;
        #200000000; @(negedge vs_i);
        frame_check("C video stopped", 0);
        if (err == 0) $display("PIP PASS"); else $display("PIP FAIL err=%0d", err);
        $finish;
    end
endmodule
