`timescale 1ns/1ps
// 云台状态机单元测试（参数缩小加速）：标定 -> 巡检 A/B/C -> 本地闯入 -> 看板告警抢占 -> 链路中断 -> 手动接管释放
module tb_gimbal;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0; reg [3:0] mc = 0; wire ms = mc == 4'd9;           // 10 拍 = 1 "ms"
    always @(posedge clk) mc <= ms ? 4'd0 : mc + 1'b1;
    reg en = 0, cv = 0, alm = 0; reg [2:0] tl = 0;
    reg pm = 0, pp = 0, ce = 0, cf = 0, sv = 0; reg [2:0] ct = 1; reg pe = 0; reg [2:0] iz = 0;
    wire [7:0] pan, tilt; wire [3:0] st, er, cal; wire [2:0] tt; wire mv, it;
    gimbal_control #(.STEP_MS(2), .SETTLE_MS(10), .LINK_TIMEOUT_MS(40), .TASK_TIMEOUT_MS(2000),
        .PATROL_HOLD_MS(30), .RELEASE_MS(50)) dut (
        .clk(clk), .rst_n(rst_n), .ms_tick(ms), .enable(en), .command_valid(cv), .alarm(alm), .tool(tl),
        .pan_minus(pm), .pan_plus(pp), .tilt_minus(1'b0), .tilt_plus(1'b0), .center(ce), .confirm(cf),
        .save_preset(sv), .calibration_tool(ct), .patrol_en(pe), .intr_zone(iz),
        .pan(pan), .tilt(tilt), .state(st), .error(er), .task_tool(tt), .calibrated(cal), .moving(mv), .intr_task(it));

    // 看板心跳：link=1 时每 20ms 一个 !G（告警位 alm、工位 tl）
    reg link = 0; integer lc = 0;
    always @(posedge clk) begin cv <= 0; if (link) begin lc = lc + 1; if (lc == 200) begin lc = 0; cv <= 1; end end end

    // 巡检到站记录
    reg [2:0] seq [0:7]; reg [7:0] seqp [0:7]; integer ns = 0; reg hold_q = 0;
    always @(posedge clk) begin
        hold_q <= st == 4'd4 && dut.phold;
        if (st == 4'd4 && dut.phold && !hold_q && ns < 8) begin seq[ns] = tt; seqp[ns] = pan; ns = ns + 1; end
    end
    reg mv_seen = 0; always @(posedge clk) if (st == 4'd2 && mv) mv_seen = 1;

    integer errs = 0;
    task expect(input c, input [8*56:1] what);
        begin
            if (!c) begin errs = errs + 1; $display("FAIL: %0s (st=%0d pan=%0d tt=%0d er=%0d it=%0d)", what, st, pan, tt, er, it); end
            else $display("ok  : %0s", what);
        end
    endtask
    task key(input integer k);                          // 0 水平- 1 水平+ 2 回中 3 确认 4 保存预置位
        begin
            @(posedge clk); case (k) 0: pm <= 1; 1: pp <= 1; 2: ce <= 1; 3: cf <= 1; default: sv <= 1; endcase
            @(posedge clk); pm <= 0; pp <= 0; ce <= 0; cf <= 0; sv <= 0; repeat (5) @(posedge clk);
        end
    endtask
    task waitms(input integer m); repeat (m * 10) @(posedge clk); endtask
    task waitst(input [3:0] s, input integer mmax); integer n;
        begin n = 0; while (st != s && n < mmax * 10) begin @(posedge clk); n = n + 1; end end
    endtask

    initial begin
        repeat (3) @(posedge clk); rst_n = 1; en = 1; waitms(2);
        expect(st == 4'd1, "enable -> IDLE");
        // ---- 标定：A 70 度、B 90 度、C 110 度 ----
        ct = 1; key(0); key(0); key(0); key(0);
        expect(st == 4'd9 && pan == 70, "direction keys -> MANUAL, pan 70");
        key(4);
        ct = 2; key(2); key(4);
        ct = 3; key(1); key(1); key(1); key(1); key(4);
        expect(cal[2:0] == 3'b111 && pan == 110, "A/B/C presets saved");
        waitms(60);
        expect(st == 4'd1, "no key for RELEASE_MS -> back to IDLE");
        // ---- 巡检 ----
        pe = 1; waitms(700);
        expect(ns >= 4 && seq[0] == 1 && seqp[0] == 70 && seq[1] == 2 && seqp[1] == 90 &&
               seq[2] == 3 && seqp[2] == 110 && seq[3] == 1 && seqp[3] == 70, "patrol A -> B -> C -> A");
        // ---- 本地闯入 B ----
        iz = 3'b010; waitst(4'd5, 500);
        expect(st == 4'd5 && it && tt == 2 && pan == 90 && mv_seen, "intrusion B -> point at B, wait for confirm");
        waitms(50);
        expect(st == 4'd5, "REVIEW holds until confirm (no link needed)");
        key(3);
        expect(st == 4'd6, "confirm -> CONFIRMED");
        waitms(30);
        expect(st == 4'd6, "zone still busy -> stay CONFIRMED");
        iz = 3'b000; waitst(4'd4, 500);
        expect(st == 4'd4 && !it, "zone quiet -> return to centre, resume patrol");
        // ---- 看板告警 C 抢占巡检 ----
        link = 1; alm = 1; tl = 3; waitst(4'd5, 1000);
        expect(st == 4'd5 && !it && tt == 3 && pan == 110, "board alarm C preempts patrol");
        iz = 3'b001; waitms(30);
        expect(st == 4'd5 && tt == 3, "intrusion ignored while board alarm is active");
        iz = 3'b000; alm = 0; waitst(4'd4, 1000);
        expect(st == 4'd4, "board alarm cleared -> return, resume patrol");
        // ---- 告警中链路中断 ----
        alm = 1; tl = 1; waitst(4'd5, 1000); link = 0; waitst(4'd8, 200);
        expect(st == 4'd8 && er == 1, "link lost during board alarm -> FAILED err 1");
        waitms(80);
        expect(st == 4'd8, "board alarm still latched -> no local release");
        link = 1; alm = 0; waitst(4'd4, 500);
        expect(st == 4'd4, "link back with normal status -> patrol again");
        // ---- 巡检中手动接管 ----
        key(1);
        expect(st == 4'd9, "direction key during patrol -> MANUAL");
        waitst(4'd4, 200);
        expect(st == 4'd4, "released after RELEASE_MS -> patrol again");
        // ---- 只开闯入联动（巡检关） ----
        pe = 0; waitst(4'd1, 200);
        iz = 3'b100; waitst(4'd5, 500);
        expect(st == 4'd5 && it && tt == 3 && pan == 110, "guard only: intrusion C");
        key(3); iz = 3'b000; waitst(4'd1, 500);
        expect(st == 4'd1 && pan == 90, "after confirm + quiet -> centre, stay IDLE");
        $display("GIMBAL %s errors=%0d", errs == 0 ? "PASS" : "FAIL", errs);
        $finish;
    end
endmodule
