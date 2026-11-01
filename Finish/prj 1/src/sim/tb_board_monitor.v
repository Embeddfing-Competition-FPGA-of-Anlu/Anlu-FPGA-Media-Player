`timescale 1ns/1ps
// board_monitor 功能仿真：动态演示、自动切 LIVE、CRC/序号、无声警告自动出现/消失、超时、批次包、校时
module tb_board_monitor;
    localparam integer HZ=4000;           // 仿真里 1 秒 = 4000 拍
    reg clk=0, rst_n=0;
    always #5 clk=~clk;
    reg [7:0] rx_data=0; reg rx_valid=0;
    wire handled, ok, query, valid, ctx, stale, warn, failing, demo;
    wire [2:0] reasons, tool; wire [7:0] age;
    wire [15:0] seq, mv, ma, temp, lot, wafer, defects, yield, up, pkts, errs;
    wire [47:0] dt;
    wire [15:0] vmin, vmax, imax, tmax; wire [1:0] lks; wire [7:0] lkq;
    wire [115:0] resp; wire [6:0] resp_len;
    board_monitor #(.LOCAL_DEMO(1), .CLK_HZ(HZ), .TIMEOUT_S(5), .WARN_S(2)) dut (
        .clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_valid(rx_valid), .manual_alarm(1'b0),
        .picture_valid(1'b1), .detail_mode(1'b0), .event_overflows(16'd0),
        .link_eol(1'b0), .lim_idx(8'hd6), .lim_load(1'b0),
        .handled(handled), .command_ok(ok), .query(query), .response(resp), .response_len(resp_len),
        .valid(valid), .ctx_valid(ctx), .stale(stale), .warn(warn), .reasons(reasons), .failing(failing),
        .demo(demo), .tool_id(tool), .age(age), .sequence_id(seq), .millivolts(mv), .milliamps(ma),
        .temperature(temp), .lot(lot), .wafer(wafer), .defects(defects), .yield(yield),
        .uptime_min(up), .packets(pkts), .errors(errs), .datetime(dt),
        .vmin(vmin), .vmax(vmax), .imax(imax), .tmax(tmax), .link_state(lks), .link_q(lkq));

    integer errors=0;
    reg last_ok;
    task send_byte(input [7:0] b);
        begin @(negedge clk); rx_data=b; rx_valid=1; @(negedge clk); rx_valid=0; repeat(3) @(negedge clk); end
    endtask
    task send_line(input [8*40-1:0] s);
        integer n;
        begin
            for(n=39;n>=0;n=n-1) if(s[n*8 +: 8]!=0) send_byte(s[n*8 +: 8]);
            @(negedge clk); rx_data=8'h0d; rx_valid=1; #1 last_ok=handled && ok; @(negedge clk); rx_valid=0;
            repeat(3) @(negedge clk);
        end
    endtask
    task secs(input integer n); begin repeat(n*HZ) @(negedge clk); end endtask
    task expect(input cond, input [8*48-1:0] what);
        begin if(!cond) begin errors=errors+1; $display("FAIL: %0s", what); end else $display("ok  : %0s", what); end
    endtask

    integer k; reg [15:0] s0; reg seen_warn, seen_clear;
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        secs(1);
        expect(demo && valid && ctx && tool==1 && mv>=3200 && mv<=3400 && !warn, "power-up DEMO, data valid");
        s0=seq; secs(9);
        expect(seq!=s0 && wafer>=2, "DEMO data moves (seq, wafer)");
        // 演示周期第 40~60 秒温度越限：警告自动出现，之后自动消失
        seen_warn=0; seen_clear=0;
        for(k=0;k<60;k=k+1) begin
            secs(1);
            if(warn && reasons[1]) seen_warn=1;
            if(seen_warn && !warn) seen_clear=1;
        end
        expect(seen_warn, "DEMO excursion -> silent warning");
        expect(seen_clear, "warning clears by itself after recovery");

        send_line("!M1,17,3300,42,350");
        expect(last_ok && !demo && valid && !ctx && seq==17 && pkts==1, "first !M switches to LIVE");
        send_line("!M1,17,3300,42,350");
        expect(!last_ok && errs==1, "duplicate seq rejected, error counted");
        send_line("!M1,18,3300,42,350*34B1");
        expect(last_ok && seq==18, "good CRC accepted");
        send_line("!M1,19,3300,42,350*34B1");
        expect(!last_ok && seq==18, "bad CRC rejected");
        send_line("!L1234,3,2,992");
        expect(last_ok && ctx && lot==1234 && wafer==3 && defects==2 && yield==992, "!L lot context");
        // 越限 2 秒弹警告；恢复正常 2 秒自动消失（持续发包，不会超时）
        for(k=0;k<6;k=k+1) begin send_line({"!M1,",8'd48+k[7:0]+8'd2,"0,3300,42,650"}); secs(1); end
        expect(warn && reasons[1] && failing, "over-limit 2s -> warning (no latch)");
        for(k=0;k<5;k=k+1) begin send_line({"!M1,1",8'd48+k[7:0],"0,3300,42,350"}); secs(1); end
        expect(!warn && !failing, "back to normal -> warning gone, no ACK needed");
        secs(9);
        expect(stale && warn && reasons[2], "no packet > 5s -> STALE warning");
        send_line("!M1,200,3300,42,350");
        secs(3);
        expect(!stale && !warn, "packets resume -> STALE clears");
        send_line("!Z261231235958");
        expect(last_ok && dt==48'h261231235958, "!Z sets date/time");
        secs(2);
        expect(dt[47:24]==24'h270101 && dt[23:0]==24'h000000, "date/time rolls over year");
        send_line("!Z261332000000");
        expect(!last_ok, "bad date rejected");
        send_line("!Q");
        expect(last_ok && resp[115:100]==16'd1, "query response, B=tool");
        expect(vmin==3200 && vmax==3400 && imax==60 && tmax==600, "default limits");
        send_line("!T3100,3500,80,500");
        expect(last_ok && vmin==3100 && vmax==3500 && imax==80 && tmax==500, "!T sets limits");
        send_line("!T3500,3100,80,500");
        expect(!last_ok && vmin==3100, "!T vmin>=vmax rejected");
        send_line("!D1");
        secs(1);
        expect(last_ok && demo && valid && ctx, "!D1 -> back to DEMO");
        if(errors==0) $display("ALL PASS"); else $display("%0d FAILURES", errors);
        $finish;
    end
endmodule
