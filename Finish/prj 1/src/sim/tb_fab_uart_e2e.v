`timescale 1ns/1ps
// 端到端：USB 串口 + 板间串口 -> uart_cmd / link_uart -> board_monitor
//   板间一路回 OK/ER（link_uart），USB 一路回 OK/ER 和 !Q 的 ASCII 应答（uart_cmd）
module tb_fab_uart_e2e;
    localparam integer CPB=8;
    reg clk=0, rst_n=0;
    always #5 clk=~clk;
    reg rx=1, lrx=1;
    wire tx, ltx;
    wire [7:0] u_data, m_data; wire u_valid, m_valid, from_link, link_eol, m_handled, m_ok, m_query;
    wire [115:0] st_resp; wire [6:0] st_len;
    wire [15:0] vmin, tmax; wire [1:0] link_state; wire [7:0] link_q;
    uart_cmd #(.CLKS_PER_BIT(CPB)) u_cmd (
        .clk(clk), .rst_n(rst_n), .rx(rx), .tx(tx),
        .next_p(), .prev_p(), .auto_p(), .sel_p(), .ovl_p(), .save_p(), .sel_idx(),
        .p_wr(1'b0), .p_id(3'd0), .p_val(3'd0),
        .viz(), .filt(), .fx(),
        .cfg_load(1'b0), .cfg_viz(2'd0), .cfg_filt(1'b0), .cfg_fx(3'd0),
        .sub_vec(), .sub_len(), .sub_upd(),
        .monitor_rx_data(u_data), .monitor_rx_valid(u_valid),
        .monitor_handled(m_handled && !from_link), .monitor_ok(m_ok), .monitor_query(m_query),
        .monitor_response(st_resp), .monitor_event(1'b0), .monitor_response_len(st_len));
    link_uart #(.CLKS_PER_BIT(CPB)) u_link (
        .clk(clk), .rst_n(rst_n), .rx(lrx), .tx(ltx), .usb_data(u_data), .usb_valid(u_valid),
        .mon_data(m_data), .mon_valid(m_valid), .from_link(from_link), .link_eol(link_eol),
        .mon_handled(m_handled), .mon_ok(m_ok));
    board_monitor #(.LOCAL_DEMO(0), .CLK_HZ(100000000)) u_mon (
        .clk(clk), .rst_n(rst_n),
        .rx_data(m_data), .rx_valid(m_valid), .manual_alarm(1'b0), .picture_valid(1'b1), .detail_mode(1'b0),
        .event_overflows(16'h0007), .link_eol(link_eol), .lim_idx(8'hd6), .lim_load(1'b0),
        .handled(m_handled), .command_ok(m_ok), .query(m_query), .response(st_resp), .response_len(st_len),
        .valid(), .ctx_valid(), .stale(), .warn(), .reasons(), .failing(), .demo(), .tool_id(), .age(),
        .sequence_id(), .millivolts(), .milliamps(), .temperature(), .lot(), .wafer(), .defects(), .yield(),
        .uptime_min(), .packets(), .errors(), .datetime(),
        .vmin(vmin), .vmax(), .imax(), .tmax(tmax), .link_state(link_state), .link_q(link_q));

    // 发送：sel=0 USB，sel=1 板间
    task send_byte(input sel, input [7:0] b);
        integer n;
        begin
            if(sel) lrx=0; else rx=0; repeat(CPB) @(posedge clk);
            for(n=0;n<8;n=n+1) begin if(sel) lrx=b[n]; else rx=b[n]; repeat(CPB) @(posedge clk); end
            if(sel) lrx=1; else rx=1; repeat(CPB*2) @(posedge clk);
        end
    endtask
    task send_line(input sel, input [8*40-1:0] s);
        integer n;
        begin
            for(n=39;n>=0;n=n-1) if(s[n*8 +: 8]!=0) send_byte(sel, s[n*8 +: 8]);
            send_byte(sel, 8'h0d);
        end
    endtask
    // 两路接收端：把 TX 收到的字符拼成一行
    wire [7:0] ud, ld; wire uv, lv;
    uart_rx_byte #(.CLKS_PER_BIT(CPB)) u_mon_rx (.clk(clk), .rst_n(rst_n), .rx(tx), .data(ud), .valid(uv), .framing_error());
    uart_rx_byte #(.CLKS_PER_BIT(CPB)) l_mon_rx (.clk(clk), .rst_n(rst_n), .rx(ltx), .data(ld), .valid(lv), .framing_error());
    reg [8*64-1:0] ubuf, ulast, lbuf, llast;
    always @(posedge clk) begin
        if(uv) begin
            if(ud==8'h0a) begin ulast=ubuf; $display("USB  TX: %0s", ubuf); ubuf=0; end
            else if(ud!=8'h0d) ubuf={ubuf[8*63-1:0],ud};
        end
        if(lv) begin
            if(ld==8'h0a) begin llast=lbuf; $display("LINK TX: %0s", lbuf); lbuf=0; end
            else if(ld!=8'h0d) lbuf={lbuf[8*63-1:0],ld};
        end
    end
    task wait_reply; begin repeat(CPB*10*70) @(posedge clk); end endtask

    integer errors=0;
    task expect(input cond, input [8*48-1:0] what);
        begin if(!cond) begin errors=errors+1; $display("FAIL: %0s", what); end else $display("ok  : %0s", what); end
    endtask
    initial begin
        ubuf=0; lbuf=0; ulast=0; llast=0;
        repeat(10) @(posedge clk); rst_n=1; repeat(10) @(posedge clk);
        expect(link_state==0, "power-up link IDLE");
        send_line(1, "!M1,17,3300,42,350"); wait_reply;
        expect(llast[15:0]=="OK" && link_state==1, "link !M -> link OK");
        send_line(1, "!M1,17,3300,42,350"); wait_reply;
        expect(llast[15:0]=="ER", "link duplicate -> link ER");
        llast=0;
        send_line(1, "HELLO"); wait_reply;
        expect(llast[15:0]=="ER", "link garbage line -> link ER");
        send_line(1, "!T3100,3400,60,500"); wait_reply;
        expect(llast[15:0]=="OK" && vmin==3100 && tmax==500, "link !T sets limits");
        expect(link_q==87, "link quality 14/16 = 87%");
        send_line(0, "!Q"); wait_reply; wait_reply;
        expect(ulast[8*55-1:0]=="ST B=0001 V=1 F=0040 N=0011 U=0CE4 I=002A T=015E O=0007", "USB !Q status");
        send_line(0, "!X1"); wait_reply;
        expect(ulast[15:0]=="ER", "USB unknown ! command -> ER");
        if(errors==0) $display("E2E PASS"); else $display("%0d FAILURES", errors);
        $finish;
    end
endmodule
