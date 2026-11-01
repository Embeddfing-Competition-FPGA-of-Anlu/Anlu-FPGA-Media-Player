`timescale 1ns/1ps
// 板对板联合仿真：发送板 sender_top --串口--> 看板 link_uart + board_monitor --OK/ER--> 发送板
//   时钟都按 1MHz 仿真（波特率分频 8），发送板 1 “秒” = 20ms
module tb_board2board;
    reg clk=0; always #500 clk=~clk;              // 1MHz
    reg rst_n=0;
    reg [7:0] sw=8'b0000_0011;                    // AUTO + CRC
    wire [3:0] row; reg [3:0] col_r;
    integer kr=-1, kc=-1;                         // 当前按下的键
    always @* begin col_r=4'b1111; if(kr>=0 && row[kr]==1'b0) col_r[kc]=1'b0; end
    wire s2b, b2s;
    wire [7:0] led; wire [3:0] seg_sel; wire [7:0] seg_led;
    sender_top #(.CLK_HZ(1000000), .BAUD(115200), .SEC_MS(20)) u_snd (
        .clk(clk), .row(row), .col(col_r), .seg_sel(seg_sel), .seg_led(seg_led),
        .sw(sw), .led(led), .link_tx(s2b), .link_rx(b2s),
        .cam_pclk(1'b0), .cam_vsync(1'b0), .cam_href(1'b0), .cam_d(8'd0),
        .cam_scl(), .cam_sda(), .cam_rst_n(), .cam_pwdn(), .vid_tx(), .servo_pan(), .servo_tilt());

    wire [7:0] m_data; wire m_valid, from_link, link_eol, m_handled, m_ok, m_query;
    wire valid, ctx, demo, failing; wire [2:0] tool; wire [15:0] seq, mv, ma, temp, lot, wafer, vmin, tmax, pkts;
    wire [1:0] link_state; wire [7:0] link_q;
    link_uart #(.CLKS_PER_BIT(8)) u_link (
        .clk(clk), .rst_n(rst_n), .rx(s2b), .tx(b2s), .usb_data(8'd0), .usb_valid(1'b0),
        .mon_data(m_data), .mon_valid(m_valid), .from_link(from_link), .link_eol(link_eol),
        .mon_handled(m_handled), .mon_ok(m_ok),
        .gimbal_tool(3'd0), .gimbal_alarm(1'b0), .gimbal_reason(3'd0), .reply_overflow());
    board_monitor #(.LOCAL_DEMO(1), .CLK_HZ(1000000)) u_mon (
        .clk(clk), .rst_n(rst_n), .rx_data(m_data), .rx_valid(m_valid), .manual_alarm(1'b0),
        .picture_valid(1'b1), .detail_mode(1'b0), .event_overflows(16'd0),
        .link_eol(link_eol), .lim_idx(8'hd6), .lim_load(1'b0),
        .handled(m_handled), .command_ok(m_ok), .query(m_query), .response(), .response_len(),
        .valid(valid), .ctx_valid(ctx), .stale(), .warn(), .reasons(), .failing(failing), .demo(demo),
        .tool_id(tool), .age(), .sequence_id(seq), .millivolts(mv), .milliamps(ma), .temperature(temp),
        .lot(lot), .wafer(wafer), .defects(), .yield(), .uptime_min(), .packets(pkts), .errors(), .datetime(),
        .vmin(vmin), .vmax(), .imax(), .tmax(tmax), .link_state(link_state), .link_q(link_q));

    integer errors=0;
    task expect(input cond, input [8*48-1:0] what);
        begin if(!cond) begin errors=errors+1; $display("FAIL: %0s", what); end else $display("ok  : %0s", what); end
    endtask
    task ms(input integer n); begin repeat(n*1000) @(posedge clk); end endtask
    // k = 操作者视角键号；硬件扫描号 = 15 - k
    task press(input integer k); begin kr=(15-k)/4; kc=(15-k)%4; ms(70); kr=-1; kc=-1; ms(70); end endtask
    // 打印看板收到的每一行
    reg [8*48-1:0] lbuf;
    always @(posedge clk) if(m_valid) begin
        if(m_data==8'h0d) begin $display("  board <- %0s   %0s", lbuf, (m_handled && m_ok) ? "OK" : "ER"); lbuf=0; end
        else lbuf={lbuf[8*47-1:0],m_data};
    end

    initial begin
        lbuf=0;
        #2000 rst_n=1;
        ms(80);
        expect(!demo && valid && tool==1 && mv==3300 && ma==42 && temp==350, "first packets: !D0 then !M -> LIVE");
        expect(ctx && lot==1234 && wafer==1, "!L lot context received");
        expect(u_snd.ok_cnt>=3 && u_snd.er_cnt==0 && link_state==1, "sender counts OK replies, link OK");
        press(0);                                 // V+
        ms(30);
        expect(mv==3310, "key V+ -> board shows 3310 mV");
        press(7);                                 // 故障注入：温度 +30C
        ms(30);
        expect(temp==650 && failing, "fault key -> board over-limit");
        press(7);
        press(14);                                // 阈值 VMIN -50
        press(15);                                // 发阈值
        ms(20);
        expect(vmin==3150 && tmax==600, "threshold keys -> board vmin 3150");
        press(8);                                 // 下一片
        ms(20);
        expect(wafer==2, "next wafer key -> board wafer 2");
        expect(u_snd.er_cnt==0 && link_q==100, "no errors, link quality 100%");
        if(errors==0) $display("B2B PASS"); else $display("%0d FAILURES", errors);
        $finish;
    end
endmodule
