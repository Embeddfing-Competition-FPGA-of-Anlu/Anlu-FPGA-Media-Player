`timescale 1ns/1ps

// uart_cmd 仿真：行缓冲改为移位寄存器后，检查命令解析与字幕字符顺序（按 player_text_overlay 的取法还原）
module tb_uart_cmd;
    localparam integer CPB = 16;
    reg clk = 0, rst_n = 0, rx = 1;
    always #5 clk = ~clk;

    wire [1:0] viz;  wire filt;  wire [2:0] fx;
    wire [511:0] sub_vec;  wire [6:0] sub_len;
    wire next_p;
    uart_cmd #(.CLKS_PER_BIT(CPB)) dut (
        .clk(clk), .rst_n(rst_n), .rx(rx), .tx(),
        .next_p(next_p), .prev_p(), .auto_p(), .sel_p(), .ovl_p(), .save_p(), .sel_idx(),
        .p_wr(1'b0), .p_id(3'd0), .p_val(3'd0),
        .viz(viz), .filt(filt), .fx(fx),
        .cfg_load(1'b0), .cfg_viz(2'd0), .cfg_filt(1'b0), .cfg_fx(3'd0),
        .sub_vec(sub_vec), .sub_len(sub_len));

    task send_byte(input [7:0] b);
        integer i;
        begin
            rx = 0; repeat (CPB) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin rx = b[i]; repeat (CPB) @(posedge clk); end
            rx = 1; repeat (CPB * 2) @(posedge clk);
        end
    endtask
    task send_line(input [8*16-1:0] str, input integer n);
        integer i;
        begin
            for (i = n - 1; i >= 0; i = i - 1) send_byte(str[i*8 +: 8]);
            send_byte(8'h0d);
            repeat (CPB * 50) @(posedge clk);
        end
    endtask

    integer errs = 0, k;
    reg [5:0] idx;
    reg [8*16-1:0] got;
    initial begin
        #100 rst_n = 1;
        send_line("!V1", 3);  if (viz  !== 2'd1) begin errs = errs + 1; $display("V1 fail %0d", viz); end
        send_line("!F1", 3);  if (filt !== 1'b1) begin errs = errs + 1; $display("F1 fail %0d", filt); end
        send_line("!C3", 3);  if (fx   !== 3'd3) begin errs = errs + 1; $display("C3 fail %0d", fx); end
        send_line("!X9", 3);  // 非法命令：参数不应变化
        if (viz !== 2'd1 || filt !== 1'b1) begin errs = errs + 1; $display("bad cmd changed state"); end
        send_line("hello fpga", 10);
        got = 0;
        for (k = 0; k < sub_len; k = k + 1) begin
            idx = k[5:0] - sub_len[5:0];
            got = {got[8*15-1:0], sub_vec[idx*8 +: 8]};
        end
        $display("subtitle len=%0d text=\"%0s\"", sub_len, got);
        if (sub_len !== 7'd10 || got[79:0] !== "HELLO FPGA") errs = errs + 1;
        if (errs == 0) $display("UART PASS"); else $display("UART FAIL errs=%0d", errs);
        $finish;
    end
endmodule
