`timescale 1ns/1ps
// 板间串口（杜邦线直连，115200 8N1，3.3V TTL，协议与 USB 串口完全相同）
//   看板板：P4 = link_rx（接发送板 TX），N4 = link_tx（接发送板 RX），GND 必须共地
//   USB 串口（电脑调试）和板间串口都可以给 board_monitor 发 !M/!L/!T/!Z/!Q/!D：
//     按“行”合并：哪一路先发来 '!' 就锁定这一行，直到 CR/LF，另一路这期间的字节不送 board_monitor
//     （另一路是普通字幕行时照常交给 uart_cmd，不受影响）；锁定超过 10ms 没收到行尾自动释放
//   板间这一路由本模块回 "OK\r\n"/"ER\r\n"（发送板据此统计通信成功率）；USB 那一路仍由 uart_cmd 回复
module link_uart #(
    parameter integer CLKS_PER_BIT = 868,
    parameter integer LOCK_TIMEOUT = 1000000,         // 10ms @100MHz
    parameter integer ENABLE_GIMBAL = 0,
    parameter integer GIMBAL_PERIOD = 50000000        // 500ms @100MHz
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output wire       tx,
    input  wire [7:0] usb_data,                     // uart_cmd 收到的 USB 字节
    input  wire       usb_valid,
    output wire [7:0] mon_data,                     // 送 board_monitor
    output wire       mon_valid,
    output wire       from_link,                    // 当前这一字节来自板间串口
    output wire       link_eol,                     // 板间串口一行结束（单拍）
    input  wire       mon_handled, mon_ok,          // board_monitor 对这一行的判定（与行尾同拍）
    input  wire [2:0] gimbal_tool,
    input  wire       gimbal_alarm,
    input  wire [2:0] gimbal_reason,
    output reg        reply_overflow
);
    wire [7:0] l_data;
    wire       l_valid;
    uart_rx_byte #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx), .data(l_data), .valid(l_valid), .framing_error());

    localparam [1:0] FREE=2'd0, USB=2'd1, LINK=2'd2;
    reg [1:0]  own;
    reg [19:0] lock_cnt;
    wire l_eol=l_valid && (l_data==8'h0d || l_data==8'h0a);
    wire u_eol=usb_valid && (usb_data==8'h0d || usb_data==8'h0a);
    // 板间优先：两路同拍开始新行时取板间
    wire l_take=l_valid && (own==LINK || (own==FREE && l_data=="!"));
    wire u_take=usb_valid && !l_take && (own==USB || (own==FREE && usb_data=="!"));
    reg flush;                                      // 超时：补一个 CR 清掉 board_monitor 的半行（提前一拍比较）
    assign mon_valid=l_take || u_take || flush;
    assign mon_data=flush ? 8'h0d : l_take ? l_data : usb_data;
    assign from_link=l_take;
    // 板间每个非空行都算一次（不是 ! 开头的乱码行不送 board_monitor，但同样回 ER、记为一次失败）
    reg lline;
    assign link_eol=l_eol && lline;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) lline<=0;
        else if(l_valid) lline<=!l_eol;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin own<=FREE; lock_cnt<=0; flush<=0; end
        else begin
            flush<=own!=FREE && !l_take && !u_take && lock_cnt==LOCK_TIMEOUT-1;
            if(flush || (l_take && l_eol) || (u_take && u_eol)) own<=FREE;
            else if(l_take) own<=LINK;
            else if(u_take) own<=USB;
            lock_cnt<=(own==FREE || l_take || u_take) ? 20'd0 : lock_cnt+1'b1;
        end

    // Whole-line arbitration: ACKs are queued while a G line is transmitting.
    reg [3:0] t_idx;
    reg t_act,t_ok,t_start,t_wait,busy_q,t_gimbal;
    reg [7:0] ack_fifo;
    reg [2:0] wr_ptr, rd_ptr;
    reg [3:0] ack_count;
    reg [31:0] g_count;
    reg [2:0] sent_tool, sent_reason;
    reg sent_alarm;
    reg [7:0] g_tool, g_alarm, g_reason, g_check;
    wire g_dirty = gimbal_tool!=sent_tool || gimbal_alarm!=sent_alarm || gimbal_reason!=sent_reason;
    wire take_ack = !t_act && ack_count!=0;
    wire put_ack = link_eol && (ack_count<8 || take_ack);
    function [7:0] hexchar; input [3:0] h; hexchar=h<10 ? 8'd48+{4'd0,h} : 8'd55+{4'd0,h}; endfunction
    wire t_busy;
    reg [7:0] t_byte;
    always @* begin
        if(!t_gimbal) t_byte=t_idx==0 ? (t_ok ? "O" : "E") : t_idx==1 ? (t_ok ? "K" : "R") : t_idx==2 ? 8'h0d : 8'h0a;
        else case(t_idx)
            0:t_byte="!"; 1:t_byte="G"; 2:t_byte=g_tool; 3:t_byte=g_alarm; 4:t_byte=g_reason;
            5:t_byte="*"; 6:t_byte=hexchar(g_check[7:4]); 7:t_byte=hexchar(g_check[3:0]);
            8:t_byte=13; default:t_byte=10;
        endcase
    end
    uart_tx_byte #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n), .start(t_start), .data(t_byte), .tx(tx), .busy(t_busy), .done());
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            t_idx<=0; t_act<=0; t_ok<=0; t_start<=0; t_wait<=0; busy_q<=0; t_gimbal<=0;
            ack_fifo<=0; wr_ptr<=0; rd_ptr<=0; ack_count<=0; reply_overflow<=0;
            g_count<=0; sent_tool<=7; sent_alarm<=0; sent_reason<=0;
            g_tool<=0; g_alarm<=0; g_reason<=0; g_check<=0;
        end
        else begin
            t_start<=0; busy_q<=t_busy;
            if(g_count<GIMBAL_PERIOD) g_count<=g_count+1'b1;
            if(link_eol && !put_ack) reply_overflow<=1;
            if(put_ack) begin ack_fifo[wr_ptr]<=l_take && mon_handled && mon_ok; wr_ptr<=wr_ptr+1'b1; end
            case({put_ack,take_ack})
                2'b10:ack_count<=ack_count+1'b1;
                2'b01:ack_count<=ack_count-1'b1;
                default:;
            endcase
            if(take_ack) begin
                t_act<=1; t_idx<=0; t_wait<=0; t_gimbal<=0; t_ok<=ack_fifo[rd_ptr]; rd_ptr<=rd_ptr+1'b1;
            end else if(!t_act && !link_eol && ENABLE_GIMBAL && (g_dirty || g_count>=GIMBAL_PERIOD)) begin
                t_act<=1; t_idx<=0; t_wait<=0; t_gimbal<=1; g_count<=0;
                sent_tool<=gimbal_tool; sent_alarm<=gimbal_alarm; sent_reason<=gimbal_reason;
                g_tool<=8'd48+{5'd0,gimbal_tool}; g_alarm<=8'd48+{7'd0,gimbal_alarm};
                g_reason<=8'd48+{5'd0,gimbal_reason};
                g_check<=8'h21 ^ 8'h47 ^ (8'd48+{5'd0,gimbal_tool}) ^
                         (8'd48+{7'd0,gimbal_alarm}) ^ (8'd48+{5'd0,gimbal_reason});
            end
            else if(t_act && !t_wait && !t_busy) begin t_start<=1; t_wait<=1; end
            else if(t_wait && busy_q && !t_busy) begin
                t_wait<=0; t_idx<=t_idx+1'b1;
                if((!t_gimbal && t_idx==3) || (t_gimbal && t_idx==9)) t_act<=0;
            end
        end
endmodule
