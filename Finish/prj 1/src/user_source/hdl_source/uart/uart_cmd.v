`timescale 1ns/1ps

// 扩展6：UART 下发字幕 + 远程控制（sd_card_clk 100MHz 域，115200 8N1）
//   协议：按行（CR 或 LF 结束，最多 64 字符），每行处理完回 "OK\r\n" 或 "ER\r\n"
//     普通文本          -> 设为滚动字幕（小写自动转大写；只支持 A-Z 0-9 和 :.,!?-_/ 空格）
//     !N / !P / !A      -> 下一张 / 上一张 / 自动轮播开关
//     !S1..!S4          -> 直接选中第 n 张
//     !V0..!V3          -> 可视化：关 / 频谱 / 波形 / 频谱+波形
//     !F0 / !F1         -> 白天 / 夜间模式
//     !M !L !T !Z !Q !D -> 交给 board_monitor（设备参数 / 批次 / 阈值 / 校时 / 查询 / 数据来源）
//     其它 ! 开头的行回 ER，不会被当成字幕
//     !C0..!C7          -> 色彩特效：关 / 灰度 / 怀旧 / 负片 / 色调分离 / 二值化 / 暖色 / 冷色
//     !O                -> 叠加层模式循环
//     !W                -> 立即保存参数到 Flash
//   字幕整行校验完才替换（行缓冲与显示缓冲分开），收到一半的行不会出现在屏幕上
//   按键菜单通过 p_wr/p_id/p_val 写同一组寄存器（串口与菜单等价）
module uart_cmd #(
    parameter integer CLKS_PER_BIT = 868            // 100MHz / 115200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output wire        tx,
    output reg         next_p, prev_p, auto_p, sel_p, ovl_p, save_p,
    output reg  [1:0]  sel_idx,
    input  wire        p_wr,                         // 菜单参数写
    input  wire [2:0]  p_id,                         // 0 可视化 1 白天/夜间 2 特效
    input  wire [2:0]  p_val,
    output reg  [1:0]  viz,
    output reg         filt,                         // 0 白天 1 夜间
    output reg  [2:0]  fx,
    input  wire        cfg_load,                     // Flash 上电恢复
    input  wire [1:0]  cfg_viz,
    input  wire        cfg_filt,
    input  wire [2:0]  cfg_fx,
    output reg  [511:0] sub_vec,                     // 字符顺序见下方行缓冲说明
    output reg  [6:0]  sub_len,
    output wire [7:0] monitor_rx_data,
    output wire monitor_rx_valid,
    input wire monitor_handled, monitor_ok, monitor_query,
    input wire [115:0] monitor_response,         // 原始数值，按 kind 填进 ST / EV 模板
    input wire monitor_event,                    // 1：事件应答 EV，0：状态应答 ST
    input wire [6:0] monitor_response_len,
    output reg         sub_upd                       // 单拍：字幕刚更新（跨域用）
);
    wire [7:0] rx_data;
    wire       rx_valid;
    uart_rx_byte #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx), .data(rx_data), .valid(rx_valid), .framing_error());

    assign monitor_rx_data=rx_data;
    assign monitor_rx_valid=rx_valid;
    reg line_overflow;
    reg bang;                                        // 本行以 ! 开头
    // Byte RAM replaces the 512-flop line shift register. Only the last
    // three bytes are needed immediately by the legacy command decoder.
    reg [23:0] lb;
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [6:0] line_ram[0:63];
    reg [6:0] line_read;
    reg [5:0] copy_index;
    reg [6:0] copy_length;
    reg [1:0] copy_state;
    
    reg [6:0]   len;
    wire [7:0]  uc = (rx_data >= "a" && rx_data <= "z") ? rx_data - 8'd32 : rx_data;
    wire        eol = rx_valid && (rx_data == 8'h0d || rx_data == 8'h0a);
    always @(posedge clk) begin
        if(rx_valid && !eol && rx_data>=8'h20 && rx_data<8'h7f && len<64)
            line_ram[len[5:0]]<=uc[6:0];
        line_read<=line_ram[copy_index];
    end
    // RAM 按接收顺序存储；复制后字幕仍使用原高位移入格式。
    // 命令只有 2~3 个字符，直接取最高的三个字节
    wire [7:0]  b0 = lb[23:16];                  // 最后一个字符
    wire [7:0]  b1 = lb[15:8];
    wire [7:0]  b2 = lb[7:0];
    wire [7:0]  c0 = (len == 7'd3) ? b2 : b1;      // 首字符（应为 "!"）
    wire [7:0]  c1 = (len == 7'd3) ? b1 : b0;
    wire [7:0]  c2 = b0;
    wire [3:0]  d  = c2[3:0];
    wire        dig = (c2 >= "0") && (c2 <= "9") && (len == 7'd3);
    wire        one = (len == 7'd2);

    // 应答发送
    reg [6:0] t_idx, t_length;
    // Freeze only numeric nibbles; fixed protocol labels are rebuilt at TX.
    reg [115:0] t_payload;
    reg [1:0] t_kind;
    function [7:0] hexascii;
        input [3:0] n;
        begin hexascii=n<10 ? 8'd48+{4'd0,n} : 8'd55+{4'd0,n}; end
    endfunction
    reg        t_act, t_ok, t_start, t_wait, busy_q;
    wire       t_busy;
    // 应答模板放 BRAM：地址 {kind, 字节序号}，8'h01 表示"此处输出下一个十六进制半字节"
    //   kind 0 OK  1 ER  2 ST 状态  3 EV 事件；半字节按 t_payload 从高到低依次取
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [7:0] tx_rom[0:255];
    initial begin
        tx_rom[0]=8'h4f;
        tx_rom[1]=8'h4b;
        tx_rom[2]=8'h0d;
        tx_rom[3]=8'h0a;
        tx_rom[4]=8'h0a;
        tx_rom[5]=8'h0a;
        tx_rom[6]=8'h0a;
        tx_rom[7]=8'h0a;
        tx_rom[8]=8'h0a;
        tx_rom[9]=8'h0a;
        tx_rom[10]=8'h0a;
        tx_rom[11]=8'h0a;
        tx_rom[12]=8'h0a;
        tx_rom[13]=8'h0a;
        tx_rom[14]=8'h0a;
        tx_rom[15]=8'h0a;
        tx_rom[16]=8'h0a;
        tx_rom[17]=8'h0a;
        tx_rom[18]=8'h0a;
        tx_rom[19]=8'h0a;
        tx_rom[20]=8'h0a;
        tx_rom[21]=8'h0a;
        tx_rom[22]=8'h0a;
        tx_rom[23]=8'h0a;
        tx_rom[24]=8'h0a;
        tx_rom[25]=8'h0a;
        tx_rom[26]=8'h0a;
        tx_rom[27]=8'h0a;
        tx_rom[28]=8'h0a;
        tx_rom[29]=8'h0a;
        tx_rom[30]=8'h0a;
        tx_rom[31]=8'h0a;
        tx_rom[32]=8'h0a;
        tx_rom[33]=8'h0a;
        tx_rom[34]=8'h0a;
        tx_rom[35]=8'h0a;
        tx_rom[36]=8'h0a;
        tx_rom[37]=8'h0a;
        tx_rom[38]=8'h0a;
        tx_rom[39]=8'h0a;
        tx_rom[40]=8'h0a;
        tx_rom[41]=8'h0a;
        tx_rom[42]=8'h0a;
        tx_rom[43]=8'h0a;
        tx_rom[44]=8'h0a;
        tx_rom[45]=8'h0a;
        tx_rom[46]=8'h0a;
        tx_rom[47]=8'h0a;
        tx_rom[48]=8'h0a;
        tx_rom[49]=8'h0a;
        tx_rom[50]=8'h0a;
        tx_rom[51]=8'h0a;
        tx_rom[52]=8'h0a;
        tx_rom[53]=8'h0a;
        tx_rom[54]=8'h0a;
        tx_rom[55]=8'h0a;
        tx_rom[56]=8'h0a;
        tx_rom[57]=8'h0a;
        tx_rom[58]=8'h0a;
        tx_rom[59]=8'h0a;
        tx_rom[60]=8'h0a;
        tx_rom[61]=8'h0a;
        tx_rom[62]=8'h0a;
        tx_rom[63]=8'h0a;
        tx_rom[64]=8'h45;
        tx_rom[65]=8'h52;
        tx_rom[66]=8'h0d;
        tx_rom[67]=8'h0a;
        tx_rom[68]=8'h0a;
        tx_rom[69]=8'h0a;
        tx_rom[70]=8'h0a;
        tx_rom[71]=8'h0a;
        tx_rom[72]=8'h0a;
        tx_rom[73]=8'h0a;
        tx_rom[74]=8'h0a;
        tx_rom[75]=8'h0a;
        tx_rom[76]=8'h0a;
        tx_rom[77]=8'h0a;
        tx_rom[78]=8'h0a;
        tx_rom[79]=8'h0a;
        tx_rom[80]=8'h0a;
        tx_rom[81]=8'h0a;
        tx_rom[82]=8'h0a;
        tx_rom[83]=8'h0a;
        tx_rom[84]=8'h0a;
        tx_rom[85]=8'h0a;
        tx_rom[86]=8'h0a;
        tx_rom[87]=8'h0a;
        tx_rom[88]=8'h0a;
        tx_rom[89]=8'h0a;
        tx_rom[90]=8'h0a;
        tx_rom[91]=8'h0a;
        tx_rom[92]=8'h0a;
        tx_rom[93]=8'h0a;
        tx_rom[94]=8'h0a;
        tx_rom[95]=8'h0a;
        tx_rom[96]=8'h0a;
        tx_rom[97]=8'h0a;
        tx_rom[98]=8'h0a;
        tx_rom[99]=8'h0a;
        tx_rom[100]=8'h0a;
        tx_rom[101]=8'h0a;
        tx_rom[102]=8'h0a;
        tx_rom[103]=8'h0a;
        tx_rom[104]=8'h0a;
        tx_rom[105]=8'h0a;
        tx_rom[106]=8'h0a;
        tx_rom[107]=8'h0a;
        tx_rom[108]=8'h0a;
        tx_rom[109]=8'h0a;
        tx_rom[110]=8'h0a;
        tx_rom[111]=8'h0a;
        tx_rom[112]=8'h0a;
        tx_rom[113]=8'h0a;
        tx_rom[114]=8'h0a;
        tx_rom[115]=8'h0a;
        tx_rom[116]=8'h0a;
        tx_rom[117]=8'h0a;
        tx_rom[118]=8'h0a;
        tx_rom[119]=8'h0a;
        tx_rom[120]=8'h0a;
        tx_rom[121]=8'h0a;
        tx_rom[122]=8'h0a;
        tx_rom[123]=8'h0a;
        tx_rom[124]=8'h0a;
        tx_rom[125]=8'h0a;
        tx_rom[126]=8'h0a;
        tx_rom[127]=8'h0a;
        tx_rom[128]=8'h53;
        tx_rom[129]=8'h54;
        tx_rom[130]=8'h20;
        tx_rom[131]=8'h42;
        tx_rom[132]=8'h3d;
        tx_rom[133]=8'h01;
        tx_rom[134]=8'h01;
        tx_rom[135]=8'h01;
        tx_rom[136]=8'h01;
        tx_rom[137]=8'h20;
        tx_rom[138]=8'h56;
        tx_rom[139]=8'h3d;
        tx_rom[140]=8'h01;
        tx_rom[141]=8'h20;
        tx_rom[142]=8'h46;
        tx_rom[143]=8'h3d;
        tx_rom[144]=8'h01;
        tx_rom[145]=8'h01;
        tx_rom[146]=8'h01;
        tx_rom[147]=8'h01;
        tx_rom[148]=8'h20;
        tx_rom[149]=8'h4e;
        tx_rom[150]=8'h3d;
        tx_rom[151]=8'h01;
        tx_rom[152]=8'h01;
        tx_rom[153]=8'h01;
        tx_rom[154]=8'h01;
        tx_rom[155]=8'h20;
        tx_rom[156]=8'h55;
        tx_rom[157]=8'h3d;
        tx_rom[158]=8'h01;
        tx_rom[159]=8'h01;
        tx_rom[160]=8'h01;
        tx_rom[161]=8'h01;
        tx_rom[162]=8'h20;
        tx_rom[163]=8'h49;
        tx_rom[164]=8'h3d;
        tx_rom[165]=8'h01;
        tx_rom[166]=8'h01;
        tx_rom[167]=8'h01;
        tx_rom[168]=8'h01;
        tx_rom[169]=8'h20;
        tx_rom[170]=8'h54;
        tx_rom[171]=8'h3d;
        tx_rom[172]=8'h01;
        tx_rom[173]=8'h01;
        tx_rom[174]=8'h01;
        tx_rom[175]=8'h01;
        tx_rom[176]=8'h20;
        tx_rom[177]=8'h4f;
        tx_rom[178]=8'h3d;
        tx_rom[179]=8'h01;
        tx_rom[180]=8'h01;
        tx_rom[181]=8'h01;
        tx_rom[182]=8'h01;
        tx_rom[183]=8'h0d;
        tx_rom[184]=8'h0a;
        tx_rom[185]=8'h0a;
        tx_rom[186]=8'h0a;
        tx_rom[187]=8'h0a;
        tx_rom[188]=8'h0a;
        tx_rom[189]=8'h0a;
        tx_rom[190]=8'h0a;
        tx_rom[191]=8'h0a;
        tx_rom[192]=8'h45;
        tx_rom[193]=8'h56;
        tx_rom[194]=8'h20;
        tx_rom[195]=8'h4e;
        tx_rom[196]=8'h3d;
        tx_rom[197]=8'h01;
        tx_rom[198]=8'h01;
        tx_rom[199]=8'h01;
        tx_rom[200]=8'h01;
        tx_rom[201]=8'h20;
        tx_rom[202]=8'h4d;
        tx_rom[203]=8'h3d;
        tx_rom[204]=8'h01;
        tx_rom[205]=8'h01;
        tx_rom[206]=8'h01;
        tx_rom[207]=8'h01;
        tx_rom[208]=8'h20;
        tx_rom[209]=8'h42;
        tx_rom[210]=8'h3d;
        tx_rom[211]=8'h01;
        tx_rom[212]=8'h01;
        tx_rom[213]=8'h01;
        tx_rom[214]=8'h01;
        tx_rom[215]=8'h20;
        tx_rom[216]=8'h53;
        tx_rom[217]=8'h3d;
        tx_rom[218]=8'h01;
        tx_rom[219]=8'h01;
        tx_rom[220]=8'h01;
        tx_rom[221]=8'h01;
        tx_rom[222]=8'h20;
        tx_rom[223]=8'h58;
        tx_rom[224]=8'h3d;
        tx_rom[225]=8'h01;
        tx_rom[226]=8'h01;
        tx_rom[227]=8'h01;
        tx_rom[228]=8'h01;
        tx_rom[229]=8'h0d;
        tx_rom[230]=8'h0a;
        tx_rom[231]=8'h0a;
        tx_rom[232]=8'h0a;
        tx_rom[233]=8'h0a;
        tx_rom[234]=8'h0a;
        tx_rom[235]=8'h0a;
        tx_rom[236]=8'h0a;
        tx_rom[237]=8'h0a;
        tx_rom[238]=8'h0a;
        tx_rom[239]=8'h0a;
        tx_rom[240]=8'h0a;
        tx_rom[241]=8'h0a;
        tx_rom[242]=8'h0a;
        tx_rom[243]=8'h0a;
        tx_rom[244]=8'h0a;
        tx_rom[245]=8'h0a;
        tx_rom[246]=8'h0a;
        tx_rom[247]=8'h0a;
        tx_rom[248]=8'h0a;
        tx_rom[249]=8'h0a;
        tx_rom[250]=8'h0a;
        tx_rom[251]=8'h0a;
        tx_rom[252]=8'h0a;
        tx_rom[253]=8'h0a;
        tx_rom[254]=8'h0a;
        tx_rom[255]=8'h0a;
    end
    reg [7:0] rom_q;
    reg [4:0] t_nib;
    always @(posedge clk) rom_q<=tx_rom[{t_kind,t_idx[5:0]}];
    wire [3:0] nib=t_payload[115-{t_nib,2'b00} -: 4];
    wire [7:0] t_byte=rom_q==8'h01 ? hexascii(nib) : rom_q;
    uart_tx_byte #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n), .start(t_start), .data(t_byte), .tx(tx), .busy(t_busy), .done());

    task reply;
        input ok;
        begin
            t_ok<=ok; t_act<=1; t_idx<=0; t_nib<=0; t_wait<=0; t_length<=4;
            t_kind<=ok ? 0 : 1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lb<=0; len<=0; copy_index<=0; copy_length<=0; copy_state<=0; line_overflow<=0; t_payload<=0; t_kind<=0; t_length<=4;
            {next_p, prev_p, auto_p, sel_p, ovl_p, save_p} <= 6'd0;  sel_idx <= 2'd0;
            viz <= 2'd3;  filt <= 1'b0;  fx <= 3'd0;  bang <= 1'b0;
            sub_vec <= 512'd0;  sub_len <= 7'd0;  sub_upd <= 1'b0;
            t_idx <= 7'd0;  t_nib <= 5'd0;  t_act <= 1'b0;  t_ok <= 1'b0;  t_start <= 1'b0;  t_wait <= 1'b0;  busy_q <= 1'b0;
        end else begin
            {next_p, prev_p, auto_p, sel_p, ovl_p, save_p} <= 6'd0;
            t_start <= 1'b0;
            sub_upd <= 1'b0;
            // Read one byte every two clocks, then publish one complete update.
            if(copy_state==1) copy_state<=2;
            else if(copy_state==2) begin
                sub_vec<={1'b0,line_read,sub_vec[511:8]};
                if({1'b0,copy_index}+7'd1==copy_length) begin
                    sub_len<=copy_length;sub_upd<=1;copy_state<=0;
                end else begin copy_index<=copy_index+1'b1;copy_state<=1;end
            end
            if (cfg_load) begin
                viz <= cfg_viz;  filt <= cfg_filt;  fx <= cfg_fx;
            end
            if (p_wr) begin
                case (p_id)
                    3'd0: viz  <= p_val[1:0];
                    3'd1: filt <= p_val[0];
                    3'd2: fx   <= p_val;
                    default: ;
                endcase
            end

            if (eol) begin
                len<=0; line_overflow<=0;
                if (len != 7'd0) begin
                    if(line_overflow) reply(1'b0);
                    else if(monitor_handled) begin
                        reply(monitor_ok);
                        if(monitor_ok && monitor_query) begin
                            t_kind<=monitor_event ? 3 : 2;
                            t_payload<=monitor_response;
                            t_length<=monitor_response_len;
                        end
                    end else if (c0 == "!" && (len == 7'd2 || len == 7'd3)) begin
                        reply(1'b1);
                        if      (c1 == "N" && one) next_p <= 1'b1;
                        else if (c1 == "P" && one) prev_p <= 1'b1;
                        else if (c1 == "A" && one) auto_p <= 1'b1;
                        else if (c1 == "O" && one) ovl_p  <= 1'b1;
                        else if (c1 == "W" && one) save_p <= 1'b1;
                        else if (c1 == "S" && dig && d >= 4'd1 && d <= 4'd4) begin sel_p <= 1'b1; sel_idx <= d[1:0] - 2'd1; end
                        else if (c1 == "V" && dig && d <= 4'd3) viz  <= d[1:0];
                        else if (c1 == "F" && dig && d <= 4'd1) filt <= d[0];
                        else if (c1 == "C" && dig && d <= 4'd7) fx   <= d[2:0];
                        else reply(1'b0);
                    end else begin
                        if (bang) reply(1'b0);   // 未知或被板间串口占用的 ! 命令
                        else begin copy_index<=0;copy_length<=len;copy_state<=1;reply(1'b1); end
                    end
                end
            end else if (rx_valid && rx_data >= 8'h20 && rx_data < 8'h7f && len < 7'd64) begin
                lb <= {uc, lb[23:8]};
                if (len == 7'd0) bang <= (uc == "!");
                len <= len + 7'd1;
            end else if(rx_valid && len==64) begin
                line_overflow<=1;
            end

            // 应答逐字节发送；t_wait=已发起一字节、等它发完（busy 下降沿）。
    // rom_q 在 t_idx 更新后一拍有效，而 t_start 也晚一拍发出，uart_tx_byte 锁存时数据已就绪
            busy_q <= t_busy;
            if (t_act && !t_wait && !t_busy) begin
                t_start <= 1'b1;  t_wait <= 1'b1;
            end else if (t_wait && busy_q && !t_busy) begin
                t_wait <= 1'b0;
                t_idx<=t_idx+1'b1;
                if(rom_q==8'h01) t_nib<=t_nib+1'b1;
                if(t_idx==t_length-1'b1) t_act <= 1'b0;
            end
        end
    end
endmodule
