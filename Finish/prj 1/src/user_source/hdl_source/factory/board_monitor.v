`timescale 1ns/1ps
// 晶圆厂看板数据源（sd_card_clk 域）
//   DEMO：上电默认。内部按时间生成演示数据：电压/电流/温度漂移+噪声，每 8 秒推进一片晶圆、
//         25 片换批，缺陷随机累计、良率随之变化；每 64 秒腔体温度越限一次，演示警告和自动恢复
//   LIVE：收到第一个合法 !M 或 !L 包自动切换；停发超过 TIMEOUT_S 秒显示 STALE；
//         !D1 回到 DEMO，!D0 进入 LIVE 等数据
//   串口：!Mtool,seq,mV,mA,temp10[*CCCC]      设备参数（tool 1~4）
//         !Llot,wafer,defects,yield10[*CCCC]   批次上下文（wafer 1~25，yield 0~1000 = 0~100.0%）
//         !Tvmin,vmax,imax,tmax10[*CCCC]       下发超限阈值（mV, mV, mA, 0.1C），与菜单档位以最后一次为准
//         !Zyymmddhhmmss                        校准日期时间（板上没有 RTC，上电从默认时间开始走）
//         !Q 查询   !D0 / !D1 切换数据来源
//   CRC 可选：CRC-16-CCITT-FALSE（poly 1021, init FFFF），从 '!' 到 '*' 前一字符
//   警告（无声）：越限或超时持续 WARN_S 秒 -> warn=1，屏幕弹警告横幅；恢复正常 WARN_S 秒后自动消失。
//   报警声音只由 SW3 手动报警触发（顶层），这里不锁存、不需要确认
module board_monitor #(
    parameter integer LOCAL_DEMO=1,       // 上电默认模式：1 DEMO，0 LIVE
    parameter integer CLK_HZ=100000000,
    parameter integer TIMEOUT_S=5,
    parameter integer WARN_S=2
)(
    input wire clk,rst_n,
    input wire [7:0] rx_data,
    input wire rx_valid,manual_alarm,picture_valid,detail_mode,
    input wire [15:0] event_overflows,
    input wire link_eol,                  // 板间串口一行结束（统计通信质量）
    input wire [7:0] lim_idx,             // 菜单阈值档位 {tmax,imax,vmax,vmin} 各 2 位
    input wire lim_load,                  // 菜单改档位 / Flash 读回
    output wire handled,command_ok,query,
    output wire [115:0] response,         // !Q 应答原始数值，ASCII 由 uart_cmd 发送时生成
    output wire [6:0] response_len,
    output reg valid,                     // 设备参数有效
    output reg ctx_valid,                 // 批次上下文有效
    output wire stale,
    output reg warn,
    output reg [2:0] reasons,             // {超时, 温度, 电压/电流}
    output wire failing,
    output reg demo,
    output reg [2:0] tool_id,
    output reg [7:0] age,
    output reg [15:0] sequence_id,millivolts,milliamps,temperature,
    output reg [15:0] lot,wafer,defects,yield,
    output reg [15:0] uptime_min,packets,errors,
    output wire [47:0] datetime,          // BCD {yy,mo,dd,hh,mi,ss}
    output reg [15:0] vmin,vmax,imax,tmax, // 当前生效阈值
    output wire [1:0] link_state,         // 板间链路：0 未连接 1 正常 2 中断
    output reg [7:0] link_q               // 最近 16 行板间通信成功率 0~100%
);
    // ---------------- 时基：1/4 秒和 1 秒 ----------------
    localparam integer QCYC=CLK_HZ/4;
    reg [27:0] qcnt;
    reg [1:0] qsub;
    wire qtick=qcnt==QCYC-1;
    wire stick=qtick && qsub==2'd3;

    // ---------------- 串口行解析 ----------------
    wire eol=rx_valid && (rx_data==13 || rx_data==10);
    wire digit=rx_data>="0" && rx_data<="9";
    wire hexdig=digit || (rx_data>="A" && rx_data<="F") || (rx_data>="a" && rx_data<="f");
    wire [3:0] hexval=digit ? rx_data[3:0] : rx_data[3:0]+4'd9;
    reg [6:0] length;
    reg prefix;
    reg [7:0] op, last;
    reg malformed;
    reg [2:0] field;
    reg [3:0] digits;
    reg [19:0] accumulator;
    reg [15:0] fields [0:3];
    reg star;
    reg [2:0] crc_digits;
    reg [15:0] crc, crc_rx;
    reg [47:0] zbuf;
    reg [3:0] zdig;
    function [15:0] crc_byte;
        input [15:0] c;
        input [7:0] b;
        integer j;
        reg [15:0] t;
        begin
            t=c ^ {b,8'd0};
            for(j=0;j<8;j=j+1) t=t[15] ? ({t[14:0],1'b0} ^ 16'h1021) : {t[14:0],1'b0};
            crc_byte=t;
        end
    endfunction
    wire numop=op=="M" || op=="L" || op=="T";
    wire [2:0] nf=op=="M" ? 3'd4 : 3'd3;  // 最后一个字段之前的逗号数
    wire known=numop || op=="Z" || op=="Q" || op=="D";
    assign handled=eol && prefix && known;
    assign query=op=="Q";
    assign response_len=57;

    // 阈值档位表（菜单 V MIN / V MAX / I MAX / T MAX 各 4 档），上电默认 3200mV / 3400mV / 60mA / 60.0C
    function [15:0] tab_vmin; input [1:0] i; tab_vmin=i==0 ? 16'd3000 : i==1 ? 16'd3100 : i==2 ? 16'd3200 : 16'd3250; endfunction
    function [15:0] tab_vmax; input [1:0] i; tab_vmax=i==0 ? 16'd3350 : i==1 ? 16'd3400 : i==2 ? 16'd3450 : 16'd3500; endfunction
    function [15:0] tab_imax; input [1:0] i; tab_imax=i==0 ? 16'd50 : i==1 ? 16'd60 : i==2 ? 16'd80 : 16'd100; endfunction
    function [15:0] tab_tmax; input [1:0] i; tab_tmax=i==0 ? 16'd450 : i==1 ? 16'd500 : i==2 ? 16'd550 : 16'd600; endfunction
    wire [63:0] lim={vmin,vmax,imax,tmax};

    // 前几个字段在最后一个字段开始前已经完整，提前一拍算好，避免压在 EOL 控制路径上
    reg seq_armed;
    wire [15:0] seq_diff=fields[1]-sequence_id;
    reg m_ok,l_ok,t_ok;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin m_ok<=0; l_ok<=0; t_ok<=0; end
        else begin
            m_ok<=fields[0]>=1 && fields[0]<=4 && fields[2]<=20000 && fields[3]<=5000 &&
                (demo || !seq_armed || fields[0][2:0]!=tool_id || (seq_diff!=0 && !seq_diff[15]));
            l_ok<=fields[1]>=1 && fields[1]<=25 && fields[2]<=9999;
            t_ok<=fields[0]<fields[1] && fields[1]<=20000 && fields[2]!=0 && fields[2]<=5000;
        end
    wire [7:0] zmo=zbuf[39:32],zdd=zbuf[31:24],zhh=zbuf[23:16],zmi=zbuf[15:8],zss=zbuf[7:0];
    wire z_ok_c=length==14 && zdig==12 &&
        (zmo[7:4]==0 ? zmo[3:0]!=0 : (zmo[7:4]==1 && zmo[3:0]<=2)) &&
        (zdd[7:4]==0 ? zdd[3:0]!=0 : (zdd[7:4]<=2 || (zdd[7:4]==3 && zdd[3:0]<=1))) &&
        (zhh[7:4]<=1 || (zhh[7:4]==2 && zhh[3:0]<=3)) && zmi[7:4]<=5 && zss[7:4]<=5;
    // CRC、末字段范围、日期合法性都在收到最后一个字符后的下一拍算好；行尾至少晚一个字节时间，
    // 所以行尾判定只剩几个寄存器相与，不再把比较器压在 command_ok -> 几十个寄存器的路径上
    reg crc_ok,acc_m,acc_l,acc_t,z_ok;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin crc_ok<=1; acc_m<=0; acc_l<=0; acc_t<=0; z_ok<=0; end
        else begin
            crc_ok<=!star || (crc_digits==4 && crc_rx==crc);
            acc_m<=m_ok && accumulator<=1500;
            acc_l<=l_ok && accumulator<=1000;
            acc_t<=t_ok && accumulator!=0 && accumulator<=1500;
            z_ok<=z_ok_c;
        end
    wire num_ok=field==nf && digits!=0 && crc_ok && (op=="M" ? acc_m : op=="L" ? acc_l : acc_t);
    assign command_ok=!malformed && ((numop && num_ok) || (op=="Z" && z_ok) ||
        (op=="Q" && length==2) || (op=="D" && length==3 && (last=="0" || last=="1")));

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            length<=0; prefix<=0; op<=0; last<=0; malformed<=0;
            field<=0; digits<=0; accumulator<=0; star<=0; crc_digits<=0; crc<=16'hffff; crc_rx<=0;
            zbuf<=0; zdig<=0;
            for (i=0;i<4;i=i+1) fields[i]<=0;
        end else if (eol) begin
            length<=0; prefix<=0; op<=0; last<=0; malformed<=0;
            field<=0; digits<=0; accumulator<=0; star<=0; crc_digits<=0; crc<=16'hffff; crc_rx<=0; zdig<=0;
        end else if (rx_valid) begin
            if (length < 64) length<=length+1'b1; else malformed<=1;
            last<=rx_data;
            if (length==0) prefix <= rx_data=="!";
            if (length==1) op <= (rx_data >= "a" && rx_data <= "z") ? rx_data-32 : rx_data;
            if (!star && rx_data!="*") crc<=crc_byte(crc,rx_data);
            if (length>=2 && numop) begin
                if (star) begin
                    if (hexdig && crc_digits<4) begin crc_rx<={crc_rx[11:0],hexval}; crc_digits<=crc_digits+1'b1; end
                    else malformed<=1;
                end else if (rx_data=="*") begin
                    if (field==nf && digits!=0) star<=1; else malformed<=1;
                end else if (digit && digits<5 && accumulator*20'd10 + (rx_data-8'd48) <= 65535) begin
                    accumulator<=accumulator*20'd10+(rx_data-8'd48); digits<=digits+1'b1;
                end else if (rx_data=="," && field<nf && digits!=0) begin
                    fields[field]<=accumulator[15:0]; field<=field+1'b1; accumulator<=0; digits<=0;
                end else malformed<=1;
            end
            if (length>=2 && op=="Z") begin
                if (digit && zdig<12) begin zbuf<={zbuf[43:0],rx_data[3:0]}; zdig<=zdig+1'b1; end
                else malformed<=1;
            end
            if (rx_data<32 || rx_data>126) malformed<=1;
        end
    end

    // ---------------- 日期时间（BCD，带月份天数和闰年） ----------------
    reg [7:0] t_yy,t_mo,t_dd,t_hh,t_mi,t_ss;
    assign datetime={t_yy,t_mo,t_dd,t_hh,t_mi,t_ss};
    function [7:0] inc_bcd;
        input [7:0] v;
        inc_bcd=v[3:0]==4'd9 ? {v[7:4]+4'd1,4'd0} : v+8'd1;
    endfunction
    wire leap=t_yy[4] ? (t_yy[3:0]==2 || t_yy[3:0]==6) : (t_yy[3:0]==0 || t_yy[3:0]==4 || t_yy[3:0]==8);
    wire [7:0] mlast=t_mo==8'h02 ? (leap ? 8'h29 : 8'h28) :
        (t_mo==8'h04 || t_mo==8'h06 || t_mo==8'h09 || t_mo==8'h11) ? 8'h30 : 8'h31;

    // ---------------- 演示数据发生器 ----------------
    reg [15:0] lfsr;
    reg [5:0] dsec;                       // 64 秒演示周期
    reg [2:0] wsec;                       // 每 8 秒一片晶圆
    reg [5:0] up_s;
    wire [2:0] up3=dsec[2:0];             // 40~47 s：升温
    wire [5:0] dn6=6'd60-dsec;            // 54~59 s：降温
    wire [2:0] dn3=dn6[2:0];
    wire [8:0] up40={1'b0,up3,5'd0}+{3'd0,up3,3'd0};
    wire [8:0] dn40={1'b0,dn3,5'd0}+{3'd0,dn3,3'd0};
    wire [15:0] t_base=16'd350+{12'd0,lfsr[3:0]};
    wire [15:0] t_demo=(dsec>=40 && dsec<48) ? t_base+{7'd0,up40} :
                       (dsec>=48 && dsec<54) ? 16'd630+{12'd0,lfsr[3:0]} :
                       (dsec>=54 && dsec<60) ? t_base+{7'd0,dn40} : t_base;
    wire [1:0] dnew=lfsr[9:8]==3 ? 2'd2 : lfsr[9:8]==2 ? 2'd1 : 2'd0;

    // ---------------- 状态与警告 ----------------
    reg armed;
    reg [2:0] bad_s,good_s;
    assign stale=!demo && armed && age>=TIMEOUT_S;
    wire voltage_bad=valid && (millivolts<lim[63:48] || millivolts>lim[47:32]);
    wire current_bad=valid && milliamps>lim[31:16];
    wire temp_bad=valid && temperature>lim[15:0];
    assign failing=voltage_bad || current_bad || temp_bad;
    wire bad=failing || stale;

    // F 标志位：bit8 DEMO，bit7 越限，bit6 图片有效，bit5 详情，bit4 STALE，bit3 上下文有效，bit2 警告，bit1 手动报警，bit0 报警/警告
    wire [15:0] flags={7'd0,demo,failing,picture_valid,detail_mode,stale,ctx_valid,warn,manual_alarm,warn|manual_alarm};
    // 串口格式：ST B=tttt V=v F=ffff N=nnnn U=uuuu I=iiii T=tttt O=oooo
    assign response={13'd0,tool_id,3'd0,valid,flags,sequence_id,millivolts,milliamps,temperature,event_overflows};

    wire accept=handled && command_ok;

    // 板间通信质量：最近 16 行板间命令的成功/失败；超过 TIMEOUT_S 秒没有合法行判为中断
    reg link_seen;
    reg [7:0] link_age;
    reg [15:0] link_hist;
    assign link_state=!link_seen ? 2'd0 : link_age>=TIMEOUT_S ? 2'd2 : 2'd1;
    function [4:0] pop16;
        input [15:0] v;
        integer j;
        begin pop16=0; for(j=0;j<16;j=j+1) pop16=pop16+v[j]; end
    endfunction
    wire [4:0] lp=pop16(link_hist);
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin link_seen<=0; link_age<=0; link_hist<=16'hffff; link_q<=0; end
        else begin
            link_q<={1'b0,lp,2'b00}+{2'b00,lp,1'b0}+{5'd0,lp[4:2]};    // lp*6.25
            if(link_eol) begin
                link_hist<={link_hist[14:0],accept};
                if(accept) begin link_seen<=1; link_age<=0; end
            end else if(stick && link_seen && link_age!=8'hff) link_age<=link_age+1'b1;
        end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            qcnt<=0;qsub<=0;age<=0;armed<=0;seq_armed<=0;bad_s<=0;good_s<=0;
            valid<=0;ctx_valid<=0;warn<=0;reasons<=0;demo<=(LOCAL_DEMO!=0);tool_id<=0;
            sequence_id<=0;millivolts<=0;milliamps<=0;temperature<=0;
            lot<=16'd1234;wafer<=16'd1;defects<=0;yield<=16'd1000;
            vmin<=16'd3200;vmax<=16'd3400;imax<=16'd60;tmax<=16'd600;
            uptime_min<=0;packets<=0;errors<=0;
            lfsr<=16'hace1;dsec<=0;wsec<=0;up_s<=0;
            t_yy<=8'h26;t_mo<=8'h10;t_dd<=8'h23;t_hh<=8'h08;t_mi<=8'h00;t_ss<=8'h00;
        end else begin
            qcnt<=qtick ? 28'd0 : qcnt+1'b1;
            if(qtick) qsub<=qsub+1'b1;

            if(stick) begin
                // 运行时间
                if(up_s==59) begin up_s<=0; uptime_min<=uptime_min+1'b1; end else up_s<=up_s+1'b1;
                // 日期时间
                if(t_ss!=8'h59) t_ss<=inc_bcd(t_ss);
                else begin
                    t_ss<=0;
                    if(t_mi!=8'h59) t_mi<=inc_bcd(t_mi);
                    else begin
                        t_mi<=0;
                        if(t_hh!=8'h23) t_hh<=inc_bcd(t_hh);
                        else begin
                            t_hh<=0;
                            if(t_dd<mlast) t_dd<=inc_bcd(t_dd);
                            else begin
                                t_dd<=8'h01;
                                if(t_mo!=8'h12) t_mo<=inc_bcd(t_mo);
                                else begin t_mo<=8'h01; t_yy<=inc_bcd(t_yy); end
                            end
                        end
                    end
                end
                if(!demo && age<255) age<=age+1'b1;
                // 警告：连续 WARN_S 秒异常才弹出，连续 WARN_S 秒正常自动消失，阈值附近不闪烁
                if(bad) begin
                    good_s<=0;
                    if(bad_s<7) bad_s<=bad_s+1'b1;
                    if(bad_s>=WARN_S-1) begin warn<=1; reasons<={stale,temp_bad,voltage_bad|current_bad}; end
                end else begin
                    bad_s<=0;
                    if(warn) begin
                        if(good_s>=WARN_S-1) begin warn<=0; reasons<=0; good_s<=0; end
                        else good_s<=good_s+1'b1;
                    end
                end
            end

            if(handled && !command_ok && errors!=16'hffff) errors<=errors+1'b1;
            if(accept && op=="T") begin
                vmin<=fields[0];vmax<=fields[1];imax<=fields[2];tmax<=accumulator[15:0];
                if(packets!=16'hffff) packets<=packets+1'b1;
            end else if(lim_load) begin
                vmin<=tab_vmin(lim_idx[1:0]);vmax<=tab_vmax(lim_idx[3:2]);
                imax<=tab_imax(lim_idx[5:4]);tmax<=tab_tmax(lim_idx[7:6]);
            end
            if(accept && op=="Z") begin
                t_yy<=zbuf[47:40];t_mo<=zmo;t_dd<=zdd;t_hh<=zhh;t_mi<=zmi;t_ss<=zss;
            end

            if(accept && op=="D") begin
                // 切换数据来源：清掉旧数据重新建立基准
                demo<=last[0];
                valid<=0;ctx_valid<=0;armed<=0;age<=0;seq_armed<=0;tool_id<=0;
                warn<=0;reasons<=0;bad_s<=0;good_s<=0;
            end else if(accept && op=="M") begin
                if(demo) begin demo<=0; ctx_valid<=0; warn<=0; reasons<=0; bad_s<=0; good_s<=0; end
                valid<=1;armed<=1;age<=0;seq_armed<=1;
                tool_id<=fields[0][2:0];sequence_id<=fields[1];
                millivolts<=fields[2];milliamps<=fields[3];temperature<=accumulator[15:0];
                if(packets!=16'hffff) packets<=packets+1'b1;
            end else if(accept && op=="L") begin
                if(demo) begin demo<=0; valid<=0; seq_armed<=0; tool_id<=0; warn<=0; reasons<=0; bad_s<=0; good_s<=0; end
                ctx_valid<=1;armed<=1;age<=0;
                lot<=fields[0];wafer<=fields[1];defects<=fields[2];yield<=accumulator[15:0];
                if(packets!=16'hffff) packets<=packets+1'b1;
            end else if(demo && qtick) begin
                // 演示值，不是测量值；屏幕和 !Q 应答都带 DEMO 标志
                lfsr<={lfsr[14:0],lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
                valid<=1;ctx_valid<=1;armed<=1;age<=0;tool_id<=3'd1;
                millivolts<=16'd3292+{12'd0,lfsr[7:4]};
                milliamps<=16'd40+{12'd0,lfsr[11:8]};
                temperature<=t_demo;
                yield<=16'd1000-{defects[13:0],2'b00};
                if(stick) begin
                    dsec<=dsec+1'b1;
                    sequence_id<=sequence_id+1'b1;
                    wsec<=wsec+1'b1;
                    if(wsec==3'd7) begin
                        if(wafer>=25) begin wafer<=16'd1; lot<=lot+1'b1; defects<=0; end
                        else begin wafer<=wafer+1'b1; defects<=defects+{14'd0,dnew}; end
                    end
                end
            end
        end
    end
endmodule
