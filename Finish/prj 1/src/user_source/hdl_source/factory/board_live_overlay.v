`timescale 1ns/1ps
// 晶圆厂看板叠加层（video_clk 域，640x480）
//   页面文字和动态字段放在 BRAM：每页 32 行 x 64 列地址（显示 30 x 40），每格 16x16 像素，9 位代码。
//   网格表由 tools/fab_layout.py 生成，同时生成 PIC1~4.BMP 底图，两边坐标一致，不会错位。
//   格子代码：000 透明  020~05F 字符  001~007 状态文字  008~00B DEMO/LIVE  00C~013 设备名
//             018~01F 缺陷点  060~07F 柱条{柱号2,段3}  080~0FF 数值字段 0~15{字段4,位3}
//             100~13F 数值字段 16~23  180~183 板间链路状态  1C0~1C7 振动柱条
//   背景透明：只在有内容的格子后面压暗一半，图片其余部分原样显示
//   警告横幅（第 13~14 行）：越限/超时/振动超限时琥珀色半透明、无声；SW3 手动报警时红色，边框闪烁
//   第 21 行：声学/振动 + 板间链路；SW4 打开时换成阈值行（ROM 第 31 行）
module board_live_overlay(
    input wire clk,rst_n,frame_tick,de,
    input wire hide,                      // 缩略图 / 无图：隐藏页面字段（横幅照常）
    input wire menu,                      // 菜单打开：页面字段和横幅都让开，只留报警边框
    input wire clean,                     // 叠加层 CLEAN：隐藏页面字段
    input wire detail,                    // SW4：第 21 行显示阈值
    input wire [9:0] x,x_pre,
    input wire [9:0] y,y_pre,
    input wire [1:0] profile,
    input wire [270:0] data,
    input wire [6:0] vib,                 // 高频段振动能量 0~80
    input wire [15:0] peak_hz,
    input wire peak_ok,
    input wire [6:0] vib_lim,
    input wire vib_warn,
    input wire [23:0] rgb_in,
    output wire [23:0] rgb_out,
    output wire banner_on                 // 警告横幅正在显示（产线监控页画中画让出第 13~14 行）
);
    wire manual=data[270],valid=data[269],stale=data[268],warn=data[267],failing=data[266];
    wire demo=data[265],ctx=data[264];
    wire [2:0] reasons=data[263:261],tool=data[260:258];
    wire [7:0] age=data[257:250];
    wire [1:0] link_state=data[249:248];
    wire [7:0] link_q=data[247:240];
    wire [15:0] mv=data[239:224],ma=data[223:208],temp=data[207:192],seq=data[191:176];
    wire [15:0] lot=data[175:160],wafer=data[159:144],defects=data[143:128],yield=data[127:112];
    wire [15:0] uptime=data[111:96],pkts=data[95:80],errs=data[79:64];
    wire [15:0] vmin=data[63:48],vmax=data[47:32],imax=data[31:16],tmax=data[15:0];
    wire v_bad=valid && (mv<vmin || mv>vmax);
    wire i_bad=valid && ma>imax;
    wire t_bad=valid && temp>tmax;
    wire vib_bad=vib>=vib_lim;

    // ---------------- 20 个数值字段轮流转 BCD（每个 17 拍，一帧内刷新上千次） ----------------
    reg [4:0] fsel;
    reg [4:0] cnt;
    reg [15:0] bin;
    reg [19:0] bcd,adj;
    reg [19:0] num [0:19];
    reg [15:0] fval;
    always @* case(fsel)
        0:fval=mv; 1:fval=ma; 2:fval=temp; 3:fval=seq; 4:fval={8'd0,age}; 5:fval=lot; 6:fval=wafer; 7:fval=defects;
        8:fval=yield; 9:fval=uptime; 10:fval=pkts; 11:fval=errs;
        12:fval=vmin; 13:fval=vmax; 14:fval=imax; 15:fval=tmax;
        16:fval={9'd0,vib}; 17:fval=peak_hz; 18:fval={8'd0,link_q}; default:fval={9'd0,vib_lim};
    endcase
    integer k;
    always @* begin
        adj=bcd;
        for(k=0;k<5;k=k+1) if(bcd[k*4 +: 4]>=5) adj[k*4 +: 4]=bcd[k*4 +: 4]+4'd3;
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin fsel<=0; cnt<=0; bin<=0; bcd<=0; end
        else if(cnt==0) begin bin<=fval; bcd<=0; cnt<=16; end
        else begin
            bcd<={adj[18:0],bin[15]}; bin<={bin[14:0],1'b0}; cnt<=cnt-1'b1;
            if(cnt==1) fsel<=(fsel==5'd19) ? 5'd0 : fsel+1'b1;
        end
    always @(posedge clk) if(cnt==1) num[fsel]<={adj[18:0],bin[15]};

    function [4:0] glyph_row;
        input [7:0] character;
        input [2:0] row;
        begin
            glyph_row = 5'b00000;
            case (character)
                "%": case(row) 0:glyph_row=5'b11001;1:glyph_row=5'b11010;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b10110;6:glyph_row=5'b00110;default:glyph_row=0;endcase
                "<": case(row) 1:glyph_row=5'b00010;2:glyph_row=5'b00100;3:glyph_row=5'b01000;4:glyph_row=5'b00100;5:glyph_row=5'b00010;default:glyph_row=0;endcase
                ">": case(row) 1:glyph_row=5'b01000;2:glyph_row=5'b00100;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;default:glyph_row=0;endcase
                "=": case(row) 2:glyph_row=5'b11111;4:glyph_row=5'b11111;default:glyph_row=0;endcase
                "0": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10011;3:glyph_row=5'b10101;4:glyph_row=5'b11001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "1": case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b01100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "2": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00001;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "3": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b00001;2:glyph_row=5'b00001;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "4": case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00110;2:glyph_row=5'b01010;3:glyph_row=5'b10010;4:glyph_row=5'b11111;5:glyph_row=5'b00010;6:glyph_row=5'b00010;default:glyph_row=0;endcase
                "5": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "6": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "7": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b01000;default:glyph_row=0;endcase
                "8": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "9": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01111;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "A": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "B": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "C": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b01111;default:glyph_row=0;endcase
                "D": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "E": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "F": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "G": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10000;3:glyph_row=5'b10111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "H": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "I": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "J": case(row) 0:glyph_row=5'b00111;1:glyph_row=5'b00010;2:glyph_row=5'b00010;3:glyph_row=5'b00010;4:glyph_row=5'b10010;5:glyph_row=5'b10010;6:glyph_row=5'b01100;default:glyph_row=0;endcase
                "K": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10010;2:glyph_row=5'b10100;3:glyph_row=5'b11000;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "L": case(row) 0:glyph_row=5'b10000;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "M": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11011;2:glyph_row=5'b10101;3:glyph_row=5'b10101;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "N": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11001;2:glyph_row=5'b10101;3:glyph_row=5'b10011;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "O": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "P": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "Q": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10101;5:glyph_row=5'b10010;6:glyph_row=5'b01101;default:glyph_row=0;endcase
                "R": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "S": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "T": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "U": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "V": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b01010;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "W": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10101;4:glyph_row=5'b10101;5:glyph_row=5'b10101;6:glyph_row=5'b01010;default:glyph_row=0;endcase
                "X": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b01010;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "Y": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "Z": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                ":": case(row) 1:glyph_row=5'b00100;2:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                ".": case(row) 5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                ",": case(row) 5:glyph_row=5'b00100;6:glyph_row=5'b01000;default:glyph_row=0;endcase
                "!": case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                "?": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;5:glyph_row=5'b00100;default:glyph_row=0;endcase
                "-": case(row) 3:glyph_row=5'b11111;default:glyph_row=0;endcase
                "_": case(row) 6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "/": case(row) 0:glyph_row=5'b00001;1:glyph_row=5'b00010;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                default: glyph_row = 5'b00000;
            endcase
        end
    endfunction

    // ---------------- 预取一拍：读网格 ROM ----------------
    reg [5:0] blink;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) blink<=0; else if(frame_tick) blink<=blink+1'b1;
    wire banner=manual || warn || vib_warn;
    // 优先级：手动报警 > 链路超时 > 温度 > 电压/电流 > 振动
    wire [2:0] variant=manual ? 3'd0 : !warn ? 3'd4 : reasons[2] ? 3'd1 : reasons[1] ? 3'd2 : 3'd3;
    wire [5:0] col_p=x_pre[9:4];
    wire [4:0] row_p=y_pre[8:4];
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [8:0] page_rom[0:8191];
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [8:0] ban_rom[0:1023];
    integer r;
    initial begin
        for(r=0;r<8192;r=r+1) page_rom[r]=9'h000;
        for(r=0;r<1024;r=r+1) ban_rom[r]=9'h000;
        // ==== ROM BEGIN (tools/fab_layout.py 生成，勿手改) ====
        page_rom[193]=9'h008;
        page_rom[194]=9'h009;
        page_rom[195]=9'h00a;
        page_rom[196]=9'h00b;
        page_rom[197]=9'h020;
        page_rom[198]=9'h00c;
        page_rom[199]=9'h00d;
        page_rom[200]=9'h00e;
        page_rom[201]=9'h00f;
        page_rom[202]=9'h010;
        page_rom[203]=9'h011;
        page_rom[204]=9'h012;
        page_rom[205]=9'h013;
        page_rom[206]=9'h020;
        page_rom[207]=9'h001;
        page_rom[208]=9'h002;
        page_rom[209]=9'h003;
        page_rom[210]=9'h004;
        page_rom[211]=9'h005;
        page_rom[212]=9'h006;
        page_rom[213]=9'h007;
        page_rom[214]=9'h020;
        page_rom[215]=9'h053;
        page_rom[216]=9'h045;
        page_rom[217]=9'h051;
        page_rom[218]=9'h098;
        page_rom[219]=9'h099;
        page_rom[220]=9'h09a;
        page_rom[221]=9'h09b;
        page_rom[222]=9'h09c;
        page_rom[223]=9'h020;
        page_rom[224]=9'h041;
        page_rom[225]=9'h047;
        page_rom[226]=9'h045;
        page_rom[227]=9'h0a2;
        page_rom[228]=9'h0a3;
        page_rom[229]=9'h0a4;
        page_rom[230]=9'h053;
        page_rom[454]=9'h00c;
        page_rom[455]=9'h00d;
        page_rom[456]=9'h00e;
        page_rom[457]=9'h00f;
        page_rom[458]=9'h010;
        page_rom[459]=9'h011;
        page_rom[460]=9'h012;
        page_rom[461]=9'h013;
        page_rom[472]=9'h070;
        page_rom[473]=9'h071;
        page_rom[474]=9'h072;
        page_rom[475]=9'h073;
        page_rom[476]=9'h074;
        page_rom[477]=9'h075;
        page_rom[478]=9'h076;
        page_rom[479]=9'h077;
        page_rom[481]=9'h0b3;
        page_rom[482]=9'h0b4;
        page_rom[483]=9'h02f;
        page_rom[484]=9'h032;
        page_rom[485]=9'h035;
        page_rom[582]=9'h04c;
        page_rom[583]=9'h0a9;
        page_rom[584]=9'h0aa;
        page_rom[585]=9'h0ab;
        page_rom[586]=9'h0ac;
        page_rom[600]=9'h078;
        page_rom[601]=9'h079;
        page_rom[602]=9'h07a;
        page_rom[603]=9'h07b;
        page_rom[604]=9'h07c;
        page_rom[605]=9'h07d;
        page_rom[606]=9'h07e;
        page_rom[607]=9'h07f;
        page_rom[609]=9'h0c1;
        page_rom[610]=9'h0c2;
        page_rom[611]=9'h0c3;
        page_rom[612]=9'h02e;
        page_rom[613]=9'h0c4;
        page_rom[614]=9'h025;
        page_rom[710]=9'h057;
        page_rom[711]=9'h0b3;
        page_rom[712]=9'h0b4;
        page_rom[713]=9'h02f;
        page_rom[714]=9'h032;
        page_rom[715]=9'h035;
        page_rom[728]=9'h0ba;
        page_rom[729]=9'h0bb;
        page_rom[730]=9'h0bc;
        page_rom[731]=9'h020;
        page_rom[732]=9'h050;
        page_rom[733]=9'h043;
        page_rom[734]=9'h053;
        page_rom[1093]=9'h080;
        page_rom[1094]=9'h081;
        page_rom[1095]=9'h02e;
        page_rom[1096]=9'h082;
        page_rom[1097]=9'h083;
        page_rom[1098]=9'h084;
        page_rom[1099]=9'h056;
        page_rom[1105]=9'h089;
        page_rom[1106]=9'h08a;
        page_rom[1107]=9'h08b;
        page_rom[1108]=9'h08c;
        page_rom[1109]=9'h04d;
        page_rom[1110]=9'h041;
        page_rom[1117]=9'h091;
        page_rom[1118]=9'h092;
        page_rom[1119]=9'h093;
        page_rom[1120]=9'h02e;
        page_rom[1121]=9'h094;
        page_rom[1122]=9'h043;
        page_rom[1221]=9'h001;
        page_rom[1222]=9'h002;
        page_rom[1223]=9'h003;
        page_rom[1224]=9'h004;
        page_rom[1225]=9'h005;
        page_rom[1226]=9'h006;
        page_rom[1227]=9'h007;
        page_rom[1233]=9'h0d0;
        page_rom[1234]=9'h0d1;
        page_rom[1235]=9'h0d2;
        page_rom[1236]=9'h0d3;
        page_rom[1237]=9'h0d4;
        page_rom[1245]=9'h0d8;
        page_rom[1246]=9'h0d9;
        page_rom[1247]=9'h0da;
        page_rom[1248]=9'h0db;
        page_rom[1249]=9'h0dc;
        page_rom[1346]=9'h056;
        page_rom[1347]=9'h049;
        page_rom[1348]=9'h042;
        page_rom[1349]=9'h102;
        page_rom[1350]=9'h103;
        page_rom[1351]=9'h104;
        page_rom[1352]=9'h02f;
        page_rom[1353]=9'h11b;
        page_rom[1354]=9'h11c;
        page_rom[1355]=9'h020;
        page_rom[1356]=9'h050;
        page_rom[1357]=9'h045;
        page_rom[1358]=9'h041;
        page_rom[1359]=9'h04b;
        page_rom[1360]=9'h108;
        page_rom[1361]=9'h109;
        page_rom[1362]=9'h10a;
        page_rom[1363]=9'h10b;
        page_rom[1364]=9'h10c;
        page_rom[1365]=9'h048;
        page_rom[1366]=9'h05a;
        page_rom[1367]=9'h020;
        page_rom[1368]=9'h04c;
        page_rom[1369]=9'h049;
        page_rom[1370]=9'h04e;
        page_rom[1371]=9'h04b;
        page_rom[1372]=9'h020;
        page_rom[1373]=9'h180;
        page_rom[1374]=9'h181;
        page_rom[1375]=9'h182;
        page_rom[1376]=9'h183;
        page_rom[1377]=9'h112;
        page_rom[1378]=9'h113;
        page_rom[1379]=9'h114;
        page_rom[1380]=9'h025;
        page_rom[1986]=9'h04c;
        page_rom[1987]=9'h049;
        page_rom[1988]=9'h04d;
        page_rom[1989]=9'h049;
        page_rom[1990]=9'h054;
        page_rom[1991]=9'h020;
        page_rom[1992]=9'h056;
        page_rom[1993]=9'h0e0;
        page_rom[1994]=9'h0e1;
        page_rom[1995]=9'h02e;
        page_rom[1996]=9'h0e2;
        page_rom[1997]=9'h0e3;
        page_rom[1998]=9'h02d;
        page_rom[1999]=9'h0e8;
        page_rom[2000]=9'h0e9;
        page_rom[2001]=9'h02e;
        page_rom[2002]=9'h0ea;
        page_rom[2003]=9'h0eb;
        page_rom[2004]=9'h056;
        page_rom[2005]=9'h020;
        page_rom[2006]=9'h049;
        page_rom[2007]=9'h03c;
        page_rom[2008]=9'h0f2;
        page_rom[2009]=9'h0f3;
        page_rom[2010]=9'h0f4;
        page_rom[2011]=9'h04d;
        page_rom[2012]=9'h041;
        page_rom[2013]=9'h020;
        page_rom[2014]=9'h054;
        page_rom[2015]=9'h03c;
        page_rom[2016]=9'h0f9;
        page_rom[2017]=9'h0fa;
        page_rom[2018]=9'h0fb;
        page_rom[2019]=9'h043;
        page_rom[2241]=9'h008;
        page_rom[2242]=9'h009;
        page_rom[2243]=9'h00a;
        page_rom[2244]=9'h00b;
        page_rom[2245]=9'h020;
        page_rom[2246]=9'h00c;
        page_rom[2247]=9'h00d;
        page_rom[2248]=9'h00e;
        page_rom[2249]=9'h00f;
        page_rom[2250]=9'h010;
        page_rom[2251]=9'h011;
        page_rom[2252]=9'h012;
        page_rom[2253]=9'h013;
        page_rom[2254]=9'h020;
        page_rom[2255]=9'h001;
        page_rom[2256]=9'h002;
        page_rom[2257]=9'h003;
        page_rom[2258]=9'h004;
        page_rom[2259]=9'h005;
        page_rom[2260]=9'h006;
        page_rom[2261]=9'h007;
        page_rom[2262]=9'h020;
        page_rom[2263]=9'h053;
        page_rom[2264]=9'h045;
        page_rom[2265]=9'h051;
        page_rom[2266]=9'h098;
        page_rom[2267]=9'h099;
        page_rom[2268]=9'h09a;
        page_rom[2269]=9'h09b;
        page_rom[2270]=9'h09c;
        page_rom[2271]=9'h020;
        page_rom[2272]=9'h041;
        page_rom[2273]=9'h047;
        page_rom[2274]=9'h045;
        page_rom[2275]=9'h0a2;
        page_rom[2276]=9'h0a3;
        page_rom[2277]=9'h0a4;
        page_rom[2278]=9'h053;
        page_rom[2498]=9'h080;
        page_rom[2499]=9'h081;
        page_rom[2500]=9'h02e;
        page_rom[2501]=9'h082;
        page_rom[2502]=9'h083;
        page_rom[2503]=9'h084;
        page_rom[2504]=9'h056;
        page_rom[2511]=9'h089;
        page_rom[2512]=9'h08a;
        page_rom[2513]=9'h08b;
        page_rom[2514]=9'h08c;
        page_rom[2515]=9'h04d;
        page_rom[2516]=9'h041;
        page_rom[2524]=9'h091;
        page_rom[2525]=9'h092;
        page_rom[2526]=9'h093;
        page_rom[2527]=9'h02e;
        page_rom[2528]=9'h094;
        page_rom[2529]=9'h043;
        page_rom[2626]=9'h001;
        page_rom[2627]=9'h002;
        page_rom[2628]=9'h003;
        page_rom[2629]=9'h004;
        page_rom[2630]=9'h005;
        page_rom[2631]=9'h006;
        page_rom[2632]=9'h007;
        page_rom[2639]=9'h068;
        page_rom[2640]=9'h069;
        page_rom[2641]=9'h06a;
        page_rom[2642]=9'h06b;
        page_rom[2643]=9'h06c;
        page_rom[2644]=9'h06d;
        page_rom[2645]=9'h06e;
        page_rom[2646]=9'h06f;
        page_rom[2652]=9'h060;
        page_rom[2653]=9'h061;
        page_rom[2654]=9'h062;
        page_rom[2655]=9'h063;
        page_rom[2656]=9'h064;
        page_rom[2657]=9'h065;
        page_rom[2658]=9'h066;
        page_rom[2659]=9'h067;
        page_rom[3077]=9'h0e0;
        page_rom[3078]=9'h0e1;
        page_rom[3079]=9'h02e;
        page_rom[3080]=9'h0e2;
        page_rom[3081]=9'h0e3;
        page_rom[3082]=9'h056;
        page_rom[3090]=9'h0f2;
        page_rom[3091]=9'h0f3;
        page_rom[3092]=9'h0f4;
        page_rom[3093]=9'h04d;
        page_rom[3094]=9'h041;
        page_rom[3103]=9'h0f9;
        page_rom[3104]=9'h0fa;
        page_rom[3105]=9'h0fb;
        page_rom[3106]=9'h043;
        page_rom[3205]=9'h0e8;
        page_rom[3206]=9'h0e9;
        page_rom[3207]=9'h02e;
        page_rom[3208]=9'h0ea;
        page_rom[3209]=9'h0eb;
        page_rom[3210]=9'h056;
        page_rom[3218]=9'h0d0;
        page_rom[3219]=9'h0d1;
        page_rom[3220]=9'h0d2;
        page_rom[3221]=9'h0d3;
        page_rom[3222]=9'h0d4;
        page_rom[3231]=9'h0a2;
        page_rom[3232]=9'h0a3;
        page_rom[3233]=9'h0a4;
        page_rom[3234]=9'h053;
        page_rom[3394]=9'h056;
        page_rom[3395]=9'h049;
        page_rom[3396]=9'h042;
        page_rom[3397]=9'h102;
        page_rom[3398]=9'h103;
        page_rom[3399]=9'h104;
        page_rom[3400]=9'h02f;
        page_rom[3401]=9'h11b;
        page_rom[3402]=9'h11c;
        page_rom[3403]=9'h020;
        page_rom[3404]=9'h050;
        page_rom[3405]=9'h045;
        page_rom[3406]=9'h041;
        page_rom[3407]=9'h04b;
        page_rom[3408]=9'h108;
        page_rom[3409]=9'h109;
        page_rom[3410]=9'h10a;
        page_rom[3411]=9'h10b;
        page_rom[3412]=9'h10c;
        page_rom[3413]=9'h048;
        page_rom[3414]=9'h05a;
        page_rom[3415]=9'h020;
        page_rom[3416]=9'h04c;
        page_rom[3417]=9'h049;
        page_rom[3418]=9'h04e;
        page_rom[3419]=9'h04b;
        page_rom[3420]=9'h020;
        page_rom[3421]=9'h180;
        page_rom[3422]=9'h181;
        page_rom[3423]=9'h182;
        page_rom[3424]=9'h183;
        page_rom[3425]=9'h112;
        page_rom[3426]=9'h113;
        page_rom[3427]=9'h114;
        page_rom[3428]=9'h025;
        page_rom[4034]=9'h04c;
        page_rom[4035]=9'h049;
        page_rom[4036]=9'h04d;
        page_rom[4037]=9'h049;
        page_rom[4038]=9'h054;
        page_rom[4039]=9'h020;
        page_rom[4040]=9'h056;
        page_rom[4041]=9'h0e0;
        page_rom[4042]=9'h0e1;
        page_rom[4043]=9'h02e;
        page_rom[4044]=9'h0e2;
        page_rom[4045]=9'h0e3;
        page_rom[4046]=9'h02d;
        page_rom[4047]=9'h0e8;
        page_rom[4048]=9'h0e9;
        page_rom[4049]=9'h02e;
        page_rom[4050]=9'h0ea;
        page_rom[4051]=9'h0eb;
        page_rom[4052]=9'h056;
        page_rom[4053]=9'h020;
        page_rom[4054]=9'h049;
        page_rom[4055]=9'h03c;
        page_rom[4056]=9'h0f2;
        page_rom[4057]=9'h0f3;
        page_rom[4058]=9'h0f4;
        page_rom[4059]=9'h04d;
        page_rom[4060]=9'h041;
        page_rom[4061]=9'h020;
        page_rom[4062]=9'h054;
        page_rom[4063]=9'h03c;
        page_rom[4064]=9'h0f9;
        page_rom[4065]=9'h0fa;
        page_rom[4066]=9'h0fb;
        page_rom[4067]=9'h043;
        page_rom[4289]=9'h008;
        page_rom[4290]=9'h009;
        page_rom[4291]=9'h00a;
        page_rom[4292]=9'h00b;
        page_rom[4293]=9'h020;
        page_rom[4294]=9'h00c;
        page_rom[4295]=9'h00d;
        page_rom[4296]=9'h00e;
        page_rom[4297]=9'h00f;
        page_rom[4298]=9'h010;
        page_rom[4299]=9'h011;
        page_rom[4300]=9'h012;
        page_rom[4301]=9'h013;
        page_rom[4302]=9'h020;
        page_rom[4303]=9'h001;
        page_rom[4304]=9'h002;
        page_rom[4305]=9'h003;
        page_rom[4306]=9'h004;
        page_rom[4307]=9'h005;
        page_rom[4308]=9'h006;
        page_rom[4309]=9'h007;
        page_rom[4310]=9'h020;
        page_rom[4311]=9'h053;
        page_rom[4312]=9'h045;
        page_rom[4313]=9'h051;
        page_rom[4314]=9'h098;
        page_rom[4315]=9'h099;
        page_rom[4316]=9'h09a;
        page_rom[4317]=9'h09b;
        page_rom[4318]=9'h09c;
        page_rom[4319]=9'h020;
        page_rom[4320]=9'h041;
        page_rom[4321]=9'h047;
        page_rom[4322]=9'h045;
        page_rom[4323]=9'h0a2;
        page_rom[4324]=9'h0a3;
        page_rom[4325]=9'h0a4;
        page_rom[4326]=9'h053;
        page_rom[4569]=9'h0a9;
        page_rom[4570]=9'h0aa;
        page_rom[4571]=9'h0ab;
        page_rom[4572]=9'h0ac;
        page_rom[4615]=9'h018;
        page_rom[4620]=9'h019;
        page_rom[4681]=9'h01a;
        page_rom[4697]=9'h0b3;
        page_rom[4698]=9'h0b4;
        page_rom[4699]=9'h02f;
        page_rom[4700]=9'h032;
        page_rom[4701]=9'h035;
        page_rom[4750]=9'h01b;
        page_rom[4806]=9'h01c;
        page_rom[4811]=9'h01d;
        page_rom[4825]=9'h0ba;
        page_rom[4826]=9'h0bb;
        page_rom[4827]=9'h0bc;
        page_rom[4828]=9'h020;
        page_rom[4829]=9'h050;
        page_rom[4830]=9'h043;
        page_rom[4831]=9'h053;
        page_rom[4872]=9'h01e;
        page_rom[4877]=9'h01f;
        page_rom[5145]=9'h0c1;
        page_rom[5146]=9'h0c2;
        page_rom[5147]=9'h0c3;
        page_rom[5148]=9'h02e;
        page_rom[5149]=9'h0c4;
        page_rom[5150]=9'h025;
        page_rom[5191]=9'h0ba;
        page_rom[5192]=9'h0bb;
        page_rom[5193]=9'h0bc;
        page_rom[5194]=9'h020;
        page_rom[5195]=9'h050;
        page_rom[5196]=9'h043;
        page_rom[5197]=9'h053;
        page_rom[5273]=9'h078;
        page_rom[5274]=9'h079;
        page_rom[5275]=9'h07a;
        page_rom[5276]=9'h07b;
        page_rom[5277]=9'h07c;
        page_rom[5278]=9'h07d;
        page_rom[5279]=9'h07e;
        page_rom[5280]=9'h07f;
        page_rom[5442]=9'h056;
        page_rom[5443]=9'h049;
        page_rom[5444]=9'h042;
        page_rom[5445]=9'h102;
        page_rom[5446]=9'h103;
        page_rom[5447]=9'h104;
        page_rom[5448]=9'h02f;
        page_rom[5449]=9'h11b;
        page_rom[5450]=9'h11c;
        page_rom[5451]=9'h020;
        page_rom[5452]=9'h050;
        page_rom[5453]=9'h045;
        page_rom[5454]=9'h041;
        page_rom[5455]=9'h04b;
        page_rom[5456]=9'h108;
        page_rom[5457]=9'h109;
        page_rom[5458]=9'h10a;
        page_rom[5459]=9'h10b;
        page_rom[5460]=9'h10c;
        page_rom[5461]=9'h048;
        page_rom[5462]=9'h05a;
        page_rom[5463]=9'h020;
        page_rom[5464]=9'h04c;
        page_rom[5465]=9'h049;
        page_rom[5466]=9'h04e;
        page_rom[5467]=9'h04b;
        page_rom[5468]=9'h020;
        page_rom[5469]=9'h180;
        page_rom[5470]=9'h181;
        page_rom[5471]=9'h182;
        page_rom[5472]=9'h183;
        page_rom[5473]=9'h112;
        page_rom[5474]=9'h113;
        page_rom[5475]=9'h114;
        page_rom[5476]=9'h025;
        page_rom[6082]=9'h04c;
        page_rom[6083]=9'h049;
        page_rom[6084]=9'h04d;
        page_rom[6085]=9'h049;
        page_rom[6086]=9'h054;
        page_rom[6087]=9'h020;
        page_rom[6088]=9'h056;
        page_rom[6089]=9'h0e0;
        page_rom[6090]=9'h0e1;
        page_rom[6091]=9'h02e;
        page_rom[6092]=9'h0e2;
        page_rom[6093]=9'h0e3;
        page_rom[6094]=9'h02d;
        page_rom[6095]=9'h0e8;
        page_rom[6096]=9'h0e9;
        page_rom[6097]=9'h02e;
        page_rom[6098]=9'h0ea;
        page_rom[6099]=9'h0eb;
        page_rom[6100]=9'h056;
        page_rom[6101]=9'h020;
        page_rom[6102]=9'h049;
        page_rom[6103]=9'h03c;
        page_rom[6104]=9'h0f2;
        page_rom[6105]=9'h0f3;
        page_rom[6106]=9'h0f4;
        page_rom[6107]=9'h04d;
        page_rom[6108]=9'h041;
        page_rom[6109]=9'h020;
        page_rom[6110]=9'h054;
        page_rom[6111]=9'h03c;
        page_rom[6112]=9'h0f9;
        page_rom[6113]=9'h0fa;
        page_rom[6114]=9'h0fb;
        page_rom[6115]=9'h043;
        page_rom[6337]=9'h008;
        page_rom[6338]=9'h009;
        page_rom[6339]=9'h00a;
        page_rom[6340]=9'h00b;
        page_rom[6341]=9'h020;
        page_rom[6342]=9'h00c;
        page_rom[6343]=9'h00d;
        page_rom[6344]=9'h00e;
        page_rom[6345]=9'h00f;
        page_rom[6346]=9'h010;
        page_rom[6347]=9'h011;
        page_rom[6348]=9'h012;
        page_rom[6349]=9'h013;
        page_rom[6350]=9'h020;
        page_rom[6351]=9'h001;
        page_rom[6352]=9'h002;
        page_rom[6353]=9'h003;
        page_rom[6354]=9'h004;
        page_rom[6355]=9'h005;
        page_rom[6356]=9'h006;
        page_rom[6357]=9'h007;
        page_rom[6358]=9'h020;
        page_rom[6359]=9'h053;
        page_rom[6360]=9'h045;
        page_rom[6361]=9'h051;
        page_rom[6362]=9'h098;
        page_rom[6363]=9'h099;
        page_rom[6364]=9'h09a;
        page_rom[6365]=9'h09b;
        page_rom[6366]=9'h09c;
        page_rom[6367]=9'h020;
        page_rom[6368]=9'h041;
        page_rom[6369]=9'h047;
        page_rom[6370]=9'h045;
        page_rom[6371]=9'h0a2;
        page_rom[6372]=9'h0a3;
        page_rom[6373]=9'h0a4;
        page_rom[6374]=9'h053;
        page_rom[6598]=9'h00c;
        page_rom[6599]=9'h00d;
        page_rom[6600]=9'h00e;
        page_rom[6601]=9'h00f;
        page_rom[6602]=9'h010;
        page_rom[6603]=9'h011;
        page_rom[6604]=9'h012;
        page_rom[6605]=9'h013;
        page_rom[6726]=9'h001;
        page_rom[6727]=9'h002;
        page_rom[6728]=9'h003;
        page_rom[6729]=9'h004;
        page_rom[6730]=9'h005;
        page_rom[6731]=9'h006;
        page_rom[6732]=9'h007;
        page_rom[6733]=9'h020;
        page_rom[6734]=9'h008;
        page_rom[6735]=9'h009;
        page_rom[6736]=9'h00a;
        page_rom[6737]=9'h00b;
        page_rom[6854]=9'h0c8;
        page_rom[6855]=9'h0c9;
        page_rom[6856]=9'h0ca;
        page_rom[6857]=9'h0cb;
        page_rom[6858]=9'h0cc;
        page_rom[6859]=9'h020;
        page_rom[6860]=9'h04d;
        page_rom[6861]=9'h049;
        page_rom[6862]=9'h04e;
        page_rom[7174]=9'h091;
        page_rom[7175]=9'h092;
        page_rom[7176]=9'h093;
        page_rom[7177]=9'h02e;
        page_rom[7178]=9'h094;
        page_rom[7179]=9'h043;
        page_rom[7301]=9'h102;
        page_rom[7302]=9'h103;
        page_rom[7303]=9'h104;
        page_rom[7304]=9'h020;
        page_rom[7305]=9'h1c0;
        page_rom[7306]=9'h1c1;
        page_rom[7307]=9'h1c2;
        page_rom[7308]=9'h1c3;
        page_rom[7309]=9'h1c4;
        page_rom[7310]=9'h1c5;
        page_rom[7311]=9'h1c6;
        page_rom[7312]=9'h1c7;
        page_rom[7490]=9'h056;
        page_rom[7491]=9'h049;
        page_rom[7492]=9'h042;
        page_rom[7493]=9'h102;
        page_rom[7494]=9'h103;
        page_rom[7495]=9'h104;
        page_rom[7496]=9'h02f;
        page_rom[7497]=9'h11b;
        page_rom[7498]=9'h11c;
        page_rom[7499]=9'h020;
        page_rom[7500]=9'h050;
        page_rom[7501]=9'h045;
        page_rom[7502]=9'h041;
        page_rom[7503]=9'h04b;
        page_rom[7504]=9'h108;
        page_rom[7505]=9'h109;
        page_rom[7506]=9'h10a;
        page_rom[7507]=9'h10b;
        page_rom[7508]=9'h10c;
        page_rom[7509]=9'h048;
        page_rom[7510]=9'h05a;
        page_rom[7511]=9'h020;
        page_rom[7512]=9'h04c;
        page_rom[7513]=9'h049;
        page_rom[7514]=9'h04e;
        page_rom[7515]=9'h04b;
        page_rom[7516]=9'h020;
        page_rom[7517]=9'h180;
        page_rom[7518]=9'h181;
        page_rom[7519]=9'h182;
        page_rom[7520]=9'h183;
        page_rom[7521]=9'h112;
        page_rom[7522]=9'h113;
        page_rom[7523]=9'h114;
        page_rom[7524]=9'h025;
        page_rom[8130]=9'h04c;
        page_rom[8131]=9'h049;
        page_rom[8132]=9'h04d;
        page_rom[8133]=9'h049;
        page_rom[8134]=9'h054;
        page_rom[8135]=9'h020;
        page_rom[8136]=9'h056;
        page_rom[8137]=9'h0e0;
        page_rom[8138]=9'h0e1;
        page_rom[8139]=9'h02e;
        page_rom[8140]=9'h0e2;
        page_rom[8141]=9'h0e3;
        page_rom[8142]=9'h02d;
        page_rom[8143]=9'h0e8;
        page_rom[8144]=9'h0e9;
        page_rom[8145]=9'h02e;
        page_rom[8146]=9'h0ea;
        page_rom[8147]=9'h0eb;
        page_rom[8148]=9'h056;
        page_rom[8149]=9'h020;
        page_rom[8150]=9'h049;
        page_rom[8151]=9'h03c;
        page_rom[8152]=9'h0f2;
        page_rom[8153]=9'h0f3;
        page_rom[8154]=9'h0f4;
        page_rom[8155]=9'h04d;
        page_rom[8156]=9'h041;
        page_rom[8157]=9'h020;
        page_rom[8158]=9'h054;
        page_rom[8159]=9'h03c;
        page_rom[8160]=9'h0f9;
        page_rom[8161]=9'h0fa;
        page_rom[8162]=9'h0fb;
        page_rom[8163]=9'h043;
        ban_rom[0]=9'h020;
        ban_rom[1]=9'h020;
        ban_rom[2]=9'h020;
        ban_rom[3]=9'h020;
        ban_rom[4]=9'h020;
        ban_rom[5]=9'h020;
        ban_rom[6]=9'h020;
        ban_rom[7]=9'h021;
        ban_rom[8]=9'h021;
        ban_rom[9]=9'h020;
        ban_rom[10]=9'h04d;
        ban_rom[11]=9'h041;
        ban_rom[12]=9'h04e;
        ban_rom[13]=9'h055;
        ban_rom[14]=9'h041;
        ban_rom[15]=9'h04c;
        ban_rom[16]=9'h020;
        ban_rom[17]=9'h041;
        ban_rom[18]=9'h04c;
        ban_rom[19]=9'h041;
        ban_rom[20]=9'h052;
        ban_rom[21]=9'h04d;
        ban_rom[22]=9'h020;
        ban_rom[23]=9'h020;
        ban_rom[24]=9'h053;
        ban_rom[25]=9'h057;
        ban_rom[26]=9'h033;
        ban_rom[27]=9'h020;
        ban_rom[28]=9'h04f;
        ban_rom[29]=9'h04e;
        ban_rom[30]=9'h020;
        ban_rom[31]=9'h021;
        ban_rom[32]=9'h021;
        ban_rom[64]=9'h020;
        ban_rom[65]=9'h020;
        ban_rom[66]=9'h020;
        ban_rom[67]=9'h020;
        ban_rom[68]=9'h020;
        ban_rom[69]=9'h053;
        ban_rom[70]=9'h049;
        ban_rom[71]=9'h052;
        ban_rom[72]=9'h045;
        ban_rom[73]=9'h04e;
        ban_rom[74]=9'h020;
        ban_rom[75]=9'h04f;
        ban_rom[76]=9'h04e;
        ban_rom[77]=9'h020;
        ban_rom[78]=9'h02d;
        ban_rom[79]=9'h020;
        ban_rom[80]=9'h053;
        ban_rom[81]=9'h045;
        ban_rom[82]=9'h054;
        ban_rom[83]=9'h020;
        ban_rom[84]=9'h053;
        ban_rom[85]=9'h057;
        ban_rom[86]=9'h033;
        ban_rom[87]=9'h020;
        ban_rom[88]=9'h04f;
        ban_rom[89]=9'h046;
        ban_rom[90]=9'h046;
        ban_rom[91]=9'h020;
        ban_rom[92]=9'h054;
        ban_rom[93]=9'h04f;
        ban_rom[94]=9'h020;
        ban_rom[95]=9'h053;
        ban_rom[96]=9'h054;
        ban_rom[97]=9'h04f;
        ban_rom[98]=9'h050;
        ban_rom[128]=9'h020;
        ban_rom[129]=9'h020;
        ban_rom[130]=9'h020;
        ban_rom[131]=9'h020;
        ban_rom[132]=9'h020;
        ban_rom[133]=9'h020;
        ban_rom[134]=9'h020;
        ban_rom[135]=9'h020;
        ban_rom[136]=9'h057;
        ban_rom[137]=9'h041;
        ban_rom[138]=9'h052;
        ban_rom[139]=9'h04e;
        ban_rom[140]=9'h049;
        ban_rom[141]=9'h04e;
        ban_rom[142]=9'h047;
        ban_rom[143]=9'h03a;
        ban_rom[144]=9'h020;
        ban_rom[145]=9'h044;
        ban_rom[146]=9'h041;
        ban_rom[147]=9'h054;
        ban_rom[148]=9'h041;
        ban_rom[149]=9'h020;
        ban_rom[150]=9'h04c;
        ban_rom[151]=9'h049;
        ban_rom[152]=9'h04e;
        ban_rom[153]=9'h04b;
        ban_rom[154]=9'h020;
        ban_rom[155]=9'h04c;
        ban_rom[156]=9'h04f;
        ban_rom[157]=9'h053;
        ban_rom[158]=9'h054;
        ban_rom[192]=9'h020;
        ban_rom[193]=9'h020;
        ban_rom[194]=9'h020;
        ban_rom[195]=9'h020;
        ban_rom[196]=9'h020;
        ban_rom[197]=9'h020;
        ban_rom[198]=9'h020;
        ban_rom[199]=9'h020;
        ban_rom[200]=9'h04e;
        ban_rom[201]=9'h04f;
        ban_rom[202]=9'h020;
        ban_rom[203]=9'h056;
        ban_rom[204]=9'h041;
        ban_rom[205]=9'h04c;
        ban_rom[206]=9'h049;
        ban_rom[207]=9'h044;
        ban_rom[208]=9'h020;
        ban_rom[209]=9'h050;
        ban_rom[210]=9'h041;
        ban_rom[211]=9'h043;
        ban_rom[212]=9'h04b;
        ban_rom[213]=9'h045;
        ban_rom[214]=9'h054;
        ban_rom[215]=9'h020;
        ban_rom[216]=9'h046;
        ban_rom[217]=9'h04f;
        ban_rom[218]=9'h052;
        ban_rom[219]=9'h0a2;
        ban_rom[220]=9'h0a3;
        ban_rom[221]=9'h0a4;
        ban_rom[222]=9'h053;
        ban_rom[256]=9'h020;
        ban_rom[257]=9'h020;
        ban_rom[258]=9'h020;
        ban_rom[259]=9'h020;
        ban_rom[260]=9'h020;
        ban_rom[261]=9'h020;
        ban_rom[262]=9'h020;
        ban_rom[263]=9'h057;
        ban_rom[264]=9'h041;
        ban_rom[265]=9'h052;
        ban_rom[266]=9'h04e;
        ban_rom[267]=9'h049;
        ban_rom[268]=9'h04e;
        ban_rom[269]=9'h047;
        ban_rom[270]=9'h03a;
        ban_rom[271]=9'h020;
        ban_rom[272]=9'h043;
        ban_rom[273]=9'h048;
        ban_rom[274]=9'h041;
        ban_rom[275]=9'h04d;
        ban_rom[276]=9'h042;
        ban_rom[277]=9'h045;
        ban_rom[278]=9'h052;
        ban_rom[279]=9'h020;
        ban_rom[280]=9'h054;
        ban_rom[281]=9'h045;
        ban_rom[282]=9'h04d;
        ban_rom[283]=9'h050;
        ban_rom[284]=9'h020;
        ban_rom[285]=9'h048;
        ban_rom[286]=9'h049;
        ban_rom[287]=9'h047;
        ban_rom[288]=9'h048;
        ban_rom[320]=9'h020;
        ban_rom[321]=9'h020;
        ban_rom[322]=9'h020;
        ban_rom[323]=9'h020;
        ban_rom[324]=9'h020;
        ban_rom[325]=9'h020;
        ban_rom[326]=9'h020;
        ban_rom[327]=9'h020;
        ban_rom[328]=9'h020;
        ban_rom[329]=9'h054;
        ban_rom[330]=9'h045;
        ban_rom[331]=9'h04d;
        ban_rom[332]=9'h050;
        ban_rom[333]=9'h091;
        ban_rom[334]=9'h092;
        ban_rom[335]=9'h093;
        ban_rom[336]=9'h02e;
        ban_rom[337]=9'h094;
        ban_rom[338]=9'h043;
        ban_rom[339]=9'h020;
        ban_rom[340]=9'h020;
        ban_rom[341]=9'h04c;
        ban_rom[342]=9'h049;
        ban_rom[343]=9'h04d;
        ban_rom[344]=9'h049;
        ban_rom[345]=9'h054;
        ban_rom[346]=9'h0f9;
        ban_rom[347]=9'h0fa;
        ban_rom[348]=9'h0fb;
        ban_rom[349]=9'h043;
        ban_rom[384]=9'h020;
        ban_rom[385]=9'h020;
        ban_rom[386]=9'h020;
        ban_rom[387]=9'h020;
        ban_rom[388]=9'h057;
        ban_rom[389]=9'h041;
        ban_rom[390]=9'h052;
        ban_rom[391]=9'h04e;
        ban_rom[392]=9'h049;
        ban_rom[393]=9'h04e;
        ban_rom[394]=9'h047;
        ban_rom[395]=9'h03a;
        ban_rom[396]=9'h020;
        ban_rom[397]=9'h050;
        ban_rom[398]=9'h04f;
        ban_rom[399]=9'h057;
        ban_rom[400]=9'h045;
        ban_rom[401]=9'h052;
        ban_rom[402]=9'h02f;
        ban_rom[403]=9'h04c;
        ban_rom[404]=9'h04f;
        ban_rom[405]=9'h041;
        ban_rom[406]=9'h044;
        ban_rom[407]=9'h020;
        ban_rom[408]=9'h04f;
        ban_rom[409]=9'h055;
        ban_rom[410]=9'h054;
        ban_rom[411]=9'h020;
        ban_rom[412]=9'h04f;
        ban_rom[413]=9'h046;
        ban_rom[414]=9'h020;
        ban_rom[415]=9'h052;
        ban_rom[416]=9'h041;
        ban_rom[417]=9'h04e;
        ban_rom[418]=9'h047;
        ban_rom[419]=9'h045;
        ban_rom[448]=9'h020;
        ban_rom[449]=9'h020;
        ban_rom[450]=9'h020;
        ban_rom[451]=9'h020;
        ban_rom[452]=9'h020;
        ban_rom[453]=9'h020;
        ban_rom[454]=9'h020;
        ban_rom[455]=9'h020;
        ban_rom[456]=9'h020;
        ban_rom[457]=9'h020;
        ban_rom[458]=9'h020;
        ban_rom[459]=9'h056;
        ban_rom[460]=9'h080;
        ban_rom[461]=9'h081;
        ban_rom[462]=9'h02e;
        ban_rom[463]=9'h082;
        ban_rom[464]=9'h083;
        ban_rom[465]=9'h084;
        ban_rom[466]=9'h056;
        ban_rom[467]=9'h020;
        ban_rom[468]=9'h020;
        ban_rom[469]=9'h049;
        ban_rom[470]=9'h089;
        ban_rom[471]=9'h08a;
        ban_rom[472]=9'h08b;
        ban_rom[473]=9'h08c;
        ban_rom[474]=9'h04d;
        ban_rom[475]=9'h041;
        ban_rom[512]=9'h020;
        ban_rom[513]=9'h020;
        ban_rom[514]=9'h020;
        ban_rom[515]=9'h020;
        ban_rom[516]=9'h020;
        ban_rom[517]=9'h020;
        ban_rom[518]=9'h020;
        ban_rom[519]=9'h020;
        ban_rom[520]=9'h057;
        ban_rom[521]=9'h041;
        ban_rom[522]=9'h052;
        ban_rom[523]=9'h04e;
        ban_rom[524]=9'h049;
        ban_rom[525]=9'h04e;
        ban_rom[526]=9'h047;
        ban_rom[527]=9'h03a;
        ban_rom[528]=9'h020;
        ban_rom[529]=9'h056;
        ban_rom[530]=9'h049;
        ban_rom[531]=9'h042;
        ban_rom[532]=9'h052;
        ban_rom[533]=9'h041;
        ban_rom[534]=9'h054;
        ban_rom[535]=9'h049;
        ban_rom[536]=9'h04f;
        ban_rom[537]=9'h04e;
        ban_rom[538]=9'h020;
        ban_rom[539]=9'h048;
        ban_rom[540]=9'h049;
        ban_rom[541]=9'h047;
        ban_rom[542]=9'h048;
        ban_rom[576]=9'h020;
        ban_rom[577]=9'h020;
        ban_rom[578]=9'h020;
        ban_rom[579]=9'h020;
        ban_rom[580]=9'h020;
        ban_rom[581]=9'h020;
        ban_rom[582]=9'h020;
        ban_rom[583]=9'h048;
        ban_rom[584]=9'h046;
        ban_rom[585]=9'h102;
        ban_rom[586]=9'h103;
        ban_rom[587]=9'h104;
        ban_rom[588]=9'h020;
        ban_rom[589]=9'h04c;
        ban_rom[590]=9'h049;
        ban_rom[591]=9'h04d;
        ban_rom[592]=9'h049;
        ban_rom[593]=9'h054;
        ban_rom[594]=9'h11b;
        ban_rom[595]=9'h11c;
        ban_rom[596]=9'h020;
        ban_rom[597]=9'h020;
        ban_rom[598]=9'h050;
        ban_rom[599]=9'h045;
        ban_rom[600]=9'h041;
        ban_rom[601]=9'h04b;
        ban_rom[602]=9'h108;
        ban_rom[603]=9'h109;
        ban_rom[604]=9'h10a;
        ban_rom[605]=9'h10b;
        ban_rom[606]=9'h10c;
        ban_rom[607]=9'h048;
        ban_rom[608]=9'h05a;
        // ==== ROM END ====
    end
    reg [8:0] pcode,bcode;
    reg ban_q,in_q;
    always @(posedge clk) begin
        pcode<=page_rom[{profile,(row_p==5'd21 && detail) ? 5'd31 : row_p,col_p}];
        bcode<=ban_rom[{variant,row_p==5'd14,col_p}];
        ban_q<=(row_p==5'd13 || row_p==5'd14) && col_p<40;
        in_q<=col_p<40 && row_p<30;
    end

    // ---------------- 当前拍：解码 ----------------
    wire ban_on=banner && !menu && ban_q;
    assign banner_on=banner && !menu;
    wire page_on=!hide && !menu && !clean && in_q;
    wire [8:0] code=ban_on ? bcode : page_on ? pcode : 9'h000;
    wire ext=code[8];
    wire isnum=ext ? code[7:6]==2'b00 : code[7];
    wire islink=ext && code[7:6]==2'b10;
    wire isvbar=ext && code[7:6]==2'b11;
    wire isbar=!ext && code[7:5]==3'b011;

    // 数值：前导零按字段规则变空格，无数据显示 "-"
    wire [4:0] nf=ext ? {2'b10,code[5:3]} : {1'b0,code[6:3]};
    wire [2:0] nd=code[2:0];
    wire [19:0] nv=num[nf];
    reg [3:0] dig;
    always @* case(nd) 0:dig=nv[19:16]; 1:dig=nv[15:12]; 2:dig=nv[11:8]; 3:dig=nv[7:4]; default:dig=nv[3:0]; endcase
    reg [2:0] keep;                       // 前 keep 位允许消零
    always @* case(nf)
        0,12,13:keep=1; 2,8,15:keep=3; 3,5,6:keep=0; default:keep=4;
    endcase
    wire lz0=nv[19:16]==0, lz1=lz0 && nv[15:12]==0, lz2=lz1 && nv[11:8]==0, lz3=lz2 && nv[7:4]==0;
    reg lz;
    always @* case(nd) 0:lz=lz0; 1:lz=lz1; 2:lz=lz2; 3:lz=lz3; default:lz=0; endcase
    wire nblank=nd<keep && lz;
    wire ndash=(nf<=4 && !valid) || (nf>=5 && nf<=8 && !ctx) || (nf==17 && !peak_ok) || (nf==18 && link_state==0);

    // 柱条：段 s 点亮条件 val*8 >= lim*(s+1)，不用除法
    wire [1:0] bi=code[4:3];
    wire [2:0] bs=code[2:0];
    reg [15:0] bval,blim;
    reg bok;
    always @* begin
        if(isvbar) begin bval={9'd0,vib}; blim={9'd0,vib_lim}; bok=1'b1; end
        else case(bi)
            0:begin bval=temp; blim=tmax; bok=valid; end
            1:begin bval=ma; blim=imax; bok=valid; end
            2:begin bval=wafer; blim=16'd25; bok=ctx; end
            default:begin bval=yield; blim=16'd1000; bok=ctx; end
        endcase
    end
    wire [19:0] bth=blim*({1'b0,bs}+4'd1);
    wire bfill=bok && {1'b0,bval,3'b000}>=bth;
    wire bover=bok && (isvbar || !bi[1]) && bval>=blim;

    wire [55:0] st_txt=stale ? "STALE  " : !valid ? "NO DATA" : warn ? "WARNING" : failing ? "OVER LM" : "NORMAL ";
    wire [31:0] md_txt=demo ? "DEMO" : "LIVE";
    wire [31:0] lk_txt=link_state==2'd1 ? " OK " : link_state==2'd2 ? "LOST" : "IDLE";
    reg [63:0] tn;
    always @* case(tool) 1:tn="ETCH-01 "; 2:tn="CVD-02  "; 3:tn="CMP-03  "; 4:tn="LITHO-04"; default:tn="--------"; endcase
    wire [2:0] sk=code[2:0]-3'd1;         // 001~007
    wire [1:0] mk=code[1:0];              // 008~00B / 180~183
    wire [2:0] tk=code[2:0]-3'd4;         // 00C~013
    wire [2:0] dk=code[2:0];              // 018~01F

    localparam [23:0] C_LABEL=24'hb8d8f0, C_VAL=24'h60f0f0, C_WHITE=24'hffffff, C_GRAY=24'h9098a0,
                      C_AMBER=24'hffb040, C_GREEN=24'h50f080, C_RED=24'hff4040, C_EMPTY=24'h304858;
    reg [7:0] ch;
    reg [23:0] fg;
    reg solid,mark,mfill;
    always @* begin
        ch=8'h20; fg=C_LABEL; solid=0; mark=0; mfill=0;
        if(isnum) begin
            ch=ndash ? "-" : nblank ? " " : 8'd48+{4'd0,dig};
            fg=(nf<=2 && stale) ? C_GRAY :
               ((nf==0 && v_bad) || (nf==1 && i_bad) || (nf==2 && t_bad) || (nf==16 && vib_bad)) ? C_AMBER :
               ban_on ? C_WHITE : C_VAL;
        end else if(isbar || isvbar) begin
            solid=1;
            fg=!bfill ? C_EMPTY : bover ? C_RED : (!isvbar && bi==3) ? C_GREEN : C_VAL;
        end else if(islink) begin
            ch=lk_txt[(3-mk)*8 +: 8];
            fg=link_state==2'd1 ? C_GREEN : link_state==2'd2 ? C_AMBER : C_GRAY;
        end else if(code[7:0]>=8'h20) begin
            ch=code[7:0]; fg=ban_on ? C_WHITE : C_LABEL;
        end else if(code[7:0]>=8'h18) begin
            mark=1; mfill=ctx && defects>{13'd0,dk}; fg=C_RED;
        end else if(code[7:0]>=8'h0c && code[7:0]<=8'h13) begin
            ch=tn[(7-tk)*8 +: 8]; fg=C_VAL;
        end else if(code[7:0]>=8'h08) begin
            ch=md_txt[(3-mk)*8 +: 8]; fg=demo ? C_AMBER : C_GREEN;
        end else if(code!=0) begin
            ch=st_txt[(6-sk)*8 +: 8];
            fg=(stale || !valid) ? C_GRAY : (warn || failing) ? C_AMBER : C_GREEN;
        end
    end
    // 5x7 点阵放大 2 倍，在 16x16 格子里居中（列 3~12，行 1~14）
    wire [3:0] cx=x[3:0]-4'd3, cy=y[3:0]-4'd1;
    wire in_glyph=x[3:0]>=3 && x[3:0]<13 && y[3:0]>=1 && y[3:0]<15;
    wire [4:0] grow=glyph_row(ch,cy[3:1]);
    wire ink=code!=0 && (
        (solid && x[3:0]>=1 && x[3:0]<15 && y[3:0]>=3 && y[3:0]<13) ||
        (mark && mfill && x[3:0]>=4 && x[3:0]<12 && y[3:0]>=4 && y[3:0]<12) ||
        (!solid && !mark && in_glyph && grow[3'd4-cx[3:1]]));
    wire shadow=code!=0 && !(mark && !mfill);
    wire border=manual && blink[4] && (x<8 || x>=632 || y<8 || y>=472);
    wire [23:0] half={1'b0,rgb_in[23:17],1'b0,rgb_in[15:9],1'b0,rgb_in[7:1]};
    wire [23:0] tint=manual ? 24'h500000 : 24'h402800;
    assign rgb_out=!de ? rgb_in : border ? C_RED : ink ? fg : ban_on ? half+tint : shadow ? half : rgb_in;
endmodule
