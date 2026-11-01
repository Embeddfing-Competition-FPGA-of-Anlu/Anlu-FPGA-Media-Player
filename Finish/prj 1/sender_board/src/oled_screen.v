`timescale 1ns/1ps
// 控制板 OLED 操作屏：128x64 = 8 行 x 21 字（6x8 字格）
//   第 0 行      模式 + 链路 + 开关（A=SW0 自动发送 C=SW1 CRC N=SW2 噪声 W=SW3 自动走片，* = 正在发送）
//   第 1 行反色  与数码管联动："7SEG>" + 数码管当前显示量的名称、数值、单位（数码管只能显示 4 位，这里给全称）
//   第 2~3 行    当前模式的关键数据
//   第 4~7 行    4x4 键盘图，位置与实物键一一对应，标签就是按下去的作用（随拨码模式自动切换）：
//                ">"  按下会改变数码管当前显示量的键
//                反色 当前生效的选项/开关（算法、故障注入、自动云台）
//                闪烁 需要处理（阈值改了还没下发、云台等人工确认）
//                刚按下的键反色 0.3 秒，作为按键确认
//   SW5 摄像头诊断时第 2~7 行一次列出全部诊断值，不用逐页翻
//   像素接口：驱动给 pix_page/pix_col，4 拍后 pix_data 有效
module oled_screen (
    input  wire        clk, rst_n,
    input  wire [2:0]  pix_page, input wire [6:0] pix_col,
    output reg  [7:0]  pix_data,
    input  wire        blink, kflash,
    input  wire [3:0]  lastkey,
    input  wire        test_mode, cam_mode, vis_mode,
    input  wire        auto_on, crc_on, noise_on, auto_wafer, fault, tx_act, thr_dirty,
    input  wire [1:0]  link_st,                       // 0 WAIT 1 OK 2 ERR 3 LOST
    input  wire [4:0]  item,                          // 数码管当前显示量（与 sender_top 编号一致）
    input  wire [15:0] dshow, input wire [1:0] sdp, input wire stw, cam_bad,
    input  wire [1:0]  cam_st, input wire [15:0] cam_id, input wire [7:0] cam_dmask,
    input  wire [15:0] mv, ma, t_tx, wafer, lot, defects,
    input  wire [2:0]  tool, input wire [1:0] lsel,
    input  wire [2:0]  vmode, input wire [7:0] vthr, pan, tilt,
    input  wire        armed, input wire [3:0] gstate, gerror, gcal,
    input  wire [7:0]  cam_fps, cam_vps, cam_sps,
    input  wire [11:0] cam_vlines, cam_hlen, cam_pmhz,
    input  wire [6:0]  hitp,
    input  wire [2:0]  ksel,                          // 卷积核 0 锐化 1 降噪 2 轮廓 3 浮雕 4 CNN
    input  wire [1:0]  gmode,                         // 云台自动模式 bit0 巡检 bit1 闯入联动
    input  wire [6:0]  zp0, zp1, zp2,                 // A/B/C 工位区运动占比 %
    input  wire [2:0]  zact,                          // 正在报警（去抖后）的工位
    input  wire        gintr,                         // 当前云台任务来自本地闯入
    input  wire [2:0]  gtool                          // 当前任务 / 巡检指向的工位
);
    localparam [4:0] I_V=0, I_I=1, I_T=2, I_TW=3, I_DEF=4, I_LOT=5, I_VLO=6, I_VHI=7, I_IHI=8, I_THI=9, I_OK=10, I_TST=11,
                     I_CAM=12, I_VPS=13, I_LIN=14, I_HL=15, I_PCK=16, I_DAT=17, I_ID=18, I_SND=19,
                     I_VIS=20, I_THR=21, I_PAN=22, I_TLT=23, I_GIM=24, I_CAL=25;
    localparam [7:0] SP = 8'h20, DASH = 8'h2D, DOT = 8'h2E, GT = 8'h3E;

    // ---------------- 共用串行二进制 -> BCD：16 个量轮流转换，每个 17 拍 ----------------
    reg [3:0] bidx; reg [4:0] bcnt; reg [15:0] bbin, bsrc; reg [19:0] bacc, badj; reg bload; reg [319:0] bcdv; integer k;
    always @* case (bidx)
        4'd0: bsrc = mv;               4'd1: bsrc = ma;               4'd2: bsrc = t_tx;             4'd3: bsrc = wafer;
        4'd4: bsrc = lot;              4'd5: bsrc = defects;          4'd6: bsrc = {8'd0, vthr};     4'd7: bsrc = {9'd0, zp0};
        4'd8: bsrc = {9'd0, zp1};     4'd9: bsrc = {8'd0, cam_fps};  4'd10: bsrc = {8'd0, cam_vps}; 4'd11: bsrc = {4'd0, cam_vlines};
        4'd12: bsrc = {4'd0, cam_hlen}; 4'd13: bsrc = {4'd0, cam_pmhz}; 4'd14: bsrc = {8'd0, cam_sps}; default: bsrc = {9'd0, zp2};
    endcase
    always @* begin badj = bacc; for (k = 0; k < 5; k = k + 1) if (bacc[k*4 +: 4] >= 4'd5) badj[k*4 +: 4] = bacc[k*4 +: 4] + 4'd3; end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin bidx <= 0; bcnt <= 0; bbin <= 0; bacc <= 0; bload <= 1'b1; bcdv <= 0; end
        else if (bload) begin bbin <= bsrc; bacc <= 0; bcnt <= 5'd16; bload <= 1'b0; end
        else if (bcnt != 0) begin bacc <= {badj[18:0], bbin[15]}; bbin <= {bbin[14:0], 1'b0}; bcnt <= bcnt - 1'b1; end
        else begin bcdv[bidx*20 +: 20] <= bacc; bidx <= bidx + 1'b1; bload <= 1'b1; end
    // ---------------- 格式化 ----------------
    function [7:0] dc; input [3:0] n; dc = 8'h30 + {4'd0, n}; endfunction
    function [7:0] hx; input [3:0] n; hx = (n < 4'd10) ? 8'h30 + {4'd0, n} : 8'h37 + {4'd0, n}; endfunction
    // 5 位右对齐，消前导零（个位总显示）
    function [39:0] d5; input [19:0] b; reg z4, z3, z2, z1;
        begin
            z4 = b[19:16] == 0; z3 = z4 && b[15:12] == 0; z2 = z3 && b[11:8] == 0; z1 = z2 && b[7:4] == 0;
            d5 = {z4 ? SP : dc(b[19:16]), z3 ? SP : dc(b[15:12]), z2 ? SP : dc(b[11:8]), z1 ? SP : dc(b[7:4]), dc(b[3:0])};
        end
    endfunction
    wire [19:0] b_mv = bcdv[0 +: 20],   b_ma = bcdv[20 +: 20],  b_t = bcdv[40 +: 20],   b_wf = bcdv[60 +: 20];
    wire [19:0] b_lot = bcdv[80 +: 20], b_def = bcdv[100 +: 20], b_thr = bcdv[120 +: 20], b_z0 = bcdv[140 +: 20];
    wire [19:0] b_z1 = bcdv[160 +: 20], b_fps = bcdv[180 +: 20], b_vps = bcdv[200 +: 20], b_lin = bcdv[220 +: 20];
    wire [19:0] b_hl = bcdv[240 +: 20], b_pck = bcdv[260 +: 20], b_sps = bcdv[280 +: 20], b_z2 = bcdv[300 +: 20];
    wire [39:0] f_ma = d5(b_ma), f_lot = d5(b_lot), f_def = d5(b_def), f_thr = d5(b_thr), f_z0 = d5(b_z0), f_z1 = d5(b_z1);
    wire [39:0] f_fps = d5(b_fps), f_vps = d5(b_vps), f_lin = d5(b_lin), f_hl = d5(b_hl), f_sps = d5(b_sps), f_z2 = d5(b_z2);
    // 电压 mV：<10V "d.ddd"，>=10V "dd.dd"
    wire [39:0] f_v = (b_mv[19:16] != 0) ? {dc(b_mv[19:16]), dc(b_mv[15:12]), DOT, dc(b_mv[11:8]), dc(b_mv[7:4])}
                                         : {dc(b_mv[15:12]), DOT, dc(b_mv[11:8]), dc(b_mv[7:4]), dc(b_mv[3:0])};
    // 温度 0.1C：" dd.d" / "ddd.d"
    wire [39:0] f_t = {(b_t[15:12] == 0) ? SP : dc(b_t[15:12]), (b_t[15:8] == 0) ? SP : dc(b_t[11:8]), dc(b_t[7:4]), DOT, dc(b_t[3:0])};
    // PCLK 0.1MHz："ddd.d"
    wire [39:0] f_pck = {(b_pck[15:12] == 0) ? SP : dc(b_pck[15:12]), (b_pck[15:8] == 0) ? SP : dc(b_pck[11:8]), dc(b_pck[7:4]), DOT, dc(b_pck[3:0])};
    wire [15:0] f_wf = {dc(b_wf[7:4]), dc(b_wf[3:0])};                  // 片号两位带前导零

    // ---------------- 数码管镜像：与 sender_top 的消隐 / 小数点 / 设备-片号 / 十六进制规则相同 ----------------
    wire [3:0] n0 = dshow[15:12], n1 = dshow[11:8], n2 = dshow[7:4], n3 = dshow[3:0];
    wire sz0 = n0 == 0, sz1 = sz0 && n1 == 0, sz2 = sz1 && n2 == 0;
    wire [1:0] blim = (sdp == 0) ? 2'd3 : sdp - 1'b1;
    wire [7:0] c0 = (!stw && blim > 0 && sz0) ? SP : dc(n0);
    wire [7:0] c1 = stw ? DASH : (blim > 1 && sz1) ? SP : dc(n1);
    wire [7:0] c2 = (!stw && blim > 2 && sz2) ? SP : dc(n2);
    wire [7:0] c3 = dc(n3);
    reg [39:0] f_seg;                                                     // 5 字符：4 位数字 + 可能的小数点
    always @* begin
        if (cam_bad) f_seg = (cam_st == 2'd2) ? {SP, "ERR", SP} : {SP, "----"};
        else if (item == I_ID) f_seg = {SP, hx(cam_id[15:12]), hx(cam_id[11:8]), hx(cam_id[7:4]), hx(cam_id[3:0])};
        else if (item == I_DAT) f_seg = {SP, SP, SP, hx(cam_dmask[7:4]), hx(cam_dmask[3:0])};
        else case (sdp)
            2'd1: f_seg = {c0, DOT, c1, c2, c3};
            2'd2: f_seg = {c0, c1, DOT, c2, c3};
            2'd3: f_seg = {c0, c1, c2, DOT, c3};
            default: f_seg = {SP, c0, c1, c2, c3};
        endcase
    end
    // 数码管显示量的全称（5 字）+ 单位（4 字）
    reg [71:0] nu;
    always @* case (item)
        I_V:   nu = {"VOLT ", "V   "};   I_I:   nu = {"CURR ", "mA  "};   I_T:   nu = {"TEMP ", "C   "};
        I_TW:  nu = {"TL-WF", "    "};   I_DEF: nu = {"DEFCT", "PCS "};   I_LOT: nu = {"LOT  ", "    "};
        I_VLO: nu = {"V-LO ", "V   "};   I_VHI: nu = {"V-HI ", "V   "};   I_IHI: nu = {"I-HI ", "mA  "};
        I_THI: nu = {"T-HI ", "C   "};   I_OK:  nu = {"OKCNT", "    "};   I_CAM: nu = {"CAPT ", "FPS "};
        I_VPS: nu = {"VSYNC", "/S  "};   I_LIN: nu = {"LINES", "    "};   I_HL:  nu = {"HLEN ", "PCLK"};
        I_PCK: nu = {"PCLK ", "MHZ "};   I_DAT: nu = {"DATA ", "HEX "};   I_ID:  nu = {"CHIP ", "HEX "};
        I_SND: nu = {"SEND ", "FPS "};   I_THR: nu = {"SENS ", "%   "};   I_PAN: nu = {"PAN  ", "DEG "};
        I_TLT: nu = {"TILT ", "DEG "};   I_GIM: nu = {"TASK ", "    "};   I_CAL: nu = {"CAL  ", "MASK"};
        I_VIS: case (vmode)
                   3'd1: nu = {"EDGE ", "%   "};  3'd2: case (ksel) 3'd0: nu = {"SHARP", "FPS "}; 3'd1: nu = {"BLUR ", "FPS "}; 3'd2: nu = {"LAPL ", "%   "};
                                    3'd3: nu = {"EMBOS", "FPS "}; default: nu = {"CNN  ", "%   "}; endcase  3'd3: nu = {"MOVE ", "%   "};
                   default: nu = {"SRC  ", "FPS "};
               endcase
        default: nu = {"TEST ", "    "};
    endcase

    // 键盘图：每键 4 字标签
    reg [23:0] lname;
    always @* case (lsel) 2'd0: lname = "VLO"; 2'd1: lname = "VHI"; 2'd2: lname = "IHI"; default: lname = "THI"; endcase
    function [31:0] klab; input [3:0] k; input v, t; input [23:0] ln;
        begin
            if (t) klab = {"K", (k >= 4'd10) ? 8'h31 : 8'h30, dc((k >= 4'd10) ? k - 4'd10 : k), SP};
            else if (v) case (k)
                0: klab = "SRC ";  1: klab = "EDGE";  2: klab = "CONV";  3: klab = "MOVE";
                4: klab = "MODE";  5: klab = "SEN-";  6: klab = "SEN+";  7: klab = "AUTO";
                8: klab = "ACK ";  9: klab = "TLT+"; 10: klab = "TLT-"; 11: klab = "TASK";
               12: klab = "PAN-"; 13: klab = "HOME"; 14: klab = "PAN+"; default: klab = "SAVE";
            endcase
            else case (k)
                0: klab = "VOL+";  1: klab = "VOL-";  2: klab = "CUR+";  3: klab = "CUR-";
                4: klab = "TMP+";  5: klab = "TMP-";  6: klab = "TOOL";  7: klab = "FALT";
                8: klab = "WAF+";  9: klab = "DEF+"; 10: klab = "LOT+"; 11: klab = "SEND";
               12: klab = "LIM "; 13: klab = {"+", ln}; 14: klab = {"-", ln}; default: klab = "APLY";
            endcase
        end
    endfunction
    // ">"：会改变数码管当前显示量的键
    reg [15:0] kmark;
    always @* if (test_mode || cam_mode) kmark = 16'h0; else case (item)
        I_V: kmark = 16'h0003;   I_I: kmark = 16'h000C;   I_T: kmark = 16'h00B0;   I_TW: kmark = 16'h0540;
        I_DEF: kmark = 16'h0200; I_LOT: kmark = 16'h0500; I_VLO, I_VHI, I_IHI, I_THI: kmark = 16'h7000;
        I_VIS: kmark = 16'h000F; I_THR: kmark = 16'h0060; I_PAN: kmark = 16'h7000; I_TLT: kmark = 16'h2600;
        I_GIM: kmark = 16'h0890; I_CAL: kmark = 16'h8000; default: kmark = 16'h0;
    endcase
    // 反色：当前生效 / 待处理（闪烁）/ 刚按下（翻转）
    reg [15:0] kpers;
    always @* begin
        kpers = 16'h0;
        if (test_mode) kpers[lastkey] = 1'b1;
        else if (vis_mode) begin
            if (vmode <= 3'd3) kpers[vmode] = 1'b1;
            kpers[4] = gmode != 2'd0;                                    // 云台自动模式开着
            kpers[7] = armed;
            kpers[8] = (gstate == 4'd5) && blink;                        // 待人工确认
        end else begin
            kpers[7] = fault;
            kpers[15] = thr_dirty && blink;                              // 阈值改了还没下发
        end
    end
    wire [15:0] kflm = (kflash && !test_mode) ? (16'h1 << lastkey) : 16'h0;
    wire [15:0] kinv = kpers | kflm;
    // 16 个键格，每格 5 字：标记 + 4 字标签；cells 高位是 0 号键
    wire [639:0] cells;
    genvar gi;
    generate for (gi = 0; gi < 16; gi = gi + 1) begin : g_cell
        wire [7:0] mk = kmark[gi] ? GT : SP;
        assign cells[(15-gi)*40 +: 40] = {mk, klab(gi, vis_mode, test_mode, lname)};
    end endgenerate
    // 每行 21 字：空格 + 4 个键格；反色只盖标签 4 字（bit 20 = 第 0 字）
    wire [167:0] krow0 = {SP, cells[600 +: 40], cells[560 +: 40], cells[520 +: 40], cells[480 +: 40]};
    wire [167:0] krow1 = {SP, cells[440 +: 40], cells[400 +: 40], cells[360 +: 40], cells[320 +: 40]};
    wire [167:0] krow2 = {SP, cells[280 +: 40], cells[240 +: 40], cells[200 +: 40], cells[160 +: 40]};
    wire [167:0] krow3 = {SP, cells[120 +: 40], cells[80 +: 40],  cells[40 +: 40],  cells[0 +: 40]};
    wire [20:0] kinv0 = {2'b00, {4{kinv[0]}},  1'b0, {4{kinv[1]}},  1'b0, {4{kinv[2]}},  1'b0, {4{kinv[3]}}};
    wire [20:0] kinv1 = {2'b00, {4{kinv[4]}},  1'b0, {4{kinv[5]}},  1'b0, {4{kinv[6]}},  1'b0, {4{kinv[7]}}};
    wire [20:0] kinv2 = {2'b00, {4{kinv[8]}},  1'b0, {4{kinv[9]}},  1'b0, {4{kinv[10]}}, 1'b0, {4{kinv[11]}}};
    wire [20:0] kinv3 = {2'b00, {4{kinv[12]}}, 1'b0, {4{kinv[13]}}, 1'b0, {4{kinv[14]}}, 1'b0, {4{kinv[15]}}};

    // ---------------- 各行文字（每行恰好 21 字） ----------------
    reg [31:0] mtxt, ltxt; reg [47:0] sname; reg [167:0] crow5;
    always @* begin
        if (test_mode) mtxt = "TEST"; else if (cam_mode) mtxt = "CAM "; else if (vis_mode) mtxt = "VIS "; else mtxt = "DATA";
        case (link_st) 2'd1: ltxt = "OK  "; 2'd2: ltxt = "ERR "; 2'd3: ltxt = "LOST"; default: ltxt = "WAIT"; endcase
        case (gstate)
            4'd0: sname = "OFF   "; 4'd1: sname = "IDLE  "; 4'd2: sname = "AIM   "; 4'd3: sname = "WAIT  "; 4'd4: sname = "PATROL";
            4'd5: sname = "REVIEW"; 4'd6: sname = "ACKED "; 4'd7: sname = "HOME  "; 4'd8: sname = "FAIL  ";
            4'd9: sname = "MANUAL"; default: sname = "?     ";
        endcase
        case (cam_st)
            2'd1: crow5 = "CAMERA OK            ";
            2'd2: crow5 = "NO CHIP ID: CHECK P2 ";
            default: crow5 = "CAMERA CONFIGURING...";
        endcase
    end
    wire [7:0] fa = auto_on ? "A" : DASH, fc = crc_on ? "C" : DASH, fn = noise_on ? "N" : DASH, fw = auto_wafer ? "W" : DASH;
    wire [7:0] ftx = tx_act ? "*" : SP;
    wire [7:0] ca = gcal[0] ? "A" : DASH, cb = gcal[1] ? "B" : DASH, cc = gcal[2] ? "C" : DASH;
    wire [15:0] flt = fault ? "F!" : "  ";
    wire [7:0] kt = (lastkey >= 4'd10) ? "1" : "0";
    wire [7:0] ko = dc((lastkey >= 4'd10) ? lastkey - 4'd10 : lastkey);
    wire [7:0] l3 = lastkey[3] ? "1" : "0", l2 = lastkey[2] ? "1" : "0", l1 = lastkey[1] ? "1" : "0", l0 = lastkey[0] ? "1" : "0";
    wire [4:0] cpage = item - I_CAM;

    wire [167:0] row0 = {mtxt, SP, "LK:", ltxt, SP, fa, fc, fn, fw, SP, SP, ftx, SP};
    wire [167:0] row1 = test_mode ? {"LAST KEY ", kt, ko, " LED:", l3, l2, l1, l0, SP}
                                  : {"7SEG>", nu[71:32], SP, f_seg, SP, nu[31:0]};
    wire [167:0] d_row2 = {"V", f_v, " I", f_ma[31:0], " T", f_t, flt};
    wire [167:0] d_row3 = thr_dirty ? "LIMIT NOT SENT: APLY "
                                    : {"T", dc({1'b0, tool}), DASH, f_wf, " LOT", f_lot, " D", f_def[31:0], SP};
    reg [31:0] gmname;
    always @* case (gmode) 2'd1: gmname = "PTRL"; 2'd2: gmname = "GUAR"; 2'd3: gmname = "BOTH"; default: gmname = "OFF "; endcase
    // 云台模式 + 各工位区运动占比；正在报警的工位字母反色（bit 15/10/5 = 第 5/10/15 字）
    wire [167:0] v_row2 = {gmname, " A", f_z0[23:0], " B", f_z1[23:0], " C", f_z2[23:0], SP, SP};
    wire [20:0]  v_inv2 = {5'd0, zact[0], 4'd0, zact[1], 4'd0, zact[2], 5'd0};
    wire [167:0] v_row3 = {"GIM ", sname, " CAL:", ca, cb, cc, " E", dc(gerror)};
    wire [167:0] c_row2 = {"FPS", f_fps[23:0], " VS", f_vps[23:0], " LINE", f_lin[31:0]};
    wire [167:0] c_row3 = {"HLEN", f_hl[31:0], " PCLK", f_pck, "MHZ"};
    wire [167:0] c_row4 = {"DATA ", hx(cam_dmask[7:4]), hx(cam_dmask[3:0]), " ID ",
                           hx(cam_id[15:12]), hx(cam_id[11:8]), hx(cam_id[7:4]), hx(cam_id[3:0]), " TX", f_sps[15:0], SP};
    wire [167:0] c_row6 = {"7SEG PAGE ", dc({1'b0, cpage[2:0]} + 4'd1), "/8        "};
    wire [167:0] c_row7 = "ANY KEY: NEXT PAGE   ";
    wire [167:0] t_row2 = "KEYPAD CHECK MODE    ";
    wire [167:0] t_row3 = "PRESS KEY: NO ACTION ";

    // ---------------- 像素流水线：页/列 -> 字格 -> 字符 -> 字模 ----------------
    reg [2:0] p1; reg [4:0] e1; reg [2:0] s1; reg v1;
    wire [12:0] m43 = {6'd0, pix_col} * 13'd43;
    wire [4:0] cidx = m43[12:8];                                        // pix_col / 6（0~127 精确）
    wire [7:0] cidx6 = {3'd0, cidx} * 8'd6;
    reg [167:0] rt; reg [20:0] ri; reg [4:0] e2; reg [2:0] s2; reg v2;
    reg [7:0] ch3; reg inv3; reg [2:0] s3; reg v3;
    wire [7:0] fbits;
    oled_font5x7 u_font (.ch(ch3), .col(s3), .bits(fbits));
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            p1 <= 0; e1 <= 0; s1 <= 0; v1 <= 0; rt <= 0; ri <= 0; e2 <= 0; s2 <= 0; v2 <= 0;
            ch3 <= SP; inv3 <= 0; s3 <= 0; v3 <= 0; pix_data <= 0;
        end else begin
            p1 <= pix_page; e1 <= cidx; s1 <= pix_col - cidx6[6:0]; v1 <= cidx < 5'd21;
            e2 <= e1; s2 <= s1; v2 <= v1;
            case (p1)
                3'd0: begin rt <= row0; ri <= 21'h0; end
                3'd1: begin rt <= row1; ri <= 21'h1FFFFF; end
                3'd2: begin rt <= test_mode ? t_row2 : cam_mode ? c_row2 : vis_mode ? v_row2 : d_row2;
                            ri <= (vis_mode && !test_mode && !cam_mode) ? v_inv2 : 21'h0; end
                3'd3: begin rt <= test_mode ? t_row3 : cam_mode ? c_row3 : vis_mode ? v_row3 : d_row3;
                            ri <= (!test_mode && !cam_mode && !vis_mode && thr_dirty && blink) ? 21'h1FFFFF : 21'h0; end
                3'd4: begin rt <= cam_mode ? c_row4 : krow0; ri <= cam_mode ? 21'h0 : kinv0; end
                3'd5: begin rt <= cam_mode ? crow5  : krow1; ri <= cam_mode ? 21'h0 : kinv1; end
                3'd6: begin rt <= cam_mode ? c_row6 : krow2; ri <= cam_mode ? 21'h0 : kinv2; end
                default: begin rt <= cam_mode ? c_row7 : krow3; ri <= cam_mode ? 21'h0 : kinv3; end
            endcase
            ch3 <= v2 ? rt[(5'd20 - e2)*8 +: 8] : SP;
            inv3 <= v2 && ri[5'd20 - e2];
            s3 <= s2; v3 <= v2;
            pix_data <= v3 ? (fbits ^ {8{inv3}}) : 8'h00;
        end
endmodule
