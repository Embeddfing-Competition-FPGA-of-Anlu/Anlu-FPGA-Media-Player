`timescale 1ns/1ps
// 鍙戦€佹澘锛圖MZ_EG4S20 鏉匡紝EG4S20BG256锛?0MHz锛夛細宸ヨ壓鍙傛暟閲囬泦/妯℃嫙缁堢锛岀粡鏉块棿涓插彛缁欑湅鏉垮彂鏄剧ず鏁版嵁
//   鎺ョ嚎锛欽1(TX) -> 鐪嬫澘 P4锛孠2(RX) <- 鐪嬫澘 N4锛孏ND 鍏卞湴锛?15200 8N1锛?.3V TTL锛屽崗璁笌 USB 涓插彛鐩稿悓
//   !Mtool,seq,mV,mA,temp10[*CCCC]  !Llot,wafer,defects,yield10[*CCCC]  !Tvmin,vmax,imax,tmax10[*CCCC]
//   澶嶄綅鍚庣涓€鍖呭墠鍏堝彂 !D0锛氱湅鏉垮垏 LIVE 骞堕噸寤哄簭鍙峰熀鍑嗭紙鍙戦€佹澘閲嶅惎鍚庡簭鍙蜂粠 1 寮€濮嬩篃鑳借鎺ユ敹锛?
//   鐪嬫澘瀵规瘡琛屽洖 OK/ER锛孡ED/鏁扮爜绠℃樉绀洪€氫俊鎯呭喌
// 4x4 閿洏锛堟搷浣滆€呰瑙掞紝浠庡乏涓婅寮€濮嬮€愯缂栧彿 0~15锛涚‖浠舵壂鎻忓彿 = 15 - 閿彿锛屼笂鏉跨‘璁わ級锛?
//   0 V+10mV  1 V-10mV  2 I+1mA  3 I-1mA  4 T+0.5C  5 T-0.5C  6 璁惧 1~4  7 鏁呴殰娉ㄥ叆锛堟俯搴?30C锛?
//   8 涓嬩竴鐗囨櫠鍦? 9 缂洪櫡+1  10 鏂版壒娆? 11 绔嬪嵆鍙?M+L  12 閫夐槇鍊?VMIN/VMAX/IMAX/TMAX  13 闃堝€?  14 闃堝€?  15 鍙戦槇鍊?
// 鎷ㄧ爜锛歋W0 AUTO锛堟瘡绉?!M锛屾瘡 4 绉?!L锛? SW1 CRC  SW2 鏁版嵁鍣０  SW3 鑷姩璧扮墖锛堟瘡 8 绉掞級  SW4 鏁扮爜绠℃樉绀?OK 娆℃暟
//       SW5 鎽勫儚澶磋瘖鏂〉锛堟寜浠绘剰閿炕椤碉紝鎸夐敭涓嶆敼鏁版嵁锛夛細CAPt 甯х巼 / U5yn VSYNC 娆℃暟 / Lin 姣忓抚琛屾暟 / HLEn 姣忚瀛楄妭鏁?
//           / PCL PCLK MHz / dAtA D0~D7 璺冲彉浣嶏紙鍗佸叚杩涘埗锛? CHiP 璇诲埌鐨勮姱鐗?ID锛堝簲涓?5640锛? SEnd 姣忕鍙戠粰鐪嬫澘鐨勫抚鏁般€侰APt 椤?---- 閰嶇疆涓紝Err 鑺墖 ID 涓嶅
//       SW6 鐜板満瑙嗚妯″紡锛堟寜閿笉鏀瑰伐鑹烘暟鎹紝鏀逛负閫夌畻娉?/ 璋冮槇鍊?/ 鎺у埗浜戝彴锛夛細
//           0 鍘熷浘  1 杈圭紭  2 浜屽€煎寲  3 甯у樊杩愬姩  4 绾㈣壊鏍囪瘑   5 闃堝€?  6 闃堝€?
//           9 淇话+锛堜笂锛? 12 姘村钩-锛堝乏锛? 13 鍥炰腑  14 姘村钩+锛堝彸锛? 10 淇话-锛堜笅锛?
//           鏁扮爜绠★細Src/EdGE/bin/nove/rEd + 鍛戒腑鍗犳瘮 %锛堝師鍥炬樉绀哄彂閫佸抚鐜囷級锛宼hr 闃堝€硷紝PAn/tiLt 瑙掑害
//       SW7 鏍囧畾锛氭寜閿笉鍔ㄤ綔銆佷笉鍙戦€侊紱鏁扮爜绠″厛浜?tE5t 鍐嶆樉绀?1234锛堢湅浣嶅簭锛夛紱LED0~3锛圔14 B15 B16 C15锛?
//       浠ヤ簩杩涘埗鏄剧ず鍒氭寜涓嬬殑閿彿 0~15锛孡ED4锛圕16锛夋瘡鎸変竴娆＄炕杞竴娆?
// LED锛? 鍙戦€? 1 涓婁竴琛?OK  2 涓婁竴琛?ER  3 閾捐矾姝ｅ父锛? 绉掑唴鏈?OK锛? 4 鏁呴殰娉ㄥ叆  5 AUTO  6 K2 鏀跺埌浠讳綍瀛楄妭  7 蹇冭烦
module sender_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer SEC_MS = 1000,           // 涓€鈥滅鈥濆灏戞绉掞紙浠跨湡閲屾敼灏忥級
    parameter SW_ACTIVE_HIGH = 1,              // 鎷ㄧ爜 ON = 楂樼數骞筹紱瀹炴澘鐩稿弽鏀?0
    parameter LED_ACTIVE_HIGH = 1,             // 瀹炴澘 LED 楂樼數骞崇偣浜紙涓婃澘纭锛?
    parameter DIGIT0_LEFT = 0,                 // 瀹炴澘 seg_sel[0] 鏄渶鍙充竴浣嶏紙SW7 鏍囧畾鏄剧ず 4321 宸茬‘璁わ級
    parameter integer GIMBAL_PAN_MIN = 30, GIMBAL_PAN_MAX = 150,
    parameter integer GIMBAL_TILT_MIN = 45, GIMBAL_TILT_MAX = 135
)(
    input  wire clk,
    output reg [3:0] row, input wire [3:0] col,
    output reg [3:0] seg_sel, output reg [7:0] seg_led,
    input  wire [7:0] sw, output wire [7:0] led,
    output wire link_tx, input wire link_rx,
    // OV5640锛堟彃 P2 宸︿晶 40 閽堬級+ 瑙嗛涓插彛锛圥2-37 P12 -> 鐪嬫澘 M1锛?
    inout  wire cam_scl, cam_sda,
    output wire cam_rst_n, cam_pwdn,
    input  wire cam_pclk, cam_vsync, cam_href, input wire [7:0] cam_d,
    output wire vid_tx,
    // 浜戝彴鑸垫満 PWM锛?0Hz锛?.5~2.5ms锛夛細P2-35 M12 姘村钩  P2-36 N12 淇话
    output wire servo_pan, servo_tilt,
    output wire oled_scl, oled_sda
);
    reg [11:0] por = 12'd0;
    wire rst_n = &por;
    always @(posedge clk) if (!rst_n) por <= por + 1'b1;

    // ---------------- 鏃跺熀 / 鎷ㄧ爜 / 閿洏 ----------------
    localparam integer MS = CLK_HZ / 1000;
    reg [19:0] ms_cnt; reg [11:0] sec_cnt; reg [3:0] secs;
    wire ms_tick = ms_cnt == MS - 1;
    wire s_tick = ms_tick && sec_cnt == SEC_MS - 1;
    reg [7:0] sw_s0, sw_s1; reg [3:0] col_s;
    wire auto_on = sw_s1[0], crc_on = sw_s1[1], noise_on = sw_s1[2], auto_wafer = sw_s1[3], show_ok = sw_s1[4];
    wire cam_mode = sw_s1[5], vis_mode = sw_s1[6] && !sw_s1[5] && !sw_s1[7];
    reg [1:0] rsel; reg [4:0] db; reg [15:0] kraw, k1, kst, kp_raw;
    always @(posedge clk) begin sw_s0 <= SW_ACTIVE_HIGH ? sw : ~sw; sw_s1 <= sw_s0; col_s <= col; end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ms_cnt<=0; sec_cnt<=0; secs<=0; rsel<=0; row<=4'b1110; db<=0; kraw<=0; k1<=0; kst<=0; kp_raw<=0; end
        else begin
            ms_cnt <= ms_tick ? 20'd0 : ms_cnt + 1'b1;
            kp_raw <= 16'd0;
            if (ms_tick) begin
                sec_cnt <= s_tick ? 12'd0 : sec_cnt + 1'b1;
                if (s_tick) secs <= secs + 1'b1;
                kraw[rsel*4 +: 4] <= ~col_s;               // 琛岀嚎鎷変綆鎵弿锛屽垪绾挎湁涓婃媺锛屾寜涓嬭 0
                rsel <= rsel + 1'b1;
                row <= ~(4'b0001 << (rsel + 2'd1));
                db <= (db == 5'd19) ? 5'd0 : db + 1'b1;
                if (db == 5'd19) begin k1 <= kraw; kst <= k1 & kraw; kp_raw <= (k1 & kraw) & ~kst; end   // 20ms 涓ゆ涓€鑷存墠绠楁寜涓?
            end
        end

    // 鏍囧畾妯″紡锛氭寜閿彧璁板綍閿彿锛屼笉鏀规暟鍊笺€佷笉鍙戦€?
    wire test_mode = sw_s1[7];
    wire [15:0] kp_map;                            // 鎵弿鍙?n -> 閿彿 15-n
    genvar gk;
    generate for (gk = 0; gk < 16; gk = gk + 1) begin : g_kmap assign kp_map[gk] = kp_raw[15-gk]; end endgenerate
    wire [15:0] kpress = (test_mode || cam_mode || vis_mode) ? 16'd0 : kp_map;
    // 鐜板満瑙嗚锛氱畻娉?/ 闃堝€?/ 浜戝彴瑙掑害锛坴sel锛氭暟鐮佺鏄剧ず鍝竴椤?0 绠楁硶 1 闃堝€?2 姘村钩 3 淇话锛?
    wire [15:0] vk = vis_mode ? kp_map : 16'd0;
    reg [2:0] vmode; reg [7:0] vthr; reg [2:0] vsel;
    wire [7:0] pan, tilt;
    reg gimbal_armed;
    reg [2:0] ksel;                                // 卷积核：0 锐化 1 降噪 2 轮廓 3 浮雕 4 CNN（卷积模式下再按键 2 切换）
    reg [1:0] gmode;                               // 云台自动模式：bit0 巡检  bit1 闯入联动（键 4 循环 OFF/PTRL/GUAR/BOTH）
    reg [2:0] zact;                                // 去抖后的工位闯入 A/B/C（下面由摄像头分区统计生成）
    wire gimbal_moving, gimbal_intr;
    wire [3:0] gimbal_state, gimbal_error, gimbal_calibrated;
    wire [2:0] gimbal_task_tool;
    wire gimbal_command_valid, gimbal_alarm;
    wire [2:0] gimbal_tool, gimbal_reason;
    reg [2:0] tool;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin vmode <= 0; vthr <= 8'd5; vsel <= 0; gimbal_armed<=0; gmode <= 0; ksel <= 0; end
        else begin
            // 0 原图 1 边缘 2 增强 3 帧差分区；4 循环云台自动模式；5/6 闯入灵敏度（工位区运动占比 %，1~50）
            if (|vk[3:0]) begin vmode <= enc16({12'd0, vk[3:0]}); vsel <= 0; end
            if (vk[2] && vmode == 3'd2) ksel <= (ksel == 3'd4) ? 3'd0 : ksel + 1'b1;   // 已在卷积模式：换下一个核
            if (vk[4]) begin gmode <= gmode + 1'b1; vsel <= 4; end
            if (vk[5]) begin vsel <= 1; if (vthr > 8'd1) vthr <= vthr - 8'd1; end
            if (vk[6]) begin vsel <= 1; if (vthr < 8'd50) vthr <= vthr + 8'd1; end
            if (vk[12] || vk[14] || vk[13]) vsel<=2;
            if (vk[9] || vk[10]) vsel<=3;
            if (vk[7]) begin gimbal_armed<=~gimbal_armed; vsel<=4; end
            if (vk[8] || vk[11]) vsel<=4;
            if (vk[15]) vsel<=5;
        end
    gimbal_control #(.PAN_MIN(GIMBAL_PAN_MIN), .PAN_MAX(GIMBAL_PAN_MAX),
        .TILT_MIN(GIMBAL_TILT_MIN), .TILT_MAX(GIMBAL_TILT_MAX)) u_gimbal (
        .clk(clk), .rst_n(rst_n), .ms_tick(ms_tick), .enable(gimbal_armed && !cam_mode && !test_mode),
        .command_valid(gimbal_command_valid), .alarm(gimbal_alarm), .tool(gimbal_tool),
        .pan_minus(vk[12]), .pan_plus(vk[14]),
        .tilt_minus(vk[10]), .tilt_plus(vk[9]), .center(vk[13]), .confirm(vk[8]),
        .save_preset(vk[15]), .calibration_tool(tool), .patrol_en(gmode[0]), .intr_zone(gmode[1] ? zact : 3'd0),
        .pan(pan), .tilt(tilt), .state(gimbal_state), .error(gimbal_error), .task_tool(gimbal_task_tool),
        .calibrated(gimbal_calibrated), .moving(gimbal_moving), .intr_task(gimbal_intr));
    servo_pwm #(.CLK_HZ(CLK_HZ)) u_pan  (.clk(clk), .rst_n(rst_n), .angle(pan),  .pwm(servo_pan));
    servo_pwm #(.CLK_HZ(CLK_HZ)) u_tilt (.clk(clk), .rst_n(rst_n), .angle(tilt), .pwm(servo_tilt));
    reg [2:0] cpg;                                 // 鎽勫儚澶磋瘖鏂〉 0~7
    always @(posedge clk or negedge rst_n)
        if (!rst_n) cpg <= 0; else if (cam_mode && !test_mode && |kp_map) cpg <= cpg + 1'b1;
    reg [3:0] lastkey; reg ktog;
    function [3:0] enc16; input [15:0] v; integer j; begin enc16 = 0; for (j = 0; j < 16; j = j + 1) if (v[j]) enc16 = j; end endfunction
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin lastkey <= 0; ktog <= 0; end else if (|kp_map) begin lastkey <= enc16(kp_map); ktog <= ~ktog; end

    // ---------------- 瑕佸彂閫佺殑鏁板€?----------------
    reg [15:0] mv, ma, t, lot, wafer, defects, vmin, vmax, imax, tmax, seq, lfsr;
    reg [2:0] dsel; reg [1:0] lsel; reg fault, req_m, req_l, req_t, synced;
    reg tk_m, tk_l, tk_t, tk_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            mv<=3300; ma<=42; t<=350; tool<=1; fault<=0; lot<=1234; wafer<=1; defects<=0;
            vmin<=3200; vmax<=3400; imax<=60; tmax<=600; lsel<=0; dsel<=0; req_m<=0; req_l<=0; req_t<=0; lfsr<=16'hace1;
        end else begin
            if (ms_tick) lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
            if (tk_m) req_m <= 0;  if (tk_l) req_l <= 0;  if (tk_t) req_t <= 0;
            if (tk_d) req_l <= 1;                         // 鍚屾鍚庤ˉ鍙戜竴娆℃壒娆?
            if (s_tick && auto_on) begin req_m <= 1; if (secs[1:0] == 2'd3) req_l <= 1; end
            if ((s_tick && auto_wafer && secs[2:0] == 3'd7) || kpress[8]) begin
                req_l <= 1;
                if (wafer >= 25) begin wafer <= 1; lot <= lot + 1'b1; defects <= 0; end
                else begin wafer <= wafer + 1'b1; if (!kpress[8]) defects <= defects + {14'd0, lfsr[1:0]==2'd3, lfsr[1:0]==2'd2}; end
            end
            if (kpress[0] && mv <= 19990) mv <= mv + 16'd10;
            if (kpress[1] && mv >= 10)    mv <= mv - 16'd10;
            if (kpress[2] && ma < 5000)   ma <= ma + 1'b1;
            if (kpress[3] && ma != 0)     ma <= ma - 1'b1;
            if (kpress[4] && t <= 1495)   t <= t + 16'd5;
            if (kpress[5] && t >= 5)      t <= t - 16'd5;
            if (kpress[6]) tool <= (tool >= 3'd3) ? 3'd1 : tool + 1'b1;   // 设备 1~3 = 工位 A/B/C
            if (kpress[7]) fault <= ~fault;
            if (|kpress[7:0]) req_m <= 1;
            if (kpress[9] && defects < 9999) defects <= defects + 1'b1;
            if (kpress[10]) begin lot <= lot + 1'b1; wafer <= 1; defects <= 0; end
            if (|kpress[10:9]) req_l <= 1;
            if (kpress[11]) begin req_m <= 1; req_l <= 1; end
            if (kpress[12] && dsel == 3'd6) lsel <= lsel + 1'b1;   // first press of 12 only switches to limit view
            if (kpress[13]) case (lsel)
                0: if (vmin + 16'd50 < vmax) vmin <= vmin + 16'd50;
                1: if (vmax <= 19950) vmax <= vmax + 16'd50;
                2: if (imax <= 4990) imax <= imax + 16'd10;
                default: if (tmax <= 1450) tmax <= tmax + 16'd50;
            endcase
            if (kpress[14]) case (lsel)
                0: if (vmin >= 50) vmin <= vmin - 16'd50;
                1: if (vmax >= vmin + 16'd100) vmax <= vmax - 16'd50;
                2: if (imax >= 20) imax <= imax - 16'd10;
                default: if (tmax >= 100) tmax <= tmax - 16'd50;
            endcase
            if (kpress[15]) req_t <= 1;
            if (|kpress[1:0]) dsel <= 0;  if (|kpress[3:2]) dsel <= 1;  if (|kpress[5:4] || kpress[7]) dsel <= 2;
            if (kpress[6] || kpress[8]) dsel <= 3;  if (kpress[9]) dsel <= 4;  if (kpress[10]) dsel <= 5;
            if (|kpress[15:12]) dsel <= 6;
        end
    // 鍣０ / 鏁呴殰娉ㄥ叆鍙姞鍦ㄥ彂鍑哄幓鐨勫€间笂
    wire [15:0] mv_n = (noise_on && mv >= 8) ? mv + {12'd0, lfsr[7:4]} - 16'd8 : mv;
    wire [15:0] mv_tx = mv_n > 20000 ? 16'd20000 : mv_n;
    wire [15:0] ma_tx = (noise_on && ma >= 1) ? ma + {14'd0, lfsr[9:8]} - 16'd1 : ma;
    wire [15:0] t_n = (noise_on && t >= 4) ? t + {13'd0, lfsr[12:10]} - 16'd4 : t;
    wire [15:0] t_f = fault ? t_n + 16'd300 : t_n;
    wire [15:0] t_tx = t_f > 1500 ? 16'd1500 : t_f;
    wire [17:0] d4 = {defects, 2'b00};
    wire [15:0] yld = d4 >= 18'd1000 ? 16'd0 : 16'd1000 - d4[15:0];

    // ---------------- 琛岀粍瑁?+ 鍙戦€侊細瀛楁浜岃繘鍒?-> 鍗佽繘鍒讹紙娑堝墠瀵奸浂锛夛紝鍙€?*CRC锛孋R 缁撴潫 ----------------
    localparam integer CPB = CLK_HZ / BAUD;
    localparam [3:0] S_IDLE=0, S_OP=1, S_LOAD=2, S_CONV=3, S_DIG=4, S_SEP=5, S_HEX=6, S_CR=7;
    reg [3:0] st; reg [7:0] opc; reg [79:0] fv; reg [2:0] nfld, fi, di; reg [15:0] bin; reg [19:0] bcd;
    reg [4:0] cc; reg started, crc_en; reg [15:0] crc; reg [1:0] hi;
    reg tx_start, tx_wait; reg [7:0] tx_data; wire tx_busy;
    uart_tx_byte #(.CLKS_PER_BIT(CPB)) u_tx (.clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .tx(link_tx), .busy(tx_busy), .done());
    wire can = !tx_start && !tx_wait && !tx_busy;
    function [15:0] crc_b;
        input [15:0] c; input [7:0] b; integer j; reg [15:0] r;
        begin r = c ^ {b, 8'd0}; for (j = 0; j < 8; j = j + 1) r = r[15] ? ({r[14:0], 1'b0} ^ 16'h1021) : {r[14:0], 1'b0}; crc_b = r; end
    endfunction
    function [7:0] hexc; input [3:0] v; hexc = v < 10 ? 8'd48 + {4'd0, v} : 8'd55 + {4'd0, v}; endfunction
    reg [19:0] adj; integer k;
    always @* begin adj = bcd; for (k = 0; k < 5; k = k + 1) if (bcd[k*4 +: 4] >= 5) adj[k*4 +: 4] = bcd[k*4 +: 4] + 4'd3; end
    reg [3:0] dg;
    always @* case (di) 0: dg = bcd[19:16]; 1: dg = bcd[15:12]; 2: dg = bcd[11:8]; 3: dg = bcd[7:4]; default: dg = bcd[3:0]; endcase
    reg [3:0] hn;
    always @* case (hi) 0: hn = crc[15:12]; 1: hn = crc[11:8]; 2: hn = crc[7:4]; default: hn = crc[3:0]; endcase
    task emit(input [7:0] b, input upd);
        begin tx_start <= 1; tx_wait <= 1; tx_data <= b; if (upd) crc <= crc_b(crc, b); end
    endtask
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            st<=S_IDLE; tx_start<=0; tx_wait<=0; tx_data<=8'hff; opc<=0; fv<=0; nfld<=0; fi<=0; di<=0; bin<=0; bcd<=0;
            cc<=0; started<=0; crc<=16'hffff; crc_en<=0; hi<=0; seq<=0; synced<=0; tk_m<=0; tk_l<=0; tk_t<=0; tk_d<=0;
        end else begin
            tx_start <= 0; tk_m <= 0; tk_l <= 0; tk_t <= 0; tk_d <= 0;
            if (tx_busy) tx_wait <= 0;
            case (st)
            S_IDLE: if (can && (req_m || req_l || req_t)) begin
                fi <= 0; crc_en <= crc_on;
                if (!synced) begin opc <= "D"; nfld <= 1; fv <= 80'd0; crc_en <= 0; synced <= 1; tk_d <= 1; end
                else if (req_t) begin opc <= "T"; nfld <= 4; fv <= {vmin, vmax, imax, tmax, 16'd0}; tk_t <= 1; end
                else if (req_m) begin opc <= "M"; nfld <= 5; fv <= {13'd0, tool, seq + 16'd1, mv_tx, ma_tx, t_tx}; seq <= seq + 1'b1; tk_m <= 1; end
                else begin opc <= "L"; nfld <= 4; fv <= {lot, wafer, defects, yld, 16'd0}; tk_l <= 1; end
                tx_start <= 1; tx_wait <= 1; tx_data <= "!"; crc <= crc_b(16'hffff, "!"); st <= S_OP;
            end
            S_OP:   if (can) begin emit(opc, 1); st <= S_LOAD; end
            S_LOAD: begin bin <= fv[79:64]; bcd <= 0; cc <= 16; di <= 0; started <= 0; st <= S_CONV; end
            S_CONV: if (cc != 0) begin bcd <= {adj[18:0], bin[15]}; bin <= {bin[14:0], 1'b0}; cc <= cc - 1'b1; end
                    else st <= S_DIG;
            S_DIG:  if (can) begin
                if (started || dg != 0 || di == 3'd4) begin emit(8'd48 + {4'd0, dg}, 1); started <= 1; end
                if (di == 3'd4) st <= S_SEP; else di <= di + 1'b1;
            end
            S_SEP:  if (can) begin
                if (fi + 3'd1 < nfld) begin emit(",", 1); fi <= fi + 1'b1; fv <= {fv[63:0], 16'd0}; st <= S_LOAD; end
                else if (crc_en) begin emit("*", 0); hi <= 0; st <= S_HEX; end
                else begin emit(8'h0d, 0); st <= S_IDLE; end
            end
            S_HEX:  if (can) begin emit(hexc(hn), 0); if (hi == 2'd3) st <= S_CR; else hi <= hi + 1'b1; end
            S_CR:   if (can) begin emit(8'h0d, 0); st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    // ---------------- 鐪嬫澘搴旂瓟 OK / ER ----------------
    wire [7:0] rx_d; wire rx_v, rx_framing_error;
    uart_rx_byte #(.CLKS_PER_BIT(CPB)) u_rx (.clk(clk), .rst_n(rst_n), .rx(link_rx), .data(rx_d), .valid(rx_v), .framing_error(rx_framing_error));
    gimbal_command_rx #(.CLK_HZ(CLK_HZ)) u_gimbal_rx (
        .clk(clk), .rst_n(rst_n), .ms_tick(ms_tick), .data(rx_d), .valid(rx_v), .framing_error(rx_framing_error),
        .command_valid(gimbal_command_valid), .tool(gimbal_tool), .alarm(gimbal_alarm), .reason(gimbal_reason));
    reg [15:0] rl, ok_cnt, er_cnt; reg last_ok, last_er; reg [13:0] alive; reg [5:0] tx_led, rx_led;
    wire link_ok = ok_cnt != 0 && alive < 3 * SEC_MS;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin rl<=0; ok_cnt<=0; er_cnt<=0; last_ok<=0; last_er<=0; alive<=0; tx_led<=0; rx_led<=0; end
        else begin
            if (tx_start) tx_led <= 6'd50; else if (ms_tick && tx_led != 0) tx_led <= tx_led - 1'b1;
            if (rx_v) rx_led <= 6'd50; else if (ms_tick && rx_led != 0) rx_led <= rx_led - 1'b1;
            if (ms_tick && alive != 14'h3fff) alive <= alive + 1'b1;
            if (rx_v) begin
                if (rx_d == 8'h0d || rx_d == 8'h0a) begin
                    if (rl == "OK") begin ok_cnt <= ok_cnt + 1'b1; last_ok <= 1; last_er <= 0; alive <= 0; end
                    else if (rl == "ER") begin er_cnt <= er_cnt + 1'b1; last_ok <= 0; last_er <= 1; end
                    rl <= 0;
                end else rl <= {rl[7:0], rx_d};
            end
        end
    // ---------------- 鎽勫儚澶达細涓婄數閰嶇疆 + 160x120 鐏板害閲囬泦 + 瑙嗛涓插彛 ----------------
    wire scl_oe, sda_oe; wire [1:0] cam_st; wire [15:0] cam_id; wire [7:0] cam_fps, cam_vps, cam_dmask, cam_sps; wire [6:0] cam_hitp; wire [11:0] cam_vlines, cam_hlen, cam_pmhz;
    assign cam_scl = scl_oe ? 1'b0 : 1'bz;
    assign cam_sda = sda_oe ? 1'b0 : 1'bz;
    ov5640_init #(.CLK_HZ(CLK_HZ)) u_cam_init (.clk(clk), .rst_n(rst_n), .cam_rst_n(cam_rst_n), .cam_pwdn(cam_pwdn),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_in(cam_sda), .status(cam_st), .chip_id(cam_id));
    wire [6:0] zp0, zp1, zp2; wire ztick;
    cam_video #(.CLK_HZ(CLK_HZ)) u_cam_vid (.clk(clk), .rst_n(rst_n), .run(cam_st == 2'd1), .s_tick(s_tick), .vmode(vis_mode ? vmode : 3'd0), .ksel(ksel), .zact(zact),
        .pclk(cam_pclk), .vsync(cam_vsync), .href(cam_href), .d(cam_d), .vid_tx(vid_tx),
        .fps(cam_fps), .vps(cam_vps), .vlines(cam_vlines), .hlen(cam_hlen), .pmhz(cam_pmhz), .dmask(cam_dmask), .sps(cam_sps), .hitp(cam_hitp),
        .zp0(zp0), .zp1(zp1), .zp2(zp2), .ztick(ztick));
    // ---------------- 工位闯入去抖：某区运动占比 >= vthr% 连续 2 帧算闯入，< vthr% 连续 8 帧解除 ----------------
    //   云台在转、或停下不到 1.5 秒时冻结（箭头/巡检头自己的运动不算闯入）；1 秒没有新帧（摄像头断开）清零
    localparam integer ZON_N = 2, ZOFF_N = 8, ZHOLD_MS = 1500;
    reg [10:0] zhold; reg [10:0] zage; reg [1:0] zon0, zon1, zon2; reg [3:0] zof0, zof1, zof2;
    wire [2:0] zhi = {zp2 >= vthr[6:0], zp1 >= vthr[6:0], zp0 >= vthr[6:0]};
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin zhold <= 0; zage <= 0; zact <= 0; zon0 <= 0; zon1 <= 0; zon2 <= 0; zof0 <= 0; zof1 <= 0; zof2 <= 0; end
        else begin
            if (gimbal_moving) zhold <= ZHOLD_MS; else if (ms_tick && zhold != 0) zhold <= zhold - 1'b1;
            if (ztick) zage <= 0; else if (ms_tick && zage != 11'h7ff) zage <= zage + 1'b1;
            if (zage >= 11'd1000) begin zact <= 0; zon0 <= 0; zon1 <= 0; zon2 <= 0; zof0 <= 0; zof1 <= 0; zof2 <= 0; end
            else if (ztick && zhold == 0 && !gimbal_moving) begin
                if (zhi[0]) begin zof0 <= 0; if (zon0 >= ZON_N - 1) zact[0] <= 1; else zon0 <= zon0 + 1'b1; end
                else begin zon0 <= 0; if (zof0 >= ZOFF_N - 1) zact[0] <= 0; else zof0 <= zof0 + 1'b1; end
                if (zhi[1]) begin zof1 <= 0; if (zon1 >= ZON_N - 1) zact[1] <= 1; else zon1 <= zon1 + 1'b1; end
                else begin zon1 <= 0; if (zof1 >= ZOFF_N - 1) zact[1] <= 0; else zof1 <= zof1 + 1'b1; end
                if (zhi[2]) begin zof2 <= 0; if (zon2 >= ZON_N - 1) zact[2] <= 1; else zon2 <= zon2 + 1'b1; end
                else begin zon2 <= 0; if (zof2 >= ZOFF_N - 1) zact[2] <= 0; else zof2 <= zof2 + 1'b1; end
            end
        end

    wire [7:0] led_raw = test_mode ? {sec_cnt < SEC_MS / 2, 2'b00, ktog, lastkey}
                                   : vis_mode ? {gimbal_armed,gimbal_state,(|gimbal_error),gimbal_calibrated[tool-1],gimbal_state==4'd6}
                                   : {sec_cnt < SEC_MS / 2, rx_led != 0, auto_on, fault, link_ok, last_er, last_ok, tx_led != 0};
    assign led = LED_ACTIVE_HIGH ? led_raw : ~led_raw;

    // ---------------- I2C OLED operator screen (see oled_screen.v) ----------------
    reg [4:0] item, item_q;
    reg [5:0] vm_q;                                // {卷积核, 算法}：变了就重亮名称
    reg [13:0] dval;
    reg [15:0] dshow;                              // BCD value shown on the 7-seg display (mirrored on OLED row 1)
    reg thr_dirty;                                 // thresholds edited with 13/14 but not yet sent with 15
    reg [8:0] kflash_ms;                           // 0.3 s key-press flash on the OLED keymap
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin thr_dirty <= 1'b0; kflash_ms <= 9'd0; end
        else begin
            if (kpress[13] || kpress[14]) thr_dirty <= 1'b1; else if (tk_t) thr_dirty <= 1'b0;
            if (|kp_map) kflash_ms <= 9'd300; else if (ms_tick && kflash_ms != 0) kflash_ms <= kflash_ms - 1'b1;
        end
    wire [1:0] link_st = last_er ? 2'd2 : link_ok ? 2'd1 : (ok_cnt != 0) ? 2'd3 : 2'd0;

    // ---------------- 鏁扮爜绠★細鍒囨崲鏄剧ず閲忔椂鍏堜寒 1 绉掑悕绉帮紝鍐嶆樉绀哄甫鍗曚綅灏忔暟鐐圭殑鏁板€硷紱10 绉掓棤鎸夐敭閲嶄寒涓€娆″悕绉?----------------
    //   UoLt 鐢靛帇 V(3.300)  Curr 鐢垫祦 mA(42)  tEnP 娓╁害 C(35.0)  tooL 璁惧-鐗囧彿(1-05)  dEF 缂洪櫡鏁? Lot 鎵规鍙?
    //   U-Lo/U-Hi 鐢靛帇涓?涓婇檺 V  C-Hi 鐢垫祦涓婇檺 mA  t-Hi 娓╁害涓婇檺 C  Good 鏀跺埌 OK 娆℃暟(SW4)  tE5t 鏍囧畾(SW7)
    //   SW5 鎽勫儚澶磋瘖鏂細CAPt 甯х巼  U5yn VSYNC 涓婂崌娌?绉? Lin 姣忓抚琛屾暟  HLEn 姣忚瀛楄妭鏁? PCL PCLK MHz  dAtA 璺冲彉鏁版嵁浣?
    //   鏈€鍙充綅灏忔暟鐐逛寒 = 姝ｅ湪鍙戦€侊紱鍚嶇О鏈熼棿鎸変换鎰忛敭绔嬪嵆鍥炲埌鏁板€?
    localparam [4:0] I_V=0, I_I=1, I_T=2, I_TW=3, I_DEF=4, I_LOT=5, I_VLO=6, I_VHI=7, I_IHI=8, I_THI=9, I_OK=10, I_TST=11,
                     I_CAM=12, I_VPS=13, I_LIN=14, I_HL=15, I_PCK=16, I_DAT=17, I_ID=18, I_SND=19,
                     I_VIS=20, I_THR=21, I_PAN=22, I_TLT=23, I_GIM=24, I_CAL=25;
    always @* begin
        if (test_mode) item = I_TST;
        else if (cam_mode) item = I_CAM + {2'd0, cpg};
        else if (vis_mode) item = I_VIS + {2'd0, vsel};
        else if (show_ok) item = I_OK;
        else case (dsel)
            0: item = I_V; 1: item = I_I; 2: item = I_T; 3: item = I_TW; 4: item = I_DEF; 5: item = I_LOT;
            default: item = I_VLO + {3'd0, lsel};
        endcase
    end
    function [13:0] c14; input [15:0] v; c14 = v > 16'd9999 ? 14'd9999 : v[13:0]; endfunction
    // 鐢靛帇鎸変紡鏄剧ず锛?10V 涓?x.xxx锛?=10V 涓?xx.xx锛坢V/10 = mV*52429>>19锛?~20000 绮剧‘锛?
    reg [15:0] vsrc;
    always @* case (item) I_VLO: vsrc = vmin; I_VHI: vsrc = vmax; default: vsrc = mv; endcase
    wire [31:0] vprod = vsrc * 32'd52429;
    reg [1:0] dpv;                // dpv锛氬皬鏁扮偣鍦ㄧ dpv 浣嶏紙浠庡乏鏁?1~3锛変箣鍚庯紝0 = 鏃?
    always @* begin
        dpv = 0;
        case (item)
            I_V, I_VLO, I_VHI: if (vsrc >= 16'd10000) begin dval = {1'b0, vprod[31:19]}; dpv = 2; end
                               else begin dval = vsrc[13:0]; dpv = 1; end
            I_I:   dval = c14(ma);
            I_T:   begin dval = c14(t_tx); dpv = 3; end
            I_TW:  dval = c14({tool, 10'd0} - {tool, 4'd0} - {tool, 3'd0} + wafer);   // 璁惧鍙?x1000 + 鐗囧彿锛岀櫨浣嶆樉绀烘垚 -
            I_DEF: dval = c14(defects);
            I_LOT: dval = c14(lot);
            I_IHI: dval = c14(imax);
            I_THI: begin dval = c14(tmax); dpv = 3; end
            I_OK:  dval = c14(ok_cnt);
            I_CAM: dval = {6'd0, cam_fps};
            I_VPS: dval = {6'd0, cam_vps};
            I_LIN: dval = c14({4'd0, cam_vlines});
            I_HL:  dval = c14({4'd0, cam_hlen});
            I_PCK: begin dval = c14({4'd0, cam_pmhz}); dpv = 3; end
            I_SND: dval = {6'd0, cam_sps};
            // 原图、锐化、降噪、浮雕没有“命中”含义，显示发送帧率；其余显示命中占比
            I_VIS: dval = (vmode == 3'd0 || (vmode == 3'd2 && (ksel <= 3'd1 || ksel == 3'd3))) ? {6'd0, cam_sps} : {7'd0, cam_hitp};
            I_THR: dval = {6'd0, vthr};
            I_PAN: dval = {6'd0, pan};
            I_TLT: dval = {6'd0, tilt};
            I_GIM: dval = gimbal_state*14'd1000 + gimbal_error*14'd100 + {11'd0,gimbal_task_tool};
            I_CAL: dval = {10'd0,gimbal_calibrated};
            I_DAT, I_ID: dval = 14'd0;             // 鍗佸叚杩涘埗锛岃涓嬮潰 vg
            default: dval = 14'd1234;
        endcase
    end
    reg [11:0] lbl; reg [13:0] idle;
    wire show_lbl = lbl != 0;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin item_q <= I_V; vm_q <= 0; lbl <= SEC_MS; idle <= 0; end
        else begin
            item_q <= item; vm_q <= {ksel, vmode};
            if (item != item_q || (item == I_VIS && {ksel, vmode} != vm_q)) begin lbl <= SEC_MS; idle <= 0; end
            else if (|kp_map) begin lbl <= 0; idle <= 0; end
            else if (ms_tick) begin
                if (lbl != 0) lbl <= lbl - 1'b1;
                if (idle == 10 * SEC_MS - 1) begin idle <= 0; lbl <= SEC_MS; end else idle <= idle + 1'b1;
            end
        end
    // 瀛楀舰鐮侊細0~9 鏁板瓧锛屽叾浣欎负瀛楁瘝
    localparam [4:0] G_BL=10, G_DASH=11, G_U=12, G_o=13, G_L=14, G_t=15, G_C=16, G_u=17, G_r=18, G_n=19,
                     G_P=20, G_d=21, G_E=22, G_F=23, G_H=24, G_i=25, G_G=26, G_A=27, G_b=28, G_y=29, G_S=5;
    function [19:0] lbl_txt; input [4:0] it;
        case (it)
            I_V:   lbl_txt = {G_U, G_o, G_L, G_t};
            I_I:   lbl_txt = {G_C, G_u, G_r, G_r};
            I_T:   lbl_txt = {G_t, G_E, G_n, G_P};
            I_TW:  lbl_txt = {G_t, G_o, G_o, G_L};
            I_DEF: lbl_txt = {G_BL, G_d, G_E, G_F};
            I_LOT: lbl_txt = {G_BL, G_L, G_o, G_t};
            I_VLO: lbl_txt = {G_U, G_DASH, G_L, G_o};
            I_VHI: lbl_txt = {G_U, G_DASH, G_H, G_i};
            I_IHI: lbl_txt = {G_C, G_DASH, G_H, G_i};
            I_THI: lbl_txt = {G_t, G_DASH, G_H, G_i};
            I_OK:  lbl_txt = {G_G, G_o, G_o, G_d};
            I_CAM: lbl_txt = {G_C, G_A, G_P, G_t};
            I_VPS: lbl_txt = {G_U, G_S, G_y, G_n};
            I_LIN: lbl_txt = {G_BL, G_L, G_i, G_n};
            I_HL:  lbl_txt = {G_H, G_L, G_E, G_n};
            I_PCK: lbl_txt = {G_P, G_C, G_L, G_BL};
            I_DAT: lbl_txt = {G_d, G_A, G_t, G_A};
            I_ID:  lbl_txt = {G_C, G_H, G_i, G_P};
            I_SND: lbl_txt = {G_S, G_E, G_n, G_d};
            I_THR: lbl_txt = {G_S, G_E, G_n, G_S};
            I_PAN: lbl_txt = {G_BL, G_P, G_A, G_n};
            I_TLT: lbl_txt = {G_t, G_i, G_L, G_t};
            I_GIM: lbl_txt = {G_A, G_u, G_t, G_o};
            I_CAL: lbl_txt = {G_C, G_A, G_L, G_BL};
            default: lbl_txt = {G_t, G_E, G_S, G_t};
        endcase
    endfunction
    function [6:0] seg7; input [4:0] g;
        case (g)
            0: seg7 = 7'h3f; 1: seg7 = 7'h06; 2: seg7 = 7'h5b; 3: seg7 = 7'h4f; 4: seg7 = 7'h66;
            5: seg7 = 7'h6d; 6: seg7 = 7'h7d; 7: seg7 = 7'h07; 8: seg7 = 7'h7f; 9: seg7 = 7'h6f;
            G_DASH: seg7 = 7'h40; G_U: seg7 = 7'h3e; G_o: seg7 = 7'h5c; G_L: seg7 = 7'h38; G_t: seg7 = 7'h78;
            G_C: seg7 = 7'h39; G_u: seg7 = 7'h1c; G_r: seg7 = 7'h50; G_n: seg7 = 7'h54; G_P: seg7 = 7'h73;
            G_d: seg7 = 7'h5e; G_E: seg7 = 7'h79; G_F: seg7 = 7'h71; G_H: seg7 = 7'h76; G_i: seg7 = 7'h10;
            G_G: seg7 = 7'h3d; G_A: seg7 = 7'h77; G_b: seg7 = 7'h7c; G_y: seg7 = 7'h6e; default: seg7 = 7'h00;
        endcase
    endfunction
    // 涓茶 BCD锛歞val 杩?cb 鏃跺悓鏃惰涓嬪皬鏁扮偣浣嶇疆/璁惧-鐗囧彿鏍煎紡锛岃浆鎹㈠畬涓€璧烽€佸埌 dshow
    reg [13:0] cb; reg [15:0] cbcd; reg [3:0] ccnt; reg [15:0] cadj; integer q;
    reg [1:0] cdp, sdp; reg ctw, stw;
    always @* begin cadj = cbcd; for (q = 0; q < 4; q = q + 1) if (cbcd[q*4 +: 4] >= 5) cadj[q*4 +: 4] = cbcd[q*4 +: 4] + 4'd3; end
    reg [1:0] dig;
    reg [3:0] nib;
    always @* case (dig) 0: nib = dshow[15:12]; 1: nib = dshow[11:8]; 2: nib = dshow[7:4]; default: nib = dshow[3:0]; endcase
    reg [19:0] vtxt;
    always @* case (vmode)
        3'd1: vtxt = {G_E, G_d, G_G, G_E}; 3'd2: case (ksel) 3'd0: vtxt = {G_S, G_H, G_r, G_P}; 3'd1: vtxt = {G_b, G_L, G_u, G_r}; 3'd2: vtxt = {G_BL, G_L, G_A, G_P};
                            3'd3: vtxt = {G_E, G_n, G_b, G_o}; default: vtxt = {G_C, G_n, G_n, G_BL}; endcase 3'd3: vtxt = {G_n, G_o, G_U, G_E};
        default: vtxt = {G_BL, G_S, G_r, G_C};
    endcase
    wire [19:0] ltxt = (item == I_VIS) ? vtxt : lbl_txt(item);
    reg [4:0] lg;
    always @* case (dig) 0: lg = ltxt[19:15]; 1: lg = ltxt[14:10]; 2: lg = ltxt[9:5]; default: lg = ltxt[4:0]; endcase
    wire z0 = dshow[15:12] == 0, z1 = z0 && dshow[11:8] == 0, z2 = z1 && dshow[7:4] == 0;
    wire [1:0] blim = sdp == 0 ? 2'd3 : sdp - 1'b1;          // 灏忔暟鐐瑰墠涓€浣嶈捣涓嶅啀娑堥殣
    wire blank = !stw && dig < blim && ((dig == 0 && z0) || (dig == 1 && z1) || (dig == 2 && z2));
    // CAPt 椤靛湪鎽勫儚澶存湭灏辩华鏃朵笉鏄剧ず鏁板€硷細閰嶇疆涓?----锛岃姱鐗?ID 涓嶅 Err锛? 绉掑悗鑷姩閲嶈瘯锛夛紱鍏朵綑璇婃柇椤靛缁堟樉绀哄疄娴嬪€?
    wire cam_bad = item == I_CAM && cam_st != 2'd1;
    reg [4:0] cg;
    always @* case (dig) 0: cg = cam_st == 2'd2 ? G_E : G_DASH; 1: cg = cam_st == 2'd2 ? G_r : G_DASH;
                         2: cg = cam_st == 2'd2 ? G_r : G_DASH; default: cg = cam_st == 2'd2 ? G_BL : G_DASH; endcase
    function [4:0] hexg; input [3:0] v;
        case (v) 10: hexg = G_A; 11: hexg = G_b; 12: hexg = G_C; 13: hexg = G_d; 14: hexg = G_E; 15: hexg = G_F;
                 default: hexg = {1'b0, v}; endcase
    endfunction
    wire [15:0] hv = item == I_ID ? cam_id : {8'd0, cam_dmask};
    reg [4:0] hg;
    always @* case (dig) 0: hg = item == I_ID ? hexg(hv[15:12]) : G_BL; 1: hg = item == I_ID ? hexg(hv[11:8]) : G_BL;
                         2: hg = hexg(hv[7:4]); default: hg = hexg(hv[3:0]); endcase
    wire [4:0] vg = cam_bad ? cg : (item == I_DAT || item == I_ID) ? hg : (stw && dig == 1) ? G_DASH : blank ? G_BL : {1'b0, nib};
    wire dot = !show_lbl && !cam_bad && ((sdp != 0 && dig == sdp - 1'b1) || (dig == 3 && tx_led != 0));
    wire [1:0] phys = DIGIT0_LEFT ? dig : 2'd3 - dig;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cb<=0; cbcd<=0; dshow<=0; ccnt<=0; cdp<=0; sdp<=0; ctw<=0; stw<=0; dig<=0; seg_sel<=4'b1111; seg_led<=0; end
        else begin
            if (ccnt == 0) begin
                dshow <= cbcd; sdp <= cdp; stw <= ctw;
                cb <= dval; cdp <= dpv; ctw <= item == I_TW; cbcd <= 0; ccnt <= 4'd14;
            end else begin cbcd <= {cadj[14:0], cb[13]}; cb <= {cb[12:0], 1'b0}; ccnt <= ccnt - 1'b1; end
            if (ms_tick) dig <= dig + 1'b1;
            seg_sel <= ~(4'b0001 << phys);
            seg_led <= {dot, seg7(show_lbl ? lg : vg)};
        end
    wire [2:0] oled_page; wire [6:0] oled_col; wire [7:0] oled_pix;
    wire oled_scl_oe, oled_sda_oe;
    oled_screen u_oled_screen (
        .clk(clk), .rst_n(rst_n), .pix_page(oled_page), .pix_col(oled_col), .pix_data(oled_pix),
        .blink(sec_cnt < SEC_MS / 2), .kflash(kflash_ms != 0), .lastkey(lastkey),
        .test_mode(test_mode), .cam_mode(cam_mode), .vis_mode(vis_mode),
        .auto_on(auto_on), .crc_on(crc_on), .noise_on(noise_on), .auto_wafer(auto_wafer), .fault(fault),
        .tx_act(tx_led != 0), .thr_dirty(thr_dirty), .link_st(link_st),
        .item(item), .dshow(dshow), .sdp(sdp), .stw(stw), .cam_bad(cam_bad),
        .cam_st(cam_st), .cam_id(cam_id), .cam_dmask(cam_dmask),
        .mv(mv), .ma(ma), .t_tx(t_tx), .wafer(wafer), .lot(lot), .defects(defects),
        .tool(tool), .lsel(lsel), .vmode(vmode), .vthr(vthr), .pan(pan), .tilt(tilt),
        .armed(gimbal_armed), .gstate(gimbal_state), .gerror(gimbal_error), .gcal(gimbal_calibrated),
        .cam_fps(cam_fps), .cam_vps(cam_vps), .cam_sps(cam_sps),
        .cam_vlines(cam_vlines), .cam_hlen(cam_hlen), .cam_pmhz(cam_pmhz), .hitp(cam_hitp),
        .gmode(gmode), .ksel(ksel), .zp0(zp0), .zp1(zp1), .zp2(zp2), .zact(zact), .gintr(gimbal_intr), .gtool(gimbal_task_tool));
    oled_i2c_ssd1306 #(.CLK_HZ(CLK_HZ), .I2C_HZ(200000)) u_oled (
        .clk(clk), .rst_n(rst_n), .pix_page(oled_page), .pix_col(oled_col), .pix_data(oled_pix),
        .scl_oe(oled_scl_oe), .sda_oe(oled_sda_oe));
    assign oled_scl = oled_scl_oe ? 1'b0 : 1'bz;
    assign oled_sda = oled_sda_oe ? 1'b0 : 1'bz;
endmodule
