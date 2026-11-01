`timescale 1ns/1ps

// 扩展5：64 频段实时频谱分析（video_clk 25MHz 域），从 PCM 原始样本计算频谱能量
//   每个 48kHz 样本都参与分析（8 级样本 FIFO 吸收 HDMI 音频包突发，溢出置 overrun）
//   块长 N=512，Hann 窗；64 个指数分布频点（约 94Hz~20kHz），每块约 10.7ms（93.75 块/秒，高于 60 帧/秒）
//   每个样本到来后，用 64 拍流水把 64 个频点的 DFT 累加器各更新一次：
//       acc_re[k] += xw[n]*cos(2*pi*k*n/512)，acc_im[k] += xw[n]*sin(...)
//   累加器放 BRAM（64x52bit），读写地址相差 2 拍，无冲突；块末输出 |X| ≈ max+min/2 -> 对数刻度 -> 柱高
//   显示动态（每块更新一次，约 94 次/秒）：快攻慢放（每块落 1 格）+ 峰值保持 24 块后每块落 1 格
//   柱状态放 RAM，块末 64 拍结果流顺序“读-改-写”；显示用双 bank RAM，帧边界整组切换，一帧内柱高不变
//   与课程 FFT 方案相比：频点可任意指定（低频细、高频粗，更符合听感），每样本确定 64 拍完成，延迟固定
module spectrum_dft64 (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    input  wire signed [15:0] sample,
    input  wire               frame_tick,       // 每帧一拍（场消隐内）
    input  wire [5:0]         rd_bar,           // 渲染读口：柱号，1 拍后出数据
    output reg  [6:0]         rd_h,             // 柱高 0..80
    output reg  [6:0]         rd_pk,            // 峰值
    output reg  [6:0]         bass,             // 低频能量（柱 1..6 最大值），供音画联动
    output reg                overrun           // 样本 FIFO 曾溢出（粘滞）
);
    // 自动生成：Q[i] = round(32767*cos(2*pi*i/512)), i=0..128
    function [15:0] qcos;
        input [7:0] i;
        case (i)
            8'd0: qcos = 16'd32767;
            8'd1: qcos = 16'd32765;
            8'd2: qcos = 16'd32757;
            8'd3: qcos = 16'd32745;
            8'd4: qcos = 16'd32728;
            8'd5: qcos = 16'd32705;
            8'd6: qcos = 16'd32678;
            8'd7: qcos = 16'd32646;
            8'd8: qcos = 16'd32609;
            8'd9: qcos = 16'd32567;
            8'd10: qcos = 16'd32521;
            8'd11: qcos = 16'd32469;
            8'd12: qcos = 16'd32412;
            8'd13: qcos = 16'd32351;
            8'd14: qcos = 16'd32285;
            8'd15: qcos = 16'd32213;
            8'd16: qcos = 16'd32137;
            8'd17: qcos = 16'd32057;
            8'd18: qcos = 16'd31971;
            8'd19: qcos = 16'd31880;
            8'd20: qcos = 16'd31785;
            8'd21: qcos = 16'd31685;
            8'd22: qcos = 16'd31580;
            8'd23: qcos = 16'd31470;
            8'd24: qcos = 16'd31356;
            8'd25: qcos = 16'd31237;
            8'd26: qcos = 16'd31113;
            8'd27: qcos = 16'd30985;
            8'd28: qcos = 16'd30852;
            8'd29: qcos = 16'd30714;
            8'd30: qcos = 16'd30571;
            8'd31: qcos = 16'd30424;
            8'd32: qcos = 16'd30273;
            8'd33: qcos = 16'd30117;
            8'd34: qcos = 16'd29956;
            8'd35: qcos = 16'd29791;
            8'd36: qcos = 16'd29621;
            8'd37: qcos = 16'd29447;
            8'd38: qcos = 16'd29268;
            8'd39: qcos = 16'd29085;
            8'd40: qcos = 16'd28898;
            8'd41: qcos = 16'd28706;
            8'd42: qcos = 16'd28510;
            8'd43: qcos = 16'd28310;
            8'd44: qcos = 16'd28105;
            8'd45: qcos = 16'd27896;
            8'd46: qcos = 16'd27683;
            8'd47: qcos = 16'd27466;
            8'd48: qcos = 16'd27245;
            8'd49: qcos = 16'd27019;
            8'd50: qcos = 16'd26790;
            8'd51: qcos = 16'd26556;
            8'd52: qcos = 16'd26319;
            8'd53: qcos = 16'd26077;
            8'd54: qcos = 16'd25832;
            8'd55: qcos = 16'd25582;
            8'd56: qcos = 16'd25329;
            8'd57: qcos = 16'd25072;
            8'd58: qcos = 16'd24811;
            8'd59: qcos = 16'd24547;
            8'd60: qcos = 16'd24279;
            8'd61: qcos = 16'd24007;
            8'd62: qcos = 16'd23731;
            8'd63: qcos = 16'd23452;
            8'd64: qcos = 16'd23170;
            8'd65: qcos = 16'd22884;
            8'd66: qcos = 16'd22594;
            8'd67: qcos = 16'd22301;
            8'd68: qcos = 16'd22005;
            8'd69: qcos = 16'd21705;
            8'd70: qcos = 16'd21403;
            8'd71: qcos = 16'd21096;
            8'd72: qcos = 16'd20787;
            8'd73: qcos = 16'd20475;
            8'd74: qcos = 16'd20159;
            8'd75: qcos = 16'd19841;
            8'd76: qcos = 16'd19519;
            8'd77: qcos = 16'd19195;
            8'd78: qcos = 16'd18868;
            8'd79: qcos = 16'd18537;
            8'd80: qcos = 16'd18204;
            8'd81: qcos = 16'd17869;
            8'd82: qcos = 16'd17530;
            8'd83: qcos = 16'd17189;
            8'd84: qcos = 16'd16846;
            8'd85: qcos = 16'd16499;
            8'd86: qcos = 16'd16151;
            8'd87: qcos = 16'd15800;
            8'd88: qcos = 16'd15446;
            8'd89: qcos = 16'd15090;
            8'd90: qcos = 16'd14732;
            8'd91: qcos = 16'd14372;
            8'd92: qcos = 16'd14010;
            8'd93: qcos = 16'd13645;
            8'd94: qcos = 16'd13279;
            8'd95: qcos = 16'd12910;
            8'd96: qcos = 16'd12539;
            8'd97: qcos = 16'd12167;
            8'd98: qcos = 16'd11793;
            8'd99: qcos = 16'd11417;
            8'd100: qcos = 16'd11039;
            8'd101: qcos = 16'd10659;
            8'd102: qcos = 16'd10278;
            8'd103: qcos = 16'd9896;
            8'd104: qcos = 16'd9512;
            8'd105: qcos = 16'd9126;
            8'd106: qcos = 16'd8739;
            8'd107: qcos = 16'd8351;
            8'd108: qcos = 16'd7962;
            8'd109: qcos = 16'd7571;
            8'd110: qcos = 16'd7179;
            8'd111: qcos = 16'd6786;
            8'd112: qcos = 16'd6393;
            8'd113: qcos = 16'd5998;
            8'd114: qcos = 16'd5602;
            8'd115: qcos = 16'd5205;
            8'd116: qcos = 16'd4808;
            8'd117: qcos = 16'd4410;
            8'd118: qcos = 16'd4011;
            8'd119: qcos = 16'd3612;
            8'd120: qcos = 16'd3212;
            8'd121: qcos = 16'd2811;
            8'd122: qcos = 16'd2410;
            8'd123: qcos = 16'd2009;
            8'd124: qcos = 16'd1608;
            8'd125: qcos = 16'd1206;
            8'd126: qcos = 16'd804;
            8'd127: qcos = 16'd402;
            8'd128: qcos = 16'd0;
            default: qcos = 16'd0;
        endcase
    endfunction
    // 自动生成：64 个频点（bin*93.75Hz，fs=48kHz，N=512），指数分布：低频细、高频粗，约 94Hz ~ 19969Hz
    function [7:0] bin_k;
        input [5:0] b;
        case (b)
            6'd0: bin_k = 8'd1;
            6'd1: bin_k = 8'd2;
            6'd2: bin_k = 8'd3;
            6'd3: bin_k = 8'd4;
            6'd4: bin_k = 8'd5;
            6'd5: bin_k = 8'd6;
            6'd6: bin_k = 8'd7;
            6'd7: bin_k = 8'd8;
            6'd8: bin_k = 8'd9;
            6'd9: bin_k = 8'd10;
            6'd10: bin_k = 8'd11;
            6'd11: bin_k = 8'd12;
            6'd12: bin_k = 8'd13;
            6'd13: bin_k = 8'd14;
            6'd14: bin_k = 8'd15;
            6'd15: bin_k = 8'd16;
            6'd16: bin_k = 8'd17;
            6'd17: bin_k = 8'd18;
            6'd18: bin_k = 8'd19;
            6'd19: bin_k = 8'd20;
            6'd20: bin_k = 8'd21;
            6'd21: bin_k = 8'd22;
            6'd22: bin_k = 8'd23;
            6'd23: bin_k = 8'd24;
            6'd24: bin_k = 8'd25;
            6'd25: bin_k = 8'd26;
            6'd26: bin_k = 8'd27;
            6'd27: bin_k = 8'd28;
            6'd28: bin_k = 8'd29;
            6'd29: bin_k = 8'd30;
            6'd30: bin_k = 8'd31;
            6'd31: bin_k = 8'd32;
            6'd32: bin_k = 8'd33;
            6'd33: bin_k = 8'd36;
            6'd34: bin_k = 8'd38;
            6'd35: bin_k = 8'd41;
            6'd36: bin_k = 8'd43;
            6'd37: bin_k = 8'd46;
            6'd38: bin_k = 8'd49;
            6'd39: bin_k = 8'd52;
            6'd40: bin_k = 8'd55;
            6'd41: bin_k = 8'd59;
            6'd42: bin_k = 8'd62;
            6'd43: bin_k = 8'd66;
            6'd44: bin_k = 8'd70;
            6'd45: bin_k = 8'd75;
            6'd46: bin_k = 8'd79;
            6'd47: bin_k = 8'd84;
            6'd48: bin_k = 8'd89;
            6'd49: bin_k = 8'd95;
            6'd50: bin_k = 8'd101;
            6'd51: bin_k = 8'd107;
            6'd52: bin_k = 8'd113;
            6'd53: bin_k = 8'd120;
            6'd54: bin_k = 8'd127;
            6'd55: bin_k = 8'd135;
            6'd56: bin_k = 8'd143;
            6'd57: bin_k = 8'd151;
            6'd58: bin_k = 8'd160;
            6'd59: bin_k = 8'd169;
            6'd60: bin_k = 8'd179;
            6'd61: bin_k = 8'd190;
            6'd62: bin_k = 8'd201;
            6'd63: bin_k = 8'd213;
            default: bin_k = 8'd1;
        endcase
    endfunction
    function signed [16:0] cosv;                // round(32767*cos(2*pi*p/512))
        input [8:0] p;
        reg   [15:0] m;
        begin
            case (p[8:7])
                2'd0: begin m = qcos({1'b0, p[6:0]});         cosv =  $signed({1'b0, m}); end
                2'd1: begin m = qcos(8'd128 - {1'b0, p[6:0]}); cosv = -$signed({1'b0, m}); end
                2'd2: begin m = qcos({1'b0, p[6:0]});         cosv = -$signed({1'b0, m}); end
                default: begin m = qcos(8'd128 - {1'b0, p[6:0]}); cosv = $signed({1'b0, m}); end
            endcase
        end
    endfunction
    function [7:0] log_lvl;                     // 8*log2(m)，高 3 位小数，约 0.75dB 一级
        input [25:0] m;
        integer i;
        reg found;
        begin
            log_lvl = 8'd0;  found = 1'b0;
            for (i = 25; i >= 3; i = i - 1)
                if (!found && m[i]) begin
                    found = 1'b1;  log_lvl = {i[4:0], m[i-1 -: 3]};
                end
        end
    endfunction

    // ---------------- 样本 FIFO（8 深） ----------------
    reg signed [15:0] fifo [0:7];
    reg [3:0] wp, rp;
    wire fifo_empty = (wp == rp);
    wire fifo_full  = ((wp - rp) == 4'd8);
    always @(posedge clk) if (sample_valid && !fifo_full) fifo[wp[2:0]] <= sample;

    // ---------------- 控制：取样 -> 加窗 -> 64 拍扫频 ----------------
    localparam [1:0] S_IDLE = 2'd0, S_WIN = 2'd1, S_MUL = 2'd2, S_SWEEP = 2'd3;
    reg [1:0]  st;
    reg [8:0]  n, n_s;
    reg [5:0]  bcnt;
    reg signed [15:0] x_q, xw;
    reg signed [16:0] hann_q;
    wire signed [32:0] xh = x_q * hann_q;

    // 流水各级
    reg        a_v, a_first, a_last;  reg [5:0] a_bin;
    reg        b_v, b_first, b_last;  reg [5:0] b_bin;  reg [8:0] b_ph;  reg signed [15:0] b_xw;
    reg        c_v, c_first, c_last;  reg [5:0] c_bin;  reg signed [16:0] c_cos, c_sin;  reg signed [15:0] c_xw;
    reg        d_v, d_first, d_last;  reg [5:0] d_bin;  reg signed [32:0] d_mr, d_mi;
    reg        f_v;  reg [5:0] f_bin;  reg [25:0] f_ar, f_ai;
    reg        g_v;  reg [5:0] g_bin;  reg [25:0] g_mag;
    reg        h_v;  reg [5:0] h_bin;  reg [6:0]  h_ht;
    wire [16:0] kn = bin_k(a_bin) * n_s;

    // 累加器 BRAM：{re[25:0], im[25:0]}
    reg [51:0] acc_mem [0:63];
    reg [51:0] acc_rd;
    always @(posedge clk) acc_rd <= acc_mem[c_bin];   // C-stage read address -> data lines up with D-stage add/write-back (read/write addresses differ by 1 bin, no conflict)

    wire signed [17:0] tr = (d_mr + 33'sd16384) >>> 15;
    wire signed [17:0] ti = (d_mi + 33'sd16384) >>> 15;
    wire signed [25:0] old_re = d_first ? 26'sd0 : $signed(acc_rd[51:26]);
    wire signed [25:0] old_im = d_first ? 26'sd0 : $signed(acc_rd[25:0]);
    wire signed [25:0] new_re = old_re + {{8{tr[17]}}, tr};
    wire signed [25:0] new_im = old_im + {{8{ti[17]}}, ti};
    always @(posedge clk) if (d_v) acc_mem[d_bin] <= {new_re, new_im};

    wire [25:0] mx = (f_ar > f_ai) ? f_ar : f_ai;
    wire [25:0] mn = (f_ar > f_ai) ? f_ai : f_ar;
    wire [7:0]  lv = log_lvl(g_mag);
    wire [6:0]  ht = (lv >= 8'd176) ? 7'd80 : (lv <= 8'd96) ? 7'd0 : lv - 8'd96;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= 4'd0; rp <= 4'd0; overrun <= 1'b0;
            st <= S_IDLE; n <= 9'd0; n_s <= 9'd0; bcnt <= 6'd0;
            x_q <= 16'sd0; xw <= 16'sd0; hann_q <= 17'sd0;
            {a_v, b_v, c_v, d_v, f_v, g_v, h_v} <= 7'd0;
        end else begin
            if (sample_valid) begin
                if (fifo_full) overrun <= 1'b1; else wp <= wp + 4'd1;
            end
            a_v <= 1'b0;
            case (st)
                S_IDLE: if (!fifo_empty) begin
                    x_q <= fifo[rp[2:0]];  rp <= rp + 4'd1;
                    n_s <= n;  n <= n + 9'd1;  st <= S_WIN;
                end
                S_WIN: begin                                   // Hann: (1-cos)/2，Q15
                    hann_q <= 17'sd16384 - (cosv(n_s) >>> 1);  st <= S_MUL;
                end
                S_MUL: begin
                    xw <= xh[30:15];  bcnt <= 6'd0;  st <= S_SWEEP;
                end
                default: begin
                    a_v <= 1'b1;  a_bin <= bcnt;
                    a_first <= (n_s == 9'd0);  a_last <= (n_s == 9'd511);
                    bcnt <= bcnt + 6'd1;
                    if (bcnt == 6'd63) st <= S_IDLE;
                end
            endcase
            // B：相位 k*n mod 512
            b_v <= a_v;  b_bin <= a_bin;  b_first <= a_first;  b_last <= a_last;
            b_ph <= kn[8:0];  b_xw <= xw;
            // C：查余弦/正弦（累加器在 C 级发读地址）
            c_v <= b_v;  c_bin <= b_bin;  c_first <= b_first;  c_last <= b_last;  c_xw <= b_xw;
            c_cos <= cosv(b_ph);  c_sin <= cosv(b_ph - 9'd128);
            // D：两路乘法（DSP）
            d_v <= c_v;  d_bin <= c_bin;  d_first <= c_first;  d_last <= c_last;
            d_mr <= c_xw * c_cos;  d_mi <= c_xw * c_sin;
            // E（写回同拍）-> F：块末取绝对值
            f_v <= d_v && d_last;  f_bin <= d_bin;
            f_ar <= new_re[25] ? -new_re : new_re;
            f_ai <= new_im[25] ? -new_im : new_im;
            // G：|X| ≈ max + min/2
            g_v <= f_v;  g_bin <= f_bin;  g_mag <= mx + (mn >> 1);
            // H：对数刻度 -> 柱高
            h_v <= g_v;  h_bin <= g_bin;  h_ht <= ht;
        end
    end

    // ---------------- 64 根柱子：状态 RAM 读-改-写（单写者，无冲突） ----------------
    reg [18:0] st_mem [0:63];                   // {hold[4:0], pk[6:0], disp[6:0]}
    reg [18:0] st_q;
    always @(posedge clk) st_q <= st_mem[g_bin];   // G 级发读地址，H 级拿到同一柱的旧状态
    wire [6:0] o_disp = st_q[6:0];
    wire [6:0] o_pk   = st_q[13:7];
    wire [4:0] o_hold = st_q[18:14];
    wire [6:0] n_disp = (h_ht > o_disp) ? h_ht : (o_disp != 7'd0) ? o_disp - 7'd1 : 7'd0;
    wire       pk_up  = (h_ht >= o_pk);
    wire [6:0] n_pk   = pk_up ? h_ht : (o_hold == 5'd0 && o_pk != 7'd0) ? o_pk - 7'd1 : o_pk;
    wire [4:0] n_hold = pk_up ? 5'd24 : (o_hold != 5'd0) ? o_hold - 5'd1 : 5'd0;
    always @(posedge clk) if (h_v) st_mem[h_bin] <= {n_hold, n_pk, n_disp};

    // 显示 RAM：两个 bank，写满一整块后在下一个帧边界切换给渲染端
    reg [13:0] dsp_mem [0:127];
    reg        wbank, ready;
    reg [6:0]  bass_run;
    always @(posedge clk) if (h_v) dsp_mem[{wbank, h_bin}] <= {n_pk, n_disp};
    reg [13:0] dsp_q;
    // 上电清零（BRAM 初值），保证首块读-改-写从 0 开始，仿真与上板一致
    integer z;
    initial for (z = 0; z < 128; z = z + 1) begin dsp_mem[z] = 14'd0; if (z < 64) st_mem[z] = 19'd0; end
    always @(posedge clk) dsp_q <= dsp_mem[{~wbank, rd_bar}];
    always @* begin rd_h = dsp_q[6:0];  rd_pk = dsp_q[13:7]; end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wbank <= 1'b0;  ready <= 1'b0;  bass <= 7'd0;  bass_run <= 7'd0;
        end else begin
            if (h_v) begin
                if (h_bin == 6'd0) bass_run <= 7'd0;
                else if (h_bin <= 6'd6 && n_disp > bass_run) bass_run <= n_disp;
                if (h_bin == 6'd63) begin ready <= 1'b1;  bass <= bass_run; end
            end
            // 帧边界且不在写块中途：整组切换
            if (frame_tick && ready && !h_v) begin
                wbank <= ~wbank;  ready <= 1'b0;
            end
        end
    end
endmodule
