`timescale 1ns/1ps
// OV5640 上电 + SCCB 配置（流程同正点原子 atk_mc5640_init）：
//   PWDN/RST 上电时序 -> 读改写 0x3103、软复位 0x3008=0x82 -> 读芯片 ID 0x300A/0x300B（应为 0x5640）
//   -> 逐条写 ov5640_rom（YUV422 160x120）。ID 不对（没插好/没上电）时保持上电，每秒重新读一次 ID
// SCCB：器件地址 0x3C（写 0x78 / 读 0x79），16 位寄存器地址，约 50kHz，开漏（oe=1 拉低，模块上有上拉）
module ov5640_init #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SCCB_HZ = 50000                 // 只靠上拉，放慢一点留余量
)(
    input  wire clk, rst_n,
    output reg  cam_rst_n, cam_pwdn,
    output reg  scl_oe, sda_oe,
    input  wire sda_in,
    output reg  [1:0] status,           // 0 初始化中  1 运行  2 芯片 ID 不对
    output reg  [15:0] chip_id          // 最近一次读到的 ID（FFFF = 没应答，0000 = SDA 被拉低）
);
    localparam integer QC = CLK_HZ / (SCCB_HZ * 4);
    localparam integer MS = CLK_HZ / 1000;

    // ---------------- SCCB 事务：写 = S 78 AH AL D P；读 = S 78 AH AL P S 79 [R+NACK] P ----------------
    localparam [1:0] K_S = 0, K_W = 1, K_R = 2, K_P = 3;
    reg t_go, t_rd, t_done, act; reg [15:0] t_reg; reg [7:0] t_dat, t_q;
    reg [3:0] k, b; reg [1:0] q; reg [15:0] qc; reg [1:0] sda_s;
    reg [1:0] kind; reg [7:0] byt; reg last;
    always @* begin
        last = 0; byt = 8'h78; kind = K_W;
        if (!t_rd) case (k)
            0: kind = K_S;
            1: ;
            2: byt = t_reg[15:8];
            3: byt = t_reg[7:0];
            4: byt = t_dat;
            default: begin kind = K_P; last = 1; end
        endcase else case (k)
            0, 5: kind = K_S;
            1: ;
            2: byt = t_reg[15:8];
            3: byt = t_reg[7:0];
            4: kind = K_P;
            6: byt = 8'h79;
            7: kind = K_R;
            default: begin kind = K_P; last = 1; end
        endcase
    end
    // 每个符号 4 个 1/4 周期；字节 9 位（第 9 位放开 SDA：写时等 ACK，读时给 NACK）
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin act<=0; t_done<=0; k<=0; b<=0; q<=0; qc<=0; scl_oe<=0; sda_oe<=0; t_q<=0; sda_s<=2'b11; end
        else begin
            sda_s <= {sda_s[0], sda_in};
            t_done <= 0;
            if (t_go) begin act <= 1; k <= 0; b <= 0; q <= 0; qc <= 0; end
            else if (act) begin
                if (qc != QC - 1) qc <= qc + 1'b1;
                else begin
                    qc <= 0; q <= q + 1'b1;
                    case (kind)
                        K_S: case (q) 0: begin scl_oe <= 0; sda_oe <= 0; end 2: sda_oe <= 1; 3: scl_oe <= 1; default: ; endcase
                        K_P: case (q) 0: sda_oe <= 1; 1: scl_oe <= 0; 2: sda_oe <= 0; default: ; endcase
                        default: case (q)
                            0: sda_oe <= (kind == K_W && b < 8) ? ~byt[3'd7 - b[2:0]] : 1'b0;
                            1: scl_oe <= 0;
                            2: if (kind == K_R && b < 8) t_q <= {t_q[6:0], sda_s[1]};
                            default: scl_oe <= 1;
                        endcase
                    endcase
                    if (q == 2'd3) begin
                        if ((kind == K_W || kind == K_R) && b != 4'd8) b <= b + 1'b1;
                        else begin b <= 0; if (last) begin act <= 0; t_done <= 1; end else k <= k + 1'b1; end
                    end
                end
            end
        end

    // ---------------- 上电 / 配置流程 ----------------
    localparam [3:0] P_PWR=0, P_RST=1, P_WAKE=2, P_R3103=3, P_W3103=4, P_W3008=5, P_SWR=6, P_IDH=7, P_IDL=8,
                     P_TAB=9, P_RUN=10, P_ERR=11;
    reg [3:0] ph; reg iss; reg [10:0] wms; reg [16:0] mc; reg [8:0] ti; reg [7:0] tmp;
    wire [23:0] rv; wire tlast;
    ov5640_rom u_rom (.idx(ti), .rv(rv), .last(tlast));
    wire ms_tick = mc == MS - 1;
    wire wdone = wms == 0;
    task xfer(input rd, input [15:0] ra, input [7:0] dv);
        begin t_go <= 1; t_rd <= rd; t_reg <= ra; t_dat <= dv; iss <= 1; end
    endtask
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            ph<=P_PWR; iss<=0; wms<=20; mc<=0; ti<=0; tmp<=0; t_go<=0; t_rd<=0; t_reg<=0; t_dat<=0;
            cam_rst_n<=0; cam_pwdn<=1; status<=0; chip_id<=0;
        end else begin
            t_go <= 0;
            mc <= ms_tick ? 17'd0 : mc + 1'b1;
            if (ms_tick && wms != 0) wms <= wms - 1'b1;
            case (ph)
                P_PWR:   if (wdone) begin cam_pwdn <= 0; wms <= 5; ph <= P_RST; end          // RST 低、PWDN 高 20ms
                P_RST:   if (wdone) begin cam_rst_n <= 1; wms <= 20; ph <= P_WAKE; end       // PWDN 低 5ms 后放开 RST
                P_WAKE:  if (wdone) ph <= P_R3103;
                P_R3103: if (!iss) xfer(1, 16'h3103, 8'h00); else if (t_done) begin iss <= 0; tmp <= t_q & 8'hfd; ph <= P_W3103; end
                P_W3103: if (!iss) xfer(0, 16'h3103, tmp);   else if (t_done) begin iss <= 0; ph <= P_W3008; end
                P_W3008: if (!iss) xfer(0, 16'h3008, 8'h82); else if (t_done) begin iss <= 0; wms <= 10; ph <= P_SWR; end
                P_SWR:   if (wdone) ph <= P_IDH;
                P_IDH:   if (!iss) xfer(1, 16'h300a, 8'h00); else if (t_done) begin iss <= 0; tmp <= t_q; ph <= P_IDL; end
                P_IDL:   if (!iss) xfer(1, 16'h300b, 8'h00);
                         else if (t_done) begin
                             iss <= 0; ti <= 0; chip_id <= {tmp, t_q};
                             if ({tmp, t_q} == 16'h5640) ph <= P_TAB; else begin status <= 2; wms <= 1000; ph <= P_ERR; end
                         end
                P_TAB:   if (!iss) xfer(0, rv[23:8], rv[7:0]);
                         else if (t_done) begin iss <= 0; if (tlast) begin status <= 1; ph <= P_RUN; end else ti <= ti + 1'b1; end
                P_ERR:   if (wdone) ph <= P_R3103;                                       // 保持上电只重试通信，方便量电压
                default: ;
            endcase
        end
endmodule
