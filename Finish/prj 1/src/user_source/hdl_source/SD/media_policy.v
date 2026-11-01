`timescale 1ns/1ps

// 2.9 第三层：媒体策略（菜单版）
// 只做“按键事件 -> 行为命令/参数”的翻译，按当前界面解释同一个按键：
//
//   按键          全屏播放          菜单（选择项）        菜单（编辑值）      缩略图界面
//   key1 单击     下一张            光标下移              值 +1               焦点右移
//   key3 单击     上一张            光标上移              值 -1               焦点左移
//   key2 单击     打开菜单          进入编辑 / 执行动作   确认，回到选择      打开焦点图
//   key2 长按 1s  进入缩略图        关闭菜单              关闭菜单            返回全屏
//   key1 长按 1s  自动轮播 开/关    关闭菜单              关闭菜单            -
//   菜单 10 秒无操作自动关闭；数值类改动立即生效（边调边看）
//
// 菜单项（menu_item）：
//   0 亮度  1 对比度  2 音量  3 轮播周期  4 自动轮播  5 可视化  6 白天/夜间  7 色彩特效
//   8 叠加层  9 当前图片  10~14 超限阈值 V MIN / V MAX / I MAX / T MAX / VIB LIM（各 4 档，掉电保存）
//   15 缩略图(动作)  16 保存(动作)
//   17 声音来源：MUSIC（设备运行录音 MUSIC.WAV）/ TONE（48kHz 内部测试音，WAV 原地暂停）
//   18 退出
// 报警只有两种：参数越限/数据超时时屏幕自动弹警告（无声，恢复后自动消失）；SW3 拨上手动报警（警报音）。
// 菜单里不再有确认/解除项。
// 可视化/白天夜间/特效在 uart_cmd 里保存，本模块通过 p_wr/p_id/p_val 写过去，
// 串口和菜单改的是同一份寄存器
module media_policy #(
    parameter integer CLK_HZ = 100_000_000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       prev_key_event,       // key3 单击
    input  wire       next_key_event,       // key1 单击
    input  wire       next_key_long_event,  // key1 长按后松开
    input  wire       auto_key_stable_n,    // key2 消抖后电平（低有效）
    input  wire [2:0] found_count,
    input  wire [1:0] disp_idx,
    // Flash 上电恢复（单拍，值已解包）与 UART 叠加层循环
    input  wire       cfg_load,
    input  wire [2:0] cfg_bright, cfg_contrast,
    input  wire [3:0] cfg_vol,
    input  wire [1:0] cfg_period, cfg_overlay,
    input  wire       ovl_cycle,
    input  wire       cfg_snd,
    // uart_cmd 中参数的当前值（菜单在此基础上 +1/-1）
    input  wire [1:0] cur_viz,
    input  wire       cur_filt,
    input  wire [2:0] cur_fx,
    input  wire [9:0] cfg_lim,              // Flash 读回的阈值档位
    output reg        p_wr,                 // 参数写：p_id 0 可视化 1 白天/夜间 2 特效
    output reg  [2:0] p_id,
    output reg  [2:0] p_val,
    output reg        save_req,             // 菜单“保存”
    output reg        snd_tone,             // 1：强制旋律音
    output reg        next_cmd,
    output reg        prev_cmd,
    output reg        auto_cmd,
    output reg        select_cmd,
    output reg  [1:0] select_idx,
    output reg        view_mode,            // 0 全屏 1 缩略图
    output reg  [1:0] focus,
    output reg        menu_active,
    output reg  [4:0] menu_item,
    output reg        menu_edit,            // 1：正在编辑当前项的值
    output reg  [9:0] lim_idx,              // 阈值档位 {vib,tmax,imax,vmax,vmin} 各 2 位
    output reg        lim_load,             // 档位变了（单拍）
    output reg  [2:0] bright_lvl,
    output reg  [2:0] contrast_lvl,
    output reg  [3:0] vol_lvl,
    output reg  [1:0] period_sel,
    output reg  [1:0] overlay_mode,
    output reg        vol_event,            // 音量改变（数码管强制显示 VOL 页）
    output reg        prd_event             // 周期改变
);
    localparam [31:0] LONG_CYCLES = CLK_HZ;           // 1 s
    localparam [31:0] IDLE_CYCLES = CLK_HZ * 10;      // 10 s

    localparam [4:0] M_BRT = 5'd0,  M_CON = 5'd1,  M_VOL = 5'd2,  M_PRD = 5'd3,
                     M_AUTO = 5'd4, M_VIZ = 5'd5,  M_FLT = 5'd6,  M_FX  = 5'd7,
                     M_OVL = 5'd8,  M_PIC = 5'd9,  M_VMIN = 5'd10, M_VMAX = 5'd11,
                     M_IMAX = 5'd12, M_TMAX = 5'd13, M_VIB = 5'd14,
                     M_THUMB = 5'd15, M_SAVE = 5'd16, M_SND = 5'd17,
                     M_EXIT = 5'd18, M_LAST = 5'd18;
    // 默认档位：VIB 40 / T 60.0C / I 60mA / V MAX 3400mV / V MIN 3200mV（与 board_monitor 复位值一致）
    localparam [9:0] LIM_DEF = {2'd1, 2'd3, 2'd1, 2'd1, 2'd2};
    function [1:0] step2;
        input [1:0] v;
        input u;
        step2 = u ? ((v == 2'd3) ? v : v + 2'd1) : ((v == 2'd0) ? v : v - 2'd1);
    endfunction

    // ---------- key2 单击 / 长按 ----------
    reg        k2_q, k2_long_done;
    reg [31:0] k2_cnt;
    reg        k2_short, k2_long;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            k2_q <= 1'b1;  k2_long_done <= 1'b0;  k2_cnt <= 32'd0;
            k2_short <= 1'b0;  k2_long <= 1'b0;
        end else begin
            k2_q <= auto_key_stable_n;
            k2_short <= 1'b0;
            k2_long  <= 1'b0;
            if (!auto_key_stable_n) begin
                if (k2_q) begin
                    k2_cnt <= 32'd0;  k2_long_done <= 1'b0;
                end else if (!k2_long_done) begin
                    if (k2_cnt >= LONG_CYCLES - 1) begin
                        k2_long <= 1'b1;  k2_long_done <= 1'b1;   // 按满 1s 立即生效
                    end else
                        k2_cnt <= k2_cnt + 32'd1;
                end
            end else if (!k2_q && !k2_long_done)
                k2_short <= 1'b1;                                 // 不到 1s 松开
        end
    end

    wire nk_ev  = next_key_event;
    wire pk_ev  = prev_key_event;
    wire nl_ev  = next_key_long_event;
    wire k2s_ev = k2_short;
    wire k2l_ev = k2_long;

    // 焦点在已找到的图片之间循环
    wire [1:0] last_idx = (found_count == 3'd0) ? 2'd0 : found_count[1:0] - 2'd1;
    wire [1:0] focus_nx = (found_count >= 3'd4) ? focus + 2'd1 : (focus >= last_idx) ? 2'd0 : focus + 2'd1;
    wire [1:0] focus_pv = (found_count >= 3'd4) ? focus - 2'd1 : (focus == 2'd0) ? last_idx : focus - 2'd1;

    wire       up = nk_ev;

    reg [31:0] idle_cnt;
    reg        idle_to;                     // 超时比较提前一拍寄存，缩短到菜单判断的组合路径
    always @(posedge clk or negedge rst_n)
        if (!rst_n) idle_to <= 1'b0;
        else        idle_to <= menu_active && (idle_cnt >= IDLE_CYCLES);
    wire any_key = nk_ev | pk_ev | nl_ev | k2s_ev | k2l_ev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            next_cmd <= 1'b0; prev_cmd <= 1'b0; auto_cmd <= 1'b0; select_cmd <= 1'b0;
            select_idx <= 2'd0; view_mode <= 1'b0; focus <= 2'd0;
            menu_active <= 1'b0; menu_item <= 5'd0; menu_edit <= 1'b0;
            lim_idx <= LIM_DEF; lim_load <= 1'b0;
            bright_lvl <= 3'd3; contrast_lvl <= 3'd3; vol_lvl <= 4'd8; period_sel <= 2'd1;
            overlay_mode <= 2'd0; vol_event <= 1'b0; prd_event <= 1'b0; idle_cnt <= 32'd0;
            p_wr <= 1'b0; p_id <= 3'd0; p_val <= 3'd0; save_req <= 1'b0;
            snd_tone <= 1'b0;
        end else begin
            next_cmd <= 1'b0; prev_cmd <= 1'b0; auto_cmd <= 1'b0; select_cmd <= 1'b0;
            vol_event <= 1'b0; prd_event <= 1'b0; p_wr <= 1'b0; save_req <= 1'b0; lim_load <= 1'b0;

            if (ovl_cycle)
                overlay_mode <= (overlay_mode == 2'd2) ? 2'd0 : overlay_mode + 2'd1;
            if (cfg_load) begin                                   // 越界值按默认处理
                bright_lvl   <= (cfg_bright   > 3'd6)  ? 3'd3 : cfg_bright;
                contrast_lvl <= (cfg_contrast > 3'd6)  ? 3'd3 : cfg_contrast;
                vol_lvl      <= (cfg_vol      > 4'd10) ? 4'd8 : cfg_vol;
                period_sel   <= cfg_period;
                snd_tone     <= cfg_snd;
                overlay_mode <= (cfg_overlay  == 2'd3) ? 2'd0 : cfg_overlay;
                lim_idx      <= cfg_lim;  lim_load <= 1'b1;
            end

            if (menu_active) begin
                idle_cnt <= any_key ? 32'd0 : idle_cnt + 32'd1;
                if (nl_ev || k2l_ev || idle_to) begin
                    menu_active <= 1'b0;  menu_edit <= 1'b0;
                end else if (!menu_edit) begin
                    // ---- 选择项 ----
                    if (nk_ev)      menu_item <= (menu_item == M_LAST) ? 5'd0 : menu_item + 5'd1;
                    else if (pk_ev) menu_item <= (menu_item == 5'd0) ? M_LAST : menu_item - 5'd1;
                    else if (k2s_ev) begin
                        case (menu_item)
                            M_THUMB: begin
                                menu_active <= 1'b0;
                                if (found_count != 3'd0) begin view_mode <= 1'b1;  focus <= disp_idx; end
                            end
                            M_SAVE:  save_req <= 1'b1;
                            M_EXIT:  menu_active <= 1'b0;
                            default: menu_edit <= 1'b1;
                        endcase
                    end
                end else begin
                    // ---- 编辑值 ----
                    if (k2s_ev) menu_edit <= 1'b0;
                    else if (nk_ev || pk_ev) begin
                        case (menu_item)
                            M_BRT: if (up && bright_lvl < 3'd6) bright_lvl <= bright_lvl + 3'd1;
                                   else if (!up && bright_lvl != 3'd0) bright_lvl <= bright_lvl - 3'd1;
                            M_CON: if (up && contrast_lvl < 3'd6) contrast_lvl <= contrast_lvl + 3'd1;
                                   else if (!up && contrast_lvl != 3'd0) contrast_lvl <= contrast_lvl - 3'd1;
                            M_VOL: begin
                                   vol_event <= 1'b1;
                                   if (up && vol_lvl < 4'd10) vol_lvl <= vol_lvl + 4'd1;
                                   else if (!up && vol_lvl != 4'd0) vol_lvl <= vol_lvl - 4'd1;
                            end
                            M_PRD: begin
                                   prd_event <= 1'b1;
                                   period_sel <= up ? period_sel + 2'd1 : period_sel - 2'd1;   // 2/3/5/10 循环
                            end
                            M_AUTO: auto_cmd <= 1'b1;
                            M_VIZ:  begin p_wr <= 1'b1; p_id <= 3'd0; p_val <= {1'b0, up ? cur_viz + 2'd1 : cur_viz - 2'd1}; end
                            M_FLT:  begin p_wr <= 1'b1; p_id <= 3'd1; p_val <= {2'd0, ~cur_filt}; end
                            M_FX:   begin p_wr <= 1'b1; p_id <= 3'd2; p_val <= up ? cur_fx + 3'd1 : cur_fx - 3'd1; end
                            M_OVL:  overlay_mode <= up ? ((overlay_mode >= 2'd2) ? 2'd0 : overlay_mode + 2'd1)
                                                       : ((overlay_mode == 2'd0) ? 2'd2 : overlay_mode - 2'd1);
                            M_PIC:  if (up) next_cmd <= 1'b1; else prev_cmd <= 1'b1;
                            M_VMIN: begin lim_idx[1:0] <= step2(lim_idx[1:0], up); lim_load <= 1'b1; end
                            M_VMAX: begin lim_idx[3:2] <= step2(lim_idx[3:2], up); lim_load <= 1'b1; end
                            M_IMAX: begin lim_idx[5:4] <= step2(lim_idx[5:4], up); lim_load <= 1'b1; end
                            M_TMAX: begin lim_idx[7:6] <= step2(lim_idx[7:6], up); lim_load <= 1'b1; end
                            M_VIB:  begin lim_idx[9:8] <= step2(lim_idx[9:8], up); lim_load <= 1'b1; end
                            M_SND:  snd_tone <= ~snd_tone;
                            default: ;
                        endcase
                    end
                end
            end else if (view_mode) begin
                if (nk_ev)       focus <= focus_nx;
                else if (pk_ev)  focus <= focus_pv;
                else if (k2s_ev) begin
                    select_cmd <= 1'b1;  select_idx <= focus;  view_mode <= 1'b0;
                end else if (k2l_ev)
                    view_mode <= 1'b0;
            end else begin
                if (nk_ev)            next_cmd <= 1'b1;
                else if (pk_ev)       prev_cmd <= 1'b1;
                else if (nl_ev)       auto_cmd <= 1'b1;
                else if (k2s_ev) begin
                    menu_active <= 1'b1;  menu_edit <= 1'b0;  idle_cnt <= 32'd0;
                end else if (k2l_ev && found_count != 3'd0) begin
                    view_mode <= 1'b1;  focus <= disp_idx;
                end
            end
        end
    end
endmodule
