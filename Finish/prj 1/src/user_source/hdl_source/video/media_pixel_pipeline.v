`timescale 1ns/1ps

// 像素处理流水线，video_clk 25MHz 域，640x480，固定 11 拍延迟（hs/vs/de 同步打拍）
//   SDRAM 图像 ─> [3 拍延迟 + 夜间模式] ─┬─> [缩略图界面] ─> 选择 ─> [亮度/对比度]
//                                          └────────────────┘     ─> [淡入淡出/擦除]
//   ─> [色彩特效] ─> [状态OSD/参数行/滚动字幕] ─> [64段频谱] ─> [音频波形 + 设备声学/振动监测]
//   ─> [按键菜单] ─> [晶圆厂看板：动态字段 + 警告横幅] ─> HDMI
//   菜单/亮度等控制量在“最后一个有效行结束”时（场消隐开始）一起锁存，一帧内不变；
//   看板数据和字幕在顶层跨域时已经整组同步，这里不再复制一份帧锁存（省 830 多个寄存器），
//   数值在帧中间变化最多让一帧上下半屏差一个刷新，看不出来
module media_pixel_pipeline #(parameter DYNAMIC_MODE=1) (
    input             clk,
    input             rst_n,
    input             hs_in, vs_in, de_in,
    input      [23:0] rgb_in,
    // 控制（已跨域到本域，慢变）
    input      [1:0]  disp_idx,
    input             player_active,
    input             view_mode,
    input      [1:0]  focus,
    input      [3:0]  complete, failed,
    input      [2:0]  found,
    input      [2:0]  bright_lvl, contrast_lvl,
    input      [3:0]  vol_lvl,
    input      [1:0]  period_sel, overlay_mode,
    input wire mode_800,
    input      [4:0]  menu_item,          // 19 项菜单
    input             menu_active, menu_edit, auto_on, wav_mode,
    input      [2:0]  err_code,
    input      [1:0]  viz,
    input             filt,               // 0 白天 1 夜间
    input      [2:0]  fx,                 // 色彩特效
    input             snd_tone,           // 1：菜单强制旋律音
    input      [9:0]  lim_idx,            // 阈值档位（菜单显示；[9:8] 振动阈值）
    input             detail_mode,
    input [7:0] picture_ids,
    input [318:0] process_data,           // {manual, 看板数据 270 位, 日期时间 48 位}
    output [1:0] selected_profile,
    output selected_valid,
    input             uf_pulse,
    input      [511:0] sub_vec,
    input      [6:0]  sub_len,
    input             pcm_tap_valid,
    input  signed [15:0] pcm_tap_left,
    // 缩略图写口（sd_card_clk 域）
    input             wclk, we_even, we_odd,
    input      [13:0] waddr,
    input      [15:0] wdata,
    // 转场
    input      [7:0]  alpha,
    input             wipe_active,
    input      [1:0]  wipe_kind,
    input      [4:0]  ramp,
    output            frame_start,
    output            index_commit,
    output reg        pa_lat,
    output reg [1:0]  read_idx,
    output            emerg_lat,          // SW3 手动报警（帧锁存），驱动警报音
    output            banner_on,          // 看板警告横幅显示中
    output            hs_out, vs_out, de_out,
    output     [9:0]  x_out, y_out,       // 与 rgb_out 对齐的像素坐标
    output reg [23:0] rgb_out
);
    localparam integer N = DYNAMIC_MODE?10:9;
    localparam integer VIZ_STAGE=DYNAMIC_MODE?8:7;
    localparam integer BOARD_STAGE=DYNAMIC_MODE?9:8;
    reg hs_a, vs_a, de_a;  reg [23:0] rgb_a;  reg [9:0] x_a;  reg [9:0] y_a;
    reg [N:1] hs_sr, vs_sr, de_sr;
    reg [9:0] x_sr [1:N];
    reg [9:0] y_sr [1:N];
    reg [23:0] sel_rgb, fade_rgb, menu_rgb_q,text_rgb_q;
    reg commit_pend;
    wire [23:0] thumb_rgb, bc_rgb, fx_rgb, fade_px, osd_rgb, txt_rgb, spec_rgb, wave_rgb, alrt_rgb, ovl_rgb;
    wire native_800=DYNAMIC_MODE && mode_800;
    function [8:0] ui_y;input[9:0] n;begin ui_y=n>=480?9'd511:n[8:0];end endfunction
    reg view_l, menu_l, edit_l, auto_l, wav_l;  reg [1:0] focus_l, ovl_l, prd_l;  reg [4:0] item_l;
    reg [2:0] fx_l;  reg snd_l, night_l;
    reg [3:0] comp_l, fail_l, vol_l;    reg [2:0] found_l, bl_l, cl_l, err_l;
    reg [1:0] viz_l;  reg [9:0] lim_l;
    reg [6:0] uf_cnt;

    function [7:0] gain;                 // 档位 -> Q1.7 增益：0.50 .. 1.50
        input [2:0] l;
        case (l) 3'd0: gain = 8'd64;  3'd1: gain = 8'd85;  3'd2: gain = 8'd107; 3'd3: gain = 8'd128;
                 3'd4: gain = 8'd149; 3'd5: gain = 8'd171; default: gain = 8'd192; endcase
    endfunction
    // 夜间模式：整体压暗并偏暖（R 3/4，G 5/8，B 3/8），只用移位相加
    function [23:0] night;
        input [23:0] c;
        night={{1'b0,c[23:17]}+{2'b0,c[23:18]},
               {1'b0,c[15:9]}+{3'b0,c[15:11]},
               {2'b0,c[7:2]}+{3'b0,c[7:3]}};
    endfunction

    wire frame_end = de_a && !de_in && (y_a == (native_800?599:479));
    assign frame_start  = native_800 ? (!vs_sr[N] && vs_sr[N-1]) : (vs_sr[N] && !vs_sr[N-1]);
    assign index_commit = frame_start && commit_pend;
    assign emerg_lat    = process_data[318];

    integer k;
    always @(posedge clk) begin
        x_sr[1] <= x_a;  y_sr[1] <= y_a;
        for (k = 2; k <= N; k = k + 1) begin x_sr[k] <= x_sr[k-1]; y_sr[k] <= y_sr[k-1]; end
    end

    reg detail_l;
    reg [1:0] profile_l;
    wire [23:0] board_rgb;
    reg [23:0] d1, d2, d3;               // 原 3x3 滤波的 3 拍延迟
    assign selected_profile=profile_l;
    assign selected_valid=pa_lat && comp_l[read_idx];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            detail_l<=0; profile_l<=0;
            hs_a <= 1'b1; vs_a <= 1'b1; de_a <= 1'b0; rgb_a <= 24'd0; x_a <= 10'd0; y_a <= 9'd0;
            hs_sr <= {N{1'b1}}; vs_sr <= {N{1'b1}}; de_sr <= {N{1'b0}};
            sel_rgb <= 24'd0; fade_rgb <= 24'd0;menu_rgb_q<=0;text_rgb_q<=0; d1<=0; d2<=0; d3<=0;
            commit_pend <= 1'b0; pa_lat <= 1'b0; read_idx <= 2'd0;
            {view_l, menu_l, edit_l, auto_l, wav_l} <= 5'd0; {focus_l, ovl_l} <= 4'd0; item_l <= 5'd0;
            prd_l <= 2'd1; fx_l <= 3'd0; snd_l <= 1'b0; night_l <= 1'b0;
            {comp_l, fail_l} <= 8'd0; vol_l <= 4'd8; found_l <= 3'd0; bl_l <= 3'd3; cl_l <= 3'd3; err_l <= 3'd0;
            viz_l <= 2'd3; lim_l <= 10'd0;
            uf_cnt <= 7'd0;
        end else begin
            // 第 0 拍：坐标
            hs_a <= hs_in;  vs_a <= vs_in;  de_a <= de_in;  rgb_a <= de_in ? rgb_in : 24'd0;
            x_a  <= (de_in && de_a) ? x_a + 10'd1 : 10'd0;
            if (native_800 ? vs_in : !vs_in) y_a <= 0;
            else if (de_a && !de_in)  y_a <= y_a + 9'd1;

            hs_sr <= {hs_sr[N-1:1], hs_a};  vs_sr <= {vs_sr[N-1:1], vs_a};  de_sr <= {de_sr[N-1:1], de_a};

            // 显示健康：读 FIFO 断供后保持 UF 约 2 秒（120 帧）
            if (uf_pulse)                           uf_cnt <= 7'd120;
            else if (frame_start && uf_cnt != 7'd0) uf_cnt <= uf_cnt - 7'd1;

            // 帧边界锁存（最后一个有效行刚结束，离下一帧读 SDRAM 还有约 10 行）
            if (frame_end) begin
                detail_l<=detail_mode;
                profile_l<=picture_ids[disp_idx*2 +: 2];
                if (disp_idx != read_idx) commit_pend <= 1'b1;      // 切图：揭示转场
                read_idx <= disp_idx;
                pa_lat <= player_active;
                view_l <= view_mode; menu_l <= menu_active; edit_l <= menu_edit; auto_l <= auto_on; wav_l <= wav_mode;
                night_l <= filt; fx_l <= fx; snd_l <= snd_tone;
                focus_l <= focus; ovl_l <= overlay_mode; item_l <= menu_item; prd_l <= period_sel;
                comp_l <= complete; fail_l <= failed; vol_l <= vol_lvl; found_l <= found;
                bl_l <= bright_lvl; cl_l <= contrast_lvl; err_l <= err_code;
                viz_l <= viz; lim_l <= lim_idx;
            end
            if (index_commit) commit_pend <= 1'b0;

            // 第 1~3 拍：延迟（与缩略图对齐），第 3 拍做夜间模式
            d1 <= rgb_a;  d2 <= d1;  d3 <= night_l ? night(d2) : d2;
            // 第 4 拍：全屏 / 缩略图 选择
            sel_rgb <= !pa_lat ? 24'd0 : view_l ? thumb_rgb : d3;
            // 第 7 拍：转场
            fade_rgb <= fade_px;
            // 第 8 拍寄存菜单，第 9 拍输出看板叠加，hs/vs/de 同步增加一拍
            text_rgb_q <= txt_rgb;
            menu_rgb_q <= ovl_rgb;
            rgb_out <= board_rgb;
        end
    end

    // ---------------- 缩略图界面（3 拍） ----------------
    thumb_view u_thumb (
        .clk(clk), .rst_n(rst_n), .de(de_a), .x(x_a), .y(ui_y(y_a)),
        .focus(focus_l), .complete(comp_l), .failed(fail_l), .found(found_l),
        .wclk(wclk), .we_even(we_even), .we_odd(we_odd), .waddr(waddr), .wdata(wdata),
        .rgb(thumb_rgb));

    // ---------------- 亮度 / 对比度（2 拍：第 4 -> 6） ----------------
    brightness_contrast_adjust u_bc (
        .clk(clk), .rst_n(rst_n), .rgb_in(sel_rgb),
        .bright_gain(gain(bl_l)), .contrast_gain(gain(cl_l)), .rgb_out(bc_rgb));

    // ---------------- 转场：淡入淡出 / 擦除揭示（第 6 拍坐标） ----------------
    wire [4:0] rp = ramp + 5'd1;
    wire [9:0] wx = {rp, 5'd0} + {2'd0, rp, 3'd0};          // rp*40
    wire [9:0] wy = {rp, 5'd0} - {4'd0, rp, 1'b0};          // rp*30
    wire [9:0] hx = {1'b0, rp, 4'd0} + {3'd0, rp, 2'd0};    // rp*20
    wire [9:0] hy = {1'b0, rp, 4'd0} - {5'd0, rp};          // rp*15
    wire [9:0]  px = x_sr[6];
    wire [9:0]  py = y_sr[6];
    reg         revealed;
    always @* begin
        case (wipe_kind)
            2'd0: revealed = (px < wx);
            2'd1: revealed = (py < wy);
            2'd2: revealed = (px >= 10'd640 - wx);
            default: revealed = (px + hx >= 10'd320) && (px < 10'd320 + hx) &&
                                (py + hy >= 10'd240) && (py < 10'd240 + hy);
        endcase
    end
    color_effect u_fx (.mode(fx_l), .rgb_in(bc_rgb), .rgb_out(fx_rgb));
    wire [15:0] mr = fx_rgb[23:16] * alpha;
    wire [15:0] mg = fx_rgb[15:8]  * alpha;
    wire [15:0] mb = fx_rgb[7:0]   * alpha;
    assign fade_px = (wipe_active && !revealed) ? 24'd0 :
                     (alpha == 8'd255) ? fx_rgb : {mr[15:8], mg[15:8], mb[15:8]};

    wire [5:0]   spec_rd, ovl_rd;
    wire [6:0]   spec_h, spec_pk;
    wire [6:0]   bass;

    // ---------------- 叠加层（第 7 拍） ----------------
    // 叠加层模式：0 FULL（状态/参数行/字幕/频谱波形/看板） 1 INFO（不显示字幕和频谱） 2 CLEAN（全部关闭，只留警告）
    wire status_en = (ovl_l != 2'd2);
    wire param_en  = (ovl_l != 2'd2);
    wire sub_en    = (ovl_l == 2'd0) && !view_l;
    wire viz_en    = (ovl_l == 2'd0) && !view_l;
    assign spec_rd=ovl_rd;
    osd_overlay_24 #(.OSD_X(8), .OSD_Y(8)) u_status (
        .clk(clk), .rst_n(rst_n), .vsync(native_800 ? ~vs_sr[7] : vs_sr[7]), .de(de_sr[7]), .rgb_in(fade_rgb),
        .pixel_x(x_sr[7]), .active_index(read_idx), .auto_enabled(auto_l),
        .error_code({5'd0, err_l}), .rgb_out(osd_rgb), .de_out(), .osd_active());
    player_text_overlay #(.PREFETCH_FONT(DYNAMIC_MODE)) u_text (
        .pixel_x_pre(x_sr[6]),.pixel_y_pre(ui_y(y_sr[6])),
        .clk(clk), .rst_n(rst_n), .vsync(native_800 ? ~vs_sr[7] : vs_sr[7]), .de(de_sr[7]),
        .rgb_in(status_en ? osd_rgb : fade_rgb), .pixel_x(x_sr[7]), .pixel_y(ui_y(y_sr[7])),
        .param_en(param_en), .sub_en(sub_en), .menu_active(1'b0), .menu_item(2'd0),
        .bright_lvl(bl_l), .contrast_lvl(cl_l), .vol_lvl(vol_l), .period_sel(prd_l),
        .auto_on(auto_l), .wav_mode(wav_l && !snd_l),
        .night_on(night_l), .hc_on(1'b0), .filt(3'd0), .uf_flag(uf_cnt != 7'd0),
        .sub_vec(sub_vec), .sub_len(sub_len),
        .tool(process_data[308:306]), .dt(process_data[47:0]), .rgb_out(txt_rgb));

    // 64 段频谱：每个 PCM 样本都参与分析，柱高按帧更新
    spectrum_dft64 u_spec (
        .clk(clk), .rst_n(rst_n), .sample_valid(pcm_tap_valid), .sample(pcm_tap_left),
        .frame_tick(frame_start), .rd_bar(spec_rd), .rd_h(spec_h), .rd_pk(spec_pk),
        .bass(bass), .overrun());
    spectrum_overlay #(.X0(64), .Y_BOT(444), .H_MAX(80)) u_spec_ovl (
        .de(de_sr[VIZ_STAGE]), .pixel_x(x_sr[VIZ_STAGE]), .pixel_x_pre(x_sr[VIZ_STAGE-1]), .pixel_y(ui_y(y_sr[VIZ_STAGE])),
        .enable(viz_en && viz_l[0]), .rgb_in(DYNAMIC_MODE?text_rgb_q:txt_rgb),
        .rd_bar(ovl_rd), .h(spec_h), .pk(spec_pk), .rgb_out(spec_rgb));
    promax_audio_waveform #(.WAVE_X(64), .WAVE_Y(372), .WAVE_WIDTH(512), .WAVE_HEIGHT(64)) u_wave (
        .clk(clk), .rst_n(rst_n), .vsync(native_800 ? ~vs_sr[VIZ_STAGE] : vs_sr[VIZ_STAGE]), .de(de_sr[VIZ_STAGE]), .rgb_in(spec_rgb),
        .pixel_x(x_sr[VIZ_STAGE]), .pixel_x_pre(x_sr[VIZ_STAGE-1]), .enable(viz_en && viz_l[1]), .pcm_tap_valid(pcm_tap_valid),
        .pcm_tap_left(pcm_tap_left), .rgb_out(wave_rgb), .waveform_active());
    // 设备声学 / 振动监测：读频谱高频段能量，超过阈值 1 秒弹无声警告（菜单 VIB LIM 4 档）
    wire [6:0]  vib_level;
    wire [15:0] vib_peak;
    wire        vib_peak_ok, vib_warn;
    wire [6:0]  vib_lim = lim_l[9:8]==2'd0 ? 7'd30 : lim_l[9:8]==2'd1 ? 7'd40 : lim_l[9:8]==2'd2 ? 7'd50 : 7'd60;
    vib_monitor #(.X0(64)) u_vib (
        .clk(clk), .rst_n(rst_n), .frame_tick(frame_start), .de(de_sr[VIZ_STAGE]),
        .x(x_sr[VIZ_STAGE]), .y(y_sr[VIZ_STAGE]), .h(spec_h), .lim(vib_lim),
        .level(vib_level), .peak_hz(vib_peak), .peak_ok(vib_peak_ok), .warn(vib_warn));
    menu_overlay #(.HDMI_MODE_MENU(DYNAMIC_MODE)) u_menu (
        .clk(clk),.pixel_x_pre(x_sr[VIZ_STAGE-1]),.pixel_y_pre(ui_y(y_sr[VIZ_STAGE-1])),
        .de(de_sr[VIZ_STAGE]), .pixel_x(x_sr[VIZ_STAGE]), .pixel_y(ui_y(y_sr[VIZ_STAGE])), .enable(menu_l),
        .item(item_l), .edit(edit_l), .bright_lvl(bl_l), .contrast_lvl(cl_l), .vol_lvl(vol_l),
        .period_sel(prd_l), .auto_on(auto_l), .viz(viz_l), .filt(night_l), .fx(fx_l),
        .overlay_mode(ovl_l), .pic_idx(read_idx), .found(found_l),
        .lim_idx(lim_l), .snd_tone(snd_l), .wav_ok(wav_l),
        .rgb_in(wave_rgb), .rgb_out(ovl_rgb));

    // 晶圆厂看板：放在最上层，警告横幅和报警边框不被菜单以外的层遮住
    board_live_overlay u_board_overlay (
        .clk(clk), .rst_n(rst_n), .frame_tick(frame_start), .de(de_sr[BOARD_STAGE]),
        .hide(view_l || !pa_lat), .menu(menu_l), .clean(ovl_l == 2'd2), .detail(detail_l),
        .x(x_sr[BOARD_STAGE]), .y(y_sr[BOARD_STAGE]), .x_pre(x_sr[BOARD_STAGE-1]), .y_pre(y_sr[BOARD_STAGE-1]),
        .profile(profile_l), .data(process_data[318:48]),
        .vib(vib_level), .peak_hz(vib_peak), .peak_ok(vib_peak_ok), .vib_lim(vib_lim), .vib_warn(vib_warn),
        .rgb_in(menu_rgb_q), .rgb_out(board_rgb), .banner_on(banner_on));

    assign hs_out = hs_sr[N];
    assign vs_out = vs_sr[N];
    assign de_out = de_sr[N];
    assign x_out = x_sr[N];
    assign y_out = y_sr[N];
endmodule
