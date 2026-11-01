`timescale 1ns/1ps

// ============================================================================
// 完赛工程顶层：TF 卡多媒体播放器（安路 EG4S20 + HDMI 1.4b 音视频）
//   第2~6课：按键三层架构 / HDMI 音频 / 稳定性 / 复位与固化 / FAT32
//   第7课  ：四 bank 帧缓存，四张图后台预载，切图瞬间完成
//   第8课  ：状态 OSD + 参数行 + 滚动字幕
//   第9课  ：淡入淡出 / 擦除揭示 / 中心展开转场
//   第10课 ：MUSIC1~4.WAV 随图切换、循环播放（无 WAV 时回退为每张图一段旋律）
//   第11课 ：音频波形与峰值条
//   第12课 ：加载时生成缩略图，2 倍双线性放大显示
//   第13课 ：亮度/对比度/音量/周期参数菜单
//   第14课 ：2x2 缩略图浏览，焦点移动并选中全屏
// 时钟域：sd_card_clk 100M（SD/控制）  ext_mem_clk 125M（SDRAM）
//         video_clk 25M（像素/音频）    hdmi_5x_clk 125M（TMDS 串行）
// ============================================================================
module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 下一张 / 菜单下移(值+1) / 焦点右；长按：自动轮播
    input                       key2,           // 打开菜单 / 进入编辑(确认) / 选中；长按：缩略图 / 关菜单
    input                       key3,           // 上一张 / 菜单上移(值-1) / 焦点左
    output [7:0]                seg_sel,
    output [7:0]                seg_data,
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,
    // SW3：手动报警（警报音 + 红色闪烁边框）  SW4：显示阈值行
    input                       sw3,
    input                       sw4,
    // 扩展6：板载 USB-UART（115200 8N1，字幕与远程控制）
    input                       uart_rx,
    output                      uart_tx,
    // 板间串口（杜邦线直连发送板：发送板 J1(TX) -> P4，N4 -> 发送板 K2(RX)，共地）
    input                       link_rx,
    output                      link_tx,
    // 摄像头视频串口（发送板 P2-37 P12 -> M1，3.125Mbps，画中画）
    input                       vid_rx,
    // 扩展：参数掉电保存（板载 SPI Flash 顶端 8KB 双槽，远离比特流区）
    output                      flash_cs_n,
    output                      flash_sclk,
    output                      flash_mosi,
    input                       flash_miso,
    output                      flash_wp_n,
    output                      flash_hold_n
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter FRAME_PIXELS  = 24'd307200;       // 640*480，每像素一个 32 位字
parameter BUF0_ADDR     = 24'd0;            // 四个 bank 依次排列，共 1228800 字（4.7MB）
parameter BUF1_ADDR     = 24'd480000;
parameter BUF2_ADDR     = 24'd960000;
parameter BUF3_ADDR     = 24'd1440000;
localparam integer KEY_CLK_HZ = 100_000_000;
localparam integer AFIFO_AW   = 12;

// 扩展用到的跨模块信号（先声明，后面各段使用）
wire        write_flip;          // sd 域：本张图是否自下而上写入
wire [8:0]  write_used;          // sd 域：写 FIFO 水位（缩放器背压）
wire        read_underflow;      // video 域：显示读 FIFO 断供（诊断）
wire        emerg_lat;           // video 域：帧锁存后的 SW3 手动报警（驱动警报音）
wire manual_switch, detail_switch;
wire [7:0] picture_ids, picture_ids_v;
wire manual_alarm;
wire process_valid, process_ctx, process_stale, process_warn, process_failing, process_demo;
wire [2:0] process_reasons, process_tool;
wire [7:0] process_age;
wire [15:0] process_sequence, process_voltage, process_current, process_temperature;
wire [15:0] process_lot, process_wafer, process_defects, process_yield, process_uptime, process_packets, process_errors;
wire [47:0] process_datetime;
wire [15:0] process_vmin, process_vmax, process_imax, process_tmax;
wire [1:0] process_link_state;
wire [7:0] process_link_q;
wire [318:0] process_data, process_data_v;
wire [1:0] selected_profile;
wire selected_valid;
wire [2:0] displayed_sd;
wire [2:0] displayed_board={1'b0,displayed_sd[1:0]}+3'd1;
wire [7:0] monitor_rx_data, usb_rx_data;
wire usb_rx_valid, link_from, link_eol;
wire [9:0] lim_idx, cfg_lim;
wire lim_load;
wire monitor_rx_valid, monitor_handled, monitor_ok, monitor_query;
wire [115:0] status_response, event_response;
wire [6:0] status_length, event_length;
wire [15:0] event_overflows;
wire event_available;

// ===================== 时钟与复位（第5课，不变） =====================
wire sd_card_clk, ext_mem_clk, ext_mem_clk_sft, video_clk, hdmi_5x_clk;
wire sys_pll_lock, video_pll_lock;
// 只用 640x480：800x600 运行时切换已删除，各模块的 mode_800 接常量 0，综合时相关逻辑整段裁掉
wire mode_800=1'b0, mode_pause=1'b0, mode_halt=1'b0;
wire media_idle,dma_idle;
wire [20:0] native_frame_pixels=21'd307200;
wire video_domain_rst_n,dma_rst_n;
wire sdcard_rst_n, media_sd_rst_n, mem_rst_n, video_rst_n, hdmi_5x_rst_n;
wire Sdr_init_done, Sdr_init_ref_vld, Sdr_busy, Sdr_init_done_sd;
wire ext_rst_n = rst_n & sys_pll_lock;
assign video_domain_rst_n=ext_rst_n & video_pll_lock;

sys_pll sys_pll_m0(
    .refclk(clk), .clk0_out(sd_card_clk), .clk1_out(ext_mem_clk), .clk2_out(ext_mem_clk_sft),
    .clk3_out(),.clk4_out(),
    .reset(~rst_n), .extlock(sys_pll_lock));
video_pll video_pll_m0(
    .refclk(clk), .clk0_out(video_clk), .clk1_out(hdmi_5x_clk),
    .reset(~rst_n), .extlock(video_pll_lock));

reset_sync u_dma_reset(.clk(ext_mem_clk),.ext_rst_n(ext_rst_n),.rst_n_sync(dma_rst_n));
reset_sync u_sdcard_reset (.clk(sd_card_clk), .ext_rst_n(ext_rst_n), .rst_n_sync(sdcard_rst_n));
// The SD controller stays alive while the four media banks are reloaded.
// Release the media parser/preloader in its own clock domain after the switch.
// SD 域复位按模块组拆成 4 个同步器：原来一个释放寄存器带 1500 个复位端，走线 12ns，恢复时间违例
wire rst_ui_n, rst_ser_n, rst_cfg_n, rst_cdc_n;
reset_sync u_rst_ui  (.clk(sd_card_clk), .ext_rst_n(ext_rst_n), .rst_n_sync(rst_ui_n));
reset_sync u_rst_ser (.clk(sd_card_clk), .ext_rst_n(ext_rst_n), .rst_n_sync(rst_ser_n));
reset_sync u_rst_cfg (.clk(sd_card_clk), .ext_rst_n(ext_rst_n), .rst_n_sync(rst_cfg_n));
reset_sync u_rst_cdc (.clk(sd_card_clk), .ext_rst_n(ext_rst_n), .rst_n_sync(rst_cdc_n));
reset_sync u_media_sd_reset(.clk(sd_card_clk),.ext_rst_n(sdcard_rst_n),.rst_n_sync(media_sd_rst_n));
reset_sync u_mem_reset    (.clk(ext_mem_clk), .ext_rst_n(ext_rst_n), .rst_n_sync(mem_rst_n));
reset_sync u_video_reset  (.clk(video_clk),   .ext_rst_n(video_domain_rst_n), .rst_n_sync(video_rst_n));
reset_sync u_hdmi_5x_reset(.clk(hdmi_5x_clk), .ext_rst_n(video_domain_rst_n), .rst_n_sync(hdmi_5x_rst_n));
sync_2ff u_sdram_init_sync (.clk(sd_card_clk), .rst_n(sdcard_rst_n),
                            .async_in(Sdr_init_done), .sync_out(Sdr_init_done_sd));

// mem_clk 域整帧写完单拍 -> 电平翻转，供 sd_card_clk 域可靠同步
wire frame_write_finish;
reg  frame_write_toggle_mem;
always @(posedge ext_mem_clk or negedge mem_rst_n)
    if (!mem_rst_n) frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish) frame_write_toggle_mem <= ~frame_write_toggle_mem;

// ===================== 按键：路由 -> 策略 -> 执行（第2课 2.9 + 第13/14课） =====================
wire prev_key_event, next_key_event, next_key_long_event, auto_key_stable_n;
dual_key_chord_router #(
    .DEBOUNCE_CYCLES(KEY_CLK_HZ / 50), .HOLD_CYCLES(KEY_CLK_HZ * 3), .KEY3_LONG_CYCLES(KEY_CLK_HZ)
) u_key_router (
    .clk(sd_card_clk), .rst_n(rst_ui_n), .save_enabled(1'b1),
    .key2_n(key3), .key3_n(key1),
    .key2_event(prev_key_event), .key3_event(next_key_event), .key3_long_event(next_key_long_event),
    .save_event(), .save_locked(), .key2_stable_n(), .key3_stable_n());
dual_key_long_hold_guard #(.DEBOUNCE_CYCLES(KEY_CLK_HZ / 50), .HOLD_CYCLES(KEY_CLK_HZ * 3)) u_key_auto_db (
    .clk(sd_card_clk), .rst_n(rst_ui_n), .enabled(1'b0), .key2_n(key2), .key3_n(1'b1),
    .trigger(), .locked(), .key2_stable_n(auto_key_stable_n), .key3_stable_n());

wire resize_busy;
wire [39:0] image_widths={4{10'd640}};
wire       key_next, key_prev, key_auto, key_select, view_mode, menu_active, vol_event, prd_event;
wire [1:0] key_select_idx, focus, period_sel, overlay_mode, disp_idx;
wire [4:0] menu_item;
wire       menu_edit, p_wr, m_save, snd_tone, cfg_snd;
wire [7:0] cfg_live_mode;
wire [2:0] p_id, p_val, fx_mode, cfg_fx;
wire [2:0] bright_lvl, contrast_lvl, found_count, err_code;
wire [3:0] vol_lvl, image_complete, image_failed;
wire       cfg_load;
wire [2:0] cfg_bright, cfg_contrast;
wire       cfg_filt;
wire [3:0] cfg_vol;
wire [1:0] cfg_period, cfg_overlay, cfg_viz;


// ===================== UART：字幕 + 远程控制（扩展6） =====================
wire        u_next, u_prev, u_auto, u_sel, u_ovl, u_save;
wire [1:0]  u_sel_idx, viz_mode;
wire        filt_mode;
assign manual_alarm=manual_switch;               // 报警声音只由 SW3 拉响
wire [511:0] sub_vec;
wire [6:0]  sub_len;
wire        sub_upd;
uart_cmd #(.CLKS_PER_BIT(KEY_CLK_HZ / 115200)) u_uart_cmd (
    .clk(sd_card_clk), .rst_n(rst_ser_n), .rx(uart_rx), .tx(uart_tx),
    .next_p(u_next), .prev_p(u_prev), .auto_p(u_auto), .sel_p(u_sel), .ovl_p(u_ovl), .save_p(u_save),
    .sel_idx(u_sel_idx), .p_wr(p_wr), .p_id(p_id), .p_val(p_val),
    .viz(viz_mode), .filt(filt_mode), .fx(fx_mode),
    .cfg_load(cfg_load), .cfg_viz(cfg_viz), .cfg_filt(cfg_filt), .cfg_fx(cfg_fx),
    .sub_vec(sub_vec), .sub_len(sub_len), .sub_upd(sub_upd),
    .monitor_rx_data(usb_rx_data), .monitor_rx_valid(usb_rx_valid),
    .monitor_handled(monitor_handled && !link_from), .monitor_ok(monitor_ok), .monitor_query(monitor_query),
    .monitor_response(event_available ? event_response : status_response), .monitor_event(event_available),
    .monitor_response_len(event_available ? event_length : status_length));
// 板间串口：和 USB 串口按行合并送 board_monitor；板间这一路由 link_uart 回 OK/ER
link_uart #(.CLKS_PER_BIT(KEY_CLK_HZ / 115200), .ENABLE_GIMBAL(1), .GIMBAL_PERIOD(KEY_CLK_HZ/2)) u_link_uart (
    .clk(sd_card_clk), .rst_n(rst_ser_n), .rx(link_rx), .tx(link_tx),
    .usb_data(usb_rx_data), .usb_valid(usb_rx_valid),
    .mon_data(monitor_rx_data), .mon_valid(monitor_rx_valid),
    .from_link(link_from), .link_eol(link_eol),
    .mon_handled(monitor_handled), .mon_ok(monitor_ok),
    .gimbal_tool(process_demo ? 3'd0 : process_tool),
    .gimbal_alarm(!process_demo && process_valid && process_warn),
    .gimbal_reason(process_demo ? 3'd0 : process_reasons), .reply_overflow());

// SW3 手动报警期间冻结切图；参数警告只弹提示，不影响操作
wire       next_cmd=(key_next | u_next) & ~manual_alarm;
wire       prev_cmd=(key_prev | u_prev) & ~manual_alarm;
wire       auto_cmd   = key_auto | u_auto;
wire       select_cmd=(key_select | u_sel) & ~manual_alarm;
wire [1:0] select_idx = u_sel ? u_sel_idx : key_select_idx;

media_policy #(.CLK_HZ(KEY_CLK_HZ)) u_media_policy (
    .clk(sd_card_clk), .rst_n(rst_ui_n),
    .prev_key_event(prev_key_event), .next_key_event(next_key_event),
    .next_key_long_event(next_key_long_event),
    .auto_key_stable_n(auto_key_stable_n), .found_count(found_count), .disp_idx(disp_idx),
    .cfg_load(cfg_load), .cfg_bright(cfg_bright), .cfg_contrast(cfg_contrast), .cfg_vol(cfg_vol),
    .cfg_period(cfg_period), .cfg_overlay(cfg_overlay), .ovl_cycle(u_ovl), .cfg_snd(cfg_snd), .snd_tone(snd_tone),
    .cur_viz(viz_mode), .cur_filt(filt_mode), .cur_fx(fx_mode), .cfg_lim(cfg_lim),
    .p_wr(p_wr), .p_id(p_id), .p_val(p_val), .save_req(m_save),
    .next_cmd(key_next), .prev_cmd(key_prev), .auto_cmd(key_auto),
    .select_cmd(key_select), .select_idx(key_select_idx), .view_mode(view_mode), .focus(focus),
    .menu_active(menu_active), .menu_item(menu_item), .menu_edit(menu_edit), .lim_idx(lim_idx), .lim_load(lim_load),
    .bright_lvl(bright_lvl), .contrast_lvl(contrast_lvl), .vol_lvl(vol_lvl), .period_sel(period_sel),
    .overlay_mode(overlay_mode), .vol_event(vol_event), .prd_event(prd_event));

// ===================== 参数掉电保存（扩展，参考米醋工程 player_config_persistence） =====================
// 记录格式沿用参考模块的 period_ms / volume / mode 三个字段，把本工程参数打包进去：
//   volume[7:0]  = {bright[2:0], contrast[2:0], period[1:0]}
//   mode[0]      = 声音来源（0 音乐 1 旋律音）
//   period_ms    = 1000 + {阈值档位[9:0], fx[2:0], overlay[1:0], 夜间, viz[1:0], vol[3:0]}（MAX_PERIOD_MS 相应放宽）
// 保存策略：参数变化后稳定 3 秒再写一次（或 UART !W 立即写），避免每次按键都擦 Flash；
// 槽地址 0x7FE000 / 0x7FF000（8MB 顶端两个 4KB 扇区），比特流约 0.7MB，从 0 地址开始，不重叠
wire [30:0] cfg_snap = {snd_tone, lim_idx, fx_mode, overlay_mode, filt_mode, viz_mode, vol_lvl, bright_lvl, contrast_lvl, period_sel};
wire        cfg_ready, cfg_busy, cfg_apply, cfg_stored_valid;
wire [31:0] cfg_live_period;
wire [7:0]  cfg_live_volume;
reg  [30:0] cfg_saved, cfg_last;
reg  [31:0] cfg_stable_cnt;
reg         cfg_armed, cfg_booted, cfg_upd, cfg_save, cfg_load_r, cfg_force;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        cfg_saved <= 31'd0;  cfg_last <= 31'd0;  cfg_stable_cnt <= 32'd0;
        cfg_armed <= 1'b0;  cfg_booted <= 1'b0;  cfg_force <= 1'b0;
        cfg_upd <= 1'b0;  cfg_save <= 1'b0;  cfg_load_r <= 1'b0;
    end else begin
        cfg_upd    <= 1'b0;
        cfg_save   <= cfg_upd;                                   // 先更新快照，下一拍保存
        cfg_load_r <= cfg_apply && !cfg_booted && cfg_stored_valid;
        if (cfg_apply && !cfg_booted) cfg_booted <= 1'b1;        // 只有上电第一次 apply 是“读回”
        cfg_last <= cfg_snap;
        if (u_save || m_save) cfg_force <= 1'b1;
        if (cfg_snap != cfg_last)
            cfg_stable_cnt <= 32'd0;                             // 参数还在变：重新计时
        else if (cfg_stable_cnt < KEY_CLK_HZ * 3)
            cfg_stable_cnt <= cfg_stable_cnt + 32'd1;
        if (!cfg_armed) begin
            // 读回参数生效并稳定 1 秒后才开始监视变化，避免上电就写一次
            if (cfg_booted && cfg_stable_cnt >= KEY_CLK_HZ) begin cfg_armed <= 1'b1;  cfg_saved <= cfg_snap; end
        end else if (((cfg_snap != cfg_saved && cfg_stable_cnt >= KEY_CLK_HZ * 3) || cfg_force) &&
                     cfg_ready && !cfg_busy && !cfg_upd && !cfg_save) begin
            cfg_upd <= 1'b1;  cfg_saved <= cfg_snap;  cfg_force <= 1'b0;
        end
    end
end

wire [21:0] cfg_pack_lo = {lim_idx, fx_mode, overlay_mode, filt_mode, viz_mode, vol_lvl};
wire [31:0] cfg_unp32   = cfg_live_period - 32'd1000;
assign cfg_load     = cfg_load_r;
assign cfg_bright   = cfg_live_volume[7:5];
assign cfg_contrast = cfg_live_volume[4:2];
assign cfg_period   = cfg_live_volume[1:0];
assign cfg_vol      = cfg_unp32[3:0];
assign cfg_viz      = cfg_unp32[5:4];
assign cfg_filt     = cfg_unp32[6];
assign cfg_lim      = cfg_unp32[21:12];
assign cfg_overlay  = cfg_unp32[8:7];
assign cfg_fx       = cfg_unp32[11:9];
assign cfg_snd      = cfg_live_mode[0];

player_config_persistence #(
    .SPI_CLK_DIV(4), .CLKS_PER_MS(32'd100000),
    .ERASE_TIMEOUT_CYCLES(32'd300000000), .PROGRAM_TIMEOUT_CYCLES(32'd10000000),
    .MAX_PERIOD_MS(32'd1000 + 32'd4194303),
    .DEFAULT_PERIOD_MS(32'd1000 + {10'd0, 10'h1d6, 12'd0} + 32'd8 + 32'd48), .DEFAULT_VOLUME({3'd3, 3'd3, 2'd1}), .DEFAULT_MODE(8'd0)
) u_cfg_store (
    .clk(sd_card_clk), .rst_n(rst_cfg_n),
    .cfg_update_valid(cfg_upd), .cfg_period_ms(32'd1000 + {10'd0, cfg_pack_lo}),
    .cfg_volume({bright_lvl, contrast_lvl, period_sel}), .cfg_mode({7'd0, snd_tone}),
    .save_request(cfg_save), .service_ready(cfg_ready), .busy(cfg_busy),
    .load_done(), .save_done(), .apply_valid(cfg_apply), .config_rejected(), .fatal_error(),
    .error_code(), .request_dropped(), .stored_config_valid(cfg_stored_valid), .active_version(),
    .live_period_ms(cfg_live_period), .live_period_cycles(), .live_volume(cfg_live_volume), .live_mode(cfg_live_mode),
    .jedec_id(), .status1(), .status2(), .status3(),
    .flash_miso(flash_miso), .flash_cs_n(flash_cs_n), .flash_sclk(flash_sclk), .flash_mosi(flash_mosi),
    .flash_wp_n(flash_wp_n), .flash_hold_n(flash_hold_n));

// ===================== TF 卡媒体子系统（第4/6/7/10/12课） =====================
wire        display_valid, pic_changed, auto_play_on, error_active;
wire        switch_req, switch_auto, fade_go, fade_busy;
wire        sd_card_write_req, sd_card_write_req_ack, sd_card_write_en;
wire [31:0] sd_card_write_data;
wire [1:0]  write_buf_idx;
wire        thumb_we_even, thumb_we_odd;
wire [13:0] thumb_waddr;
wire [15:0] thumb_wdata;
wire        pcm_wr_en, wav_mode, wav_stereo;
wire        audio_clear, audio_clear_ack;
wire [7:0]  pcm_wr_data;
wire [AFIFO_AW:0] pcm_fifo_used;

sd_card_bmp #(.DYNAMIC_MODE(1),.CLK_FREQ_HZ(KEY_CLK_HZ), .AUDIO_FIFO_AW(AFIFO_AW)) sd_card_bmp_m0 (
    .clk(sd_card_clk), .rst(~media_sd_rst_n), .sdram_init_done(Sdr_init_done_sd),
    .mode_800(mode_800),.mode_pause(mode_pause),.media_idle(media_idle),
    .resize_apply(1'b0), .image_widths(image_widths), .resize_busy(resize_busy),
    .next_cmd(next_cmd), .prev_cmd(prev_cmd), .auto_cmd(auto_cmd),
    .select_cmd(select_cmd), .select_idx(select_idx), .period_sel(period_sel),
    .auto_pause(view_mode | menu_active | manual_alarm),
    .fade_go(fade_go), .fade_busy(fade_busy), .switch_req(switch_req), .switch_auto(switch_auto),
    .disp_idx(disp_idx), .display_valid(display_valid), .pic_changed(pic_changed),
    .auto_play_on(auto_play_on), .image_complete(image_complete), .image_failed(image_failed),
    .found_count(found_count), .picture_ids(picture_ids), .err_code(err_code), .error_active(error_active),
    .write_used(write_used), .write_flip(write_flip),
    .write_finish_toggle(frame_write_toggle_mem), .write_buf_idx(write_buf_idx),
    .write_req(sd_card_write_req), .write_req_ack(sd_card_write_req_ack),
    .write_en(sd_card_write_en), .write_data(sd_card_write_data),
    .thumb_we_even(thumb_we_even), .thumb_we_odd(thumb_we_odd),
    .thumb_waddr(thumb_waddr), .thumb_wdata(thumb_wdata),
    .pcm_wr_en(pcm_wr_en), .pcm_wr_data(pcm_wr_data), .pcm_fifo_used(pcm_fifo_used),
    .wav_mode(wav_mode), .wav_stereo(wav_stereo),
    .audio_clear(audio_clear), .audio_clear_ack(audio_clear_ack),
    .SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso));

// ===================== 数码管：PIC n / VOL ddd / PRD ddS / E+错误码 =====================
wire [7:0] vol_pct = {vol_lvl, 3'd0} + {3'd0, vol_lvl, 1'b0};          // 档位 * 10
wire [5:0] prd_sec = (period_sel == 2'd0) ? 6'd2 : (period_sel == 2'd1) ? 6'd3 :
                     (period_sel == 2'd2) ? 6'd5 : 6'd10;
seg7_panel #(.CLK_HZ(KEY_CLK_HZ), .SCAN_STEP_CYCLES(8192),
             .PAGE_PERIOD_CYCLES(KEY_CLK_HZ), .HOLD_CYCLES(KEY_CLK_HZ * 2)) u_seg7_panel (
    .clk(sd_card_clk), .rst_n(rst_ui_n), .enable(1'b1), .media_id(disp_idx),
    .volume(vol_pct), .vol_valid(1'b1), .period_sec(prd_sec),
    .pic_event(pic_changed), .vol_event(vol_event), .prd_event(prd_event | auto_cmd),
    .err_code(error_active ? err_code : 3'd0), .seg_n(seg_data), .dig_n(seg_sel));

// ===================== 控制量 sd_card_clk -> video_clk（toggle 握手，整组同步） =====================
wire        display_valid_v, view_v, menu_v, auto_v, wav_mode_v, wav_stereo_v;
wire [1:0]  disp_idx_v, focus_v, prd_v, ovl_v;
wire [4:0]  item_v;
wire detail_v;
wire        edit_v;
wire [2:0]  fx_v;
wire        snd_v;
wire [3:0]  comp_v, fail_v, vol_v;
wire [2:0]  found_v, bl_v, cl_v, err_v;
wire        filt_v;
wire [1:0]  viz_v;
wire [9:0]  lim_v;
cdc_bus_sync #(.W(70)) u_ctrl_cdc (
    .src_clk(sd_card_clk), .src_rst_n(rst_cdc_n),
    .src_data({disp_idx, display_valid, view_mode, focus, image_complete, image_failed, found_count,
               bright_lvl, contrast_lvl, vol_lvl, period_sel, overlay_mode, menu_active, menu_item,
               auto_play_on, wav_mode, wav_stereo, (error_active ? err_code : 3'd0),
               viz_mode, filt_mode, lim_idx, menu_edit, fx_mode, snd_tone, detail_switch, picture_ids}),
    .dst_clk(video_clk), .dst_rst_n(video_rst_n),
    .dst_data({disp_idx_v, display_valid_v, view_v, focus_v, comp_v, fail_v, found_v,
               bl_v, cl_v, vol_v, prd_v, ovl_v, menu_v, item_v, auto_v, wav_mode_v, wav_stereo_v, err_v,
               viz_v, filt_v, lim_v, edit_v, fx_v, snd_v, detail_v, picture_ids_v}));

// 字幕整行（64 字符 + 长度）跨域：uart_cmd 只在收完一行时更新 sub_vec 并给 sub_upd 单拍，
// 之后至少一整行串口时间（>100us）保持不变。源域把 sub_upd 变成翻转位，目的域三级同步检测到翻转
// 再整组采样（此时数据已稳定上千拍），不需要 519 位比较器和保持寄存器；像素域在帧边界再锁存一次
reg          sub_tgl;
always @(posedge sd_card_clk or negedge sdcard_rst_n)
    if (!sdcard_rst_n) sub_tgl <= 1'b0;
    else if (sub_upd)  sub_tgl <= ~sub_tgl;
reg  [2:0]   sub_s;
reg  [511:0] sub_vec_v;
reg  [6:0]   sub_len_v;
always @(posedge video_clk or negedge video_rst_n)
    if (!video_rst_n) begin
        sub_s <= 3'd0;  sub_vec_v <= 512'd0;  sub_len_v <= 7'd0;
    end else begin
        sub_s <= {sub_s[1:0], sub_tgl};
        if (sub_s[2] ^ sub_s[1]) begin sub_vec_v <= sub_vec;  sub_len_v <= sub_len; end
    end

// SW3/SW4：两级同步 + 20ms 稳定计数（sd_card_clk 域）。板上拨码按“拨到 ON = 低电平”处理（上拉），
// 若实板相反，把 SW_ACTIVE_LOW 改成 0
localparam SW_ACTIVE_LOW = 1;
reg  [1:0]  sw_s0, sw_s1, sw_stable;
reg  [20:0] sw_cnt;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        sw_s0 <= 2'b00;  sw_s1 <= 2'b00;  sw_stable <= 2'b00;  sw_cnt <= 21'd0;
    end else begin
        sw_s0 <= SW_ACTIVE_LOW ? ~{sw4, sw3} : {sw4, sw3};
        sw_s1 <= sw_s0;
        if (sw_s1 == sw_stable)          sw_cnt <= 21'd0;
        else if (sw_cnt >= 21'd2000000) begin sw_stable <= sw_s1;  sw_cnt <= 21'd0; end
        else                             sw_cnt <= sw_cnt + 21'd1;
    end
end
assign manual_switch=sw_stable[0];
assign detail_switch=sw_stable[1];
cdc_bus_sync #(.W(3)) u_displayed_back (
    .src_clk(video_clk), .src_rst_n(video_rst_n), .src_data({selected_valid,selected_profile}),
    .dst_clk(sd_card_clk), .dst_rst_n(rst_cdc_n), .dst_data(displayed_sd));
// 上电 DEMO：内部动态演示数据；收到第一个合法 !M / !L 包自动切到 LIVE
board_monitor #(.LOCAL_DEMO(1), .CLK_HZ(KEY_CLK_HZ)) u_monitor (
    .clk(sd_card_clk), .rst_n(rst_ser_n), .rx_data(monitor_rx_data), .rx_valid(monitor_rx_valid),
    .manual_alarm(manual_alarm), .picture_valid(displayed_sd[2]),
    .detail_mode(detail_switch), .event_overflows(event_overflows),
    .link_eol(link_eol), .lim_idx(lim_idx[7:0]), .lim_load(lim_load),
    .handled(monitor_handled), .command_ok(monitor_ok), .query(monitor_query),
    .response(status_response), .response_len(status_length),
    .valid(process_valid), .ctx_valid(process_ctx), .stale(process_stale), .warn(process_warn),
    .reasons(process_reasons), .failing(process_failing), .demo(process_demo), .tool_id(process_tool), .age(process_age),
    .sequence_id(process_sequence), .millivolts(process_voltage), .milliamps(process_current), .temperature(process_temperature),
    .lot(process_lot), .wafer(process_wafer), .defects(process_defects), .yield(process_yield),
    .uptime_min(process_uptime), .packets(process_packets), .errors(process_errors), .datetime(process_datetime),
    .vmin(process_vmin), .vmax(process_vmax), .imax(process_imax), .tmax(process_tmax),
    .link_state(process_link_state), .link_q(process_link_q));
reg manual_previous, detail_previous, menu_previous, edit_previous;
reg [4:0] item_previous;
always @(posedge sd_card_clk or negedge sdcard_rst_n)
    if(!sdcard_rst_n) begin manual_previous<=0;detail_previous<=0;menu_previous<=0;edit_previous<=0;item_previous<=0;end
    else begin manual_previous<=manual_switch;detail_previous<=detail_switch;menu_previous<=menu_active;edit_previous<=menu_edit;item_previous<=menu_item;end
wire [7:0] operation_events={menu_previous^menu_active,detail_previous^detail_switch,
    manual_previous^manual_switch,(p_wr | (edit_previous^menu_edit) | (item_previous!=menu_item)),
    key_select,key_auto,prev_key_event,next_key_event};
board_event_queue u_operations (
    .clk(sd_card_clk), .rst_n(rst_ser_n), .events(operation_events), .board(displayed_board),
    .manual(manual_switch), .detail(detail_switch), .menu(menu_active), .auto_on(auto_play_on),
    .extra({5'd0,menu_item,p_id,p_val}),
    .pop(monitor_handled && monitor_ok && monitor_query && event_available && !link_from),
    .available(event_available), .response(event_response), .response_len(event_length), .overflow_count(event_overflows));
// 看板数据 sd -> video：{manual, 状态 7, 原因 3, tool 3, age 8, 链路 2+8, 数值 16x15, 日期时间 48} = 319 位
assign process_data={manual_alarm,process_valid,process_stale,process_warn,process_failing,process_demo,process_ctx,
    process_reasons,process_tool,process_age,process_link_state,process_link_q,process_voltage,process_current,process_temperature,process_sequence,
    process_lot,process_wafer,process_defects,process_yield,process_uptime,process_packets,process_errors,
    process_vmin,process_vmax,process_imax,process_tmax,process_datetime};
cdc_bus_sync #(.W(319), .FREE_RUN(1)) u_process_cdc (
    .src_clk(sd_card_clk), .src_rst_n(rst_cdc_n), .src_data(process_data),
    .dst_clk(video_clk), .dst_rst_n(video_rst_n), .dst_data(process_data_v));

// ===================== 视频时序与四 bank 帧缓存（第7课） =====================
wire hs_0, vs_0, de_0, hs, vs, de;
wire video_read_req, video_read_req_ack, video_read_en;
wire [31:0] video_read_data;
wire [23:0] vout_data_raw;
wire [1:0]  read_idx;
wire App_rd_en, Sdr_rd_en, App_wr_en;
wire [ADDR_BITS-1:0] App_rd_addr, App_wr_addr;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout, App_wr_din;
wire [3:0] App_wr_dm;

video_timing_data #(.DYNAMIC_MODE(1)) video_timing_data_m0(
    .mode_800(mode_800),
    .video_clk(video_clk), .rst(~video_rst_n), .read_req(video_read_req),
    .read_req_ack(video_read_req_ack), .hs(hs_0), .vs(vs_0), .de(de_0));
video_delay video_delay_m0(
    .video_clk(video_clk), .rst(~video_rst_n), .read_en(video_read_en),
    .read_data(video_read_data[31:8]), .hs(hs_0), .vs(vs_0), .de(de_0),
    .hs_r(hs), .vs_r(vs), .de_r(de), .vout_data(vout_data_raw));

// 读：显示 bank = read_idx（像素流水线在场消隐开始时锁存）；写：预载 bank = write_buf_idx
frame_read_write #(.DYNAMIC_MODE(1),.WRITE_V_FLIP(1), .FRAME_WIDTH(640), .FRAME_HEIGHT(480)) frame_read_write_m0(
    .mode_800(mode_800),.read_halt(mode_halt),.dma_idle(dma_idle),
    .mem_clk(ext_mem_clk), .rst(~dma_rst_n),
    .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
    .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr), .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
    .read_clk(video_clk), .read_req(video_read_req), .read_req_ack(video_read_req_ack), .read_finish(),
    .read_addr_0(BUF0_ADDR), .read_addr_1(BUF1_ADDR), .read_addr_2(BUF2_ADDR), .read_addr_3(BUF3_ADDR),
    .read_addr_index(read_idx), .read_len(native_frame_pixels),
    .read_en(video_read_en), .read_data(video_read_data),
    .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
    .write_clk(sd_card_clk), .write_req(sd_card_write_req), .write_req_ack(sd_card_write_req_ack),
    .write_finish(frame_write_finish),
    .write_addr_0(BUF0_ADDR), .write_addr_1(BUF1_ADDR), .write_addr_2(BUF2_ADDR), .write_addr_3(BUF3_ADDR),
    .write_addr_index(write_buf_idx), .write_len(native_frame_pixels),
    .write_en(sd_card_write_en), .write_data(sd_card_write_data),
    .write_flip(write_flip), .write_used(write_used), .read_underflow(read_underflow));

sdram U3(
    .Clk(ext_mem_clk), .Clk_sft(ext_mem_clk_sft), .Rst(~mem_rst_n),
    .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
    .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_dm(App_wr_dm), .App_wr_din(App_wr_din),
    .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr), .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout));

// ===================== 像素流水线 + 转场（第8/9/11/12/13/14课） =====================
wire        hs_p, vs_p, de_p, pa_lat, frame_start_p, index_commit;
wire [23:0] rgb_p;
wire        hs_m, vs_m, de_m;              // 像素流水线输出，经摄像头画中画后为 hs_p/vs_p/de_p/rgb_p
wire [23:0] rgb_m;
wire        banner_v;
wire [9:0]  x_m, y_m;
wire [7:0]  fade_alpha;
wire        wipe_active;
wire [1:0]  wipe_kind;
wire [4:0]  fade_ramp;
wire        pcm_tap_valid;
wire signed [15:0] pcm_tap_left, spec_tap_left, wav_raw_left;

media_pixel_pipeline #(.DYNAMIC_MODE(1)) u_pipeline (
    .clk(video_clk), .rst_n(video_rst_n),
    .hs_in(hs), .vs_in(vs), .de_in(de), .rgb_in(vout_data_raw),
    .disp_idx(disp_idx_v), .player_active(display_valid_v), .view_mode(view_v), .focus(focus_v),
    .complete(comp_v), .failed(fail_v), .found(found_v),
    .bright_lvl(bl_v), .contrast_lvl(cl_v), .vol_lvl(vol_v), .period_sel(prd_v),
    .mode_800(mode_800),
    .overlay_mode(ovl_v), .menu_item(item_v), .menu_active(menu_v), .menu_edit(edit_v), .auto_on(auto_v),
    .wav_mode(wav_mode_v), .err_code(err_v),
    .viz(viz_v), .filt(filt_v), .fx(fx_v), .snd_tone(snd_v), .lim_idx(lim_v),
    .detail_mode(detail_v),
    .picture_ids(picture_ids_v), .process_data(process_data_v),
    .selected_profile(selected_profile), .selected_valid(selected_valid), .uf_pulse(read_underflow),
    .sub_vec(sub_vec_v), .sub_len(sub_len_v),
    .pcm_tap_valid(pcm_tap_valid), .pcm_tap_left(spec_tap_left),
    .wclk(sd_card_clk), .we_even(thumb_we_even), .we_odd(thumb_we_odd),
    .waddr(thumb_waddr), .wdata(thumb_wdata),
    .alpha(fade_alpha), .wipe_active(wipe_active), .wipe_kind(wipe_kind), .ramp(fade_ramp),
    .frame_start(frame_start_p), .index_commit(index_commit), .pa_lat(pa_lat), .read_idx(read_idx),
    .emerg_lat(emerg_lat), .banner_on(banner_v),
    .hs_out(hs_m), .vs_out(vs_m), .de_out(de_m), .x_out(x_m), .y_out(y_m), .rgb_out(rgb_m));

// 产线监控页（PIC4，profile 3）的现场画面：彩色 160x120 放大到 320x240，嵌在背景图的方框里；
// 菜单 / 缩略图 / 其他页不显示，警告横幅时让出第 13~14 行，0.5 秒无数据露出背景图的“等待现场画面”
cam_pip #(.CPB(KEY_CLK_HZ / 3125000), .X0(304), .Y0(88), .MARK_STATIONS(1)) u_cam_pip (
    .sclk(sd_card_clk), .srst_n(sdcard_rst_n), .vid_rx(vid_rx),
    .vclk(video_clk), .vrst_n(video_rst_n),
    .en(selected_valid && selected_profile == 2'd3 && !menu_v && !view_v), .ban(banner_v),
    .review_alert(process_data_v[317] && process_data_v[315] && !process_data_v[313]),
    .review_tool(process_data_v[308:306]),
    .hs_in(hs_m), .vs_in(vs_m), .de_in(de_m), .x_in(x_m), .y_in(y_m), .rgb_in(rgb_m),
    .hs_out(hs_p), .vs_out(vs_p), .de_out(de_p), .rgb_out(rgb_p));

video_fade_transition #(.FADE_FRAMES(16), .HOLD_TIMEOUT(30)) u_fade (
    .ctrl_clk(sd_card_clk), .ctrl_rst_n(rst_cdc_n),
    .switch_req(switch_req), .switch_auto(switch_auto), .fade_go(fade_go), .fade_busy(fade_busy),
    .pix_clk(video_clk), .pix_rst_n(video_rst_n), .player_active(pa_lat),
    .frame_start(frame_start_p), .index_commit(index_commit),
    .alpha(fade_alpha), .wipe_active(wipe_active), .wipe_kind(wipe_kind), .ramp(fade_ramp));

// ===================== WAV 音频：异步 FIFO -> 48kHz PCM 源（第10课） =====================
wire        afifo_rd_en, afifo_empty;
wire [7:0]  afifo_rd_data;
wire        wav_valid, wav_ready;
wire signed [15:0] wav_left, wav_right;
wire [7:0]  vol_gain = {vol_v, 4'd0} + {1'b0, vol_v, 3'd0} + {4'd0, vol_v};   // 档位 * 25

// 双端同时清理，异步置复位、各域同步释放；调度器等待完整的请求/应答往返。
wire audio_wr_rst_n, audio_rd_rst_n;
audio_stream_reset u_audio_reset (
    .wr_clk(sd_card_clk), .wr_rst_n(media_sd_rst_n),
    .rd_clk(video_clk), .rd_rst_n(video_rst_n),
    .clear_req(audio_clear), .clear_ack(audio_clear_ack),
    .fifo_wr_rst_n(audio_wr_rst_n), .fifo_rd_rst_n(audio_rd_rst_n));

async_byte_fifo #(.AW(AFIFO_AW)) u_audio_fifo (
    .wr_clk(sd_card_clk), .wr_rst_n(audio_wr_rst_n), .wr_en(pcm_wr_en & audio_wr_rst_n), .wr_data(pcm_wr_data),
    .wr_used(pcm_fifo_used),
    .rd_clk(video_clk), .rd_rst_n(audio_rd_rst_n), .rd_en(afifo_rd_en & audio_rd_rst_n),
    .rd_data(afifo_rd_data), .rd_empty(afifo_empty));

wav_pcm_source #(.DYNAMIC_MODE(1)) u_wav_src (
    .mode_800(mode_800),
    .clk(video_clk), .rst_n(audio_rd_rst_n), .stereo(wav_stereo_v), .gain(vol_gain),
    .fifo_rd_en(afifo_rd_en), .fifo_rd_data(afifo_rd_data), .fifo_empty(afifo_empty | ~wav_mode_v),
    .sample_valid(wav_valid), .sample_ready(wav_ready),
    .sample_left(wav_left), .sample_right(wav_right), .raw_left(wav_raw_left), .underflow());

// SW3 手动报警：警报音替换外部音源；WAV 原地暂停（FIFO 背压让 SD 读取也暂停），拨下后接着放
wire        siren_valid, siren_ready;
wire signed [15:0] siren_left, siren_right;
siren_tone #(.DYNAMIC_MODE(1)) u_siren (
    .mode_800(mode_800),
    .clk(video_clk), .rst_n(video_rst_n), .enable(emerg_lat), .volume(8'd200),
    .sample_valid(siren_valid), .sample_ready(siren_ready),
    .sample_left(siren_left), .sample_right(siren_right));
wire        ext_ready;
// 菜单 SOUND=TONE：WAV 原地暂停（ready 拉低，FIFO 背压让 SD 读取也停），HDMI 用内部旋律音；切回 MUSIC 接着放
wire        wav_play  = wav_mode_v & ~snd_v & audio_rd_rst_n;
wire        ext_sel   = wav_play | emerg_lat;
// 频谱/振动监测：播 WAV（设备录音）时取音量之前的原始采样，音量调到 0 也照常监测
assign spec_tap_left = (wav_play && !emerg_lat) ? wav_raw_left : pcm_tap_left;
wire        ext_valid = emerg_lat ? siren_valid : wav_valid;
wire signed [15:0] ext_left  = emerg_lat ? siren_left : wav_left;
wire signed [15:0] ext_right = emerg_lat ? siren_right : wav_right;
assign      siren_ready = emerg_lat ? ext_ready : 1'b1;
assign      wav_ready   = (emerg_lat | ~wav_play) ? 1'b0 : ext_ready;

// ===================== HDMI 音视频复用（第3课，输入改为流水线输出） =====================
// 行内像素坐标与"下一行是否有视频"判断，供 data island 定位（原逻辑不变）
wire [10:0] H_TOTAL_PIX=mode_800?11'd1056:11'd800;
wire [10:0] VIDEO_PREAMBLE_BEGIN=mode_800?11'd1046:11'd790;
reg         de_d0, vs_d0;
reg  [10:0]  video_x_cnt;
wire [10:0] video_x_raw=(de_p && !de_d0)?11'd0:video_x_cnt;
wire        line_start  = (video_x_raw == 10'd0);
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        de_d0 <= 1'b0;  vs_d0 <= 1'b1;  video_x_cnt <= 10'd0;
    end else begin
        de_d0 <= de_p;  vs_d0 <= vs_p;
        video_x_cnt <= (video_x_raw == H_TOTAL_PIX - 1'b1) ? 10'd0 : video_x_raw + 1'b1;
    end
end

reg  [9:0] line_cnt, cur_first_line, cur_last_line, act_first_line, act_last_line;
reg        cur_seen;
wire       next_line_active = (line_cnt + 1'b1 >= act_first_line) && (line_cnt + 1'b1 <= act_last_line);
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        line_cnt <= 10'd0;  cur_first_line <= 10'd0;  cur_last_line <= 10'd0;  cur_seen <= 1'b0;
        act_first_line <= 10'd36;  act_last_line <= 10'd515;
    end else begin
        if (vs_d0 && !vs_p) begin
            line_cnt <= 10'd0;  cur_seen <= 1'b0;
            if (cur_seen && line_cnt >= 10'd500) begin
                act_first_line <= cur_first_line;  act_last_line <= cur_last_line;
            end
        end else if (line_start)
            line_cnt <= line_cnt + 1'b1;
        if (de_p && !de_d0) begin
            if (!cur_seen) cur_first_line <= line_cnt + 1'b1;
            cur_seen <= 1'b1;  cur_last_line <= line_cnt + 1'b1;
        end
    end
end
wire [10:0] video_x=(!next_line_active && video_x_raw>=VIDEO_PREAMBLE_BEGIN)?VIDEO_PREAMBLE_BEGIN-1'b1:video_x_raw;

wire [9:0] hdmi_ch0_symbol, hdmi_ch1_symbol, hdmi_ch2_symbol;
media_hdmi_audio_bridge #(.DYNAMIC_MODE(1)) u_media_hdmi_audio_bridge (
    .mode_800(mode_800),
    .pixel_clk(video_clk), .pixel_rst_n(video_rst_n),
    .video_hsync(hs_p), .video_vsync(vs_p), .video_de(de_p), .video_x(video_x), .video_rgb(rgb_p),
    .media_id(read_idx), .volume(vol_gain),
    .channel0_symbol(hdmi_ch0_symbol), .channel1_symbol(hdmi_ch1_symbol), .channel2_symbol(hdmi_ch2_symbol),
    .sequence_error(), .tone_overflow(), .pcm_contract_error(), .blip_active(),
    .pcm_tap_valid(pcm_tap_valid), .pcm_tap_left(pcm_tap_left),
    .ext_select(ext_sel), .ext_valid(ext_valid), .ext_ready(ext_ready),
    .ext_left(ext_left), .ext_right(ext_right));

assign HDMI_DDC_SCL = 1'bz;
assign HDMI_DDC_SDA = 1'bz;

hdmi_phy_wrapper #(.DEVICE("EG")) u_hdmi2phy_wrapper(
    .I_pixel_clk(video_clk), .I_serial_clk(hdmi_5x_clk), .I_rst(~(video_rst_n & hdmi_5x_rst_n)),
    .I_tmds_channel_0(hdmi_ch0_symbol), .I_tmds_channel_1(hdmi_ch1_symbol),
    .I_tmds_channel_2(hdmi_ch2_symbol), .I_tmds_channel_clk(10'b0000011111),
    .O_tmds_ch0_p(HDMI_D0_P), .O_tmds_ch1_p(HDMI_D1_P), .O_tmds_ch2_p(HDMI_D2_P), .O_tmds_clk_p(HDMI_CLK_P));

endmodule
