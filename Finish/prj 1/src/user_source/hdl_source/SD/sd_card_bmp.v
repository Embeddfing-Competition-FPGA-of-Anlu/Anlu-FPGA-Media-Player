`timescale 1ns/1ps

// TF 卡媒体子系统（完赛整合版），sd_card_clk 100MHz 域
//   bank_preload_ctrl：扫描 PIC1~4.BMP，逐张后台预载到 SDRAM bank 0~3（第7课），重试/跳过（第4课）
//   bank_nav_ctrl    ：切图 = 换显示 bank，经转场门控（第9课），缩略图直接选中（第14课）
//   fat32_bmp_reader ：FAT32 扫描/文件流/BMP 头校验（第6课），预载完成后流式读对应 MUSIC1~4.WAV（第10课）
//   thumb_capture    ：加载像素流时同步生成 80x60 缩略图（第12课）
//   wav_stream_parser/audio_play_ctrl：WAV 头解析、循环播放、FIFO 背压（第10课）
module sd_card_bmp #(
    parameter integer DYNAMIC_MODE=0,
    parameter integer CLK_FREQ_HZ        = 100_000_000,
    parameter integer SD_READ_TIMEOUT_MS = 500,
    parameter [1:0]   MAX_RETRIES        = 2'd1,
    parameter integer AUDIO_FIFO_AW      = 12
)(
    input              clk,
    input              rst,
    input wire mode_800,mode_pause,
    output wire media_idle,
    input              sdram_init_done,
    input wire resize_apply,
    input wire [39:0] image_widths,
    output wire resize_busy,
    // 导航
    input              next_cmd,
    input              prev_cmd,
    input              auto_cmd,
    input              select_cmd,
    input       [1:0]  select_idx,
    input       [1:0]  period_sel,
    input              auto_pause,
    // 转场握手
    input              fade_go,
    input              fade_busy,
    output             switch_req,
    output             switch_auto,
    // 状态
    output      [1:0]  disp_idx,
    output             display_valid,
    output             pic_changed,
    output             auto_play_on,
    output      [3:0]  image_complete,
    output      [3:0]  image_failed,
    output      [7:0]  picture_ids,
    output      [2:0]  found_count,
    output      [2:0]  err_code,
    output             error_active,
    input       [8:0]  write_used,       // 写 FIFO 水位（mem 域计数经 FIFO IP 给出的写侧值）
    output             write_flip,       // 1 = 自下而上（正高度 BMP），写调度做上下翻转
    // SDRAM 写侧
    input              write_finish_toggle,
    output      [1:0]  write_buf_idx,
    output             write_req,
    input              write_req_ack,
    output             write_en,
    output      [31:0] write_data,
    // 缩略图写口
    output             thumb_we_even,
    output             thumb_we_odd,
    output      [13:0] thumb_waddr,
    output      [15:0] thumb_wdata,
    // WAV PCM 字节 -> 异步 FIFO
    output             pcm_wr_en,
    output      [7:0]  pcm_wr_data,
    input  [AUDIO_FIFO_AW:0] pcm_fifo_used,
    output             wav_mode,
    output             wav_stereo,
    output             audio_clear,
    input              audio_clear_ack,
    // SD 卡
    output             SD_nCS,
    output             SD_DCLK,
    output             SD_MOSI,
    input              SD_MISO
);
    wire        sd_sec_read, sd_sec_read_data_valid, sd_sec_read_end, sd_init_done;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;
    wire        bmp_ready, scan_start, scan_done, scan_found_valid, load_error, load_start, load_busy;
    wire [31:0] scan_found_cluster, scan_found_size, load_cluster, load_size;
    wire [2:0]  reader_error;
    wire        bmp_wr_en;
    wire [23:0] bmp_data;
    wire        preload_done, sd_ready, sd_card_rst;
    wire        music_found, audio_start, audio_stop, audio_hold, audio_done, audio_err;
    wire        audio_byte_valid, fmt_ok, fmt_error;
    wire [1:0]  audio_idx;
    wire        parser_pcm_valid;
    wire [7:0]  audio_byte;
    wire pause_mode=DYNAMIC_MODE && mode_pause;
    assign media_idle=bmp_ready && !load_busy && !load_start && !audio_start && preload_done;
    wire nav_valid;
    reg [2:0] resize_state;
    reg [31:0] hide_count;
    reg [1:0] resize_idx;
    wire reload_go=resize_state==2;
    assign resize_busy=resize_state!=0 || !preload_done;
    assign display_valid=nav_valid && !resize_busy && image_complete[disp_idx];
    // Hide for >2 frames before writing; drain WAV before a new image load.
    always @(posedge clk or posedge rst)
        if(rst) begin resize_state<=0;hide_count<=0;resize_idx<=0;end
        else case(resize_state)
            0: if(resize_apply && preload_done && nav_valid) begin
                resize_idx<=disp_idx;hide_count<=0;resize_state<=1;
            end
            1: if(hide_count<CLK_FREQ_HZ/20) hide_count<=hide_count+1'b1;
               else if(bmp_ready && !load_busy && !audio_start && sd_ready) resize_state<=2;
            2: resize_state<=3;
            3: if(load_busy) resize_state<=4;
            4: if(!load_busy) resize_state<=0;
            default: resize_state<=0;
        endcase
    wire        top_down;
    assign write_flip = ~top_down;

    // write_req_ack 来自 mem_clk 域，先打两拍再给读取器（扇出多，避免采样不一致）
    reg [1:0] wr_ack_sync;
    always @(posedge clk or posedge rst)
        if (rst) wr_ack_sync <= 2'b00; else wr_ack_sync <= {wr_ack_sync[0], write_req_ack};

    assign write_en   = bmp_wr_en;
    assign write_data = {bmp_data, 8'd0};

    bank_preload_ctrl #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .MAX_RETRIES(MAX_RETRIES)) u_preload (
        .clk(clk), .rst(rst), .sdram_init_done(sdram_init_done),
        .reload_start(reload_go), .reload_idx(resize_idx),
        .reader_ready(bmp_ready), .sd_init_done(sd_init_done),
        .scan_done(scan_done), .scan_found_valid(scan_found_valid),
        .scan_found_cluster(scan_found_cluster), .scan_found_size(scan_found_size),
        .load_error(load_error), .reader_error(reader_error),
        .write_finish_toggle(write_finish_toggle),
        .scan_start(scan_start), .load_start(load_start), .load_cluster(load_cluster),
        .load_size(load_size), .load_idx(write_buf_idx), .load_busy(load_busy),
        .image_complete(image_complete), .image_failed(image_failed), .found_count(found_count),
        .preload_done(preload_done), .sd_ready(sd_ready), .sd_card_rst(sd_card_rst),
        .err_code(err_code), .error_active(error_active));

    bank_nav_ctrl #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_nav (
        .clk(clk), .rst(rst), .image_complete(image_complete), .found_count(found_count),
        .next_cmd(next_cmd && !resize_busy), .prev_cmd(prev_cmd && !resize_busy), .select_cmd(select_cmd && !resize_busy), .select_idx(select_idx),
        .navigation_hold(resize_busy | pause_mode), .auto_cmd(auto_cmd), .period_sel(period_sel), .auto_pause(auto_pause | resize_busy | pause_mode),
        .fade_go(fade_go), .fade_busy(fade_busy), .switch_req(switch_req), .switch_auto(switch_auto),
        .disp_idx(disp_idx), .disp_valid(nav_valid), .pic_changed(pic_changed), .auto_on(auto_play_on));

    fat32_bmp_reader #(.DYNAMIC_MODE(DYNAMIC_MODE), .CLK_FREQ_HZ(CLK_FREQ_HZ), .SD_READ_TIMEOUT_MS(SD_READ_TIMEOUT_MS)) u_reader (
        .clk(clk), .rst(rst), .ready(bmp_ready),
        .scan_start(scan_start), .scan_done(scan_done), .scan_found_valid(scan_found_valid),
        .scan_found_cluster(scan_found_cluster), .scan_found_size(scan_found_size),
        .scan_found_total(), .scan_error(),
        .mode_800(mode_800),
        .target_width(image_widths[write_buf_idx*10 +: 10]),
        .load_start(load_start), .load_cluster(load_cluster), .load_size(load_size),
        .sd_init_done(sd_init_done), .state_code(), .wr_used(write_used), .top_down(top_down),
        .write_req(write_req), .write_req_ack(wr_ack_sync[1]),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .bmp_data_wr_en(bmp_wr_en), .bmp_data(bmp_data),
        .bmp_error(reader_error), .load_error(load_error), .fs_error_code(),
        .picture_ids(picture_ids), .audio_idx(audio_idx), .music_found(music_found), .audio_start(audio_start), .audio_stop(audio_stop),
        .audio_hold(audio_hold), .audio_byte_valid(audio_byte_valid), .audio_byte(audio_byte),
        .audio_done(audio_done), .audio_err(audio_err), .audio_busy());

    sd_card_top u_sd (
        .clk(clk), .rst(sd_card_rst),
        .SD_nCS(SD_nCS), .SD_DCLK(SD_DCLK), .SD_MOSI(SD_MOSI), .SD_MISO(SD_MISO),
        .sd_init_done(sd_init_done),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .sd_sec_write(1'b0), .sd_sec_write_addr(32'd0), .sd_sec_write_data(),
        .sd_sec_write_data_req(), .sd_sec_write_end());

    thumb_capture #(.DYNAMIC_MODE(DYNAMIC_MODE)) u_thumb_cap (
        .mode_800(mode_800),
        .clk(clk), .rst_n(~rst), .load_begin(load_start), .load_idx(write_buf_idx), .top_down(top_down),
        .pix_valid(bmp_wr_en && load_busy), .pix_rgb(bmp_data),
        .we_even(thumb_we_even), .we_odd(thumb_we_odd), .waddr(thumb_waddr), .wdata(thumb_wdata));

    wav_stream_parser u_wav (
        .clk(clk), .rst_n(~rst & ~audio_clear), .start(audio_start),
        .byte_valid(audio_byte_valid), .byte_data(audio_byte),
        .pcm_valid(parser_pcm_valid), .pcm_byte(pcm_wr_data),
        .fmt_ok(fmt_ok), .stereo(wav_stereo), .fmt_error(fmt_error));

    assign pcm_wr_en = parser_pcm_valid && !audio_clear;

    audio_play_ctrl #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .FIFO_AW(AUDIO_FIFO_AW)) u_audio_ctrl (
        .clk(clk), .rst_n(~rst), .pause(resize_busy | pause_mode), .preload_done(preload_done), .music_found(music_found),
        .media_idx(disp_idx), .display_valid(display_valid), .audio_idx(audio_idx),
        .audio_clear(audio_clear), .audio_clear_ack(audio_clear_ack),
        .reader_ready(bmp_ready), .sd_ready(sd_ready),
        .audio_done(audio_done), .audio_err(audio_err), .fmt_ok(fmt_ok), .fmt_error(fmt_error),
        .fifo_used(pcm_fifo_used), .audio_start(audio_start), .audio_stop(audio_stop),
        .audio_hold(audio_hold), .wav_mode(wav_mode));
endmodule
