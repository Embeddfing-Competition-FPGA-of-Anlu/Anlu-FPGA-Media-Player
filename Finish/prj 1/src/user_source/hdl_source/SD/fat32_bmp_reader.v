`timescale 1ns/1ps

// 第6课：FAT32 版 BMP 读取器，替换 bmp_read
//   扫描：fat32_volume_scanner 识别 MBR/BPB，在根目录按文件名找 PIC1~4.BMP，
//         找到的文件按 PIC1->PIC4 顺序逐个通过 scan_found_valid 报给上层（缺失的自动跳过）
//   加载：fat32_file_streamer 按 FAT 链读文件字节流（支持碎片化文件、任意位置、大容量卡），
//         本模块从字节流里解析 BMP 头（第4课校验 + 扩展3：任意尺寸 1..2048、24/32 位、正负高度），
//         像素区字节进 2KB 字节 FIFO，由 bmp_scaler 等比缩放成 640x480 像素流输出
//   底层：两个 FAT32 模块分时共用 sd_sector_adapter -> 官方 sd_card_top
// 对上层（sd_card_bmp）的接口与 bmp_read 保持一致，只是"扇区号"换成了"起始簇号+文件大小"。
module fat32_bmp_reader #(
    parameter integer DYNAMIC_MODE=0,
    parameter integer CLK_FREQ_HZ        = 100_000_000,
    parameter integer SD_READ_TIMEOUT_MS = 500
)(
    input                       clk,
    input                       rst,
    output                      ready,

    // 扫描：找到的文件逐个报出
    input                       scan_start,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_cluster,
    output reg [31:0]           scan_found_size,
    output reg [2:0]            scan_found_total,
    output reg                  scan_error,         // 单拍：文件系统识别失败或一张图都没找到

    // 按起始簇号 + 文件大小加载一张图
    input wire mode_800,
    input wire [9:0] target_width,
    input                       load_start,
    input  [31:0]               load_cluster,
    input  [31:0]               load_size,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [8:0]                wr_used,            // 写 FIFO 水位（缩放器背压）
    output reg                  top_down,           // 本张图自上而下存储（负高度），不需要翻转

    output reg                  write_req,
    input                       write_req_ack,

    output                      sd_sec_read,
    output [31:0]               sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output                      bmp_data_wr_en,
    output [23:0]               bmp_data,

    // 错误码：0=无 1=BMP头 2=SD超时 3=像素截断 4=文件系统/FAT链错误
    output reg [2:0]            bmp_error,
    output reg                  load_error,
    output reg [7:0]            fs_error_code,      // FAT32 模块的详细错误码（调试用）

    // 按显示 bank 选择 MUSIC1~4.WAV；文件参数在 audio_start 时锁存。
    output [7:0]                picture_ids,
    input  [1:0]                audio_idx,          // 显示 bank 编号
    output                      music_found,        // 当前 bank 对应的 WAV 存在
    input                       audio_start,        // 单拍：从头开始读选中的 WAV
    input                       audio_stop,         // 单拍：切图/格式错误时安全中止
    input                       audio_hold,         // 下游 FIFO 剩余空间不足一个扇区
    output                      audio_byte_valid,
    output [7:0]                audio_byte,
    output reg                  audio_done,         // 单拍：文件读完（或出错）
    output reg                  audio_err,          // 与 audio_done 同拍：本次读取出错
    output                      audio_busy
);

localparam [2:0] ERR_NONE      = 3'd0;
localparam [2:0] ERR_HEADER    = 3'd1;
localparam [2:0] ERR_TIMEOUT   = 3'd2;
localparam [2:0] ERR_TRUNCATED = 3'd3;
localparam [2:0] ERR_FS        = 3'd4;

localparam [31:0] TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * SD_READ_TIMEOUT_MS;

localparam [3:0] ST_IDLE       = 4'd0;
localparam [3:0] ST_SCAN       = 4'd1;
localparam [3:0] ST_SCAN_EMIT  = 4'd2;
localparam [3:0] ST_LOAD_WAIT  = 4'd3;
localparam [3:0] ST_LOAD_HDR   = 4'd4;
localparam [3:0] ST_HDR_CHECK  = 4'd5;
localparam [3:0] ST_LOAD_DATA  = 4'd6;
localparam [3:0] ST_ABORT      = 4'd7;
localparam [3:0] ST_AUDIO      = 4'd8;
localparam [3:0] ST_AUDIO_ABT  = 4'd9;
localparam [3:0] ST_FLUSH      = 4'd10;     // 文件读完，等缩放器把 FIFO 里的像素全部输出

reg  [3:0]  state;
wire        sub_rst_n = ~rst & sd_init_done;   // SD 卡重新初始化时 FAT32 模块一起复位

// ---------------- 文件系统参数（扫描完成时锁存，SD 卡重新初始化后仍可用） ----------------
reg  [31:0] load_cluster_latched;
reg  [31:0] load_size_latched;
reg  [7:0]  fs_spc;
reg  [31:0] fs_fat_start;
reg  [31:0] fs_data_start;
reg  [31:0] fs_max_cluster;

// ---------------- 卷扫描器 ----------------
reg         scanner_start;
wire        scanner_done;
wire [7:0]  scanner_error;
wire [7:0]  scanner_spc;
wire [31:0] scanner_fat_start;
wire [31:0] scanner_data_start;
wire [31:0] scanner_max_cluster;
wire [3:0]  scanner_valid;
wire [31:0] scanner_cluster0, scanner_cluster1, scanner_cluster2, scanner_cluster3;
wire [31:0] scanner_size0, scanner_size1, scanner_size2, scanner_size3;
wire [3:0]  scanner_music_valid;
wire [127:0] scanner_music_cluster, scanner_music_size;
reg  [3:0]  music_valid_latched;
reg  [127:0] music_clusters, music_sizes;
reg  [1:0]  bank_music_id [0:3];
wire [1:0]  selected_music = bank_music_id[audio_idx];
assign picture_ids={bank_music_id[3],bank_music_id[2],bank_music_id[1],bank_music_id[0]};
assign music_found = music_valid_latched[selected_music];
reg  [31:0] music_cluster_latched;
reg  [31:0] music_size_latched;
wire        scanner_sector_req;
wire [31:0] scanner_sector_lba;

// ---------------- 文件流读取器 ----------------
reg         streamer_start;
reg         streamer_abort;
wire        img_hold;
wire        streamer_done;
wire        file_valid;
wire [7:0]  file_byte;
wire [31:0] file_byte_offset;
wire [7:0]  streamer_error;
wire        streamer_sector_req;
wire [31:0] streamer_sector_lba;

// ---------------- 扇区仲裁 + 适配层 ----------------
// 扫描和加载不会同时进行：扫描阶段用扫描器，其余时间用流读取器。
// 返回信号直接广播给两个模块，不在等待状态的模块会忽略。
wire        scan_phase = (state == ST_SCAN);
wire        audio_phase = (state == ST_AUDIO) || (state == ST_AUDIO_ABT);
wire        arb_sector_req = scan_phase ? scanner_sector_req : streamer_sector_req;
wire [31:0] arb_sector_lba = scan_phase ? scanner_sector_lba : streamer_sector_lba;
wire        sector_busy;
wire        sector_valid;
wire [7:0]  sector_byte;
wire [8:0]  sector_byte_index;
wire        sector_done;
wire [7:0]  sector_error;
wire        sector_timeout;
reg         timeout_seen;          // 本次扫描/加载中发生过读卡超时

fat32_volume_scanner u_scanner (
    .clk                 (clk),
    .rst_n               (sub_rst_n),
    .scan_start          (scanner_start),
    .scan_busy           (),
    .scan_done           (scanner_done),
    .fs_error            (scanner_error),
    .partition_lba       (),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .resource_valid      (scanner_valid),
    .resource0_cluster   (scanner_cluster0),
    .resource1_cluster   (scanner_cluster1),
    .resource2_cluster   (scanner_cluster2),
    .resource3_cluster   (scanner_cluster3),
    .resource0_size      (scanner_size0),
    .resource1_size      (scanner_size1),
    .resource2_size      (scanner_size2),
    .resource3_size      (scanner_size3),
    .music_valid         (scanner_music_valid),
    .music_cluster       (scanner_music_cluster),
    .music_size          (scanner_music_size),
    .sector_req          (scanner_sector_req),
    .sector_lba          (scanner_sector_lba),
    .sector_busy         (sector_busy),
    .sector_valid        (sector_valid),
    .sector_byte         (sector_byte),
    .sector_byte_index   (sector_byte_index),
    .sector_done         (sector_done),
    .sector_error        (sector_error)
);

fat32_file_streamer u_streamer (
    .clk                 (clk),
    .rst_n               (sub_rst_n),
    .file_start          (streamer_start),
    .file_abort          (streamer_abort),
    .file_hold           (audio_phase ? audio_hold : img_hold),
    .first_cluster       (audio_phase ? music_cluster_latched : load_cluster_latched),
    .file_size           (audio_phase ? music_size_latched    : load_size_latched),
    .sectors_per_cluster (fs_spc),
    .fat_start_lba       (fs_fat_start),
    .data_start_lba      (fs_data_start),
    .max_cluster         (fs_max_cluster),
    .file_busy           (),
    .file_done           (streamer_done),
    .file_valid          (file_valid),
    .file_byte           (file_byte),
    .file_byte_offset    (file_byte_offset),
    .fs_error            (streamer_error),
    .sector_req          (streamer_sector_req),
    .sector_lba          (streamer_sector_lba),
    .sector_busy         (sector_busy),
    .sector_valid        (sector_valid),
    .sector_byte         (sector_byte),
    .sector_byte_index   (sector_byte_index),
    .sector_done         (sector_done),
    .sector_error        (sector_error)
);

sd_sector_adapter #(
    .TIMEOUT_CYCLES         (TIMEOUT_CYCLES)
) u_adapter (
    .clk                    (clk),
    .rst_n                  (sub_rst_n),
    .sector_req             (arb_sector_req),
    .sector_lba             (arb_sector_lba),
    .sector_busy            (sector_busy),
    .sector_valid           (sector_valid),
    .sector_byte            (sector_byte),
    .sector_byte_index      (sector_byte_index),
    .sector_done            (sector_done),
    .sector_error           (sector_error),
    .sector_timeout         (sector_timeout),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_end        (sd_sec_read_end)
);

// ---------------- BMP 头字段（从文件字节流里抓取） ----------------
reg  [7:0]  header_0;
reg  [7:0]  header_1;
reg  [31:0] file_len;
reg  [31:0] pixel_offset;
reg  [31:0] dib_size;
reg  [31:0] width;
reg  [31:0] height;
reg  [15:0] planes;
reg  [15:0] bit_count;
reg  [31:0] compression;
reg  [31:0] wait_cnt;
reg  [1:0]  emit_idx;

// 扩展3：源图参数。宽高取绝对值，行字节数按 4 字节补齐；乘法结果寄存，头字段在 offset 53 前早已到齐
wire        h_neg   = height[31];
wire [31:0] h_abs   = h_neg ? (~height + 32'd1) : height;
// Exactly +1..+2048 or -1..-2048, without a 32-bit negate before comparison.
wire height_in_range = ((height[31:12] == 20'd0) &&
                        (height[11:0] != 12'd0) && (height[11:0] <= 12'd2048)) ||
                       (height[31:11] == 21'h1fffff);
wire        is32    = (bit_count == 16'd32);
reg  [13:0] stride;
reg  [31:0] pix_bytes;            // 像素区总字节数
reg  [31:0] pixel_end;            // pixel_offset + pix_bytes
always @(posedge clk) begin
    stride    <= is32 ? {width[11:0], 2'b00} : (({2'b0, width[11:0]} + {1'b0, width[11:0], 1'b0} + 14'd3) & 14'h3ffc);
    pix_bytes <= stride * h_abs[11:0];
    pixel_end <= pixel_offset + pix_bytes;
end

// 校验：BM、DIB>=40、宽高 1..2048、单平面、24 位无压缩 或 32 位（无压缩/BITFIELDS，按标准 BGRA 掩码）
wire header_match = (header_0 == "B") && (header_1 == "M") &&
                    (dib_size >= 32'd40) &&
                    (width  != 32'd0) && (width  <= 32'd2048) &&
                    height_in_range &&
                    (planes       == 16'd1) &&
                    (((bit_count == 16'd24) && (compression == 32'd0)) ||
                     ((bit_count == 16'd32) && (compression == 32'd0 || compression == 32'd3))) &&
                    (pixel_offset >= 32'd54) &&
                    (pixel_offset <  file_len) &&
                    (file_len          >= pixel_end) &&
                    (load_size_latched >= pixel_end);

// 像素区字节 -> 字节 FIFO（与状态无关地按偏移判断；offset>=54 时头字段都已锁存）
reg         px_push;
reg  [7:0]  px_byte;
reg  [31:0] push_cnt;
reg         fifo_clr, sc_start, sc_abort;
wire [11:0] fifo_count;
wire        fifo_empty, fifo_pop;
wire [7:0]  fifo_dout;
wire        sc_busy;
wire        load_phase = (state == ST_LOAD_HDR) || (state == ST_HDR_CHECK) || (state == ST_LOAD_DATA);
assign      img_hold   = (fifo_count >= 12'd1024);       // 再来一个扇区（512B）也装得下
always @(posedge clk or posedge rst) begin
    if (rst) begin
        px_push <= 1'b0;  px_byte <= 8'd0;  push_cnt <= 32'd0;
    end else begin
        px_push <= load_phase && file_valid && (file_byte_offset >= 32'd54) &&
                   (file_byte_offset >= pixel_offset) && (file_byte_offset < pixel_end);
        px_byte <= file_byte;
        if (fifo_clr)     push_cnt <= 32'd0;
        else if (px_push) push_cnt <= push_cnt + 32'd1;
    end
end

byte_fifo #(.AW(11)) u_pix_fifo (
    .clk(clk), .rst_n(~rst), .clr(fifo_clr), .push(px_push), .din(px_byte),
    .pop(fifo_pop), .dout(fifo_dout), .count(fifo_count), .empty(fifo_empty));

bmp_scaler #(.DYNAMIC_MODE(DYNAMIC_MODE)) u_scaler (
    .mode_800(mode_800),
    .target_width(target_width),
    .clk(clk), .rst_n(~rst), .start(sc_start), .abort(sc_abort || !sd_init_done),
    .src_w(width[11:0]), .src_h(h_abs[11:0]), .bpp32(is32),
    .fifo_empty(fifo_empty), .fifo_dout(fifo_dout), .fifo_pop(fifo_pop),
    .wr_used(wr_used), .pix_valid(bmp_data_wr_en), .pix_rgb(bmp_data), .busy(sc_busy));

assign ready = (state == ST_IDLE) && sd_init_done && !sc_busy;
assign audio_byte_valid = (state == ST_AUDIO) && file_valid;
assign audio_byte       = file_byte;
assign audio_busy       = audio_phase;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        dib_size     <= 32'd0;
        width        <= 32'd0;
        height       <= 32'd0;
        planes       <= 16'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (state == ST_LOAD_HDR && file_valid) begin
        case (file_byte_offset[5:0])
            6'd0 : header_0 <= file_byte;
            6'd1 : header_1 <= file_byte;
            6'd2 : file_len[7:0]   <= file_byte;
            6'd3 : file_len[15:8]  <= file_byte;
            6'd4 : file_len[23:16] <= file_byte;
            6'd5 : file_len[31:24] <= file_byte;
            6'd10: pixel_offset[7:0]   <= file_byte;
            6'd11: pixel_offset[15:8]  <= file_byte;
            6'd12: pixel_offset[23:16] <= file_byte;
            6'd13: pixel_offset[31:24] <= file_byte;
            6'd14: dib_size[7:0]   <= file_byte;
            6'd15: dib_size[15:8]  <= file_byte;
            6'd16: dib_size[23:16] <= file_byte;
            6'd17: dib_size[31:24] <= file_byte;
            6'd18: width[7:0]   <= file_byte;
            6'd19: width[15:8]  <= file_byte;
            6'd20: width[23:16] <= file_byte;
            6'd21: width[31:24] <= file_byte;
            6'd22: height[7:0]   <= file_byte;
            6'd23: height[15:8]  <= file_byte;
            6'd24: height[23:16] <= file_byte;
            6'd25: height[31:24] <= file_byte;
            6'd26: planes[7:0]  <= file_byte;
            6'd27: planes[15:8] <= file_byte;
            6'd28: bit_count[7:0]  <= file_byte;
            6'd29: bit_count[15:8] <= file_byte;
            6'd30: compression[7:0]   <= file_byte;
            6'd31: compression[15:8]  <= file_byte;
            6'd32: compression[23:16] <= file_byte;
            6'd33: compression[31:24] <= file_byte;
            default: ;
        endcase
    end
end

// ---------------- 主状态机 ----------------
task report_error;
    input [2:0] code;
    begin
        bmp_error  <= code;
        load_error <= 1'b1;
        sc_abort   <= 1'b1;
    end
endtask

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state              <= ST_IDLE;
        state_code         <= 4'd0;
        write_req          <= 1'b0;
        scan_done          <= 1'b0;
        scan_found_valid   <= 1'b0;
        scan_found_cluster <= 32'd0;
        scan_found_size    <= 32'd0;
        scan_found_total   <= 3'd0;
        scan_error         <= 1'b0;
        scanner_start      <= 1'b0;
        streamer_start     <= 1'b0;
        streamer_abort     <= 1'b0;
        load_cluster_latched <= 32'd0;
        load_size_latched  <= 32'd0;
        fs_spc             <= 8'd0;
        fs_fat_start       <= 32'd0;
        fs_data_start      <= 32'd0;
        fs_max_cluster     <= 32'd0;
        bmp_error          <= ERR_NONE;
        load_error         <= 1'b0;
        fs_error_code      <= 8'd0;
        wait_cnt           <= 32'd0;
        emit_idx           <= 2'd0;
        timeout_seen       <= 1'b0;
        music_valid_latched <= 4'd0;
        music_clusters     <= 128'd0;
        music_sizes        <= 128'd0;
        bank_music_id[0]   <= 2'd0;
        bank_music_id[1]   <= 2'd1;
        bank_music_id[2]   <= 2'd2;
        bank_music_id[3]   <= 2'd3;
        music_cluster_latched <= 32'd0;
        music_size_latched <= 32'd0;
        audio_done         <= 1'b0;
        audio_err          <= 1'b0;
        fifo_clr           <= 1'b0;
        sc_start           <= 1'b0;
        sc_abort           <= 1'b0;
        top_down           <= 1'b0;
    end else if (!sd_init_done) begin
        // SD 卡重新初始化：若正在放音乐，报一次 done+err，上层稍后从头重放
        audio_done       <= audio_phase;
        audio_err        <= audio_phase;
        // SD 卡（重新）初始化中：停在空闲，扫描结果与文件系统参数保留
        state            <= ST_IDLE;
        state_code       <= 4'd0;
        write_req        <= 1'b0;
        scan_found_valid <= 1'b0;
        scan_error       <= 1'b0;
        scanner_start    <= 1'b0;
        streamer_start   <= 1'b0;
        streamer_abort   <= 1'b0;
        load_error       <= 1'b0;
        timeout_seen     <= 1'b0;
    end else begin
        scan_found_valid <= 1'b0;
        scan_error       <= 1'b0;
        scanner_start    <= 1'b0;
        streamer_start   <= 1'b0;
        streamer_abort   <= 1'b0;
        load_error       <= 1'b0;
        audio_done       <= 1'b0;
        audio_err        <= 1'b0;
        fifo_clr         <= 1'b0;
        sc_start         <= 1'b0;
        sc_abort         <= 1'b0;
        if (sector_timeout)
            timeout_seen <= 1'b1;

        case (state)
            ST_IDLE: begin
                state_code   <= 4'd1;
                write_req    <= 1'b0;
                timeout_seen <= 1'b0;
                if (scan_start) begin
                    bmp_error        <= ERR_NONE;
                    fs_error_code    <= 8'd0;
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    scanner_start    <= 1'b1;
                    music_valid_latched <= 4'd0;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    bmp_error            <= ERR_NONE;
                    load_cluster_latched <= load_cluster;
                    load_size_latched    <= load_size;
                    // 先和 SDRAM 写侧握手，再开始读文件，像素字节不会在等待应答时丢失
                    write_req            <= 1'b1;
                    wait_cnt             <= 32'd0;
                    fifo_clr             <= 1'b1;
                    state                <= ST_LOAD_WAIT;
                end else if (audio_start && music_found) begin
                    // 不让切图改变正在读取的文件参数；旧流先由 audio_stop 收尾。
                    music_cluster_latched <= music_clusters[selected_music * 32 +: 32];
                    music_size_latched <= music_sizes[selected_music * 32 +: 32];
                    streamer_start <= 1'b1;
                    state          <= ST_AUDIO;
                end
            end

            ST_AUDIO: begin
                state_code <= 4'd5;
                if (streamer_done) begin
                    audio_done <= 1'b1;
                    audio_err  <= (streamer_error != 8'd0) || timeout_seen || sector_timeout;
                    state      <= ST_IDLE;
                end else if (audio_stop) begin
                    streamer_abort <= 1'b1;
                    state          <= ST_AUDIO_ABT;
                end
            end

            ST_AUDIO_ABT: begin
                if (streamer_done) begin
                    audio_done <= 1'b1;
                    audio_err  <= 1'b1;
                    state      <= ST_IDLE;
                end
            end

            ST_SCAN: begin
                state_code <= 4'd2;
                if (scanner_done) begin
                    fs_error_code <= scanner_error;
                    if (scanner_error != 8'd0) begin
                        // 文件系统识别失败 / 根目录里一张 PIC 都没有
                        bmp_error  <= (timeout_seen || sector_timeout) ? ERR_TIMEOUT : ERR_FS;
                        scan_error <= 1'b1;
                        scan_done  <= 1'b1;
                        state      <= ST_IDLE;
                    end else begin
                        fs_spc         <= scanner_spc;
                        fs_fat_start   <= scanner_fat_start;
                        fs_data_start  <= scanner_data_start;
                        fs_max_cluster <= scanner_max_cluster;
                        music_valid_latched <= scanner_music_valid;
                        music_clusters <= scanner_music_cluster;
                        music_sizes <= scanner_music_size;
                        emit_idx       <= 2'd0;
                        state          <= ST_SCAN_EMIT;
                    end
                end
            end

            ST_SCAN_EMIT: begin
                // 按 PIC1 -> PIC4 顺序把找到的文件逐个报给上层，缺失的跳过
                if (scanner_valid[emit_idx]) begin
                    bank_music_id[scan_found_total[1:0]] <= emit_idx;
                    scan_found_valid <= 1'b1;
                    scan_found_total <= scan_found_total + 3'd1;
                    case (emit_idx)
                        2'd0: begin scan_found_cluster <= scanner_cluster0; scan_found_size <= scanner_size0; end
                        2'd1: begin scan_found_cluster <= scanner_cluster1; scan_found_size <= scanner_size1; end
                        2'd2: begin scan_found_cluster <= scanner_cluster2; scan_found_size <= scanner_size2; end
                        default: begin scan_found_cluster <= scanner_cluster3; scan_found_size <= scanner_size3; end
                    endcase
                end
                if (emit_idx == 2'd3) begin
                    scan_done <= 1'b1;
                    state     <= ST_IDLE;
                end
                emit_idx <= emit_idx + 2'd1;
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req      <= 1'b0;
                    streamer_start <= 1'b1;
                    state          <= ST_LOAD_HDR;
                end else if (wait_cnt >= TIMEOUT_CYCLES) begin
                    write_req <= 1'b0;
                    report_error(ERR_TIMEOUT);
                    state     <= ST_IDLE;
                end else begin
                    wait_cnt <= wait_cnt + 32'd1;
                end
            end

            ST_LOAD_HDR: begin
                state_code <= 4'd2;
                if (streamer_done) begin
                    // 文件不足 54 字节就结束了，或者读卡出错
                    fs_error_code <= streamer_error;
                    if (timeout_seen || sector_timeout)
                        report_error(ERR_TIMEOUT);
                    else if (streamer_error != 8'd0)
                        report_error(ERR_FS);
                    else
                        report_error(ERR_HEADER);
                    state <= ST_IDLE;
                end else if (file_valid && file_byte_offset == 32'd53) begin
                    state <= ST_HDR_CHECK;
                end
            end

            ST_HDR_CHECK: begin
                if (header_match) begin
                    top_down <= h_neg;                     // 写调度第一次突发前已同步过去
                    sc_start <= 1'b1;
                    state <= ST_LOAD_DATA;
                end else begin
                    // 头校验失败：中止文件读取，不再把整个坏文件读完
                    streamer_abort <= 1'b1;
                    report_error(ERR_HEADER);
                    state <= ST_ABORT;
                end
            end

            ST_LOAD_DATA: begin
                state_code <= 4'd4;
                if (streamer_done) begin
                    fs_error_code <= streamer_error;
                    if (timeout_seen || sector_timeout)
                        report_error(ERR_TIMEOUT);
                    else if (streamer_error != 8'd0)
                        report_error(ERR_FS);
                    else if (push_cnt < pix_bytes)
                        report_error(ERR_TRUNCATED);
                    state <= (timeout_seen || sector_timeout || streamer_error != 8'd0 ||
                              push_cnt < pix_bytes) ? ST_IDLE : ST_FLUSH;
                end
            end

            ST_FLUSH: begin
                // 源文件已全部进 FIFO；缩放器输出完 640x480 后回空闲，上层再等 SDRAM 写完
                state_code <= 4'd6;
                if (!sc_busy) state <= ST_IDLE;
            end

            ST_ABORT: begin
                // 等流读取器回到空闲（它会等当前扇区读完）
                if (streamer_done)
                    state <= ST_IDLE;
            end

            default: begin
                state     <= ST_IDLE;
                write_req <= 1'b0;
            end
        endcase
    end
end

endmodule
