`timescale 1ns/1ps

// 第6课：FAT32 卷扫描器（参考完赛工程 rtl/filesystem/fat32_volume_scanner.v）
// 读 LBA0 判断 MBR/直接引导扇区 -> 解析 BPB -> 扫描根目录第一个簇 -> 按 8.3 短文件名匹配 PIC1~4.BMP
// 相对完赛工程的改动：
//   1) 不再缓存整个 512 字节扇区（会综合出 4096 个寄存器），只在接收时抓取需要的字段；
//   2) 根目录扫描整个簇（原代码只读了簇里的第一个扇区，只有 16 个目录项），遇到目录结束项提前停止；
//   3) 属性只排除子目录/卷标/长文件名项（原代码要求属性必须等于 0x20）；
//   4) 找到 1~3 张也算成功（resource_valid 指出哪几张），一张都没有才报 ERR_RESOURCE_MISSING。
module fat32_volume_scanner (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    // PICn.BMP 对应 MUSICn.WAV；每个 32 位片段的低编号放在低位。
    output reg  [3:0]  music_valid,
    output reg  [127:0] music_cluster,
    output reg  [127:0] music_size,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);

    localparam ERR_NONE            = 8'h00;
    localparam ERR_SECTOR          = 8'h10;
    localparam ERR_BOOT_SIGNATURE  = 8'h20;
    localparam ERR_UNSUPPORTED_BPB = 8'h21;
    localparam ERR_RESOURCE_MISSING= 8'h22;

    localparam ST_IDLE       = 4'd0;
    localparam ST_READ0_REQ  = 4'd1;
    localparam ST_READ0_WAIT = 4'd2;
    localparam ST_BOOT_PARSE = 4'd3;
    localparam ST_BOOT_REQ   = 4'd4;
    localparam ST_BOOT_WAIT  = 4'd5;
    localparam ST_ROOT_REQ   = 4'd6;
    localparam ST_ROOT_WAIT  = 4'd7;
    localparam ST_DIR_SCAN   = 4'd8;
    localparam ST_FINISH     = 4'd9;
    localparam ST_BOOT_LAYOUT= 4'd10;
    localparam ST_BOOT_ROOT  = 4'd11;

    reg [3:0] state;

    // 接收 LBA0 / 引导扇区时抓取的字段（小端）
    reg [7:0]  cap_byte0;          // 偏移 0：跳转指令 EB/E9 表示直接引导扇区
    reg [7:0]  cap_part_type;      // 偏移 450：第一个分区类型
    reg [31:0] cap_part_lba;       // 偏移 454：第一个分区起始 LBA
    reg        bpb_bad_q, parse_wait;
    reg [31:0] root_off;
    reg [15:0] cap_bytes_per_sec;  // 偏移 11
    reg [7:0]  cap_spc;            // 偏移 13
    reg [15:0] cap_reserved;       // 偏移 14
    reg [7:0]  cap_fat_count;      // 偏移 16
    reg [15:0] cap_root_entries;   // 偏移 17（FAT32 必须为 0）
    reg [15:0] cap_fat_size16;     // 偏移 22（FAT32 必须为 0）
    reg [31:0] cap_total_sectors;  // 偏移 32
    reg [31:0] cap_fat_size32;     // 偏移 36
    reg [31:0] cap_root_cluster;   // 偏移 44
    reg [7:0]  cap_sig0;           // 偏移 510
    reg [7:0]  cap_sig1;           // 偏移 511

    reg [31:0] root_cluster;
    reg [31:0] total_sectors;
    reg [15:0] reserved_sectors;
    reg [31:0] data_sector_count;
    reg [31:0] fat_span;
    reg [2:0]  sectors_per_cluster_shift;
    reg [7:0]  root_sector_index;  // 根目录簇内的扇区号
    reg        dir_end_seen;       // 遇到目录结束项（首字节 0x00），后面的项都无效

    reg [7:0] dir_name0;
    reg [7:0] dir_name1;
    reg [7:0] dir_name2;
    reg [7:0] dir_name3;
    reg [7:0] dir_name4;
    reg [7:0] dir_name5;
    reg [7:0] dir_name6;
    reg [7:0] dir_name7;
    reg [7:0] dir_name8;
    reg [7:0] dir_name9;
    reg [7:0] dir_name10;
    reg       dir_entry_match;
    reg       dir_music_match;
    reg [1:0] dir_music_index;
    reg [1:0] dir_resource_index;
    reg [15:0] dir_cluster_high;
    reg [15:0] dir_cluster_low;
    reg [31:0] dir_file_size;

    wire capture_boot = sector_valid &&
                        (state == ST_READ0_WAIT || state == ST_BOOT_WAIT);

    assign scan_busy = (state != ST_IDLE);

    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = value != 0 && ((value & (value - 1'b1)) == 0);
        end
    endfunction

    function [2:0] spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd2:   spc_shift = 3'd1;
                8'd4:   spc_shift = 3'd2;
                8'd8:   spc_shift = 3'd3;
                8'd16:  spc_shift = 3'd4;
                8'd32:  spc_shift = 3'd5;
                8'd64:  spc_shift = 3'd6;
                8'd128: spc_shift = 3'd7;
                default: spc_shift = 3'd0;
            endcase
        end
    endfunction

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    // 引导扇区 / MBR 字段抓取
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cap_byte0 <= 8'd0;
            cap_part_type <= 8'd0;
            cap_part_lba <= 32'd0;
            cap_bytes_per_sec <= 16'd0;
            cap_spc <= 8'd0;
            cap_reserved <= 16'd0;
            cap_fat_count <= 8'd0;
            cap_root_entries <= 16'd0;
            cap_fat_size16 <= 16'd0;
            cap_total_sectors <= 32'd0;
            cap_fat_size32 <= 32'd0;
            cap_root_cluster <= 32'd0;
            cap_sig0 <= 8'd0;
            cap_sig1 <= 8'd0;
        end else if (capture_boot) begin
            case (sector_byte_index)
                9'd0:   cap_byte0 <= sector_byte;
                9'd11:  cap_bytes_per_sec[7:0] <= sector_byte;
                9'd12:  cap_bytes_per_sec[15:8] <= sector_byte;
                9'd13:  cap_spc <= sector_byte;
                9'd14:  cap_reserved[7:0] <= sector_byte;
                9'd15:  cap_reserved[15:8] <= sector_byte;
                9'd16:  cap_fat_count <= sector_byte;
                9'd17:  cap_root_entries[7:0] <= sector_byte;
                9'd18:  cap_root_entries[15:8] <= sector_byte;
                9'd22:  cap_fat_size16[7:0] <= sector_byte;
                9'd23:  cap_fat_size16[15:8] <= sector_byte;
                9'd32:  cap_total_sectors[7:0] <= sector_byte;
                9'd33:  cap_total_sectors[15:8] <= sector_byte;
                9'd34:  cap_total_sectors[23:16] <= sector_byte;
                9'd35:  cap_total_sectors[31:24] <= sector_byte;
                9'd36:  cap_fat_size32[7:0] <= sector_byte;
                9'd37:  cap_fat_size32[15:8] <= sector_byte;
                9'd38:  cap_fat_size32[23:16] <= sector_byte;
                9'd39:  cap_fat_size32[31:24] <= sector_byte;
                9'd44:  cap_root_cluster[7:0] <= sector_byte;
                9'd45:  cap_root_cluster[15:8] <= sector_byte;
                9'd46:  cap_root_cluster[23:16] <= sector_byte;
                9'd47:  cap_root_cluster[31:24] <= sector_byte;
                9'd450: cap_part_type <= sector_byte;
                9'd454: cap_part_lba[7:0] <= sector_byte;
                9'd455: cap_part_lba[15:8] <= sector_byte;
                9'd456: cap_part_lba[23:16] <= sector_byte;
                9'd457: cap_part_lba[31:24] <= sector_byte;
                9'd510: cap_sig0 <= sector_byte;
                9'd511: cap_sig1 <= sector_byte;
                default: ;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            scan_done <= 1'b0;
            fs_error <= ERR_NONE;
            partition_lba <= 32'd0;
            sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0;
            data_start_lba <= 32'd0;
            max_cluster <= 32'd0;
            resource_valid <= 4'd0;
            resource0_cluster <= 32'd0;
            resource1_cluster <= 32'd0;
            resource2_cluster <= 32'd0;
            resource3_cluster <= 32'd0;
            resource0_size <= 32'd0;
            resource1_size <= 32'd0;
            resource2_size <= 32'd0;
            resource3_size <= 32'd0;
            music_valid <= 4'd0;
            music_cluster <= 128'd0;
            music_size <= 128'd0;
            dir_music_index <= 2'd0;
            dir_music_match <= 1'b0;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            root_cluster <= 32'd0;
            total_sectors <= 32'd0;
            reserved_sectors <= 16'd0;
            data_sector_count <= 32'd0;
            fat_span <= 32'd0;
            sectors_per_cluster_shift <= 3'd0;
            root_sector_index <= 8'd0;
            dir_end_seen <= 1'b0;
            dir_name0 <= 8'd0;
            dir_name1 <= 8'd0;
            dir_name2 <= 8'd0;
            dir_name3 <= 8'd0;
            dir_name4 <= 8'd0;
            dir_name5 <= 8'd0;
            dir_name6 <= 8'd0;
            dir_name7 <= 8'd0;
            dir_name8 <= 8'd0;
            dir_name9 <= 8'd0;
            dir_name10 <= 8'd0;
            dir_entry_match <= 1'b0;
            dir_resource_index <= 2'd3;
            dir_cluster_high <= 16'd0;
            dir_cluster_low <= 16'd0;
            dir_file_size <= 32'd0;
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;

            // 根目录扇区：每 32 字节一个目录项，边收边匹配
            if (sector_valid && state == ST_ROOT_WAIT) begin
                case (sector_byte_index[4:0])
                    5'd0: begin
                        dir_name0 <= sector_byte;
                        dir_entry_match <= 1'b0;
                        dir_music_match <= 1'b0;
                        if (sector_byte == 8'h00)
                            dir_end_seen <= 1'b1;
                    end
                    5'd1:  dir_name1 <= sector_byte;
                    5'd2:  dir_name2 <= sector_byte;
                    5'd3:  dir_name3 <= sector_byte;
                    5'd4:  dir_name4 <= sector_byte;
                    5'd5:  dir_name5 <= sector_byte;
                    5'd6:  dir_name6 <= sector_byte;
                    5'd7:  dir_name7 <= sector_byte;
                    5'd8:  dir_name8 <= sector_byte;
                    5'd9:  dir_name9 <= sector_byte;
                    5'd10: dir_name10 <= sector_byte;
                    5'd11: begin
                        // 属性：排除长文件名项(0x0F)、卷标(0x08)、子目录(0x10)
                        if (!dir_end_seen && sector_byte != 8'h0f &&
                            (sector_byte & 8'h18) == 8'h00 &&
                            dir_name0 == "P" && dir_name1 == "I" &&
                            dir_name2 == "C" && dir_name3 >= "1" &&
                            dir_name3 <= "4" && dir_name4 == " " &&
                            dir_name5 == " " && dir_name6 == " " &&
                            dir_name7 == " " && dir_name8 == "B" &&
                            dir_name9 == "M" && dir_name10 == "P") begin
                            dir_entry_match <= 1'b1;
                            dir_resource_index <= dir_name3 - "1";
                        end
                        // MUSIC1~4.WAV，8.3 短名 MUSICn  WAV
                        if (!dir_end_seen && sector_byte != 8'h0f &&
                            (sector_byte & 8'h18) == 8'h00 &&
                            dir_name0 == "M" && dir_name1 == "U" &&
                            dir_name2 == "S" && dir_name3 == "I" &&
                            dir_name4 == "C" && dir_name5 >= "1" && dir_name5 <= "4" &&
                            dir_name6 == " " && dir_name7 == " " &&
                            dir_name8 == "W" && dir_name9 == "A" &&
                            dir_name10 == "V") begin
                            dir_music_match <= 1'b1;
                            dir_music_index <= dir_name5 - "1";
                        end
                    end
                    5'd20: dir_cluster_high[7:0] <= sector_byte;
                    5'd21: dir_cluster_high[15:8] <= sector_byte;
                    5'd26: dir_cluster_low[7:0] <= sector_byte;
                    5'd27: dir_cluster_low[15:8] <= sector_byte;
                    5'd28: dir_file_size[7:0] <= sector_byte;
                    5'd29: dir_file_size[15:8] <= sector_byte;
                    5'd30: dir_file_size[23:16] <= sector_byte;
                    5'd31: begin
                        dir_file_size[31:24] <= sector_byte;
                        // 同名文件只取第一个
                        if (dir_entry_match && !resource_valid[dir_resource_index]) begin
                            case (dir_resource_index)
                                2'd0: begin
                                    resource0_cluster <= {4'd0, dir_cluster_high[11:0], dir_cluster_low};  // 高4位为FAT32保留位
                                    resource0_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[0] <= 1'b1;
                                end
                                2'd1: begin
                                    resource1_cluster <= {4'd0, dir_cluster_high[11:0], dir_cluster_low};  // 高4位为FAT32保留位
                                    resource1_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[1] <= 1'b1;
                                end
                                2'd2: begin
                                    resource2_cluster <= {4'd0, dir_cluster_high[11:0], dir_cluster_low};  // 高4位为FAT32保留位
                                    resource2_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[2] <= 1'b1;
                                end
                                default: begin
                                    resource3_cluster <= {4'd0, dir_cluster_high[11:0], dir_cluster_low};  // 高4位为FAT32保留位
                                    resource3_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[3] <= 1'b1;
                                end
                            endcase
                        end
                        if (dir_music_match && !music_valid[dir_music_index]) begin
                            music_cluster[dir_music_index * 32 +: 32] <= {4'd0, dir_cluster_high[11:0], dir_cluster_low};
                            music_size[dir_music_index * 32 +: 32] <= {sector_byte, dir_file_size[23:0]};
                            music_valid[dir_music_index] <= 1'b1;
                        end
                    end
                    default: begin end
                endcase
            end

            case (state)
                ST_IDLE: if (scan_start) begin
                    fs_error <= ERR_NONE;
                    partition_lba <= 32'd0;
                    resource_valid <= 4'd0;
                    music_valid <= 4'd0;
                    state <= ST_READ0_REQ;
                end
                ST_READ0_REQ: if (!sector_busy) begin
                    sector_lba <= 32'd0;
                    sector_req <= 1'b1;
                    state <= ST_READ0_WAIT;
                end
                ST_READ0_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (cap_sig0 != 8'h55 || cap_sig1 != 8'haa)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    // 无MBR的超级软盘格式：跳转指令 + 512字节/扇区才认定为引导扇区，避免把以EB开头的MBR误判
                    else if ((cap_byte0 == 8'heb || cap_byte0 == 8'he9) && cap_bytes_per_sec == 16'd512) begin
                        partition_lba <= 32'd0;
                        state <= ST_BOOT_PARSE;
                    end else if (cap_part_type == 8'h0b || cap_part_type == 8'h0c) begin
                        partition_lba <= cap_part_lba;
                        sector_lba <= cap_part_lba;
                        state <= ST_BOOT_REQ;
                    end else begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end
                end
                ST_BOOT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_BOOT_WAIT;
                end
                ST_BOOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (cap_sig0 != 8'h55 || cap_sig1 != 8'haa)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else
                        state <= ST_BOOT_PARSE;
                end
                ST_BOOT_PARSE: begin
                    // Timing fix: BPB validity is precomputed into bpb_bad_q; wait one cycle on entry so it reflects the new boot sector
                    if (!parse_wait) begin
                    end else if (bpb_bad_q) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        sectors_per_cluster <= cap_spc;
                        reserved_sectors <= cap_reserved;
                        total_sectors <= cap_total_sectors;
                        root_cluster <= cap_root_cluster;
                        sectors_per_cluster_shift <= spc_shift(cap_spc);
                        fat_span <= (cap_fat_count == 2) ? (cap_fat_size32 << 1) : cap_fat_size32;
                        fat_start_lba <= partition_lba + cap_reserved;
                        state <= ST_BOOT_LAYOUT;
                    end
                end
                ST_BOOT_LAYOUT: begin
                    data_start_lba <= partition_lba + reserved_sectors + fat_span;
                    root_off <= (root_cluster - 32'd2) << sectors_per_cluster_shift;   // timing fix: precompute root-directory offset
                    data_sector_count <= total_sectors - reserved_sectors - fat_span;
                    state <= ST_BOOT_ROOT;
                end
                ST_BOOT_ROOT: begin
                    max_cluster <= (data_sector_count >> sectors_per_cluster_shift) + 1'b1;
                    sector_lba <= data_start_lba + root_off;
                    root_sector_index <= 8'd0;
                    dir_end_seen <= 1'b0;
                    resource_valid <= 4'd0;
                    music_valid <= 4'd0;
                    state <= ST_ROOT_REQ;
                end
                ST_ROOT_REQ: if (!sector_busy) begin
                    dir_entry_match <= 1'b0;
                    dir_music_match <= 1'b0;
                    sector_req <= 1'b1;
                    state <= ST_ROOT_WAIT;
                end
                ST_ROOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else
                        state <= ST_DIR_SCAN;
                end
                ST_DIR_SCAN: begin
                    // 四张图及四段音乐都找到、遇到结束项、或根目录首簇读完即结束。
                    if ((resource_valid == 4'hf && music_valid == 4'hf) || dir_end_seen ||
                        root_sector_index + 1'b1 >= sectors_per_cluster) begin
                        if (resource_valid == 4'd0)
                            finish_with_error(ERR_RESOURCE_MISSING);
                        else
                            state <= ST_FINISH;
                    end else begin
                        root_sector_index <= root_sector_index + 1'b1;
                        sector_lba <= sector_lba + 1'b1;
                        state <= ST_ROOT_REQ;
                    end
                end
                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) bpb_bad_q <= 1'b1;
        else bpb_bad_q <= (cap_bytes_per_sec != 16'd512) || !valid_spc(cap_spc) ||
                          (cap_reserved == 0) || (cap_fat_count != 1 && cap_fat_count != 2) ||
                          (cap_root_entries != 0) || (cap_fat_size16 != 0) || (cap_total_sectors == 0) ||
                          (cap_fat_size32 == 0) || (cap_root_cluster < 2);
    // Reads 1 one cycle after entering ST_BOOT_PARSE (bpb_bad_q now reflects the current boot sector); clears on state exit
    always @(posedge clk or negedge rst_n)
        if (!rst_n) parse_wait <= 1'b0;
        else        parse_wait <= (state == ST_BOOT_PARSE);
endmodule
