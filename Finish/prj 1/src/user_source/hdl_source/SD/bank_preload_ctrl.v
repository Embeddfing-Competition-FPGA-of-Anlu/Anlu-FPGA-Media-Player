`timescale 1ns/1ps

// 第7课 四 bank 帧缓存（完赛整合版）：扫描 + 后台预载调度，sd_card_clk 域
//   上电：等 SDRAM 初始化 -> 扫描根目录 PIC1~4.BMP -> 第 i 张图写入 SDRAM bank i
//   预载：一张接一张在后台加载，不影响当前显示（显示读另一个 bank）；
//         每张图第 4 课的“重试 MAX_RETRIES 次 -> 标记失败跳过”规则不变
//   结果：image_complete / image_failed 位图交给导航模块，切图只是换读地址，瞬间完成
//   SD 卡超时：先复位 SD 卡重新初始化；扫描结果已有效时保留全部状态，只重发未完成的加载
module bank_preload_ctrl #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter [1:0]   MAX_RETRIES = 2'd1
)(
    input             clk,
    input             rst,
    input wire reload_start,
    input wire [1:0] reload_idx,
    input             sdram_init_done,
    input             reader_ready,
    input             sd_init_done,
    input             scan_done,
    input             scan_found_valid,
    input      [31:0] scan_found_cluster,
    input      [31:0] scan_found_size,
    input             load_error,
    input      [2:0]  reader_error,
    input             write_finish_toggle,   // mem_clk 域整帧写完（电平翻转）
    output reg        scan_start,
    output reg        load_start,
    output reg [31:0] load_cluster,
    output reg [31:0] load_size,
    output reg [1:0]  load_idx,              // 正在加载的图片号 = 目标 bank 号
    output reg        load_busy,
    output reg [3:0]  image_complete,
    output reg [3:0]  image_failed,
    output reg [2:0]  found_count,
    output            preload_done,
    output            sd_ready,
    output            sd_card_rst,
    output reg [2:0]  err_code,
    output reg        error_active
);
    reg [31:0] clu0, clu1, clu2, clu3;
    reg [31:0] siz0, siz1, siz2, siz3;
    reg [2:0]  wrfin_sync;
    reg        scan_kicked, scan_ok;
    reg [2:0]  preload_ptr;
    reg [1:0]  retry_cnt;
    reg        retry_pending, source_done_seen, write_done_seen;
    reg [7:0]  reinit_cnt;
    reg [31:0] rescan_cnt;

    wire write_finish_pulse = wrfin_sync[2] ^ wrfin_sync[1];
    assign sd_card_rst  = rst | (reinit_cnt != 8'd0);
    assign sd_ready     = sd_init_done && (reinit_cnt == 8'd0);
    assign preload_done = scan_ok && (preload_ptr == found_count) && !load_busy;

    function [31:0] pick;
        input [1:0] i;
        input [31:0] a, b, c, d;
        pick = (i == 2'd0) ? a : (i == 2'd1) ? b : (i == 2'd2) ? c : d;
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            {clu0, clu1, clu2, clu3} <= 128'd0;
            {siz0, siz1, siz2, siz3} <= 128'd0;
            wrfin_sync <= 3'd0;      scan_kicked <= 1'b0;   scan_ok <= 1'b0;
            preload_ptr <= 3'd0;     retry_cnt <= 2'd0;     retry_pending <= 1'b0;
            source_done_seen <= 1'b0; write_done_seen <= 1'b0;
            reinit_cnt <= 8'd0;      rescan_cnt <= 32'd0;
            scan_start <= 1'b0;      load_start <= 1'b0;
            load_cluster <= 32'd0;   load_size <= 32'd0;    load_idx <= 2'd0;
            load_busy <= 1'b0;       image_complete <= 4'd0; image_failed <= 4'd0;
            found_count <= 3'd0;     err_code <= 3'd0;      error_active <= 1'b0;
        end else begin
            wrfin_sync <= {wrfin_sync[1:0], write_finish_toggle};
            scan_start <= 1'b0;
            load_start <= 1'b0;
            if (reinit_cnt != 8'd0)
                reinit_cnt <= reinit_cnt - 8'd1;

            if (!sd_init_done) begin
                if (!scan_ok) begin
                    // 还没扫描成功：SD 卡恢复后整套重来
                    scan_kicked <= 1'b0;  found_count <= 3'd0;  preload_ptr <= 3'd0;
                    image_complete <= 4'd0; image_failed <= 4'd0; load_busy <= 1'b0;
                    retry_pending <= 1'b0; retry_cnt <= 2'd0;
                    source_done_seen <= 1'b0; write_done_seen <= 1'b0;
                end else if (load_busy) begin
                    // 扫描结果有效：保留，只把当前这张图挂起，SD 卡恢复后重发
                    retry_pending <= 1'b1;  source_done_seen <= 1'b0;  write_done_seen <= 1'b0;
                end
            end else begin
                // ---------------- 扫描 ----------------
                if (!scan_kicked && reader_ready && sdram_init_done) begin
                    scan_start <= 1'b1;  scan_kicked <= 1'b1;  scan_ok <= 1'b0;
                    found_count <= 3'd0; preload_ptr <= 3'd0;
                    image_complete <= 4'd0; image_failed <= 4'd0;
                end else if (scan_done && found_count != 3'd0 && !scan_ok) begin
                    scan_ok <= 1'b1;  error_active <= 1'b0;  err_code <= 3'd0;
                end
                if (scan_found_valid && found_count < 3'd4) begin
                    case (found_count[1:0])
                        2'd0: begin clu0 <= scan_found_cluster; siz0 <= scan_found_size; end
                        2'd1: begin clu1 <= scan_found_cluster; siz1 <= scan_found_size; end
                        2'd2: begin clu2 <= scan_found_cluster; siz2 <= scan_found_size; end
                        default: begin clu3 <= scan_found_cluster; siz3 <= scan_found_size; end
                    endcase
                    found_count <= found_count + 3'd1;
                end

                // 一张都没找到：每 1 秒复位 SD 卡重扫（换卡/插好卡后自动恢复），数码管显示 E4
                if (scan_kicked && scan_done && found_count == 3'd0 && reader_ready &&
                    reinit_cnt == 8'd0 && !scan_start) begin
                    if (rescan_cnt >= CLK_FREQ_HZ - 1) begin
                        rescan_cnt <= 32'd0;  reinit_cnt <= 8'd255;
                        error_active <= 1'b1;
                        err_code <= (reader_error == 3'd0) ? 3'd4 : reader_error;
                    end else
                        rescan_cnt <= rescan_cnt + 32'd1;
                end else
                    rescan_cnt <= 32'd0;

                // ---------------- 加载结果（第 4 课错误恢复） ----------------
                if (load_busy && load_error) begin
                    source_done_seen <= 1'b0;  write_done_seen <= 1'b0;
                    if (reader_error == 3'd2)
                        reinit_cnt <= 8'd255;              // 超时：先让 SD 卡重新初始化
                    if (retry_cnt < MAX_RETRIES) begin
                        retry_cnt <= retry_cnt + 2'd1;  retry_pending <= 1'b1;
                    end else begin
                        load_busy <= 1'b0;  retry_cnt <= 2'd0;  retry_pending <= 1'b0;
                        image_failed[load_idx] <= 1'b1;    // 跳过坏图，继续预载下一张
                        if (!error_active) begin error_active <= 1'b1; err_code <= reader_error; end
                    end
                end else if (load_busy && retry_pending && reader_ready && sd_ready) begin
                    retry_pending <= 1'b0;  source_done_seen <= 1'b0;  write_done_seen <= 1'b0;
                    load_start <= 1'b1;                    // 同一张图、同一个 bank 重新加载
                end

                if (load_busy && reader_ready && !retry_pending && !load_error && !load_start)
                    source_done_seen <= 1'b1;              // 源文件已读完送进 FIFO
                if (load_busy && write_finish_pulse)
                    write_done_seen <= 1'b1;               // SDRAM 整帧写完
                if (load_busy && source_done_seen && (write_done_seen || write_finish_pulse)) begin
                    load_busy <= 1'b0;  source_done_seen <= 1'b0;  write_done_seen <= 1'b0;
                    retry_cnt <= 2'd0;  retry_pending <= 1'b0;
                    image_complete[load_idx] <= 1'b1;
                end

                if(reload_start && preload_done && reader_ready && sd_ready) begin
                    load_idx<=reload_idx;
                    load_cluster<=pick(reload_idx,clu0,clu1,clu2,clu3);
                    load_size<=pick(reload_idx,siz0,siz1,siz2,siz3);
                    load_busy<=1;load_start<=1;retry_cnt<=0;retry_pending<=0;
                    source_done_seen<=0;write_done_seen<=0;
                    image_complete[reload_idx]<=0;image_failed[reload_idx]<=0;
                end
                // ---------------- 后台预载：按 PIC1->PIC4 顺序逐张写入各自 bank ----------------
                if (scan_done && !load_busy && reader_ready && sd_ready && !scan_start &&
                    !load_start && preload_ptr < found_count) begin
                    load_idx     <= preload_ptr[1:0];
                    load_cluster <= pick(preload_ptr[1:0], clu0, clu1, clu2, clu3);
                    load_size    <= pick(preload_ptr[1:0], siz0, siz1, siz2, siz3);
                    load_busy    <= 1'b1;  load_start <= 1'b1;
                    retry_cnt    <= 2'd0;  retry_pending <= 1'b0;
                    source_done_seen <= 1'b0;  write_done_seen <= 1'b0;
                    preload_ptr  <= preload_ptr + 3'd1;
                end
            end
        end
    end
endmodule
