`timescale 1ns/1ps

// 第10课 WAV 音频播放（完赛整合版）：WAV 头解析器，sd_card_clk 域
//   输入：fat32_file_streamer 送来的 MUSIC.WAV 文件字节流（已按 FAT 链拼好，与扇区无关）
//   处理：RIFF/WAVE 校验 -> 逐个 chunk 解析，fmt 取声道/采样率/位宽，其余 chunk 跳过
//         -> data chunk 的字节原样作为 PCM 字节输出给异步 FIFO
//   要求：PCM(1) / 16bit / 48000Hz / 单声道或立体声（doc/convert/convert_wav.py 负责转换）
module wav_stream_parser (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // 单拍：新一轮播放开始（每次循环播放都会重发）
    input  wire        byte_valid,
    input  wire [7:0]  byte_data,
    output reg         pcm_valid,
    output reg  [7:0]  pcm_byte,
    output reg         fmt_ok,         // 已解析到合法 fmt
    output reg         stereo,
    output reg         fmt_error       // 头非法/格式不支持：上层停止播放，回退到音调源
);
    localparam [2:0] S_RIFF = 3'd0;    // 12 字节 RIFF 头
    localparam [2:0] S_HDR  = 3'd1;    // 8 字节 chunk 头
    localparam [2:0] S_FMT  = 3'd2;    // fmt 体
    localparam [2:0] S_SKIP = 3'd3;    // 其它 chunk 体
    localparam [2:0] S_DATA = 3'd4;    // PCM 数据
    localparam [2:0] S_ERR  = 3'd5;

    reg [2:0]  state;
    reg [31:0] cnt;                    // 当前段内字节计数
    reg [31:0] sh;                     // 最近 4 字节（小端：sh[31:24] 是最新字节）
    reg [31:0] ck_id;
    reg [31:0] ck_size;
    // Only predicates are consumed after fmt parsing; retain their results
    // instead of the full 80-bit field values across the PCM stream.
    reg fmt_pcm, fmt_channels_ok, fmt_stereo, fmt_48k, fmt_16bit;

    wire [31:0] sh_nx   = {byte_data, sh[31:8]};
    // chunk 长度为奇数时后面有 1 字节填充
    wire [31:0] ck_span = ck_size + {31'd0, ck_size[0]};
    wire        fmt_good = fmt_pcm && fmt_16bit && fmt_48k && fmt_channels_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_RIFF;
            cnt       <= 32'd0;
            sh        <= 32'd0;
            ck_id     <= 32'd0;
            ck_size   <= 32'd0;
            fmt_pcm   <= 1'b0;
            fmt_channels_ok <= 1'b0;
            fmt_stereo <= 1'b0;
            fmt_48k   <= 1'b0;
            fmt_16bit <= 1'b0;
            pcm_valid <= 1'b0;
            pcm_byte  <= 8'd0;
            fmt_ok    <= 1'b0;
            stereo    <= 1'b0;
            fmt_error <= 1'b0;
        end else begin
            pcm_valid <= 1'b0;
            if (start) begin
                // 只复位解析状态；fmt_ok/stereo 保留，循环播放时音频侧不会来回切换
                state <= S_RIFF;
                cnt   <= 32'd0;
            end else if (byte_valid) begin
                sh <= sh_nx;
                case (state)
                    S_RIFF: begin
                        cnt <= cnt + 32'd1;
                        if (cnt == 32'd3 && sh_nx != "FFIR")        // "RIFF" 小端
                            state <= S_ERR;
                        else if (cnt == 32'd11) begin
                            cnt   <= 32'd0;
                            state <= (sh_nx == "EVAW") ? S_HDR : S_ERR;  // "WAVE"
                        end
                    end
                    S_HDR: begin
                        cnt <= cnt + 32'd1;
                        if (cnt == 32'd3)
                            ck_id <= sh_nx;
                        else if (cnt == 32'd7) begin
                            ck_size <= sh_nx;
                            cnt     <= 32'd0;
                            if (ck_id == "atad") begin              // "data"
                                if (fmt_good) state <= S_DATA;
                                else          state <= S_ERR;
                            end else if (ck_id == " tmf")           // "fmt "
                                state <= S_FMT;
                            else if (sh_nx == 32'd0)
                                state <= S_HDR;                     // 空 chunk
                            else
                                state <= S_SKIP;
                        end
                    end
                    S_FMT: begin
                        cnt <= cnt + 32'd1;
                        case (cnt)
                            32'd1:  fmt_pcm <= (sh_nx[31:16] == 16'd1);
                            32'd3: begin
                                fmt_channels_ok <= (sh_nx[31:16] == 16'd1 || sh_nx[31:16] == 16'd2);
                                fmt_stereo <= (sh_nx[31:16] == 16'd2);
                            end
                            32'd7:  fmt_48k <= (sh_nx == 32'd48000);
                            32'd15: fmt_16bit <= (sh_nx[31:16] == 16'd16);
                            default: ;
                        endcase
                        if (cnt + 32'd1 >= ck_span) begin
                            cnt   <= 32'd0;
                            state <= S_HDR;
                        end
                    end
                    S_SKIP: begin
                        cnt <= cnt + 32'd1;
                        if (cnt + 32'd1 >= ck_span) begin
                            cnt   <= 32'd0;
                            state <= S_HDR;
                        end
                    end
                    S_DATA: begin
                        pcm_valid <= 1'b1;
                        pcm_byte  <= byte_data;
                        cnt       <= cnt + 32'd1;
                        if (cnt + 32'd1 >= ck_size) begin           // data 之后若还有 chunk，继续跳过
                            cnt   <= 32'd0;
                            state <= S_HDR;
                        end
                    end
                    default: ;                                      // S_ERR：吞掉剩余字节
                endcase
            end

            if (state == S_DATA) begin
                fmt_ok <= 1'b1;
                stereo <= fmt_stereo;
            end
            if (state == S_ERR)
                fmt_error <= 1'b1;
            else if (state == S_DATA)
                fmt_error <= 1'b0;
        end
    end
endmodule
