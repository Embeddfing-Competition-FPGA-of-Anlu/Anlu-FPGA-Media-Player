`timescale 1ns/1ps

// 第10课 WAV 音频播放（完赛整合版）：PCM 采样源，video_clk 25MHz 域
//   从异步字节 FIFO 取小端 16bit 采样，按 48kHz 节拍（与 pcm_media_tone 同一累加器写法）
//   推入 8 深输出 FIFO，valid/ready 契约与 pcm_media_tone 完全一致，可直接替换音调源。
//   节拍到了但 FIFO 里还没有完整一帧：输出静音并计欠载，保证 HDMI 音频包 48kHz 连续。
module wav_pcm_source #(
    parameter integer DYNAMIC_MODE=0,
    parameter integer CLOCK_HZ  = 25000000,
    parameter integer SAMPLE_HZ = 48000
)(
    input wire mode_800,
    input  wire               clk,
    input  wire               rst_n,
    input  wire               stereo,          // 已同步到本域
    input  wire [7:0]         gain,            // 0..250，输出 = 采样 * gain / 256
    // 异步 FIFO 读侧
    output wire               fifo_rd_en,
    input  wire [7:0]         fifo_rd_data,
    input  wire               fifo_empty,
    // PCM 输出（valid/ready）
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output wire signed [15:0] raw_left,        // 乘音量之前的采样（与 sample_left 同拍），给频谱/振动监测
    output reg                underflow
);
    wire [25:0] clock_rate=(DYNAMIC_MODE && mode_800)?26'd40000000:CLOCK_HZ;
    reg  [25:0] rate_acc;
    wire        tick = rate_acc >= clock_rate - SAMPLE_HZ;

    // ---------- 字节拼装：mono 2 字节/帧，stereo 4 字节/帧 ----------
    reg  [1:0]  bidx;
    reg         rd_pend;                       // 上一拍发了读，本拍 fifo_rd_data 有效
    reg  [7:0]  lo_byte;
    reg  signed [15:0] asm_l, asm_r;
    reg         frame_ready;
    wire [1:0]  last_idx = stereo ? 2'd3 : 2'd1;

    assign fifo_rd_en = !frame_ready && !rd_pend && !fifo_empty;


    // ---------- 8 深输出 FIFO ----------
    reg  signed [15:0] ofl [0:7];
    reg  signed [15:0] ofr [0:7];
    reg  [2:0]  wp, rp;
    reg  [3:0]  ocnt;
    wire        pop  = (ocnt != 0) && sample_ready;
    wire        push = tick && (ocnt != 8 || pop);

    assign sample_valid = (ocnt != 0);
    // ---------- 音量：在 FIFO 出口乘，FIFO 里存原始采样 ----------
    wire signed [24:0] mul_l = ofl[rp] * $signed({1'b0, gain});
    wire signed [24:0] mul_r = ofr[rp] * $signed({1'b0, gain});
    assign sample_left  = mul_l[23:8];
    assign sample_right = mul_r[23:8];
    assign raw_left     = ofl[rp];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rate_acc    <= 25'd0;
            bidx        <= 2'd0;
            rd_pend     <= 1'b0;
            lo_byte     <= 8'd0;
            asm_l       <= 16'sd0;
            asm_r       <= 16'sd0;
            frame_ready <= 1'b0;
            wp          <= 3'd0;
            rp          <= 3'd0;
            ocnt        <= 4'd0;
            underflow   <= 1'b0;
        end else begin
            rate_acc <= tick ? rate_acc + SAMPLE_HZ - clock_rate : rate_acc + SAMPLE_HZ;

            rd_pend <= fifo_rd_en;
            if (rd_pend) begin
                if (!bidx[0])
                    lo_byte <= fifo_rd_data;
                else if (!bidx[1]) begin
                    asm_l <= {fifo_rd_data, lo_byte};
                    if (!stereo) asm_r <= {fifo_rd_data, lo_byte};
                end else
                    asm_r <= {fifo_rd_data, lo_byte};
                if (bidx == last_idx) begin
                    bidx        <= 2'd0;
                    frame_ready <= 1'b1;
                end else
                    bidx <= bidx + 2'd1;
            end

            if (push) begin
                if (frame_ready) begin
                    ofl[wp]     <= asm_l;
                    ofr[wp]     <= asm_r;
                    frame_ready <= 1'b0;
                end else begin
                    ofl[wp]   <= 16'sd0;
                    ofr[wp]   <= 16'sd0;
                    underflow <= 1'b1;
                end
                wp <= wp + 3'd1;
            end
            if (pop)
                rp <= rp + 3'd1;
            case ({push, pop})
                2'b10:   ocnt <= ocnt + 4'd1;
                2'b01:   ocnt <= ocnt - 4'd1;
                default: ocnt <= ocnt;
            endcase
        end
    end
endmodule
