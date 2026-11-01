`timescale 1ns/1ps

// 紧急插播配套：双音警报（hi-lo 700Hz / 1000Hz，每 0.45s 交替），video_clk 域
//   自带 48kHz 采样节拍（与 pcm_media_tone 相同的分数累加器），一项缓冲 + valid/ready，
//   保证样本率由本模块决定，不受下游取数节奏影响；三角波，幅度按音量缩放
module siren_tone #(
    parameter integer DYNAMIC_MODE=0,
    parameter integer CLOCK_HZ  = 25_000_000,
    parameter integer SAMPLE_HZ = 48_000
)(
    input wire mode_800,
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire [7:0]         volume,
    output reg                sample_valid,
    input  wire               sample_ready,
    output reg  signed [15:0] sample_left,
    output wire signed [15:0] sample_right
);
    localparam [15:0] INC_LO = 16'd956;          // 700Hz  * 65536 / 48000
    localparam [15:0] INC_HI = 16'd1365;         // 1000Hz * 65536 / 48000
    localparam [15:0] SWAP   = 16'd21600;        // 0.45s * 48000

    wire [25:0] clock_rate=(DYNAMIC_MODE && mode_800)?26'd40000000:CLOCK_HZ;
    reg [25:0] rate_acc;
    reg [15:0] phase, swap_cnt;
    reg        hi;
    wire       tick = (rate_acc >= clock_rate - SAMPLE_HZ);
    // 三角波：相位高位折叠，-16384..16383
    wire [14:0] tri_u = phase[15] ? ~phase[14:0] : phase[14:0];
    wire signed [15:0] tri_s = $signed({1'b0, tri_u}) - 16'sd16384;
    wire signed [24:0] scaled = tri_s * $signed({1'b0, volume});

    assign sample_right = sample_left;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rate_acc <= 25'd0;  phase <= 16'd0;  swap_cnt <= 16'd0;  hi <= 1'b0;
            sample_valid <= 1'b0;  sample_left <= 16'sd0;
        end else begin
            rate_acc <= tick ? rate_acc - (clock_rate - SAMPLE_HZ) : rate_acc + SAMPLE_HZ;
            if (sample_valid && sample_ready) sample_valid <= 1'b0;
            if (tick) begin
                phase <= phase + (hi ? INC_HI : INC_LO);
                if (swap_cnt >= SWAP - 1) begin swap_cnt <= 16'd0; hi <= ~hi; end
                else swap_cnt <= swap_cnt + 16'd1;
                if (!sample_valid || sample_ready) begin
                    sample_valid <= 1'b1;
                    sample_left  <= enable ? scaled[23:8] : 16'sd0;
                end
            end
        end
    end
endmodule
