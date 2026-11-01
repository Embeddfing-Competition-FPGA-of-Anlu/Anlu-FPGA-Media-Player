`timescale 1ns/1ps

// 顺序恢复除法器：NW 位被除数 / DW 位除数，NW 拍出结果（每张图只用几次，不占 DSP、不进关键路径）
module seq_div #(
    parameter integer NW = 22,
    parameter integer DW = 12
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,                     // 调用方保证非 0
    output reg           done,
    output reg  [NW-1:0] q,
    output wire [DW-1:0] r
);
    reg [NW-1:0] n;
    reg [DW-1:0] rem;
    reg [5:0]    cnt;
    reg          busy;
    wire [DW:0]  t  = {rem, n[NW-1]};
    wire         ge = (t >= {1'b0, den});
    assign r = rem;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            n <= 0;  rem <= 0;  q <= 0;  cnt <= 0;  busy <= 1'b0;  done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start) begin
                n <= num;  rem <= 0;  q <= 0;  cnt <= NW;  busy <= 1'b1;
            end else if (busy) begin
                rem <= ge ? t - {1'b0, den} : t[DW-1:0];
                q   <= {q[NW-2:0], ge};
                n   <= {n[NW-2:0], 1'b0};
                cnt <= cnt - 6'd1;
                if (cnt == 6'd1) begin busy <= 1'b0;  done <= 1'b1; end
            end
        end
    end
endmodule
