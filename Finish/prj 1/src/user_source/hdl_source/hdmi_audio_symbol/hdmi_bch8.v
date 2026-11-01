`timescale 1ns/1ps

module hdmi_bch8 #(
    parameter integer DATA_BYTES = 3
) (
    input  wire [DATA_BYTES*8-1:0] data,
    output reg  [7:0]              ecc
);
    integer byte_index;
    integer bit_index;
    reg [7:0] remainder;
    reg feedback;

    // HDMI packet BCH uses G(x)=1+x^6+x^7+x^8. HDMI 按 LSB 先发送，
    // 所以从 data[0] 开始逐位移入，用右移 LFSR（反射多项式 0x83），
    // ecc[0] 是最先发送的校验位。原课程代码按字节 MSB 先、左移 0xc1，
    // 算出的 ECC 与规范不一致，显示器会丢弃全部音频包。
    always @* begin
        remainder = 8'h00;
        feedback = 1'b0;
        byte_index = 0;
        for (bit_index = 0; bit_index < DATA_BYTES*8;
             bit_index = bit_index + 1) begin
            feedback = data[bit_index] ^ remainder[0];
            remainder = {1'b0, remainder[7:1]};
            if (feedback)
                remainder = remainder ^ 8'h83;
        end
        ecc = remainder;
    end
endmodule
