`timescale 1ns/1ps

module hdmi_audio_infoframe_packetizer (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         emit,
    output reg          packet_valid,
    input  wire         packet_ready,
    output wire [23:0]  packet_header,
    output wire [7:0]   packet_header_ecc,
    output wire [223:0] packet_body,
    output wire [31:0]  packet_body_ecc
);
    wire [7:0] ecc0, ecc1, ecc2, ecc3;
    reg send_avi;

    // 每帧发两个 InfoFrame：先 Audio InfoFrame，再 AVI InfoFrame。
    // Low byte is HB0. Audio InfoFrame: type 0x84, version 1, length 10.
    // PB0 checksum=0x70, PB1.CC=1 means two channels. Coding type,
    // sample frequency/size and channel allocation refer to the stream.
    // AVI InfoFrame: type 0x82, version 2, length 13. PB1=0x00 RGB,
    // PB2=0x18 4:3 画面比例, PB4=VIC 1 (640x480p60), PB0 checksum=0x56.
    // 部分显示器收不到 AVI InfoFrame 会按 DVI 处理，不播放音频。
    assign packet_header = send_avi ? 24'h0d0282 : 24'h0a0184;
    assign packet_body = send_avi ?
        {168'd0, 8'h00, 8'h00, 8'h01, 8'h00, 8'h18, 8'h00, 8'h56} :
        {208'd0, 8'h01, 8'h70};

    hdmi_bch8 #(.DATA_BYTES(3)) u_header_bch (
        .data(packet_header), .ecc(packet_header_ecc)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body0_bch (
        .data(packet_body[55:0]), .ecc(ecc0)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body1_bch (
        .data(packet_body[111:56]), .ecc(ecc1)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body2_bch (
        .data(packet_body[167:112]), .ecc(ecc2)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body3_bch (
        .data(packet_body[223:168]), .ecc(ecc3)
    );
    assign packet_body_ecc = {ecc3, ecc2, ecc1, ecc0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            packet_valid <= 1'b0;
            send_avi <= 1'b0;
        end else if (packet_valid) begin
            if (packet_ready) begin
                if (send_avi) begin
                    packet_valid <= 1'b0;
                    send_avi <= 1'b0;
                end else begin
                    send_avi <= 1'b1;
                end
            end
        end else if (emit) begin
            packet_valid <= 1'b1;
            send_avi <= 1'b0;
        end
    end
endmodule
