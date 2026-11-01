`timescale 1ns/1ps
// Fixed length, idempotent state: !G{tool 0..4}{alarm 0/1}{reason 0..7}*HH CR.
// HH is XOR of the five bytes before '*'. CRLF is accepted; partial lines expire.
module gimbal_command_rx #(
    parameter integer CLK_HZ = 50000000
)(
    input wire clk, rst_n, ms_tick,
    input wire [7:0] data,
    input wire valid, framing_error,
    output reg command_valid,
    output reg [2:0] tool,
    output reg alarm,
    output reg [2:0] reason
);
    reg [3:0] pos;
    reg [7:0] checksum, received;
    reg [2:0] tool_q, reason_q;
    reg alarm_q, bad;
    reg [6:0] age_ms;
    wire hex_ok = (data >= "0" && data <= "9") || (data >= "A" && data <= "F");
    wire [3:0] hex_value = data <= "9" ? data[3:0] : data[3:0] + 4'd9;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            pos<=0; checksum<=0; received<=0; tool_q<=0; reason_q<=0;
            alarm_q<=0; bad<=0; age_ms<=0; command_valid<=0; tool<=0; alarm<=0; reason<=0;
        end else begin
            command_valid <= 0;
            if (ms_tick && pos != 0) begin
                if (age_ms == 7'd99) begin pos<=0; bad<=0; end
                else age_ms <= age_ms + 1'b1;
            end
            if (framing_error) begin pos<=0; bad<=1; end
            else if (valid) begin
                age_ms <= 0;
                if (data == "!") begin pos<=1; checksum<="!"; bad<=0; end
                else if (data == 13 || data == 10) begin
                    if (pos == 8 && !bad && received == checksum &&
                        (!alarm_q || (tool_q >= 1 && tool_q <= 4))) begin
                        command_valid<=1; tool<=tool_q; alarm<=alarm_q; reason<=reason_q;
                    end
                    pos<=0;
                end else if (pos != 0) begin
                    case (pos)
                        1: if (data != "G") bad<=1;
                        2: begin tool_q<=data[2:0]; if (data < "0" || data > "4") bad<=1; end
                        3: begin alarm_q<=data[0]; if (data != "0" && data != "1") bad<=1; end
                        4: begin reason_q<=data[2:0]; if (data < "0" || data > "7") bad<=1; end
                        5: if (data != "*") bad<=1;
                        6: begin received[7:4]<=hex_value; if (!hex_ok) bad<=1; end
                        7: begin received[3:0]<=hex_value; if (!hex_ok) bad<=1; end
                        default: bad<=1;
                    endcase
                    if (pos < 5) checksum <= checksum ^ data;
                    if (pos < 9) pos <= pos + 1'b1;
                end
            end
        end
endmodule
