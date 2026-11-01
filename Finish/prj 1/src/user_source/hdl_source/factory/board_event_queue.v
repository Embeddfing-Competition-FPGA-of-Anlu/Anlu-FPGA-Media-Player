`timescale 1ns/1ps
// Eight operation snapshots; !Q drains events before returning the current status.
// Overflow is counted and visible in ST replies, never silently reported as complete.
module board_event_queue(
    input wire clk,rst_n,
    input wire [7:0] events,
    input wire [2:0] board,
    input wire manual,detail,menu,auto_on,
    input wire [15:0] extra,
    input wire pop,
    output wire available,
    output wire [115:0] response,          // 原始数值：EV N=nnnn M=mmmm B=bbbb S=ssss X=xxxx
    output wire [6:0] response_len,
    output reg [15:0] overflow_count
);
    reg [47:0] memory[0:7];
    reg [2:0] rd,wr;
    reg [3:0] count;
    reg [15:0] event_sequence;
    wire take=pop && count!=0;
    wire put=events!=0 && (count<8 || take);
    assign available=count!=0;
    wire [47:0] head=memory[rd];
    assign response={head[47:32],8'd0,head[31:24],13'd0,head[23:21],12'd0,head[20:17],head[15:0],36'd0};
    assign response_len=39;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin rd<=0;wr<=0;count<=0;event_sequence<=0;overflow_count<=0;end
        else begin
            if(events!=0) begin
                event_sequence<=event_sequence+1'b1;
                if(!put && overflow_count!=65535) overflow_count<=overflow_count+1'b1;
            end
            if(put) begin
                memory[wr]<={event_sequence+16'd1,events,board,manual,detail,menu,auto_on,1'b0,extra};
                wr<=wr+1'b1;
            end
            if(take) rd<=rd+1'b1;
            case({put,take})
                2'b10:count<=count+1'b1;
                2'b01:count<=count-1'b1;
                default:;
            endcase
        end
    end
endmodule
